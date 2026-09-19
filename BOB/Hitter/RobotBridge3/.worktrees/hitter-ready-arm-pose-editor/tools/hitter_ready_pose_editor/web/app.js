import {createEditorApi} from "./api.js";
import {computeForwardKinematics, extractHitterFkSummary} from "./fk.js";
import {createPoseState, degreesToRadians, radiansToDegrees} from "./pose_state.js";
import {HitterRenderer} from "./renderer.js";

const TOKEN_STORAGE_KEY = "hitter-ready-editor-token";
const JOINT_ORDER = [
  "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
  "wrist_roll", "wrist_pitch", "wrist_yaw",
];
const JOINT_LABELS = {
  shoulder_pitch: "肩俯仰",
  shoulder_roll: "肩横滚",
  shoulder_yaw: "肩偏航",
  elbow: "肘关节",
  wrist_roll: "腕横滚",
  wrist_pitch: "腕俯仰",
  wrist_yaw: "腕偏航",
};
const CRITICAL_LINKS = [
  "left_wrist_yaw_link",
  "right_wrist_yaw_link",
  "right_racket_link",
];
const MOTION_NPZ_ARM_INDICES = [
  11, 15, 19, 21, 23, 25, 27,
  12, 16, 20, 22, 24, 26, 28,
];
const RECORD_KEYS = [
  "schema", "unit", "joint_pos_by_name", "joint_names", "joint_pos_rad",
  "robot29", "motion_npz", "fk", "asset", "validation", "pose_name",
  "created_at",
];
const ASSET_HASH_FIELDS = {
  display_urdf_sha256: "displayUrdfSha256",
  display_mesh_set_sha256: "displayMeshSetSha256",
  validation_mjcf_sha256: "validationMjcfSha256",
  asset_yaml_sha256: "assetYamlSha256",
  urdf_kinematic_sha256: "urdfKinematicSha256",
  mjcf_kinematic_sha256: "mjcfKinematicSha256",
  asset_signature_sha256: "assetSignatureSha256",
};
const ASSET_PATH_FIELDS = [
  "display_urdf_path",
  "validation_mjcf_path",
  "asset_yaml_path",
];
const POSITION_TOLERANCE_M = 0.0005;
const ORIENTATION_TOLERANCE_RAD = 0.0017453292519943296;

function formatDegrees(radians) {
  return radiansToDegrees(radians).toFixed(2);
}

function formatVector(values) {
  return `[${values.map((value) => Number(value).toFixed(9)).join(", ")}]`;
}

function showError(element, message) {
  element.textContent = message;
  element.hidden = false;
}

function clearError(element) {
  element.textContent = "";
  element.hidden = true;
}

function sessionToken() {
  const url = new URL(window.location.href);
  const fromUrl = url.searchParams.get("token");
  if (fromUrl) {
    sessionStorage.setItem(TOKEN_STORAGE_KEY, fromUrl);
    url.searchParams.delete("token");
    history.replaceState(null, "", `${url.pathname}${url.search}${url.hash}`);
  }
  return fromUrl || sessionStorage.getItem(TOKEN_STORAGE_KEY);
}

function deepFreeze(value) {
  if (value && typeof value === "object" && !Object.isFrozen(value)) {
    for (const child of Object.values(value)) {
      deepFreeze(child);
    }
    Object.freeze(value);
  }
  return value;
}

function immutableJsonCopy(value) {
  return deepFreeze(JSON.parse(JSON.stringify(value)));
}

function isObject(value) {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function assertExactKeys(value, expected, field) {
  if (!isObject(value)) {
    throw new Error(`${field} 必须是对象`);
  }
  const actual = Object.keys(value);
  if (
    actual.length !== expected.length ||
    expected.some((name) => !Object.hasOwn(value, name))
  ) {
    throw new Error(`${field} 字段不完整`);
  }
}

function assertFiniteVector(value, length, field) {
  if (
    !Array.isArray(value) ||
    value.length !== length ||
    !value.every(Number.isFinite)
  ) {
    throw new Error(`${field} 必须包含 ${length} 个有限数字`);
  }
}

function arraysEqual(left, right) {
  return (
    Array.isArray(left) &&
    Array.isArray(right) &&
    left.length === right.length &&
    left.every((value, index) => value === right[index])
  );
}

function armPoseMatches(manifest, left, right) {
  return manifest.armJointNames.every((name) => left[name] === right[name]);
}

export function canConfirmSoftLimits(validation, checked) {
  const hasWarnings = (
    Array.isArray(validation?.soft_limit_warnings) &&
    validation.soft_limit_warnings.length > 0
  );
  const needsConfirmation = (
    validation?.needs_soft_limit_confirmation === true ||
    hasWarnings
  );
  return !needsConfirmation || checked === true;
}

export function validationMatchesCurrentPose(manifest, state, snapshot) {
  return Boolean(
    snapshot?.armPose &&
    armPoseMatches(manifest, state.armPose(), snapshot.armPose)
  );
}

export async function saveValidationTicket({
  api,
  manifest,
  state,
  context,
  confirmSoftLimit,
}) {
  if (!validationMatchesCurrentPose(manifest, state, context?.snapshot)) {
    throw new Error("姿态已变化，旧票据已在本地失效");
  }
  if (!canConfirmSoftLimits(context?.validation, confirmSoftLimit)) {
    throw new Error("请先明确确认 soft-limit 警告");
  }
  const response = await api.save(
    context.validation.validation_id,
    confirmSoftLimit === true,
  );
  const stillCurrent = validationMatchesCurrentPose(
    manifest,
    state,
    context.snapshot,
  );
  let markedClean = false;
  const effectErrors = [];
  if (stillCurrent) {
    try {
      markedClean = state.markSaved(context.snapshot.armPose);
    } catch (error) {
      effectErrors.push(error);
    }
  }
  return {
    saved: true,
    response,
    markedClean,
    retryAllowed: false,
    storageWarning: response?.storage_warning || null,
    effectErrors,
  };
}

function validateFkLinks(links, field) {
  assertExactKeys(links, CRITICAL_LINKS, field);
  for (const name of CRITICAL_LINKS) {
    const link = links[name];
    assertExactKeys(
      link,
      ["position_m", "quaternion_xyzw"],
      `${field}.${name}`,
    );
    assertFiniteVector(link.position_m, 3, `${field}.${name}.position_m`);
    assertFiniteVector(
      link.quaternion_xyzw,
      4,
      `${field}.${name}.quaternion_xyzw`,
    );
    const norm = Math.hypot(...link.quaternion_xyzw);
    if (Math.abs(norm - 1) > 1e-6) {
      throw new Error(`${field}.${name} 四元数不是单位四元数`);
    }
  }
}

function validateAssetHashes(assetHashes, manifest, field) {
  assertExactKeys(
    assetHashes,
    Object.values(ASSET_HASH_FIELDS),
    field,
  );
  for (const publicName of Object.values(ASSET_HASH_FIELDS)) {
    const value = assetHashes[publicName];
    if (
      typeof value !== "string" ||
      !/^[0-9a-f]{64}$/.test(value) ||
      value !== manifest.assetHashes[publicName]
    ) {
      throw new Error(`${field}.${publicName} 与当前模型不一致`);
    }
  }
}

function validateValidationResponse(payload, manifest, snapshot) {
  assertExactKeys(payload, [
    "validation_id",
    "expires_in_s",
    "joint_pos_by_name",
    "joint_pos_29_rad",
    "fk",
    "soft_limit_warnings",
    "needs_soft_limit_confirmation",
    "max_position_error_m",
    "max_orientation_error_rad",
    "asset_hashes",
  ], "验证响应");
  if (typeof payload.validation_id !== "string" || !payload.validation_id) {
    throw new Error("验证票据无效");
  }
  if (!Number.isFinite(payload.expires_in_s) || payload.expires_in_s <= 0) {
    throw new Error("验证票据有效期无效");
  }
  assertExactKeys(
    payload.joint_pos_by_name,
    manifest.armJointNames,
    "验证响应 joint_pos_by_name",
  );
  if (!armPoseMatches(
    manifest,
    payload.joint_pos_by_name,
    snapshot.armPose,
  )) {
    throw new Error("服务器验证姿态与本地快照不一致");
  }
  assertFiniteVector(payload.joint_pos_29_rad, 29, "验证响应 joint_pos_29_rad");
  for (const [index, name] of manifest.activeJointNames.entries()) {
    if (payload.joint_pos_29_rad[index] !== snapshot.jointPosByName29[name]) {
      throw new Error(`服务器 29-D 映射与本地快照不一致：${name}`);
    }
  }
  assertExactKeys(
    payload.fk,
    ["frame", "quaternion_convention", "links"],
    "验证响应 fk",
  );
  if (
    payload.fk.frame !== "robot_base_default" ||
    payload.fk.quaternion_convention !== "xyzw"
  ) {
    throw new Error("验证响应 FK 坐标系无效");
  }
  validateFkLinks(payload.fk.links, "验证响应 fk.links");
  if (
    !Array.isArray(payload.soft_limit_warnings) ||
    !payload.soft_limit_warnings.every((warning) => (
      typeof warning === "string" && warning.length > 0
    )) ||
    typeof payload.needs_soft_limit_confirmation !== "boolean" ||
    payload.needs_soft_limit_confirmation !== (
      payload.soft_limit_warnings.length > 0
    )
  ) {
    throw new Error("验证响应 soft-limit 警告无效");
  }
  if (
    !Number.isFinite(payload.max_position_error_m) ||
    payload.max_position_error_m < 0 ||
    !Number.isFinite(payload.max_orientation_error_rad) ||
    payload.max_orientation_error_rad < 0
  ) {
    throw new Error("验证响应 FK 误差无效");
  }
  validateAssetHashes(payload.asset_hashes, manifest, "验证响应 asset_hashes");
  return immutableJsonCopy(payload);
}

function validatedPoseList(payload) {
  assertExactKeys(payload, ["poses"], "姿态列表响应");
  if (!Array.isArray(payload.poses)) {
    throw new Error("姿态列表必须是数组");
  }
  const seen = new Set();
  return payload.poses.map((item) => {
    assertExactKeys(item, ["pose_id", "created_at"], "姿态列表项");
    if (
      typeof item.pose_id !== "string" ||
      !/^hitter_ready_arm_pose_\d{8}_\d{6}(?:_\d{3})?$/.test(item.pose_id) ||
      seen.has(item.pose_id)
    ) {
      throw new Error("服务器返回了非法或重复的姿态 ID");
    }
    if (typeof item.created_at !== "string" || Number.isNaN(Date.parse(item.created_at))) {
      throw new Error(`姿态 ${item.pose_id} 的创建时间无效`);
    }
    seen.add(item.pose_id);
    return immutableJsonCopy(item);
  });
}

function quaternionAngularDistance(left, right) {
  const dot = Math.abs(left.reduce(
    (total, value, index) => total + value * right[index],
    0,
  ));
  return 2 * Math.acos(Math.min(1, Math.max(-1, dot)));
}

function assertRecordedFkMatchesBrowser(recordFk, manifest, jointPos29) {
  const byName = Object.fromEntries(manifest.activeJointNames.map(
    (name, index) => [name, jointPos29[index]],
  ));
  const browserFk = extractHitterFkSummary(
    computeForwardKinematics(manifest, byName),
  );
  for (const name of CRITICAL_LINKS) {
    const recorded = recordFk.links[name];
    const browser = browserFk.links[name];
    const positionError = Math.hypot(
      recorded.position_m[0] - browser.position_m[0],
      recorded.position_m[1] - browser.position_m[1],
      recorded.position_m[2] - browser.position_m[2],
    );
    const orientationError = quaternionAngularDistance(
      recorded.quaternion_xyzw,
      browser.quaternion_xyzw,
    );
    if (
      positionError > POSITION_TOLERANCE_M ||
      orientationError > ORIENTATION_TOLERANCE_RAD
    ) {
      throw new Error(`保存姿态 FK 与浏览器严格重算不一致：${name}`);
    }
  }
}

export function validatedLoadedPose(payload, poseId, manifest) {
  assertExactKeys(payload, ["pose"], "保存姿态响应");
  const record = payload.pose;
  assertExactKeys(record, RECORD_KEYS, "保存姿态记录");
  if (
    record.schema !== "hitter_ready_arm_pose/v1" ||
    record.unit !== "rad" ||
    record.pose_name !== poseId ||
    typeof record.created_at !== "string" ||
    !/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\+08:00$/.test(record.created_at)
  ) {
    throw new Error("保存姿态 schema、ID、单位或创建时间无效");
  }

  if (!arraysEqual(record.joint_names, manifest.armJointNames)) {
    throw new Error("保存姿态 14-D 关节名称映射无效");
  }
  assertExactKeys(
    record.joint_pos_by_name,
    manifest.armJointNames,
    "保存姿态 joint_pos_by_name",
  );
  const armValues = manifest.armJointNames.map((name) => (
    record.joint_pos_by_name[name]
  ));
  assertFiniteVector(armValues, 14, "保存姿态 14-D radians");
  if (!arraysEqual(record.joint_pos_rad, armValues)) {
    throw new Error("保存姿态 14-D radians 与名称映射不一致");
  }

  assertExactKeys(
    record.robot29,
    ["indices", "joint_names", "joint_pos_rad"],
    "保存姿态 robot29",
  );
  if (
    !arraysEqual(record.robot29.indices, Array.from({length: 29}, (_, index) => index)) ||
    !arraysEqual(record.robot29.joint_names, manifest.activeJointNames)
  ) {
    throw new Error("保存姿态 robot29 映射标识无效");
  }
  assertFiniteVector(record.robot29.joint_pos_rad, 29, "保存姿态 robot29 radians");
  const expectedRobot29 = manifest.defaultJointPosRad.map(
    (value, index) => index < 15 ? value : armValues[index - 15],
  );
  if (!arraysEqual(record.robot29.joint_pos_rad, expectedRobot29)) {
    throw new Error("保存姿态 robot29 与启动默认值及 14-D 映射不一致");
  }
  const jointSpecs = new Map(manifest.joints.map((joint) => [joint.name, joint]));
  for (const [index, name] of manifest.armJointNames.entries()) {
    const limits = jointSpecs.get(name)?.hardLimitRad;
    if (
      !Array.isArray(limits) ||
      limits.length !== 2 ||
      armValues[index] < limits[0] ||
      armValues[index] > limits[1]
    ) {
      throw new Error(`保存姿态 ${name} 超出 hard limit`);
    }
  }

  assertExactKeys(
    record.motion_npz,
    ["indices", "joint_names", "joint_pos_rad"],
    "保存姿态 motion_npz",
  );
  if (
    !arraysEqual(record.motion_npz.indices, MOTION_NPZ_ARM_INDICES) ||
    !arraysEqual(record.motion_npz.joint_names, manifest.armJointNames) ||
    !arraysEqual(record.motion_npz.joint_pos_rad, armValues)
  ) {
    throw new Error("保存姿态 motion_npz 映射无效");
  }

  assertExactKeys(
    record.fk,
    ["frame", "root_link", "quaternion_convention", "links"],
    "保存姿态 fk",
  );
  if (
    record.fk.frame !== "robot_base_default" ||
    record.fk.root_link !== "pelvis" ||
    record.fk.quaternion_convention !== "xyzw"
  ) {
    throw new Error("保存姿态 FK 坐标系无效");
  }
  validateFkLinks(record.fk.links, "保存姿态 fk.links");
  assertRecordedFkMatchesBrowser(
    record.fk,
    manifest,
    record.robot29.joint_pos_rad,
  );

  assertExactKeys(
    record.asset,
    [...ASSET_PATH_FIELDS, ...Object.keys(ASSET_HASH_FIELDS)],
    "保存姿态 asset",
  );
  for (const name of ASSET_PATH_FIELDS) {
    if (typeof record.asset[name] !== "string" || !record.asset[name].startsWith("/")) {
      throw new Error(`保存姿态 asset.${name} 不是绝对路径`);
    }
  }
  for (const [recordName, publicName] of Object.entries(ASSET_HASH_FIELDS)) {
    if (
      typeof record.asset[recordName] !== "string" ||
      record.asset[recordName] !== manifest.assetHashes[publicName]
    ) {
      throw new Error(`保存姿态 asset.${recordName} 与当前模型不一致`);
    }
  }

  assertExactKeys(record.validation, [
    "hard_limits",
    "browser_mujoco_fk",
    "max_position_error_m",
    "max_orientation_error_rad",
    "soft_limit_warnings",
    "soft_limits",
  ], "保存姿态 validation");
  if (
    record.validation.hard_limits !== "passed" ||
    record.validation.browser_mujoco_fk !== "passed" ||
    !["passed", "warning_confirmed"].includes(record.validation.soft_limits) ||
    !Number.isFinite(record.validation.max_position_error_m) ||
    record.validation.max_position_error_m < 0 ||
    record.validation.max_position_error_m > 0.0005 ||
    !Number.isFinite(record.validation.max_orientation_error_rad) ||
    record.validation.max_orientation_error_rad < 0 ||
    record.validation.max_orientation_error_rad > 0.0017453292519943296 ||
    !Array.isArray(record.validation.soft_limit_warnings) ||
    !record.validation.soft_limit_warnings.every((warning) => (
      typeof warning === "string" && warning.length > 0
    ))
  ) {
    throw new Error("保存姿态 validation 记录无效");
  }
  return immutableJsonCopy(record.joint_pos_by_name);
}

export async function loadServerListedPose({
  api,
  poseId,
  listedPoseIds,
  manifest,
  state,
  postCommitEffects = [],
}) {
  if (!(listedPoseIds instanceof Set) || !listedPoseIds.has(poseId)) {
    throw new Error("该姿态已不在服务器列表中，请刷新");
  }
  const payload = await api.pose(poseId);
  const armPose = validatedLoadedPose(payload, poseId, manifest);
  state.loadJointPose(armPose);
  const effectErrors = [];
  let markedClean = false;
  try {
    markedClean = state.markSaved(armPose);
  } catch (error) {
    effectErrors.push(error);
  }
  if (!markedClean) {
    effectErrors.push(new Error("姿态已加载，但未能标记为 clean"));
  }
  for (const effect of postCommitEffects) {
    try {
      effect();
    } catch (error) {
      effectErrors.push(error);
    }
  }
  return {
    committed: true,
    armPose,
    markedClean,
    effectErrors,
  };
}

function createValidationSnapshot(state, manifest) {
  const jointPosByName29 = state.jointPosByName29();
  const fkResult = computeForwardKinematics(
    manifest,
    jointPosByName29,
  );
  const browserFk = extractHitterFkSummary(fkResult);
  const armPose = state.armPose();
  return immutableJsonCopy({
    armPose,
    jointPosByName29,
    browserFk,
    request: {
      joint_pos_by_name: armPose,
      browser_fk: browserFk,
      asset_signature_sha256: manifest.assetHashes.assetSignatureSha256,
    },
  });
}

function appendDefinition(list, term, description) {
  const dt = document.createElement("dt");
  dt.textContent = term;
  const dd = document.createElement("dd");
  dd.textContent = description;
  list.append(dt, dd);
}

function renderControls(container, manifest, side, state, onAcceptedChange) {
  container.replaceChildren();
  const expected = JOINT_ORDER.map((segment) => `${side}_${segment}_joint`);
  const names = manifest.armJointNames.filter((name) => name.startsWith(`${side}_`));
  if (names.length !== 7 || names.some((name, index) => name !== expected[index])) {
    throw new Error(`${side} 臂关节顺序不符合肩到腕的七关节约定`);
  }
  for (const name of names) {
    const segment = name.slice(`${side}_`.length, -"_joint".length);
    const control = state.control(name);
    const wrapper = document.createElement("article");
    wrapper.className = "joint-control";
    wrapper.dataset.jointName = name;

    const label = document.createElement("label");
    label.htmlFor = `${name}-number`;
    label.textContent = JOINT_LABELS[segment];
    const value = document.createElement("output");
    value.className = "joint-degree";
    const slider = document.createElement("input");
    slider.type = "range";
    slider.id = `${name}-slider`;
    slider.min = String(radiansToDegrees(control.sliderRangeRad[0]));
    slider.max = String(radiansToDegrees(control.sliderRangeRad[1]));
    slider.step = "0.01";
    const numeric = document.createElement("input");
    numeric.type = "number";
    numeric.id = `${name}-number`;
    numeric.min = String(radiansToDegrees(control.hardRangeRad[0]));
    numeric.max = String(radiansToDegrees(control.hardRangeRad[1]));
    numeric.step = "0.01";
    numeric.inputMode = "decimal";
    const limits = document.createElement("p");
    limits.className = "joint-limits";
    limits.textContent = `滑块 ${formatDegrees(control.sliderRangeRad[0])}° 至 ${formatDegrees(control.sliderRangeRad[1])}°；输入 ${formatDegrees(control.hardRangeRad[0])}° 至 ${formatDegrees(control.hardRangeRad[1])}°`;
    const inline = document.createElement("p");
    inline.className = "joint-inline-error";
    inline.hidden = true;

    const render = () => {
      const current = state.control(name).currentRad;
      const degrees = formatDegrees(current);
      slider.value = degrees;
      numeric.value = degrees;
      value.value = `${degrees}°`;
      const warning = state.warningFor(name);
      limits.dataset.level = warning ? "warning" : "";
      inline.textContent = warning?.message || "";
      inline.hidden = !warning;
    };
    const reject = (message) => {
      numeric.value = formatDegrees(state.radians(name));
      slider.value = formatDegrees(state.radians(name));
      showError(inline, message);
    };
    slider.addEventListener("input", () => {
      try {
        state.setSliderRadians(name, degreesToRadians(Number(slider.value)));
        clearError(inline);
        onAcceptedChange();
      } catch (error) {
        reject(`滑块值无效：${error.message}`);
      }
    });
    numeric.addEventListener("change", () => {
      const raw = numeric.value.trim();
      const degrees = Number(raw);
      if (raw === "" || !Number.isFinite(degrees)) {
        reject("请输入有限数字");
        return;
      }
      try {
        state.setDegrees(name, degrees);
        clearError(inline);
        onAcceptedChange();
      } catch (error) {
        reject(`输入超出硬限制：${error.message}`);
      }
    });
    wrapper.append(label, value, slider, numeric, limits, inline);
    container.append(wrapper);
    render();
  }
}

async function bootstrap() {
  const status = document.querySelector("#editor-status");
  const error = document.querySelector("#editor-error");
  const saveButton = document.querySelector("#save-pose");
  const saveReason = document.querySelector("#save-reason");
  const dialog = document.querySelector("#save-dialog");
  const validationStatus = document.querySelector("#validation-status");
  const validationError = document.querySelector("#validation-error");
  const validationDetails = document.querySelector("#validation-details");
  const jointSummary = document.querySelector("#validation-joints");
  const mappingSummary = document.querySelector("#validation-mapping");
  const fkSummary = document.querySelector("#validation-fk");
  const hashSummary = document.querySelector("#validation-hashes");
  const warningList = document.querySelector("#validation-warnings");
  const softConfirmRow = document.querySelector("#soft-confirm-row");
  const softConfirm = document.querySelector("#soft-confirm");
  const confirmSave = document.querySelector("#confirm-save");
  const retryValidation = document.querySelector("#retry-validation");
  const saveResult = document.querySelector("#save-result");
  const savedJsonPath = document.querySelector("#saved-json-path");
  const savedYamlPath = document.querySelector("#saved-yaml-path");
  const storageWarning = document.querySelector("#storage-warning");
  const poseList = document.querySelector("#saved-pose-list");
  const poseListStatus = document.querySelector("#pose-list-status");
  const poseListError = document.querySelector("#pose-list-error");

  const token = sessionToken();
  if (!token) {
    showError(error, "请使用服务启动时打印的带 token 链接打开页面");
    return;
  }

  const api = createEditorApi(token);
  try {
    status.textContent = "正在加载模型…";
    const [health, manifest] = await Promise.all([api.health(), api.model()]);
    if (
      health.asset_signature_sha256 !== manifest.assetHashes?.assetSignatureSha256
    ) {
      throw new Error("健康检查与模型资产签名不一致");
    }
    const state = createPoseState(manifest);
    const renderer = new HitterRenderer(
      document.querySelector("#robot-canvas"),
      (message) => { status.textContent = message; },
    );
    await renderer.load(
      manifest,
      (digest, meshId) => api.mesh(digest, meshId),
    );

    let validationContext = null;
    let validationGeneration = 0;
    let listedPoseIds = new Set();
    let defaultMatrices = null;
    let cleanLabel = "默认姿态";

    const refresh = () => {
      const result = computeForwardKinematics(manifest, state.jointPosByName29());
      extractHitterFkSummary(result);
      renderer.setPose(result.linkWorldMatrices);
      status.textContent = state.isDirty() ? "姿态已修改，尚未保存" : cleanLabel;
    };

    const invalidateValidation = (message = "姿态已变化，请重新检查后再保存") => {
      validationGeneration += 1;
      validationContext = null;
      confirmSave.disabled = true;
      softConfirm.checked = false;
      if (dialog.open) {
        validationStatus.textContent = message;
        validationDetails.hidden = true;
        saveResult.hidden = true;
        retryValidation.hidden = false;
      }
    };

    const renderAllControls = () => {
      for (const selector of [
        "#left-arm-panel .joint-control",
        "#right-arm-panel .joint-control",
      ]) {
        for (const element of document.querySelectorAll(selector)) {
          const name = element.dataset.jointName;
          const current = formatDegrees(state.radians(name));
          element.querySelector("input[type=range]").value = current;
          element.querySelector("input[type=number]").value = current;
          element.querySelector("output").value = `${current}°`;
          const warning = state.warningFor(name);
          const limits = element.querySelector(".joint-limits");
          const inline = element.querySelector(".joint-inline-error");
          limits.dataset.level = warning ? "warning" : "";
          inline.textContent = warning?.message || "";
          inline.hidden = !warning;
        }
      }
    };

    const onAcceptedChange = () => {
      invalidateValidation();
      refresh();
      renderAllControls();
    };

    const renderValidation = (validation, snapshot) => {
      jointSummary.replaceChildren();
      for (const name of manifest.armJointNames) {
        const item = document.createElement("li");
        item.textContent = `${name} = ${snapshot.armPose[name].toFixed(9)} rad`;
        jointSummary.append(item);
      }
      mappingSummary.textContent = [
        "robot29.indices = 0…28",
        "arm indices = 15…28",
        `joint_names = ${manifest.activeJointNames.join(", ")}`,
      ].join("；");

      fkSummary.replaceChildren();
      appendDefinition(fkSummary, "frame", validation.fk.frame);
      appendDefinition(
        fkSummary,
        "quaternion",
        validation.fk.quaternion_convention,
      );
      for (const name of CRITICAL_LINKS) {
        const link = validation.fk.links[name];
        appendDefinition(
          fkSummary,
          name,
          `position_m ${formatVector(link.position_m)}；quaternion_xyzw ${formatVector(link.quaternion_xyzw)}`,
        );
      }
      appendDefinition(
        fkSummary,
        "最大位置误差",
        `${validation.max_position_error_m.toExponential(6)} m`,
      );
      appendDefinition(
        fkSummary,
        "最大姿态误差",
        `${validation.max_orientation_error_rad.toExponential(6)} rad`,
      );

      hashSummary.replaceChildren();
      for (const [name, value] of Object.entries(validation.asset_hashes)) {
        appendDefinition(hashSummary, name, value);
      }

      warningList.replaceChildren();
      for (const warning of validation.soft_limit_warnings) {
        const item = document.createElement("li");
        item.textContent = warning;
        warningList.append(item);
      }
      if (validation.soft_limit_warnings.length === 0) {
        const item = document.createElement("li");
        item.textContent = "无 soft-limit 警告";
        warningList.append(item);
      }
      softConfirmRow.hidden = !validation.needs_soft_limit_confirmation;
      softConfirm.checked = false;
      validationDetails.hidden = false;
      confirmSave.disabled = validation.needs_soft_limit_confirmation;
    };

    const runValidation = async () => {
      validationGeneration += 1;
      const generation = validationGeneration;
      validationContext = null;
      clearError(validationError);
      validationStatus.textContent = "正在生成不可变快照并由服务器检查…";
      validationDetails.hidden = true;
      saveResult.hidden = true;
      retryValidation.hidden = true;
      confirmSave.disabled = true;
      softConfirm.checked = false;
      try {
        const snapshot = createValidationSnapshot(state, manifest);
        const response = await api.validate(snapshot.request);
        if (
          generation !== validationGeneration ||
          !validationMatchesCurrentPose(manifest, state, snapshot)
        ) {
          throw new Error("检查期间姿态已变化，请重新检查");
        }
        const validation = validateValidationResponse(
          response,
          manifest,
          snapshot,
        );
        validationContext = deepFreeze({snapshot, validation});
        renderValidation(validation, snapshot);
        validationStatus.textContent = `检查通过；票据将在 ${validation.expires_in_s} 秒后失效`;
      } catch (caught) {
        if (generation === validationGeneration) {
          validationContext = null;
          showError(validationError, caught.message);
          validationStatus.textContent = "检查未完成，编辑状态保持不变";
          retryValidation.hidden = false;
        }
      }
    };

    const renderPoseList = (items) => {
      poseList.replaceChildren();
      if (items.length === 0) {
        const empty = document.createElement("li");
        empty.className = "pose-list-empty";
        empty.textContent = "尚无已保存姿态";
        poseList.append(empty);
        return;
      }
      for (const item of items) {
        const row = document.createElement("li");
        const details = document.createElement("div");
        const id = document.createElement("strong");
        id.textContent = item.pose_id;
        const created = document.createElement("time");
        created.dateTime = item.created_at;
        created.textContent = item.created_at;
        details.append(id, created);
        const load = document.createElement("button");
        load.type = "button";
        load.textContent = "加载";
        load.addEventListener("click", async () => {
          if (!listedPoseIds.has(item.pose_id)) {
            showError(poseListError, "该姿态已不在服务器列表中，请刷新");
            return;
          }
          load.disabled = true;
          clearError(poseListError);
          poseListStatus.textContent = `正在验证并加载 ${item.pose_id}…`;
          try {
            const result = await loadServerListedPose({
              api,
              poseId: item.pose_id,
              listedPoseIds,
              manifest,
              state,
              postCommitEffects: [
                () => { cleanLabel = `已加载 ${item.pose_id}`; },
                () => invalidateValidation(),
                renderAllControls,
                refresh,
              ],
            });
            if (result.effectErrors.length > 0) {
              showError(
                poseListError,
                `姿态已加载，但界面刷新不完整：${result.effectErrors.map(
                  (effectError) => effectError.message,
                ).join("；")}`,
              );
              poseListStatus.textContent = "姿态状态已提交；部分显示副作用失败";
            } else {
              poseListStatus.textContent = "已完整验证 schema、映射、FK 与资产签名";
            }
          } catch (caught) {
            showError(poseListError, caught.message);
            poseListStatus.textContent = "加载失败，当前姿态保持不变";
          } finally {
            load.disabled = false;
          }
        });
        row.append(details, load);
        poseList.append(row);
      }
    };

    const refreshSavedPoses = async () => {
      clearError(poseListError);
      poseListStatus.textContent = "正在刷新已保存姿态…";
      try {
        const items = validatedPoseList(await api.poses());
        listedPoseIds = new Set(items.map((item) => item.pose_id));
        renderPoseList(items);
        poseListStatus.textContent = `共 ${items.length} 个服务器确认的姿态`;
      } catch (caught) {
        showError(poseListError, caught.message);
        poseListStatus.textContent = "列表刷新失败，可重试";
      }
    };

    softConfirm.addEventListener("change", () => {
      confirmSave.disabled = !(
        validationContext &&
        canConfirmSoftLimits(validationContext.validation, softConfirm.checked)
      );
    });
    retryValidation.addEventListener("click", runValidation);
    saveButton.addEventListener("click", () => {
      if (typeof dialog.showModal === "function") {
        if (!dialog.open) {
          dialog.showModal();
        }
      } else {
        dialog.setAttribute("open", "");
      }
      runValidation();
    });
    confirmSave.addEventListener("click", async () => {
      const context = validationContext;
      if (!context) {
        showError(validationError, "验证票据不存在，请重新检查");
        return;
      }
      if (!validationMatchesCurrentPose(manifest, state, context.snapshot)) {
        invalidateValidation();
        showError(validationError, "姿态已变化，旧票据已在本地失效");
        return;
      }
      const confirmSoft = context.validation.soft_limit_warnings.length > 0
        ? softConfirm.checked
        : false;
      if (!canConfirmSoftLimits(context.validation, confirmSoft)) {
        showError(validationError, "请先明确确认 soft-limit 警告");
        return;
      }
      confirmSave.disabled = true;
      retryValidation.hidden = true;
      clearError(validationError);
      validationStatus.textContent = "正在提交已验证票据…";
      let result;
      try {
        result = await saveValidationTicket({
          api,
          manifest,
          state,
          context,
          confirmSoftLimit: confirmSoft,
        });
      } catch (caught) {
        showError(validationError, caught.message);
        validationStatus.textContent = "保存未完成，编辑状态保持不变";
        if (validationContext === context) {
          retryValidation.hidden = false;
          confirmSave.disabled = !canConfirmSoftLimits(
            context.validation,
            softConfirm.checked,
          );
        }
        return;
      }

      validationContext = null;
      retryValidation.hidden = true;
      confirmSave.disabled = true;
      const saved = result.response;
      if (
        !isObject(saved) ||
        typeof saved.json_path !== "string" ||
        !saved.json_path.startsWith("/") ||
        typeof saved.yaml_path !== "string" ||
        !saved.yaml_path.startsWith("/")
      ) {
        showError(validationError, "保存请求成功，但服务器响应缺少绝对输出路径；请勿重复保存");
        validationStatus.textContent = "保存结果无法完整显示；旧票据已失效";
        return;
      }
      savedJsonPath.textContent = saved.json_path;
      savedYamlPath.textContent = saved.yaml_path;
      storageWarning.textContent = result.storageWarning || "";
      storageWarning.hidden = !result.storageWarning;
      saveResult.hidden = false;
      validationDetails.hidden = true;
      softConfirmRow.hidden = true;
      validationStatus.textContent = "独立 YAML + JSON 已保存";
      if (result.markedClean) {
        cleanLabel = `已保存 ${saved.pose_id || "独立姿态"}`;
      }
      try {
        refresh();
      } catch (caught) {
        showError(error, `姿态已保存，但三维视图刷新失败：${caught.message}`);
      }
      if (!result.markedClean) {
        status.textContent = "姿态已保存，但当前显示值已变化，仍标记为未保存";
      }
      await refreshSavedPoses();
    });

    const defaultResult = computeForwardKinematics(
      manifest,
      Object.fromEntries(manifest.activeJointNames.map((name, index) => [
        name,
        manifest.defaultJointPosRad[index],
      ])),
    );
    defaultMatrices = defaultResult.linkWorldMatrices;
    renderer.setGhostPose(defaultMatrices);

    renderControls(
      document.querySelector("#left-arm-panel .joint-list"),
      manifest,
      "left",
      state,
      onAcceptedChange,
    );
    renderControls(
      document.querySelector("#right-arm-panel .joint-list"),
      manifest,
      "right",
      state,
      onAcceptedChange,
    );
    document.querySelector("#reset-left").addEventListener("click", () => {
      state.resetArm("left");
      onAcceptedChange();
    });
    document.querySelector("#reset-right").addEventListener("click", () => {
      state.resetArm("right");
      onAcceptedChange();
    });
    document.querySelector("#reset-all").addEventListener("click", () => {
      state.resetAll();
      onAcceptedChange();
    });
    document.querySelector("#table-toggle").addEventListener("change", (event) => {
      renderer.setTableVisible(event.currentTarget.checked);
    });
    document.querySelector("#ghost-toggle").addEventListener("change", (event) => {
      renderer.setGhostPose(event.currentTarget.checked ? defaultMatrices : null);
    });
    document.querySelector("#refresh-pose-list").addEventListener(
      "click",
      refreshSavedPoses,
    );
    for (const button of document.querySelectorAll("[data-camera-preset]")) {
      button.addEventListener("click", () => renderer.setCameraPreset(button.dataset.cameraPreset));
    }
    window.addEventListener("beforeunload", (event) => {
      if (state.shouldWarnBeforeUnload()) {
        event.preventDefault();
        event.returnValue = "";
      }
    });

    if (health.status === "ok" && manifest.compatibleForSave) {
      saveButton.disabled = false;
      saveReason.textContent = "先生成不可变验证快照，再由票据确认保存。";
    } else {
      saveButton.disabled = true;
      const reasons = [
        ...(health.incompatibilities || []),
        ...(manifest.incompatibilities || []),
      ];
      saveReason.textContent = `只读：${reasons.join("；") || "资产不兼容"}`;
    }
    refresh();
    await refreshSavedPoses();
  } catch (caught) {
    showError(error, `无法加载编辑器：${caught.message}`);
    status.textContent = "";
  }
}

if (typeof window !== "undefined" && typeof document !== "undefined") {
  bootstrap();
}
