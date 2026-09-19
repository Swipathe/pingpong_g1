import test from "node:test";
import assert from "node:assert/strict";

import {createEditorApi, EditorApiError} from "../api.js";
import {
  canConfirmSoftLimits,
  loadServerListedPose,
  saveValidationTicket,
  validatedLoadedPose,
  validationMatchesCurrentPose,
} from "../app.js";
import {createPoseState, degreesToRadians} from "../pose_state.js";

const ARM_JOINT_NAMES = [
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
];
const BASE_JOINT_NAMES = Array.from(
  {length: 15},
  (_, index) => `base_joint_${String(index).padStart(2, "0")}`,
);
const ACTIVE_JOINT_NAMES = [...BASE_JOINT_NAMES, ...ARM_JOINT_NAMES];
const CRITICAL_LINK_NAMES = [
  "left_wrist_yaw_link",
  "right_wrist_yaw_link",
  "right_racket_link",
];
const MOTION_INDICES = [
  11, 15, 19, 21, 23, 25, 27,
  12, 16, 20, 22, 24, 26, 28,
];
const ASSET_HASHES = {
  displayUrdfSha256: "a".repeat(64),
  displayMeshSetSha256: "b".repeat(64),
  validationMjcfSha256: "c".repeat(64),
  assetYamlSha256: "d".repeat(64),
  urdfKinematicSha256: "e".repeat(64),
  mjcfKinematicSha256: "f".repeat(64),
  assetSignatureSha256: "1".repeat(64),
};

function revoluteJoint(name, index, parentLink, childLink) {
  return {
    name,
    type: "revolute",
    parentLink,
    childLink,
    originXyzM: [0, 0, 0],
    originRpyRad: [0, 0, 0],
    axisXyz: [0, 0, 1],
    hardLimitRad: [-3, 3],
    softLimitRad: [-2.7, 2.7],
    defaultRad: 0,
    robot29Index: index,
  };
}

function makeWorkflowManifest() {
  const joints = [];
  let parent = "pelvis";
  for (const [index, name] of BASE_JOINT_NAMES.entries()) {
    const child = `base_link_${String(index).padStart(2, "0")}`;
    joints.push(revoluteJoint(name, index, parent, child));
    parent = child;
  }
  const armRoot = parent;
  let leftParent = armRoot;
  for (let offset = 0; offset < 7; offset += 1) {
    const name = ARM_JOINT_NAMES[offset];
    const child = offset === 6
      ? "left_wrist_yaw_link"
      : `left_arm_link_${offset}`;
    joints.push(revoluteJoint(name, 15 + offset, leftParent, child));
    leftParent = child;
  }
  let rightParent = armRoot;
  for (let offset = 0; offset < 7; offset += 1) {
    const name = ARM_JOINT_NAMES[7 + offset];
    const child = offset === 6
      ? "right_wrist_yaw_link"
      : `right_arm_link_${offset}`;
    joints.push(revoluteJoint(name, 22 + offset, rightParent, child));
    rightParent = child;
  }
  joints.push({
    name: "right_racket_fixed",
    type: "fixed",
    parentLink: "right_wrist_yaw_link",
    childLink: "right_racket_link",
    originXyzM: [0, 0, 0],
    originRpyRad: [0, 0, 0],
    axisXyz: null,
    hardLimitRad: null,
    softLimitRad: null,
    defaultRad: null,
    robot29Index: null,
  });
  return {
    rootLink: "pelvis",
    activeJointNames: [...ACTIVE_JOINT_NAMES],
    armJointNames: [...ARM_JOINT_NAMES],
    defaultJointPosRad: Array(29).fill(0),
    joints,
    assetHashes: {...ASSET_HASHES},
  };
}

function identityCriticalLinks() {
  return Object.fromEntries(CRITICAL_LINK_NAMES.map((name) => [name, {
    position_m: [0, 0, 0],
    quaternion_xyzw: [0, 0, 0, 1],
  }]));
}

function makeSavedPoseRecord(manifest, poseId) {
  const armValues = Array(14).fill(0);
  return {
    schema: "hitter_ready_arm_pose/v1",
    unit: "rad",
    joint_pos_by_name: Object.fromEntries(
      manifest.armJointNames.map((name, index) => [name, armValues[index]]),
    ),
    joint_names: [...manifest.armJointNames],
    joint_pos_rad: [...armValues],
    robot29: {
      indices: Array.from({length: 29}, (_, index) => index),
      joint_names: [...manifest.activeJointNames],
      joint_pos_rad: Array(29).fill(0),
    },
    motion_npz: {
      indices: [...MOTION_INDICES],
      joint_names: [...manifest.armJointNames],
      joint_pos_rad: [...armValues],
    },
    fk: {
      frame: "robot_base_default",
      root_link: "pelvis",
      quaternion_convention: "xyzw",
      links: identityCriticalLinks(),
    },
    asset: {
      display_urdf_path: "/approved/display/main.urdf",
      validation_mjcf_path: "/approved/validation/model.xml",
      asset_yaml_path: "/approved/config/asset.yaml",
      display_urdf_sha256: manifest.assetHashes.displayUrdfSha256,
      display_mesh_set_sha256: manifest.assetHashes.displayMeshSetSha256,
      validation_mjcf_sha256: manifest.assetHashes.validationMjcfSha256,
      asset_yaml_sha256: manifest.assetHashes.assetYamlSha256,
      urdf_kinematic_sha256: manifest.assetHashes.urdfKinematicSha256,
      mjcf_kinematic_sha256: manifest.assetHashes.mjcfKinematicSha256,
      asset_signature_sha256: manifest.assetHashes.assetSignatureSha256,
    },
    validation: {
      hard_limits: "passed",
      browser_mujoco_fk: "passed",
      max_position_error_m: 0,
      max_orientation_error_rad: 0,
      soft_limit_warnings: [],
      soft_limits: "passed",
    },
    pose_name: poseId,
    created_at: "2026-07-27T10:11:12+08:00",
  };
}

function makeValidationContext(state, needsConfirmation = false) {
  return {
    snapshot: {armPose: state.armPose()},
    validation: {
      validation_id: "ticket-1",
      needs_soft_limit_confirmation: needsConfirmation,
      soft_limit_warnings: needsConfirmation ? ["near hard limit"] : [],
    },
  };
}

function jsonResponse(status, payload) {
  return {
    ok: status >= 200 && status < 300,
    status,
    async json() {
      return payload;
    },
  };
}

function binaryResponse(status, bytes) {
  return {
    ok: status >= 200 && status < 300,
    status,
    async arrayBuffer() {
      return bytes.buffer.slice(
        bytes.byteOffset,
        bytes.byteOffset + bytes.byteLength,
      );
    },
  };
}

test("save sends only validation ticket and explicit soft confirmation", async () => {
  const calls = [];
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return jsonResponse(201, {
      json_path: "/fixed/a.json",
      yaml_path: "/fixed/a.yaml",
    });
  });

  await api.save("ticket-1", true);

  assert.equal(calls[0].url, "/api/save");
  assert.deepEqual(JSON.parse(calls[0].options.body), {
    validation_id: "ticket-1",
    confirm_soft_limit: true,
  });
  assert.equal(calls[0].options.headers["X-Hitter-Editor-Token"], "secret");
});

test("non-2xx response preserves structured Chinese error", async () => {
  const api = createEditorApi("secret", async () => (
    jsonResponse(409, {error: "资产签名不一致", code: "asset_mismatch"})
  ));

  await assert.rejects(
    api.model(),
    (error) => (
      error instanceof EditorApiError &&
      error.message === "资产签名不一致" &&
      error.status === 409 &&
      error.code === "asset_mismatch"
    ),
  );
});

test("test_model_health_and_poses_are_gets", async () => {
  const calls = [];
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return jsonResponse(200, {});
  });

  await api.model();
  await api.health();
  await api.poses();

  assert.deepEqual(calls.map(({url, options}) => ({
    url,
    method: options.method,
    hasBody: Object.hasOwn(options, "body"),
    token: options.headers["X-Hitter-Editor-Token"],
    cache: options.cache,
  })), [
    {
      url: "/api/model",
      method: "GET",
      hasBody: false,
      token: "secret",
      cache: "no-store",
    },
    {
      url: "/api/health",
      method: "GET",
      hasBody: false,
      token: "secret",
      cache: "no-store",
    },
    {
      url: "/api/poses",
      method: "GET",
      hasBody: false,
      token: "secret",
      cache: "no-store",
    },
  ]);
});

test("test_mesh_uses_hash_and_opaque_id", async () => {
  const calls = [];
  const bytes = new Uint8Array([0, 1, 2, 255]);
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return binaryResponse(200, bytes);
  });

  const result = await api.mesh("a".repeat(64), "b".repeat(64));

  assert.deepEqual(new Uint8Array(result), bytes);
  assert.equal(
    calls[0].url,
    `/assets/${"a".repeat(64)}/${"b".repeat(64)}`,
  );
  assert.equal(calls[0].options.cache, "no-store");
  assert.equal(
    calls[0].options.headers["X-Hitter-Editor-Token"],
    "secret",
  );
  assert.equal(Object.hasOwn(calls[0].options, "body"), false);
});

test("test_validate_sends_exact_snapshot", async () => {
  const calls = [];
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return jsonResponse(200, {validation_id: "ticket-1"});
  });
  const snapshot = {
    joint_pos_by_name: {
      left_shoulder_pitch_joint: 0.125,
      right_wrist_yaw_joint: -0.25,
    },
    browser_fk: {
      frame: "robot_base_default",
      quaternion_convention: "xyzw",
      links: {},
    },
    asset_signature_sha256: "c".repeat(64),
  };

  await api.validate(snapshot);

  assert.equal(calls[0].url, "/api/validate");
  assert.equal(calls[0].options.method, "POST");
  assert.deepEqual(Object.keys(JSON.parse(calls[0].options.body)), [
    "joint_pos_by_name",
    "browser_fk",
    "asset_signature_sha256",
  ]);
  assert.deepEqual(JSON.parse(calls[0].options.body), snapshot);
  assert.equal(calls[0].options.headers["Content-Type"], "application/json");
});

test("test_pose_uses_only_listed_pose_id", async () => {
  const calls = [];
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return jsonResponse(200, {pose: {}});
  });

  await api.pose("hitter_ready_arm_pose_20260727_101112_001");
  assert.equal(
    calls[0].url,
    "/api/poses/hitter_ready_arm_pose_20260727_101112_001",
  );

  for (const poseId of [
    "../hitter_ready_arm_pose_20260727_101112",
    "hitter_ready_arm_pose_20260727_101112.json",
    "hitter_ready_arm_pose_20260727/101112",
    ".",
  ]) {
    assert.throws(
      () => api.pose(poseId),
      (error) => (
        error instanceof EditorApiError &&
        error.code === "invalid_pose_id"
      ),
    );
  }
  assert.equal(calls.length, 1);
});

test("test_timeout_aborts", async () => {
  const originalSetTimeout = globalThis.setTimeout;
  const originalClearTimeout = globalThis.clearTimeout;
  let timeoutCallback;
  let timeoutDelay;
  let observedSignal;
  globalThis.setTimeout = (callback, delay) => {
    timeoutCallback = callback;
    timeoutDelay = delay;
    return 91;
  };
  globalThis.clearTimeout = () => {};
  try {
    const api = createEditorApi("secret", async (_url, options) => {
      observedSignal = options.signal;
      return new Promise((_resolve, reject) => {
        options.signal.addEventListener("abort", () => {
          const error = new Error("aborted");
          error.name = "AbortError";
          reject(error);
        });
      });
    });
    const pending = api.model();
    await Promise.resolve();

    assert.equal(timeoutDelay, 15000);
    assert.equal(observedSignal.aborted, false);
    timeoutCallback();

    await assert.rejects(
      pending,
      (error) => (
        error instanceof EditorApiError &&
        error.message === "请求超时，请检查 SSH 隧道" &&
        error.status === 0 &&
        error.code === "timeout"
      ),
    );
    assert.equal(observedSignal.aborted, true);
  } finally {
    globalThis.setTimeout = originalSetTimeout;
    globalThis.clearTimeout = originalClearTimeout;
  }
});

test("test_invalid_json_and_network_failure", async () => {
  const invalidJsonApi = createEditorApi("secret", async () => ({
    ok: false,
    status: 502,
    async json() {
      throw new SyntaxError("not json");
    },
  }));
  await assert.rejects(
    invalidJsonApi.health(),
    (error) => (
      error instanceof EditorApiError &&
      error.message === "服务器返回无效 JSON" &&
      error.status === 502 &&
      error.code === "invalid_response"
    ),
  );

  const networkApi = createEditorApi("secret", async () => {
    throw new TypeError("connection refused");
  });
  await assert.rejects(
    networkApi.poses(),
    (error) => (
      error instanceof EditorApiError &&
      error.message === "无法连接姿态编辑服务" &&
      error.status === 0 &&
      error.code === "network"
    ),
  );
});

test("test_no_request_can_send_path", async () => {
  const calls = [];
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return jsonResponse(url === "/api/save" ? 201 : 200, {});
  });
  await api.validate({
    joint_pos_by_name: {left_elbow_joint: 0.5},
    browser_fk: {
      frame: "robot_base_default",
      quaternion_convention: "xyzw",
      links: {},
    },
    asset_signature_sha256: "d".repeat(64),
  });
  await api.save("ticket-2", false);

  const forbidden = new Set(["path", "filename", "output_dir"]);
  const scan = (value) => {
    if (Array.isArray(value)) {
      value.forEach(scan);
      return;
    }
    if (value && typeof value === "object") {
      for (const [key, child] of Object.entries(value)) {
        assert.equal(forbidden.has(key), false, `forbidden request key ${key}`);
        scan(child);
      }
    }
  };
  for (const {options} of calls) {
    if (Object.hasOwn(options, "body")) {
      scan(JSON.parse(options.body));
    }
  }
});

test("workflow soft-warning gate requires explicit confirmation", () => {
  assert.equal(canConfirmSoftLimits({
    needs_soft_limit_confirmation: true,
    soft_limit_warnings: ["right wrist is near hard limit"],
  }, false), false);
  assert.equal(canConfirmSoftLimits({
    needs_soft_limit_confirmation: true,
    soft_limit_warnings: ["right wrist is near hard limit"],
  }, true), true);
  assert.equal(canConfirmSoftLimits({
    needs_soft_limit_confirmation: false,
    soft_limit_warnings: [],
  }, false), true);
  assert.equal(canConfirmSoftLimits({
    needs_soft_limit_confirmation: false,
    soft_limit_warnings: ["unexpected warning"],
  }, false), false);
});

test("workflow invalidates validation after any accepted pose drift", () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  const snapshot = {armPose: state.armPose()};

  assert.equal(validationMatchesCurrentPose(manifest, state, snapshot), true);
  state.setDegrees("left_elbow_joint", 5);
  assert.equal(validationMatchesCurrentPose(manifest, state, snapshot), false);
});

test("workflow does not mark clean when pose changes while save is in flight", async () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  const context = makeValidationContext(state);
  let resolveSave;
  const api = {
    save() {
      return new Promise((resolve) => {
        resolveSave = resolve;
      });
    },
  };

  const pending = saveValidationTicket({
    api,
    manifest,
    state,
    context,
    confirmSoftLimit: false,
  });
  state.setDegrees("left_elbow_joint", 5);
  resolveSave({
    pose_id: "hitter_ready_arm_pose_20260727_101112",
    json_path: "/fixed/a.json",
    yaml_path: "/fixed/a.yaml",
    storage_warning: null,
  });
  const result = await pending;

  assert.equal(result.saved, true);
  assert.equal(result.markedClean, false);
  assert.equal(result.retryAllowed, false);
  assert.equal(state.isDirty(), true);
});

test("workflow treats storage_warning as successful and never retryable", async () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  const context = makeValidationContext(state);
  const result = await saveValidationTicket({
    api: {
      async save() {
        return {
          pose_id: "hitter_ready_arm_pose_20260727_101112",
          json_path: "/fixed/a.json",
          yaml_path: "/fixed/a.yaml",
          storage_warning: "姿态已保存，但目录持久化确认出现警告；请勿重复保存",
        };
      },
    },
    manifest,
    state,
    context,
    confirmSoftLimit: false,
  });

  assert.equal(result.saved, true);
  assert.equal(result.markedClean, true);
  assert.equal(result.retryAllowed, false);
  assert.equal(
    result.storageWarning,
    "姿态已保存，但目录持久化确认出现警告；请勿重复保存",
  );
  assert.equal(state.isDirty(), false);
});

test("workflow load accepts only a current server-listed opaque pose ID", async () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  const before = state.armPose();
  let poseCalls = 0;

  await assert.rejects(
    loadServerListedPose({
      api: {
        async pose() {
          poseCalls += 1;
          return {pose: {}};
        },
      },
      poseId: "hitter_ready_arm_pose_20260727_101112",
      listedPoseIds: new Set(),
      manifest,
      state,
    }),
    /不在服务器列表/,
  );
  assert.equal(poseCalls, 0);
  assert.deepEqual(state.armPose(), before);
});

test("reload rejects inconsistent lower-body robot29 values before mutation", async () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  state.setDegrees("left_elbow_joint", 7);
  const before = state.armPose();
  const poseId = "hitter_ready_arm_pose_20260727_101112";
  const record = makeSavedPoseRecord(manifest, poseId);
  record.robot29.joint_pos_rad[0] = 0.25;

  await assert.rejects(
    loadServerListedPose({
      api: {async pose() { return {pose: record}; }},
      poseId,
      listedPoseIds: new Set([poseId]),
      manifest,
      state,
    }),
    /robot29.*默认|29-D.*默认/,
  );
  assert.deepEqual(state.armPose(), before);
  assert.equal(state.isDirty(), true);
});

test("reload rejects recorded FK that disagrees with strict recomputed browser FK", async () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  state.setDegrees("left_elbow_joint", 7);
  const before = state.armPose();
  const poseId = "hitter_ready_arm_pose_20260727_101112";
  const record = makeSavedPoseRecord(manifest, poseId);
  record.fk.links.right_racket_link.position_m[0] = 0.01;

  await assert.rejects(
    loadServerListedPose({
      api: {async pose() { return {pose: record}; }},
      poseId,
      listedPoseIds: new Set([poseId]),
      manifest,
      state,
    }),
    /FK.*不一致/,
  );
  assert.deepEqual(state.armPose(), before);
  assert.equal(state.isDirty(), true);
});

test("reload rejects schema or signature drift before mutation", async () => {
  const manifest = makeWorkflowManifest();
  const poseId = "hitter_ready_arm_pose_20260727_101112";
  const cases = [
    (record) => { record.schema = "hitter_ready_arm_pose/v2"; },
    (record) => { record.asset.asset_signature_sha256 = "2".repeat(64); },
  ];

  for (const mutate of cases) {
    const state = createPoseState(manifest);
    state.setDegrees("right_elbow_joint", 8);
    const before = state.armPose();
    const record = makeSavedPoseRecord(manifest, poseId);
    mutate(record);
    await assert.rejects(loadServerListedPose({
      api: {async pose() { return {pose: record}; }},
      poseId,
      listedPoseIds: new Set([poseId]),
      manifest,
      state,
    }));
    assert.deepEqual(state.armPose(), before);
    assert.equal(state.isDirty(), true);
  }
});

test("reload commits once and isolates post-commit rendering failure", async () => {
  const manifest = makeWorkflowManifest();
  const state = createPoseState(manifest);
  state.setDegrees("left_elbow_joint", 9);
  const poseId = "hitter_ready_arm_pose_20260727_101112";
  const record = makeSavedPoseRecord(manifest, poseId);
  let successfulEffectRan = false;

  const result = await loadServerListedPose({
    api: {async pose() { return {pose: record}; }},
    poseId,
    listedPoseIds: new Set([poseId]),
    manifest,
    state,
    postCommitEffects: [
      () => { throw new Error("renderer failed"); },
      () => { successfulEffectRan = true; },
    ],
  });

  assert.equal(result.committed, true);
  assert.equal(result.effectErrors.length, 1);
  assert.equal(result.effectErrors[0].message, "renderer failed");
  assert.equal(successfulEffectRan, true);
  assert.deepEqual(state.armPose(), record.joint_pos_by_name);
  assert.equal(state.isDirty(), false);
});
