import test from "node:test";
import assert from "node:assert/strict";

import {
  createPoseState,
  degreesToRadians,
  radiansToDegrees,
} from "../pose_state.js";

const LOWER_BODY_NAMES = [
  "lower_00", "lower_01", "lower_02", "lower_03", "lower_04",
  "lower_05", "lower_06", "lower_07", "lower_08", "lower_09",
  "lower_10", "lower_11", "lower_12", "lower_13", "lower_14",
];
const ARM_JOINT_NAMES = [
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
  "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "right_wrist_pitch_joint", "right_wrist_yaw_joint",
];

function makeManifest() {
  const activeJointNames = [...LOWER_BODY_NAMES, ...ARM_JOINT_NAMES];
  const defaultJointPosRad = activeJointNames.map((_, index) => index / 100);
  const joints = activeJointNames.map((name, index) => ({
    name,
    robot29Index: index,
    defaultRad: defaultJointPosRad[index],
    hardLimitRad: [-2, 2],
    softLimitRad: [-1, 1],
  }));
  const leftRoll = joints.find((joint) => joint.name === "left_shoulder_roll_joint");
  leftRoll.hardLimitRad = [-0.8, 0.6];
  leftRoll.softLimitRad = [-0.6, 0.5];
  const rightRoll = joints.find((joint) => joint.name === "right_shoulder_roll_joint");
  rightRoll.hardLimitRad = [-0.7, 0.9];
  rightRoll.softLimitRad = [-0.4, 0.8];
  return {
    activeJointNames,
    armJointNames: ARM_JOINT_NAMES,
    defaultJointPosRad,
    joints,
    assetHashes: {assetSignatureSha256: "a".repeat(64)},
  };
}

test("slider is soft-limited while numeric input can enter hard-only range", () => {
  const state = createPoseState(makeManifest());
  assert.deepEqual(state.control("left_elbow_joint").sliderRangeRad, [-1, 1]);
  state.setDegrees("left_elbow_joint", radiansToDegrees(1.5));
  assert.equal(state.warningFor("left_elbow_joint").kind, "soft-limit");
  assert.throws(() => state.setDegrees("left_elbow_joint", radiansToDegrees(2.1)), /hard limit/);
});

test("left reset never changes right arm or lower body", () => {
  const manifest = makeManifest();
  const state = createPoseState(manifest);
  state.setDegrees("left_elbow_joint", 70);
  state.setDegrees("right_elbow_joint", 80);
  const rightBefore = state.radians("right_elbow_joint");
  state.resetArm("left");
  assert.equal(state.radians("right_elbow_joint"), rightBefore);
  assert.deepEqual(state.composeRobot29().slice(0, 15), manifest.defaultJointPosRad.slice(0, 15));
});

test("invalid saved pose is all-or-nothing", () => {
  const state = createPoseState(makeManifest());
  const before = state.armPose();
  const invalid = {...before};
  delete invalid.left_elbow_joint;
  invalid.unknown_joint = 0;
  assert.throws(() => state.loadJointPose(invalid), /joint names mismatch/);
  assert.deepEqual(state.armPose(), before);
});

test("test_degree_radian_round_trip", () => {
  for (const degrees of [-180, -45, 0, 45, 180]) {
    assert.ok(Math.abs(radiansToDegrees(degreesToRadians(degrees)) - degrees) <= 1e-12);
  }
});

test("test_asymmetric_shoulder_roll_uses_manifest_ranges", () => {
  const state = createPoseState(makeManifest());
  assert.deepEqual(state.control("left_shoulder_roll_joint").sliderRangeRad, [-0.6, 0.5]);
  assert.deepEqual(state.control("right_shoulder_roll_joint").sliderRangeRad, [-0.4, 0.8]);
});

test("test_dirty_and_reset_all", () => {
  const state = createPoseState(makeManifest());
  assert.equal(state.isDirty(), false);
  state.setDegrees("left_elbow_joint", 30);
  assert.equal(state.isDirty(), true);
  state.resetAll();
  assert.deepEqual(state.armPose(), Object.fromEntries(
    ARM_JOINT_NAMES.map((name, index) => [name, (15 + index) / 100]),
  ));
  assert.equal(state.isDirty(), false);
});

test("test_load_is_all_or_nothing", () => {
  const state = createPoseState(makeManifest());
  state.setDegrees("left_elbow_joint", 30);
  const before = state.armPose();
  const cases = [
    (() => { const pose = {...before}; delete pose.left_elbow_joint; return pose; })(),
    {...before, extra_joint: 0},
    {...before, left_elbow_joint: Number.NaN},
    {...before, left_elbow_joint: 3},
  ];
  for (const invalid of cases) {
    assert.throws(() => state.loadJointPose(invalid));
    assert.deepEqual(state.armPose(), before);
  }
});

test("test_full_29_name_map_preserves_lower_body", () => {
  const manifest = makeManifest();
  const state = createPoseState(manifest);
  state.setDegrees("left_elbow_joint", 20);
  assert.deepEqual(
    Object.entries(state.jointPosByName29()).slice(0, 15),
    LOWER_BODY_NAMES.map((name, index) => [name, index / 100]),
  );
  assert.deepEqual(state.composeRobot29().slice(0, 15), manifest.defaultJointPosRad.slice(0, 15));
});

test("test_validation_payload_has_exact_contract", () => {
  const state = createPoseState(makeManifest());
  const browserFk = {frame: "robot_base_default", links: {}};
  const payload = state.validationPayload(browserFk);
  assert.deepEqual(Object.keys(payload), [
    "joint_pos_by_name", "browser_fk", "asset_signature_sha256",
  ]);
  assert.equal(payload.browser_fk, browserFk);
  assert.equal(payload.asset_signature_sha256, "a".repeat(64));
  assert.deepEqual(Object.keys(payload.joint_pos_by_name), ARM_JOINT_NAMES);
});

test("test_beforeunload_eligibility", () => {
  const state = createPoseState(makeManifest());
  assert.equal(state.shouldWarnBeforeUnload(), false);
  state.setDegrees("left_elbow_joint", 30);
  assert.equal(state.shouldWarnBeforeUnload(), true);
  assert.equal(state.markSaved({...state.armPose()}), true);
  assert.equal(state.shouldWarnBeforeUnload(), false);
  state.setDegrees("left_elbow_joint", 31);
  assert.equal(state.shouldWarnBeforeUnload(), true);
  assert.equal(state.markSaved({...state.armPose(), left_elbow_joint: 0}), false);
  assert.equal(state.shouldWarnBeforeUnload(), true);
});

test("setDegrees rejects blank and non-number values without changing state", () => {
  const state = createPoseState(makeManifest());
  state.setDegrees("left_elbow_joint", 30);
  const before = state.armPose();
  for (const value of ["", "   ", null, undefined, {}, []]) {
    assert.throws(() => state.setDegrees("left_elbow_joint", value), /number|finite/);
    assert.deepEqual(state.armPose(), before);
  }
});

test("createPoseState rejects shifted extra or missing active joint contracts", () => {
  const shifted = makeManifest();
  shifted.activeJointNames = [
    shifted.activeJointNames[1],
    shifted.activeJointNames[0],
    ...shifted.activeJointNames.slice(2),
  ];
  const extra = makeManifest();
  extra.activeJointNames = [...extra.activeJointNames, "extra_joint"];
  extra.defaultJointPosRad = [...extra.defaultJointPosRad, 0];
  const missing = makeManifest();
  missing.activeJointNames = missing.activeJointNames.slice(0, -1);
  missing.defaultJointPosRad = missing.defaultJointPosRad.slice(0, -1);
  const shiftedArmSlice = makeManifest();
  shiftedArmSlice.armJointNames = shiftedArmSlice.activeJointNames.slice(14, 28);
  for (const malformed of [shifted, extra, missing, shiftedArmSlice]) {
    assert.throws(() => createPoseState(malformed), /manifest.*joint|joint.*manifest/);
  }
});

test("createPoseState rejects an arm name without a matching joint spec", () => {
  const manifest = makeManifest();
  manifest.joints = manifest.joints.filter(
    (joint) => joint.name !== "left_elbow_joint",
  );
  assert.throws(() => createPoseState(manifest), /invalid manifest joint/);
});
