import test from "node:test";
import assert from "node:assert/strict";

import * as fk from "../fk.js";
import {transformPoint} from "../matrix.js";
const {computeForwardKinematics, extractFkSummary} = fk;

function assertVectorClose(actual, expected, tolerance = 1e-12) {
  assert.equal(actual.length, expected.length);
  for (let index = 0; index < expected.length; index += 1) {
    assert.ok(
      Math.abs(actual[index] - expected[index]) <= tolerance,
      `component ${index}: expected ${expected[index]}, got ${actual[index]}`,
    );
  }
}

function twoJointModel() {
  return {
    rootLink: "base",
    activeJointNames: ["joint_a"],
    joints: [
      {
        name: "joint_a",
        type: "revolute",
        parentLink: "base",
        childLink: "arm",
        originXyzM: [1, 0, 0],
        originRpyRad: [0, 0, 0],
        axisXyz: [0, 0, 1],
        hardLimitRad: [-Math.PI, Math.PI],
        defaultRad: 0,
        robot29Index: 0,
      },
      {
        name: "tool_fixed",
        type: "fixed",
        parentLink: "arm",
        childLink: "tool",
        originXyzM: [0.25, 0, 0],
        originRpyRad: [0, 0, 0],
        axisXyz: null,
        hardLimitRad: null,
        defaultRad: null,
        robot29Index: null,
      },
    ],
  };
}

function hitterSummaryModel({includeRacket = true} = {}) {
  const joints = [
    {
      name: "left_wrist_fixed",
      type: "fixed",
      parentLink: "pelvis",
      childLink: "left_wrist_yaw_link",
      originXyzM: [1, 0, 0],
      originRpyRad: [0, 0, 0],
      axisXyz: null,
      hardLimitRad: null,
      defaultRad: null,
      robot29Index: null,
    },
    {
      name: "right_wrist_fixed",
      type: "fixed",
      parentLink: "pelvis",
      childLink: "right_wrist_yaw_link",
      originXyzM: [0, 2, 0],
      originRpyRad: [0, 0, 0],
      axisXyz: null,
      hardLimitRad: null,
      defaultRad: null,
      robot29Index: null,
    },
  ];
  if (includeRacket) {
    joints.push({
      name: "right_racket_fixed",
      type: "fixed",
      parentLink: "right_wrist_yaw_link",
      childLink: "right_racket_link",
      originXyzM: [0, 0, 3],
      originRpyRad: [0, 0, 0],
      axisXyz: null,
      hardLimitRad: null,
      defaultRad: null,
      robot29Index: null,
    });
  }
  return {
    rootLink: "pelvis",
    activeJointNames: [],
    joints,
  };
}

test("FK applies parent, joint origin, axis rotation, then child", () => {
  const result = computeForwardKinematics(twoJointModel(), {
    joint_a: Math.PI / 2,
  });
  assertVectorClose(
    transformPoint(result.linkWorldMatrices.arm, [1, 0, 0]),
    [1, 1, 0],
  );
  assertVectorClose(
    transformPoint(result.linkWorldMatrices.tool, [0, 0, 0]),
    [1, 0.25, 0],
  );
});

test("FK rejects missing, unknown, nonfinite, and out-of-range values", () => {
  const model = twoJointModel();
  assert.throws(
    () => computeForwardKinematics(model, {}),
    /missing joint_a/,
  );
  assert.throws(
    () => computeForwardKinematics(model, {joint_a: 0, other: 0}),
    /unknown other/,
  );
  assert.throws(
    () => computeForwardKinematics(model, {joint_a: NaN}),
    /finite/,
  );
  assert.throws(
    () => computeForwardKinematics(model, {joint_a: 4}),
    /hard limit/,
  );
});

test("fixed joints consume no active value and all topological joints are applied", () => {
  const result = computeForwardKinematics(twoJointModel(), {joint_a: 0});
  assert.deepEqual(
    Object.keys(result.linkWorldMatrices),
    ["base", "arm", "tool"],
  );
  assertVectorClose(
    transformPoint(result.linkWorldMatrices.tool, [0, 0, 0]),
    [1.25, 0, 0],
  );
});

test("FK rejects unresolved parents and unsupported joint types", () => {
  const unresolved = twoJointModel();
  unresolved.joints[0].parentLink = "missing";
  assert.throws(
    () => computeForwardKinematics(unresolved, {joint_a: 0}),
    /unresolved parent missing/,
  );

  const unsupported = twoJointModel();
  unsupported.joints[1].type = "prismatic";
  assert.throws(
    () => computeForwardKinematics(unsupported, {joint_a: 0}),
    /unsupported joint type prismatic/,
  );
});

test("generic FK summary reports its root without claiming robot_base_default", () => {
  const result = computeForwardKinematics(twoJointModel(), {
    joint_a: Math.PI / 2,
  });
  const summary = extractFkSummary(result, ["arm", "tool"]);
  assert.equal(summary.rootLink, "base");
  assert.equal(Object.hasOwn(summary, "frame"), false);
  assert.equal(Object.hasOwn(summary, "quaternion_convention"), false);
  assert.deepEqual(Object.keys(summary.links), ["arm", "tool"]);
  assertVectorClose(summary.links.arm.position_m, [1, 0, 0]);
  assertVectorClose(
    summary.links.arm.quaternion_xyzw,
    [0, 0, Math.SQRT1_2, Math.SQRT1_2],
  );
  assertVectorClose(summary.links.tool.position_m, [1, 0.25, 0]);
  assertVectorClose(
    summary.links.tool.quaternion_xyzw,
    [0, 0, Math.SQRT1_2, Math.SQRT1_2],
  );
});

test("extractFkSummary rejects a requested link without a transform", () => {
  const result = computeForwardKinematics(twoJointModel(), {joint_a: 0});
  assert.throws(
    () => extractFkSummary(result, ["missing"]),
    /missing link missing/,
  );
});

test("HITTER FK summary rejects a non-pelvis FK root", () => {
  const result = computeForwardKinematics(twoJointModel(), {joint_a: 0});
  assert.throws(
    () => fk.extractHitterFkSummary(result),
    /root must be pelvis/,
  );
});

test("HITTER FK summary rejects a missing critical link", () => {
  const result = computeForwardKinematics(
    hitterSummaryModel({includeRacket: false}),
    {},
  );
  assert.throws(
    () => fk.extractHitterFkSummary(result),
    /missing link right_racket_link/,
  );
});

test("HITTER FK summary emits only the exact canonical critical links", () => {
  const result = computeForwardKinematics(hitterSummaryModel(), {});
  assert.deepEqual(fk.extractHitterFkSummary(result), {
    frame: "robot_base_default",
    quaternion_convention: "xyzw",
    links: {
      left_wrist_yaw_link: {
        position_m: [1, 0, 0],
        quaternion_xyzw: [0, 0, 0, 1],
      },
      right_wrist_yaw_link: {
        position_m: [0, 2, 0],
        quaternion_xyzw: [0, 0, 0, 1],
      },
      right_racket_link: {
        position_m: [0, 2, 3],
        quaternion_xyzw: [0, 0, 0, 1],
      },
    },
  });
});
