import assert from "node:assert/strict";
import {createHash} from "node:crypto";
import {readFile} from "node:fs/promises";
import test from "node:test";

import {computeForwardKinematics} from "../fk.js";
import {matrixToPoseXyzw} from "../matrix.js";

const EXPECTED_ACTIVE_JOINT_NAMES = Object.freeze([
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
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
]);

const EXPECTED_ARM_JOINT_NAMES = Object.freeze(
  EXPECTED_ACTIVE_JOINT_NAMES.slice(15),
);

const EXPECTED_COMPARISON_LINK_NAMES = Object.freeze([
  "pelvis",
  "left_hip_pitch_link",
  "left_hip_roll_link",
  "left_hip_yaw_link",
  "left_knee_link",
  "left_ankle_pitch_link",
  "left_ankle_roll_link",
  "right_hip_pitch_link",
  "right_hip_roll_link",
  "right_hip_yaw_link",
  "right_knee_link",
  "right_ankle_pitch_link",
  "right_ankle_roll_link",
  "waist_yaw_link",
  "waist_roll_link",
  "torso_link",
  "left_shoulder_pitch_link",
  "left_shoulder_roll_link",
  "left_shoulder_yaw_link",
  "left_elbow_link",
  "left_wrist_roll_link",
  "left_wrist_pitch_link",
  "left_wrist_yaw_link",
  "right_shoulder_pitch_link",
  "right_shoulder_roll_link",
  "right_shoulder_yaw_link",
  "right_elbow_link",
  "right_wrist_roll_link",
  "right_wrist_pitch_link",
  "right_wrist_yaw_link",
  "right_racket_link",
]);

const EXPECTED_FIRST_RANDOM_ARM_VECTOR = Object.freeze([
  0.993353258941966,
  1.7484472934735376,
  0.4481844803991706,
  1.1274634885886543,
  0.8830717018318022,
  -0.2603179409447023,
  0.7516731209274405,
  0.018022511219104587,
  -1.0098845959014013,
  0.9134642857976445,
  1.3128523871781408,
  0.35765429192632814,
  0.4108697960504679,
  -1.1085259781208885,
]);

const EXPECTED_LAST_RANDOM_ARM_VECTOR = Object.freeze([
  -1.3082476872807136,
  -0.19907774786679133,
  1.701906388621155,
  -0.48566199239514507,
  0.7729580136641272,
  0.07847577505374681,
  1.5090886127093779,
  1.4517536683221794,
  -0.38069710183908834,
  1.3728333090743892,
  0.7086263237288053,
  -0.008018004386375788,
  0.3604103217510397,
  1.0014008616193306,
]);

const EXPECTED_RANDOM_SEQUENCE_SHA256 =
  "93503e6b529ac1b99394974e953c23e52a0736d0ef086d91d788c64179628f00";
const EXPECTED_TOPOLOGICAL_TREE_SHA256 =
  "dd768fe6a750c26e528aa0853921680afb12e88130c5c8c07223e063b217e690";
const EXPECTED_FIRST_TOPOLOGICAL_JOINT = Object.freeze([
  "pelvis_contour_joint",
  "fixed",
  "pelvis",
  "pelvis_contour_link",
]);
const EXPECTED_LAST_TOPOLOGICAL_JOINT = Object.freeze([
  "imu_in_pelvis_joint",
  "fixed",
  "pelvis",
  "imu_in_pelvis",
]);

function canonicalSha256(value) {
  return createHash("sha256")
    .update(JSON.stringify(value), "utf8")
    .digest("hex");
}

const contractPath = process.env.HITTER_LIVE_FK_CONTRACT;
assert.ok(contractPath, "HITTER_LIVE_FK_CONTRACT is required");
const contract = JSON.parse(await readFile(contractPath, "utf8"));

test("live contract preserves the exact production asset and sampling contract", () => {
  assert.equal(contract.schema, "hitter_live_fk_contract/v1");
  assert.equal(contract.frame, "robot_base_default");
  assert.equal(contract.quaternionConvention, "xyzw");
  assert.equal(contract.angleUnit, "rad");
  assert.equal(contract.coordinateHandedness, "right");
  assert.equal(contract.matrixLayout, "column-major");
  assert.equal(contract.randomSeed, 20260726);
  assert.equal(contract.nearSoftBoundaryEpsilonRad, 1e-6);
  assert.equal(contract.randomHardBoundaryEpsilonRad, 1e-5);

  assert.deepEqual(
    contract.manifest.activeJointNames,
    EXPECTED_ACTIVE_JOINT_NAMES,
  );
  assert.deepEqual(
    contract.manifest.armJointNames,
    EXPECTED_ARM_JOINT_NAMES,
  );
  assert.equal(contract.manifest.joints.length, 37);
  const topologicalTree = contract.manifest.joints.map((joint) => [
    joint.name,
    joint.type,
    joint.parentLink,
    joint.childLink,
  ]);
  assert.deepEqual(
    topologicalTree[0],
    EXPECTED_FIRST_TOPOLOGICAL_JOINT,
  );
  assert.deepEqual(
    topologicalTree.at(-1),
    EXPECTED_LAST_TOPOLOGICAL_JOINT,
  );
  assert.equal(
    canonicalSha256(topologicalTree),
    EXPECTED_TOPOLOGICAL_TREE_SHA256,
    "exact 37-joint topological tree changed",
  );
  assert.equal(contract.manifest.frame.name, "robot_base_default");
  assert.equal(contract.manifest.frame.rootLink, "pelvis");
  assert.equal(contract.manifest.frame.rootTransform, "identity");
  assert.equal(contract.manifest.frame.handedness, "right");
  assert.equal(contract.manifest.frame.matrixLayout, "column-major");
  assert.equal(contract.manifest.frame.quaternionConvention, "xyzw");
  assert.equal(contract.manifest.units.angle, "rad");

  assert.deepEqual(
    contract.comparisonLinkNames,
    EXPECTED_COMPARISON_LINK_NAMES,
  );
  assert.deepEqual(
    contract.manifest.joints
      .filter((joint) => joint.type === "revolute")
      .map((joint) => joint.childLink),
    EXPECTED_COMPARISON_LINK_NAMES.slice(1, 30),
  );
  assert.equal(contract.manifest.rightRacketLink, "right_racket_link");

  assert.deepEqual(contract.asset.hashes, {
    displayUrdfSha256: contract.manifest.assetHashes.displayUrdfSha256,
    displayMeshSetSha256:
      contract.manifest.assetHashes.displayMeshSetSha256,
    validationMjcfSha256:
      contract.manifest.assetHashes.validationMjcfSha256,
    assetYamlSha256: contract.manifest.assetHashes.assetYamlSha256,
    urdfKinematicSha256:
      contract.manifest.assetHashes.urdfKinematicSha256,
    mjcfKinematicSha256:
      contract.manifest.assetHashes.mjcfKinematicSha256,
    assetSignatureSha256:
      contract.manifest.assetHashes.assetSignatureSha256,
  });
  assert.equal(
    contract.asset.combinedAssetSignatureSha256,
    contract.manifest.assetHashes.assetSignatureSha256,
  );
  for (const key of ["urdf", "mjcf", "assetYaml", "allowedAssetRoot"]) {
    assert.equal(typeof contract.asset.paths[key], "string");
    assert.ok(contract.asset.paths[key].startsWith("/"), `${key} is absolute`);
  }

  assert.equal(contract.poses.length, 129);
  assert.equal(
    contract.poses.filter((sample) => sample.kind === "default").length,
    1,
  );
  assert.equal(
    contract.poses.filter((sample) => sample.kind === "near_soft_boundary")
      .length,
    28,
  );
  assert.equal(
    contract.poses.filter((sample) => sample.kind === "seeded_random").length,
    100,
  );
});

test("all 129 live browser poses match MuJoCo in robot_base_default", () => {
  const jointByName = Object.fromEntries(
    contract.manifest.joints.map((joint) => [joint.name, joint]),
  );
  const defaultByName = Object.fromEntries(
    contract.manifest.activeJointNames.map((name, index) => [
      name,
      contract.manifest.defaultJointPosRad[index],
    ]),
  );
  const defaultSample = contract.poses[0];
  const boundarySamples = contract.poses.slice(1, 29);
  const randomSamples = contract.poses.slice(29);
  assert.equal(defaultSample.kind, "default");
  assert.deepEqual(defaultSample.jointPosByName29, defaultByName);
  assert.equal(boundarySamples.length, 28);
  assert.equal(randomSamples.length, 100);

  for (const [index, sample] of boundarySamples.entries()) {
    const variedJointName =
      EXPECTED_ARM_JOINT_NAMES[Math.floor(index / 2)];
    const boundary = index % 2 === 0 ? "min" : "max";
    assert.equal(sample.kind, "near_soft_boundary");
    assert.equal(sample.variedJointName, variedJointName);
    assert.equal(sample.boundary, boundary);
    assert.deepEqual(
      Object.keys(sample).sort(),
      [
        "boundary",
        "jointPosByName29",
        "kind",
        "mujocoRobotBaseDefault",
        "variedJointName",
      ].sort(),
    );
    for (const name of EXPECTED_ACTIVE_JOINT_NAMES) {
      const actual = sample.jointPosByName29[name];
      if (name === variedJointName) {
        const [softMin, softMax] = jointByName[name].softLimitRad;
        const expected =
          boundary === "min" ? softMin + 1e-6 : softMax - 1e-6;
        assert.equal(actual, expected);
      } else {
        assert.equal(actual, defaultByName[name]);
      }
    }
  }

  const randomArmSequence = randomSamples.map((sample, randomIndex) => {
    assert.equal(sample.kind, "seeded_random");
    assert.equal(sample.randomIndex, randomIndex);
    assert.deepEqual(
      Object.keys(sample).sort(),
      [
        "jointPosByName29",
        "kind",
        "mujocoRobotBaseDefault",
        "randomIndex",
      ].sort(),
    );
    for (const name of EXPECTED_ACTIVE_JOINT_NAMES.slice(0, 15)) {
      assert.equal(sample.jointPosByName29[name], defaultByName[name]);
    }
    return EXPECTED_ARM_JOINT_NAMES.map(
      (name) => sample.jointPosByName29[name],
    );
  });
  assert.deepEqual(
    randomArmSequence[0],
    EXPECTED_FIRST_RANDOM_ARM_VECTOR,
  );
  assert.deepEqual(
    randomArmSequence.at(-1),
    EXPECTED_LAST_RANDOM_ARM_VECTOR,
  );
  assert.equal(
    canonicalSha256(randomArmSequence),
    EXPECTED_RANDOM_SEQUENCE_SHA256,
    "NumPy default_rng(20260726) 100x14 sequence changed",
  );

  for (const sample of contract.poses) {
    assert.deepEqual(Object.keys(sample.jointPosByName29), [
      ...EXPECTED_ACTIVE_JOINT_NAMES,
    ]);
    assert.deepEqual(Object.keys(sample.mujocoRobotBaseDefault), [
      ...EXPECTED_COMPARISON_LINK_NAMES,
    ]);
    if (sample.kind === "seeded_random") {
      for (const name of EXPECTED_ACTIVE_JOINT_NAMES) {
        const actual = sample.jointPosByName29[name];
        if (EXPECTED_ARM_JOINT_NAMES.includes(name)) {
          const [lower, upper] = jointByName[name].hardLimitRad;
          assert.ok(actual >= lower + 1e-5);
          assert.ok(actual <= upper - 1e-5);
        } else {
          assert.equal(actual, defaultByName[name]);
        }
      }
    }
  }

  let maxPositionErrorM = 0;
  let maxOrientationErrorRad = 0;
  for (const sample of contract.poses) {
    const browser = computeForwardKinematics(
      contract.manifest,
      sample.jointPosByName29,
    );
    for (const linkName of contract.comparisonLinkNames) {
      const actual = matrixToPoseXyzw(browser.linkWorldMatrices[linkName]);
      const expected = sample.mujocoRobotBaseDefault[linkName];
      const positionError = Math.hypot(
        actual.position[0] - expected.position_m[0],
        actual.position[1] - expected.position_m[1],
        actual.position[2] - expected.position_m[2],
      );
      const dot = Math.abs(
        actual.quaternion[0] * expected.quaternion_xyzw[0] +
          actual.quaternion[1] * expected.quaternion_xyzw[1] +
          actual.quaternion[2] * expected.quaternion_xyzw[2] +
          actual.quaternion[3] * expected.quaternion_xyzw[3],
      );
      const orientationError = 2 * Math.acos(Math.min(1, dot));
      maxPositionErrorM = Math.max(maxPositionErrorM, positionError);
      maxOrientationErrorRad = Math.max(
        maxOrientationErrorRad,
        orientationError,
      );
    }
  }
  console.log(
    JSON.stringify({
      poseCount: contract.poses.length,
      linkCount: contract.comparisonLinkNames.length,
      maxPositionErrorM,
      maxOrientationErrorRad,
      assetSignatureSha256:
        contract.manifest.assetHashes.assetSignatureSha256,
    }),
  );
  assert.ok(maxPositionErrorM <= 0.0005, `${maxPositionErrorM}`);
  assert.ok(
    maxOrientationErrorRad <= Math.PI / 1800,
    `${maxOrientationErrorRad}`,
  );
});
