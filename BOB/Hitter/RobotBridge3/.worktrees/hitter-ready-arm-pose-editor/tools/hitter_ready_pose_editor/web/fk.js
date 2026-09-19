import {
  identity4,
  matrixToPoseXyzw,
  multiply4,
  rotationAxisAngle4,
  rotationRpy4,
  translation4,
} from "./matrix.js";

const HITTER_ROOT_LINK = "pelvis";
const HITTER_CRITICAL_LINK_NAMES = Object.freeze([
  "left_wrist_yaw_link",
  "right_wrist_yaw_link",
  "right_racket_link",
]);

function requireObject(value, name) {
  if (value === null || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError(`${name} must be an object`);
  }
}

export function computeForwardKinematics(model, jointPositionsRad) {
  requireObject(model, "model");
  requireObject(jointPositionsRad, "joint positions");
  if (typeof model.rootLink !== "string" || model.rootLink.length === 0) {
    throw new TypeError("model rootLink must be a nonempty string");
  }
  if (!Array.isArray(model.activeJointNames)) {
    throw new TypeError("model activeJointNames must be an array");
  }
  if (!Array.isArray(model.joints)) {
    throw new TypeError("model joints must be an array");
  }

  const expected = new Set();
  for (const name of model.activeJointNames) {
    if (typeof name !== "string" || name.length === 0) {
      throw new TypeError("active joint name must be a nonempty string");
    }
    if (expected.has(name)) {
      throw new TypeError(`duplicate active joint ${name}`);
    }
    expected.add(name);
  }
  for (const name of Object.keys(jointPositionsRad)) {
    if (!expected.has(name)) {
      throw new Error(`unknown ${name}`);
    }
  }
  for (const name of expected) {
    if (!Object.hasOwn(jointPositionsRad, name)) {
      throw new Error(`missing ${name}`);
    }
  }

  const linkWorldMatrices = {[model.rootLink]: identity4()};
  const consumed = new Set();
  for (const joint of model.joints) {
    requireObject(joint, "joint");
    if (
      typeof joint.name !== "string" ||
      typeof joint.parentLink !== "string" ||
      typeof joint.childLink !== "string"
    ) {
      throw new TypeError("joint names and links must be strings");
    }
    const parent = linkWorldMatrices[joint.parentLink];
    if (!parent) {
      throw new Error(`unresolved parent ${joint.parentLink}`);
    }
    if (Object.hasOwn(linkWorldMatrices, joint.childLink)) {
      throw new Error(`duplicate child link ${joint.childLink}`);
    }
    const origin = multiply4(
      translation4(joint.originXyzM),
      rotationRpy4(joint.originRpyRad),
    );
    let local = origin;
    if (joint.type === "revolute") {
      if (!expected.has(joint.name)) {
        throw new Error(`revolute joint ${joint.name} is not active`);
      }
      if (consumed.has(joint.name)) {
        throw new Error(`duplicate revolute joint ${joint.name}`);
      }
      consumed.add(joint.name);
      const q = jointPositionsRad[joint.name];
      if (!Number.isFinite(q)) {
        throw new Error(`${joint.name} must be finite`);
      }
      if (
        !Array.isArray(joint.hardLimitRad) ||
        joint.hardLimitRad.length !== 2 ||
        !joint.hardLimitRad.every(Number.isFinite) ||
        joint.hardLimitRad[0] > joint.hardLimitRad[1]
      ) {
        throw new Error(`${joint.name} has invalid hard limit`);
      }
      const [lower, upper] = joint.hardLimitRad;
      if (q < lower || q > upper) {
        throw new Error(`${joint.name} outside hard limit`);
      }
      local = multiply4(
        origin,
        rotationAxisAngle4(joint.axisXyz, q),
      );
    } else if (joint.type !== "fixed") {
      throw new Error(`unsupported joint type ${joint.type}`);
    }
    linkWorldMatrices[joint.childLink] = multiply4(parent, local);
  }
  for (const name of expected) {
    if (!consumed.has(name)) {
      throw new Error(`active joint ${name} is not in model joints`);
    }
  }
  return {
    rootLink: model.rootLink,
    linkWorldMatrices,
  };
}

export function extractFkSummary(result, linkNames) {
  requireObject(result, "FK result");
  requireObject(result.linkWorldMatrices, "FK link matrices");
  if (typeof result.rootLink !== "string" || result.rootLink.length === 0) {
    throw new TypeError("FK result rootLink must be a nonempty string");
  }
  if (!Array.isArray(linkNames)) {
    throw new TypeError("linkNames must be an array");
  }
  const links = {};
  for (const name of linkNames) {
    if (
      typeof name !== "string" ||
      !Object.hasOwn(result.linkWorldMatrices, name)
    ) {
      throw new Error(`missing link ${name}`);
    }
    const pose = matrixToPoseXyzw(result.linkWorldMatrices[name]);
    links[name] = {
      position_m: pose.position,
      quaternion_xyzw: pose.quaternion,
    };
  }
  return {
    rootLink: result.rootLink,
    links,
  };
}

export function extractHitterFkSummary(result) {
  requireObject(result, "FK result");
  if (result.rootLink !== HITTER_ROOT_LINK) {
    throw new Error("HITTER FK root must be pelvis");
  }
  const generic = extractFkSummary(result, HITTER_CRITICAL_LINK_NAMES);
  return {
    frame: "robot_base_default",
    quaternion_convention: "xyzw",
    links: generic.links,
  };
}
