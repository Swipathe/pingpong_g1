export const degreesToRadians = (value) => value * Math.PI / 180;
export const radiansToDegrees = (value) => value * 180 / Math.PI;

function equalJointNames(actual, expected) {
  return (
    actual.length === expected.length &&
    actual.every((name) => expected.includes(name))
  );
}

export function createPoseState(manifest) {
  if (
    manifest === null ||
    typeof manifest !== "object" ||
    !Array.isArray(manifest.armJointNames) ||
    !Array.isArray(manifest.activeJointNames) ||
    !Array.isArray(manifest.defaultJointPosRad) ||
    !Array.isArray(manifest.joints)
  ) {
    throw new TypeError("invalid pose manifest");
  }
  if (
    manifest.activeJointNames.length !== 29 ||
    manifest.defaultJointPosRad.length !== 29 ||
    new Set(manifest.activeJointNames).size !== 29 ||
    !manifest.activeJointNames.every(
      (name) => typeof name === "string" && name.length > 0,
    ) ||
    !manifest.defaultJointPosRad.every(Number.isFinite) ||
    manifest.armJointNames.length !== 14 ||
    new Set(manifest.armJointNames).size !== 14 ||
    !manifest.armJointNames.every(
      (name, index) => name === manifest.activeJointNames[index + 15],
    )
  ) {
    throw new Error("invalid pose manifest joint names");
  }

  const specs = new Map(manifest.joints.map((spec) => [spec.name, spec]));
  for (const [index, name] of manifest.activeJointNames.entries()) {
    const spec = specs.get(name);
    if (
      !spec ||
      spec.robot29Index !== index ||
      spec.defaultRad !== manifest.defaultJointPosRad[index]
    ) {
      throw new Error(`invalid manifest joint contract ${name}`);
    }
  }
  for (const name of manifest.armJointNames) {
    const spec = specs.get(name);
    if (
      !spec ||
      !Number.isFinite(spec.defaultRad) ||
      !Array.isArray(spec.hardLimitRad) ||
      spec.hardLimitRad.length !== 2 ||
      !spec.hardLimitRad.every(Number.isFinite) ||
      spec.hardLimitRad[0] > spec.hardLimitRad[1] ||
      !Array.isArray(spec.softLimitRad) ||
      spec.softLimitRad.length !== 2 ||
      !spec.softLimitRad.every(Number.isFinite) ||
      spec.softLimitRad[0] > spec.softLimitRad[1] ||
      spec.softLimitRad[0] < spec.hardLimitRad[0] ||
      spec.softLimitRad[1] > spec.hardLimitRad[1]
    ) {
      throw new Error(`invalid manifest joint ${name}`);
    }
  }

  const initial = Object.fromEntries(
    manifest.armJointNames.map((name) => [name, specs.get(name).defaultRad]),
  );
  let current = {...initial};
  let saved = {...initial};

  function checkedValue(name, value) {
    if (!manifest.armJointNames.includes(name)) {
      throw new Error(`unknown ${name}`);
    }
    if (!Number.isFinite(value)) {
      throw new Error(`${name} must be finite`);
    }
    const [lower, upper] = specs.get(name).hardLimitRad;
    if (value < lower || value > upper) {
      throw new Error(`${name} outside hard limit`);
    }
    return value;
  }

  function checkedArmPose(jointPosByName) {
    if (
      jointPosByName === null ||
      typeof jointPosByName !== "object" ||
      Array.isArray(jointPosByName) ||
      !equalJointNames(Object.keys(jointPosByName), manifest.armJointNames)
    ) {
      throw new Error("joint names mismatch");
    }
    return Object.fromEntries(
      manifest.armJointNames.map((name) => [
        name,
        checkedValue(name, jointPosByName[name]),
      ]),
    );
  }

  function armPose() {
    return Object.fromEntries(
      manifest.armJointNames.map((name) => [name, current[name]]),
    );
  }

  return {
    setDegrees(name, degrees) {
      if (typeof degrees !== "number" || !Number.isFinite(degrees)) {
        throw new Error(`${name} degrees must be a finite number`);
      }
      current = {...current, [name]: checkedValue(name, degreesToRadians(degrees))};
    },
    setSliderRadians(name, radians) {
      if (!manifest.armJointNames.includes(name)) {
        throw new Error(`unknown ${name}`);
      }
      const [lower, upper] = specs.get(name).softLimitRad;
      if (!Number.isFinite(radians) || radians < lower || radians > upper) {
        throw new Error(`${name} outside slider range`);
      }
      current = {...current, [name]: checkedValue(name, radians)};
    },
    radians(name) {
      if (!manifest.armJointNames.includes(name)) {
        throw new Error(`unknown ${name}`);
      }
      return current[name];
    },
    control(name) {
      if (!manifest.armJointNames.includes(name)) {
        throw new Error(`unknown ${name}`);
      }
      const spec = specs.get(name);
      return {
        hardRangeRad: [...spec.hardLimitRad],
        sliderRangeRad: [...spec.softLimitRad],
        defaultRad: initial[name],
        currentRad: current[name],
      };
    },
    warningFor(name) {
      const value = this.radians(name);
      const [lower, upper] = specs.get(name).softLimitRad;
      return value < lower || value > upper
        ? {kind: "soft-limit", message: "当前值超出建议滑块范围"}
        : null;
    },
    armPose,
    jointPosByName29() {
      return Object.fromEntries(manifest.activeJointNames.map((name, index) => [
        name,
        current[name] ?? manifest.defaultJointPosRad[index],
      ]));
    },
    composeRobot29() {
      const byName = this.jointPosByName29();
      return manifest.activeJointNames.map((name) => byName[name]);
    },
    isDirty() {
      return manifest.armJointNames.some((name) => current[name] !== saved[name]);
    },
    shouldWarnBeforeUnload() {
      return this.isDirty();
    },
    resetArm(side) {
      if (side !== "left" && side !== "right") {
        throw new Error("arm side must be left or right");
      }
      const prefix = `${side}_`;
      current = {
        ...current,
        ...Object.fromEntries(manifest.armJointNames
          .filter((name) => name.startsWith(prefix))
          .map((name) => [name, initial[name]])),
      };
    },
    resetAll() {
      current = {...initial};
    },
    loadJointPose(jointPosByName) {
      const next = checkedArmPose(jointPosByName);
      current = next;
    },
    validationPayload(browserFk) {
      return {
        joint_pos_by_name: armPose(),
        browser_fk: browserFk,
        asset_signature_sha256: manifest.assetHashes?.assetSignatureSha256,
      };
    },
    markSaved(validatedJointPosByName) {
      try {
        const acknowledged = checkedArmPose(validatedJointPosByName);
        if (manifest.armJointNames.some((name) => acknowledged[name] !== current[name])) {
          return false;
        }
        saved = acknowledged;
        return true;
      } catch {
        return false;
      }
    },
  };
}
