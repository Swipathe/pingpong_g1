const MATRIX_LENGTH = 16;
const VECTOR_LENGTH = 3;
const RIGID_TOLERANCE = 1e-9;
const BOUNDS_LENGTH = 6;
const MIN_CAMERA_RADIUS = 0.05;

function finiteVector(value, length, name) {
  if (
    value === null ||
    value === undefined ||
    typeof value.length !== "number" ||
    value.length !== length
  ) {
    throw new TypeError(`${name} must have length ${length}`);
  }
  const result = Array.from(value);
  if (!result.every(Number.isFinite)) {
    throw new TypeError(`${name} values must be finite`);
  }
  return result;
}

function finiteMatrix(value, name = "matrix") {
  return finiteVector(value, MATRIX_LENGTH, name);
}

function finiteBounds(value, name = "bounds") {
  const result = finiteVector(value, BOUNDS_LENGTH, name);
  for (let axis = 0; axis < 3; axis += 1) {
    if (result[axis] > result[axis + 3]) {
      throw new TypeError(`${name} minimum must not exceed maximum`);
    }
  }
  return result;
}

function dot3(left, right) {
  return (
    left[0] * right[0] +
    left[1] * right[1] +
    left[2] * right[2]
  );
}

function cross3(left, right) {
  return [
    left[1] * right[2] - left[2] * right[1],
    left[2] * right[0] - left[0] * right[2],
    left[0] * right[1] - left[1] * right[0],
  ];
}

function normalize3(vector, name) {
  const value = finiteVector(vector, VECTOR_LENGTH, name);
  const length = Math.hypot(...value);
  if (!Number.isFinite(length) || length === 0) {
    throw new TypeError(`${name} must be nonzero`);
  }
  return value.map((component) => component / length);
}

function assertRigid(matrix) {
  const value = finiteMatrix(matrix);
  if (
    Math.abs(value[3]) > RIGID_TOLERANCE ||
    Math.abs(value[7]) > RIGID_TOLERANCE ||
    Math.abs(value[11]) > RIGID_TOLERANCE ||
    Math.abs(value[15] - 1) > RIGID_TOLERANCE
  ) {
    throw new TypeError("matrix must be rigid");
  }

  for (let left = 0; left < 3; left += 1) {
    for (let right = left; right < 3; right += 1) {
      let dot = 0;
      for (let row = 0; row < 3; row += 1) {
        dot += value[left * 4 + row] * value[right * 4 + row];
      }
      const expected = left === right ? 1 : 0;
      if (Math.abs(dot - expected) > RIGID_TOLERANCE) {
        throw new TypeError("matrix must be rigid");
      }
    }
  }

  const m00 = value[0];
  const m01 = value[4];
  const m02 = value[8];
  const m10 = value[1];
  const m11 = value[5];
  const m12 = value[9];
  const m20 = value[2];
  const m21 = value[6];
  const m22 = value[10];
  const determinant = (
    m00 * (m11 * m22 - m12 * m21) -
    m01 * (m10 * m22 - m12 * m20) +
    m02 * (m10 * m21 - m11 * m20)
  );
  if (Math.abs(determinant - 1) > RIGID_TOLERANCE) {
    throw new TypeError("matrix must be rigid");
  }
  return value;
}

export function identity4() {
  return new Float64Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]);
}

export function multiply4(left, right) {
  const a = finiteMatrix(left, "left matrix");
  const b = finiteMatrix(right, "right matrix");
  const result = new Float64Array(MATRIX_LENGTH);
  for (let column = 0; column < 4; column += 1) {
    for (let row = 0; row < 4; row += 1) {
      let value = 0;
      for (let inner = 0; inner < 4; inner += 1) {
        value += a[inner * 4 + row] * b[column * 4 + inner];
      }
      result[column * 4 + row] = value;
    }
  }
  return result;
}

export function translation4(xyz) {
  const [x, y, z] = finiteVector(xyz, VECTOR_LENGTH, "translation");
  const result = identity4();
  result[12] = x;
  result[13] = y;
  result[14] = z;
  return result;
}

export function scale4(xyz) {
  const [x, y, z] = finiteVector(xyz, VECTOR_LENGTH, "scale");
  const result = identity4();
  result[0] = x;
  result[5] = y;
  result[10] = z;
  return result;
}

export function rotationAxisAngle4(axis, angleRad) {
  const [rawX, rawY, rawZ] = finiteVector(
    axis,
    VECTOR_LENGTH,
    "rotation axis",
  );
  if (!Number.isFinite(angleRad)) {
    throw new TypeError("rotation angle must be finite");
  }
  const length = Math.hypot(rawX, rawY, rawZ);
  if (!Number.isFinite(length) || length === 0) {
    throw new TypeError("rotation axis must be finite and nonzero");
  }
  const x = rawX / length;
  const y = rawY / length;
  const z = rawZ / length;
  const cosine = Math.cos(angleRad);
  const sine = Math.sin(angleRad);
  const oneMinusCosine = 1 - cosine;

  const result = identity4();
  result[0] = cosine + x * x * oneMinusCosine;
  result[1] = y * x * oneMinusCosine + z * sine;
  result[2] = z * x * oneMinusCosine - y * sine;
  result[4] = x * y * oneMinusCosine - z * sine;
  result[5] = cosine + y * y * oneMinusCosine;
  result[6] = z * y * oneMinusCosine + x * sine;
  result[8] = x * z * oneMinusCosine + y * sine;
  result[9] = y * z * oneMinusCosine - x * sine;
  result[10] = cosine + z * z * oneMinusCosine;
  return result;
}

export function rotationRpy4(rpyRad) {
  const [roll, pitch, yaw] = finiteVector(
    rpyRad,
    VECTOR_LENGTH,
    "RPY rotation",
  );
  const rotationX = rotationAxisAngle4([1, 0, 0], roll);
  const rotationY = rotationAxisAngle4([0, 1, 0], pitch);
  const rotationZ = rotationAxisAngle4([0, 0, 1], yaw);
  return multiply4(rotationZ, multiply4(rotationY, rotationX));
}

export function transformPoint(matrix, point) {
  const value = finiteMatrix(matrix);
  const [x, y, z] = finiteVector(point, VECTOR_LENGTH, "point");
  const transformed = [
    value[0] * x + value[4] * y + value[8] * z + value[12],
    value[1] * x + value[5] * y + value[9] * z + value[13],
    value[2] * x + value[6] * y + value[10] * z + value[14],
  ];
  if (!transformed.every(Number.isFinite)) {
    throw new TypeError("transformed point must be finite");
  }
  return transformed;
}

export function normalMatrix3(matrix) {
  const value = finiteMatrix(matrix);
  const a00 = value[0];
  const a01 = value[4];
  const a02 = value[8];
  const a10 = value[1];
  const a11 = value[5];
  const a12 = value[9];
  const a20 = value[2];
  const a21 = value[6];
  const a22 = value[10];
  const determinant = (
    a00 * (a11 * a22 - a12 * a21) -
    a01 * (a10 * a22 - a12 * a20) +
    a02 * (a10 * a21 - a11 * a20)
  );
  if (!Number.isFinite(determinant) || Math.abs(determinant) <= Number.EPSILON) {
    throw new TypeError("matrix upper-left 3x3 must be invertible");
  }
  const inverseDeterminant = 1 / determinant;
  return new Float64Array([
    (a11 * a22 - a12 * a21) * inverseDeterminant,
    (a02 * a21 - a01 * a22) * inverseDeterminant,
    (a01 * a12 - a02 * a11) * inverseDeterminant,
    (a12 * a20 - a10 * a22) * inverseDeterminant,
    (a00 * a22 - a02 * a20) * inverseDeterminant,
    (a02 * a10 - a00 * a12) * inverseDeterminant,
    (a10 * a21 - a11 * a20) * inverseDeterminant,
    (a01 * a20 - a00 * a21) * inverseDeterminant,
    (a00 * a11 - a01 * a10) * inverseDeterminant,
  ]);
}

export function boundsCorners(bounds) {
  const value = finiteBounds(bounds);
  const result = [];
  for (const x of [value[0], value[3]]) {
    for (const y of [value[1], value[4]]) {
      for (const z of [value[2], value[5]]) {
        result.push([x, y, z]);
      }
    }
  }
  return result;
}

export function transformBounds(bounds, matrix) {
  const transformed = boundsCorners(bounds).map(
    (corner) => transformPoint(matrix, corner),
  );
  const result = [
    Infinity,
    Infinity,
    Infinity,
    -Infinity,
    -Infinity,
    -Infinity,
  ];
  for (const point of transformed) {
    for (let axis = 0; axis < 3; axis += 1) {
      result[axis] = Math.min(result[axis], point[axis]);
      result[axis + 3] = Math.max(result[axis + 3], point[axis]);
    }
  }
  return result;
}

export function unionBounds(left, right) {
  const a = finiteBounds(left, "left bounds");
  const b = finiteBounds(right, "right bounds");
  return [
    Math.min(a[0], b[0]),
    Math.min(a[1], b[1]),
    Math.min(a[2], b[2]),
    Math.max(a[3], b[3]),
    Math.max(a[4], b[4]),
    Math.max(a[5], b[5]),
  ];
}

export function cameraPreset(name) {
  const directions = {
    front: [1, 0, 0],
    back: [-1, 0, 0],
    left: [0, 1, 0],
    right: [0, -1, 0],
  };
  if (!Object.hasOwn(directions, name)) {
    throw new TypeError(`unknown camera preset ${name}`);
  }
  return {
    direction: [...directions[name]],
    up: [0, 0, 1],
  };
}

function cameraBasis(camera) {
  if (camera === null || typeof camera !== "object") {
    throw new TypeError("camera must be an object");
  }
  const direction = normalize3(camera.direction, "camera direction");
  const upHint = normalize3(camera.up, "camera up");
  const forward = direction.map((component) => -component);
  const right = normalize3(cross3(forward, upHint), "camera right");
  const up = normalize3(cross3(right, forward), "camera corrected up");
  return {direction, right, up};
}

export function computeFitCamera(bounds, fovYRad, aspect) {
  const value = finiteBounds(bounds);
  if (
    !Number.isFinite(fovYRad) ||
    fovYRad <= 0 ||
    fovYRad >= Math.PI
  ) {
    throw new TypeError("camera vertical field of view must be finite");
  }
  if (!Number.isFinite(aspect) || aspect <= 0) {
    throw new TypeError("camera aspect must be finite and positive");
  }
  const target = [
    (value[0] + value[3]) / 2,
    (value[1] + value[4]) / 2,
    (value[2] + value[5]) / 2,
  ];
  const halfExtent = [
    (value[3] - value[0]) / 2,
    (value[4] - value[1]) / 2,
    (value[5] - value[2]) / 2,
  ];
  const radius = Math.max(Math.hypot(...halfExtent), MIN_CAMERA_RADIUS);
  const preset = cameraPreset("front");
  const basis = cameraBasis(preset);
  const tangent = Math.tan(fovYRad / 2);
  let requiredDistance = 0;
  for (const corner of boundsCorners(value)) {
    const delta = corner.map((component, axis) => component - target[axis]);
    const alongDirection = dot3(delta, basis.direction);
    const horizontal = Math.abs(dot3(delta, basis.right));
    const vertical = Math.abs(dot3(delta, basis.up));
    requiredDistance = Math.max(
      requiredDistance,
      alongDirection + horizontal / (tangent * aspect),
      alongDirection + vertical / tangent,
    );
  }
  const distance = Math.max(requiredDistance * 1.05, radius * 2);
  return {
    target,
    direction: preset.direction,
    up: preset.up,
    distance,
    radius,
    fovYRad,
    aspect,
    near: Math.max(0.001, distance - radius * 1.25),
    far: distance + radius * 4,
  };
}

export function projectToNdc(camera, point) {
  const basis = cameraBasis(camera);
  const target = finiteVector(camera.target, VECTOR_LENGTH, "camera target");
  const value = finiteVector(point, VECTOR_LENGTH, "point");
  if (!Number.isFinite(camera.distance) || camera.distance <= 0) {
    throw new TypeError("camera distance must be finite and positive");
  }
  if (
    !Number.isFinite(camera.fovYRad) ||
    camera.fovYRad <= 0 ||
    camera.fovYRad >= Math.PI
  ) {
    throw new TypeError("camera vertical field of view must be finite");
  }
  if (!Number.isFinite(camera.aspect) || camera.aspect <= 0) {
    throw new TypeError("camera aspect must be finite and positive");
  }
  const delta = value.map((component, axis) => component - target[axis]);
  const depth = camera.distance - dot3(delta, basis.direction);
  if (!Number.isFinite(depth) || depth <= 0) {
    throw new TypeError("point must be in front of camera");
  }
  const tangent = Math.tan(camera.fovYRad / 2);
  return [
    dot3(delta, basis.right) / (depth * tangent * camera.aspect),
    dot3(delta, basis.up) / (depth * tangent),
    depth,
  ];
}

export function inverseRigid4(matrix) {
  const value = assertRigid(matrix);
  const result = identity4();
  result[0] = value[0];
  result[1] = value[4];
  result[2] = value[8];
  result[4] = value[1];
  result[5] = value[5];
  result[6] = value[9];
  result[8] = value[2];
  result[9] = value[6];
  result[10] = value[10];

  const x = value[12];
  const y = value[13];
  const z = value[14];
  result[12] = -(result[0] * x + result[4] * y + result[8] * z);
  result[13] = -(result[1] * x + result[5] * y + result[9] * z);
  result[14] = -(result[2] * x + result[6] * y + result[10] * z);
  return result;
}

export function matrixToPoseXyzw(matrix) {
  const value = assertRigid(matrix);
  const m00 = value[0];
  const m01 = value[4];
  const m02 = value[8];
  const m10 = value[1];
  const m11 = value[5];
  const m12 = value[9];
  const m20 = value[2];
  const m21 = value[6];
  const m22 = value[10];

  let x;
  let y;
  let z;
  let w;
  const trace = m00 + m11 + m22;
  if (trace > 0) {
    const scale = 2 * Math.sqrt(trace + 1);
    w = 0.25 * scale;
    x = (m21 - m12) / scale;
    y = (m02 - m20) / scale;
    z = (m10 - m01) / scale;
  } else if (m00 > m11 && m00 > m22) {
    const scale = 2 * Math.sqrt(1 + m00 - m11 - m22);
    w = (m21 - m12) / scale;
    x = 0.25 * scale;
    y = (m01 + m10) / scale;
    z = (m02 + m20) / scale;
  } else if (m11 > m22) {
    const scale = 2 * Math.sqrt(1 + m11 - m00 - m22);
    w = (m02 - m20) / scale;
    x = (m01 + m10) / scale;
    y = 0.25 * scale;
    z = (m12 + m21) / scale;
  } else {
    const scale = 2 * Math.sqrt(1 + m22 - m00 - m11);
    w = (m10 - m01) / scale;
    x = (m02 + m20) / scale;
    y = (m12 + m21) / scale;
    z = 0.25 * scale;
  }

  const norm = Math.hypot(x, y, z, w);
  if (!Number.isFinite(norm) || norm === 0) {
    throw new TypeError("matrix quaternion must be finite and nonzero");
  }
  const quaternion = [x / norm, y / norm, z / norm, w / norm];
  if (quaternion[3] < 0) {
    for (let index = 0; index < quaternion.length; index += 1) {
      quaternion[index] = -quaternion[index];
    }
  }
  return {
    position: [value[12], value[13], value[14]],
    quaternion,
  };
}
