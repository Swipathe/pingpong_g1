import test from "node:test";
import assert from "node:assert/strict";

import {
  boundsCorners,
  cameraPreset,
  computeFitCamera,
  identity4,
  inverseRigid4,
  matrixToPoseXyzw,
  multiply4,
  normalMatrix3,
  projectToNdc,
  rotationAxisAngle4,
  rotationRpy4,
  scale4,
  transformPoint,
  transformBounds,
  translation4,
  unionBounds,
} from "../matrix.js";
import {HitterRenderer} from "../renderer.js";

const EPSILON = 1e-12;

function assertVectorClose(actual, expected, tolerance = EPSILON) {
  assert.equal(actual.length, expected.length);
  for (let index = 0; index < expected.length; index += 1) {
    assert.ok(
      Math.abs(actual[index] - expected[index]) <= tolerance,
      `component ${index}: expected ${expected[index]}, got ${actual[index]}`,
    );
  }
}

test("identity4 returns a Float64 column-major identity matrix", () => {
  const matrix = identity4();
  assert.ok(matrix instanceof Float64Array);
  assert.deepEqual(
    Array.from(matrix),
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
  );
});

test("multiply4 composes non-commuting column-vector transforms", () => {
  const translation = translation4([1, 2, 3]);
  const rotation = rotationAxisAngle4([0, 0, 1], Math.PI / 2);

  assertVectorClose(
    transformPoint(multiply4(translation, rotation), [1, 0, 0]),
    [1, 3, 3],
  );
  assertVectorClose(
    transformPoint(multiply4(rotation, translation), [1, 0, 0]),
    [-2, 2, 3],
  );
});

test("rotationAxisAngle4 rotates 90 degrees about each cardinal axis", () => {
  assertVectorClose(
    transformPoint(rotationAxisAngle4([1, 0, 0], Math.PI / 2), [0, 1, 0]),
    [0, 0, 1],
  );
  assertVectorClose(
    transformPoint(rotationAxisAngle4([0, 1, 0], Math.PI / 2), [0, 0, 1]),
    [1, 0, 0],
  );
  assertVectorClose(
    transformPoint(rotationAxisAngle4([0, 0, 1], Math.PI / 2), [1, 0, 0]),
    [0, 1, 0],
  );
});

test("rotationAxisAngle4 normalizes an arbitrary finite axis", () => {
  assertVectorClose(
    transformPoint(
      rotationAxisAngle4([2, 2, 2], (2 * Math.PI) / 3),
      [1, 0, 0],
    ),
    [0, 1, 0],
  );
});

test("rotationRpy4 applies Rx then Ry then Rz to column vectors", () => {
  const matrix = rotationRpy4([Math.PI / 2, Math.PI / 2, 0]);
  assertVectorClose(transformPoint(matrix, [0, 1, 0]), [1, 0, 0]);
});

test("inverseRigid4 inverts rotation and translation", () => {
  const matrix = multiply4(
    translation4([0.5, -1.25, 2]),
    rotationRpy4([0.2, -0.3, 0.4]),
  );
  const inverse = inverseRigid4(matrix);
  assertVectorClose(
    Array.from(multiply4(matrix, inverse)),
    Array.from(identity4()),
    1e-11,
  );
  assertVectorClose(
    transformPoint(inverse, transformPoint(matrix, [1, 2, -3])),
    [1, 2, -3],
    1e-11,
  );
});

test("matrixToPoseXyzw returns normalized xyzw with canonical nonnegative w", () => {
  const matrix = multiply4(
    translation4([1, -2, 3]),
    rotationAxisAngle4([0, 0, 1], (3 * Math.PI) / 2),
  );
  const pose = matrixToPoseXyzw(matrix);
  assertVectorClose(pose.position, [1, -2, 3]);
  assertVectorClose(
    pose.quaternion,
    [0, 0, -Math.SQRT1_2, Math.SQRT1_2],
    1e-12,
  );
  assert.ok(pose.quaternion[3] >= 0);
  assert.ok(
    Math.abs(
      Math.hypot(
        pose.quaternion[0],
        pose.quaternion[1],
        pose.quaternion[2],
        pose.quaternion[3],
      ) - 1,
    ) <= EPSILON,
  );
});

test("matrixToPoseXyzw canonicalizes equivalent q and negative-q rotations", () => {
  const first = matrixToPoseXyzw(
    rotationAxisAngle4([0, 0, 1], (3 * Math.PI) / 2),
  ).quaternion;
  const second = matrixToPoseXyzw(
    rotationAxisAngle4([0, 0, -1], Math.PI / 2),
  ).quaternion;
  assertVectorClose(first, second);
});

test("matrix primitives reject malformed or nonfinite inputs", () => {
  assert.throws(() => translation4([1, 2]), /length 3/);
  assert.throws(() => rotationRpy4([0, Infinity, 0]), /finite/);
  assert.throws(() => rotationAxisAngle4([0, 0, 0], 1), /nonzero/);
  assert.throws(() => rotationAxisAngle4([0, 0, 1], NaN), /finite/);
  assert.throws(() => multiply4(identity4(), [1, 2]), /length 16/);
  assert.throws(() => transformPoint(identity4(), [0, NaN, 0]), /finite/);
  const nonRigid = identity4();
  nonRigid[15] = 2;
  assert.throws(() => inverseRigid4(nonRigid), /rigid/);
});

test("two transformed unit boxes union to hand-derived bounds", () => {
  const unitBounds = [0, 0, 0, 1, 1, 1];
  const first = transformBounds(
    unitBounds,
    multiply4(translation4([1, 2, 3]), scale4([2, 3, 4])),
  );
  const second = transformBounds(
    unitBounds,
    multiply4(
      translation4([-4, 0, -2]),
      rotationAxisAngle4([0, 0, 1], Math.PI / 2),
    ),
  );

  assertVectorClose(first, [1, 2, 3, 3, 5, 7]);
  assertVectorClose(second, [-5, 0, -2, -4, 1, -1]);
  assertVectorClose(unionBounds(first, second), [-5, 0, -2, 3, 5, 7]);
});

test("inverse-transpose normal matrix preserves scaled tangent orthogonality", () => {
  const model = scale4([2, 3, 4]);
  const normal = normalMatrix3(model);
  assertVectorClose(
    Array.from(normal),
    [0.5, 0, 0, 0, 1 / 3, 0, 0, 0, 0.25],
  );

  const scaledTangent = [2, -3, 0];
  const scaledNormal = [
    normal[0] + normal[3],
    normal[1] + normal[4],
    normal[2] + normal[5],
  ];
  const dot = scaledTangent.reduce(
    (sum, value, index) => sum + value * scaledNormal[index],
    0,
  );
  assert.ok(Math.abs(dot) <= EPSILON);
});

test("camera presets use the robot_base_default axes", () => {
  assert.deepEqual(cameraPreset("front").direction, [1, 0, 0]);
  assert.deepEqual(cameraPreset("back").direction, [-1, 0, 0]);
  assert.deepEqual(cameraPreset("left").direction, [0, 1, 0]);
  assert.deepEqual(cameraPreset("right").direction, [0, -1, 0]);
  assert.deepEqual(cameraPreset("front").up, [0, 0, 1]);
});

test("fit camera contains every transformed bound corner", () => {
  const camera = computeFitCamera(
    [-1, -2, -3, 4, 5, 6],
    Math.PI / 4,
    16 / 9,
  );
  assert.ok(Number.isFinite(camera.distance) && camera.distance > 0);
  for (const corner of boundsCorners([-1, -2, -3, 4, 5, 6])) {
    const ndc = projectToNdc(camera, corner);
    assert.ok(Math.abs(ndc[0]) <= 1 && Math.abs(ndc[1]) <= 1);
  }
});

test("fit camera gives a zero-extent bound a finite minimum radius", () => {
  const camera = computeFitCamera(
    [2, 2, 2, 2, 2, 2],
    Math.PI / 4,
    1,
  );
  assert.ok(Number.isFinite(camera.radius) && camera.radius > 0);
  assert.ok(Number.isFinite(camera.distance) && camera.distance > 0);
});

test("bounds helpers reject every NaN and infinite bound component", () => {
  for (const invalid of [NaN, Infinity, -Infinity]) {
    for (let index = 0; index < 6; index += 1) {
      const bounds = [-1, -2, -3, 4, 5, 6];
      bounds[index] = invalid;
      assert.throws(() => boundsCorners(bounds), /finite/);
      assert.throws(
        () => computeFitCamera(bounds, Math.PI / 4, 1),
        /finite/,
      );
    }
  }
});

class FakeCanvas {
  constructor(context) {
    this.context = context;
    this.clientWidth = 400;
    this.clientHeight = 300;
    this.width = 0;
    this.height = 0;
    this.listeners = new Map();
  }

  getContext(name) {
    assert.equal(name, "webgl");
    return this.context;
  }

  addEventListener(name, listener) {
    const listeners = this.listeners.get(name) ?? new Set();
    listeners.add(listener);
    this.listeners.set(name, listeners);
  }

  removeEventListener(name, listener) {
    this.listeners.get(name)?.delete(listener);
  }

  setPointerCapture() {}

  getBoundingClientRect() {
    return {left: 0, top: 0, width: this.clientWidth, height: this.clientHeight};
  }

  activeListenerCount() {
    return Array.from(this.listeners.values()).reduce(
      (sum, listeners) => sum + listeners.size,
      0,
    );
  }
}

class FakeWebGl {
  constructor() {
    Object.assign(this, {
      ARRAY_BUFFER: 0x8892,
      STATIC_DRAW: 0x88e4,
      FLOAT: 0x1406,
      TRIANGLES: 0x0004,
      LINES: 0x0001,
      VERTEX_SHADER: 0x8b31,
      FRAGMENT_SHADER: 0x8b30,
      COMPILE_STATUS: 0x8b81,
      LINK_STATUS: 0x8b82,
      COLOR_BUFFER_BIT: 0x4000,
      DEPTH_BUFFER_BIT: 0x0100,
      DEPTH_TEST: 0x0b71,
      BLEND: 0x0be2,
      LEQUAL: 0x0203,
      SRC_ALPHA: 0x0302,
      ONE_MINUS_SRC_ALPHA: 0x0303,
    });
    this.created = {buffers: [], shaders: [], programs: []};
    this.deleted = {buffers: [], shaders: [], programs: []};
    this.bufferUsages = [];
    this.boundBuffers = [];
    this.drawCalls = [];
    this.depthWrites = [];
    this.nextId = 1;
  }

  makeResource(kind) {
    const resource = {kind, id: this.nextId};
    this.nextId += 1;
    this.created[kind].push(resource);
    return resource;
  }

  createBuffer() { return this.makeResource("buffers"); }
  deleteBuffer(resource) { this.deleted.buffers.push(resource); }
  createShader() { return this.makeResource("shaders"); }
  deleteShader(resource) { this.deleted.shaders.push(resource); }
  createProgram() { return this.makeResource("programs"); }
  deleteProgram(resource) { this.deleted.programs.push(resource); }
  shaderSource() {}
  compileShader() {}
  getShaderParameter() { return true; }
  getShaderInfoLog() { return ""; }
  attachShader() {}
  linkProgram() {}
  getProgramParameter() { return true; }
  getProgramInfoLog() { return ""; }
  getAttribLocation(_program, name) {
    return name === "a_position" ? 0 : 1;
  }
  getUniformLocation(_program, name) { return {name}; }
  bindBuffer(_target, buffer) {
    if (buffer && this.deleted.buffers.includes(buffer)) {
      throw new Error("attempted to bind a deleted buffer");
    }
    this.boundBuffers.push(buffer);
  }
  bufferData(_target, _data, usage) { this.bufferUsages.push(usage); }
  viewport() {}
  clearColor() {}
  clear() {}
  enable() {}
  disable() {}
  depthFunc() {}
  blendFunc() {}
  depthMask(value) { this.depthWrites.push(value); }
  useProgram() {}
  uniformMatrix4fv() {}
  uniformMatrix3fv() {}
  uniform4fv() {}
  enableVertexAttribArray() {}
  vertexAttribPointer() {}
  drawArrays(mode, _first, count) { this.drawCalls.push({mode, count}); }
}

class PartialBufferFailureWebGl extends FakeWebGl {
  constructor() {
    super();
    this.bufferAttempts = 0;
  }

  createBuffer() {
    this.bufferAttempts += 1;
    if (this.bufferAttempts === 2) {
      return null;
    }
    return super.createBuffer();
  }
}

class ScheduledBufferFailureWebGl extends FakeWebGl {
  constructor() {
    super();
    this.bufferAttempts = 0;
    this.failBufferAt = null;
  }

  createBuffer() {
    this.bufferAttempts += 1;
    if (this.bufferAttempts === this.failBufferAt) {
      return null;
    }
    return super.createBuffer();
  }

  failSecondBufferOfNextGeometry() {
    this.failBufferAt = this.bufferAttempts + 2;
  }
}

function asciiTriangleStl() {
  const text = [
    "solid fixture",
    "facet normal 0 0 1",
    "outer loop",
    "vertex 0 0 0",
    "vertex 1 0 0",
    "vertex 0 1 0",
    "endloop",
    "endfacet",
    "endsolid fixture",
  ].join("\n");
  return new TextEncoder().encode(text).buffer;
}

function rendererManifest(meshId = "shared.stl") {
  return {
    limits: {
      maxRequestBodyBytes: 262144,
      maxMeshBytes: 67108864,
      maxStlTriangles: 1000000,
    },
    frame: {groundZRobotBaseM: -0.793},
    assetHashes: {displayMeshSetSha256: "display-mesh-set"},
    visuals: [
      {
        linkName: "pelvis",
        originXyzM: [0.25, 0, 0],
        originRpyRad: [0, 0, 0],
        meshScaleXyz: [2, 3, 4],
        meshId,
        colorRgba: [0.8, 0.8, 0.8, 1],
      },
      {
        linkName: "right_racket_link",
        originXyzM: [0, 0, 0],
        originRpyRad: [0, 0, 0],
        meshScaleXyz: [1, 1, 1],
        meshId,
        colorRgba: [0.9, 0.1, 0.1, 1],
      },
    ],
    tableVisuals: [
      {
        name: "table",
        frame: "robot_base_default",
        centerXyzM: [1.2, 0, -0.1],
        quaternionXyzw: [0, 0, 0, 1],
        halfSizeXyzM: [1, 0.75, 0.05],
        colorRgba: [0.1, 0.3, 0.5, 1],
      },
    ],
  };
}

function bufferDeleteCount(gl, buffer) {
  return gl.deleted.buffers.filter((deleted) => deleted === buffer).length;
}

test("missing WebGL reports status and leaves renderer methods usable", () => {
  const statuses = [];
  const canvas = new FakeCanvas(null);
  const renderer = new HitterRenderer(
    canvas,
    (message) => statuses.push(message),
  );

  assert.deepEqual(statuses, ["当前浏览器无法创建 WebGL 上下文"]);
  assert.doesNotThrow(() => renderer.setPose({pelvis: identity4()}));
  renderer.dispose();
  assert.equal(canvas.activeListenerCount(), 0);
});

test("renderer construction failure deletes every partially created GPU resource", () => {
  const gl = new PartialBufferFailureWebGl();
  const canvas = new FakeCanvas(gl);

  assert.throws(
    () => new HitterRenderer(canvas, () => {}),
    /buffer creation failed/,
  );
  assert.equal(gl.deleted.buffers.length, gl.created.buffers.length);
  assert.equal(gl.deleted.shaders.length, gl.created.shaders.length);
  assert.equal(gl.deleted.programs.length, gl.created.programs.length);
  assert.equal(canvas.activeListenerCount(), 0);
});

test("dispose during mesh load prevents late GPU allocation", async () => {
  const gl = new FakeWebGl();
  const canvas = new FakeCanvas(gl);
  const renderer = new HitterRenderer(canvas, () => {});
  let resolveMesh;
  const meshResult = new Promise((resolve) => {
    resolveMesh = resolve;
  });
  const loading = renderer.load(
    rendererManifest(),
    async () => meshResult,
  );

  renderer.dispose();
  resolveMesh(asciiTriangleStl());
  await loading;

  assert.equal(gl.deleted.buffers.length, gl.created.buffers.length);
  assert.equal(gl.deleted.shaders.length, gl.created.shaders.length);
  assert.equal(gl.deleted.programs.length, gl.created.programs.length);
  assert.equal(canvas.activeListenerCount(), 0);
});

test("rejected mesh reload preserves the committed renderable scene", async () => {
  const gl = new FakeWebGl();
  const canvas = new FakeCanvas(gl);
  const renderer = new HitterRenderer(canvas, () => {});
  await renderer.load(rendererManifest("committed.stl"), async () => (
    asciiTriangleStl()
  ));
  renderer.setPose({
    pelvis: identity4(),
    right_racket_link: translation4([0, 1, 0]),
  });
  const committedBuffers = gl.created.buffers.slice(-2);

  await assert.rejects(
    renderer.load(rendererManifest("rejected.stl"), async () => {
      throw new Error("fixture loader rejection");
    }),
    /fixture loader rejection/,
  );

  assert.deepEqual(
    committedBuffers.map((buffer) => bufferDeleteCount(gl, buffer)),
    [0, 0],
  );
  const drawsBefore = gl.drawCalls.length;
  assert.doesNotThrow(() => renderer.render());
  assert.equal(gl.drawCalls.length - drawsBefore, 4);

  renderer.dispose();
  assert.equal(gl.deleted.buffers.length, gl.created.buffers.length);
});

test("reload allocation failure cleans temporary buffers and preserves old resources", async () => {
  const gl = new ScheduledBufferFailureWebGl();
  const canvas = new FakeCanvas(gl);
  const renderer = new HitterRenderer(canvas, () => {});
  await renderer.load(rendererManifest("committed.stl"), async () => (
    asciiTriangleStl()
  ));
  renderer.setPose({
    pelvis: identity4(),
    right_racket_link: translation4([0, 1, 0]),
  });
  const committedBuffers = gl.created.buffers.slice(-2);
  const createdBeforeReload = gl.created.buffers.length;
  gl.failSecondBufferOfNextGeometry();

  await assert.rejects(
    renderer.load(rendererManifest("replacement.stl"), async () => (
      asciiTriangleStl()
    )),
    /buffer creation failed/,
  );

  const temporaryBuffers = gl.created.buffers.slice(createdBeforeReload);
  assert.equal(temporaryBuffers.length, 1);
  assert.deepEqual(
    temporaryBuffers.map((buffer) => bufferDeleteCount(gl, buffer)),
    [1],
  );
  assert.deepEqual(
    committedBuffers.map((buffer) => bufferDeleteCount(gl, buffer)),
    [0, 0],
  );
  assert.doesNotThrow(() => renderer.render());

  renderer.dispose();
  assert.ok(
    gl.created.buffers.every(
      (buffer) => bufferDeleteCount(gl, buffer) === 1,
    ),
  );
});

test("successful retry atomically swaps meshes and deletes old buffers once", async () => {
  const gl = new FakeWebGl();
  const canvas = new FakeCanvas(gl);
  const renderer = new HitterRenderer(canvas, () => {});
  await renderer.load(rendererManifest("committed.stl"), async () => (
    asciiTriangleStl()
  ));
  renderer.setPose({
    pelvis: identity4(),
    right_racket_link: translation4([0, 1, 0]),
  });
  const committedBuffers = gl.created.buffers.slice(-2);
  await assert.rejects(
    renderer.load(rendererManifest("rejected.stl"), async () => {
      throw new Error("fixture loader rejection");
    }),
    /fixture loader rejection/,
  );

  let resolveReplacement;
  const replacementResult = new Promise((resolve) => {
    resolveReplacement = resolve;
  });
  const createdBeforeRetry = gl.created.buffers.length;
  const retry = renderer.load(
    rendererManifest("replacement.stl"),
    async () => replacementResult,
  );

  assert.deepEqual(
    committedBuffers.map((buffer) => bufferDeleteCount(gl, buffer)),
    [0, 0],
  );
  assert.doesNotThrow(() => renderer.render());
  resolveReplacement(asciiTriangleStl());
  await retry;

  const replacementBuffers = gl.created.buffers.slice(createdBeforeRetry);
  assert.equal(replacementBuffers.length, 2);
  assert.deepEqual(
    committedBuffers.map((buffer) => bufferDeleteCount(gl, buffer)),
    [1, 1],
  );
  assert.deepEqual(
    replacementBuffers.map((buffer) => bufferDeleteCount(gl, buffer)),
    [0, 0],
  );
  assert.doesNotThrow(() => renderer.render());

  renderer.dispose();
  assert.ok(
    gl.created.buffers.every(
      (buffer) => bufferDeleteCount(gl, buffer) === 1,
    ),
  );
});

test("renderer reuses immutable meshes and repeated dispose releases lifecycle resources", async () => {
  const originalRatio = Object.getOwnPropertyDescriptor(
    globalThis,
    "devicePixelRatio",
  );
  const originalRequest = globalThis.requestAnimationFrame;
  const originalCancel = globalThis.cancelAnimationFrame;
  const animationFrames = new Map();
  let nextAnimationFrame = 1;
  Object.defineProperty(globalThis, "devicePixelRatio", {
    configurable: true,
    value: 3,
  });
  globalThis.requestAnimationFrame = (callback) => {
    const id = nextAnimationFrame;
    nextAnimationFrame += 1;
    animationFrames.set(id, callback);
    return id;
  };
  globalThis.cancelAnimationFrame = (id) => animationFrames.delete(id);

  try {
    for (let cycle = 0; cycle < 3; cycle += 1) {
      const gl = new FakeWebGl();
      const canvas = new FakeCanvas(gl);
      const renderer = new HitterRenderer(canvas, () => {});
      const loads = [];
      const buffersBeforeLoad = gl.created.buffers.length;
      await renderer.load(rendererManifest(), async (digest, meshId) => {
        loads.push([digest, meshId]);
        return asciiTriangleStl();
      });

      assert.deepEqual(loads, [["display-mesh-set", "shared.stl"]]);
      assert.equal(gl.created.buffers.length - buffersBeforeLoad, 2);
      assert.ok(gl.bufferUsages.every((usage) => usage === gl.STATIC_DRAW));

      const pose = {
        pelvis: identity4(),
        right_racket_link: translation4([0, 1, 0]),
      };
      renderer.setPose(pose);
      renderer.setGhostPose(pose);
      renderer.setTableVisible(true);
      renderer.setCameraPreset("right");
      renderer.fitToRobot();
      renderer.render();

      assert.equal(canvas.width, 800);
      assert.equal(canvas.height, 600);
      assert.ok(gl.drawCalls.length >= 6);
      assert.ok(gl.depthWrites.includes(false));
      assert.equal(gl.depthWrites.at(-1), true);

      renderer.dispose();
      assert.equal(canvas.activeListenerCount(), 0);
      assert.equal(gl.deleted.buffers.length, gl.created.buffers.length);
      assert.equal(gl.deleted.shaders.length, gl.created.shaders.length);
      assert.equal(gl.deleted.programs.length, gl.created.programs.length);
    }
    assert.equal(animationFrames.size, 0);
  } finally {
    if (originalRatio) {
      Object.defineProperty(globalThis, "devicePixelRatio", originalRatio);
    } else {
      delete globalThis.devicePixelRatio;
    }
    globalThis.requestAnimationFrame = originalRequest;
    globalThis.cancelAnimationFrame = originalCancel;
  }
});
