import {
  cameraPreset,
  computeFitCamera,
  identity4,
  multiply4,
  normalMatrix3,
  rotationRpy4,
  scale4,
  transformBounds,
  translation4,
  unionBounds,
} from "./matrix.js";
import {parseStl} from "./stl.js";

const FIELD_OF_VIEW_Y_RAD = Math.PI / 4;
const GRID_HALF_SIZE_M = 5;
const GRID_STEP_M = 0.5;
const GHOST_ALPHA = 0.24;

const VERTEX_SHADER_SOURCE = `
attribute vec3 a_position;
attribute vec3 a_normal;
uniform mat4 u_view_projection;
uniform mat4 u_model;
uniform mat3 u_normal;
varying vec3 v_normal;

void main() {
  gl_Position = u_view_projection * u_model * vec4(a_position, 1.0);
  v_normal = normalize(u_normal * a_normal);
}
`;

const FRAGMENT_SHADER_SOURCE = `
precision mediump float;
uniform vec4 u_color;
varying vec3 v_normal;

void main() {
  vec3 light_direction = normalize(vec3(0.35, -0.45, 0.82));
  float diffuse = max(dot(normalize(v_normal), light_direction), 0.0);
  float lighting = 0.32 + 0.68 * diffuse;
  gl_FragColor = vec4(u_color.rgb * lighting, u_color.a);
}
`;

function finiteArray(value, length, name) {
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
    throw new TypeError(`${name} must be finite`);
  }
  return result;
}

function finiteMatrix(value, name) {
  return new Float64Array(finiteArray(value, 16, name));
}

function cloneLinkMatrices(linkWorldMatrices, name) {
  if (
    linkWorldMatrices === null ||
    typeof linkWorldMatrices !== "object" ||
    Array.isArray(linkWorldMatrices)
  ) {
    throw new TypeError(`${name} must be an object`);
  }
  const result = {};
  for (const [linkName, matrix] of Object.entries(linkWorldMatrices)) {
    result[linkName] = finiteMatrix(matrix, `${name}.${linkName}`);
  }
  return result;
}

function subtract3(left, right) {
  return [
    left[0] - right[0],
    left[1] - right[1],
    left[2] - right[2],
  ];
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

function normalize3(value, name) {
  const vector = finiteArray(value, 3, name);
  const length = Math.hypot(...vector);
  if (!Number.isFinite(length) || length === 0) {
    throw new TypeError(`${name} must be nonzero`);
  }
  return vector.map((component) => component / length);
}

function quaternionXyzw4(quaternion) {
  const [rawX, rawY, rawZ, rawW] = finiteArray(
    quaternion,
    4,
    "quaternion",
  );
  const length = Math.hypot(rawX, rawY, rawZ, rawW);
  if (!Number.isFinite(length) || length === 0) {
    throw new TypeError("quaternion must be nonzero");
  }
  const x = rawX / length;
  const y = rawY / length;
  const z = rawZ / length;
  const w = rawW / length;
  const xx = x * x;
  const yy = y * y;
  const zz = z * z;
  const xy = x * y;
  const xz = x * z;
  const yz = y * z;
  const wx = w * x;
  const wy = w * y;
  const wz = w * z;
  return new Float64Array([
    1 - 2 * (yy + zz),
    2 * (xy + wz),
    2 * (xz - wy),
    0,
    2 * (xy - wz),
    1 - 2 * (xx + zz),
    2 * (yz + wx),
    0,
    2 * (xz + wy),
    2 * (yz - wx),
    1 - 2 * (xx + yy),
    0,
    0,
    0,
    0,
    1,
  ]);
}

function perspective4(fovYRad, aspect, near, far) {
  const tangent = Math.tan(fovYRad / 2);
  const range = near - far;
  return new Float64Array([
    1 / (aspect * tangent),
    0,
    0,
    0,
    0,
    1 / tangent,
    0,
    0,
    0,
    0,
    (far + near) / range,
    -1,
    0,
    0,
    (2 * far * near) / range,
    0,
  ]);
}

function lookAtCamera4(camera) {
  const direction = normalize3(camera.direction, "camera direction");
  const eye = camera.target.map(
    (component, axis) => component + direction[axis] * camera.distance,
  );
  const forward = normalize3(
    subtract3(camera.target, eye),
    "camera forward",
  );
  const side = normalize3(
    cross3(forward, camera.up),
    "camera side",
  );
  const up = cross3(side, forward);
  return new Float64Array([
    side[0],
    up[0],
    -forward[0],
    0,
    side[1],
    up[1],
    -forward[1],
    0,
    side[2],
    up[2],
    -forward[2],
    0,
    -dot3(side, eye),
    -dot3(up, eye),
    dot3(forward, eye),
    1,
  ]);
}

function visualLocalMatrix(visual) {
  return multiply4(
    translation4(visual.originXyzM),
    multiply4(
      rotationRpy4(visual.originRpyRad),
      scale4(visual.meshScaleXyz),
    ),
  );
}

function tableModelMatrix(table) {
  return multiply4(
    translation4(table.centerXyzM),
    multiply4(
      quaternionXyzw4(table.quaternionXyzw),
      scale4(table.halfSizeXyzM.map((value) => value * 2)),
    ),
  );
}

function boxGeometry() {
  const positions = [];
  const normals = [];
  const faces = [
    {normal: [1, 0, 0], corners: [[0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [0.5, 0.5, 0.5], [0.5, -0.5, 0.5]]},
    {normal: [-1, 0, 0], corners: [[-0.5, 0.5, -0.5], [-0.5, -0.5, -0.5], [-0.5, -0.5, 0.5], [-0.5, 0.5, 0.5]]},
    {normal: [0, 1, 0], corners: [[0.5, 0.5, -0.5], [-0.5, 0.5, -0.5], [-0.5, 0.5, 0.5], [0.5, 0.5, 0.5]]},
    {normal: [0, -1, 0], corners: [[-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5]]},
    {normal: [0, 0, 1], corners: [[-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5]]},
    {normal: [0, 0, -1], corners: [[-0.5, 0.5, -0.5], [0.5, 0.5, -0.5], [0.5, -0.5, -0.5], [-0.5, -0.5, -0.5]]},
  ];
  for (const face of faces) {
    for (const index of [0, 1, 2, 0, 2, 3]) {
      positions.push(...face.corners[index]);
      normals.push(...face.normal);
    }
  }
  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
  };
}

function gridGeometry() {
  const positions = [];
  for (
    let coordinate = -GRID_HALF_SIZE_M;
    coordinate <= GRID_HALF_SIZE_M + Number.EPSILON;
    coordinate += GRID_STEP_M
  ) {
    positions.push(
      -GRID_HALF_SIZE_M,
      coordinate,
      0,
      GRID_HALF_SIZE_M,
      coordinate,
      0,
      coordinate,
      -GRID_HALF_SIZE_M,
      0,
      coordinate,
      GRID_HALF_SIZE_M,
      0,
    );
  }
  const normals = new Float32Array(positions.length);
  for (let index = 2; index < normals.length; index += 3) {
    normals[index] = 1;
  }
  return {
    positions: new Float32Array(positions),
    normals,
  };
}

export class HitterRenderer {
  constructor(canvas, statusCallback) {
    if (
      canvas === null ||
      typeof canvas !== "object" ||
      typeof canvas.getContext !== "function" ||
      typeof canvas.addEventListener !== "function"
    ) {
      throw new TypeError("canvas must provide the canvas DOM interface");
    }
    this.canvas = canvas;
    this.statusCallback = (
      typeof statusCallback === "function" ? statusCallback : () => {}
    );
    this.gl = canvas.getContext("webgl");
    this.disposed = false;
    this.listeners = [];
    this.animationFrame = null;
    this.loadGeneration = 0;
    this.buffers = new Set();
    this.shaders = new Set();
    this.programs = new Set();
    this.meshResources = new Map();
    this.visuals = [];
    this.tableVisuals = [];
    this.linkWorldMatrices = {};
    this.ghostLinkWorldMatrices = null;
    this.robotBounds = null;
    this.tableVisible = true;
    this.drag = null;
    this.manifest = null;
    this.camera = {
      target: [0, 0, 0],
      direction: [1, 0, 0],
      up: [0, 0, 1],
      distance: 4,
      radius: 1,
      fovYRad: FIELD_OF_VIEW_Y_RAD,
      aspect: 1,
      near: 0.01,
      far: 100,
    };

    if (!this.gl) {
      this.statusCallback("当前浏览器无法创建 WebGL 上下文");
      return;
    }

    try {
      this.initializeProgram();
      this.boxResource = this.uploadGeometry(boxGeometry());
      this.gridResource = this.uploadGeometry(gridGeometry());
      this.registerInteractions();
      this.requestRender();
    } catch (error) {
      this.dispose();
      throw error;
    }
  }

  initializeProgram() {
    const gl = this.gl;
    const vertexShader = this.compileShader(
      gl.VERTEX_SHADER,
      VERTEX_SHADER_SOURCE,
    );
    const fragmentShader = this.compileShader(
      gl.FRAGMENT_SHADER,
      FRAGMENT_SHADER_SOURCE,
    );
    const program = gl.createProgram();
    if (!program) {
      throw new Error("WebGL program creation failed");
    }
    this.programs.add(program);
    gl.attachShader(program, vertexShader);
    gl.attachShader(program, fragmentShader);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(
        `WebGL program link failed: ${gl.getProgramInfoLog(program)}`,
      );
    }

    const attribute = (name) => {
      const location = gl.getAttribLocation(program, name);
      if (location < 0) {
        throw new Error(`WebGL attribute lookup failed: ${name}`);
      }
      return location;
    };
    const uniform = (name) => {
      const location = gl.getUniformLocation(program, name);
      if (location === null) {
        throw new Error(`WebGL uniform lookup failed: ${name}`);
      }
      return location;
    };
    this.program = program;
    this.locations = {
      position: attribute("a_position"),
      normal: attribute("a_normal"),
      viewProjection: uniform("u_view_projection"),
      model: uniform("u_model"),
      normalMatrix: uniform("u_normal"),
      color: uniform("u_color"),
    };
  }

  compileShader(type, source) {
    const gl = this.gl;
    const shader = gl.createShader(type);
    if (!shader) {
      throw new Error("WebGL shader creation failed");
    }
    this.shaders.add(shader);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      throw new Error(
        `WebGL shader compile failed: ${gl.getShaderInfoLog(shader)}`,
      );
    }
    return shader;
  }

  uploadGeometry(geometry, transactionBuffers = null) {
    const gl = this.gl;
    const positionBuffer = gl.createBuffer();
    if (!positionBuffer) {
      throw new Error("WebGL buffer creation failed");
    }
    this.buffers.add(positionBuffer);
    transactionBuffers?.add(positionBuffer);
    const normalBuffer = gl.createBuffer();
    if (!normalBuffer) {
      throw new Error("WebGL buffer creation failed");
    }
    this.buffers.add(normalBuffer);
    transactionBuffers?.add(normalBuffer);
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, geometry.positions, gl.STATIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, normalBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, geometry.normals, gl.STATIC_DRAW);
    return {
      positionBuffer,
      normalBuffer,
      count: geometry.positions.length / 3,
    };
  }

  releaseBuffers(buffers) {
    if (!this.gl) {
      return;
    }
    for (const buffer of buffers) {
      if (this.buffers.has(buffer)) {
        this.gl.deleteBuffer(buffer);
        this.buffers.delete(buffer);
      }
    }
  }

  releaseMeshResources(meshResources) {
    const buffers = new Set();
    for (const mesh of meshResources.values()) {
      buffers.add(mesh.positionBuffer);
      buffers.add(mesh.normalBuffer);
    }
    this.releaseBuffers(buffers);
    meshResources.clear();
  }

  async load(manifest, meshLoader) {
    if (this.disposed || !this.gl) {
      return;
    }
    if (
      manifest === null ||
      typeof manifest !== "object" ||
      !Array.isArray(manifest.visuals) ||
      !Array.isArray(manifest.tableVisuals) ||
      typeof meshLoader !== "function"
    ) {
      throw new TypeError("manifest and mesh loader are required");
    }
    const digest = manifest.assetHashes?.displayMeshSetSha256;
    if (typeof digest !== "string" || digest.length === 0) {
      throw new TypeError("display mesh set digest is required");
    }
    const groundZ = manifest.frame?.groundZRobotBaseM;
    if (!Number.isFinite(groundZ)) {
      throw new TypeError("manifest ground height must be finite");
    }

    this.loadGeneration += 1;
    const loadGeneration = this.loadGeneration;
    const visualSpecs = manifest.visuals.map((visual) => {
      if (typeof visual.meshId !== "string" || visual.meshId.length === 0) {
        throw new TypeError("visual mesh ID must be nonempty");
      }
      return {
        linkName: visual.linkName,
        color: finiteArray(visual.colorRgba, 4, "visual color"),
        localMatrix: visualLocalMatrix(visual),
        meshId: visual.meshId,
      };
    });
    const tableVisuals = manifest.tableVisuals.map((table) => {
      if (table.frame !== "robot_base_default") {
        throw new TypeError("table visual frame must be robot_base_default");
      }
      return {
        name: table.name,
        color: finiteArray(table.colorRgba, 4, "table color"),
        modelMatrix: tableModelMatrix(table),
      };
    });
    const gridModelMatrix = translation4([0, 0, groundZ]);
    const temporaryMeshes = new Map();
    const temporaryBuffers = new Set();
    let committed = false;
    try {
      const uniqueMeshIds = [...new Set(
        visualSpecs.map((visual) => visual.meshId),
      )];
      for (const meshId of uniqueMeshIds) {
        const arrayBuffer = await meshLoader(digest, meshId);
        if (this.disposed || loadGeneration !== this.loadGeneration) {
          return;
        }
        const mesh = parseStl(arrayBuffer, manifest.limits);
        temporaryMeshes.set(meshId, {
          ...this.uploadGeometry(mesh, temporaryBuffers),
          bounds: [
            ...mesh.bounds.min,
            ...mesh.bounds.max,
          ],
        });
      }

      if (this.disposed || loadGeneration !== this.loadGeneration) {
        return;
      }
      const visuals = visualSpecs.map((visual) => {
        const mesh = temporaryMeshes.get(visual.meshId);
        if (!mesh) {
          throw new Error(`missing uploaded mesh ${visual.meshId}`);
        }
        return {
          linkName: visual.linkName,
          color: visual.color,
          localMatrix: visual.localMatrix,
          mesh,
        };
      });
      const previousMeshes = this.meshResources;
      this.manifest = manifest;
      this.meshResources = temporaryMeshes;
      this.visuals = visuals;
      this.tableVisuals = tableVisuals;
      this.gridModelMatrix = gridModelMatrix;
      committed = true;
      temporaryBuffers.clear();
      this.releaseMeshResources(previousMeshes);
      this.updateRobotBounds();
      if (this.robotBounds) {
        this.fitToRobot();
      }
      this.requestRender();
    } finally {
      if (!committed) {
        this.releaseBuffers(temporaryBuffers);
      }
    }
  }

  setPose(linkWorldMatrices) {
    if (this.disposed) {
      return;
    }
    this.linkWorldMatrices = cloneLinkMatrices(
      linkWorldMatrices,
      "link world matrices",
    );
    this.updateRobotBounds();
    this.requestRender();
  }

  setGhostPose(linkWorldMatricesOrNull) {
    if (this.disposed) {
      return;
    }
    this.ghostLinkWorldMatrices = (
      linkWorldMatricesOrNull === null
        ? null
        : cloneLinkMatrices(
          linkWorldMatricesOrNull,
          "ghost link world matrices",
        )
    );
    this.requestRender();
  }

  setTableVisible(visible) {
    if (typeof visible !== "boolean") {
      throw new TypeError("table visibility must be boolean");
    }
    if (this.disposed) {
      return;
    }
    this.tableVisible = visible;
    this.requestRender();
  }

  setCameraPreset(name) {
    if (this.disposed) {
      return;
    }
    const preset = cameraPreset(name);
    this.camera.direction = preset.direction;
    this.camera.up = preset.up;
    this.requestRender();
  }

  fitToRobot() {
    if (this.disposed || !this.robotBounds) {
      return;
    }
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 1);
    const height = Math.max(
      1,
      this.canvas.clientHeight || this.canvas.height || 1,
    );
    const fit = computeFitCamera(
      this.robotBounds,
      FIELD_OF_VIEW_Y_RAD,
      width / height,
    );
    fit.direction = [...this.camera.direction];
    fit.up = [...this.camera.up];
    const limitingTangent = Math.tan(FIELD_OF_VIEW_Y_RAD / 2) * Math.min(
      1,
      fit.aspect,
    );
    fit.distance = Math.max(
      fit.distance,
      fit.radius * (1 + 1 / limitingTangent),
    );
    fit.near = Math.max(0.001, fit.distance - fit.radius * 1.25);
    fit.far = fit.distance + fit.radius * 4;
    this.camera = fit;
    this.requestRender();
  }

  updateRobotBounds() {
    let bounds = null;
    for (const visual of this.visuals) {
      const linkMatrix = this.linkWorldMatrices[visual.linkName];
      if (!linkMatrix) {
        continue;
      }
      const modelMatrix = multiply4(linkMatrix, visual.localMatrix);
      const visualBounds = transformBounds(visual.mesh.bounds, modelMatrix);
      bounds = bounds === null
        ? visualBounds
        : unionBounds(bounds, visualBounds);
    }
    this.robotBounds = bounds;
  }

  registerInteractions() {
    const listen = (name, listener, options) => {
      this.canvas.addEventListener(name, listener, options);
      this.listeners.push([name, listener, options]);
    };
    listen("pointerdown", (event) => {
      if (event.button !== 0) {
        return;
      }
      this.drag = {
        pointerId: event.pointerId,
        x: event.clientX,
        y: event.clientY,
      };
      this.canvas.setPointerCapture?.(event.pointerId);
    });
    listen("pointermove", (event) => {
      if (!this.drag || event.pointerId !== this.drag.pointerId) {
        return;
      }
      const deltaX = event.clientX - this.drag.x;
      const deltaY = event.clientY - this.drag.y;
      this.drag.x = event.clientX;
      this.drag.y = event.clientY;
      const direction = normalize3(this.camera.direction, "camera direction");
      let azimuth = Math.atan2(direction[1], direction[0]);
      let elevation = Math.asin(
        Math.max(-1, Math.min(1, direction[2])),
      );
      azimuth -= deltaX * 0.008;
      elevation = Math.max(
        -Math.PI / 2 + 0.02,
        Math.min(Math.PI / 2 - 0.02, elevation + deltaY * 0.008),
      );
      const cosine = Math.cos(elevation);
      this.camera.direction = [
        cosine * Math.cos(azimuth),
        cosine * Math.sin(azimuth),
        Math.sin(elevation),
      ];
      this.camera.up = [0, 0, 1];
      this.requestRender();
    });
    const endDrag = (event) => {
      if (this.drag && event.pointerId === this.drag.pointerId) {
        this.drag = null;
      }
    };
    listen("pointerup", endDrag);
    listen("pointercancel", endDrag);
    listen(
      "wheel",
      (event) => {
        event.preventDefault?.();
        const minimum = Math.max(0.05, this.camera.radius * 1.05);
        const maximum = Math.max(10, this.camera.radius * 100);
        this.camera.distance = Math.max(
          minimum,
          Math.min(
            maximum,
            this.camera.distance * Math.exp(event.deltaY * 0.001),
          ),
        );
        this.camera.near = Math.max(
          0.001,
          this.camera.distance - this.camera.radius * 1.25,
        );
        this.camera.far = this.camera.distance + this.camera.radius * 4;
        this.requestRender();
      },
      {passive: false},
    );
    listen("dblclick", () => this.fitToRobot());
  }

  requestRender() {
    if (
      this.disposed ||
      !this.gl ||
      this.animationFrame !== null ||
      typeof globalThis.requestAnimationFrame !== "function"
    ) {
      return;
    }
    this.animationFrame = globalThis.requestAnimationFrame(() => {
      this.animationFrame = null;
      this.render();
    });
  }

  resizeCanvas() {
    const ratio = Math.min(
      Number.isFinite(globalThis.devicePixelRatio)
        ? globalThis.devicePixelRatio
        : 1,
      2,
    );
    const width = Math.max(
      1,
      Math.round((this.canvas.clientWidth || 1) * ratio),
    );
    const height = Math.max(
      1,
      Math.round((this.canvas.clientHeight || 1) * ratio),
    );
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.camera.aspect = width / height;
    this.gl.viewport(0, 0, width, height);
  }

  drawResource(resource, modelMatrix, color, mode, viewProjection) {
    const gl = this.gl;
    gl.uniformMatrix4fv(
      this.locations.viewProjection,
      false,
      new Float32Array(viewProjection),
    );
    gl.uniformMatrix4fv(
      this.locations.model,
      false,
      new Float32Array(modelMatrix),
    );
    gl.uniformMatrix3fv(
      this.locations.normalMatrix,
      false,
      new Float32Array(normalMatrix3(modelMatrix)),
    );
    gl.uniform4fv(this.locations.color, new Float32Array(color));
    gl.bindBuffer(gl.ARRAY_BUFFER, resource.positionBuffer);
    gl.enableVertexAttribArray(this.locations.position);
    gl.vertexAttribPointer(
      this.locations.position,
      3,
      gl.FLOAT,
      false,
      0,
      0,
    );
    gl.bindBuffer(gl.ARRAY_BUFFER, resource.normalBuffer);
    gl.enableVertexAttribArray(this.locations.normal);
    gl.vertexAttribPointer(
      this.locations.normal,
      3,
      gl.FLOAT,
      false,
      0,
      0,
    );
    gl.drawArrays(mode, 0, resource.count);
  }

  drawRobot(linkMatrices, ghost, viewProjection) {
    for (const visual of this.visuals) {
      const linkMatrix = linkMatrices[visual.linkName];
      if (!linkMatrix) {
        continue;
      }
      const modelMatrix = multiply4(linkMatrix, visual.localMatrix);
      const color = ghost
        ? [visual.color[0], visual.color[1], visual.color[2], GHOST_ALPHA]
        : [visual.color[0], visual.color[1], visual.color[2], 1];
      this.drawResource(
        visual.mesh,
        modelMatrix,
        color,
        this.gl.TRIANGLES,
        viewProjection,
      );
    }
  }

  render() {
    if (this.disposed || !this.gl || !this.program) {
      return;
    }
    const gl = this.gl;
    this.resizeCanvas();
    const radius = Math.max(this.camera.radius, 0.05);
    this.camera.near = Math.max(
      0.001,
      this.camera.distance - radius * 1.25,
    );
    this.camera.far = Math.max(
      this.camera.near + 1,
      this.camera.distance + radius * 4,
    );
    const viewProjection = multiply4(
      perspective4(
        this.camera.fovYRad,
        this.camera.aspect,
        this.camera.near,
        this.camera.far,
      ),
      lookAtCamera4(this.camera),
    );

    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(0.025, 0.035, 0.05, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    gl.useProgram(this.program);

    gl.disable(gl.BLEND);
    gl.depthMask(true);
    this.drawRobot(this.linkWorldMatrices, false, viewProjection);

    if (this.ghostLinkWorldMatrices) {
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.depthMask(false);
      try {
        this.drawRobot(
          this.ghostLinkWorldMatrices,
          true,
          viewProjection,
        );
      } finally {
        gl.depthMask(true);
        gl.disable(gl.BLEND);
      }
    }

    if (this.tableVisible) {
      for (const table of this.tableVisuals) {
        this.drawResource(
          this.boxResource,
          table.modelMatrix,
          [table.color[0], table.color[1], table.color[2], 1],
          gl.TRIANGLES,
          viewProjection,
        );
      }
    }
    if (this.gridModelMatrix) {
      this.drawResource(
        this.gridResource,
        this.gridModelMatrix,
        [0.2, 0.42, 0.52, 1],
        gl.LINES,
        viewProjection,
      );
    }
  }

  dispose() {
    if (this.disposed) {
      return;
    }
    this.disposed = true;
    this.loadGeneration += 1;
    if (
      this.animationFrame !== null &&
      typeof globalThis.cancelAnimationFrame === "function"
    ) {
      globalThis.cancelAnimationFrame(this.animationFrame);
    }
    this.animationFrame = null;
    for (const [name, listener, options] of this.listeners) {
      this.canvas.removeEventListener(name, listener, options);
    }
    this.listeners = [];
    if (this.gl) {
      for (const buffer of this.buffers) {
        this.gl.deleteBuffer(buffer);
      }
      for (const shader of this.shaders) {
        this.gl.deleteShader(shader);
      }
      for (const program of this.programs) {
        this.gl.deleteProgram(program);
      }
    }
    this.buffers.clear();
    this.shaders.clear();
    this.programs.clear();
    this.meshResources.clear();
    this.visuals = [];
    this.tableVisuals = [];
    this.linkWorldMatrices = {};
    this.ghostLinkWorldMatrices = null;
  }
}
