const REQUIRED_MAX_MESH_BYTES = 67_108_864;
const REQUIRED_MAX_STL_TRIANGLES = 1_000_000;
const BINARY_HEADER_BYTES = 84;
const BINARY_TRIANGLE_BYTES = 50;
const FLOATS_PER_TRIANGLE = 9;
const NUMBER_TOKEN = String.raw`[^\s]+`;

export class StlParseError extends Error {
  constructor(message) {
    super(message);
    this.name = "StlParseError";
  }
}

function checkedLimits(limits) {
  if (
    limits === null ||
    typeof limits !== "object" ||
    limits.maxMeshBytes !== REQUIRED_MAX_MESH_BYTES ||
    limits.maxStlTriangles !== REQUIRED_MAX_STL_TRIANGLES
  ) {
    throw new StlParseError("invalid public manifest limits");
  }
  return limits;
}

export function validateStlLimits(
  byteLength,
  triangleCountOrNull,
  limits,
) {
  const checked = checkedLimits(limits);
  if (!Number.isSafeInteger(byteLength) || byteLength < 0) {
    throw new StlParseError("invalid STL byte length");
  }
  if (byteLength === 0) {
    throw new StlParseError("empty STL");
  }
  if (byteLength > checked.maxMeshBytes) {
    throw new StlParseError("STL exceeds byte limit");
  }
  if (
    triangleCountOrNull !== null &&
    (
      !Number.isSafeInteger(triangleCountOrNull) ||
      triangleCountOrNull < 0
    )
  ) {
    throw new StlParseError("invalid STL triangle count");
  }
  if (
    triangleCountOrNull !== null &&
    triangleCountOrNull > checked.maxStlTriangles
  ) {
    throw new StlParseError("STL exceeds triangle limit");
  }
}

function finitePositions(positions) {
  if (
    positions === null ||
    positions === undefined ||
    typeof positions.length !== "number" ||
    positions.length === 0 ||
    positions.length % FLOATS_PER_TRIANGLE !== 0
  ) {
    throw new StlParseError("positions must contain complete triangles");
  }
  const result = Array.from(positions);
  if (!result.every(Number.isFinite)) {
    throw new StlParseError("positions must be finite");
  }
  return result;
}

export function computeMeshBounds(positions) {
  const values = finitePositions(positions);
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let index = 0; index < values.length; index += 3) {
    for (let axis = 0; axis < 3; axis += 1) {
      min[axis] = Math.min(min[axis], values[index + axis]);
      max[axis] = Math.max(max[axis], values[index + axis]);
    }
  }
  return {min, max};
}

export function buildVertexNormals(positions) {
  const values = finitePositions(positions);
  const normals = new Float32Array(values.length);
  for (
    let triangleOffset = 0;
    triangleOffset < values.length;
    triangleOffset += FLOATS_PER_TRIANGLE
  ) {
    const ax = values[triangleOffset];
    const ay = values[triangleOffset + 1];
    const az = values[triangleOffset + 2];
    const abx = values[triangleOffset + 3] - ax;
    const aby = values[triangleOffset + 4] - ay;
    const abz = values[triangleOffset + 5] - az;
    const acx = values[triangleOffset + 6] - ax;
    const acy = values[triangleOffset + 7] - ay;
    const acz = values[triangleOffset + 8] - az;
    let nx = aby * acz - abz * acy;
    let ny = abz * acx - abx * acz;
    let nz = abx * acy - aby * acx;
    const length = Math.hypot(nx, ny, nz);
    if (!Number.isFinite(length)) {
      throw new StlParseError("generated normals must be finite");
    }
    if (length === 0) {
      nx = 0;
      ny = 0;
      nz = 0;
    } else {
      nx /= length;
      ny /= length;
      nz /= length;
    }
    for (let vertex = 0; vertex < 3; vertex += 1) {
      const offset = triangleOffset + vertex * 3;
      normals[offset] = nx;
      normals[offset + 1] = ny;
      normals[offset + 2] = nz;
    }
  }
  if (!Array.from(normals).every(Number.isFinite)) {
    throw new StlParseError("generated normals must be finite");
  }
  return normals;
}

function parseBinary(buffer, triangleCount) {
  if (triangleCount === 0) {
    throw new StlParseError("binary STL has zero triangles");
  }
  const view = new DataView(buffer);
  const positions = new Float32Array(
    triangleCount * FLOATS_PER_TRIANGLE,
  );
  let destination = 0;
  for (let triangle = 0; triangle < triangleCount; triangle += 1) {
    const triangleOffset = BINARY_HEADER_BYTES + triangle * BINARY_TRIANGLE_BYTES;
    for (let axis = 0; axis < 3; axis += 1) {
      const normal = view.getFloat32(triangleOffset + axis * 4, true);
      if (!Number.isFinite(normal)) {
        throw new StlParseError("binary STL normals must be finite");
      }
    }
    for (let vertex = 0; vertex < 3; vertex += 1) {
      for (let axis = 0; axis < 3; axis += 1) {
        const value = view.getFloat32(
          triangleOffset + 12 + vertex * 12 + axis * 4,
          true,
        );
        if (!Number.isFinite(value)) {
          throw new StlParseError("binary STL vertices must be finite");
        }
        positions[destination] = value;
        destination += 1;
      }
    }
  }
  return positions;
}

function decodeStrictAscii(bytes) {
  try {
    return new TextDecoder("utf-8", {fatal: true}).decode(bytes);
  } catch (error) {
    throw new StlParseError("invalid ASCII STL encoding");
  }
}

function parseAscii(text, limits) {
  const facetExpression = new RegExp(
    String.raw`\bfacet\s+normal\s+(${NUMBER_TOKEN})\s+(${NUMBER_TOKEN})\s+(${NUMBER_TOKEN})([\s\S]*?)\bendfacet\b`,
    "gi",
  );
  const vertexExpression = new RegExp(
    String.raw`\bvertex\s+(${NUMBER_TOKEN})\s+(${NUMBER_TOKEN})\s+(${NUMBER_TOKEN})`,
    "gi",
  );
  const facetStarts = text.match(/\bfacet\s+normal\b/gi) ?? [];
  const facetEnds = text.match(/\bendfacet\b/gi) ?? [];
  if (
    facetStarts.length === 0 ||
    facetStarts.length !== facetEnds.length
  ) {
    throw new StlParseError("ASCII STL must contain a complete facet");
  }

  const values = [];
  let matchedFacets = 0;
  for (const facet of text.matchAll(facetExpression)) {
    matchedFacets += 1;
    if (matchedFacets > limits.maxStlTriangles) {
      throw new StlParseError("STL exceeds triangle limit");
    }
    const sourceNormal = [
      Number(facet[1]),
      Number(facet[2]),
      Number(facet[3]),
    ];
    if (!sourceNormal.every(Number.isFinite)) {
      throw new StlParseError("ASCII STL normals must be finite");
    }

    const vertices = Array.from(facet[4].matchAll(vertexExpression));
    if (vertices.length !== 3) {
      throw new StlParseError("ASCII STL must contain a complete facet");
    }
    for (const vertex of vertices) {
      for (let axis = 1; axis <= 3; axis += 1) {
        const value = Number(vertex[axis]);
        if (!Number.isFinite(value)) {
          throw new StlParseError("ASCII STL vertices must be finite");
        }
        values.push(value);
      }
    }
  }
  if (matchedFacets !== facetStarts.length) {
    throw new StlParseError("ASCII STL must contain a complete facet");
  }
  const positions = new Float32Array(values);
  if (!Array.from(positions).every(Number.isFinite)) {
    throw new StlParseError("ASCII STL vertices must be finite");
  }
  return positions;
}

export function parseStl(arrayBuffer, limits) {
  if (!(arrayBuffer instanceof ArrayBuffer)) {
    throw new StlParseError("STL input must be an ArrayBuffer");
  }
  validateStlLimits(arrayBuffer.byteLength, null, limits);
  const checked = checkedLimits(limits);
  const bytes = new Uint8Array(arrayBuffer);
  let triangleCount = null;
  let expectedBinaryLength = null;

  if (arrayBuffer.byteLength >= BINARY_HEADER_BYTES) {
    triangleCount = new DataView(arrayBuffer).getUint32(80, true);
    expectedBinaryLength = (
      BINARY_HEADER_BYTES + triangleCount * BINARY_TRIANGLE_BYTES
    );
    if (expectedBinaryLength === arrayBuffer.byteLength) {
      validateStlLimits(arrayBuffer.byteLength, triangleCount, checked);
      const positions = parseBinary(arrayBuffer, triangleCount);
      return {
        positions,
        normals: buildVertexNormals(positions),
        bounds: computeMeshBounds(positions),
      };
    }
  }

  const containsNul = bytes.includes(0);
  let text;
  try {
    text = decodeStrictAscii(bytes);
  } catch (error) {
    if (
      triangleCount !== null &&
      expectedBinaryLength > arrayBuffer.byteLength
    ) {
      validateStlLimits(arrayBuffer.byteLength, triangleCount, checked);
      throw new StlParseError("truncated binary STL");
    }
    throw error;
  }
  if (
    triangleCount !== null &&
    expectedBinaryLength > arrayBuffer.byteLength &&
    containsNul
  ) {
    validateStlLimits(arrayBuffer.byteLength, triangleCount, checked);
    throw new StlParseError("truncated binary STL");
  }
  if (containsNul) {
    throw new StlParseError("invalid ASCII STL encoding");
  }

  const positions = parseAscii(text, checked);
  validateStlLimits(
    arrayBuffer.byteLength,
    positions.length / FLOATS_PER_TRIANGLE,
    checked,
  );
  return {
    positions,
    normals: buildVertexNormals(positions),
    bounds: computeMeshBounds(positions),
  };
}
