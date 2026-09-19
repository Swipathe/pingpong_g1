import test from "node:test";
import assert from "node:assert/strict";

import {
  StlParseError,
  buildVertexNormals,
  computeMeshBounds,
  parseStl,
  validateStlLimits,
} from "../stl.js";

const LIMITS = {
  maxMeshBytes: 67_108_864,
  maxStlTriangles: 1_000_000,
};

const SHORT_ASCII_TRIANGLE = new TextEncoder().encode(
  "facet normal 0 0 1\n" +
  "vertex 0 0 0\n" +
  "vertex 1 0 0\n" +
  "vertex 0 1 0\n" +
  "endfacet\n",
);

function binarySquare({header = "binary mesh", triangles = 2} = {}) {
  const buffer = new ArrayBuffer(84 + triangles * 50);
  const bytes = new Uint8Array(buffer);
  bytes.set(new TextEncoder().encode(header).subarray(0, 80));
  const view = new DataView(buffer);
  view.setUint32(80, triangles, true);
  const facets = [
    [[0, 0, 0], [1, 0, 0], [1, 1, 0]],
    [[0, 0, 0], [1, 1, 0], [0, 1, 0]],
  ];
  for (let triangle = 0; triangle < triangles; triangle += 1) {
    const offset = 84 + triangle * 50;
    view.setFloat32(offset, 0, true);
    view.setFloat32(offset + 4, 0, true);
    view.setFloat32(offset + 8, 1, true);
    for (let vertex = 0; vertex < 3; vertex += 1) {
      for (let axis = 0; axis < 3; axis += 1) {
        view.setFloat32(
          offset + 12 + vertex * 12 + axis * 4,
          facets[triangle][vertex][axis],
          true,
        );
      }
    }
  }
  return buffer;
}

test("short one-triangle ASCII STL parses without reading a binary count", () => {
  assert.ok(SHORT_ASCII_TRIANGLE.byteLength < 84);
  const result = parseStl(SHORT_ASCII_TRIANGLE.buffer, LIMITS);
  assert.ok(result.positions instanceof Float32Array);
  assert.ok(result.normals instanceof Float32Array);
  assert.equal(result.positions.length, 9);
  assert.ok(Array.from(result.positions).every(Number.isFinite));
  assert.deepEqual(result.bounds, {
    min: [0, 0, 0],
    max: [1, 1, 0],
  });
  assert.deepEqual(
    Array.from(result.normals),
    [0, 0, 1, 0, 0, 1, 0, 0, 1],
  );
});

test("binary two-triangle square returns 18 position floats and finite bounds", () => {
  const result = parseStl(binarySquare(), LIMITS);
  assert.equal(result.positions.length, 18);
  assert.equal(result.normals.length, 18);
  assert.ok(Array.from(result.positions).every(Number.isFinite));
  assert.ok(Array.from(result.normals).every(Number.isFinite));
  assert.deepEqual(result.bounds, {
    min: [0, 0, 0],
    max: [1, 1, 0],
  });
});

test("a binary header containing solid is still detected by exact byte length", () => {
  const result = parseStl(binarySquare({header: "solid binary square"}), LIMITS);
  assert.equal(result.positions.length, 18);
  assert.deepEqual(result.bounds.max, [1, 1, 0]);
});

test("truncated binary, zero triangles, and NaN coordinates are rejected", () => {
  const truncated = binarySquare();
  assert.throws(
    () => parseStl(truncated.slice(0, truncated.byteLength - 1), LIMITS),
    (error) => (
      error instanceof StlParseError &&
      error.message === "truncated binary STL"
    ),
  );

  const empty = new ArrayBuffer(84);
  assert.throws(
    () => parseStl(empty, LIMITS),
    (error) => error instanceof StlParseError && /zero triangles/.test(error.message),
  );

  const nan = binarySquare({triangles: 1});
  new DataView(nan).setFloat32(84 + 12, NaN, true);
  assert.throws(
    () => parseStl(nan, LIMITS),
    (error) => error instanceof StlParseError && /finite/.test(error.message),
  );
});

test("ASCII STL without a complete facet is rejected", () => {
  const incomplete = new TextEncoder().encode(
    "solid incomplete\nfacet normal 0 0 1\nvertex 0 0 0\nendsolid\n",
  );
  assert.throws(
    () => parseStl(incomplete.buffer, LIMITS),
    (error) => error instanceof StlParseError && /complete facet/.test(error.message),
  );
});

test("validateStlLimits accepts exact caps and rejects each cap plus one", () => {
  assert.doesNotThrow(() => (
    validateStlLimits(67_108_864, 1_000_000, LIMITS)
  ));
  assert.throws(
    () => validateStlLimits(67_108_865, 1, LIMITS),
    (error) => error instanceof StlParseError && /byte limit/.test(error.message),
  );
  assert.throws(
    () => validateStlLimits(84, 1_000_001, LIMITS),
    (error) => error instanceof StlParseError && /triangle limit/.test(error.message),
  );
});

test("validateStlLimits rejects empty, malformed, or drifted limits", () => {
  assert.throws(
    () => validateStlLimits(0, null, LIMITS),
    (error) => error instanceof StlParseError && /empty/.test(error.message),
  );
  assert.throws(
    () => validateStlLimits(84.5, 1, LIMITS),
    (error) => error instanceof StlParseError && /byte length/.test(error.message),
  );
  assert.throws(
    () => validateStlLimits(84, -1, LIMITS),
    (error) => error instanceof StlParseError && /triangle count/.test(error.message),
  );
  assert.throws(
    () => validateStlLimits(84, 1, {...LIMITS, maxMeshBytes: 1024}),
    (error) => error instanceof StlParseError && /public manifest limits/.test(error.message),
  );
});

test("bounds and generated normals reject malformed or nonfinite positions", () => {
  assert.deepEqual(
    computeMeshBounds(new Float32Array([0, 0, -1, 2, 3, 4, -2, 5, 1])),
    {min: [-2, 0, -1], max: [2, 5, 4]},
  );
  assert.deepEqual(
    Array.from(
      buildVertexNormals(
        new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
      ),
    ),
    [0, 0, 1, 0, 0, 1, 0, 0, 1],
  );
  assert.throws(
    () => computeMeshBounds(new Float32Array()),
    (error) => error instanceof StlParseError && /positions/.test(error.message),
  );
  assert.throws(
    () => buildVertexNormals(new Float32Array([0, 0, 0])),
    (error) => error instanceof StlParseError && /triangles/.test(error.message),
  );
  assert.throws(
    () => computeMeshBounds([0, 0, 0, 1, 0, 0, 0, NaN, 0]),
    (error) => error instanceof StlParseError && /finite/.test(error.message),
  );
});
