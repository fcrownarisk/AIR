/**
 * Minimal 3-vector algebra.
 *
 * Vectors are plain objects with named fields rather than arrays or a class:
 * the flight-dynamics code reads as physics (`velocityBody.z`) rather than as
 * index juggling, and states stay structurally cloneable.
 */

export interface Vec3 {
  readonly x: number;
  readonly y: number;
  readonly z: number;
}

export const vec3 = (x: number, y: number, z: number): Vec3 => ({ x, y, z });
export const ZERO3: Vec3 = { x: 0, y: 0, z: 0 };

export const add = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
export const sub = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
export const scale = (a: Vec3, s: number): Vec3 => ({ x: a.x * s, y: a.y * s, z: a.z * s });
export const negate = (a: Vec3): Vec3 => ({ x: -a.x, y: -a.y, z: -a.z });
export const addScaled = (a: Vec3, b: Vec3, s: number): Vec3 => ({
  x: a.x + b.x * s,
  y: a.y + b.y * s,
  z: a.z + b.z * s,
});

export const dot = (a: Vec3, b: Vec3): number => a.x * b.x + a.y * b.y + a.z * b.z;

export const cross = (a: Vec3, b: Vec3): Vec3 => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x,
});

export const length = (a: Vec3): number => Math.hypot(a.x, a.y, a.z);
export const lengthSquared = (a: Vec3): number => a.x * a.x + a.y * a.y + a.z * a.z;

export function normalize(a: Vec3): Vec3 {
  const n = length(a);
  return n > 0 ? scale(a, 1 / n) : ZERO3;
}

export const isFiniteVec3 = (a: Vec3): boolean =>
  Number.isFinite(a.x) && Number.isFinite(a.y) && Number.isFinite(a.z);

export const vec3EqualsApprox = (a: Vec3, b: Vec3, tolerance = 1e-12): boolean =>
  Math.abs(a.x - b.x) <= tolerance && Math.abs(a.y - b.y) <= tolerance && Math.abs(a.z - b.z) <= tolerance;
