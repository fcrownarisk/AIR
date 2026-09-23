/**
 * Attitude representation.
 *
 * Convention: unit quaternion `q` maps BODY axes to NED axes (a "body-to-NED"
 * rotation). Euler angles are extracted in 3-2-1 order:
 *
 *   phi   (roll)  about body x (forward),  positive = right wing down
 *   theta (pitch) about body y (right),    positive = nose up
 *   psi   (yaw)   about body z (down),     positive = nose right (heading increases clockwise from north)
 *
 * Quaternions are used for propagation because the 6-DOF equations of motion
 * integrate attitude through a full 360 degrees of pitch during a vertical
 * manoeuvre, where Euler-angle kinematics are singular.
 */

import { type Vec3, vec3 } from './vec3.ts';

export interface Quat {
  readonly w: number;
  readonly x: number;
  readonly y: number;
  readonly z: number;
}

export interface EulerAngles {
  /** Roll in radians, positive right-wing-down. */
  readonly phi: number;
  /** Pitch in radians, positive nose-up. */
  readonly theta: number;
  /** Yaw/heading in radians, positive nose-right from north. */
  readonly psi: number;
}

export const quat = (w: number, x: number, y: number, z: number): Quat => ({ w, x, y, z });
export const QUAT_IDENTITY: Quat = { w: 1, x: 0, y: 0, z: 0 };

/** Hamilton product. R(a * b) = R(a) R(b). */
export function multiply(a: Quat, b: Quat): Quat {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

export const conjugate = (q: Quat): Quat => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });
export const norm = (q: Quat): number => Math.hypot(q.w, q.x, q.y, q.z);

export function normalizeQuat(q: Quat): Quat {
  const n = norm(q);
  if (n === 0) return QUAT_IDENTITY;
  const s = 1 / n;
  return { w: q.w * s, x: q.x * s, y: q.y * s, z: q.z * s };
}

export const scaleQuat = (q: Quat, s: number): Quat => ({ w: q.w * s, x: q.x * s, y: q.y * s, z: q.z * s });
export const addQuat = (a: Quat, b: Quat): Quat => ({ w: a.w + b.w, x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
export const addScaledQuat = (a: Quat, b: Quat, s: number): Quat =>
  ({ w: a.w + b.w * s, x: a.x + b.x * s, y: a.y + b.y * s, z: a.z + b.z * s });

export const quatDot = (a: Quat, b: Quat): number => a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

/** Build a body-to-NED quaternion from 3-2-1 Euler angles: q = qz(psi) qy(theta) qx(phi). */
export function quatFromEuler(e: EulerAngles): Quat {
  const [cph, sph] = [Math.cos(e.phi / 2), Math.sin(e.phi / 2)];
  const [cth, sth] = [Math.cos(e.theta / 2), Math.sin(e.theta / 2)];
  const [cps, sps] = [Math.cos(e.psi / 2), Math.sin(e.psi / 2)];
  return {
    w: cph * cth * cps + sph * sth * sps,
    x: sph * cth * cps - cph * sth * sps,
    y: cph * sth * cps + sph * cth * sps,
    z: cph * cth * sps - sph * sth * cps,
  };
}

export function eulerFromQuat(q: Quat): EulerAngles {
  const { w, x, y, z } = q;
  const sinTheta = 2 * (w * y - z * x);
  // Clamp guards against |sinTheta| drifting a hair past 1 at the poles.
  const theta = Math.asin(Math.max(-1, Math.min(1, sinTheta)));

  // Gimbal-lock branch: at |theta| = 90 deg, yaw and roll are degenerate, so
  // roll is set to zero and all heading information is carried by psi.
  if (Math.abs(sinTheta) > 1 - 1e-12) {
    const phi = 0;
    const psi = 2 * Math.atan2(z, w) * (sinTheta > 0 ? 1 : -1);
    return { phi, theta, psi: wrapPi(psi) };
  }

  const phi = Math.atan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y));
  const psi = Math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z));
  return { phi, theta, psi: wrapPi(psi) };
}

/**
 * Rotation matrix R (body -> NED) as a 9-element row-major Float64Array.
 * Float64Array is used for matrices so the linear-algebra code can index
 * without tripping `noUncheckedIndexedAccess` on every element.
 */
export function matrixFromQuat(q: Quat): Float64Array {
  const { w, x, y, z } = q;
  const m = new Float64Array(9);
  m[0] = 1 - 2 * (y * y + z * z);
  m[1] = 2 * (x * y - w * z);
  m[2] = 2 * (x * z + w * y);
  m[3] = 2 * (x * y + w * z);
  m[4] = 1 - 2 * (x * x + z * z);
  m[5] = 2 * (y * z - w * x);
  m[6] = 2 * (x * z - w * y);
  m[7] = 2 * (y * z + w * x);
  m[8] = 1 - 2 * (x * x + y * y);
  return m;
}

export function applyMatrix(m: Float64Array, v: Vec3): Vec3 {
  return vec3(
    m[0]! * v.x + m[1]! * v.y + m[2]! * v.z,
    m[3]! * v.x + m[4]! * v.y + m[5]! * v.z,
    m[6]! * v.x + m[7]! * v.y + m[8]! * v.z,
  );
}

/** Rotate a vector from BODY frame into NED frame. */
export const bodyToNed = (q: Quat, v: Vec3): Vec3 => applyMatrix(matrixFromQuat(q), v);

/**
 * Rotate a vector from NED frame into BODY frame.
 *
 * This is R^T * v, with R from `matrixFromQuat`. It is written in terms of the
 * same nine elements as R rather than as a hand-transcribed transpose.
 *
 * An earlier version expanded the transpose by hand and got two off-diagonal
 * signs wrong, so this was NOT the inverse of `bodyToNed`:
 *   bodyToNed(q, nedToBody(q, (20,0,0)))  ->  (18.79, 0, -6.84)
 * Nothing that touches only body-frame quantities noticed — the aerodynamics,
 * the trim solver and level flight were all unaffected. It surfaced as a
 * takeoff that accelerated to 10 m/s and stopped, because the ground constraint
 * rebuilt the body velocity from the wrong attitude and quietly deleted about
 * 2400 N of thrust. Deriving it from the shared matrix removes the class of
 * error entirely; the roundtrip is asserted in the test suite.
 */
export function nedToBody(q: Quat, v: Vec3): Vec3 {
  const m = matrixFromQuat(q);
  return vec3(
    m[0]! * v.x + m[3]! * v.y + m[6]! * v.z,
    m[1]! * v.x + m[4]! * v.y + m[7]! * v.z,
    m[2]! * v.x + m[5]! * v.y + m[8]! * v.z,
  );
}

/** Quaternion kinematics for a body-frame angular rate: qdot = 0.5 * q (x) [0, omega]. */
export function quatRate(q: Quat, omega: Vec3): Quat {
  return scaleQuat(multiply(q, { w: 0, x: omega.x, y: omega.y, z: omega.z }), 0.5);
}

/** Wrap an angle in radians into (-pi, pi]. */
export function wrapPi(angle: number): number {
  const twoPi = 2 * Math.PI;
  let a = angle % twoPi;
  if (a > Math.PI) a -= twoPi;
  if (a <= -Math.PI) a += twoPi;
  return a;
}

/** Magnitude of the shortest angular difference between two angles, in radians. */
export const angleDifference = (a: number, b: number): number => Math.abs(wrapPi(a - b));
