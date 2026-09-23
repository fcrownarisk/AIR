/**
 * Attitude representation tests.
 *
 * These exist because a hand-expanded rotation matrix is exactly the kind of
 * code that looks right, passes every check that stays in one frame, and is
 * wrong in a way that only shows up when two frames are mixed. The roundtrip
 * test below is the one that would have caught the original `nedToBody`.
 */

import { test } from 'node:test';
import assert from 'node:assert/strict';

import {
  QUAT_IDENTITY,
  bodyToNed,
  conjugate,
  eulerFromQuat,
  matrixFromQuat,
  multiply,
  nedToBody,
  norm,
  normalizeQuat,
  quatFromEuler,
  quatRate,
  wrapPi,
} from './quat.ts';
import { vec3, length, dot, cross } from './vec3.ts';

const SAMPLE_EULER = [
  { phi: 0, theta: 0, psi: 0 },
  { phi: 0.1, theta: 0.2, psi: 0.3 },
  { phi: -0.4, theta: 0.5, psi: 2.8 },
  { phi: 1.2, theta: -0.9, psi: -3.0 },
  { phi: 2.9, theta: 1.1, psi: 0.02 },
  // Statically valid attitudes (|theta| < 90 deg) only; the Euler singularity
  // is exercised separately below.
  { phi: 0.7, theta: 1.4, psi: -1.9 },
];

test('euler -> quaternion -> euler roundtrips for statically valid attitudes', () => {
  for (const e of SAMPLE_EULER) {
    const back = eulerFromQuat(quatFromEuler(e));
    for (const key of ['phi', 'theta', 'psi'] as const) {
      assert.ok(
        Math.abs(wrapPi(back[key] - e[key])) < 1e-12,
        `roundtrip failed for ${key} at ${JSON.stringify(e)}: ${back[key]} vs ${e[key]}`,
      );
    }
  }
});

test('bodyToNed and nedToBody are exact inverses', () => {
  // This is the regression test for the transposed-matrix defect: the two
  // functions were NOT inverses, and it took a takeoff that accelerated to
  // 10 m/s and stopped to expose it.
  for (const e of SAMPLE_EULER) {
    const q = quatFromEuler(e);
    for (const v of [vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1), vec3(3, -7, 11), vec3(-20, 4, 0)]) {
      const roundtrip = bodyToNed(q, nedToBody(q, v));
      assert.ok(
        Math.abs(roundtrip.x - v.x) < 1e-12 &&
          Math.abs(roundtrip.y - v.y) < 1e-12 &&
          Math.abs(roundtrip.z - v.z) < 1e-12,
        `frame roundtrip failed for ${JSON.stringify(v)} at ${JSON.stringify(e)}: got ${JSON.stringify(roundtrip)}`,
      );
      const other = nedToBody(q, bodyToNed(q, v));
      assert.ok(Math.abs(other.x - v.x) < 1e-12 && Math.abs(other.y - v.y) < 1e-12 && Math.abs(other.z - v.z) < 1e-12);
    }
  }
});

test('rotation preserves length and the matrix is orthonormal with determinant 1', () => {
  for (const e of SAMPLE_EULER) {
    const q = quatFromEuler(e);
    const r = matrixFromQuat(q);

    for (const v of [vec3(1, 2, 3), vec3(-4, 0, 9)]) {
      assert.ok(Math.abs(length(bodyToNed(q, v)) - length(v)) < 1e-12);
    }

    // R R^T = I
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let sum = 0;
        for (let k = 0; k < 3; k++) sum += r[i * 3 + k]! * r[j * 3 + k]!;
        assert.ok(Math.abs(sum - (i === j ? 1 : 0)) < 1e-12, `orthonormality failed at ${i},${j}`);
      }
    }

    const det =
      r[0]! * (r[4]! * r[8]! - r[5]! * r[7]!) -
      r[1]! * (r[3]! * r[8]! - r[5]! * r[6]!) +
      r[2]! * (r[3]! * r[7]! - r[4]! * r[6]!);
    assert.ok(Math.abs(det - 1) < 1e-12);
  }
});

test('pitch attitude moves the nose up in NED (z is down)', () => {
  const q = quatFromEuler({ phi: 0, theta: 0.2, psi: 0 });
  const nose = bodyToNed(q, vec3(1, 0, 0));
  assert.ok(nose.z < 0, 'nose-up attitude must give the body x-axis a negative NED z component');
  assert.ok(Math.abs(Math.atan2(-nose.z, nose.x) - 0.2) < 1e-12);
});

test('the aircraft pitched up with a horizontal velocity has positive angle of attack', () => {
  // Wind comes from below the chord: alpha is positive. The sign of this is what
  // a sign error in nedToBody inverted.
  const theta = 0.1745;
  const q = quatFromEuler({ phi: 0, theta, psi: 0 });
  const velocityBody = nedToBody(q, vec3(20, 0, 0));
  const alpha = Math.atan2(velocityBody.z, velocityBody.x);
  assert.ok(Math.abs(alpha - theta) < 1e-9, `expected alpha = +theta, got ${alpha}`);
});

test('quaternion multiplication composes rotations in the same order as the matrices', () => {
  const a = quatFromEuler({ phi: 0.3, theta: -0.2, psi: 1.1 });
  const b = quatFromEuler({ phi: -0.5, theta: 0.4, psi: 0.2 });
  const v = vec3(0.3, -1.2, 2.4);
  const viaQuaternion = bodyToNed(normalizeQuat(multiply(a, b)), v);
  const viaMatrices = bodyToNed(a, bodyToNed(b, v));
  for (const key of ['x', 'y', 'z'] as const) {
    assert.ok(Math.abs(viaQuaternion[key] - viaMatrices[key]) < 1e-12);
  }
});

test('normalisation and conjugation behave', () => {
  const q = normalizeQuat({ w: 2, x: 0, y: 0, z: 0 });
  assert.equal(q.w, 1);
  assert.ok(Math.abs(norm(normalizeQuat({ w: 1, x: 2, y: 3, z: 4 })) - 1) < 1e-15);

  // q * q* = identity
  const r = quatFromEuler({ phi: 0.4, theta: 0.5, psi: 0.6 });
  const identity = multiply(r, conjugate(r));
  assert.ok(Math.abs(identity.w - 1) < 1e-12);
  assert.ok(Math.abs(identity.x) < 1e-12 && Math.abs(identity.y) < 1e-12 && Math.abs(identity.z) < 1e-12);
});

test('quaternion rate is consistent with finite-difference propagation', () => {
  // Rotating a body frame vector by q + dt*qdot must agree, to first order, with
  // rotating by the exact incremental rotation.
  const q = quatFromEuler({ phi: 0.1, theta: 0.2, psi: 0.3 });
  const omega = vec3(0.4, -0.7, 0.25);
  const dt = 1e-7;
  const rate = quatRate(q, omega);
  const predicted = normalizeQuat({ w: q.w + rate.w * dt, x: q.x + rate.x * dt, y: q.y + rate.y * dt, z: q.z + rate.z * dt });

  const angle = length(omega) * dt;
  const axis = vec3(omega.x / length(omega), omega.y / length(omega), omega.z / length(omega));
  const incremental = quatFromEuler({
    phi: axis.x * angle,
    theta: axis.y * angle,
    psi: axis.z * angle,
  });
  const exact = normalizeQuat(multiply(q, incremental));

  for (const key of ['w', 'x', 'y', 'z'] as const) {
    assert.ok(Math.abs(predicted[key] - exact[key]) < 1e-9, `${key}: ${predicted[key]} vs ${exact[key]}`);
  }
});

test('wrapPi maps into (-pi, pi]', () => {
  assert.ok(Math.abs(wrapPi(3 * Math.PI) - Math.PI) < 1e-12);
  assert.ok(Math.abs(wrapPi(-3 * Math.PI) - Math.PI) < 1e-12);
  assert.equal(wrapPi(0.5), 0.5);
  assert.ok(Math.abs(wrapPi(2 * Math.PI + 0.1) - 0.1) < 1e-12);
});

test('identity quaternion leaves vectors unchanged', () => {
  const v = vec3(1, -2, 3);
  const out = bodyToNed(QUAT_IDENTITY, v);
  assert.ok(dot(out, v) === dot(v, v));
  assert.ok(length(cross(out, v)) < 1e-15);
});
