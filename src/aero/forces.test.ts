/**
 * Aerodynamic transform and coefficient-model tests.
 *
 * The isometry test sweeps BOTH alpha and beta. Sweeping alpha alone is
 * insufficient: the original defect in the wind-axis basis left the transform
 * correct at zero sideslip, so every alpha-only check passed while the body
 * force magnitude was up to 40% wrong with sideslip applied.
 */

import { test } from 'node:test';
import assert from 'node:assert/strict';

import { C172 } from '../model/presets.ts';
import { inducedDragFactor, stallAngleRad } from '../model/aircraft.ts';
import { aerodynamicCoefficients, flowAngles, separationFraction } from './coefficients.ts';
import { aeroForcesBody } from './forces.ts';
import { isaAtGeometricAltitude } from '../atmos/isa.ts';
import { cross, dot, length, vec3 } from '../math/vec3.ts';

const atmosphere = isaAtGeometricAltitude(0);
const NEUTRAL = { elevatorRad: 0, aileronRad: 0, rudderRad: 0, throttle: 0 };
const NO_RATE = vec3(0, 0, 0);

/** Synthetic coefficients with chosen CL/CD/CY and a fixed dynamic pressure. */
const synthetic = (alphaRad: number, betaRad: number, cl: number, cd: number, cy: number) =>
  aeroForcesBody(C172, {
    alphaRad,
    betaRad,
    dynamicPressurePa: 500,
    lift: cl,
    drag: cd,
    pitchingMoment: 0,
    sideForce: cy,
    rollingMoment: 0,
    yawingMoment: 0,
    separation: 0,
    machNumber: 0.1,
  });

test('wind-to-body transform preserves force magnitude across alpha AND beta', () => {
  const S = C172.wing.areaM2;
  const qbar = 500;
  const CL = 0.4;
  const CD = 0.05;
  const CY = 0.02;
  const windMagnitude = Math.hypot(CL, CD, CY) * qbar * S;
  let worst = 0;

  for (let a = -1.4; a <= 1.4; a += 0.07) {
    for (let b = -1.2; b <= 1.2; b += 0.09) {
      const { forceBody } = synthetic(a, b, CL, CD, CY);
      worst = Math.max(worst, Math.abs(length(forceBody) - windMagnitude) / windMagnitude);
    }
  }

  // Measured worst case is ~4e-16, i.e. machine precision. The same assertion
  // against the original (wrong) basis returned 4.2e-2.
  assert.ok(worst < 1e-12, `force magnitude is not preserved by the wind-to-body rotation: worst relative error ${worst}`);
});

test('the aerodynamic force from pure lift is perpendicular to the velocity', () => {
  const S = C172.wing.areaM2;
  for (const a of [-1.2, -0.5, -0.1, 0, 0.1, 0.5, 1.2]) {
    const { forceBody } = synthetic(a, 0, 1.0, 0, 0);
    const vDir = vec3(Math.cos(a), 0, Math.sin(a));
    const alongVelocity = dot(forceBody, vDir);
    const expectedDragScale = 1.0 * 500 * S;
    assert.ok(
      Math.abs(alongVelocity) / expectedDragScale < 1e-12,
      `pure lift should have no component along the velocity at alpha=${a}: got ${alongVelocity}`,
    );
    assert.ok(length(cross(forceBody, vDir)) > 0, 'pure lift must not be parallel to the velocity');
  }
});

test('lift acts upwards in body axes (negative z, since body z points down)', () => {
  const { forceBody } = synthetic(0.1, 0, 1.0, 0, 0);
  assert.ok(forceBody.z < 0, 'positive lift must produce a negative body-z force');
});

test('drag acts directly along the velocity vector', () => {
  const S = C172.wing.areaM2;
  for (const a of [-1.0, -0.2, 0.2, 1.0]) {
    for (const b of [-0.4, 0, 0.4]) {
      const { forceBody } = synthetic(a, b, 0, 0.05, 0);
      const vDir = vec3(Math.cos(a) * Math.cos(b), Math.sin(b), Math.sin(a) * Math.cos(b));
      const alongVelocity = dot(forceBody, vDir);
      const expected = -0.05 * 500 * S;
      assert.ok(
        Math.abs(alongVelocity - expected) / Math.abs(expected) < 1e-12,
        `drag component along velocity wrong at a=${a}, b=${b}: ${alongVelocity} vs ${expected}`,
      );
    }
  }
});

test('side force acts along the wind y-axis and vanishes in the plane of symmetry', () => {
  const S = C172.wing.areaM2;
  const Y = 0.03;
  for (const a of [-0.8, 0, 0.8]) {
    for (const b of [-0.5, 0.5]) {
      const { forceBody } = synthetic(a, b, 0, 0, Y);
      const yWind = vec3(-Math.cos(a) * Math.sin(b), Math.cos(b), -Math.sin(a) * Math.sin(b));
      const along = dot(forceBody, yWind);
      assert.ok(Math.abs(along - Y * 500 * S) / (Y * 500 * S) < 1e-12, `side force wrong at a=${a}, b=${b}`);
    }
  }
  // With zero sideslip and no side force, the body force has no y component.
  for (const a of [-1.0, 0.3, 1.0]) {
    assert.ok(Math.abs(synthetic(a, 0, 0.5, 0.04, 0).forceBody.y) < 1e-12);
  }
});

test('flow angles are recovered from a body velocity', () => {
  for (const alpha of [-0.6, -0.1, 0, 0.1, 0.6]) {
    for (const beta of [-0.5, 0, 0.5]) {
      const V = 40;
      const velocity = vec3(
        V * Math.cos(alpha) * Math.cos(beta),
        V * Math.sin(beta),
        V * Math.sin(alpha) * Math.cos(beta),
      );
      const angles = flowAngles(velocity);
      assert.ok(Math.abs(angles.alphaRad - alpha) < 1e-12, `alpha ${angles.alphaRad} vs ${alpha}`);
      assert.ok(Math.abs(angles.betaRad - beta) < 1e-12, `beta ${angles.betaRad} vs ${beta}`);
      assert.ok(Math.abs(angles.speed - V) < 1e-12);
    }
  }
});

test('the stall break starts exactly at the linear-curve stall incidence', () => {
  const alphaStall = stallAngleRad(C172);
  assert.ok(Math.abs(alphaStall - (C172.lift.CLMax / C172.lift.CLAlpha + C172.lift.alpha0Rad)) < 1e-15);
  assert.equal(separationFraction(C172, alphaStall - 1e-6), 0);
  assert.ok(separationFraction(C172, alphaStall + C172.lift.stallBreakWidthRad / 2) > 0.4);
  assert.equal(separationFraction(C172, alphaStall + C172.lift.stallBreakWidthRad + 1e-6), 1);
});

test('lift coefficient is continuous through the stall break and falls beyond it', () => {
  const alphaStall = stallAngleRad(C172);
  const clAt = (alpha: number): number =>
    aerodynamicCoefficients(C172, atmosphere, vec3(50 * Math.cos(alpha), 0, 50 * Math.sin(alpha)), NO_RATE, NEUTRAL)
      .lift;

  // Continuity is checked by step refinement rather than against a fixed
  // threshold. A continuous (Lipschitz) curve has |CL(a + h) - CL(a)| proportional
  // to h, so shrinking the step shrinks the largest jump; a hard switch at CLMax
  // keeps a fixed-size jump no matter how small the step becomes. The first
  // version of this test used a fixed threshold, which the genuinely steep — but
  // perfectly continuous — stall break exceeded, and it would have failed while
  // telling us nothing about continuity.
  const maxJumpAt = (h: number): number => {
    let maxJump = 0;
    for (let a = -0.5; a < 0.9; a += h) {
      maxJump = Math.max(maxJump, Math.abs(clAt(a + h) - clAt(a)));
    }
    return maxJump;
  };
  const coarse = maxJumpAt(0.004);
  const fine = maxJumpAt(0.001);
  assert.ok(fine < 0.05, `steepest lift change over a 0.001 rad step is ${fine}`);
  assert.ok(
    fine < coarse / 2,
    `lift curve looks discontinuous: jump ${fine.toFixed(5)} at h=0.001 did not shrink from ${coarse.toFixed(5)} at h=0.004`,
  );

  // The linear region follows CL = CLAlpha (alpha - alpha0).
  const linearAlpha = alphaStall * 0.5;
  assert.ok(
    Math.abs(clAt(linearAlpha) - C172.lift.CLAlpha * (linearAlpha - C172.lift.alpha0Rad)) < 1e-12,
  );

  // Beyond the break the coefficient must have collapsed relative to the
  // attached-flow extrapolation, which would have reached
  // CLAlpha * (alpha - alpha0) = 3.19 at 0.6 rad. The fully separated model is
  // the flat plate CL = 2 sin(a) cos(a); note that this does NOT keep falling
  // past 45 deg — it peaks at 1.0 there — so the meaningful check is against the
  // linear model and CLMax, not against a fixed low number. An earlier version
  // asserted CL(0.6) < 0.8, which the flat-plate model simply does not satisfy.
  assert.ok(
    clAt(0.6) < 0.5 * C172.lift.CLAlpha * (0.6 - C172.lift.alpha0Rad),
    `post-stall CL ${clAt(0.6)} should be far below the attached-flow extrapolation`,
  );
  assert.ok(clAt(0.6) < C172.lift.CLMax, `post-stall CL ${clAt(0.6)} should be below CLMax`);
  assert.ok(
    Math.abs(clAt(0.6) - 2 * Math.sin(0.6) * Math.cos(0.6)) < 1e-9,
    'fully separated lift should be exactly the flat-plate value 2 sin(a) cos(a)',
  );
  // At 70 deg the flat plate has turned back down: CL = sin(140 deg) = 0.64.
  assert.ok(clAt(1.2) < 0.7, `CL at 70 deg should be low, got ${clAt(1.2)}`);
});

test('drag rises sharply after the stall', () => {
  const alphaStall = stallAngleRad(C172);
  const cdAt = (alpha: number): number =>
    aerodynamicCoefficients(C172, atmosphere, vec3(50 * Math.cos(alpha), 0, 50 * Math.sin(alpha)), NO_RATE, NEUTRAL)
      .drag;
  assert.ok(cdAt(0.5) > 3 * cdAt(alphaStall * 0.5), 'drag should rise steeply with post-stall separation');
});

test('an aircraft with no lift curve produces zero lift and constant drag', () => {
  // The degenerate spec used by the terminal-velocity test. A 0/0 in the stall
  // incidence here would poison the whole integration.
  const dragBody = {
    ...C172,
    lift: { ...C172.lift, CLAlpha: 0, CLMax: 0, alpha0Rad: 0 },
    drag: { CD0: 0.42, oswaldE: 0 },
  };
  assert.equal(stallAngleRad(dragBody), Number.POSITIVE_INFINITY);
  assert.equal(inducedDragFactor(dragBody), 0);

  for (const alpha of [0, 0.3, 1.0, 1.5707, -1.2]) {
    const speed = 30;
    const c = aerodynamicCoefficients(
      dragBody,
      atmosphere,
      vec3(speed * Math.cos(alpha), 0, speed * Math.sin(alpha)),
      NO_RATE,
      NEUTRAL,
    );
    // Compared with === rather than assert.equal: the lift expression can return
    // -0, and the strict assertion helper distinguishes -0 from +0.
    assert.ok(c.lift === 0, `lift must be zero at alpha=${alpha}, got ${c.lift}`);
    assert.ok(Math.abs(c.drag - 0.42) < 1e-12, `drag must stay at CD0, got ${c.drag} at alpha=${alpha}`);
    assert.ok(Number.isFinite(c.drag));
  }
});

test('coefficients stay finite at zero airspeed', () => {
  // A standing-start takeoff evaluates the model at exactly zero airspeed. The
  // non-dimensional rate denominators are 0 there, and an unguarded 0/0 returns
  // NaN and destroys the state within one step.
  const c = aerodynamicCoefficients(C172, atmosphere, vec3(0, 0, 0), vec3(1, 1, 1), NEUTRAL);
  for (const key of [
    'lift', 'drag', 'pitchingMoment', 'sideForce', 'rollingMoment', 'yawingMoment', 'dynamicPressurePa',
  ] as const) {
    assert.ok(Number.isFinite(c[key]), `${key} is not finite at zero airspeed: ${c[key]}`);
  }
  const { forceBody, momentBody } = aeroForcesBody(C172, c);
  for (const key of ['x', 'y', 'z'] as const) {
    assert.ok(Number.isFinite(forceBody[key]), `force.${key} not finite`);
    assert.ok(Number.isFinite(momentBody[key]), `moment.${key} not finite`);
  }
});

test('static stability: a positive incidence increment produces a nose-down moment', () => {
  const alpha = 0.1;
  const base = aerodynamicCoefficients(C172, atmosphere, vec3(50 * Math.cos(alpha), 0, 50 * Math.sin(alpha)), NO_RATE, NEUTRAL);
  const perturbed = aerodynamicCoefficients(
    C172,
    atmosphere,
    vec3(50 * Math.cos(alpha + 0.01), 0, 50 * Math.sin(alpha + 0.01)),
    NO_RATE,
    NEUTRAL,
  );
  assert.ok(perturbed.pitchingMoment < base.pitchingMoment, 'increasing alpha must pitch the nose down');
});

test('elevator trailing-edge-down produces a nose-down moment', () => {
  const alpha = 0.05;
  const velocity = vec3(50 * Math.cos(alpha), 0, 50 * Math.sin(alpha));
  const neutral = aerodynamicCoefficients(C172, atmosphere, velocity, NO_RATE, NEUTRAL);
  const teDown = aerodynamicCoefficients(C172, atmosphere, velocity, NO_RATE, { ...NEUTRAL, elevatorRad: 0.1 });
  assert.ok(teDown.pitchingMoment < neutral.pitchingMoment);
});

test('pitch, roll and yaw damping oppose their own rates', () => {
  const alpha = 0.05;
  const velocity = vec3(50 * Math.cos(alpha), 0, 50 * Math.sin(alpha));
  const at = (rate: { x: number; y: number; z: number }) =>
    aerodynamicCoefficients(C172, atmosphere, velocity, vec3(rate.x, rate.y, rate.z), NEUTRAL);

  const base = at({ x: 0, y: 0, z: 0 });
  assert.ok(at({ x: 0, y: 0.5, z: 0 }).pitchingMoment < base.pitchingMoment, 'pitch damping');
  assert.ok(at({ x: 1.0, y: 0, z: 0 }).rollingMoment < base.rollingMoment, 'roll damping');
  assert.ok(at({ x: 0, y: 0, z: 0.5 }).yawingMoment < base.yawingMoment, 'yaw damping');
});
