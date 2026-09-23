/**
 * Trim solver tests.
 *
 * Trim is the load-bearing numerical routine in this project: every scenario
 * starts from a trim point, and the solver doubles as the definition of which
 * flight conditions exist at all. A solver that reports success at a large
 * residual would make every downstream number wrong while every test passed, so
 * these assert on the residual vector itself and not only on the converged flag.
 */

import { test } from 'node:test';
import assert from 'node:assert/strict';

import { C172, GLIDER } from '../model/presets.ts';
import {
  bestLiftToDrag,
  bestLiftToDragCoefficient,
  speedForLiftCoefficient,
  stallAngleRad,
  stallSpeedMps,
} from '../model/aircraft.ts';
import { isaAtGeometricAltitude, G0 } from '../atmos/isa.ts';
import { altitudeFromState, evaluateDerivative, initialState } from '../dynamics/eom.ts';
import { trim, trimResiduals, layoutFor } from './trim.ts';
import { vec3 } from '../math/vec3.ts';
import { minimumLevelTrimSpeed, maxLevelSpeed } from '../scenarios/flight.ts';

const RESIDUAL_TOLERANCE = 1e-8;

test('level trim converges across the usable envelope with a negligible residual', () => {
  const speeds = [34, 42, 50, 58, 64];
  const altitudes = [0, 1500, 3000];
  let checked = 0;

  for (const altitudeM of altitudes) {
    for (const trueAirspeedMps of speeds) {
      const solution = trim(C172, { altitudeM, trueAirspeedMps });
      assert.ok(
        solution.converged,
        `trim failed at ${altitudeM} m / ${trueAirspeedMps} m/s: residual ${solution.residualNorm}`,
      );
      assert.ok(
        solution.residualNorm < RESIDUAL_TOLERANCE,
        `residual ${solution.residualNorm} too large at ${altitudeM} m / ${trueAirspeedMps} m/s`,
      );
      assert.equal(solution.stalled, false);
      checked += 1;
    }
  }
  assert.equal(checked, speeds.length * altitudes.length);
});

test('the residual vector really is the equation-of-motion acceleration', () => {
  // Guards against the trim solving a hand-written set of equations that has
  // drifted away from the ones the simulator integrates — the failure mode
  // where both look right and the aeroplane quietly sinks.
  const solution = trim(C172, { altitudeM: 1200, trueAirspeedMps: 55 });
  const state = initialState(
    1200,
    vec3(55 * Math.cos(solution.alphaRad), 0, 55 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );
  const { derivative } = evaluateDerivative(C172, state, solution.actuators);

  assert.ok(Math.abs(derivative.accelerationBody.x - solution.residualVector[0]) < 1e-14);
  assert.ok(Math.abs(derivative.accelerationBody.z - solution.residualVector[1]) < 1e-14);
  assert.ok(Math.abs(derivative.angularAccelerationBody.y - solution.residualVector[2]) < 1e-14);
});

test('the trim solution is stationary in the full 6-DOF state', () => {
  const solution = trim(C172, { altitudeM: 2000, trueAirspeedMps: 52 });
  const state = initialState(
    2000,
    vec3(52 * Math.cos(solution.alphaRad), 0, 52 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );
  const { derivative, diagnostics } = evaluateDerivative(C172, state, solution.actuators);

  // No linear acceleration, no angular acceleration, no attitude rate.
  for (const key of ['x', 'y', 'z'] as const) {
    assert.ok(Math.abs(derivative.accelerationBody[key]) < RESIDUAL_TOLERANCE, `acceleration.${key}`);
  }
  // Trimmed straight-and-level: the aeroplane is moving, and in NED that motion
  // is horizontal. (An earlier version of this test required all three NED
  // components to be non-zero, which contradicts the level-flight condition it
  // is checking — the vertical component is zero by definition.)
  assert.ok(
    Math.hypot(derivative.velocityNed.x, derivative.velocityNed.y) > 1,
    'the aircraft must be moving horizontally in NED',
  );
  assert.ok(Math.abs(derivative.attitudeRate.w) < 1e-15);
  assert.ok(Math.abs(derivative.angularAccelerationBody.y) < RESIDUAL_TOLERANCE);

  // Flight path angle zero means the NED velocity is level.
  assert.ok(Math.abs(derivative.velocityNed.z) < 1e-9, 'a level trim must have zero vertical velocity');
  // The reported load factor is the body-z specific force, which at a trim is
  // exactly cos(theta). It is not 1.0 unless the fuselage is level: at this trim
  // theta ~ 0.05 rad, so it reads ~0.999.
  assert.ok(
    Math.abs(diagnostics.loadFactor - Math.cos(solution.thetaRad)) < 1e-9,
    `load factor ${diagnostics.loadFactor} should equal cos(theta) ${Math.cos(solution.thetaRad)}`,
  );
});

test('trim reports failure below the stall speed instead of inventing a solution', () => {
  const vs = stallSpeedMps(C172, isaAtGeometricAltitude(0));
  const solution = trim(C172, { altitudeM: 0, trueAirspeedMps: vs * 0.7 });

  assert.equal(solution.converged, false, 'a below-stall level trim must not converge');
  assert.ok(solution.residualNorm > 1, `expected a large residual, got ${solution.residualNorm}`);
});

test('trim reports failure above maximum level speed instead of inventing a solution', () => {
  const solution = trim(C172, { altitudeM: 0, trueAirspeedMps: 90 });

  assert.equal(solution.converged, false, 'a level trim above Vmax must not converge');
  assert.ok(solution.residualNorm > 0.1);
  // Throttle should be pinned at the stop, which is the physical reason.
  assert.equal(solution.controls.throttle, 1);
});

test('minimum level trim speed agrees with the closed-form 1-g stall speed', () => {
  // This is the tight check on the lift model. The scripted stall is expected to
  // sit a few percent below Vs (the aeroplane is already descending at onset),
  // but the minimum speed at which a 1-g LEVEL trim exists should land almost
  // exactly on sqrt(2W / (rho S CLmax)).
  // The glider is searched as a GLIDE (throttle 0). An unpowered aircraft has no
  // level-flight trim at any speed, because level flight requires thrust equal
  // to the drag; asking for one returns NaN rather than a number.
  for (const [spec, altitudeM, throttle] of [
    [C172, 0, undefined],
    [C172, 3000, undefined],
    [GLIDER, 500, 0],
  ] as const) {
    const measured = minimumLevelTrimSpeed(spec, altitudeM, 0.02, throttle).speedMps;
    const closedForm = stallSpeedMps(spec, isaAtGeometricAltitude(altitudeM));
    const errorPercent = ((measured - closedForm) / closedForm) * 100;
    assert.ok(
      Math.abs(errorPercent) < 2.5,
      `${spec.name} at ${altitudeM} m: trim-min ${measured.toFixed(3)} vs Vs ${closedForm.toFixed(3)} (${errorPercent.toFixed(2)}%)`,
    );
  }
});

test('the stall-break blend does not move the stall speed far from CLmax', () => {
  // The blended lift curve peaks slightly below CLMax, which is why the measured
  // minimum trim speed comes out ~1.5% low rather than exactly on it. Assert the
  // size of that offset is small, so a future change to the blend cannot quietly
  // shift the stall speed without anyone noticing.
  const measured = minimumLevelTrimSpeed(C172, 0).speedMps;
  const closedForm = stallSpeedMps(C172, isaAtGeometricAltitude(0));
  assert.ok(measured < closedForm, 'the blended peak is expected to fall just short of CLMax');
  assert.ok(closedForm - measured < 1, `stall speed offset ${(closedForm - measured).toFixed(3)} m/s is larger than expected`);
});

test('maximum level speed is realistic for the airframe', () => {
  const result = maxLevelSpeed(C172, 0);
  const knots = result.speedMps * 1.943844;
  // Published sea-level maximum for a 172N-class aeroplane is about 126 kt.
  assert.ok(knots > 118 && knots < 136, `max level speed ${knots.toFixed(1)} kt is outside a credible band`);
});

test('maximum level speed is undefined for an unpowered airframe', () => {
  // A glider has no level trim at any speed — level flight needs thrust equal to
  // the drag and it has no engine — so it has no maximum level speed. The
  // function must report that rather than returning a number. It previously
  // returned its own 30 m/s initial guess, which read like a performance figure
  // and was nothing of the kind.
  assert.ok(Number.isNaN(maxLevelSpeed(GLIDER, 0).speedMps), 'the glider has no maximum level speed');
});

test('flight path angle and lift-to-drag are two views of the same glide', () => {
  // In a trimmed glide tan|gamma| = D/L exactly, so the coefficient ratio and
  // the trajectory angle must agree. This is a genuinely independent check: one
  // number comes from the aerodynamic coefficients, the other from the geometry
  // of the equilibrium.
  for (const airspeedMps of [25, 29, 36, 45]) {
    const solution = trim(GLIDER, { altitudeM: 600, trueAirspeedMps: airspeedMps, throttle: 0 });
    assert.ok(solution.converged, `glide trim failed at ${airspeedMps} m/s`);
    assert.ok(solution.flightPathAngleRad < 0, 'an unpowered trim must descend');

    const fromAngle = 1 / Math.abs(Math.tan(solution.flightPathAngleRad));
    assert.ok(
      Math.abs(fromAngle - solution.liftToDrag) / solution.liftToDrag < 1e-6,
      `glide ratio from angle ${fromAngle.toFixed(6)} vs from coefficients ${solution.liftToDrag.toFixed(6)}`,
    );
  }
});

test('the trimmed polar reproduces the closed-form best lift-to-drag ratio', () => {
  for (const spec of [C172, GLIDER]) {
    const atmosphere = isaAtGeometricAltitude(500);
    const clStar = bestLiftToDragCoefficient(spec);
    const vStar = speedForLiftCoefficient(spec, atmosphere, clStar);
    const solution = trim(spec, { altitudeM: 500, trueAirspeedMps: vStar, throttle: 0 });

    assert.ok(solution.converged);
    // The closed form is the parabolic-polar idealisation 1/(2 sqrt(CD0 k)),
    // evaluated at CL*; the trim resolves the full equilibrium, in which the
    // lift and the axial force balance couple through alpha. Measured agreement
    // is 5.8e-6 for the C172 and 3.2e-8 for the glider, so the band is set at
    // 1e-4 — still tight enough to catch a real change to the polar.
    assert.ok(
      Math.abs(solution.liftToDrag - bestLiftToDrag(spec)) / bestLiftToDrag(spec) < 1e-4,
      `${spec.name}: trimmed L/D ${solution.liftToDrag.toFixed(6)} vs closed form ${bestLiftToDrag(spec).toFixed(6)}`,
    );

    // The optimum found by sweeping speeds must land on the same speed.
    let bestSpeed = 0;
    let bestRatio = 0;
    for (let v = vStar * 0.7; v <= vStar * 1.3; v += 0.2) {
      const candidate = trim(spec, { altitudeM: 500, trueAirspeedMps: v, throttle: 0 });
      if (candidate.converged && candidate.liftToDrag > bestRatio) {
        bestRatio = candidate.liftToDrag;
        bestSpeed = v;
      }
    }
    assert.ok(
      Math.abs(bestSpeed - vStar) < 0.6,
      `${spec.name}: swept optimum at ${bestSpeed.toFixed(2)} m/s vs closed form ${vStar.toFixed(2)} m/s`,
    );
  }
});

test('a glide with the wrong elevator still satisfies the vertical force balance', () => {
  // The residual is checked at a deliberate non-solution, where it must NOT be
  // small: this is the control that proves the convergence assertions above are
  // not passing trivially.
  const layout = layoutFor({ altitudeM: 600, trueAirspeedMps: 29, throttle: 0 });
  const atTrim = trim(GLIDER, { altitudeM: 600, trueAirspeedMps: 29, throttle: 0 });
  const good = trimResiduals(
    GLIDER,
    { altitudeM: 600, trueAirspeedMps: 29, throttle: 0 },
    Float64Array.from([atTrim.alphaRad, atTrim.controls.elevator, atTrim.flightPathAngleRad]),
    layout,
  );
  const bad = trimResiduals(
    GLIDER,
    { altitudeM: 600, trueAirspeedMps: 29, throttle: 0 },
    Float64Array.from([atTrim.alphaRad, atTrim.controls.elevator + 0.5, atTrim.flightPathAngleRad]),
    layout,
  );

  let goodNorm = 0;
  let badNorm = 0;
  for (let i = 0; i < 3; i++) {
    goodNorm = Math.max(goodNorm, Math.abs(good[i]!));
    badNorm = Math.max(badNorm, Math.abs(bad[i]!));
  }
  assert.ok(goodNorm < RESIDUAL_TOLERANCE, `residual at the solution should be tiny, got ${goodNorm}`);
  assert.ok(badNorm > 0.1, `residual away from the solution should be large, got ${badNorm}`);
});

test('specifying both a fixed flight path angle and a fixed throttle is rejected', () => {
  assert.throws(
    () => trim(C172, { altitudeM: 1000, trueAirspeedMps: 50, flightPathAngleRad: 0, throttle: 0.5 }),
    /exactly one must be free/,
  );
});

test('a climb or descent trim carries less than 1 g of lift, in proportion to the path angle', () => {
  // gamma is kept to 0.06 rad (3.4 deg) at 50 m/s. A 5.7 deg climb at that
  // speed needs more thrust than this engine produces at full throttle, so the
  // solver legitimately fails to converge on it — the flight condition does not
  // exist, and asking for a larger angle would be testing the wrong thing.
  const gamma = 0.06;
  const climbing = trim(C172, { altitudeM: 0, trueAirspeedMps: 50, flightPathAngleRad: gamma });
  const descending = trim(C172, { altitudeM: 0, trueAirspeedMps: 50, flightPathAngleRad: -gamma });

  assert.ok(climbing.converged && descending.converged);
  assert.ok(climbing.thetaRad > descending.thetaRad);

  // In a steady climb of angle gamma, L = W cos(gamma), so the lift is slightly
  // LESS than the weight — for either sign of gamma. The original name of this
  // test claimed the opposite; the assertion below was always the correct one.
  const weight = C172.mass.massKg * G0;
  for (const [solution, pathAngle] of [
    [climbing, gamma],
    [descending, -gamma],
  ] as const) {
    const expectedLift = weight * Math.cos(pathAngle);
    assert.ok(
      expectedLift < weight,
      'a non-level trim must carry less than the full weight in lift',
    );
    assert.ok(
      Math.abs(solution.diagnostics.forces.liftN - expectedLift) / expectedLift < 0.02,
      `lift in the climb: ${solution.diagnostics.forces.liftN.toFixed(1)} vs ${expectedLift.toFixed(1)} N`,
    );
  }
});

test('trim altimeter agrees with the state it produced', () => {
  const solution = trim(C172, { altitudeM: 2500, trueAirspeedMps: 50 });
  const state = initialState(
    2500,
    vec3(50 * Math.cos(solution.alphaRad), 0, 50 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );
  assert.ok(Math.abs(altitudeFromState(state) - 2500) < 1e-12);
  assert.ok(solution.alphaRad > 0 && solution.alphaRad < stallAngleRad(C172), 'level trim sits below the stall');
});
