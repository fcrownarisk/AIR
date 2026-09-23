/**
 * Integrator and equations-of-motion tests.
 *
 * These check the things that can be asserted in closed form: how fast the
 * integrator converges, whether energy behaves, and whether a pure drag body
 * reaches the textbook terminal velocity. Two of them use degenerate aircraft
 * specs rather than the real presets, deliberately — that isolates the property
 * under test from the aerodynamics.
 */

import { test } from 'node:test';
import assert from 'node:assert/strict';

import { C172, GLIDER } from '../model/presets.ts';
import { type AircraftSpec } from '../model/aircraft.ts';
import { NEUTRAL_ACTUATORS } from '../model/controls.ts';
import { G0, isaAtGeometricAltitude } from '../atmos/isa.ts';
import { altitudeFromState, evaluateDerivative, initialState } from './eom.ts';
import { rk4Step } from './integrator.ts';
import { trim } from '../sim/trim.ts';
import { vec3, length } from '../math/vec3.ts';

/**
 * A body with drag and nothing else: no lift curve, no induced drag, no moments.
 * Falls straight down through a standard atmosphere.
 */
const DRAG_BODY: AircraftSpec = {
  ...C172,
  name: 'drag body',
  lift: { CLAlpha: 0, alpha0Rad: 0, CLMax: 0, stallBreakWidthRad: 0.1 },
  drag: { CD0: 0.42, oswaldE: 0 },
  pitch: { Cm0: 0, CmAlpha: 0, CmQ: 0, CmElevator: 0 },
  lateral: {
    CYBeta: 0, CYRudder: 0, ClBeta: 0, ClAileron: 0, CnAileron: 0,
    ClP: 0, ClR: 0, CnBeta: 0, CnRudder: 0, CnP: 0, CnR: 0,
  },
  propulsion: { maxShaftPowerW: 0, propEfficiency: 0, staticThrustN: 0, staticSpeedFloorMps: 5 },
};

/** A free body under gravity only: everything aerodynamic is switched off. */
const BALLISTIC: AircraftSpec = {
  ...DRAG_BODY,
  name: 'ballistic body',
  drag: { CD0: 0, oswaldE: 0 },
};

test('a pure drag body reaches the closed-form terminal velocity', () => {
  // Vt = sqrt(2 W / (rho S CD0)). Compare against the local atmosphere at the
  // altitude where the measurement is taken, since terminal velocity rises with
  // height.
  const targetAltitudeM = 1000;
  const atmosphere = isaAtGeometricAltitude(targetAltitudeM);
  const expected = Math.sqrt(
    (2 * DRAG_BODY.mass.massKg * G0) /
      (atmosphere.densityKgM3 * DRAG_BODY.wing.areaM2 * DRAG_BODY.drag.CD0),
  );

  let state = initialState(2500, vec3(0, 0, 0), { phi: 0, theta: 0, psi: 0 });
  let descentRate = 0;
  for (let step = 0; step < 200 / 0.02; step++) {
    const result = rk4Step(DRAG_BODY, state, NEUTRAL_ACTUATORS, 0.02);
    state = result.state;
    if (altitudeFromState(state) <= targetAltitudeM) {
      // Falling straight down with the body level, so body w IS the descent rate.
      descentRate = state.velocityBody.z;
      break;
    }
  }

  assert.ok(descentRate > 0, 'the body never reached the target altitude');
  assert.ok(
    Math.abs(descentRate - expected) / expected < 0.02,
    `terminal velocity ${descentRate.toFixed(3)} m/s vs closed form ${expected.toFixed(3)} m/s`,
  );

  // Drag should equal weight at terminal velocity.
  const diagnostics = evaluateDerivative(DRAG_BODY, state, NEUTRAL_ACTUATORS).diagnostics;
  assert.ok(
    Math.abs(diagnostics.forces.dragN - DRAG_BODY.mass.massKg * G0) / (DRAG_BODY.mass.massKg * G0) < 0.03,
    `drag ${diagnostics.forces.dragN.toFixed(1)} N vs weight ${(DRAG_BODY.mass.massKg * G0).toFixed(1)} N`,
  );
});

test('gravity-only motion conserves specific energy and integrates exactly', () => {
  // With no aerodynamic forces the trajectory is a parabola, which RK4
  // reproduces to machine precision. Any error here is a bug in the gravity or
  // kinematics terms, not integration error.
  const initial = initialState(2000, vec3(60, 0, 0), { phi: 0, theta: 0.2, psi: 0.5 });
  const energy0 = 0.5 * 60 * 60 + G0 * 2000;
  let state = initial;
  for (let step = 0; step < 30 / 0.02; step++) {
    state = rk4Step(BALLISTIC, state, NEUTRAL_ACTUATORS, 0.02).state;
  }
  const speed = length(state.velocityBody);
  const energy1 = 0.5 * speed * speed + G0 * altitudeFromState(state);
  assert.ok(
    Math.abs(energy1 - energy0) / energy0 < 1e-12,
    `specific energy drifted from ${energy0} to ${energy1}`,
  );
});

test('energy loss in a glide equals drag power', () => {
  // A genuine coupling test: the rate of change of mechanical energy must equal
  // the work done against drag. This ties the force transform, the drag model
  // and the translational equations together in one relation.
  const solution = trim(GLIDER, { altitudeM: 800, trueAirspeedMps: 32, throttle: 0 });
  let state = initialState(
    800,
    vec3(32 * Math.cos(solution.alphaRad), 0, 32 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );

  const specificEnergy = (s: typeof state, speed: number): number => 0.5 * speed * speed + G0 * altitudeFromState(s);

  const speed0 = length(state.velocityBody);
  const energy0 = specificEnergy(state, speed0);
  const durationS = 20;
  // Average drag power over the run, from the diagnostics at each step.
  let dragPowerIntegral = 0;

  for (let step = 0; step < durationS / 0.02; step++) {
    const result = rk4Step(GLIDER, state, solution.actuators, 0.02);
    dragPowerIntegral +=
      (result.diagnostics.forces.dragN * result.diagnostics.trueAirspeedMps * 0.02) / GLIDER.mass.massKg;
    state = result.state;
  }

  const speed1 = length(state.velocityBody);
  const energy1 = specificEnergy(state, speed1);
  const change = energy0 - energy1;

  assert.ok(
    Math.abs(change - dragPowerIntegral) / dragPowerIntegral < 0.02,
    `energy lost ${change.toFixed(1)} J/kg vs drag work ${dragPowerIntegral.toFixed(1)} J/kg`,
  );
});

test('RK4 converges at fourth order on the full aircraft model', () => {
  // A fourth-order method quarters its error when the step is halved, so each
  // halving cuts the error by 16 and the ratio of successive errors should be
  // about 16.
  //
  // The case has to be chosen with care. The first version of this test started
  // 10% above trim and ran for 20 s; over that trajectory RK4 is so accurate
  // that the discretisation error at 0.04 s had fallen below the round-off
  // accumulated over the run, and the measured "error ratios" were noise (3300,
  // then 0.15). Starting 50% above trim excites a large, slow phugoid whose
  // amplitude keeps the truncation error measurably above round-off at every
  // step used, and running for 60 s lets it accumulate.
  const solution = trim(C172, { altitudeM: 1500, trueAirspeedMps: 60 });
  const start = initialState(
    1500,
    vec3(90 * Math.cos(solution.alphaRad), 0, 90 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );

  const integrate = (dt: number, durationS: number): number => {
    let state = start;
    const steps = Math.round(durationS / dt);
    for (let i = 0; i < steps; i++) {
      state = rk4Step(C172, state, solution.actuators, dt).state;
    }
    return length(state.velocityBody);
  };

  const reference = integrate(0.0005, 60);
  const coarse = integrate(0.2, 60);
  const medium = integrate(0.1, 60);
  const fine = integrate(0.05, 60);

  const errorCoarse = Math.abs(coarse - reference);
  const errorMedium = Math.abs(medium - reference);
  const errorFine = Math.abs(fine - reference);

  assert.ok(errorCoarse > 0, 'the test case must not be an exact equilibrium');
  // The finest error must still be above the round-off floor, or the ratios are
  // measuring noise rather than truncation. Measured: 5.5e-7 at 0.05 s.
  assert.ok(errorFine > 1e-10, `the finest error ${errorFine} is at the round-off floor`);

  const ratio1 = errorCoarse / errorMedium;
  const ratio2 = errorMedium / errorFine;

  // Measured 16.7 and 16.3. A third-order method would give 8 and a second-order
  // method 4, so the band rejects both decisively while absorbing the fact that
  // the asymptotic regime is not reached exactly.
  assert.ok(
    ratio1 > 10 && ratio1 < 26,
    `step-halving error ratio ${ratio1.toFixed(2)} is not consistent with fourth order`,
  );
  assert.ok(
    ratio2 > 10 && ratio2 < 26,
    `step-halving error ratio ${ratio2.toFixed(2)} is not consistent with fourth order`,
  );
});

test('an exactly trimmed aircraft does not move at all', () => {
  // The strongest statement available about the trim solver: the trim point is
  // an equilibrium of the very equations the integrator advances, so the state
  // must be stationary to round-off.
  const solution = trim(C172, { altitudeM: 1500, trueAirspeedMps: 60 });
  let state = initialState(
    1500,
    vec3(60 * Math.cos(solution.alphaRad), 0, 60 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );
  const altitude0 = altitudeFromState(state);

  for (let step = 0; step < 120 / 0.02; step++) {
    state = rk4Step(C172, state, solution.actuators, 0.02).state;
  }

  const speed = length(state.velocityBody);
  assert.ok(Math.abs(altitudeFromState(state) - altitude0) < 1e-3, 'trimmed aircraft changed altitude');
  assert.ok(Math.abs(speed - 60) < 1e-4, 'trimmed aircraft changed airspeed');
});

test('a disturbed aircraft stays finite and returns to a sane attitude', () => {
  // Guards the numerical robustness of the whole pipeline: a large attitude
  // disturbance driven through the full non-linear model must not produce NaN.
  let state = initialState(2000, vec3(70, 0, 5), { phi: 0.6, theta: 0.4, psi: 1.0 });
  state = { ...state, angularRateBody: vec3(1.5, -1.0, 0.8) };

  for (let step = 0; step < 60 / 0.02; step++) {
    const result = rk4Step(C172, state, { ...NEUTRAL_ACTUATORS, elevatorRad: -0.05, aileronRad: 0.02 }, 0.02);
    state = result.state;
    for (const key of ['x', 'y', 'z'] as const) {
      assert.ok(Number.isFinite(state.velocityBody[key]), `velocity.${key} became non-finite at step ${step}`);
      assert.ok(Number.isFinite(state.angularRateBody[key]), `rate.${key} became non-finite at step ${step}`);
    }
    const quaternionNorm =
      Math.hypot(state.attitude.w, state.attitude.x, state.attitude.y, state.attitude.z);
    assert.ok(Math.abs(quaternionNorm - 1) < 1e-9, `quaternion norm drifted to ${quaternionNorm}`);
  }
});
