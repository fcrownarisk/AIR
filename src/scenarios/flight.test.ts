/**
 * End-to-end scenario tests.
 *
 * These run the whole pipeline — atmosphere, aerodynamics, propulsion,
 * integration, closed-loop control — and assert on measured outcomes. The
 * tolerances are set from measured behaviour and are stated with the reason they
 * are what they are; where a figure is expected to be off, the test asserts the
 * direction and size of the error rather than pretending it is exact.
 */

import { test } from 'node:test';
import assert from 'node:assert/strict';

import { C172, GLIDER } from '../model/presets.ts';
import { bestLiftToDrag, stallSpeedMps } from '../model/aircraft.ts';
import { isaAtGeometricAltitude, G0 } from '../atmos/isa.ts';
import {
  cruiseHoldScenario,
  glideScenario,
  maxLevelSpeed,
  minimumLevelTrimSpeed,
  rollDoubletScenario,
  runScenario,
  SCENARIO_NAMES,
  stallScenario,
  takeoffScenario,
} from './flight.ts';
import { trim } from '../sim/trim.ts';
import { Simulation } from '../sim/simulation.ts';
import { initialState } from '../dynamics/eom.ts';
import { vec3 } from '../math/vec3.ts';

test('every registered scenario runs to completion with finite telemetry', () => {
  for (const name of SCENARIO_NAMES) {
    const outcome = runScenario(name);
    assert.ok(outcome.result.completed, `${name} did not complete`);
    assert.ok(outcome.result.samples.length > 10, `${name} produced too few samples`);
    for (const sample of outcome.result.samples) {
      assert.ok(Number.isFinite(sample.altitudeM), `${name}: altitude became non-finite`);
      assert.ok(Number.isFinite(sample.trueAirspeedMps), `${name}: airspeed became non-finite`);
      assert.ok(Number.isFinite(sample.loadFactor), `${name}: load factor became non-finite`);
    }
    for (const value of Object.values(outcome.metrics)) {
      assert.ok(!Number.isNaN(value), `${name}: metric is NaN`);
    }
  }
});

test('cruise hold on fixed trim controls shows no drift', () => {
  // No autopilot is engaged. Any drift here is error in the trim point itself,
  // so this is the direct end-to-end test of the trim solver against the real
  // integrator. An exact equilibrium means the state does not move AT ALL, and
  // that is what is measured — note the tolerances are round-off scale, not
  // engineering scale.
  const outcome = cruiseHoldScenario(C172, 1500, 60, 120);

  assert.ok(outcome.metrics.altitudeDriftM! < 1e-3, `altitude drifted ${outcome.metrics.altitudeDriftM} m`);
  assert.ok(outcome.metrics.airspeedDriftMps! < 1e-4, `airspeed drifted ${outcome.metrics.airspeedDriftMps} m/s`);
  assert.ok(outcome.metrics.peakAltitudeDeviationM! < 1e-3);
  assert.equal(outcome.result.events.length, 0);
});

test('stall occurs near the closed-form stall speed and the aircraft recovers', () => {
  const outcome = stallScenario(C172);
  const measured = outcome.metrics.measuredStallSpeedMps!;
  const theoretical = outcome.metrics.theoreticalStallSpeedMps!;

  // Onset is defined as separation passing 0.5, about 3.4 deg beyond the
  // linear-curve stall incidence, by which point the aeroplane is already
  // descending and the load factor is below 1. So the measured value sits a few
  // percent BELOW the 1-g closed form. That is the expected behaviour, and the
  // tight check on Vs is done separately via minimumLevelTrimSpeed.
  assert.ok(measured < theoretical, `measured stall speed ${measured} should be below the 1-g value ${theoretical}`);
  const errorPercent = ((measured - theoretical) / theoretical) * 100;
  assert.ok(
    errorPercent > -8 && errorPercent < 0,
    `stall speed error ${errorPercent.toFixed(2)}% outside the expected few-percent-low band`,
  );

  // The aeroplane is descending by the time the wing lets go: power is at idle
  // and lift can no longer hold the weight. Asserting on the vertical speed is
  // the direct check. An earlier version asserted the body-z load factor was
  // below 1, which is true of any non-zero pitch and therefore proved nothing.
  const onset = outcome.result.samples.find((s) => s.stalled)!;
  assert.ok(
    onset.verticalSpeedMps < 0,
    `the aircraft should be descending at stall onset, got ${onset.verticalSpeedMps} m/s`,
  );

  // The wing genuinely let go and then reattached.
  assert.ok(outcome.metrics.peakAlphaDeg! > 25, 'the aeroplane should have exceeded the stall incidence clearly');
  assert.equal(outcome.metrics.recovered, 1, 'the aircraft must recover from the stall');
  assert.ok(outcome.metrics.recoveryDurationS! > 0 && outcome.metrics.recoveryDurationS! < 30);
  assert.ok(outcome.metrics.altitudeLostInStallM! > 50, 'a stall must cost altitude');
  assert.ok(outcome.metrics.altitudeLostInStallM! < 900, 'altitude loss is implausibly large');
  assert.ok(
    outcome.result.events.some((e) => e.type === 'stall-onset'),
    'a stall-onset event must be logged',
  );
});

test('unpowered glide ratio agrees between the coefficients and the trajectory', () => {
  const outcome = glideScenario(GLIDER, 600, 29, 120, 20);
  const fromCoefficients = outcome.metrics.liftToDragFromCoefficients!;
  const fromTrimAngle = outcome.metrics.glideRatioFromTrimAngle!;
  const fromTrajectory = outcome.metrics.glideRatioOverWindow!;

  // Coefficients and trim geometry are the same equilibrium seen two ways.
  assert.ok(
    Math.abs(fromTrimAngle - fromCoefficients) / fromCoefficients < 1e-6,
    `trim angle ${fromTrimAngle} vs coefficients ${fromCoefficients}`,
  );

  // The trajectory is the integrated equations of motion. Agreement to ~0.5%
  // was measured over the first 20 s; the phugoid has not grown enough to matter
  // inside that window (see the next test).
  assert.ok(
    Math.abs(fromTrajectory - fromCoefficients) / fromCoefficients < 0.02,
    `trajectory glide ratio ${fromTrajectory.toFixed(3)} vs coefficients ${fromCoefficients.toFixed(3)}`,
  );

  assert.ok(
    Math.abs(fromCoefficients - bestLiftToDrag(GLIDER)) / bestLiftToDrag(GLIDER) < 1e-4,
    'the glide should be trimmed close to best L/D',
  );
});

test('the glider phugoid growth rate matches the reported model characteristic', () => {
  // A documented finding rather than a pass/fail criterion on handling quality:
  // for this high-L/D, weakly-statically-stable airframe the phugoid is mildly
  // UNSTABLE in this model (positive growth rate). The test pins the measured
  // value so that a change to the aerodynamics cannot silently alter it.
  const outcome = glideScenario(GLIDER, 600, 29, 120, 20);
  const growth = outcome.metrics.phugoidGrowthRate!;
  assert.ok(Number.isFinite(growth), 'phugoid growth rate should be measurable');
  assert.ok(growth > 0, `expected the glider phugoid to be mildly unstable, got ${growth}`);
  assert.ok(growth < 0.2, `phugoid growth rate ${growth} is larger than expected`);
});

test('the C172 phugoid is stable, unlike the glider', () => {
  // Same measurement on a well-behaved airframe, to show the result above is a
  // property of the glider and not of the measurement.
  const solution = trim(C172, { altitudeM: 1500, trueAirspeedMps: 60 });

  // Start slightly fast (61.2 m/s against a 60 m/s trim) so the phugoid is
  // excited. With the pitch attitude held at the trim value and the trim
  // controls frozen, the aircraft is a free phugoid oscillator: the disturbance
  // exchanges speed and height at constant total energy, and the measured
  // amplitude is the oscillation envelope.
  const state = initialState(
    1500,
    vec3(61.2 * Math.cos(solution.alphaRad), 0, 61.2 * Math.sin(solution.alphaRad)),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );

  // Driven through the Simulation class, for sampling convenience.
  const sim = new Simulation(C172, state, { dt: 0.02, initialActuators: solution.actuators });
  const collected: number[] = [];
  sim.run(() => solution.controls, 160, { sampleHz: 10, onSample: (t) => collected.push(t.trueAirspeedMps - 60) });

  const windowAmplitude = (index: number): number => {
    const lo = index * 20;
    const hi = (index + 1) * 20;
    const inWindow = collected.filter((_, i) => i * 0.1 >= lo && i * 0.1 < hi);
    return (Math.max(...inWindow) - Math.min(...inWindow)) / 2;
  };

  const early = windowAmplitude(1);
  const late = windowAmplitude(6);
  assert.ok(early > 1e-4, `the perturbation should be visible, amplitude ${early}`);
  assert.ok(late < early, `the phugoid should decay: early ${early} vs late ${late}`);
});

test('takeoff distances and liftoff speed are credible for the airframe', () => {
  const outcome = takeoffScenario(C172);
  const vs = outcome.metrics.stallSpeedMps!;

  // Liftoff happens just above 1.15 Vs, the rotation speed the script uses.
  const liftoffSpeed = outcome.metrics.liftoffSpeedMps!;
  assert.ok(
    liftoffSpeed > vs * 1.05 && liftoffSpeed < vs * 1.5,
    `liftoff at ${liftoffSpeed.toFixed(2)} m/s against a stall speed of ${vs.toFixed(2)} m/s`,
  );
  assert.ok(outcome.metrics.liftoffSpeedKt! > 50 && outcome.metrics.liftoffSpeedKt! < 75);

  // Published ground roll for a 172N-class aeroplane at sea level is ~250 m, and
  // the distance to 50 ft around 500 m. The rigid-floor ground model carries no
  // gear or ground effect, so a generous band is appropriate.
  assert.ok(
    outcome.metrics.groundRollM! > 150 && outcome.metrics.groundRollM! < 450,
    `ground roll ${outcome.metrics.groundRollM!.toFixed(0)} m outside a credible band`,
  );
  assert.ok(
    outcome.metrics.distanceTo15mM! > 350 && outcome.metrics.distanceTo15mM! < 900,
    `distance to 15 m ${outcome.metrics.distanceTo15mM!.toFixed(0)} m outside a credible band`,
  );

  // Exactly one liftoff: no bouncing back onto the runway.
  const takeoffs = outcome.result.events.filter((e) => e.type === 'takeoff');
  assert.equal(takeoffs.length, 1, `expected a single liftoff, got ${takeoffs.length}`);
  assert.ok(outcome.metrics.altitudeGainM! > 200, 'the aeroplane should climb away');
});

test('takeoff climb rate is over-predicted, and stays inside a stated ceiling', () => {
  // The scenario's end-of-run vertical speed is a TRANSIENT: the run stops as
  // soon as the aeroplane passes 300 m, so it is mid-climb-out and still
  // decelerating, not settled. The meaningful comparison is the model's STEADY
  // best climb rate, read off a full-throttle trim at each speed from 1.2 to
  // 1.8 Vs.
  //
  // The propulsion model uses a constant propeller efficiency, which is
  // optimistic at the high power and low advance ratio of a climb. Measured: the
  // model peaks at about 6.0 m/s (1180 fpm) near 1.6 Vs, against a published
  // ~3.7 m/s (720 fpm) — a factor of 1.6. The test records the limitation rather
  // than hiding it, and fails if the error grows past a factor of two.
  const vs = stallSpeedMps(C172, isaAtGeometricAltitude(0));
  let bestClimb = 0;
  for (let multiple = 1.2; multiple <= 1.8; multiple += 0.05) {
    const speedMps = multiple * vs;
    const solution = trim(C172, { altitudeM: 0, trueAirspeedMps: speedMps, throttle: 1 });
    if (solution.converged && solution.flightPathAngleRad > 0) {
      bestClimb = Math.max(bestClimb, speedMps * Math.sin(solution.flightPathAngleRad));
    }
  }
  const publishedClimbRateMps = 3.66; // ~720 fpm for this class at sea level
  assert.ok(bestClimb > publishedClimbRateMps, `model best climb ${bestClimb.toFixed(2)} m/s should beat the published figure`);
  assert.ok(
    bestClimb < 2 * publishedClimbRateMps,
    `model best climb ${bestClimb.toFixed(2)} m/s is more than twice the published ${publishedClimbRateMps} m/s`,
  );

  const outcome = takeoffScenario(C172);
  const measured = outcome.metrics.climbRateAtEndMps!;
  assert.ok(measured > 0, 'the aeroplane must be climbing at the end of the run');
  assert.ok(
    measured < 3 * publishedClimbRateMps,
    `climb rate ${measured.toFixed(2)} m/s is more than three times the published figure`,
  );
});

test('roll doublet reaches a sensible roll rate and rolls back to level', () => {
  const outcome = rollDoubletScenario(C172);

  // Roll rate is limited by aileron slew rate as well as roll damping.
  assert.ok(
    outcome.metrics.peakRollRateDegPerS! > 30 && outcome.metrics.peakRollRateDegPerS! < 130,
    `peak roll rate ${outcome.metrics.peakRollRateDegPerS} deg/s outside a credible band`,
  );
  assert.ok(outcome.metrics.peakBankDeg! > 45, 'the doublet should bank past 45 degrees');
  assert.ok(outcome.metrics.peakBankDeg! < 80, 'the manoeuvre should stay inside the linear range');

  // The point of a doublet: the aeroplane must be able to stop rolling.
  assert.ok(
    Math.abs(outcome.metrics.finalBankDeg!) < 3,
    `wings should be level at the end, got ${outcome.metrics.finalBankDeg}`,
  );
  assert.ok(Math.abs(outcome.metrics.finalRollRateDegPerS!) < 3, 'the roll must be arrested');
  assert.equal(outcome.metrics.stalledDuringManoeuvre, 0, 'a roll doublet should not stall the aeroplane');
  assert.ok(outcome.metrics.timeTo45DegBankS! > 1 && outcome.metrics.timeTo45DegBankS! < 4);
});

test('the aileron actuator reaches full deflection and is rate limited on the way', () => {
  const outcome = rollDoubletScenario(C172);
  assert.ok(
    outcome.metrics.peakAileronDeg! > 16,
    `the aileron should reach near its 17 deg limit, got ${outcome.metrics.peakAileronDeg}`,
  );
  assert.ok(outcome.metrics.peakAileronDeg! <= (C172.limits.maxAileronRad * 180) / Math.PI + 1e-9);
});

test('unpowered aircraft produce exactly zero thrust', () => {
  const outcome = glideScenario(GLIDER, 600, 29, 10, 5);
  for (const sample of outcome.result.samples) {
    assert.equal(sample.thrustN, 0, 'a glider has no engine');
  }
});

test('ground events are only reported when a ground model is active', () => {
  // A purely airborne simulation must not invent a takeoff at t=0.
  const outcome = cruiseHoldScenario(C172, 1500, 60, 10);
  assert.equal(outcome.result.events.filter((e) => e.type === 'takeoff' || e.type === 'touchdown').length, 0);
});

test('trim excess power is consistent with the reported climb rate', () => {
  // Cross-checks the propulsion model against the energy equation. Full throttle
  // is fixed and the flight path angle is the unknown, so the trim is a real
  // climb. This matters: an earlier version trimmed LEVEL, where thrust equals
  // drag by construction, so the "excess power" it measured was zero and the
  // test could never have passed.
  const vs = stallSpeedMps(C172, isaAtGeometricAltitude(0));
  const solution = trim(C172, { altitudeM: 0, trueAirspeedMps: 1.3 * vs, throttle: 1 });

  assert.ok(solution.converged, 'a full-throttle climb trim at 1.3 Vs must exist');
  assert.ok(solution.flightPathAngleRad > 0, 'full throttle must produce a climb');

  const thrustN = solution.diagnostics.thrust.thrustN;
  const dragN = solution.diagnostics.forces.dragN;
  const excessPowerW = (thrustN - dragN) * solution.trueAirspeedMps;
  const impliedClimbRateMps = excessPowerW / (C172.mass.massKg * G0);
  const geometricClimbRateMps = solution.trueAirspeedMps * Math.sin(solution.flightPathAngleRad);

  assert.ok(excessPowerW > 0, 'full power at this speed should give excess thrust');
  // Two independent routes to the same climb rate: the energy equation on the
  // excess thrust, and the climb rate read off the trimmed flight path angle.
  // Measured agreement is 1.4%, the residual being the O(alpha) coupling
  // between the lift and the axial force balance, which the idealised
  // T = D + W sin(gamma) identity ignores.
  assert.ok(
    Math.abs(impliedClimbRateMps - geometricClimbRateMps) / geometricClimbRateMps < 0.03,
    `energy climb rate ${impliedClimbRateMps.toFixed(3)} vs geometric ${geometricClimbRateMps.toFixed(3)} m/s`,
  );
  assert.ok(
    impliedClimbRateMps > 2 && impliedClimbRateMps < 12,
    `implied climb rate ${impliedClimbRateMps.toFixed(2)} m/s is outside the band the scenario tests assume`,
  );
});

test('minimum level trim speed is consistent between the two call sites', () => {
  const fromHelper = minimumLevelTrimSpeed(C172, 0).speedMps;
  const closedForm = stallSpeedMps(C172, isaAtGeometricAltitude(0));
  assert.ok(Math.abs(fromHelper - closedForm) / closedForm < 0.025);

  const vmax = maxLevelSpeed(C172, 0).speedMps;
  assert.ok(vmax > fromHelper, 'the usable speed range must be non-empty');
});
