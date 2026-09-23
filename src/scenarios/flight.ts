/**
 * Scripted flight scenarios.
 *
 * Each scenario builds an initial condition, runs a control script, and reduces
 * the resulting run to a small set of measured metrics. The metrics are the
 * point: they are what the test suite asserts on and what the CLI prints. A
 * scenario that only "runs without throwing" is not evidence of anything.
 */

import { type AircraftSpec, stallSpeedMps } from '../model/aircraft.ts';
import { C172, GLIDER } from '../model/presets.ts';
import { controls, type ActuatorState, type ControlInputs } from '../model/controls.ts';
import { isaAtGeometricAltitude } from '../atmos/isa.ts';
import { initialState, type AircraftState } from '../dynamics/eom.ts';
import { Simulation, type FlightEvent, type RunResult } from '../sim/simulation.ts';
import { type Telemetry } from '../sim/telemetry.ts';
import { trim, type TrimSolution } from '../sim/trim.ts';
import { vec3 } from '../math/vec3.ts';
import { altitudeHoldElevator, mergeControls, pitchHoldElevator } from './autopilot.ts';

const RAD_TO_DEG = 180 / Math.PI;
const KT = 1.943844;

export interface ScenarioOutcome {
  readonly name: string;
  readonly description: string;
  readonly spec: AircraftSpec;
  readonly result: RunResult;
  readonly metrics: Readonly<Record<string, number>>;
  readonly notes: readonly string[];
}

const stateFromTrim = (solution: TrimSolution, altitudeM: number): AircraftState =>
  initialState(
    altitudeM,
    vec3(
      solution.trueAirspeedMps * Math.cos(solution.alphaRad),
      0,
      solution.trueAirspeedMps * Math.sin(solution.alphaRad),
    ),
    { phi: 0, theta: solution.thetaRad, psi: 0 },
  );

const actuatorsFromTrim = (solution: TrimSolution): ActuatorState => solution.actuators;

/** Max value of a numeric field over the samples. */
function maxOf(samples: readonly Telemetry[], pick: (t: Telemetry) => number): number {
  let max = Number.NEGATIVE_INFINITY;
  for (const s of samples) max = Math.max(max, pick(s));
  return max;
}

function minOf(samples: readonly Telemetry[], pick: (t: Telemetry) => number): number {
  let min = Number.POSITIVE_INFINITY;
  for (const s of samples) min = Math.min(min, pick(s));
  return min;
}

/**
 * Takeoff and climb.
 *
 * Phases, as a pilot would fly them: full power from a standing start, hold the
 * aeroplane level on the runway, rotate at 1.15 Vs, then lower the nose to hold
 * best-rate-of-climb speed once airborne.
 *
 * Ground roll and liftoff distance come from the rigid-floor ground model, so
 * they carry that model's caveats (no gear, no ground effect, no slipstream).
 */
export function takeoffScenario(spec: AircraftSpec = C172, groundAltitudeM = 0): ScenarioOutcome {
  const atmosphere = isaAtGeometricAltitude(groundAltitudeM);
  const vs = stallSpeedMps(spec, atmosphere);
  const rotateSpeedMps = 1.15 * vs;
  // Best rate of climb speed. 1.5 x Vs is the ratio a light single actually
  // flies (a 172's Vy of 79 kt against a 51 kt Vs), not 1.3.
  const climbSpeedMps = 1.5 * vs;

  const sim = new Simulation(spec, initialState(groundAltitudeM, vec3(0, 0, 0), { phi: 0, theta: 0, psi: 0 }), {
    dt: 0.02,
    groundAltitudeM,
    initialActuators: { elevatorRad: 0, aileronRad: 0, rudderRad: 0, throttle: 0 },
  });

  let liftoffTime = Number.NaN;
  let liftoffSpeed = Number.NaN;
  let rotateTime = Number.NaN;
  let wasAirborne = false;

  const result = sim.run(
    (_state, timeS, telemetry) => {
      const airborne = telemetry.altitudeM > 0.5;
      if (airborne) wasAirborne = true;

      if (!wasAirborne) {
        // On the ground the elevator commands a pitch attitude directly (see
        // the ground model in Simulation). Hold the nose down for the roll,
        // then rotate at 1.15 Vs.
        if (telemetry.trueAirspeedMps < rotateSpeedMps) {
          return controls({ throttle: 1, elevator: -0.15 });
        }
        if (Number.isNaN(rotateTime)) rotateTime = timeS;
        // About half of the available elevator rotates the aeroplane to 11 deg,
        // leaving authority in hand for the transition to the climb.
        return controls({ throttle: 1, elevator: 0.55 });
      }

      // Airborne. Hold speed with pitch rather than chasing an attitude: the
      // aeroplane leaves the ground AT the rotation attitude and slightly slow,
      // so the job immediately after liftoff is to keep the wing flying and let
      // it accelerate to best-rate-of-climb speed. Commanding a nose-up attitude
      // here is what stalls an aeroplane on climb-out.
      //
      // The pitch-rate term is not decoration. Without it this law is a bare
      // proportional controller on a system whose elevator has already been
      // commanded to full nose-up for the rotation; the aeroplane over-rotates
      // to nearly 30 deg of alpha, stalls, drops back onto the runway and takes
      // off a second time. Damping on pitch rate is the pitch damper that would
      // be fitted to any aeroplane flown by a controller this simple.
      const speedError = telemetry.trueAirspeedMps - climbSpeedMps;
      const command = 0.06 * speedError - 0.025 * telemetry.pitchRateDegPerS;
      // Immediately after liftoff the aeroplane is at the rotation attitude and
      // slow. Hold a positive-climb attitude briefly so the wheels clear before
      // the speed law takes over.
      if (telemetry.altitudeM < 25) {
        return controls({
          throttle: 1,
          elevator: clampUnit(pitchHoldElevator(telemetry, (8 * Math.PI) / 180, 0.12, 0.03) + command * 0.5),
        });
      }
      return controls({ throttle: 1, elevator: clampUnit(command) });
    },
    240,
    {
      sampleHz: 20,
      stopWhen: (telemetry, timeS) => {
        if (telemetry.altitudeM > 15 && Number.isNaN(liftoffTime)) {
          liftoffTime = timeS;
          liftoffSpeed = telemetry.trueAirspeedMps;
        }
        return telemetry.altitudeM > 300;
      },
    },
  );

  const takeoffEvent = result.events.find((e) => e.type === 'takeoff');
  const groundRollM = takeoffEvent ? horizontalDistanceAt(result, takeoffEvent.timeS) : Number.NaN;
  // Speed at the moment of liftoff. Note this is NOT the same as the speed when
  // the aeroplane passes 15 m: it accelerates in the initial climb, so anything
  // measured higher up reads several m/s fast.
  const speedAtTakeoffMps = takeoffEvent
    ? sampleAt(result, takeoffEvent.timeS).trueAirspeedMps
    : Number.NaN;

  return {
    name: 'takeoff-climb',
    description: 'Standing start, full power, rotate at 1.15 Vs, climb at 1.5 Vs.',
    spec,
    result,
    metrics: {
      stallSpeedMps: vs,
      rotateSpeedMps,
      rotateTimeS: rotateTime,
      liftoffSpeedMps: speedAtTakeoffMps,
      liftoffSpeedKt: speedAtTakeoffMps * KT,
      speedAt15mMps: liftoffSpeed,
      takeoffTimeS: takeoffEvent ? takeoffEvent.timeS : Number.NaN,
      groundRollM,
      distanceTo15mM: horizontalDistanceAt(result, liftoffTime),
      altitudeGainM: result.finalTelemetry.altitudeM - groundAltitudeM,
      climbRateAtEndMps: result.finalTelemetry.verticalSpeedMps,
      peakClimbRateMps: maxOf(result.samples, (t) => t.verticalSpeedMps),
    },
    notes: [
      'Ground model is a rigid floor: no landing gear, no ground effect, no propeller slipstream.',
      'Ground roll and liftoff distance are therefore indicative rather than POH-equivalent.',
      'CLIMB RATE IS OVER-PREDICTED. The model\'s steady best climb is about 6.1 m/s (1200 fpm) at 1.57 Vs, against a published ~3.7 m/s (720 fpm) for this class — a factor of about 1.6. The propulsion model uses a constant 0.8 propeller efficiency, but a real fixed-pitch propeller is far less efficient at the low airspeed and high power of a climb, so thrust at Vy is too high; over the whole climb speed range the thrust is additionally pinned at the 2400 N static cap. Correcting this needs efficiency as a function of advance ratio; until then treat climb rate as optimistic. Ground roll, liftoff speed and distance to 15 m are not affected by that limitation and land close to published figures.',
      'Airspeed at the 15 m point (speedAt15mMps) is not the liftoff speed: the aeroplane accelerates during the initial climb, so it reads several m/s fast.',
    ],
  };
}

/** Horizontal distance travelled between two times, from the sampled ground speed. */
function horizontalDistanceBetween(result: RunResult, startS: number, endS: number): number {
  let distance = 0;
  for (let i = 1; i < result.samples.length; i++) {
    const previous = result.samples[i - 1]!;
    const current = result.samples[i]!;
    if (previous.timeS < startS) continue;
    if (current.timeS > endS) break;
    distance += 0.5 * (previous.groundSpeedMps + current.groundSpeedMps) * (current.timeS - previous.timeS);
  }
  return distance;
}

/** Horizontal distance travelled by a given time. */
function horizontalDistanceAt(result: RunResult, timeS: number): number {
  return horizontalDistanceBetween(result, 0, timeS);
}

/** Telemetry sample nearest to a given time. */
function sampleAt(result: RunResult, timeS: number): Telemetry {
  let best = result.samples[0]!;
  for (const sample of result.samples) {
    if (Math.abs(sample.timeS - timeS) < Math.abs(best.timeS - timeS)) best = sample;
  }
  return best;
}

const clampUnit = (v: number): number => (v < -1 ? -1 : v > 1 ? 1 : v);

/**
 * Trimmed cruise hold.
 *
 * No controller: the aircraft flies on the constant controls the trim solver
 * produced. Any drift is therefore drift in the TRIM, not in a controller, which
 * makes this the direct test of whether the trim point is genuinely an
 * equilibrium of the same equations the simulator integrates.
 */
export function cruiseHoldScenario(
  spec: AircraftSpec = C172,
  altitudeM = 1500,
  airspeedMps = 60,
  durationS = 120,
): ScenarioOutcome {
  const solution = trim(spec, { altitudeM, trueAirspeedMps: airspeedMps });
  const sim = new Simulation(spec, stateFromTrim(solution, altitudeM), {
    dt: 0.02,
    initialActuators: actuatorsFromTrim(solution),
  });
  const held = solution.controls;
  const result = sim.run(() => held, durationS, { sampleHz: 5 });

  const final = result.finalTelemetry;
  return {
    name: 'cruise-hold',
    description: `Trimmed at ${altitudeM} m and ${airspeedMps} m/s, then held on fixed controls.`,
    spec,
    result,
    metrics: {
      trimResidualNorm: solution.residualNorm,
      trimAlphaDeg: solution.alphaRad * RAD_TO_DEG,
      trimThrottle: solution.controls.throttle,
      altitudeDriftM: Math.abs(final.altitudeM - altitudeM),
      peakAltitudeDeviationM: Math.max(
        Math.abs(maxOf(result.samples, (t) => t.altitudeM) - altitudeM),
        Math.abs(minOf(result.samples, (t) => t.altitudeM) - altitudeM),
      ),
      airspeedDriftMps: Math.abs(final.trueAirspeedMps - airspeedMps),
      peakAirspeedDeviationMps: Math.max(
        Math.abs(maxOf(result.samples, (t) => t.trueAirspeedMps) - airspeedMps),
        Math.abs(minOf(result.samples, (t) => t.trueAirspeedMps) - airspeedMps),
      ),
      liftToDrag: solution.liftToDrag,
    },
    notes: ['Drift here is trim error, not controller error: no autopilot is engaged.'],
  };
}

/**
 * Stall and recovery.
 *
 * The aeroplane is trimmed in level flight, power is pulled to idle, and the
 * elevator is used to hold altitude. Airspeed falls until the wing can no longer
 * support the weight, at which point separation climbs through 0.5 and the
 * altitude hold loses the argument.
 *
 * Then a recovery phase: power up, release the back pressure, let the nose drop
 * until the wing reattaches, and only then raise the nose to arrest the
 * descent. Holding the nose up into a developed stall is how a stall becomes an
 * accident, and it is also what a naive script does — airspeed falls, the
 * altitude error grows, the elevator saturates nose-up, and the aeroplane stays
 * stalled for the rest of the run.
 *
 * Note what "measured stall speed" means here. The onset criterion is the lift
 * curve's separation fraction passing 0.5, which happens at roughly the midpoint
 * of the stall break — about 3.4 deg beyond the linear-curve stall incidence —
 * by which point the aeroplane is already descending and the load factor is
 * below 1. The measured value therefore sits a few percent BELOW the 1-g
 * closed-form Vs, which is the correct and expected result rather than an error.
 * The tight check against the closed form is done separately, on the minimum
 * speed at which a 1-g level trim exists at all.
 */
export function stallScenario(
  spec: AircraftSpec = C172,
  altitudeM = 1500,
  entryAirspeedMps = 62,
  durationS = 150,
): ScenarioOutcome {
  const solution = trim(spec, { altitudeM, trueAirspeedMps: entryAirspeedMps });
  const sim = new Simulation(spec, stateFromTrim(solution, altitudeM), {
    dt: 0.02,
    initialActuators: actuatorsFromTrim(solution),
  });

  const vs = stallSpeedMps(spec, isaAtGeometricAltitude(altitudeM));
  // Speeds for the three phases of the recovery, as multiples of 1-g stall speed:
  // break the stall, fly again, then climb away.
  const breakSpeedMps = 1.15 * vs;
  const recoverySpeedMps = 1.3 * vs;
  const targetRecoverySpeedMps = 1.35 * vs;

  const onset = { timeS: Number.NaN, speedMps: Number.NaN, alphaDeg: Number.NaN };
  let recoveryTime = Number.NaN;
  let phase: 'decelerate' | 'stalled' | 'recover' | 'climb' | 'done' = 'decelerate';
  let lowestAltitudeM = Number.POSITIVE_INFINITY;

  const result = sim.run(
    (_state, _timeS, telemetry) => {
      lowestAltitudeM = Math.min(lowestAltitudeM, telemetry.altitudeM);

      if (phase === 'decelerate') {
        // Power off, hold altitude with the elevator: the aeroplane decelerates
        // and alpha rises until the wing gives up.
        return controls({ throttle: 0, elevator: altitudeHoldElevator(telemetry, altitudeM) });
      }
      if (phase === 'stalled') {
        // Break the stall: unload the wing and add power.
        //
        // The transition is on airspeed ALONE. An earlier version also required
        // alpha below 8 deg, which never fired: alpha recovers within a second
        // but airspeed lags, so by the time the speed condition was met the
        // alpha condition had been violated again during the next oscillation.
        if (telemetry.trueAirspeedMps > breakSpeedMps) phase = 'recover';
        return controls({
          throttle: 1,
          elevator: -0.5,
          aileron: clampUnit(-0.02 * telemetry.rollDeg),
        });
      }
      if (phase === 'recover') {
        // Regain flying speed by holding it with PITCH, not by chasing altitude.
        //
        // This is the subtle part. The obvious recovery law — altitude hold —
        // is actively wrong here: the aeroplane is a few hundred metres low, so
        // the controller saturates nose-up, drives alpha straight back past the
        // stall, and the aeroplane settles into a stable deep-stall descent with
        // the elevator against the stop. Holding airspeed with pitch instead
        // keeps alpha low until the wing has reattached and the speed is back.
        if (telemetry.trueAirspeedMps > targetRecoverySpeedMps) phase = 'climb';
        const speedError = telemetry.trueAirspeedMps - recoverySpeedMps;
        return controls({
          throttle: 1,
          elevator: clampUnit(0.06 * speedError),
          aileron: clampUnit(-0.02 * telemetry.rollDeg),
        });
      }
      if (phase === 'climb') {
        if (!Number.isNaN(onset.timeS) && Number.isNaN(recoveryTime) && telemetry.verticalSpeedMps > 0.5) {
          recoveryTime = telemetry.timeS;
          phase = 'done';
        }
        return controls({ throttle: 1, elevator: pitchHoldElevator(telemetry, (6 * Math.PI) / 180) });
      }
      return controls({ throttle: 0.75, elevator: altitudeHoldElevator(telemetry, altitudeM) });
    },
    durationS,
    {
      sampleHz: 20,
      stopWhen: (telemetry, timeS) => {
        if (telemetry.stalled && phase === 'decelerate') {
          phase = 'stalled';
          onset.timeS = timeS;
          onset.speedMps = telemetry.trueAirspeedMps;
          onset.alphaDeg = telemetry.alphaDeg;
        }
        return phase === 'done';
      },
    },
  );

  const theoretical = stallSpeedMps(spec, isaAtGeometricAltitude(altitudeM));
  return {
    name: 'stall',
    description: 'Power to idle, hold altitude with elevator until the wing stalls, then recover.',
    spec,
    result,
    metrics: {
      theoreticalStallSpeedMps: theoretical,
      measuredStallSpeedMps: onset.speedMps,
      stallSpeedErrorPercent: ((onset.speedMps - theoretical) / theoretical) * 100,
      stallSpeedKt: onset.speedMps * KT,
      stallAlphaDeg: onset.alphaDeg,
      timeToStallS: onset.timeS,
      recoveryTimeS: recoveryTime,
      recoveryDurationS: recoveryTime - onset.timeS,
      stallRecoverySpeedMps: recoverySpeedMps,
      recovered: Number.isFinite(recoveryTime) ? 1 : 0,
      altitudeLostInStallM: altitudeM - lowestAltitudeM,
      peakAlphaDeg: maxOf(result.samples, (t) => t.alphaDeg),
      minAirspeedMps: minOf(result.samples, (t) => t.trueAirspeedMps),
      finalAltitudeM: result.finalTelemetry.altitudeM,
    },
    notes: [
      'Stall onset is defined as separation fraction crossing 0.5, the midpoint of the lift-curve blend.',
      'Because the aeroplane is already descending at onset, the measured stall speed sits a few percent below the 1-g closed-form Vs. That is expected.',
      'The model has no stall hysteresis, so recovery is symmetric with entry.',
    ],
  };
}

/**
 * Glide performance.
 *
 * Two independent measurements of the same physical quantity: the lift-to-drag
 * ratio read off the trimmed coefficients, and the glide ratio measured from the
 * trajectory (horizontal distance flown per metre of altitude lost). They should
 * agree, and the test suite asserts that they do — it is a genuine cross-check,
 * because one comes from the aerodynamic model and the other from integrating
 * the equations of motion.
 *
 * The trajectory number is taken over a WINDOW (`measurementWindowS`) rather
 * than the whole run, and that is not a fudge. As the aeroplane descends the
 * density rises, which shifts the equilibrium slightly and continuously excites
 * the phugoid. For the C172 that mode is well damped and dies out. For the
 * glider — L/D 44.6 with a weak static margin — it is very lightly damped and
 * in this model mildly UNSTABLE (measured lambda is about +0.06 /s), so over a
 * long run the glide ratio wanders away from the steady-state value. That is a
 * real property of the airframe model and is reported rather than hidden; see
 * `longRunGlideRatio` and `phugoidGrowthRate`.
 */
export function glideScenario(
  spec: AircraftSpec = GLIDER,
  altitudeM = 600,
  airspeedMps = 29,
  durationS = 120,
  measurementWindowS = 20,
): ScenarioOutcome {
  const solution = trim(spec, { altitudeM, trueAirspeedMps: airspeedMps, throttle: 0 });
  const sim = new Simulation(spec, stateFromTrim(solution, altitudeM), {
    dt: 0.02,
    initialActuators: actuatorsFromTrim(solution),
  });

  const held: ControlInputs = solution.controls;
  const result = sim.run(() => held, durationS, { sampleHz: 5 });

  const windowStart = sampleAt(result, 0);
  const windowEnd = sampleAt(result, measurementWindowS);
  const windowDistance = horizontalDistanceBetween(result, windowStart.timeS, windowEnd.timeS);
  const windowAltitudeLost = windowStart.altitudeM - windowEnd.altitudeM;

  const first = result.samples[0]!;
  const last = result.samples[result.samples.length - 1]!;
  const totalDistance = horizontalDistanceAt(result, last.timeS);
  const totalAltitudeLost = first.altitudeM - last.altitudeM;

  // Instantaneous glide slope at the trim point: for a trimmed glide, tan|gamma|
  // = D/L exactly, which is the cleanest available check on the trim itself.
  const instantaneousRatio =
    Math.abs(Math.tan(solution.flightPathAngleRad)) > 1e-12
      ? 1 / Math.abs(Math.tan(solution.flightPathAngleRad))
      : Number.POSITIVE_INFINITY;

  // Phugoid growth rate, fitted to the envelope of the airspeed oscillation.
  const phugoidGrowthRate = fitExponentialGrowthRate(result, airspeedMps);

  return {
    name: 'glide',
    description: `Unpowered glide from ${altitudeM} m at ${airspeedMps} m/s.`,
    spec,
    result,
    metrics: {
      trimFlightPathAngleDeg: solution.flightPathAngleRad * RAD_TO_DEG,
      liftToDragFromCoefficients: solution.liftToDrag,
      glideRatioFromTrimAngle: instantaneousRatio,
      glideRatioOverWindow: windowAltitudeLost > 1e-6 ? windowDistance / windowAltitudeLost : Number.NaN,
      longRunGlideRatio: totalAltitudeLost > 1e-6 ? totalDistance / totalAltitudeLost : Number.NaN,
      phugoidGrowthRate,
      sinkRateMps: -last.verticalSpeedMps,
      initialSinkRateMps: setupInitialSinkRate(solution),
      horizontalDistanceM: totalDistance,
      altitudeLostM: totalAltitudeLost,
      trimResidualNorm: solution.residualNorm,
    },
    notes: [
      'glideRatioOverWindow is measured over the first ' +
        measurementWindowS +
        ' s; longRunGlideRatio covers the whole run and includes phugoid divergence.',
      'phugoidGrowthRate > 0 means the airspeed oscillation is growing, i.e. an unstable phugoid.',
    ],
  };
}

const setupInitialSinkRate = (solution: TrimSolution): number =>
  solution.trueAirspeedMps * Math.sin(-solution.flightPathAngleRad);

/** Least-squares fit of ln(envelope amplitude) against time, in 10 s windows from 10 s onward. */
function fitExponentialGrowthRate(result: RunResult, referenceSpeed: number): number {
  const amplitudes: number[] = [];
  for (let window = 0; window < 12; window++) {
    const lo = (window + 1) * 10;
    const hi = (window + 2) * 10;
    const inWindow = result.samples.filter((s) => s.timeS >= lo && s.timeS < hi);
    if (inWindow.length < 3) break;
    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    for (const s of inWindow) {
      const deviation = s.trueAirspeedMps - referenceSpeed;
      min = Math.min(min, deviation);
      max = Math.max(max, deviation);
    }
    amplitudes.push((max - min) / 2);
  }
  const usable = amplitudes.filter((a) => a > 1e-9);
  if (usable.length < 3) return Number.NaN;
  const xs = usable.map((_, i) => (i + 1.5) * 10);
  const ys = usable.map((a) => Math.log(a));
  const n = xs.length;
  const meanX = xs.reduce((a, b) => a + b, 0) / n;
  const meanY = ys.reduce((a, b) => a + b, 0) / n;
  const numerator = xs.reduce((acc, x, i) => acc + (x - meanX) * (ys[i]! - meanY), 0);
  const denominator = xs.reduce((acc, x) => acc + (x - meanX) ** 2, 0);
  return denominator > 0 ? numerator / denominator : Number.NaN;
}

/**
 * Roll doublet.
 *
 * Aileron one way, then the other, then neutral. Measures roll performance and,
 * more importantly, whether the aeroplane can STOP rolling — roll damping is
 * what makes a doublet recoverable.
 *
 * Pulse length matters. At 1.5 s per pulse with full aileron the aeroplane banks
 * past 80 deg, the lift vector tilts, alpha climbs past the stall and the run
 * stops being a roll test and becomes a departure. 0.6 s pulses keep the bank
 * inside the linear range, where the roll rate and the damping can actually be
 * read off the response.
 *
 * The final phase levels the wings with an aileron law on bank angle and roll
 * rate, and holds altitude with the elevator, so the run ends in controlled
 * flight rather than wherever it happened to stop.
 */
export function rollDoubletScenario(spec: AircraftSpec = C172, altitudeM = 1500, airspeedMps = 62): ScenarioOutcome {
  const solution = trim(spec, { altitudeM, trueAirspeedMps: airspeedMps });
  const sim = new Simulation(spec, stateFromTrim(solution, altitudeM), {
    dt: 0.02,
    initialActuators: actuatorsFromTrim(solution),
  });
  const held = solution.controls;

  const result = sim.run(
    (_state, timeS, telemetry) => {
      const elevator = altitudeHoldElevator(telemetry, altitudeM, { kp: 0.0025, kd: 0.035, kSpeed: 0.05 });
      if (timeS < 1.0) return mergeControls(held, { aileron: 0, elevator });
      if (timeS < 1.8) return mergeControls(held, { aileron: 1, elevator });
      if (timeS < 2.6) return mergeControls(held, { aileron: -1, elevator });
      if (timeS < 3.4) return mergeControls(held, { aileron: 1, elevator });
      // Level the wings: bank right needs left aileron, and damp on roll rate.
      const leveler = clampUnit(-0.02 * telemetry.rollDeg - 0.01 * telemetry.rollRateDegPerS);
      return mergeControls(held, { aileron: leveler, elevator });
    },
    14,
    { sampleHz: 50 },
  );

  let peakRollRateDegPerS = 0;
  let peakBankDeg = 0;
  let maxAlphaDeg = 0;
  for (const sample of result.samples) {
    peakRollRateDegPerS = Math.max(peakRollRateDegPerS, Math.abs(sample.rollRateDegPerS));
    peakBankDeg = Math.max(peakBankDeg, Math.abs(sample.rollDeg));
    maxAlphaDeg = Math.max(maxAlphaDeg, sample.alphaDeg);
  }
  const stalledDuringManoeuvre = result.events.some((e) => e.type === 'stall-onset');

  return {
    name: 'roll-doublet',
    description: 'Aileron doublet with 0.8 s pulses, then roll levelling.',
    spec,
    result,
    metrics: {
      peakRollRateDegPerS,
      peakBankDeg,
      finalBankDeg: result.finalTelemetry.rollDeg,
      finalRollRateDegPerS: result.finalTelemetry.rollRateDegPerS,
      timeTo45DegBankS: timeToBank(result, 45),
      altitudeLossM: altitudeM - minOf(result.samples, (t) => t.altitudeM),
      maxAlphaDeg,
      stalledDuringManoeuvre: stalledDuringManoeuvre ? 1 : 0,
      peakAileronDeg: maxOf(result.samples, (t) => Math.abs(t.aileronDeg)),
    },
    notes: ['Peak roll rate is limited by aileron slew rate as well as by roll damping.'],
  };
}

/** First time the bank angle magnitude reaches a target, or NaN. */
function timeToBank(result: RunResult, targetDeg: number): number {
  for (const sample of result.samples) {
    if (Math.abs(sample.rollDeg) >= targetDeg) return sample.timeS;
  }
  return Number.NaN;
}

/**
 * Minimum speed at which a trimmed flight condition exists.
 *
 * This is the honest operational definition of stall speed for this model, and
 * the one that should be compared against the closed form
 * Vs = sqrt(2W / (rho S CLmax)). It is a stronger test than watching a scripted
 * deceleration, because the answer is produced by the same solver that flies
 * every scenario.
 *
 * With no `throttle` argument the trim is LEVEL (gamma fixed at zero) and the
 * function returns the minimum speed for powered level flight. That condition
 * does not exist for an unpowered aircraft at any speed — level flight needs
 * thrust equal to the drag, and a glider has none — so a glider must be searched
 * with `throttle = 0`, which finds the minimum speed of a trimmed GLIDE instead.
 *
 * The bracket needs care. Unlike maximum level speed, feasibility here is not a
 * half-line: for powered flight it is the INTERVAL [Vs, Vmax], because below Vs
 * there is not enough lift and above Vmax not enough thrust. A naive bisection
 * between a low and a high guess therefore converges to whichever boundary lies
 * between them — bracketing 12 m/s against 80 m/s finds Vmax, not Vs. So the
 * feasible anchor is located first by expanding upward, then the lower bound is
 * found by shrinking downward until the solver fails, and only then is the
 * interval bisected.
 */
export function minimumLevelTrimSpeed(
  spec: AircraftSpec = C172,
  altitudeM = 0,
  toleranceMps = 0.02,
  throttle?: number,
): { speedMps: number; searchSteps: number } {
  const requestFor = (speedMps: number) =>
    throttle === undefined
      ? { altitudeM, trueAirspeedMps: speedMps }
      : { altitudeM, trueAirspeedMps: speedMps, throttle };

  const feasibleAt = (speedMps: number): boolean => {
    const solution = trim(spec, requestFor(speedMps));
    // A converged trim that sits past the stall break is not a valid flight
    // condition, even though the equations have a solution there.
    return solution.converged && !solution.stalled;
  };

  const atmosphere = isaAtGeometricAltitude(altitudeM);
  let steps = 0;

  // Locate a feasible anchor.
  let high = stallSpeedMps(spec, atmosphere) * 1.25;
  while (!feasibleAt(high) && steps < 40) {
    high *= 1.12;
    steps += 1;
  }
  if (!feasibleAt(high)) return { speedMps: Number.NaN, searchSteps: steps };

  // Shrink until the solver fails, giving an infeasible lower bound.
  let low = high * 0.6;
  while (feasibleAt(low) && steps < 40) {
    high = low;
    low *= 0.8;
    steps += 1;
  }

  while (high - low > toleranceMps && steps < 80) {
    const mid = 0.5 * (low + high);
    if (feasibleAt(mid)) high = mid;
    else low = mid;
    steps += 1;
  }
  return { speedMps: high, searchSteps: steps };
}

/**
 * Envelope sweep: trim the whole usable speed range at several altitudes and
 * report where trim stops converging. The boundary is physical — the speed at
 * which full throttle can no longer hold level flight — so this doubles as a
 * max-level-speed calculation.
 */
export interface EnvelopeRow {
  readonly altitudeM: number;
  readonly airspeedMps: number;
  readonly trim: TrimSolution;
}

export function envelopeSweep(
  spec: AircraftSpec = C172,
  altitudes: readonly number[] = [0, 1500, 3000],
  speeds: readonly number[] = [32, 40, 48, 56, 62, 68, 74, 80],
  throttle?: number,
): readonly EnvelopeRow[] {
  const rows: EnvelopeRow[] = [];
  for (const altitudeM of altitudes) {
    for (const airspeedMps of speeds) {
      const request =
        throttle === undefined
          ? { altitudeM, trueAirspeedMps: airspeedMps }
          : { altitudeM, trueAirspeedMps: airspeedMps, throttle };
      rows.push({ altitudeM, airspeedMps, trim: trim(spec, request) });
    }
  }
  return rows;
}

export interface MaxLevelSpeedResult {
  readonly altitudeM: number;
  readonly speedMps: number;
  readonly searchSteps: number;
}

/**
 * Maximum level speed by bisection on throttle saturation.
 *
 * Level trim is feasible exactly when the required thrust is at or below what
 * full throttle produces. Rather than solving that analytically against a
 * separate thrust equation, this bisects on the trim solver's own convergence
 * flag, so the answer is defined by the same code that flies the aircraft.
 *
 * A feasible anchor is located first, and the search returns NaN when none
 * exists. That case is not hypothetical: an unpowered airframe has no level trim
 * at ANY speed, because level flight requires thrust equal to the drag. An
 * earlier version of this function assumed 30 m/s was always feasible and simply
 * returned it when the bisection never moved — so the glider reported a maximum
 * level speed of 30 m/s, which was its own initial guess dressed up as a result.
 */
export function maxLevelSpeed(spec: AircraftSpec = C172, altitudeM = 0, toleranceMps = 0.05): MaxLevelSpeedResult {
  const feasible = (speedMps: number): boolean =>
    trim(spec, { altitudeM, trueAirspeedMps: speedMps }).converged;

  let steps = 0;
  let low = 30; // feasible for the powered preset
  if (!feasible(low)) {
    let probe = 34;
    while (probe <= 200 && !feasible(probe) && steps < 60) {
      probe *= 1.1;
      steps += 1;
    }
    if (!feasible(probe)) return { altitudeM, speedMps: Number.NaN, searchSteps: steps };
    low = probe;
  }

  let high = low * 2;
  while (feasible(high) && steps < 60) {
    low = high;
    high *= 1.5;
    steps += 1;
  }

  while (high - low > toleranceMps && steps < 140) {
    const mid = 0.5 * (low + high);
    if (feasible(mid)) low = mid;
    else high = mid;
    steps += 1;
  }
  return { altitudeM, speedMps: low, searchSteps: steps };
}

export const SCENARIO_NAMES = ['takeoff', 'cruise', 'stall', 'glide', 'roll'] as const;
export type ScenarioName = (typeof SCENARIO_NAMES)[number];

export function runScenario(name: ScenarioName): ScenarioOutcome {
  switch (name) {
    case 'takeoff':
      return takeoffScenario();
    case 'cruise':
      return cruiseHoldScenario();
    case 'stall':
      return stallScenario();
    case 'glide':
      return glideScenario();
    case 'roll':
      return rollDoubletScenario();
    default: {
      const exhaustive: never = name;
      throw new Error(`unknown scenario ${String(exhaustive)}`);
    }
  }
}

export type { FlightEvent };
