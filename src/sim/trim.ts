/**
 * Steady-flight trim solver.
 *
 * A trim point is a flight condition in which the aeroplane is in equilibrium:
 * no net force and no net pitching moment. Finding one is the prerequisite for
 * every scenario in this project, and it doubles as the strongest available test
 * of the whole model, because a converged trim exercises the atmosphere, the
 * lift and drag models, the force transform, the propulsion model and the moment
 * balance simultaneously.
 *
 * Formulation. At a trim point beta = 0, p = q = r = 0, and the aircraft flies a
 * straight line at flight path angle gamma (positive = climbing). The body
 * pitch is therefore
 *
 *   theta = alpha + gamma
 *
 * and the equilibrium conditions, from the body-axis equations of motion with
 * all accelerations zero, are
 *
 *   r1 = X/m + T/m - g sin(theta)      = 0     (axial force balance)
 *   r2 = Z/m + g cos(theta)            = 0     (normal force balance)
 *   r3 = M/Iyy                         = 0     (pitch moment balance)
 *
 * Three equations. Which quantities are the unknowns is a choice, and it is the
 * choice that makes this solver cover both level flight and gliding:
 *
 *   - Fix gamma (default 0, level flight) and solve for (alpha, elevator, throttle)
 *   - Fix throttle (e.g. 0 for a glide)  and solve for (alpha, elevator, gamma)
 *
 * That is exactly one line of code different, and it means the powered and
 * unpowered cases share one code path rather than being two models that can
 * disagree.
 *
 * The residuals are not hand-written: they are read straight out of
 * `evaluateDerivative`. The body-frame accelerations it returns, with the rate
 * terms zero, ARE the three residuals. Trimming against the same function the
 * simulator integrates removes any possibility of the trim solution and the
 * simulation drifting apart, which is a failure mode that is very hard to spot
 * by inspection — the trim looks right, the simulation looks right, and the
 * aircraft slowly sinks.
 *
 * Method: damped Newton with a central-difference Jacobian and a backtracking
 * line search. The line search matters near the stall break, where a full
 * Newton step overshoots into the post-stall region, the Jacobian there points
 * somewhere unhelpful, and the iteration diverges without it.
 */

import { type AircraftSpec, stallAngleRad, weightN } from '../model/aircraft.ts';
import { commandedSurfaces, type ActuatorState, type ControlInputs } from '../model/controls.ts';
import { evaluateDerivative, type StepDiagnostics } from '../dynamics/eom.ts';
import { quatFromEuler } from '../math/quat.ts';
import { vec3 } from '../math/vec3.ts';
import { solveLinearSystem } from '../math/linsolve.ts';
import { G0, isaAtGeometricAltitude } from '../atmos/isa.ts';

export interface TrimRequest {
  /** Altitude above mean sea level (m, geometric). */
  readonly altitudeM: number;
  /** True airspeed (m/s). */
  readonly trueAirspeedMps: number;
  /** Fixed flight path angle (rad, positive = climb). Defaults to 0 (level flight). */
  readonly flightPathAngleRad?: number;
  /** Fixed throttle 0..1. When given, flight path angle becomes an unknown instead. */
  readonly throttle?: number;
  readonly initialGuess?: {
    readonly alphaRad?: number;
    readonly elevator?: number;
    readonly throttle?: number;
    readonly flightPathAngleRad?: number;
  };
  readonly maxIterations?: number;
  /** Convergence threshold on the infinity norm of the residual vector. */
  readonly tolerance?: number;
}

export interface TrimSolution {
  readonly converged: boolean;
  readonly iterations: number;
  /** Infinity norm of the final residual vector. Units are mixed (m/s^2 and rad/s^2). */
  readonly residualNorm: number;
  readonly residualVector: readonly [number, number, number];
  readonly altitudeM: number;
  readonly trueAirspeedMps: number;
  /** Angle of attack (rad). */
  readonly alphaRad: number;
  /** Pitch attitude (rad). */
  readonly thetaRad: number;
  /** Flight path angle (rad, positive = climb). */
  readonly flightPathAngleRad: number;
  /** Pilot controls that hold this trim. Feed straight into the simulator. */
  readonly controls: ControlInputs;
  /** Resulting surface positions. */
  readonly actuators: ActuatorState;
  /** True when the trim incidence is beyond the stall break — a physically invalid solution. */
  readonly stalled: boolean;
  readonly diagnostics: StepDiagnostics;
  readonly liftToDrag: number;
  /** Required lift coefficient. */
  readonly liftCoefficient: number;
  readonly dragCoefficient: number;
}

const clamp = (v: number, lo: number, hi: number): number => (v < lo ? lo : v > hi ? hi : v);

const ALPHA_BOUND = 1.45; // ~83 deg; beyond this the body-axis formulation is meaningless
const GAMMA_BOUND = 1.45;

interface TrimLayout {
  /** Whether gamma is fixed (true) or is unknown slot 2 (false). */
  readonly gammaFixed: boolean;
  readonly gammaRad: number;
  readonly throttleFixed: boolean;
  readonly throttle: number;
}

function unpack(layout: TrimLayout, x: Float64Array): { alphaRad: number; elevator: number; throttle: number; gammaRad: number } {
  const alphaRad = clamp(x[0]!, -ALPHA_BOUND, ALPHA_BOUND);
  const elevator = clamp(x[1]!, -1, 1);
  if (layout.gammaFixed) {
    return { alphaRad, elevator, throttle: clamp(x[2]!, 0, 1), gammaRad: layout.gammaRad };
  }
  return { alphaRad, elevator, throttle: layout.throttle, gammaRad: clamp(x[2]!, -GAMMA_BOUND, GAMMA_BOUND) };
}

/**
 * The three equilibrium residuals at a candidate (alpha, elevator, third).
 *
 * Exported for the tests, which assert on the residuals directly rather than
 * only on convergence — a solver that reports success at a residual of 0.1
 * would otherwise pass.
 */
export function trimResiduals(
  spec: AircraftSpec,
  request: TrimRequest,
  x: Float64Array,
  layout: TrimLayout,
): Float64Array {
  const { alphaRad, elevator, throttle, gammaRad } = unpack(layout, x);
  const theta = alphaRad + gammaRad;
  const V = request.trueAirspeedMps;

  // A state that is stationary in the body frame and consistent with this
  // candidate flight condition: velocity along body x rotated by alpha, attitude
  // level in roll and yaw, pitch = alpha + gamma, and no rotation.
  const state = {
    positionNed: vec3(0, 0, -request.altitudeM),
    velocityBody: vec3(V * Math.cos(alphaRad), 0, V * Math.sin(alphaRad)),
    attitude: quatFromEuler({ phi: 0, theta, psi: 0 }),
    angularRateBody: vec3(0, 0, 0),
  };
  const actuators = commandedSurfaces(spec.limits, { elevator, aileron: 0, rudder: 0, throttle });

  const { derivative } = evaluateDerivative(spec, state, actuators);

  return Float64Array.from([
    derivative.accelerationBody.x,
    derivative.accelerationBody.z,
    derivative.angularAccelerationBody.y,
  ]);
}

export function layoutFor(request: TrimRequest): TrimLayout {
  const hasGamma = request.flightPathAngleRad !== undefined;
  const hasThrottle = request.throttle !== undefined;
  if (hasGamma && hasThrottle) {
    throw new Error(
      'TrimRequest specifies both flightPathAngleRad and throttle; exactly one must be free. ' +
        'Fix gamma to solve for throttle (powered flight), or fix throttle to solve for gamma (glide).',
    );
  }
  if (hasThrottle) {
    return { gammaFixed: false, gammaRad: 0, throttleFixed: true, throttle: request.throttle! };
  }
  return { gammaFixed: true, gammaRad: request.flightPathAngleRad ?? 0, throttleFixed: false, throttle: 0 };
}

const infinityNorm = (v: Float64Array): number => {
  let max = 0;
  for (const value of v) max = Math.max(max, Math.abs(value));
  return max;
};

export function trim(spec: AircraftSpec, request: TrimRequest): TrimSolution {
  const layout = layoutFor(request);
  const maxIterations = request.maxIterations ?? 60;
  const tolerance = request.tolerance ?? 1e-9;

  const V = request.trueAirspeedMps;

  // Initial guess: the alpha that would produce the lift coefficient needed to
  // balance a fraction of the weight at this dynamic pressure. The 0.85 factor
  // leaves room for the fact that gamma and the thrust vector are not purely
  // vertical, and in a glide the required lift is below the weight.
  const guess = request.initialGuess ?? {};
  const roughLift = 0.85;
  const densityAtAltitude = isaAtGeometricAltitude(request.altitudeM).densityKgM3;

  const neededCL = (roughLift * weightN(spec)) / (0.5 * densityAtAltitude * V * V * spec.wing.areaM2);
  const alphaGuess =
    guess.alphaRad ?? clamp(neededCL / spec.lift.CLAlpha + spec.lift.alpha0Rad, -ALPHA_BOUND, ALPHA_BOUND);

  const x = Float64Array.from([
    alphaGuess,
    guess.elevator ?? 0,
    layout.gammaFixed ? (guess.throttle ?? 0.5) : (guess.flightPathAngleRad ?? -0.05),
  ]);

  let residual = trimResiduals(spec, request, x, layout);
  let norm = infinityNorm(residual);
  let iterations = 0;

  const n = 3;
  const jacobian = new Float64Array(n * n);
  const rhs = new Float64Array(n);

  while (iterations < maxIterations && norm > tolerance) {
    // Central-difference Jacobian. A relative step keeps the perturbation
    // meaningful for entries of very different magnitude (alpha ~ 0.1 rad
    // against throttle ~ 1).
    for (let col = 0; col < n; col++) {
      const h = 1e-6 * Math.max(Math.abs(x[col]!), 1);
      const xPlus = Float64Array.from(x);
      const xMinus = Float64Array.from(x);
      xPlus[col] = x[col]! + h;
      xMinus[col] = x[col]! - h;
      const rPlus = trimResiduals(spec, request, xPlus, layout);
      const rMinus = trimResiduals(spec, request, xMinus, layout);
      for (let row = 0; row < n; row++) {
        jacobian[row * n + col] = (rPlus[row]! - rMinus[row]!) / (2 * h);
      }
    }

    for (let row = 0; row < n; row++) rhs[row] = -residual[row]!;
    const step = solveLinearSystem(jacobian, rhs, n);
    if (!step) break;

    // Backtracking line search on the residual norm.
    let accepted = false;
    let scale = 1;
    for (let attempt = 0; attempt < 12; attempt++) {
      const candidate = Float64Array.from(x);
      for (let k = 0; k < n; k++) candidate[k] = clampStep(x[k]!, step[k]! * scale, k, layout);
      const candidateResidual = trimResiduals(spec, request, candidate, layout);
      const candidateNorm = infinityNorm(candidateResidual);
      if (candidateNorm < norm || candidateNorm <= tolerance) {
        for (let k = 0; k < n; k++) x[k] = candidate[k]!;
        residual = candidateResidual;
        norm = candidateNorm;
        accepted = true;
        break;
      }
      scale *= 0.5;
    }

    iterations += 1;
    if (!accepted) break;
  }

  // One more residual evaluation at the reported solution, so the returned norm
  // always describes the returned parameters even when the loop exited on a
  // rejected step.
  residual = trimResiduals(spec, request, x, layout);
  norm = infinityNorm(residual);

  const { alphaRad, elevator, throttle, gammaRad } = unpack(layout, x);
  const theta = alphaRad + gammaRad;
  const controls: ControlInputs = { elevator, aileron: 0, rudder: 0, throttle };
  const actuators = commandedSurfaces(spec.limits, controls);
  const state = {
    positionNed: vec3(0, 0, -request.altitudeM),
    velocityBody: vec3(V * Math.cos(alphaRad), 0, V * Math.sin(alphaRad)),
    attitude: quatFromEuler({ phi: 0, theta, psi: 0 }),
    angularRateBody: vec3(0, 0, 0),
  };
  const { diagnostics } = evaluateDerivative(spec, state, actuators);

  const cl = diagnostics.coefficients.lift;
  const cd = diagnostics.coefficients.drag;

  return {
    converged: norm <= tolerance,
    iterations,
    residualNorm: norm,
    residualVector: [residual[0]!, residual[1]!, residual[2]!],
    altitudeM: request.altitudeM,
    trueAirspeedMps: V,
    alphaRad,
    thetaRad: theta,
    flightPathAngleRad: gammaRad,
    controls,
    actuators,
    stalled: Math.abs(alphaRad) > stallAngleRad(spec),
    diagnostics,
    liftToDrag: cd > 0 ? cl / cd : 0,
    liftCoefficient: cl,
    dragCoefficient: cd,
  };
}

/** Apply a Newton step to slot k, respecting that slot's physical bounds. */
function clampStep(value: number, step: number, slot: number, layout: TrimLayout): number {
  if (slot === 0) return clamp(value + step, -ALPHA_BOUND, ALPHA_BOUND);
  if (slot === 1) return clamp(value + step, -1, 1);
  return layout.gammaFixed
    ? clamp(value + step, 0, 1)
    : clamp(value + step, -GAMMA_BOUND, GAMMA_BOUND);
}

/** Standard-gravity climb rate available at a given excess power (m/s), for quick sanity checks. */
export const climbRateFromExcessPower = (excessPowerW: number, massKg: number): number =>
  excessPowerW / (massKg * G0);
