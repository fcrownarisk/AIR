/**
 * Non-dimensional aerodynamic coefficients.
 *
 * The lift model blends two standard models across the stall:
 *
 *   ATTACHED  (|alpha| < alpha_stall)
 *     CL = CLAlpha (alpha - alpha0)         linear lift curve
 *     CD = CD0 + k CL^2                     parabolic drag polar, k = 1/(pi e AR)
 *
 *   SEPARATED (|alpha| > alpha_stall + break width)
 *     CL = 2 sin(alpha) cos(alpha)          flat-plate / Newtonian-like
 *     CD = CD0 + 2 sin^2(alpha)             flat-plate normal force resolved into drag
 *
 *   BETWEEN the two, a smoothstep blends them.
 *
 * Why blend rather than switch at CLMax: a hard switch puts a step
 * discontinuity into the derivative function, and RK4 loses its convergence
 * order across it (the error estimate assumes a bounded fourth derivative).
 * Testing an RK4 convergence sweep with a step in the lift curve shows the
 * order collapsing toward 1 near the stall, which is how this was found.
 *
 * Known limitations, stated rather than buried:
 *   - Incompressible. No Prandtl-Glauert or wave-drag correction, so the model
 *     is not valid near or above the critical Mach number.
 *   - No hysteresis. Real wings reattach at a higher incidence than they stall
 *     at; the blend is symmetric and path-independent.
 *   - No Reynolds-number or flap effects. CLMax is a single value.
 *   - No propeller slipstream. The accelerated flow behind the propeller is not
 *     modelled, which understates elevator authority at low airspeed on takeoff.
 */

import { type AircraftSpec, inducedDragFactor, stallAngleRad } from '../model/aircraft.ts';
import { type ActuatorState } from '../model/controls.ts';
import { type Atmosphere } from '../atmos/isa.ts';
import { type Vec3 } from '../math/vec3.ts';

export interface AeroCoefficients {
  readonly alphaRad: number;
  readonly betaRad: number;
  /** Dynamic pressure (Pa). */
  readonly dynamicPressurePa: number;
  readonly lift: number;
  readonly drag: number;
  readonly pitchingMoment: number;
  readonly sideForce: number;
  readonly rollingMoment: number;
  readonly yawingMoment: number;
  /** 0 = fully attached, 1 = fully separated. */
  readonly separation: number;
  readonly machNumber: number;
}

/**
 * Airspeed below which the direction of the velocity vector is dominated by
 * noise in the integrator, so alpha and beta are undefined in any useful sense.
 * Coefficients are evaluated anyway (the dynamic pressure that multiplies them
 * is ~0), but the non-dimensional rates are zeroed to avoid division blow-up.
 */
export const MIN_AIRSPEED_FOR_ANGLES_MPS = 0.5;

const smoothstep = (t: number): number => {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  return t * t * (3 - 2 * t);
};

/** Angle of attack and sideslip from a body-frame velocity. */
export function flowAngles(velocityBody: Vec3): { alphaRad: number; betaRad: number; speed: number } {
  const { x: u, y: v, z: w } = velocityBody;
  const speed = Math.hypot(u, v, w);
  if (speed < MIN_AIRSPEED_FOR_ANGLES_MPS) {
    return { alphaRad: 0, betaRad: 0, speed };
  }
  return {
    alphaRad: Math.atan2(w, u),
    betaRad: Math.asin(Math.max(-1, Math.min(1, v / speed))),
    speed,
  };
}

/** Fraction of flow separation, 0 (attached) to 1 (fully stalled). */
export function separationFraction(spec: AircraftSpec, alphaRad: number): number {
  const alphaStall = stallAngleRad(spec);
  if (!Number.isFinite(alphaStall)) return 0; // no lift curve => nothing to stall
  const width = Math.max(spec.lift.stallBreakWidthRad, 1e-9);
  return smoothstep((Math.abs(alphaRad) - alphaStall) / width);
}

export function aerodynamicCoefficients(
  spec: AircraftSpec,
  atmosphere: Atmosphere,
  velocityBody: Vec3,
  angularRateBody: Vec3,
  actuators: ActuatorState,
): AeroCoefficients {
  const { alphaRad, betaRad, speed } = flowAngles(velocityBody);
  const dynamicPressurePa = 0.5 * atmosphere.densityKgM3 * speed * speed;
  const { CLAlpha, alpha0Rad } = spec.lift;
  const k = inducedDragFactor(spec);

  const sigma = separationFraction(spec, alphaRad);

  // Attached-flow model.
  const clAttached = CLAlpha * (alphaRad - alpha0Rad);
  const cdAttached = spec.drag.CD0 + k * clAttached * clAttached;

  // Separated-flow model.
  const clSeparated = 2 * Math.sin(alphaRad) * Math.cos(alphaRad);
  const cdSeparated = spec.drag.CD0 + 2 * Math.sin(alphaRad) * Math.sin(alphaRad);

  const lift = (1 - sigma) * clAttached + sigma * clSeparated;
  const drag = (1 - sigma) * cdAttached + sigma * cdSeparated;

  // Non-dimensional rates. Undefined below the speed floor: with speed = 0 the
  // ratio (p * b) / (2 * V) is 0/0, which is NaN rather than zero, and a single
  // NaN poisons the whole state within one step. That is exactly what a
  // standing-start takeoff hits on its first iteration, so the denominator is
  // floored as well as the numerator.
  const rateValid = speed >= MIN_AIRSPEED_FOR_ANGLES_MPS;
  const p = rateValid ? angularRateBody.x : 0;
  const q = rateValid ? angularRateBody.y : 0;
  const r = rateValid ? angularRateBody.z : 0;
  const speedForRates = rateValid ? speed : MIN_AIRSPEED_FOR_ANGLES_MPS;
  const pHat = (p * spec.wing.spanM) / (2 * speedForRates);
  const qHat = (q * spec.wing.chordM) / (2 * speedForRates);
  const rHat = (r * spec.wing.spanM) / (2 * speedForRates);

  const { elevatorRad, aileronRad, rudderRad } = actuators;

  const pitchingMoment = rateValid
    ? spec.pitch.Cm0 +
      spec.pitch.CmAlpha * alphaRad +
      spec.pitch.CmQ * qHat +
      spec.pitch.CmElevator * elevatorRad
    : spec.pitch.Cm0 + spec.pitch.CmAlpha * alphaRad + spec.pitch.CmElevator * elevatorRad;

  const sideForce = spec.lateral.CYBeta * betaRad + spec.lateral.CYRudder * rudderRad;

  const rollingMoment =
    spec.lateral.ClBeta * betaRad +
    spec.lateral.ClAileron * aileronRad +
    spec.lateral.ClP * pHat +
    spec.lateral.ClR * rHat;

  const yawingMoment =
    spec.lateral.CnBeta * betaRad +
    spec.lateral.CnAileron * aileronRad +
    spec.lateral.CnRudder * rudderRad +
    spec.lateral.CnP * pHat +
    spec.lateral.CnR * rHat;

  return {
    alphaRad,
    betaRad,
    dynamicPressurePa,
    lift,
    drag,
    pitchingMoment,
    sideForce,
    rollingMoment,
    yawingMoment,
    separation: sigma,
    machNumber: speed / atmosphere.speedOfSoundMps,
  };
}
