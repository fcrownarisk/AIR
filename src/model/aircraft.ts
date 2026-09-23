/**
 * Aircraft definition.
 *
 * A single `AircraftSpec` carries everything the dynamics needs: mass and
 * inertia, wing geometry, non-dimensional stability derivatives, engine ratings
 * and actuator limits. Every field is documented with its sign convention,
 * because most real-world mistakes in a model like this are sign conventions,
 * not arithmetic.
 *
 * Axis system (NED / body, all right-handed):
 *   body x = forward,  body y = right,  body z = DOWN
 *   NED    x = north,  NED  y = east,   NED  z = DOWN
 *
 * Airspeed conventions:
 *   alpha (angle of attack) = atan2(w, u)   positive = nose above velocity vector
 *   beta  (sideslip)        = asin(v / V)   positive = velocity has a right-ward component
 *
 * All coefficients are non-dimensional unless the field name says otherwise.
 * Angular-rate derivatives (CmQ, ClP, ...) act on non-dimensional rates
 * p*b/(2V), q*cbar/(2V), r*b/(2V).
 */

import { type Atmosphere } from '../atmos/isa.ts';

export interface WingGeometry {
  /** Reference wing area (m^2). */
  readonly areaM2: number;
  /** Wing span (m). */
  readonly spanM: number;
  /** Mean aerodynamic chord (m). */
  readonly chordM: number;
  /** Aspect ratio b^2 / S; stored explicitly so it can be checked against geometry. */
  readonly aspectRatio: number;
}

export interface MassProperties {
  /** Mass (kg). */
  readonly massKg: number;
  /** Roll inertia about body x (kg m^2). */
  readonly ixx: number;
  /** Pitch inertia about body y (kg m^2). */
  readonly iyy: number;
  /** Yaw inertia about body z (kg m^2). */
  readonly izz: number;
}

export interface LiftModel {
  /** Lift-curve slope dCL/dalpha (per radian). */
  readonly CLAlpha: number;
  /** Zero-lift angle of attack (rad). Negative for a cambered wing. */
  readonly alpha0Rad: number;
  /** Maximum lift coefficient before the break. */
  readonly CLMax: number;
  /**
   * Width of the stall transition (rad). The blend from the linear lift curve
   * to the flat-plate post-stall model is a smoothstep over this band starting
   * at the stall incidence. Without it the lift curve has a corner at CLMax and
   * the integrator sees a discontinuity in the derivative.
   */
  readonly stallBreakWidthRad: number;
}

export interface DragModel {
  /** Parasite drag coefficient at zero lift. */
  readonly CD0: number;
  /** Oswald span efficiency factor for the induced-drag term. */
  readonly oswaldE: number;
}

export interface PitchModel {
  /** Pitching-moment coefficient at zero lift and zero rate; positive = nose-up. */
  readonly Cm0: number;
  /** dCm/dalpha (per radian). Negative = statically stable. */
  readonly CmAlpha: number;
  /** Pitch-damping derivative dCm/d(q cbar / 2V). */
  readonly CmQ: number;
  /**
   * dCm/d(elevator surface deflection), per radian of SURFACE angle.
   * Sign convention: surface positive = trailing edge DOWN, which produces a
   * nose-DOWN (negative) contribution, hence the negative value. Pilot input is
   * mapped to surfaces in `actuators.ts`.
   */
  readonly CmElevator: number;
}

export interface LateralModel {
  /** dCY/dbeta (per radian). Negative: positive sideslip produces a leftward force. */
  readonly CYBeta: number;
  /** dCY/d(rudder surface), per radian. Rudder TE-left gives a leftward force. */
  readonly CYRudder: number;

  /** dCl/dbeta (per radian) — dihedral effect. Negative: right sideslip rolls left. */
  readonly ClBeta: number;
  /** dCl/d(aileron) per radian. Aileron command positive = roll right. */
  readonly ClAileron: number;
  /** dCl/d(aileron), yawing — adverse yaw. */
  readonly CnAileron: number;
  /** Roll-damping derivative dCl/d(p b / 2V). */
  readonly ClP: number;
  /** Roll due to yaw rate dCl/d(r b / 2V). */
  readonly ClR: number;

  /** dCn/dbeta (per radian) — weathercock stability. Positive = nose into the wind. */
  readonly CnBeta: number;
  /** dCn/d(rudder) per radian. Rudder command positive = yaw right. */
  readonly CnRudder: number;
  /** Yaw due to roll rate dCn/d(p b / 2V). */
  readonly CnP: number;
  /** Yaw-damping derivative dCn/d(r b / 2V). */
  readonly CnR: number;
}

export interface Propulsion {
  /** Rated shaft power at sea level (W). 0 = unpowered. */
  readonly maxShaftPowerW: number;
  /** Propeller efficiency, thrust power / shaft power. */
  readonly propEfficiency: number;
  /** Thrust cap at zero airspeed (N), where the power equation is singular. */
  readonly staticThrustN: number;
  /**
   * IAS below which the propeller is treated as static (m/s). The power-based
   * thrust model T = eta P / V diverges as V -> 0, so thrust is held at the
   * static cap below this speed.
   */
  readonly staticSpeedFloorMps: number;
}

export interface ActuatorLimits {
  /** Maximum elevator surface deflection (rad). Positive = trailing edge down. */
  readonly maxElevatorRad: number;
  /** Maximum aileron deflection (rad). */
  readonly maxAileronRad: number;
  /** Maximum rudder deflection (rad). */
  readonly maxRudderRad: number;
  /** Slew rate of the elevator (rad/s). */
  readonly elevatorRateRadPerS: number;
  readonly aileronRateRadPerS: number;
  readonly rudderRateRadPerS: number;
}

export interface AircraftSpec {
  readonly name: string;
  readonly mass: MassProperties;
  readonly wing: WingGeometry;
  readonly lift: LiftModel;
  readonly drag: DragModel;
  readonly pitch: PitchModel;
  readonly lateral: LateralModel;
  readonly propulsion: Propulsion;
  readonly limits: ActuatorLimits;
}

/** Weight in newtons under ISA standard gravity. */
export const weightN = (spec: AircraftSpec): number => spec.mass.massKg * 9.80665;

/** Wing loading W/S in N/m^2. */
export const wingLoading = (spec: AircraftSpec): number => weightN(spec) / spec.wing.areaM2;

/**
 * Incidence at which the linear lift curve reaches CLMax (rad).
 *
 * Returns Infinity when the lift-curve slope is zero or negative, which makes
 * the stall blend inert. That is deliberate: it lets a test build a pure
 * drag-body spec (CLAlpha = 0, CLMax = 0) and get CL == 0 at every incidence
 * instead of a 0/0.
 */
export function stallAngleRad(spec: AircraftSpec): number {
  if (!(spec.lift.CLAlpha > 0)) return Number.POSITIVE_INFINITY;
  return spec.lift.CLMax / spec.lift.CLAlpha + spec.lift.alpha0Rad;
}

/** Induced-drag factor k = 1 / (pi e AR). Zero when the polar is undefined. */
export function inducedDragFactor(spec: AircraftSpec): number {
  const { oswaldE } = spec.drag;
  const { aspectRatio } = spec.wing;
  if (!(oswaldE > 0) || !(aspectRatio > 0)) return 0;
  return 1 / (Math.PI * oswaldE * aspectRatio);
}

/** True airspeed for a given lift coefficient in steady level flight (m/s). */
export const speedForLiftCoefficient = (spec: AircraftSpec, atmosphere: Atmosphere, cl: number): number =>
  Math.sqrt((2 * weightN(spec)) / (atmosphere.densityKgM3 * spec.wing.areaM2 * cl));

/**
 * 1-g stall speed in level flight (m/s): Vs = sqrt(2W / (rho S CLmax)).
 * This is the closed-form value the simulated stall is checked against.
 */
export const stallSpeedMps = (spec: AircraftSpec, atmosphere: Atmosphere): number =>
  speedForLiftCoefficient(spec, atmosphere, spec.lift.CLMax);

/** Best lift-to-drag ratio for a parabolic polar: max L/D = 1 / (2 sqrt(CD0 k)). */
export function bestLiftToDrag(spec: AircraftSpec): number {
  const k = inducedDragFactor(spec);
  const { CD0 } = spec.drag;
  if (!(CD0 > 0) || !(k > 0)) return 0;
  return 1 / (2 * Math.sqrt(CD0 * k));
}

/** Lift coefficient at best L/D: CL = sqrt(CD0 / k). */
export function bestLiftToDragCoefficient(spec: AircraftSpec): number {
  const k = inducedDragFactor(spec);
  if (!(k > 0)) return 0;
  return Math.sqrt(spec.drag.CD0 / k);
}
