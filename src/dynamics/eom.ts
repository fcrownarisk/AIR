/**
 * The 6-DOF rigid-body equations of motion.
 *
 * State (13 numbers):
 *   position in NED (m), velocity in BODY axes (m/s),
 *   attitude as a body-to-NED quaternion, angular rate in BODY axes (rad/s)
 *
 * Translational equations, body axes, NED with z DOWN:
 *
 *   u_dot = X/m + g_x + (r v - q w)
 *   v_dot = Y/m + g_y + (p w - r u)
 *   w_dot = Z/m + g_z + (q u - p v)
 *
 * where (X, Y, Z) are aerodynamic plus propulsive forces in body axes and the
 * gravity components in body axes are
 *
 *   g_x = -g sin(theta)
 *   g_y =  g sin(phi) cos(theta)
 *   g_z =  g cos(phi) cos(theta)
 *
 * Rotational equations, with products of inertia neglected (the airframes here
 * are close enough to symmetric that Ixy, Ixz, Iyz are small):
 *
 *   p_dot = (L + (Iyy - Izz) q r) / Ixx
 *   q_dot = (M + (Izz - Ixx) r p) / Iyy
 *   r_dot = (N + (Ixx - Iyy) p q) / Izz
 *
 * Attitude propagates through the quaternion kinematic relation
 * q_dot = 0.5 q (x) [0, p, q, r], which has no singularity at 90 degrees of
 * pitch — the reason a quaternion is used rather than Euler rates.
 *
 * Modelling assumptions, stated explicitly:
 *   - Flat, non-rotating Earth. No Coriolis or centrifugal terms, no Earth rate.
 *     Irrelevant below a few hundred m/s over the durations simulated here.
 *   - Constant mass. No fuel burn.
 *   - No aeroelasticity, no structural modes, no gear or flap dynamics.
 *   - No propeller gyroscopic moment on the airframe.
 */

import { type AircraftSpec } from '../model/aircraft.ts';
import { type ActuatorState } from '../model/controls.ts';
import { type Atmosphere } from '../atmos/isa.ts';
import { G0, isaAtGeometricAltitude } from '../atmos/isa.ts';
import { aeroForcesBody, type AeroForces } from '../aero/forces.ts';
import { aerodynamicCoefficients, type AeroCoefficients } from '../aero/coefficients.ts';
import { propellerThrust, type ThrustResult } from '../propulsion/propeller.ts';
import { type Quat } from '../math/quat.ts';
import {
  QUAT_IDENTITY,
  bodyToNed,
  eulerFromQuat,
  nedToBody,
  quatFromEuler,
  quatRate,
  normalizeQuat,
} from '../math/quat.ts';
import { vec3, addScaled, type Vec3 } from '../math/vec3.ts';

export interface AircraftState {
  /** Position in NED (m). Down is positive, so altitude = -positionNed.z. */
  readonly positionNed: Vec3;
  /** Velocity in body axes (m/s): u forward, v right, w down. */
  readonly velocityBody: Vec3;
  /** Attitude, body to NED. */
  readonly attitude: Quat;
  /** Angular rate in body axes (rad/s): p roll, q pitch, r yaw. */
  readonly angularRateBody: Vec3;
}

export interface AircraftDerivative {
  readonly velocityNed: Vec3;
  readonly accelerationBody: Vec3;
  readonly attitudeRate: Quat;
  readonly angularAccelerationBody: Vec3;
}

/** Everything computed while evaluating the derivative, kept for telemetry and for the trim solver's residuals. */
export interface StepDiagnostics {
  readonly altitudeM: number;
  readonly atmosphere: Atmosphere;
  readonly coefficients: AeroCoefficients;
  readonly forces: AeroForces;
  readonly thrust: ThrustResult;
  readonly trueAirspeedMps: number;
  readonly alphaRad: number;
  readonly betaRad: number;
  readonly euler: { phi: number; theta: number; psi: number };
  /**
   * Body-z specific force divided by g — the reading of an accelerometer whose
   * sensitive axis is normal to the fuselage, positive toward the floor.
   *
   * At a trimmed condition this is exactly cos(theta), NOT 1.0. At any non-zero
   * pitch the weight has a component along body z that the wing must carry, so
   * the specific force normal to the fuselage is correspondingly reduced: a
   * level trim at alpha = 0.045 rad reads 0.999, and at alpha = 0.15 rad it reads
   * 0.989. The familiar "1 g in level flight" refers to the total lift divided by
   * the weight in the earth-vertical sense, which is a different quantity.
   */
  readonly loadFactor: number;
  /** Mechanical energy per unit mass: 0.5 V^2 + g h (J/kg). */
  readonly specificEnergy: number;
}

export interface DerivativeResult {
  readonly derivative: AircraftDerivative;
  readonly diagnostics: StepDiagnostics;
}

export const altitudeFromState = (state: AircraftState): number => -state.positionNed.z;

export function stateFrom(
  positionNed: Vec3,
  velocityBody: Vec3,
  attitude: Quat,
  angularRateBody: Vec3,
): AircraftState {
  return { positionNed, velocityBody, attitude, angularRateBody };
}

/** Build a state from altitude, body velocity and Euler attitude — the form used to start simulations. */
export function initialState(
  altitudeM: number,
  velocityBody: Vec3,
  euler: { phi: number; theta: number; psi: number },
): AircraftState {
  return {
    positionNed: vec3(0, 0, -altitudeM),
    velocityBody,
    attitude: normalizeQuat(quatFromEuler(euler)),
    angularRateBody: vec3(0, 0, 0),
  };
}

export function evaluateDerivative(
  spec: AircraftSpec,
  state: AircraftState,
  actuators: ActuatorState,
): DerivativeResult {
  const altitudeM = altitudeFromState(state);
  const atmosphere = isaAtGeometricAltitude(altitudeM);

  const coefficients = aerodynamicCoefficients(
    spec,
    atmosphere,
    state.velocityBody,
    state.angularRateBody,
    actuators,
  );
  const forces = aeroForcesBody(spec, coefficients);

  const airspeed = Math.hypot(state.velocityBody.x, state.velocityBody.y, state.velocityBody.z);
  const thrust = propellerThrust(spec, atmosphere, airspeed, actuators.throttle);

  const m = spec.mass.massKg;
  const { x: p, y: q, z: r } = state.angularRateBody;
  const { x: u, y: v, z: w } = state.velocityBody;
  const { x: X, y: Y, z: Z } = forces.forceBody;

  const euler = eulerFromQuat(state.attitude);
  const { phi, theta } = euler;

  // Gravity in body axes.
  const sinTheta = Math.sin(theta);
  const cosTheta = Math.cos(theta);
  const sinPhi = Math.sin(phi);
  const cosPhi = Math.cos(phi);
  const gx = -G0 * sinTheta;
  const gy = G0 * sinPhi * cosTheta;
  const gz = G0 * cosPhi * cosTheta;

  const accelerationBody = vec3(
    (X + thrust.thrustN) / m + gx + (r * v - q * w),
    Y / m + gy + (p * w - r * u),
    Z / m + gz + (q * u - p * v),
  );

  const { x: L, y: M, z: N } = forces.momentBody;
  const { ixx, iyy, izz } = spec.mass;
  const angularAccelerationBody = vec3(
    (L + (iyy - izz) * q * r) / ixx,
    (M + (izz - ixx) * r * p) / iyy,
    (N + (ixx - iyy) * p * q) / izz,
  );

  const velocityNed = bodyToNed(state.attitude, state.velocityBody);

  return {
    derivative: {
      velocityNed,
      accelerationBody,
      attitudeRate: quatRate(state.attitude, state.angularRateBody),
      angularAccelerationBody,
    },
    diagnostics: {
      altitudeM,
      atmosphere,
      coefficients,
      forces,
      thrust,
      trueAirspeedMps: airspeed,
      alphaRad: coefficients.alphaRad,
      betaRad: coefficients.betaRad,
      euler,
      // Specific force along body z. Equals cos(theta) at a trim; see the field
      // documentation on StepDiagnostics.loadFactor.
      loadFactor: -Z / (m * G0),
      specificEnergy: 0.5 * airspeed * airspeed + G0 * altitudeM,
    },
  };
}

/** Advance a state by `dt` along a derivative: s + dt * ds. */
export function integrateState(state: AircraftState, derivative: AircraftDerivative, dt: number): AircraftState {
  return {
    positionNed: addScaled(state.positionNed, derivative.velocityNed, dt),
    velocityBody: addScaled(state.velocityBody, derivative.accelerationBody, dt),
    attitude: normalizeQuat({
      w: state.attitude.w + derivative.attitudeRate.w * dt,
      x: state.attitude.x + derivative.attitudeRate.x * dt,
      y: state.attitude.y + derivative.attitudeRate.y * dt,
      z: state.attitude.z + derivative.attitudeRate.z * dt,
    }),
    angularRateBody: addScaled(state.angularRateBody, derivative.angularAccelerationBody, dt),
  };
}

/** True when every state component is finite. Used to abort runs that have gone numerically unstable. */
export function isStateFinite(state: AircraftState): boolean {
  const values = [
    state.positionNed.x, state.positionNed.y, state.positionNed.z,
    state.velocityBody.x, state.velocityBody.y, state.velocityBody.z,
    state.attitude.w, state.attitude.x, state.attitude.y, state.attitude.z,
    state.angularRateBody.x, state.angularRateBody.y, state.angularRateBody.z,
  ];
  return values.every(Number.isFinite);
}

export const IDENTITY_STATE: AircraftState = {
  positionNed: vec3(0, 0, 0),
  velocityBody: vec3(0, 0, 0),
  attitude: QUAT_IDENTITY,
  angularRateBody: vec3(0, 0, 0),
};

export { nedToBody };
