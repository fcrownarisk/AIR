/**
 * Telemetry: the derived quantities a pilot or an engineer would want to read.
 *
 * This is the headless equivalent of a HUD. There is no renderer in this
 * project by design, so `Telemetry` is the interface: everything downstream
 * (CLI panels, tests, a future UI) reads these numbers rather than recomputing
 * them from raw state.
 */

import { type AircraftSpec, stallSpeedMps } from '../model/aircraft.ts';
import { type Atmosphere } from '../atmos/isa.ts';
import { type AircraftState, type StepDiagnostics, altitudeFromState } from '../dynamics/eom.ts';
import { bodyToNed } from '../math/quat.ts';

export interface Telemetry {
  readonly timeS: number;
  readonly altitudeM: number;
  /** True airspeed (m/s). */
  readonly trueAirspeedMps: number;
  /** True airspeed in knots, for comparison against published figures. */
  readonly trueAirspeedKt: number;
  /** Horizontal speed over the ground (m/s). */
  readonly groundSpeedMps: number;
  /** Vertical speed, positive = climbing (m/s). */
  readonly verticalSpeedMps: number;
  readonly machNumber: number;
  readonly alphaDeg: number;
  readonly betaDeg: number;
  readonly pitchDeg: number;
  readonly rollDeg: number;
  /** Body roll rate p (deg/s), positive right-wing-down. */
  readonly rollRateDegPerS: number;
  /** Body pitch rate q (deg/s), positive nose-up. */
  readonly pitchRateDegPerS: number;
  /** Body yaw rate r (deg/s), positive nose-right. */
  readonly yawRateDegPerS: number;
  /** Heading in degrees, 0 = north, increasing clockwise. */
  readonly headingDeg: number;
  /** Flight path angle in degrees, positive = climbing. */
  readonly flightPathAngleDeg: number;
  /**
   * Body-z specific force over g — the accelerometer reading normal to the
   * fuselage. Equal to cos(pitch) at a trimmed condition, so a level trim at a
   * few degrees of incidence reads just under 1.0. See the field note on
   * StepDiagnostics.loadFactor.
   */
  readonly loadFactor: number;
  readonly liftN: number;
  readonly dragN: number;
  readonly sideForceN: number;
  readonly thrustN: number;
  readonly dynamicPressurePa: number;
  /** 0 = attached flow, 1 = fully separated. */
  readonly separation: number;
  readonly stalled: boolean;
  readonly elevatorDeg: number;
  readonly aileronDeg: number;
  readonly rudderDeg: number;
  readonly throttle: number;
  /** Mechanical energy per unit mass (J/kg). */
  readonly specificEnergyJPerKg: number;
  /** 1-g stall speed at the current altitude (m/s). */
  readonly stallSpeedMps: number;
  /** trueAirspeed - stallSpeed. Negative means the wing cannot make 1 g of lift. */
  readonly speedMarginMps: number;
}

const RAD_TO_DEG = 180 / Math.PI;

export function buildTelemetry(
  spec: AircraftSpec,
  state: AircraftState,
  atmosphere: Atmosphere,
  diagnostics: StepDiagnostics,
  actuators: { elevatorRad: number; aileronRad: number; rudderRad: number; throttle: number },
  timeS: number,
): Telemetry {
  const altitudeM = altitudeFromState(state);
  const velocityNed = bodyToNed(state.attitude, state.velocityBody);
  const groundSpeedMps = Math.hypot(velocityNed.x, velocityNed.y);
  const verticalSpeedMps = -velocityNed.z;
  const flightPathAngleDeg = Math.atan2(verticalSpeedMps, groundSpeedMps) * RAD_TO_DEG;
  const vs = stallSpeedMps(spec, atmosphere);

  return {
    timeS,
    altitudeM,
    trueAirspeedMps: diagnostics.trueAirspeedMps,
    trueAirspeedKt: diagnostics.trueAirspeedMps * 1.943844,
    groundSpeedMps,
    verticalSpeedMps,
    machNumber: diagnostics.coefficients.machNumber,
    alphaDeg: diagnostics.alphaRad * RAD_TO_DEG,
    betaDeg: diagnostics.betaRad * RAD_TO_DEG,
    pitchDeg: diagnostics.euler.theta * RAD_TO_DEG,
    rollDeg: diagnostics.euler.phi * RAD_TO_DEG,
    rollRateDegPerS: state.angularRateBody.x * RAD_TO_DEG,
    pitchRateDegPerS: state.angularRateBody.y * RAD_TO_DEG,
    yawRateDegPerS: state.angularRateBody.z * RAD_TO_DEG,
    headingDeg: ((diagnostics.euler.psi * RAD_TO_DEG) % 360 + 360) % 360,
    flightPathAngleDeg,
    loadFactor: diagnostics.loadFactor,
    liftN: diagnostics.forces.liftN,
    dragN: diagnostics.forces.dragN,
    sideForceN: diagnostics.forces.sideN,
    thrustN: diagnostics.thrust.thrustN,
    dynamicPressurePa: diagnostics.coefficients.dynamicPressurePa,
    separation: diagnostics.coefficients.separation,
    stalled: diagnostics.coefficients.separation > 0.5,
    elevatorDeg: actuators.elevatorRad * RAD_TO_DEG,
    aileronDeg: actuators.aileronRad * RAD_TO_DEG,
    rudderDeg: actuators.rudderRad * RAD_TO_DEG,
    throttle: actuators.throttle,
    specificEnergyJPerKg: diagnostics.specificEnergy,
    stallSpeedMps: vs,
    speedMarginMps: diagnostics.trueAirspeedMps - vs,
  };
}
