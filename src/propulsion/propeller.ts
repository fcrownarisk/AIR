/**
 * Propeller thrust model.
 *
 * Thrust is derived from available shaft power rather than being a fixed force,
 * because thrust and airspeed are not independent: a propeller converts power
 * into thrust, so T = eta P / V. That single relation is enough to reproduce
 * the two behaviours that matter — high thrust at low speed for takeoff, and
 * thrust falling off as the aeroplane accelerates in level flight.
 *
 * Available shaft power lapses with air density, the standard approximation for
 * a normally aspirated engine:
 *
 *   P_available = throttle * P_max * (rho / rho0)
 *
 * The T = eta P / V relation is singular as V -> 0, so below
 * `staticSpeedFloorMps` thrust is held at the rated static value instead. Real
 * propellers produce roughly constant thrust over the first few metres per
 * second of run, so this is the right shape, not merely a numerical patch.
 *
 * Assumptions:
 *   - Thrust acts along body x through the centre of gravity. No thrust-line
 *     offset, so no pitching moment from power changes.
 *   - Constant propulsive efficiency. Real efficiency varies with advance ratio.
 *   - No slipstream effect on the wing and tail.
 *
 * An unpowered aircraft (maxShaftPowerW = 0) produces exactly zero thrust, with
 * no special-casing needed: the model degenerates cleanly.
 */

import { type AircraftSpec } from '../model/aircraft.ts';
import { SEA_LEVEL, type Atmosphere } from '../atmos/isa.ts';

export interface ThrustResult {
  /** Thrust along body x (N). */
  readonly thrustN: number;
  /** Shaft power actually produced (W). */
  readonly shaftPowerW: number;
  /** True when thrust is limited by the static cap rather than by power. */
  readonly staticLimited: boolean;
}

export function propellerThrust(
  spec: AircraftSpec,
  atmosphere: Atmosphere,
  airspeedMps: number,
  throttle: number,
): ThrustResult {
  const { maxShaftPowerW, propEfficiency, staticThrustN, staticSpeedFloorMps } = spec.propulsion;
  const t = Math.max(0, Math.min(1, throttle));

  if (maxShaftPowerW <= 0 || propEfficiency <= 0) {
    return { thrustN: 0, shaftPowerW: 0, staticLimited: false };
  }

  const densityRatio = atmosphere.densityKgM3 / SEA_LEVEL.densityKgM3;
  const shaftPowerW = t * maxShaftPowerW * densityRatio;

  const staticCapN = t * staticThrustN * densityRatio;
  const vEff = Math.max(airspeedMps, staticSpeedFloorMps);
  const powerLimitedThrustN = (propEfficiency * shaftPowerW) / vEff;

  if (powerLimitedThrustN > staticCapN) {
    return { thrustN: staticCapN, shaftPowerW, staticLimited: true };
  }
  return { thrustN: powerLimitedThrustN, shaftPowerW, staticLimited: false };
}
