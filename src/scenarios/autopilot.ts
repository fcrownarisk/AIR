/**
 * Small closed-loop controllers used by the scenarios.
 *
 * These are deliberately simple proportional-derivative laws, not a flight
 * control system. They exist so a scenario can say "hold 1500 m" rather than
 * hand-tuning a control schedule, and so that a stall can be demonstrated the
 * way a pilot would demonstrate it: reduce power, then raise the nose to hold
 * altitude until the wing gives up.
 *
 * Gains are in per-unit-of-pilot-input terms, so a gain of 0.004 means 250 m of
 * altitude error saturates the elevator. They were chosen by observing the
 * closed-loop response, not derived; the tests assert on the physics the
 * resulting flight exhibits, not on controller elegance.
 */

import { type ControlInputs } from '../model/controls.ts';
import { type Telemetry } from '../sim/telemetry.ts';

const clamp = (v: number, lo: number, hi: number): number => (v < lo ? lo : v > hi ? hi : v);

export interface AltitudeHoldGains {
  /** Elevator per metre of altitude error (1/m). */
  readonly kp: number;
  /** Elevator per m/s of vertical speed (s/m). */
  readonly kd: number;
  /** Elevator per m/s of airspeed error (s/m). */
  readonly kSpeed: number;
}

export const DEFAULT_ALTITUDE_HOLD: AltitudeHoldGains = { kp: 0.0025, kd: 0.035, kSpeed: 0.05 };

/**
 * Elevator command to hold an altitude, with airspeed damping.
 *
 * Sign check: elevator positive = nose up. If the aircraft is BELOW target,
 * (target - altitude) is positive, so the command pitches up. Correct.
 */
export function altitudeHoldElevator(
  telemetry: Telemetry,
  targetAltitudeM: number,
  gains: AltitudeHoldGains = DEFAULT_ALTITUDE_HOLD,
): number {
  const error = targetAltitudeM - telemetry.altitudeM;
  const damping = -gains.kd * telemetry.verticalSpeedMps;
  return clamp(gains.kp * error + damping, -1, 1);
}

export interface SpeedHoldGains {
  /** Throttle per m/s of airspeed error (s/m). */
  readonly kp: number;
}

export const DEFAULT_SPEED_HOLD: SpeedHoldGains = { kp: 0.06 };

export function speedHoldThrottle(
  telemetry: Telemetry,
  targetAirspeedMps: number,
  trimThrottle: number,
  gains: SpeedHoldGains = DEFAULT_SPEED_HOLD,
): number {
  const error = targetAirspeedMps - telemetry.trueAirspeedMps;
  return clamp(trimThrottle + gains.kp * error, 0, 1);
}

/** Elevator command to hold a pitch attitude (rad). */
export function pitchHoldElevator(
  telemetry: Telemetry,
  targetPitchRad: number,
  kp = 0.08,
  kd = 0.01,
): number {
  const targetDeg = (targetPitchRad * 180) / Math.PI;
  // Damping on the actual body pitch rate. An earlier version used vertical
  // speed as a proxy, which is not the same quantity: vertical speed also
  // responds to airspeed and bank, so the loop damped the wrong variable and
  // drove a slow oscillation during rotation. The rate is available directly
  // from the state, so use it.
  return clamp(kp * (targetDeg - telemetry.pitchDeg) - kd * telemetry.pitchRateDegPerS, -1, 1);
}

export const mergeControls = (
  base: ControlInputs,
  overrides: Partial<ControlInputs>,
): ControlInputs => ({
  elevator: overrides.elevator ?? base.elevator,
  aileron: overrides.aileron ?? base.aileron,
  rudder: overrides.rudder ?? base.rudder,
  throttle: overrides.throttle ?? base.throttle,
});
