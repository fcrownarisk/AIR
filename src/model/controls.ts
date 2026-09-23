/**
 * Pilot controls and the actuators they drive.
 *
 * Two distinct concepts are kept separate on purpose:
 *
 *   ControlInputs  — what the pilot asks for, normalised to [-1, 1] or [0, 1]
 *   ActuatorState  — where the surfaces actually are, in radians
 *
 * They differ because surfaces do not teleport. A 60 deg/s elevator cannot
 * follow a full stick reversal instantly, and that rate limit is what makes
 * stall recovery and roll performance behave the way they do.
 *
 * Pilot sign conventions (chosen so positive means the intuitive direction):
 *   elevator  +1 = stick back   = nose up    -> surface TE UP   (negative surface angle)
 *   aileron   +1 = stick right  = roll right -> positive surface angle
 *   rudder    +1 = right pedal  = yaw right  -> positive surface angle
 *   throttle   0 = idle, 1 = full
 */

import { type ActuatorLimits } from './aircraft.ts';

export interface ControlInputs {
  /** -1 (full nose-down) .. +1 (full nose-up). */
  readonly elevator: number;
  /** -1 (roll left) .. +1 (roll right). */
  readonly aileron: number;
  /** -1 (yaw left) .. +1 (yaw right). */
  readonly rudder: number;
  /** 0 (idle) .. 1 (full power). */
  readonly throttle: number;
}

/** Surface positions in radians, plus the current throttle setting. */
export interface ActuatorState {
  /** Elevator surface angle. Positive = trailing edge DOWN = nose-down moment. */
  readonly elevatorRad: number;
  /** Aileron surface angle. Positive = roll right. */
  readonly aileronRad: number;
  /** Rudder surface angle. Positive = yaw right. */
  readonly rudderRad: number;
  /** Throttle setting actually applied, 0..1. */
  readonly throttle: number;
}

export const NEUTRAL_CONTROLS: ControlInputs = { elevator: 0, aileron: 0, rudder: 0, throttle: 0 };

export const NEUTRAL_ACTUATORS: ActuatorState = {
  elevatorRad: 0,
  aileronRad: 0,
  rudderRad: 0,
  throttle: 0,
};

export const controls = (partial: Partial<ControlInputs> = {}): ControlInputs => ({
  elevator: partial.elevator ?? 0,
  aileron: partial.aileron ?? 0,
  rudder: partial.rudder ?? 0,
  throttle: partial.throttle ?? 0,
});

const clamp = (v: number, lo: number, hi: number): number => (v < lo ? lo : v > hi ? hi : v);

/** Commanded SURFACE angles for a given pilot input. Note the elevator inversion. */
export function commandedSurfaces(limits: ActuatorLimits, input: ControlInputs): ActuatorState {
  const elevator = clamp(input.elevator, -1, 1);
  return {
    // Stick back (elevator = +1) must deflect the elevator trailing edge UP,
    // which is a negative surface angle in the trailing-edge-down convention.
    elevatorRad: -elevator * limits.maxElevatorRad,
    aileronRad: clamp(input.aileron, -1, 1) * limits.maxAileronRad,
    rudderRad: clamp(input.rudder, -1, 1) * limits.maxRudderRad,
    throttle: clamp(input.throttle, 0, 1),
  };
}

const rateLimited = (current: number, target: number, maxDelta: number): number => {
  const delta = target - current;
  if (delta > maxDelta) return current + maxDelta;
  if (delta < -maxDelta) return current - maxDelta;
  return target;
};

/**
 * Advance actuator positions toward the commanded positions over `dt` seconds,
 * respecting slew-rate limits. Throttle is treated as a first-order lag rather
 * than a rate limit, because an engine responds proportionally to how far the
 * lever was moved.
 */
export function slewActuators(
  limits: ActuatorLimits,
  current: ActuatorState,
  input: ControlInputs,
  dt: number,
): ActuatorState {
  const command = commandedSurfaces(limits, input);
  return {
    elevatorRad: rateLimited(current.elevatorRad, command.elevatorRad, limits.elevatorRateRadPerS * dt),
    aileronRad: rateLimited(current.aileronRad, command.aileronRad, limits.aileronRateRadPerS * dt),
    rudderRad: rateLimited(current.rudderRad, command.rudderRad, limits.rudderRateRadPerS * dt),
    // Engine spool lag, 1.5 s time constant.
    throttle: current.throttle + (command.throttle - current.throttle) * Math.min(1, dt / 1.5),
  };
}
