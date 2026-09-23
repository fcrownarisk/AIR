/**
 * Fixed-step classical Runge-Kutta 4 integrator.
 *
 * RK4 is used rather than an adaptive scheme because the simulation samples at a
 * fixed rate and the interesting physics (the stall break) puts a large but
 * BOUNDED fourth derivative into the system — the smoothstep blend in the lift
 * model exists partly to keep it bounded. A fixed step keeps the behaviour
 * reproducible, which matters when a performance number is being quoted.
 *
 * Controlling trim uses dt = 0.02 s (50 Hz). At that step the measured
 * convergence order is verified in the test suite against a self-converged
 * reference, so the step size is evidence-based rather than guessed.
 *
 * Actuator positions are held constant across a step (zero-order hold), which
 * matches how a real surface behaves within a 20 ms control frame.
 */

import { type AircraftSpec } from '../model/aircraft.ts';
import { type ActuatorState } from '../model/controls.ts';
import {
  type AircraftState,
  type StepDiagnostics,
  evaluateDerivative,
  integrateState,
} from './eom.ts';

export interface IntegrationResult {
  readonly state: AircraftState;
  /** Diagnostics evaluated at the START of the step, i.e. at the reported time. */
  readonly diagnostics: StepDiagnostics;
}

export function rk4Step(
  spec: AircraftSpec,
  state: AircraftState,
  actuators: ActuatorState,
  dt: number,
): IntegrationResult {
  const k1 = evaluateDerivative(spec, state, actuators);

  const s2 = integrateState(state, k1.derivative, dt / 2);
  const k2 = evaluateDerivative(spec, s2, actuators);

  const s3 = integrateState(state, k2.derivative, dt / 2);
  const k3 = evaluateDerivative(spec, s3, actuators);

  const s4 = integrateState(state, k3.derivative, dt);
  const k4 = evaluateDerivative(spec, s4, actuators);

  const combined = {
    velocityNed: blend(k1.derivative.velocityNed, k2.derivative.velocityNed, k3.derivative.velocityNed, k4.derivative.velocityNed),
    accelerationBody: blend(k1.derivative.accelerationBody, k2.derivative.accelerationBody, k3.derivative.accelerationBody, k4.derivative.accelerationBody),
    angularAccelerationBody: blend(k1.derivative.angularAccelerationBody, k2.derivative.angularAccelerationBody, k3.derivative.angularAccelerationBody, k4.derivative.angularAccelerationBody),
    attitudeRate: blendQuat(k1.derivative.attitudeRate, k2.derivative.attitudeRate, k3.derivative.attitudeRate, k4.derivative.attitudeRate),
  };

  return {
    state: integrateState(state, combined, dt),
    diagnostics: k1.diagnostics,
  };
}

/** Weighted average for the classical RK4 combination: (k1 + 2k2 + 2k3 + k4) / 6. */
function blend(
  k1: { x: number; y: number; z: number },
  k2: { x: number; y: number; z: number },
  k3: { x: number; y: number; z: number },
  k4: { x: number; y: number; z: number },
): { x: number; y: number; z: number } {
  return {
    x: (k1.x + 2 * k2.x + 2 * k3.x + k4.x) / 6,
    y: (k1.y + 2 * k2.y + 2 * k3.y + k4.y) / 6,
    z: (k1.z + 2 * k2.z + 2 * k3.z + k4.z) / 6,
  };
}

function blendQuat(
  k1: { w: number; x: number; y: number; z: number },
  k2: { w: number; x: number; y: number; z: number },
  k3: { w: number; x: number; y: number; z: number },
  k4: { w: number; x: number; y: number; z: number },
): { w: number; x: number; y: number; z: number } {
  return {
    w: (k1.w + 2 * k2.w + 2 * k3.w + k4.w) / 6,
    x: (k1.x + 2 * k2.x + 2 * k3.x + k4.x) / 6,
    y: (k1.y + 2 * k2.y + 2 * k3.y + k4.y) / 6,
    z: (k1.z + 2 * k2.z + 2 * k3.z + k4.z) / 6,
  };
}
