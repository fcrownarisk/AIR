/**
 * Simulation harness.
 *
 * Owns the state, the actuators, the clock and the event log. The caller
 * supplies a control script — a function from (state, time, telemetry) to pilot
 * inputs — which is what makes the same harness serve an autopilot test, a
 * scripted flight profile and a stability probe without special-casing.
 *
 * Ground handling is opt-in via `groundAltitudeM` and is deliberately minimal:
 * a rigid floor that stops penetration and removes downward velocity, plus
 * optional rolling friction. It is NOT a landing-gear model. There is no
 * oleo travel, no wheel normal force distribution, no brake torque, no
 * nose-wheel steering and no ground effect. Takeoff distances from this model
 * should be treated as indicative. The limitation is stated here rather than
 * left for someone to discover by comparing against a POH.
 */

import { type AircraftSpec } from '../model/aircraft.ts';
import {
  type ActuatorState,
  type ControlInputs,
  NEUTRAL_ACTUATORS,
  slewActuators,
} from '../model/controls.ts';
import { isaAtGeometricAltitude, G0 } from '../atmos/isa.ts';
import {
  type AircraftState,
  type StepDiagnostics,
  altitudeFromState,
  evaluateDerivative,
  isStateFinite,
} from '../dynamics/eom.ts';
import { bodyToNed, eulerFromQuat, nedToBody, normalizeQuat, quatFromEuler } from '../math/quat.ts';
import { vec3 } from '../math/vec3.ts';
import { rk4Step } from '../dynamics/integrator.ts';
import { buildTelemetry, type Telemetry } from './telemetry.ts';

export interface SimulationOptions {
  /** Integration step (s). Default 0.02 (50 Hz). */
  readonly dt?: number;
  /** Actuator positions to start from. Defaults to all-neutral. */
  readonly initialActuators?: ActuatorState;
  /**
   * Enables the rigid-floor ground constraint at this altitude (m). Leave
   * undefined for a purely airborne simulation. See the module note on the
   * ground model's limits.
   */
  readonly groundAltitudeM?: number;
  /** Rolling friction coefficient applied while on the ground. Default 0.02. */
  readonly rollingFrictionCoefficient?: number;
}

/**
 * Altitude above the ground at which the aeroplane is considered airborne for
 * event reporting. Without this margin, an aeroplane whose lift is within a
 * fraction of a percent of its weight at the instant of liftoff flickers on and
 * off the constraint every step, and the event log fills with hundreds of
 * alternating takeoff/touchdown pairs. The model has no ground effect, so there
 * is no physical cushion to smooth the transition; the hysteresis supplies one.
 */
const AIRBORNE_ALTITUDE_M = 1.0;

/** Time constant for the on-ground attitude constraint (s). */
const GROUND_ATTITUDE_TIME_CONSTANT_S = 0.8;
/**
 * Pitch attitude produced by full aft elevator while on the ground (rad, 20 deg).
 *
 * Chosen so that a realistic rotation uses only about half the available
 * elevator. If the ground authority is too small, the takeoff has to command
 * full aft stick just to rotate; when the wheels leave, that same full-aft
 * setting over-rotates the aeroplane before the elevator can slew back, and it
 * stalls on climb-out.
 *
 * The mapping is deliberately 1:1 with the pilot input rather than amplified.
 * The ground model maps elevator straight to pitch, whereas in the air the same
 * input produces an aerodynamic pitching moment, so keeping the two responses
 * comparable is what makes the transition from runway to air benign.
 */
const GROUND_MAX_ROTATION_RAD = 0.35;

export interface SimulationStep {
  readonly timeS: number;
  readonly state: AircraftState;
  readonly telemetry: Telemetry;
  readonly diagnostics: StepDiagnostics;
  readonly actuators: ActuatorState;
  readonly onGround: boolean;
}

export type ControlScript = (state: AircraftState, timeS: number, telemetry: Telemetry) => ControlInputs;

export interface FlightEvent {
  readonly timeS: number;
  readonly type: 'takeoff' | 'touchdown' | 'stall-onset' | 'stall-recovery' | 'run-aborted';
  readonly detail: string;
}

export interface RunResult {
  readonly samples: readonly Telemetry[];
  readonly events: readonly FlightEvent[];
  readonly finalState: AircraftState;
  readonly finalTelemetry: Telemetry;
  readonly durationS: number;
  readonly steps: number;
  readonly completed: boolean;
}

export interface RunOptions {
  /** Telemetry sample rate (Hz). Default 10. */
  readonly sampleHz?: number;
  /** Stop early when this predicate returns true. */
  readonly stopWhen?: (telemetry: Telemetry, timeS: number) => boolean;
  /** Called at each telemetry sample. */
  readonly onSample?: (telemetry: Telemetry) => void;
}

export class Simulation {
  readonly spec: AircraftSpec;
  readonly dt: number;

  #state: AircraftState;
  #actuators: ActuatorState;
  #timeS = 0;
  #onGround = false;
  readonly #groundAltitudeM: number | undefined;
  readonly #rollingFriction: number;

  constructor(spec: AircraftSpec, initialState: AircraftState, options: SimulationOptions = {}) {
    this.spec = spec;
    this.dt = options.dt ?? 0.02;
    if (!(this.dt > 0)) throw new Error('Simulation dt must be positive');
    this.#state = initialState;
    this.#actuators = options.initialActuators ?? NEUTRAL_ACTUATORS;
    this.#groundAltitudeM = options.groundAltitudeM;
    this.#rollingFriction = options.rollingFrictionCoefficient ?? 0.02;
    if (this.#groundAltitudeM !== undefined) {
      this.#onGround = altitudeFromState(initialState) <= this.#groundAltitudeM + 1e-9;
    }
  }

  get timeS(): number {
    return this.#timeS;
  }

  get state(): AircraftState {
    return this.#state;
  }

  get actuators(): ActuatorState {
    return this.#actuators;
  }

  get onGround(): boolean {
    return this.#onGround;
  }

  /** Advance one integration step under the given pilot inputs. */
  step(controls: ControlInputs): SimulationStep {
    const nextActuators = slewActuators(this.spec.limits, this.#actuators, controls, this.dt);
    const result = rk4Step(this.spec, this.#state, nextActuators, this.dt);

    let nextState = result.state;
    let onGround = false;

    if (this.#groundAltitudeM !== undefined) {
      const ground = this.#groundAltitudeM;
      if (altitudeFromState(nextState) <= ground) {
        onGround = true;

        // Attitude while on the ground.
        //
        // The aerodynamic pitching moment must NOT be integrated on the ground.
        // A 172-class aeroplane has a positive Cm0 (nose-up at zero lift), so
        // with a bare floor it rotates itself onto its tail before reaching
        // rotation speed, lifts off at roughly half the correct speed, and then
        // chatters on and off the ground. The gear is what resists that moment.
        //
        // So on the ground the attitude is driven by the pilot's elevator input
        // through a first-order lag, which stands in for the nosewheel and
        // mainwheel reactions: theta tracks elevator * maxGroundPitch, and roll
        // is levelled. This is a simplification, stated as one — the real
        // stiffness of a nose oleo and the timing of a rotation are not modelled.
        const eulerNow = eulerFromQuat(nextState.attitude);
        const rate = Math.min(1, this.dt / GROUND_ATTITUDE_TIME_CONSTANT_S);
        const targetPitch = Math.max(
          -GROUND_MAX_ROTATION_RAD,
          Math.min(GROUND_MAX_ROTATION_RAD, controls.elevator * GROUND_MAX_ROTATION_RAD),
        );
        const constrainedAttitude = normalizeQuat(
          quatFromEuler({
            phi: eulerNow.phi * (1 - rate),
            theta: eulerNow.theta + (targetPitch - eulerNow.theta) * rate,
            psi: eulerNow.psi,
          }),
        );

        // Bring the state back onto the floor, then express the constrained
        // velocity back in body axes. Converting through NED (rather than
        // editing w directly) keeps the body velocity consistent with attitude:
        // an aircraft sitting at 10 deg of pitch has w = V sin(10 deg), not 0.
        const ned = bodyToNed(constrainedAttitude, nextState.velocityBody);
        let vNorth = ned.x;
        let vEast = ned.y;
        const vDown = Math.min(ned.z, 0); // no downward motion through the floor

        // Rolling friction opposes the horizontal velocity.
        const horizontalSpeed = Math.hypot(vNorth, vEast);
        if (horizontalSpeed > 1e-6 && this.#rollingFriction > 0) {
          const decel = this.#rollingFriction * G0 * this.dt;
          const scale = Math.max(0, horizontalSpeed - decel) / horizontalSpeed;
          vNorth *= scale;
          vEast *= scale;
        }

        const constrainedNed = vec3(vNorth, vEast, vDown);
        nextState = {
          positionNed: vec3(nextState.positionNed.x, nextState.positionNed.y, -ground),
          velocityBody: nedToBody(constrainedAttitude, constrainedNed),
          attitude: constrainedAttitude,
          angularRateBody: vec3(0, 0, 0),
        };
      }
    }

    this.#state = nextState;
    this.#actuators = nextActuators;
    this.#timeS += this.dt;
    this.#onGround = onGround;

    return {
      timeS: this.#timeS,
      state: this.#state,
      // Diagnostics describe the START of the step; at the final step the
      // difference is one dt and is irrelevant at the reported precision.
      telemetry: buildTelemetry(
        this.spec,
        this.#state,
        isaAtGeometricAltitude(altitudeFromState(this.#state)),
        result.diagnostics,
        this.#actuators,
        this.#timeS,
      ),
      diagnostics: result.diagnostics,
      actuators: this.#actuators,
      onGround,
    };
  }

  /**
   * Run a control script for a duration, sampling telemetry and detecting events.
   *
   * The loop is deliberately plain: a fixed step, a control evaluation, an event
   * check. No adaptive stepping means a run is reproducible bit-for-bit, which
   * matters when the output is a performance number rather than a picture.
   */
  run(script: ControlScript, durationS: number, options: RunOptions = {}): RunResult {
    const sampleHz = options.sampleHz ?? 10;
    const sampleEvery = Math.max(1, Math.round(1 / (sampleHz * this.dt)));
    const totalSteps = Math.max(1, Math.round(durationS / this.dt));

    const samples: Telemetry[] = [];
    const events: FlightEvent[] = [];
    let completed = true;

    // Ground events are only meaningful when a ground model is active. Without
    // one the aeroplane is airborne for the whole run and emitting a synthetic
    // 'takeoff' at t=0 would be a lie.
    const groundModelActive = this.#groundAltitudeM !== undefined;
    let airborne = groundModelActive
      ? !this.#onGround && altitudeFromState(this.#state) > AIRBORNE_ALTITUDE_M
      : true;
    // Emit the initial condition, so a run always has at least one sample.
    const initial = evaluateDerivative(this.spec, this.#state, this.#actuators);
    let telemetry = buildTelemetry(
      this.spec,
      this.#state,
      initial.diagnostics.atmosphere,
      initial.diagnostics,
      this.#actuators,
      this.#timeS,
    );
    samples.push(telemetry);
    options.onSample?.(telemetry);
    let previousStalled = telemetry.stalled;

    for (let i = 1; i <= totalSteps; i++) {
      const controls = script(this.#state, this.#timeS, telemetry);
      const stepResult = this.step(controls);
      telemetry = stepResult.telemetry;

      if (groundModelActive) {
        if (stepResult.onGround && airborne) {
          airborne = false;
          events.push({ timeS: this.#timeS, type: 'touchdown', detail: 'ground contact' });
        } else if (!stepResult.onGround && !airborne && telemetry.altitudeM > AIRBORNE_ALTITUDE_M) {
          airborne = true;
          events.push({
            timeS: this.#timeS,
            type: 'takeoff',
            detail: `airborne at ${telemetry.trueAirspeedMps.toFixed(2)} m/s`,
          });
        }
      }

      if (telemetry.stalled && !previousStalled) {
        events.push({
          timeS: this.#timeS,
          type: 'stall-onset',
          detail: `alpha ${telemetry.alphaDeg.toFixed(1)} deg, TAS ${telemetry.trueAirspeedMps.toFixed(2)} m/s, separation ${telemetry.separation.toFixed(2)}`,
        });
      } else if (!telemetry.stalled && previousStalled) {
        events.push({
          timeS: this.#timeS,
          type: 'stall-recovery',
          detail: `alpha back to ${telemetry.alphaDeg.toFixed(1)} deg at TAS ${telemetry.trueAirspeedMps.toFixed(2)} m/s`,
        });
      }
      previousStalled = telemetry.stalled;

      if (options.stopWhen?.(telemetry, this.#timeS)) {
        samples.push(telemetry);
        options.onSample?.(telemetry);
        break;
      }

      if (i % sampleEvery === 0) {
        samples.push(telemetry);
        options.onSample?.(telemetry);
      }

      if (!isStateFinite(this.#state)) {
        events.push({
          timeS: this.#timeS,
          type: 'run-aborted',
          detail: 'non-finite state; the integration diverged',
        });
        completed = false;
        break;
      }
    }

    return {
      samples,
      events,
      finalState: this.#state,
      finalTelemetry: telemetry,
      durationS: this.#timeS,
      steps: totalSteps,
      completed,
    };
  }
}

/** Diagnostics at a state; keeps the initial-sample path identical to the stepping path. */
function evaluateAt(spec: AircraftSpec, state: AircraftState): StepDiagnostics {
  return evaluateDerivative(spec, state, NEUTRAL_ACTUATORS).diagnostics;
}

export { evaluateAt };
