/**
 * AirCraft — headless 6-DOF flight dynamics sandbox.
 *
 * Public surface. Everything the library offers is reachable from here.
 *
 * Zero runtime dependencies: Node 22.18+ executes these .ts sources directly
 * via type stripping. There is no build step. `npm run typecheck` invokes tsc
 * from outside the project tree (see scripts/tsc.mjs).
 */

export * as vec3 from './math/vec3.ts';
export * as quat from './math/quat.ts';
export { solveLinearSystem } from './math/linsolve.ts';

export {
  isa,
  isaAtGeometricAltitude,
  geometricToGeopotential,
  geopotentialToGeometric,
  SEA_LEVEL,
  G0,
  R_AIR,
  GAMMA_AIR,
  VALID_CEILING_M,
  type Atmosphere,
} from './atmos/isa.ts';

export {
  weightN,
  wingLoading,
  stallAngleRad,
  stallSpeedMps,
  inducedDragFactor,
  speedForLiftCoefficient,
  bestLiftToDrag,
  bestLiftToDragCoefficient,
  type AircraftSpec,
  type WingGeometry,
  type MassProperties,
  type LiftModel,
  type DragModel,
  type PitchModel,
  type LateralModel,
  type Propulsion,
  type ActuatorLimits,
} from './model/aircraft.ts';

export { C172, GLIDER, PRESETS, PRESET_NAMES } from './model/presets.ts';

export {
  controls,
  commandedSurfaces,
  slewActuators,
  NEUTRAL_CONTROLS,
  NEUTRAL_ACTUATORS,
  type ControlInputs,
  type ActuatorState,
} from './model/controls.ts';

export {
  aerodynamicCoefficients,
  flowAngles,
  separationFraction,
  MIN_AIRSPEED_FOR_ANGLES_MPS,
  type AeroCoefficients,
} from './aero/coefficients.ts';

export { aeroForcesBody, type AeroForces } from './aero/forces.ts';

export { propellerThrust, type ThrustResult } from './propulsion/propeller.ts';

export {
  evaluateDerivative,
  integrateState,
  initialState,
  stateFrom,
  altitudeFromState,
  isStateFinite,
  IDENTITY_STATE,
  type AircraftState,
  type AircraftDerivative,
  type DerivativeResult,
  type StepDiagnostics,
} from './dynamics/eom.ts';

export { rk4Step, type IntegrationResult } from './dynamics/integrator.ts';

export { buildTelemetry, type Telemetry } from './sim/telemetry.ts';

export {
  Simulation,
  type ControlScript,
  type FlightEvent,
  type RunOptions,
  type RunResult,
  type SimulationOptions,
  type SimulationStep,
} from './sim/simulation.ts';

export {
  trim,
  trimResiduals,
  layoutFor,
  climbRateFromExcessPower,
  type TrimRequest,
  type TrimSolution,
} from './sim/trim.ts';

export {
  altitudeHoldElevator,
  speedHoldThrottle,
  pitchHoldElevator,
  mergeControls,
  DEFAULT_ALTITUDE_HOLD,
  DEFAULT_SPEED_HOLD,
} from './scenarios/autopilot.ts';

export {
  takeoffScenario,
  cruiseHoldScenario,
  stallScenario,
  glideScenario,
  rollDoubletScenario,
  envelopeSweep,
  maxLevelSpeed,
  minimumLevelTrimSpeed,
  runScenario,
  SCENARIO_NAMES,
  type ScenarioName,
  type ScenarioOutcome,
  type EnvelopeRow,
  type MaxLevelSpeedResult,
} from './scenarios/flight.ts';
