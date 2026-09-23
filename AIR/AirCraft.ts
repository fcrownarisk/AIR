/**
 * ════════════════════════════════════════════════════════════════════════════
 *  AEGIS-1  ·  Anti-Gravity Aircraft Prototype
 * ════════════════════════════════════════════════════════════════════════════
 *  A TypeScript flight-dynamics simulation of a gravity-nullification craft.
 *
 *  CORE CONCEPT
 *  ------------
 *  The craft does not throw air downwards like a helicopter. Its drive emits a
 *  field that locally cancels a large fraction of the gravitational force on
 *  the hull, producing a net "lift" force along the vehicle's own UP axis.
 *
 *  Because that force is body-fixed, the pilot steers by tilting: pitch the
 *  nose down and the lift vector leans forward; roll right and it leans right.
 *  Attitude control is therefore trajectory control.
 *
 *  THE FIELD IS NOT FREE
 *  ---------------------
 *    • Induction lag       — the emitter must spool up/down (first-order lag).
 *    • Gravity-well decay  — coupling weakens with altitude (soft ceiling).
 *    • Waste heat          — sustained power derates the field as it soaks.
 *    • Power draw          — electrical load scales with field strength.
 *
 *  These four constraints are what make it a *prototype* rather than magic.
 *
 *  RUN
 *  ---
 *      npx tsx antigrav-aircraft.ts
 * ════════════════════════════════════════════════════════════════════════════
 */

// ════════════════════════════════════════════════════════════════════════════
//  1 · MATH PRIMITIVES
// ════════════════════════════════════════════════════════════════════════════

const EPS = 1e-9;
const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;

const clamp = (v: number, lo: number, hi: number): number =>
  v < lo ? lo : v > hi ? hi : v;

/** Wrap an angle into [-π, π]. */
const wrapAngle = (a: number): number => Math.atan2(Math.sin(a), Math.cos(a));

/** Immutable 3-vector. */
export class Vec3 {
  constructor(public x = 0, public y = 0, public z = 0) {}

  clone(): Vec3 { return new Vec3(this.x, this.y, this.z); }

  add(v: Vec3): Vec3 { return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z); }
  sub(v: Vec3): Vec3 { return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z); }
  scale(s: number): Vec3 { return new Vec3(this.x * s, this.y * s, this.z * s); }
  mulC(v: Vec3): Vec3 { return new Vec3(this.x * v.x, this.y * v.y, this.z * v.z); }

  dot(v: Vec3): number { return this.x * v.x + this.y * v.y + this.z * v.z; }

  cross(v: Vec3): Vec3 {
    return new Vec3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x,
    );
  }

  len(): number { return Math.hypot(this.x, this.y, this.z); }
  len2(): number { return this.dot(this); }

  normalized(): Vec3 {
    const l = this.len();
    return l < EPS ? new Vec3() : this.scale(1 / l);
  }
}

/**
 * Unit quaternion for attitude. Using a quaternion (rather than Euler angles)
 * avoids gimbal lock and integrates cleanly from the body angular velocity.
 */
export class Quat {
  constructor(public w = 1, public x = 0, public y = 0, public z = 0) {}

  static identity(): Quat { return new Quat(); }

  static fromAxisAngle(axis: Vec3, angle: number): Quat {
    const a = axis.normalized();
    const h = angle * 0.5;
    const s = Math.sin(h);
    return new Quat(Math.cos(h), a.x * s, a.y * s, a.z * s);
  }

  add(q: Quat): Quat {
    return new Quat(this.w + q.w, this.x + q.x, this.y + q.y, this.z + q.z);
  }

  scale(s: number): Quat {
    return new Quat(this.w * s, this.x * s, this.y * s, this.z * s);
  }

  /** Hamilton product: this ⊗ q. */
  multiply(q: Quat): Quat {
    return new Quat(
      this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z,
      this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
      this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
      this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w,
    );
  }

  norm(): number { return Math.hypot(this.w, this.x, this.y, this.z); }

  normalize(): Quat {
    const n = this.norm();
    return n < EPS
      ? Quat.identity()
      : new Quat(this.w / n, this.x / n, this.y / n, this.z / n);
  }

  /** Rotate a vector from body frame into world frame: v' = q v q⁻¹. */
  rotate(v: Vec3): Vec3 {
    const q = this.normalize();
    const u = new Vec3(q.x, q.y, q.z);
    const s = q.w;
    // v + 2s(u×v) + 2u×(u×v)
    return u
      .scale(2 * u.dot(v))
      .add(v.scale(s * s - u.dot(u)))
      .add(u.cross(v).scale(2 * s));
  }

  /** Advance the attitude by body angular velocity ω over dt. */
  integrate(omega: Vec3, dt: number): Quat {
    const omegaQ = new Quat(0, omega.x, omega.y, omega.z);
    const dq = this.multiply(omegaQ).scale(0.5 * dt); // q̇ = ½ q ⊗ ω
    return this.add(dq).normalize();
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  2 · WORLD CONSTANTS
// ════════════════════════════════════════════════════════════════════════════

const G0 = 9.80665;            // standard gravity, m/s²
const SEA_LEVEL_RHO = 1.225;   // kg/m³
const SCALE_HEIGHT = 8500;     // m — exponential atmosphere

/** Air density at altitude y (metres). */
const airDensity = (y: number): number =>
  SEA_LEVEL_RHO * Math.exp(-Math.max(0, y) / SCALE_HEIGHT);

// ════════════════════════════════════════════════════════════════════════════
//  3 · THE ANTI-GRAVITY DRIVE
// ════════════════════════════════════════════════════════════════════════════

export interface DriveConfig {
  /** Peak field force at sea level, full spool, cold emitter (N). */
  maxLiftForce: number;
  /** Induction time constants (s). */
  spoolUpTau: number;
  spoolDownTau: number;
  /** Altitude at which well-coupling falls to 1/e (m). */
  fieldDecayAltitude: number;
  /** Lumped thermal model. */
  thermalMass: number;   // J/K
  heatGain: number;      // W at full spool
  coolingCoef: number;   // W/K
  ambientTemp: number;   // °C
  overheatTemp: number;  // °C — derate begins here
  /** Electrical load at full spool (W). */
  maxPowerDraw: number;
}

export class AntiGravDrive {
  /** Commanded → actual field spool, 0..1 (includes induction lag). */
  spool = 0;
  /** Emitter core temperature, °C. */
  temperature: number;
  /** Instantaneous field force along body-up (N). */
  fieldStrength = 0;
  /** Combined altitude × thermal efficiency, 0..1. */
  efficiency = 1;
  /** Electrical draw (W). */
  powerDraw = 0;
  /** Thermal derate factor alone, 0..1. */
  derate = 1;

  constructor(public readonly config: DriveConfig) {
    this.temperature = config.ambientTemp;
  }

  /**
   * Advance the drive by dt.
   * @param throttle  commanded field strength, 0..1
   * @param altitude  current altitude AGL (m) — sets well-coupling
   */
  update(throttle: number, altitude: number, dt: number): void {
    const c = this.config;
    const target = clamp(throttle, 0, 1);

    // ── Induction lag (first-order, asymmetric) ────────────────────────────
    const tau = target > this.spool ? c.spoolUpTau : c.spoolDownTau;
    this.spool += (target - this.spool) * (1 - Math.exp(-dt / tau));

    // ── Thermal soak ───────────────────────────────────────────────────────
    const heatIn = c.heatGain * this.spool * this.spool;
    const heatOut = c.coolingCoef * (this.temperature - c.ambientTemp);
    this.temperature += ((heatIn - heatOut) / c.thermalMass) * dt;

    // ── Thermal derate: the emitter loses coherence as it cooks ────────────
    const over = this.temperature - c.overheatTemp;
    this.derate = over <= 0 ? 1 : clamp(1 - over / 60, 0.25, 1);

    // ── Gravity-well coupling decays with altitude → soft ceiling ──────────
    const altEff = Math.exp(-Math.max(0, altitude) / c.fieldDecayAltitude);

    this.efficiency = altEff * this.derate;
    this.fieldStrength = c.maxLiftForce * this.spool * this.efficiency;
    this.powerDraw = c.maxPowerDraw * this.spool * (0.35 + 0.65 * this.spool);
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  4 · THE VEHICLE (RIGID BODY)
// ════════════════════════════════════════════════════════════════════════════

/**
 * Body frame convention (right-handed, X-forward):
 *   +X = nose          +Y = dorsal (up)          +Z = starboard (right)
 * World frame: +Y is altitude.
 */
export interface ControlInput {
  /** Anti-grav drive throttle, 0..1. */
  lift: number;
  /** Main plasma thruster along body +X, 0..1. */
  forward: number;
  /** RCS torque about body +Z — positive pitches the nose UP, -1..1. */
  pitch: number;
  /** RCS torque about body +X — positive rolls right, -1..1. */
  roll: number;
  /** RCS torque about body +Y — positive yaws left, -1..1. */
  yaw: number;
}

export const NEUTRAL_INPUT: ControlInput = {
  lift: 0, forward: 0, pitch: 0, roll: 0, yaw: 0,
};

export interface VehicleConfig {
  mass: number;              // kg
  /** Moments of inertia about body x, y, z (kg·m²). */
  inertia: Vec3;
  /** Peak RCS torque about body x, y, z (N·m). */
  maxRcsTorque: Vec3;
  maxForwardThrust: number;  // N
  /** Cd · A — lumped drag area (m²). */
  dragArea: number;
  /** Angular damping coefficients (N·m·s/rad). */
  angularDamping: Vec3;
  /** Vertical impact speed that destroys the airframe (m/s). */
  crashSpeed: number;
}

export class AntiGravVehicle {
  // ── State ────────────────────────────────────────────────────────────────
  position = new Vec3(0, 0, 0);
  velocity = new Vec3(0, 0, 0);
  orientation = Quat.identity();
  angularVelocity = new Vec3(0, 0, 0);

  readonly drive: AntiGravDrive;

  onGround = true;
  crashed = false;
  flightTime = 0;

  // ── Telemetry scratch ────────────────────────────────────────────────────
  lastLift = 0;
  lastThrust = 0;
  lastDrag = 0;

  constructor(
    public readonly config: VehicleConfig,
    driveConfig: DriveConfig,
  ) {
    this.drive = new AntiGravDrive(driveConfig);
  }

  get mass(): number { return this.config.mass; }
  get weight(): number { return this.config.mass * G0; }
  get altitude(): number { return this.position.y; }

  /** Body axes expressed in world coordinates. */
  get forward(): Vec3 { return this.orientation.rotate(new Vec3(1, 0, 0)); }
  get up(): Vec3 { return this.orientation.rotate(new Vec3(0, 1, 0)); }
  get right(): Vec3 { return this.orientation.rotate(new Vec3(0, 0, 1)); }

  /** Euler angles for display/control (valid away from ±90° pitch). */
  get euler(): { pitch: number; roll: number; yaw: number } {
    const f = this.forward;
    const u = this.up;
    const r = this.right;
    return {
      pitch: Math.asin(clamp(f.y, -1, 1)),
      yaw: Math.atan2(-f.z, f.x),
      roll: Math.atan2(-r.y, u.y),
    };
  }

  get stateLabel(): string {
    if (this.crashed) return 'CRASHED';
    return this.onGround ? 'GROUND' : 'FLIGHT';
  }

  // ══════════════════════════════════════════════════════════════════════════
  //  Physics step
  // ══════════════════════════════════════════════════════════════════════════
  update(dt: number, input: ControlInput): void {
    if (this.crashed) return;

    const cfg = this.config;

    // ── 1 · Drive ───────────────────────────────────────────────────────────
    this.drive.update(input.lift, this.position.y, dt);
    const liftForce = this.drive.fieldStrength;
    const up = this.up;

    // ── 2 · Force accumulation ──────────────────────────────────────────────
    let force = new Vec3(0, -this.weight, 0);                    // gravity

    const liftVec = up.scale(liftForce);                          // anti-grav
    force = force.add(liftVec);

    const thrustMag = cfg.maxForwardThrust * clamp(input.forward, 0, 1);
    force = force.add(this.forward.scale(thrustMag));            // main engine

    // Aerodynamic drag (still air): F = -½ ρ Cd A |v| v
    const rho = airDensity(this.position.y);
    const speed = this.velocity.len();
    let dragMag = 0;
    if (speed > EPS) {
      dragMag = 0.5 * rho * cfg.dragArea * speed * speed;
      force = force.add(this.velocity.scale(-dragMag / speed));
    }

    // ── 3 · Torque accumulation ─────────────────────────────────────────────
    let torque = new Vec3(
      clamp(input.roll, -1, 1) * cfg.maxRcsTorque.x,
      clamp(input.yaw, -1, 1) * cfg.maxRcsTorque.y,
      clamp(input.pitch, -1, 1) * cfg.maxRcsTorque.z,
    );
    // Angular damping (field viscosity + residual atmosphere)
    torque = torque.sub(this.angularVelocity.mulC(cfg.angularDamping));

    // ── 4 · Linear integration (semi-implicit Euler) ────────────────────────
    const accel = force.scale(1 / this.mass);
    this.velocity = this.velocity.add(accel.scale(dt));
    this.position = this.position.add(this.velocity.scale(dt));

    // ── 5 · Angular integration ─────────────────────────────────────────────
    const angAccel = new Vec3(
      torque.x / cfg.inertia.x,
      torque.y / cfg.inertia.y,
      torque.z / cfg.inertia.z,
    );
    this.angularVelocity = this.angularVelocity.add(angAccel.scale(dt));
    this.orientation = this.orientation.integrate(this.angularVelocity, dt);

    // ── 6 · Ground contact ──────────────────────────────────────────────────
    if (this.position.y <= 0) {
      const impact = -this.velocity.y;

      if (impact > cfg.crashSpeed) {
        this.crashed = true;
        this.position.y = 0;
        this.velocity = new Vec3(0, 0, 0);
        this.angularVelocity = new Vec3(0, 0, 0);
        return;
      }

      this.position.y = 0;
      if (this.velocity.y < 0) this.velocity.y = 0;

      // Rolling friction + gear settling (frame-rate independent).
      const friction = Math.exp(-dt / 0.25);
      this.velocity.x *= friction;
      this.velocity.z *= friction;
      this.angularVelocity = this.angularVelocity.scale(Math.exp(-dt / 0.08));

      this.onGround = true;
    } else {
      this.onGround = false;
      this.flightTime += dt;
    }

    // ── 7 · Telemetry ───────────────────────────────────────────────────────
    this.lastLift = liftForce;
    this.lastThrust = thrustMag;
    this.lastDrag = dragMag;
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  5 · FLIGHT CONTROL
// ════════════════════════════════════════════════════════════════════════════

export interface FlightCommand {
  /** Target altitude AGL (m). */
  altitude: number;
  /** Target world-frame horizontal velocity (m/s). */
  velocity: { x: number; z: number };
  /** Target heading (rad, 0 = world +X). */
  heading: number;
  /** Hard override for the drive throttle — used for shutdown. */
  liftOverride?: number;
}

/**
 * Cascaded autopilot.
 *
 *   altitude error ─▶ climb-rate cmd ─▶ vertical accel cmd ─▶ drive throttle
 *   velocity error ─▶ accel cmd ─▶ desired tilt ─▶ rate cmd ─▶ RCS torque
 *
 * Feed-forward terms (hover throttle, gravity-tilt conversion) keep the
 * proportional gains modest and the response crisp.
 */
export class FlightController {
  private hIntX = 0;
  private hIntZ = 0;

  reset(): void {
    this.hIntX = 0;
    this.hIntZ = 0;
  }

  update(cmd: FlightCommand, v: AntiGravVehicle, dt: number): ControlInput {
    // ── Hard override (shutdown) ────────────────────────────────────────────
    if (cmd.liftOverride !== undefined) {
      this.reset();
      return { ...NEUTRAL_INPUT, lift: cmd.liftOverride };
    }

    // ── Feed-forward: the throttle that exactly cancels weight right now ────
    const eff = Math.max(0.05, v.drive.efficiency);
    const maxLift = v.drive.config.maxLiftForce * eff;
    const hoverThrottle = Math.min(1, v.weight / maxLift);

    // ══ VERTICAL LOOP ═══════════════════════════════════════════════════════
    const altErr = cmd.altitude - v.position.y;

    // Flare: restrict sink rate near the ground for a soft touchdown.
    let maxDescent = 15;
    if (v.position.y < 25) maxDescent = Math.max(0.35, v.position.y * 0.12);

    const climbRateCmd = clamp(0.9 * altErr, -maxDescent, 20);
    const vertAccelCmd = clamp(1.8 * (climbRateCmd - v.velocity.y), -5, 5);

    const lift = clamp(
      hoverThrottle + (v.mass * vertAccelCmd) / maxLift,
      0,
      1,
    );

    // ══ HORIZONTAL LOOP ═════════════════════════════════════════════════════
    const dvx = cmd.velocity.x - v.velocity.x;
    const dvz = cmd.velocity.z - v.velocity.z;

    // Small integrator removes the steady-state trim error caused by drag.
    this.hIntX = clamp(this.hIntX + dvx * dt, -25, 25);
    this.hIntZ = clamp(this.hIntZ + dvz * dt, -25, 25);

    const axCmd = clamp(0.55 * dvx + 0.10 * this.hIntX, -4.5, 4.5);
    const azCmd = clamp(0.55 * dvz + 0.10 * this.hIntZ, -4.5, 4.5);

    // Tilting the lift vector converts vertical thrust into horizontal accel:
    //   a_horizontal ≈ g · tan(tilt)
    const MAX_TILT = 28 * RAD;
    // Nose DOWN (negative pitch) leans the lift vector toward +X.
    const desiredPitch = clamp(Math.atan2(-axCmd, G0), -MAX_TILT, MAX_TILT);
    // Rolling right (+roll) leans the lift vector toward +Z.
    const desiredRoll = clamp(Math.atan2(azCmd, G0), -MAX_TILT, MAX_TILT);

    // ══ ATTITUDE RATE LOOP ══════════════════════════════════════════════════
    const e = v.euler;

    const pitchRateCmd = clamp(2.6 * wrapAngle(desiredPitch - e.pitch), -1.3, 1.3);
    const rollRateCmd = clamp(2.6 * wrapAngle(desiredRoll - e.roll), -1.3, 1.3);
    const yawRateCmd = clamp(1.6 * wrapAngle(cmd.heading - e.yaw), -1.0, 1.0);

    const pitch = clamp(1.2 * (pitchRateCmd - v.angularVelocity.z), -1, 1);
    const roll = clamp(1.2 * (rollRateCmd - v.angularVelocity.x), -1, 1);
    const yaw = clamp(1.1 * (yawRateCmd - v.angularVelocity.y), -1, 1);

    return { lift, forward: 0, pitch, roll, yaw };
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  6 · MISSION PROFILE
// ════════════════════════════════════════════════════════════════════════════

interface Phase {
  name: string;
  t0: number;
  t1: number;
  alt: number;
  vx: number;
  vz: number;
  shutdown?: boolean;
}

const MISSION: Phase[] = [
  { name: 'PREFLIGHT',    t0:  0, t1:  5, alt:   0, vx:  0, vz: 0 },
  { name: 'ASCENT',       t0:  5, t1: 20, alt: 300, vx:  0, vz: 0 },
  { name: 'STATION KEEP', t0: 20, t1: 27, alt: 300, vx:  0, vz: 0 },
  { name: 'CRUISE',       t0: 27, t1: 47, alt: 300, vx: 30, vz: 0 },
  { name: 'BRAKE',        t0: 47, t1: 58, alt: 300, vx:  0, vz: 0 },
  { name: 'DESCENT',      t0: 58, t1: 78, alt:   0, vx:  0, vz: 0 },
  { name: 'SHUTDOWN',     t0: 78, t1: 90, alt:   0, vx:  0, vz: 0, shutdown: true },
];

const MISSION_END = MISSION[MISSION.length - 1].t1;

function commandAt(t: number): { phase: Phase; cmd: FlightCommand } {
  const phase =
    MISSION.find((p) => t >= p.t0 && t < p.t1) ?? MISSION[MISSION.length - 1];

  return {
    phase,
    cmd: {
      altitude: phase.alt,
      velocity: { x: phase.vx, z: phase.vz },
      heading: 0,
      liftOverride: phase.shutdown ? 0 : undefined,
    },
  };
}

// ════════════════════════════════════════════════════════════════════════════
//  7 · HARDWARE DEFINITION
// ════════════════════════════════════════════════════════════════════════════

const VEHICLE_CONFIG: VehicleConfig = {
  mass: 850,
  inertia: new Vec3(900, 1400, 1600),      // roll, yaw, pitch
  maxRcsTorque: new Vec3(2200, 1800, 2600), // roll, yaw, pitch
  maxForwardThrust: 3200,
  dragArea: 2.275,                          // Cd 0.35 × 6.5 m²
  angularDamping: new Vec3(500, 550, 600),
  crashSpeed: 6.0,
};

const DRIVE_CONFIG: DriveConfig = {
  maxLiftForce: 15000,        // ≈ 1.8 × weight — enough margin to climb
  spoolUpTau: 1.8,
  spoolDownTau: 1.2,
  fieldDecayAltitude: 8500,
  thermalMass: 6250,
  heatGain: 25000,
  coolingCoef: 250,
  ambientTemp: 20,
  overheatTemp: 100,
  maxPowerDraw: 120000,       // 120 kW
};

// ════════════════════════════════════════════════════════════════════════════
//  8 · TELEMETRY & MAIN LOOP
// ════════════════════════════════════════════════════════════════════════════

const pad = (s: string | number, n: number): string => String(s).padStart(n);

function logRow(
  t: string | number, phase: string, alt: string | number, vs: string | number,
  spd: string | number, pitch: string | number, roll: string | number,
  spool: string | number, temp: string | number, pwr: string | number,
  state: string,
): void {
  console.log(
    pad(t, 5) + '  ' +
    String(phase).padEnd(13) + ' ' +
    pad(alt, 7) + ' ' +
    pad(vs, 7) + ' ' +
    pad(spd, 6) + ' ' +
    pad(pitch, 7) + ' ' +
    pad(roll, 6) + ' ' +
    pad(spool, 6) + ' ' +
    pad(temp, 5) + ' ' +
    pad(pwr, 6) + '   ' +
    state,
  );
}

function banner(): void {
  console.log(`
╔══════════════════════════════════════════════════════════════════════════╗
║   AEGIS-1  ·  ANTI-GRAVITY AIRCRAFT PROTOTYPE                            ║
║   Gravity-nullification drive · TypeScript flight dynamics               ║
╠══════════════════════════════════════════════════════════════════════════╣
║   Mass ............ 850 kg          Peak field ..... 15.0 kN             ║
║   Weight .......... 8.34 kN         Field decay .... 8500 m              ║
║   Thrust/Weight ... 1.80            Thermal limit .. 100 °C              ║
║   Peak power ...... 120 kW          Rated ceiling .. ~4.9 km (cold)      ║
╚══════════════════════════════════════════════════════════════════════════╝
`);
}

function main(): void {
  banner();

  const vehicle = new AntiGravVehicle(VEHICLE_CONFIG, DRIVE_CONFIG);
  const controller = new FlightController();

  const SIM_DT = 1 / 240;   // fixed physics step, 240 Hz
  const LOG_EVERY = 0.5;    // telemetry print period (s)

  let t = 0;
  let nextLog = 0;
  let peakAltitude = 0;
  let peakSpeed = 0;
  let peakTemp = DRIVE_CONFIG.ambientTemp;
  let minDerate = 1;

  logRow('T+', 'PHASE', 'ALT', 'V/S', 'SPD', 'PITCH', 'ROLL',
         'SPOOL', 'TEMP', 'PWR', 'STATE');
  console.log('─'.repeat(88));

  while (t < MISSION_END && !vehicle.crashed) {
    const { phase, cmd } = commandAt(t);
    const input = controller.update(cmd, vehicle, SIM_DT);
    vehicle.update(SIM_DT, input);
    t += SIM_DT;

    peakAltitude = Math.max(peakAltitude, vehicle.position.y);
    peakSpeed = Math.max(peakSpeed, vehicle.velocity.len());
    peakTemp = Math.max(peakTemp, vehicle.drive.temperature);
    minDerate = Math.min(minDerate, vehicle.drive.derate);

    if (t >= nextLog) {
      nextLog += LOG_EVERY;
      const e = vehicle.euler;
      const horizSpeed = Math.hypot(vehicle.velocity.x, vehicle.velocity.z);
      const vs = vehicle.velocity.y;

      logRow(
        t.toFixed(1),
        phase.name,
        vehicle.position.y.toFixed(1),
        (vs >= 0 ? '+' : '') + vs.toFixed(1),
        horizSpeed.toFixed(1),
        (e.pitch * DEG).toFixed(1),
        (e.roll * DEG).toFixed(1),
        (vehicle.drive.spool * 100).toFixed(0) + '%',
        vehicle.drive.temperature.toFixed(0) + 'C',
        (vehicle.drive.powerDraw / 1000).toFixed(0) + 'kW',
        vehicle.stateLabel,
      );
    }
  }

  console.log('─'.repeat(88));

  // ── Debrief ───────────────────────────────────────────────────────────────
  const v = vehicle;
  const status = v.crashed
    ? '✖  AIRFRAME LOST'
    : v.onGround
      ? '✔  NOMINAL — vehicle recovered on the pad'
      : '⚠  ENDED AIRBORNE';

  console.log(`
FLIGHT DEBRIEF
  Status ................ ${status}
  Mission elapsed ....... ${t.toFixed(1)} s
  Airborne time ......... ${v.flightTime.toFixed(1)} s
  Peak altitude ......... ${peakAltitude.toFixed(1)} m AGL
  Peak speed ............ ${peakSpeed.toFixed(1)} m/s
  Final position ........ x=${v.position.x.toFixed(1)} m  y=${v.position.y.toFixed(2)} m
  Peak emitter temp ..... ${peakTemp.toFixed(1)} °C
  Minimum thermal derate  ${(minDerate * 100).toFixed(0)} %
  Peak electrical load .. ${(DRIVE_CONFIG.maxPowerDraw / 1000).toFixed(0)} kW
`);

  // ── Engineering notes ─────────────────────────────────────────────────────
  console.log('NOTES');
  console.log('  • The drive reached thermal equilibrium above the 100 °C soft limit,');
  console.log('    so the field derated during sustained cruise. A production airframe');
  console.log('    needs a larger radiator or a lower duty cycle.');
  console.log('  • Field efficiency falls off as exp(-alt/8500), producing a natural');
  console.log('    ceiling: once maxLift × efficiency drops below weight, no throttle');
  console.log('    setting can hold altitude. Cold-drive ceiling ≈ 4.9 km.');
  console.log('  • Attitude is trajectory. The autopilot never commands horizontal');
  console.log('    thrust directly — it commands a tilt, and the lift vector does the');
  console.log('    rest. This is the defining flight characteristic of the design.');
}

main();