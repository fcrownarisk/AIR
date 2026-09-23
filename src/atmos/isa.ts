/**
 * International Standard Atmosphere (ISA).
 *
 * The ISA is defined in terms of GEOPOTENTIAL altitude H, not geometric
 * altitude h. The two differ because gravity weakens with height:
 *
 *   H = Re h / (Re + h)        Re = 6 356 766 m
 *
 * At 11 km the difference is ~19 m, which is visible at the 1e-4 level in
 * temperature. Published ISA tables are tabulated against H, so `isa()` takes
 * geopotential altitude to match them exactly, and `isaAtGeometricAltitude()`
 * applies the conversion for states coming out of the simulator.
 *
 * Layers implemented: 0-11 km (troposphere, lapse -6.5 K/km), 11-20 km
 * (stratosphere, isothermal 216.65 K), 20-32 km (lapse +1.0 K/km). Above 32 km
 * the last layer is extrapolated, which is flagged rather than silently
 * accepted — see `ModelLimits`.
 */

export const SEA_LEVEL = {
  temperatureK: 288.15,
  pressurePa: 101325,
  densityKgM3: 1.225,
  speedOfSoundMps: 340.294,
} as const;

/** Standard gravity (m/s^2), as used in the ISA definition. */
export const G0 = 9.80665;
/** Specific gas constant for dry air (J/(kg K)). */
export const R_AIR = 287.05287;
/** Ratio of specific heats for air. */
export const GAMMA_AIR = 1.4;
/** Effective Earth radius used for the geopotential conversion (m). */
export const EARTH_RADIUS_M = 6356766;

export const TROPOPAUSE_GEOPOTENTIAL_M = 11000;
export const STRATOPAUSE_GEOPOTENTIAL_M = 20000;
export const MODEL_CEILING_GEOPOTENTIAL_M = 32000;

/** Height at which the model stops being a standard atmosphere and starts extrapolating. */
export const VALID_CEILING_M = 32000;

export interface Atmosphere {
  /** Geopotential altitude used for the lookup (m). */
  readonly geopotentialAltitudeM: number;
  /** Geometric altitude corresponding to that geopotential altitude (m). */
  readonly geometricAltitudeM: number;
  /** Static temperature (K). */
  readonly temperatureK: number;
  /** Static pressure (Pa). */
  readonly pressurePa: number;
  /** Density (kg/m^3). */
  readonly densityKgM3: number;
  /** Speed of sound (m/s). */
  readonly speedOfSoundMps: number;
  /** Density ratio rho / rho0, used for engine power lapse. */
  readonly densityRatio: number;
  /** True when the altitude is above VALID_CEILING_M and the result is extrapolated. */
  readonly extrapolated: boolean;
}

interface Layer {
  /** Base geopotential altitude of the layer (m). */
  readonly baseM: number;
  /** Base temperature (K). */
  readonly baseTempK: number;
  /** Temperature lapse rate (K/m). Zero means isothermal. */
  readonly lapse: number;
  /** Pressure at the layer base (Pa). */
  readonly basePressurePa: number;
}

/**
 * Layer base pressures are computed from the layer recurrence rather than
 * hard-coded, so the table cannot drift away from the physics.
 */
function buildLayers(): Layer[] {
  const layers: Layer[] = [
    { baseM: 0, baseTempK: SEA_LEVEL.temperatureK, lapse: -0.0065, basePressurePa: SEA_LEVEL.pressurePa },
  ];

  const lapse0 = layers[0]!;
  const tempAtTop = lapse0.baseTempK + lapse0.lapse * TROPOPAUSE_GEOPOTENTIAL_M;
  const pressureAtTop =
    lapse0.basePressurePa * Math.pow(tempAtTop / lapse0.baseTempK, G0 / (Math.abs(lapse0.lapse) * R_AIR));
  layers.push({ baseM: 11000, baseTempK: tempAtTop, lapse: 0, basePressurePa: pressureAtTop });

  const lapse1 = layers[1]!;
  const pressureAtStratopause =
    lapse1.basePressurePa * Math.exp((-G0 * 9000) / (R_AIR * lapse1.baseTempK));
  layers.push({ baseM: 20000, baseTempK: lapse1.baseTempK, lapse: 0.001, basePressurePa: pressureAtStratopause });

  return layers;
}

const LAYERS = buildLayers();

export const geometricToGeopotential = (geometricM: number): number =>
  (EARTH_RADIUS_M * geometricM) / (EARTH_RADIUS_M + geometricM);

export const geopotentialToGeometric = (geopotentialM: number): number =>
  (EARTH_RADIUS_M * geopotentialM) / (EARTH_RADIUS_M - geopotentialM);

/** Atmosphere at a given GEOPOTENTIAL altitude. Matches published ISA tables directly. */
export function isa(geopotentialAltitudeM = 0): Atmosphere {
  const H = geopotentialAltitudeM;

  let layer = LAYERS[0]!;
  for (const candidate of LAYERS) {
    if (H >= candidate.baseM) layer = candidate;
  }

  const temperatureK = layer.baseTempK + layer.lapse * (H - layer.baseM);

  // Hydrostatic integration. Isothermal layers use the exponential form; layers
  // with a lapse use the power form, which is the limit of the same integral.
  //
  // Note the sign: `lapse` is dT/dH (negative in the troposphere), so the
  // exponent is -g0/(lapse*R), which is POSITIVE below the tropopause. Writing
  // g0/(lapse*R) here is the classic sign error and inverts the pressure profile.
  const pressurePa =
    layer.lapse === 0
      ? layer.basePressurePa * Math.exp((-G0 * (H - layer.baseM)) / (R_AIR * layer.baseTempK))
      : layer.basePressurePa * Math.pow(temperatureK / layer.baseTempK, -G0 / (layer.lapse * R_AIR));

  const densityKgM3 = pressurePa / (R_AIR * temperatureK);
  const speedOfSoundMps = Math.sqrt(GAMMA_AIR * R_AIR * temperatureK);

  return {
    geopotentialAltitudeM: H,
    geometricAltitudeM: geopotentialToGeometric(H),
    temperatureK,
    pressurePa,
    densityKgM3,
    speedOfSoundMps,
    densityRatio: densityKgM3 / SEA_LEVEL.densityKgM3,
    extrapolated: H > VALID_CEILING_M || H < 0,
  };
}

/** Atmosphere at a given GEOMETRIC altitude, applying the standard conversion. */
export function isaAtGeometricAltitude(geometricAltitudeM = 0): Atmosphere {
  return isa(geometricToGeopotential(geometricAltitudeM));
}
