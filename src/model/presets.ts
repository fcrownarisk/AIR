/**
 * Aircraft presets.
 *
 * C172 — a Cessna 172N-class four-seat trainer. Geometry, mass and inertia are
 * the published values for that airframe; the non-dimensional derivatives are
 * the standard published stability-derivative set for the type. Two modelling
 * caveats worth stating plainly rather than hiding:
 *
 *  1. CD0 = 0.030 is a clean-configuration parasite drag coefficient for a
 *     fixed-gear aeroplane. Real total drag at a given speed includes cooling
 *     drag and trim drag; treat absolute cruise speed as accurate to a few knots,
 *     not to a tenth.
 *
 *  2. `maxShaftPowerW` is rated power (160 hp). A normally aspirated engine
 *     loses power with density, which the thrust model applies as
 *     P_available = throttle * P_max * (rho/rho0). Consequently a "75% power"
 *     cruise at 5000 ft corresponds to roughly 86% throttle in this model,
 *     because 75% of SEA-LEVEL rated power is more than the engine can produce
 *     at that altitude. That is a property of how power is quoted, not a bug.
 *
 * GLIDER — an unpowered 15 m sailplane, used to exercise the glide and
 * best-L/D paths with an aeroplane that has no engine at all.
 */

import { type AircraftSpec } from './aircraft.ts';

/** Cessna 172N-class: 1043 kg, 16.17 m^2 wing, 160 hp, fixed gear. */
export const C172: AircraftSpec = {
  name: 'C172 (four-seat trainer)',
  mass: {
    massKg: 1043,
    // Published inertias for the type, converted from slug-ft^2.
    ixx: 1285,
    iyy: 1825,
    izz: 2667,
  },
  wing: {
    areaM2: 16.17,
    spanM: 11.0,
    chordM: 1.47,
    aspectRatio: 7.48,
  },
  lift: {
    CLAlpha: 4.9,
    alpha0Rad: -0.05,
    CLMax: 1.6,
    stallBreakWidthRad: 0.12,
  },
  drag: {
    CD0: 0.03,
    oswaldE: 0.75,
  },
  pitch: {
    Cm0: 0.05,
    CmAlpha: -0.89,
    CmQ: -12.4,
    CmElevator: -1.28,
  },
  lateral: {
    CYBeta: -0.31,
    CYRudder: -0.12,
    ClBeta: -0.089,
    ClAileron: 0.178,
    CnAileron: -0.012,
    ClP: -0.47,
    ClR: 0.096,
    CnBeta: 0.065,
    CnRudder: 0.09,
    CnP: -0.03,
    CnR: -0.15,
  },
  propulsion: {
    maxShaftPowerW: 119312, // 160 hp
    propEfficiency: 0.8,
    staticThrustN: 2400,
    staticSpeedFloorMps: 5,
  },
  limits: {
    maxElevatorRad: 0.35, // 20 deg
    maxAileronRad: 0.3, // 17 deg
    maxRudderRad: 0.35, // 20 deg
    elevatorRateRadPerS: 1.05, // 60 deg/s
    aileronRateRadPerS: 1.4, // 80 deg/s
    rudderRateRadPerS: 1.05,
  },
};

/** 15 m unpowered sailplane, mass 350 kg. Best L/D ~44.6 by design. */
export const GLIDER: AircraftSpec = {
  name: 'Glider (15 m class sailplane)',
  mass: {
    massKg: 350,
    ixx: 1400,
    iyy: 2600,
    izz: 3800,
  },
  wing: {
    areaM2: 10.0,
    spanM: 15.0,
    chordM: 0.667,
    aspectRatio: 22.5,
  },
  lift: {
    CLAlpha: 5.6,
    alpha0Rad: -0.03,
    CLMax: 1.35,
    stallBreakWidthRad: 0.1,
  },
  drag: {
    CD0: 0.008,
    oswaldE: 0.9,
  },
  pitch: {
    Cm0: 0.0,
    CmAlpha: -0.6,
    CmQ: -14.0,
    CmElevator: -1.1,
  },
  lateral: {
    CYBeta: -0.35,
    CYRudder: -0.1,
    ClBeta: -0.2,
    ClAileron: 0.16,
    CnAileron: -0.01,
    ClP: -0.5,
    ClR: 0.1,
    CnBeta: 0.08,
    CnRudder: 0.08,
    CnP: -0.03,
    CnR: -0.14,
  },
  propulsion: {
    maxShaftPowerW: 0,
    propEfficiency: 0,
    staticThrustN: 0,
    staticSpeedFloorMps: 5,
  },
  limits: {
    maxElevatorRad: 0.35,
    maxAileronRad: 0.35,
    maxRudderRad: 0.4,
    elevatorRateRadPerS: 1.05,
    aileronRateRadPerS: 1.4,
    rudderRateRadPerS: 1.05,
  },
};

export const PRESETS: Readonly<Record<string, AircraftSpec>> = { C172, GLIDER };

export const PRESET_NAMES: readonly string[] = Object.keys(PRESETS);
