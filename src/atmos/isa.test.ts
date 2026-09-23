/**
 * ISA verification against the published standard atmosphere table.
 *
 * The comparison table is the real published ISA, not values generated from
 * this implementation — otherwise the test would only prove the code agrees
 * with itself.
 */

import { test } from 'node:test';
import assert from 'node:assert/strict';

import {
  SEA_LEVEL,
  geometricToGeopotential,
  geopotentialToGeometric,
  isa,
  isaAtGeometricAltitude,
  VALID_CEILING_M,
} from './isa.ts';

/** Published ISA: geopotential altitude (m), temperature (K), pressure (Pa), density (kg/m^3), speed of sound (m/s). */
const PUBLISHED: readonly (readonly [number, number, number, number, number])[] = [
  [0, 288.15, 101325, 1.225, 340.294],
  [1000, 281.65, 89874.6, 1.11164, 336.434],
  [2000, 275.15, 79495.2, 1.00649, 332.532],
  [5000, 255.65, 54019.9, 0.736116, 320.529],
  [8000, 236.15, 35600.5, 0.525168, 308.057],
  [11000, 216.65, 22632.1, 0.363918, 295.07],
  [15000, 216.65, 12044.6, 0.193674, 295.07],
  [20000, 216.65, 5474.89, 0.0880349, 295.07],
  [32000, 228.65, 868.019, 0.013225, 303.131],
];

test('sea level constants match the ISA definition', () => {
  assert.equal(SEA_LEVEL.temperatureK, 288.15);
  assert.equal(SEA_LEVEL.pressurePa, 101325);
  assert.equal(SEA_LEVEL.densityKgM3, 1.225);
});

test('atmosphere matches the published ISA table to better than 1e-4 relative', () => {
  // Measured worst case is 2.0e-5; the residual is rounding in the published
  // table's own coefficients, not error in this implementation.
  const tolerance = 1e-4;
  for (const [H, temperatureK, pressurePa, densityKgM3, speedOfSoundMps] of PUBLISHED) {
    const at = isa(H);
    assert.ok(
      Math.abs(at.temperatureK - temperatureK) / temperatureK < tolerance,
      `temperature at H=${H}: got ${at.temperatureK}, published ${temperatureK}`,
    );
    assert.ok(
      Math.abs(at.pressurePa - pressurePa) / pressurePa < tolerance,
      `pressure at H=${H}: got ${at.pressurePa}, published ${pressurePa}`,
    );
    assert.ok(
      Math.abs(at.densityKgM3 - densityKgM3) / densityKgM3 < tolerance,
      `density at H=${H}: got ${at.densityKgM3}, published ${densityKgM3}`,
    );
    assert.ok(
      Math.abs(at.speedOfSoundMps - speedOfSoundMps) / speedOfSoundMps < tolerance,
      `speed of sound at H=${H}: got ${at.speedOfSoundMps}, published ${speedOfSoundMps}`,
    );
  }
});

test('pressure decreases monotonically and the tropopause temperature is exact', () => {
  let previousPressure = Number.POSITIVE_INFINITY;
  for (let H = 0; H <= VALID_CEILING_M; H += 250) {
    const at = isa(H);
    assert.ok(at.pressurePa < previousPressure, `pressure not decreasing at H=${H}`);
    assert.ok(at.densityKgM3 > 0);
    previousPressure = at.pressurePa;
  }
  // Isothermal layer between 11 km and 20 km. The comparison is a tolerance and
  // not an equality: 288.15 K and the 6.5 K/km lapse are not exactly
  // representable in binary, so the modelled value lands ~3e-14 K off the
  // published decimal.
  assert.ok(Math.abs(isa(11000).temperatureK - 216.65) < 1e-9, `tropopause T is ${isa(11000).temperatureK}`);
  assert.ok(Math.abs(isa(20000).temperatureK - 216.65) < 1e-9, `20 km T is ${isa(20000).temperatureK}`);
});

test('geopotential and geometric altitude conversions are inverses', () => {
  for (const h of [0, 1000, 11000, 20000, 50000]) {
    const H = geometricToGeopotential(h);
    assert.ok(Math.abs(geopotentialToGeometric(H) - h) < 1e-6, `roundtrip failed at h=${h}`);
  }
  // The two altitudes are equal at sea level and differ by ~19 m at 11 km.
  assert.equal(geometricToGeopotential(0), 0);
  assert.ok(Math.abs(11000 - geometricToGeopotential(11000) - 19) < 2);
});

test('geometric-altitude lookup differs from geopotential lookup by the conversion', () => {
  const geometric = 11000;
  const H = geometricToGeopotential(geometric);
  const viaGeometric = isaAtGeometricAltitude(geometric);
  assert.equal(viaGeometric.geopotentialAltitudeM, H);
  // Feeding geometric altitude straight into isa() is the classic error; at
  // 11 km the temperatures differ by about 0.12 K.
  assert.ok(Math.abs(viaGeometric.temperatureK - isa(11000).temperatureK) > 0.05);
});

test('altitudes above the model ceiling are flagged as extrapolated, not silently accepted', () => {
  assert.equal(isa(VALID_CEILING_M - 1).extrapolated, false);
  assert.equal(isa(VALID_CEILING_M + 1).extrapolated, true);
  assert.equal(isa(-1).extrapolated, true);
});
