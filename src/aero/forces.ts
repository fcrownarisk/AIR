/**
 * Aerodynamic forces and moments in the BODY frame.
 *
 * Aerodynamic data is naturally expressed in WIND axes: drag along the velocity
 * vector, lift perpendicular to it, side force across it. The equations of
 * motion are written in BODY axes, so the resultant has to be rotated by
 * (alpha, beta).
 *
 * Derivation, written out because this transform is where sign errors live.
 * Wind axes expressed in body coordinates (Stevens & Lewis convention):
 *
 *   x_w = [ cos a cos b,  sin b,          sin a cos b  ]   (along velocity)
 *   y_w = [-cos a sin b,  cos b,         -sin a sin b  ]   (to the right)
 *   z_w = [-sin a,        0,              cos a        ]   (perpendicular, "down")
 *
 * The `cos b` on the third element of x_w is easy to drop and easy to miss:
 * the velocity components are u = V cos a cos b, v = V sin b, w = V sin a cos b.
 * Writing x_w as [cos a cos b, sin b, sin a] gives a vector whose norm is
 * sqrt(1 + sin^2 a sin^2 b), i.e. NOT a unit vector whenever sideslip is
 * non-zero, and the body force then comes out up to 30-40% wrong in magnitude.
 * It is invisible at beta = 0, which is exactly why the test suite asserts
 * |F_body| == |F_wind| over a sweep of BOTH angles rather than only alpha.
 *
 * Check at a = b = 0: x_w = [1,0,0], y_w = [0,1,0], z_w = [0,0,1], and
 * x_w x y_w = z_w, so the triad is right-handed and orthonormal.
 *
 * In wind axes the force vector is F_w = (-D, Y, -L): drag opposes x_w, side
 * force acts along y_w, and lift opposes z_w (which points down-ish). Rotating
 * into body axes by taking components along each body axis:
 *
 *   F_b = (-D) x_w + (Y) y_w + (-L) z_w
 *
 * which expands to
 *
 *   X = L sin a - D cos a cos b - Y cos a sin b
 *   Y = Y cos b - D sin b
 *   Z = -L cos a - D sin a cos b - Y sin a sin b
 *
 * Sanity check on the sign of Z: body z points DOWN, so positive lift (L > 0)
 * must produce a NEGATIVE Z. The first term is -L cos a, which is negative for
 * lifting flight. Correct.
 *
 * Moments use the usual reference lengths:
 *   rolling  = qbar S b Cl
 *   pitching = qbar S cbar Cm
 *   yawing   = qbar S b Cn
 */

import { type AircraftSpec } from '../model/aircraft.ts';
import { vec3, type Vec3 } from '../math/vec3.ts';
import { type AeroCoefficients } from './coefficients.ts';

export interface AeroForces {
  /** Aerodynamic force in the body frame (N). */
  readonly forceBody: Vec3;
  /** Aerodynamic moment in the body frame (N m), about body x, y, z. */
  readonly momentBody: Vec3;
  /** Lift force magnitude along the wind axis (N). */
  readonly liftN: number;
  /** Drag force magnitude along the wind axis (N). */
  readonly dragN: number;
  /** Side force magnitude along the wind axis (N). */
  readonly sideN: number;
}

export function aeroForcesBody(spec: AircraftSpec, coef: AeroCoefficients): AeroForces {
  const qbar = coef.dynamicPressurePa;
  const S = spec.wing.areaM2;
  const b = spec.wing.spanM;
  const cbar = spec.wing.chordM;

  const liftN = qbar * S * coef.lift;
  const dragN = qbar * S * coef.drag;
  const sideN = qbar * S * coef.sideForce;

  const ca = Math.cos(coef.alphaRad);
  const sa = Math.sin(coef.alphaRad);
  const cb = Math.cos(coef.betaRad);
  const sb = Math.sin(coef.betaRad);

  const forceBody = vec3(
    liftN * sa - dragN * ca * cb - sideN * ca * sb,
    sideN * cb - dragN * sb,
    -liftN * ca - dragN * sa * cb - sideN * sa * sb,
  );

  const momentBody = vec3(
    qbar * S * b * coef.rollingMoment,
    qbar * S * cbar * coef.pitchingMoment,
    qbar * S * b * coef.yawingMoment,
  );

  return { forceBody, momentBody, liftN, dragN, sideN };
}
