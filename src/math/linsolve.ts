/**
 * Dense linear solver for the small systems appearing in this project (3x3
 * Jacobians). Gaussian elimination with partial pivoting.
 *
 * Returns null rather than throwing when the matrix is singular, because a
 * singular trim Jacobian is a legitimate physical outcome (an attainable
 * flight condition that the parameterisation cannot reach), not a bug. The
 * caller reports it as non-convergence.
 */

export function solveLinearSystem(a: Float64Array, b: Float64Array, n: number): Float64Array | null {
  const m = Float64Array.from(a);
  const rhs = Float64Array.from(b);

  for (let col = 0; col < n; col++) {
    // Partial pivoting: swap in the row with the largest magnitude in this column.
    let pivotRow = col;
    let pivotValue = Math.abs(m[col * n + col]!);
    for (let row = col + 1; row < n; row++) {
      const candidate = Math.abs(m[row * n + col]!);
      if (candidate > pivotValue) {
        pivotValue = candidate;
        pivotRow = row;
      }
    }

    if (pivotValue < 1e-14) return null;

    if (pivotRow !== col) {
      for (let k = 0; k < n; k++) {
        const tmp = m[col * n + k]!;
        m[col * n + k] = m[pivotRow * n + k]!;
        m[pivotRow * n + k] = tmp;
      }
      const tmp = rhs[col]!;
      rhs[col] = rhs[pivotRow]!;
      rhs[pivotRow] = tmp;
    }

    const pivot = m[col * n + col]!;
    for (let row = col + 1; row < n; row++) {
      const factor = m[row * n + col]! / pivot;
      if (factor === 0) continue;
      for (let k = col; k < n; k++) {
        m[row * n + k] = m[row * n + k]! - factor * m[col * n + k]!;
      }
      rhs[row] = rhs[row]! - factor * rhs[col]!;
    }
  }

  // Back substitution.
  const x = new Float64Array(n);
  for (let row = n - 1; row >= 0; row--) {
    let sum = rhs[row]!;
    for (let k = row + 1; k < n; k++) sum -= m[row * n + k]! * x[k]!;
    x[row] = sum / m[row * n + row]!;
  }

  return x.every(Number.isFinite) ? x : null;
}
