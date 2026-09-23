# AirCraft

A headless 6-DOF flight-dynamics sandbox in TypeScript. Zero runtime
dependencies. No build step — Node runs the `.ts` sources directly.

It is a *library plus a text HUD*, not a game and not a flight simulator in the
graphical sense. The design goal was the opposite of a demo: every physical claim
the model makes is either checked numerically against an independent reference or
declared as a known limitation in the code and in this file. Nothing here is
"expected to work" — it is measured.

Two airframes ship: a Cessna 172N-class four-seat trainer and an unpowered 15 m
sailplane. Five scripted scenarios exercise them: takeoff, cruise, stall, glide,
and a roll doublet.

---

## Quick start

Requires Node 22.18 or newer (native TypeScript type stripping is unflagged from
that version).

```
node src/cli.ts list                      # aircraft and scenarios
node src/cli.ts info C172                 # derived performance figures
node src/cli.ts run takeoff --hud         # run a scenario, print the trace
node src/cli.ts run glide --hud --every 20
node src/cli.ts trim C172 1500 60         # solve a trimmed flight condition
node src/cli.ts trim GLIDER 600 29 --throttle 0
node src/cli.ts envelope C172             # trim sweep across the envelope

node --test                               # the verification suite (65 tests)
node scripts/tsc.mjs -p tsconfig.json     # strict typecheck
```

`npm run typecheck` and `npm test` are the same two commands. The typechecker
lives **outside** the project tree (see `scripts/tsc.mjs`), so the project stays
`node_modules`-free; `npm run where` prints which toolchain it resolved.

---

## Conventions

Getting these wrong is the most common source of silent error in a model like
this, so they are stated once, precisely, and referenced from the code.

**Frames.** Two right-handed frames.

| | x | y | z |
|---|----|----|----|
| body | forward | right | **down** |
| NED  | north   | east  | **down** |

**Attitude.** Body-to-NED, carried as a unit quaternion. Euler angles are
extracted 3-2-1: `phi` roll, `theta` pitch, `psi` yaw, all radians. The quaternion
is used for propagation because the kinematic relation `q̇ = ½ q ⊗ [0,p,q,r]` has
no singularity at 90° of pitch, which Euler-rate propagation does.

**Air data.** `alpha = atan2(w, u)` (nose above the velocity vector, positive),
`beta = asin(v / V)` (velocity has a rightward component, positive).

**Angular rates.** `p`, `q`, `r` in body axes, rad/s. The non-dimensional
derivatives act on `p·b/(2V)`, `q·c̄/(2V)`, `r·b/(2V)`.

**Signs that trip people up.**

- Control surfaces positive = trailing edge **down**. The pilot's stick-back
  elevator command is therefore inverted when mapped to a surface angle
  (`model/controls.ts`), and `CmElevator` is negative because a TE-down surface
  produces a nose-down moment.
- Aileron command positive = roll right.
- Body-z lift appears as a **negative** body-z force, because body z points down.
- Altitude is `-positionNed.z`, since NED z is down.

**Load factor.** The reported `loadFactor` is the body-z specific force over g —
the reading of an accelerometer whose axis is normal to the fuselage. At a
trimmed condition it is exactly `cos(theta)`, **not** 1.0: at 2.55° of pitch it
reads 0.99901. The familiar "1 g in level flight" refers to total lift over
weight, which is a different quantity. Verified in `trim.test.ts`.

---

## The model

```
atmos/isa.ts            ISA atmosphere, geopotential ↔ geometric altitude
model/aircraft.ts       AircraftSpec + derived quantities
model/presets.ts        C172, GLIDER
model/controls.ts       pilot inputs → rate-limited surface positions
aero/coefficients.ts    CL, CD, Cm, CY, Cl, Cn, with the stall blend
aero/forces.ts          wind-axis → body-axis force and moment transform
propulsion/propeller.ts T = ηP/V, static cap, density lapse
dynamics/eom.ts         the 6-DOF equations of motion
dynamics/integrator.ts  classical RK4, fixed step
sim/telemetry.ts        the HUD interface
sim/simulation.ts       state, clock, ground constraint, event log
sim/trim.ts             damped-Newton trim solver
scenarios/autopilot.ts  simple feedback laws
scenarios/flight.ts     the five scenarios + envelope tools
cli.ts                  command-line text HUD
```

**Equations of motion** are integrated in body axes with NED gravity
(`gx = -g sinθ`, `gy = g sinφ cosθ`, `gz = g cosφ cosθ`) and Euler's rotational
equations with products of inertia neglected.

**Atmosphere.** ISA to 32 km, with the geopotential/geometric distinction handled
properly (they differ by ~19 m at 11 km). Above 32 km the result is flagged
`extrapolated` rather than silently accepted.

**Aerodynamics.** Attached flow is a linear lift curve with a parabolic drag
polar. The stall is a *smoothstep blend* into a flat-plate model
(`CL = 2 sinα cosα`), not a switch at CLMax. The blend exists because a step
discontinuity in the derivative destroys RK4's convergence order across it.

**Propulsion.** `T = ηP/V` with a static cap below a speed floor and a
density lapse `P ∝ ρ/ρ₀`. An unpowered airframe produces exactly zero thrust with
no special-casing.

**Ground.** A rigid floor that stops penetration, removes downward velocity and
applies rolling friction. On the ground the attitude is driven by the elevator
through a first-order lag rather than by integrating the aerodynamic pitching
moment — see the note in `sim/simulation.ts` for why. This is **not** a landing
gear model.

**Trim.** Three equilibrium residuals read straight out of `evaluateDerivative`,
solved by damped Newton with a central-difference Jacobian and a backtracking
line search. Two layouts share one code path: fix γ and solve for throttle
(powered), or fix throttle and solve for γ (glide).

---

## Verification gates (measured)

Every figure below is produced by `node --test` and asserted against a
*independent* reference — a published table, a closed-form expression, an energy
budget, or a cross-check between two routes to the same number. The tolerances
were set from the measurements, not guessed.

| Gate | Reference | Measured | Band |
|---|---|---|---|
| ISA temperature, pressure, density, speed of sound, 9 altitudes | published ISA table | worst **2.0e-5** relative | < 1e-4 |
| Tropopause temperature | 216.65 K exactly | **3e-14 K** off | < 1e-9 |
| Geopotential ↔ geometric roundtrip | exact inverse | < 1e-6 m | |
| Wind→body force transform isometry, α × β sweep | magnitude preserved | **4.2e-16** | < 1e-12 |
| Lift ⟂ velocity, drag ∥ velocity, side force on wind-y | geometry | < 1e-12 relative | |
| Quaternion `nedToBody ∘ bodyToNed` | identity | exact | |
| Rotation matrix orthonormal, det = 1, length preserved | | machine precision | |
| Lift-curve continuity across the stall break | jump scales with step | shrinks > 2× per 4× step | |
| Pure-drag body terminal velocity | √(2W/ρSC_D) closed form | matches | |
| Gravity-only motion | specific energy conserved | | |
| Glide energy loss vs drag work | ∫DV dt | < 2% | |
| RK4 order on the full model | error ratio per step-halving | **16.7, 16.3** (4th order = 16) | 10–26 |
| Trim residual ‖r‖∞ across the envelope | — | **< 1e-9** | |
| Trim residual = EOM acceleration | identity | < 1e-14 | |
| Trim stationarity (a, α̇ at the trim point) | — | < 1e-8 | |
| Cruise hold, 120 s on frozen trim controls | zero drift | altitude **1e-6 m**, airspeed **0** | < 1e-3 m |
| Scripted stall speed vs 1-g Vs (1500 m) | √(2W/ρSC_Lmax) | 26.53 vs 27.34 m/s (**−2.95%**) | −8%…0 |
| Minimum level trim speed vs closed-form Vs | same | 48.64 vs 49.39 kt (**−1.51%**) | ±2.5% |
| Maximum level speed | throttle saturation | **128.76 kt** | 118–136 kt |
| Glide ratio: coefficients vs trajectory | independent routes | 44.585 vs 44.782 (**+0.44%**) | < 2% |
| Climb rate: energy `(T−D)V/mg` vs geometry `V sinγ` | independent routes | **1.4%** apart | < 3% |
| Roll doublet: peak rate / bank / recovery | | 78.6 °/s, 51.0°, returns to **0.015°** | |
| Takeoff: ground roll / liftoff / events | published ~250–300 m, ~300 m roll | **290 m**, 62.4 kt, **1** liftoff | |
| Steady best climb rate | published ~3.7 m/s (720 fpm) | 6.08 m/s (1196 fpm) — **1.6×** | factor < 2 |

### What the gates actually caught

These are not hypothetical; each one was found by a numeric assertion that
failed, and several would have been invisible to inspection.

- **ISA pressure sign.** The exponent was written `g₀/(R·L)` instead of
  `−g₀/(R·L)`. Caught by the published-table comparison.
- **The wind-to-body transform was not an isometry at nonzero sideslip.** One
  basis vector's third component was `sinα` instead of `sinα·cosβ`, making the
  body force up to 40% wrong with sideslip. Sweeping α alone passed every check;
  it was only caught because the gate swept α **and** β.
- **`nedToBody` was not the inverse of `bodyToNed`.** A hand-transposed matrix
  had two wrong-sign off-diagonals. Symptom: takeoff plateaued at 10 m/s behind
  a phantom 2400 N of resistance. Caught by the quaternion roundtrip.
- **0/0 at zero airspeed.** The non-dimensional rate denominators were 0 at a
  standing start, so the state went non-finite on the first integration step.
- **Ground attitude.** Integrating the aerodynamic pitching moment against a bare
  floor made the 172 rotate onto its tail and lift off at half the correct speed.
- **`loadFactor` was documented as 1.0 in level flight** when it is exactly
  `cos(theta)`.
- **`maxLevelSpeed` returned its own initial guess** (30 m/s) for an unpowered
  airframe — no level trim exists for a glider at any speed, so the honest answer
  is "undefined", now NaN.
- **An RK4 convergence test was measuring round-off, not truncation** (ratios
  3300 and 0.15), because the case was over-resolved. Fixed by a larger
  disturbance and a longer run.
- **Post-stall lift does not fall below 0.8 at 34°.** The flat-plate model peaks
  at 1.0 at 45°; the test now checks the correct thing.
- **The glider's phugoid is mildly unstable** (λ ≈ +0.066 /s). That is a real
  property of a high-L/D airframe with a weak static margin in this model; the
  test pins the measured value rather than pretending otherwise.

---

## Known limitations

Stated here and, where they bite, in the module that owns them.

**Physics**
- Flat, non-rotating Earth. No Coriolis, no Earth rate.
- Constant mass; no fuel burn.
- No aeroelasticity, no structural modes, no propeller gyroscopic moment.
- Products of inertia neglected (the presets are close enough to symmetric).

**Aerodynamics**
- Incompressible: no Prandtl-Glauert or wave-drag correction, so the model is not
  valid near the critical Mach number.
- No stall hysteresis: recovery is symmetric with entry, and the blend is
  path-independent.
- One CLMax per airframe; no Reynolds-number or flap effects.
- No propeller slipstream, which understates elevator authority at low airspeed.

**Propulsion**
- Constant propeller efficiency. This is the model's largest quantitative error:
  **steady climb rate is over-predicted by a factor of ~1.6** (6.08 m/s against a
  published 3.7 m/s), because a real fixed-pitch propeller is far less efficient
  at the low advance ratio of a climb. Fixing it needs η as a function of advance
  ratio.
- Thrust acts along body x through the CG, so power changes produce no pitching
  moment.
- In the 172 the thrust is additionally pinned at the 2400 N static cap across
  the whole climb speed range.

**Ground**
- Rigid floor. No oleo travel, no wheel load distribution, no brakes, no
  nose-wheel steering, no ground effect. Takeoff distances are indicative, not
  POH-equivalent (the numbers do land close, but treat that as encouraging rather
  than as validation).

**Numerics**
- Fixed step, no adaptivity. A run is reproducible bit-for-bit, which is what you
  want when the output is a performance number.

---

## Reading the numbers

Some outputs look like errors and are not:

- The **scripted stall speed** sits ~3% *below* the closed-form 1-g Vs. Onset is
  defined as separation crossing 0.5, about 3.4° past the linear-curve stall
  incidence, by which point the aeroplane is already descending and the load
  factor is below 1. The tight check against the closed form is done separately,
  on the minimum speed at which a 1-g trim exists at all (−1.5%).
- The **glider's long-run glide ratio** (52.1) is higher than its best L/D (44.6).
  That is the unstable phugoid wandering off the steady state, not free energy;
  the window-based ratio (44.78) is the one that agrees with the coefficients.
- The **15 m airspeed** on takeoff is several m/s above the liftoff speed. The
  aeroplane accelerates during the initial climb.

---

## License

MIT.
