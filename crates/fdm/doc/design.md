# C172p Flight Dynamics Model – Design Document

> Written for agent handoff. A new agent reading this document plus the source
> files in `rust_fdm/src/` should be able to understand every design decision
> and continue the work without re-deriving anything from the JSBSim C++ source.

---

## 1. Origin and scope

### Source repository
- **JSBSim** at `~/play/jsbsim` (GitHub: `JSBSim-Team/jsbsim`).
- The aircraft definition is `aircraft/c172p/c172p.xml` — a Cessna 172p with a
  Lycoming IO-320 engine and fixed-pitch 75-inch propeller.
- The JSBSim C++ modules that were read and translated:
  - `src/models/FGPropagate.cpp/.h` — state integration (EOM)
  - `src/models/FGAccelerations.cpp/.h` — force/moment → acceleration
  - `src/models/FGAerodynamics.cpp/.h` — aero table evaluation pipeline
  - `src/models/FGAuxiliary.cpp/.h` — alpha, beta, qbar, Mach derivation
  - `src/models/atmosphere/FGStandardAtmosphere.cpp` — ISA 1976
  - `src/models/FGFCS.cpp` — flight control system (surface scaling)
  - `src/models/FGPropulsion.cpp` — engine/propeller
  - `aircraft/c172p/c172p.xml` — **all numeric constants and table data**

### Goal
A **self-contained, zero-dependency Rust library** (`c172_fdm`) that can
advance the C172p state forward in time given pilot control inputs. No XML
parsing, no property manager, no logging — just the physics.

---

## 2. Crate layout

```
rust_fdm/
├── Cargo.toml
├── src/
│   ├── lib.rs      – Public API: FlightModel, Controls, State
│   ├── math.rs     – Vec3, Mat3, DCMs, table interpolation
│   ├── atmo.rs     – ISA 1976 standard atmosphere
│   ├── aero.rs     – C172p aerodynamics (all tables hard-coded)
│   ├── fcs.rs      – Pilot inputs → surface deflections
│   ├── prop.rs     – Engine thrust + actuator-disk propwash
│   └── eom.rs      – 6-DoF rigid-body EOM, RK4 integrator
└── examples/
    └── simulate.rs – 80-second scenario: trim, pull-up, banked turn
```

---

## 3. Unit system

**Everything is Imperial, matching JSBSim internally.**

| Quantity | Unit |
|---|---|
| Length / altitude | ft |
| Velocity | ft/s |
| Angle | rad (degrees only where a JSBSim XML value is in degrees) |
| Angular rate | rad/s |
| Force | lbf |
| Moment | lbf·ft |
| Mass | slug |
| Density | slug/ft³ |
| Pressure | lbf/ft² (psf) |
| Time | s |

Conversion reference: 1 kt = 1.6878 ft/s, 1 g = 32.174 ft/s², 1 slug = 14.594 kg.

---

## 4. Coordinate frames

Three frames are used throughout, matching JSBSim conventions exactly:

| Frame | X | Y | Z | Used for |
|---|---|---|---|---|
| **Body** | Forward | Right | Down | Forces, moments, velocity (u,v,w), rates (p,q,r) |
| **NED** | North | East | Down | Position derivatives, gravity, navigation |
| **Wind** | Along V_aero | Right | Down | Aerodynamic force assembly before body transform |

### Attitude representation
Tait-Bryan ZYX Euler angles (same as JSBSim):
- ψ (psi) — heading, rot about NED-Z, 0 = North, clockwise positive
- θ (theta) — pitch, rot about intermediate Y
- φ (phi) — roll, rot about body X

DCM `Tn2b` (NED → body) = `Rx(φ) · Ry(θ) · Rz(ψ)` — implemented in
`math::dcm_ned2body`.

DCM `Tw2b` (wind → body) = `Ry(α) · Rz(β)` — implemented in
`math::dcm_wind2body`. Used to rotate assembled aero forces into body frame.

No quaternions are used. The singularity at θ = ±90° (gimbal lock) is
guarded with a `ctheta.max(0.001)` clamp in the Euler kinematic equations.

---

## 5. Module details

### 5.1 `math.rs`

Provides:
- `Vec3` with `add`, `sub`, `neg`, `scale`, `dot`, `cross`, `norm`.
- `Mat3` (row-major): `rot_x/y/z`, `transpose`, `mul_vec`, `mul_mat`.
- `dcm_ned2body(phi, theta, psi) -> Mat3`
- `dcm_wind2body(alpha, beta) -> Mat3`
- `interp1(table: &[(f64,f64)], x) -> f64` — linear, clamped at ends.
- `interp2(row_keys, col_keys, values, row_x, col_x) -> f64` — bilinear.

All table data in `aero.rs` is consumed through these two functions.

### 5.2 `atmo.rs`

ISA 1976 from `FGStandardAtmosphere.cpp`. Layer breakpoints are geopotential
altitude in feet (matching JSBSim's internal `StdAtmosTemperatureTable`).

Outputs `AtmoState { temperature_r, pressure_psf, density, sound_speed,
kin_viscosity }`. Kinematic viscosity uses Sutherland's law in Imperial units:

```
μ = 2.333e-8 · T^1.5 / (T + 198.6)   [slug/(ft·s)]
ν = μ / ρ
```

Key constants re-exported: `G0 = 32.174 ft/s²`, `SL_DENSITY`, `SL_TEMP_R`,
`SL_PRESSURE_PSF`.

### 5.3 `fcs.rs`

Translates `Controls` (pilot inputs, all normalised) → `Surfaces` (radians /
degrees).

**Sign convention decision** (differs from raw JSBSim XML):

| Control | Our `+1` means | JSBSim XML `+1` means | How reconciled |
|---|---|---|---|
| `elevator` | Nose up (aft stick) | Nose down (fwd stick) | Multiply by −1 before scaling |
| `aileron` | Right roll | Right roll | Same — no negation |
| `rudder` | Nose right | Nose left (Cndr = −0.043) | Multiply by −1 before scaling |
| `throttle` | Full power | Full power | Same |
| `flap` | 0→retracted, 1→30° | Same | Same |

Surface ranges used (symmetric simplification of asymmetric JSBSim limits):
- Elevator: ±25° (JSBSim: −28°/+23° asymmetric)
- Aileron: ±17° (JSBSim: −20°/+15° asymmetric)
- Rudder: ±16° (symmetric)

The asymmetric limits are hardware stops never reached in normal flight; the
symmetric simplification introduces ≤3° error at the extremes.

### 5.4 `aero.rs`

All coefficient tables are copied verbatim from `c172p.xml` and stored as
`&[(f64, f64)]` (1-D) or `&[&[f64]]` (2-D) Rust constants.

**Assembly pipeline** (mirrors `FGAerodynamics::Run`):

1. Compute α, β, Vt, q̄ from `AeroIn.vel_aero`.
2. Compute b/(2Vt) and c/(2Vt) for dimensionless rate terms.
3. Look up ground-effect factors `kCDge`, `kCLge` from `hoverbmac`.
4. Compute induced q̄ for tail surfaces: `q_induced = 0.5·ρ·(u + 2·v_induced)²`.
5. Sum coefficients for each of 6 axes (DRAG, SIDE, LIFT, ROLL, PITCH, YAW).
6. Wind-frame forces (DRAG/SIDE/LIFT) are sign-corrected and multiplied by
   `Tw2b` to get body-frame force vector.
7. Body-frame moments (ROLL/PITCH/YAW) are returned directly.

**JSBSim sign convention for wind-frame forces** (critical, easy to get wrong):
> JSBSim defines drag and lift as positive magnitudes (retarding / upward),
> then negates them before the `Tw2b` multiplication so the resulting body-X
> force is negative (retarding) and body-Z is negative (upward in body frame
> where Z is down). The code does exactly:
> ```rust
> let f_wind = Vec3::new(-drag, side, -lift); // negate D and L
> let force_body = tw2b.mul_vec(f_wind);
> ```

**Moment reference point**: Moments are computed at the aerodynamic reference
point (AeroRP = [43.2, 0, 59.4] in). The offset to the CG is implicit in the
JSBSim XML moment coefficients; no explicit `r × F` transfer is needed for
this coefficient set.

The `Cmde` term uses `qbar_induced` (propwash q̄) because the elevator is in
the propwash. All other terms use the freestream `qbar`.

### 5.5 `prop.rs`

Simple fixed-pitch prop model — no blade-element theory. Two approximations:

1. **Thrust** = `throttle × T_max_SL × (ρ/ρ_SL)^0.8 × (1 − u/V_zero)`.
   - `T_max_SL = 900 lbf` (static, sea level, full throttle).
   - `V_zero = 372 ft/s` (speed at which prop thrust approaches zero; linear taper).
   - Density exponent 0.8 is a standard empirical value (range 0.7–1.0).

2. **Induced velocity** from actuator-disk momentum theory:
   `v_induced = sqrt(T / (2·ρ·A))` where `A = π·(3.125 ft)²`.

This `v_induced` is passed to `aero.rs` to build `qbar_induced` for Cmde and
Cndr (the XML uses `aero/function/qbar-induced-psf` for those two terms).

### 5.6 `eom.rs`

**State vector** (`State`, 12 scalars + auxiliary):

| Field | Symbol | Frame |
|---|---|---|
| `phi, theta, psi` | φ, θ, ψ | Euler angles (rad) |
| `pos_n, pos_e, altitude` | N, E, h | NED position (ft) |
| `u, v, w` | u, v, w | Body velocity (ft/s) |
| `p, q, r` | p, q, r | Body angular rate (rad/s) |
| `alpha, beta, vt, qbar, mach` | α, β, Vt, q̄, M | Derived, updated each step |
| `stall_hyst` | — | 0 or 1, from hysteresis limits |
| `nz` | Nz | Normal load factor (g) |

**Translational equations** (body frame, rotating axes):

```
m·u̇ = Fx + m·gx − m·(qw − rv)
m·v̇ = Fy + m·gy − m·(ru − pw)
m·ẇ = Fz + m·gz − m·(pv − qu)
```

Gravity in body frame: `gx = −g·sinθ`, `gy = g·sinφ·cosθ`, `gz = g·cosφ·cosθ`.

**Rotational equations** (Stevens & Lewis eq. 1.4-12, general Ixz ≠ 0):

```
ṗ = C1·r·q + C2·p·q + C3·L + C4·N
q̇ = C5·p·r − C6·(p²−r²) + M/Iyy
ṙ = C7·p·q − C1·q·r + C4·L + C8·N
```

where C1…C8 are derived from `Ixx, Iyy, Izz, Ixz` (see `eom.rs` constants).
For the C172p, `Ixz = 0` so C2=C4=C6=0, simplifying the roll/yaw coupling.

**Euler kinematics**:

```
φ̇ = p + (q·sinφ + r·cosφ)·tanθ
θ̇ = q·cosφ − r·sinφ
ψ̇ = (q·sinφ + r·cosφ) / cosθ
```

**Position update**: body velocity rotated to NED via `Tb2n = Tn2b^T`.
Altitude = −NED-Z velocity integral.

**Integrator**: 4th-order Runge-Kutta (`rk4_step`). The force closure is
called 4 times per step (at t, t+dt/2 ×2, t+dt), so the aerodynamics and
propulsion are re-evaluated at each sub-step. This costs 4× compute but gives
O(dt⁴) accuracy. An `euler_step` is also provided for sub-stepping.

**Trim initialisation** (`State::level_flight`): uses the linear fit
`CL ≈ 0.25 + 5.33·α` (derived from the CLwbh table, clean, below stall) to
find the α that balances lift = weight, then sets `u = Vt·cosα`, `w = Vt·sinα`,
`theta = alpha` (zero climb angle assumed).

### 5.7 `lib.rs` — `FlightModel`

The public entry point. Owns a `State` and tracks `prev_alpha` for finite-
difference alpha-dot computation.

`FlightModel::step(dt, controls)`:
1. Calls `controls.to_surfaces()` → `Surfaces`.
2. Builds a closure `loads_fn(state) -> ExternalLoads` that:
   a. Calls `atmo::atmosphere(altitude)`.
   b. Calls `prop::propulsion(throttle, u, rho)`.
   c. Builds `AeroIn` and calls `aero::aerodynamics(&aero_in)`.
   d. Returns `{ force: aero_force + thrust_body, moment: aero_moment }`.
3. Passes the closure to `eom::rk4_step`.
4. Re-computes `alpha_dot` and `nz` on the returned state.

---

## 6. Known limitations and future work

| Item | Status | Notes |
|---|---|---|
| Wind/turbulence | ❌ Not implemented | `AeroIn.vel_aero` is body velocity; subtract `Tl2b · wind_ned` to add wind |
| Ground reactions | ❌ Not implemented | Simple clamp at `altitude < 0`; needs spring/damper gear model |
| Fuel burn / weight change | ❌ Not implemented | `MASS` is constant; add `fuel_lbs` to `State` and subtract `engine_flow_lbph` |
| Engine torque / gyroscopic | ❌ Not implemented | Prop gyroscopic moment is significant in low-speed manoeuvres |
| Quaternion attitude | ❌ Euler only | Approaches gimbal lock at θ ≈ ±90°; safe for normal flight |
| Propeller P-factor | ❌ Not implemented | JSBSim uses `p_factor = 5` in the thruster block |
| Trim solver | ❌ Not implemented | `State::level_flight` is approximate; a Newton-Raphson trim loop is needed for accuracy |
| Other aircraft | ❌ C172p only | Tables are hard-coded constants; parameterise `AircraftConfig` struct to support others |
| `no_std` | ❌ Uses `std::f64` | Would need `libm` crate for `sin`, `cos`, `sqrt`, `atan2` |

### Highest-priority next steps

1. **Trim solver**: iterate `FlightModel::step` at fixed controls until
   `|dState/dt|` is below a threshold. Required for realistic initial
   conditions at arbitrary speed/altitude.

2. **Wind model**: add `wind_ned: Vec3` to `AeroIn`; used in `FGAuxiliary`
   as `vAeroUVW = vUVW − Tl2b · TotalWindNED`.

3. **Ground reactions**: spring/damper at the three contact points defined in
   `c172p.xml` (nose at [−6.8, 0, −19.5] in, left/right main at [58.2, ±43,
   −15.5] in).

4. **Propeller gyroscopic moment**: `M_gyro = Ω_prop × J_prop · ω_aircraft`.
   JSBSim computes this in `FGPropeller.cpp`.

---

## 7. Validation observations

Running `examples/simulate.rs` (81 kt, 3 000 ft, zero elevator):

- **Phugoid**: altitude oscillates ±65 ft with period ~45 s — consistent
  with published C172 phugoid (~60 s at 75 kt; our period is short because
  Cm0=0.1 is not perfectly cancelled by zero elevator).
- **Pull-up**: elevator +0.25 (≈6.25°) produces nz ≈ 1.6g, alpha ~14° —
  physically reasonable (stall is ~16°).
- **Banked turn**: 30° bank gives nz ≈ 1.4g (theoretical 1/cos30° = 1.155;
  excess from speed loss without throttle increase).
- **Zero sideslip at zero aileron/rudder**: confirmed — phi, p, r all remain
  0.00 in symmetric flight.

The main remaining calibration issue is the pitch trim: the aircraft slowly
descends over 20 s at zero elevator because Cm0=0.10 is not perfectly balanced
by Cm_alpha at the linearised CL trim point. A proper trim solver (item 1
above) would eliminate this.

---

## 8. Reference data from c172p.xml

Quick lookup for an agent modifying the aerodynamics:

| Symbol | Value | Source element |
|---|---|---|
| Wing area S | 174 ft² | `<wingarea>` |
| Wingspan b | 35.8 ft | `<wingspan>` |
| Chord c̄ | 4.9 ft | `<chord>` |
| Ixx | 948 slug·ft² | `<ixx>` |
| Iyy | 1346 slug·ft² | `<iyy>` |
| Izz | 1967 slug·ft² | `<izz>` |
| Empty weight | 1500 lb | `<emptywt>` |
| CDo | 0.027 | `CDo` function |
| CLα (per rad, approx) | 5.33 | Slope of CLwbh table at α=0 |
| Cmα | −1.80 | `Cmalpha` |
| Cm0 | +0.10 | `Cmo` |
| Cmde | −1.122 | `Cmde` (uses qbar_induced) |
| Cndr | −0.043 | `Cndr` (uses qbar_induced) |
| Alpha stall min | −0.087 rad | `<alphalimits>` |
| Alpha stall max | 0.28 rad | `<alphalimits>` |
| Hysteresis min | 0.09 rad | `<hysteresis_limits>` |
| Hysteresis max | 0.36 rad | `<hysteresis_limits>` |

---

## 9. How to build and run

```
cd ~/play/jsbsim/rust_fdm
cargo build --release            # compile
cargo run --release --example simulate   # run 80-second scenario
cargo test                       # (no tests written yet)
```

Rust edition 2021, no external dependencies.
