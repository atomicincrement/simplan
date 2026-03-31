//! Rigid-body equations of motion for the C172p.
//!
//! **Coordinate frames**
//! - Body:  X forward, Y right, Z down.
//! - NED:   X North, Y East, Z Down (local horizontal).
//! - Euler angles (Tait-Bryan, ZYX):
//!   - ψ (psi)   – heading / yaw   about NED Z
//!   - θ (theta) – pitch            about intermediate Y
//!   - φ (phi)   – roll             about body X
//!
//! **Units – imperial throughout:**
//! - length: ft,  velocity: ft/s,  angle: rad,  time: s
//! - force: lbf,  moment: lbf·ft,  mass: slug
//!
//! **References**
//! - Stevens & Lewis, "Aircraft Control and Simulation", 2nd ed., Wiley 2003.
//! - JSBSim FGPropagate.cpp, FGAccelerations.cpp

use crate::atmo::{self, G0};
use crate::math::{dcm_ned2body, Vec3};

// ── Aircraft mass properties (c172p.xml, mass_balance block) ─────────────────

/// Empty weight + typical pilot (500 lb fuel)
const WEIGHT_LBF: f64 = 1500.0 + 180.0 + 200.0; // lbf
pub const MASS: f64 = WEIGHT_LBF / G0; // slug

/// Moments of inertia (slug·ft²) – from <mass_balance> in c172p.xml.
pub const IXX: f64 = 948.0;
pub const IYY: f64 = 1346.0;
pub const IZZ: f64 = 1967.0;
/// Products of inertia ≈ 0 (symmetric aircraft).
pub const IXZ: f64 = 0.0;

// Denominator for coupled p–r integration with Ixz ≠ 0.
// Γ = Ixx·Izz − Ixz²
const GAMMA: f64 = IXX * IZZ - IXZ * IXZ;

// Coupled inertia combinations (Stevens & Lewis, eq. 1.4-12)
const C1: f64 = ((IYY - IZZ) * IZZ - IXZ * IXZ) / GAMMA;
const C2: f64 = (IXX - IYY + IZZ) * IXZ / GAMMA;
const C3: f64 = IZZ / GAMMA;
const C4: f64 = IXZ / GAMMA;
const C5: f64 = (IZZ - IXX) / IYY;
const C6: f64 = IXZ / IYY;
const C7: f64 = ((IXX - IYY) * IXX + IXZ * IXZ) / GAMMA;
const C8: f64 = IXX / GAMMA;

// ── State vector ──────────────────────────────────────────────────────────────

/// Complete 12-DoF rigid-body state.
#[derive(Debug, Clone, Copy)]
pub struct State {
    // ── Euler attitude angles (rad) ──────────────────────────────────────────
    /// Roll angle (rad).
    pub phi: f64,
    /// Pitch angle (rad).
    pub theta: f64,
    /// Heading / yaw (rad, 0 = North, increases clockwise).
    pub psi: f64,

    // ── NED position (ft from origin) ────────────────────────────────────────
    pub pos_n: f64,
    pub pos_e: f64,
    /// Altitude above MSL (ft, positive up).
    pub altitude: f64,

    // ── Body-frame velocity (ft/s) ───────────────────────────────────────────
    /// Forward.
    pub u: f64,
    /// Right.
    pub v: f64,
    /// Down.
    pub w: f64,

    // ── Body-frame angular rate (rad/s) ─────────────────────────────────────
    /// Roll rate.
    pub p: f64,
    /// Pitch rate.
    pub q: f64,
    /// Yaw rate.
    pub r: f64,

    // ── Simulation time (s) ──────────────────────────────────────────────────
    pub t: f64,

    // ── Auxiliary / derived (updated each step) ──────────────────────────────
    /// Angle of attack (rad).
    pub alpha: f64,
    /// Sideslip (rad).
    pub beta: f64,
    /// True airspeed (ft/s).
    pub vt: f64,
    /// Dynamic pressure (lbf/ft²).
    pub qbar: f64,
    /// Mach number.
    pub mach: f64,
    /// Stall hysteresis flag (0 or 1), computed from alpha history.
    pub stall_hyst: f64,
    /// Load factor (g).
    pub nz: f64,
}

impl State {
    /// Build an approximate trimmed level-flight initial condition.
    ///
    /// Uses a linear fit to the CLwbh table (valid below stall) to find the
    /// angle of attack that balances lift against weight, then sets body-frame
    /// u and w accordingly.
    pub fn level_flight(altitude_ft: f64, airspeed_fps: f64, heading_rad: f64) -> Self {
        // Atmospheric density at altitude
        let rho = crate::atmo::atmosphere(altitude_ft).density;
        let vt  = airspeed_fps;
        let qbar = 0.5 * rho * vt * vt;

        // C172p weight (from eom constants)
        let weight = MASS * G0; // lbf

        // Required CL for level flight
        let cl_needed = weight / (qbar * crate::aero::WING_AREA);

        // Linear CLwbh fit: CL ≈ 0.25 + 5.33·α  (valid 0 ≤ α ≤ 0.09 rad)
        // Clamp to a sensible range so we never start in stall.
        let alpha = ((cl_needed - 0.25) / 5.33_f64).clamp(0.0, 0.25);

        // For level flight: climb angle γ = 0 → θ = α
        let theta = alpha;
        let u = vt * theta.cos();
        let w = vt * theta.sin();

        State {
            phi: 0.0,
            theta,
            psi: heading_rad,
            pos_n: 0.0,
            pos_e: 0.0,
            altitude: altitude_ft,
            u,
            v: 0.0,
            w,
            p: 0.0,
            q: 0.0,
            r: 0.0,
            t: 0.0,
            alpha,
            beta: 0.0,
            vt,
            qbar,
            mach: 0.0,
            stall_hyst: 0.0,
            nz: 1.0,
        }
    }
}

// ── Raw 12-element derivative vector (for integrators) ───────────────────────

#[derive(Clone, Copy, Default)]
struct Deriv {
    dphi: f64,
    dtheta: f64,
    dpsi: f64,
    dpn: f64,
    dpe: f64,
    dalt: f64,
    du: f64,
    dv: f64,
    dw: f64,
    dp: f64,
    dq: f64,
    dr: f64,
}

fn add_state_deriv(s: &State, d: &Deriv, dt: f64) -> State {
    State {
        phi:      s.phi      + d.dphi   * dt,
        theta:    s.theta    + d.dtheta * dt,
        psi:      s.psi      + d.dpsi   * dt,
        pos_n:    s.pos_n    + d.dpn    * dt,
        pos_e:    s.pos_e    + d.dpe    * dt,
        altitude: s.altitude + d.dalt   * dt,
        u: s.u + d.du * dt,
        v: s.v + d.dv * dt,
        w: s.w + d.dw * dt,
        p: s.p + d.dp * dt,
        q: s.q + d.dq * dt,
        r: s.r + d.dr * dt,
        // cosmetic fields – will be recomputed after integration
        ..*s
    }
}

fn scale_deriv(d: &Deriv, s: f64) -> Deriv {
    Deriv {
        dphi:   d.dphi   * s,
        dtheta: d.dtheta * s,
        dpsi:   d.dpsi   * s,
        dpn:    d.dpn    * s,
        dpe:    d.dpe    * s,
        dalt:   d.dalt   * s,
        du: d.du * s,
        dv: d.dv * s,
        dw: d.dw * s,
        dp: d.dp * s,
        dq: d.dq * s,
        dr: d.dr * s,
    }
}

fn add_derivs(a: &Deriv, b: &Deriv) -> Deriv {
    Deriv {
        dphi:   a.dphi   + b.dphi,
        dtheta: a.dtheta + b.dtheta,
        dpsi:   a.dpsi   + b.dpsi,
        dpn:    a.dpn    + b.dpn,
        dpe:    a.dpe    + b.dpe,
        dalt:   a.dalt   + b.dalt,
        du: a.du + b.du,
        dv: a.dv + b.dv,
        dw: a.dw + b.dw,
        dp: a.dp + b.dp,
        dq: a.dq + b.dq,
        dr: a.dr + b.dr,
    }
}

// ── Derivative computation ────────────────────────────────────────────────────

/// External forces/moments to be added to the EOM.
#[derive(Clone, Copy, Default)]
pub struct ExternalLoads {
    /// Total force in body frame (lbf): aero + thrust.
    pub force: Vec3,
    /// Total moment in body frame (lbf·ft): aero.
    pub moment: Vec3,
}

fn compute_deriv(s: &State, loads: &ExternalLoads) -> Deriv {
    let phi   = s.phi;
    let theta = s.theta;
    let psi   = s.psi;
    let u     = s.u;
    let v     = s.v;
    let w     = s.w;
    let p     = s.p;
    let q     = s.q;
    let r     = s.r;

    let (sphi, cphi)     = phi.sin_cos();
    let (stheta, ctheta) = theta.sin_cos();
    let ttheta           = stheta / ctheta.max(0.001); // guard against 90° pitch

    // ── Forces ───────────────────────────────────────────────────────────────
    let fx = loads.force.x;
    let fy = loads.force.y;
    let fz = loads.force.z;

    // Gravity in body frame (NED gravity = [0, 0, g], rotate to body).
    //   gx = −g · sin(θ)
    //   gy =  g · sin(φ)·cos(θ)
    //   gz =  g · cos(φ)·cos(θ)
    let gx = -G0 * stheta;
    let gy =  G0 * sphi * ctheta;
    let gz =  G0 * cphi * ctheta;

    // Newton's 2nd law in rotating body frame:
    //   m·u̇ = Fx + m·gx − m·(q·w − r·v)
    let du = (fx / MASS) + gx - (q * w - r * v);
    let dv = (fy / MASS) + gy - (r * u - p * w);
    let dw = (fz / MASS) + gz - (p * v - q * u);

    // ── Moments ──────────────────────────────────────────────────────────────
    let roll_m  = loads.moment.x; // L
    let pitch_m = loads.moment.y; // M
    let yaw_m   = loads.moment.z; // N

    // Stevens & Lewis eqs. 1.4-12 (general with Ixz coupling)
    let dp = C1 * r * q + C2 * p * q + C3 * roll_m  + C4 * yaw_m;
    let dq = C5 * p * r - C6 * (p * p - r * r) + (1.0 / IYY) * pitch_m;
    let dr = C7 * p * q - C1 * q * r + C4 * roll_m  + C8 * yaw_m;

    // ── Euler angle kinematics ────────────────────────────────────────────────
    let dphi   = p + (q * sphi + r * cphi) * ttheta;
    let dtheta = q * cphi - r * sphi;
    let dpsi   = (q * sphi + r * cphi) / ctheta.max(0.001);

    // ── NED position derivatives ──────────────────────────────────────────────
    // Build Tn2b (NED → body) then use its transpose (body → NED).
    let tn2b = dcm_ned2body(phi, theta, psi);
    let tb2n = tn2b.transpose();

    let vel_body = Vec3::new(u, v, w);
    let vel_ned  = tb2n.mul_vec(vel_body);

    //  Down = −dalt
    let dpn  = vel_ned.x;
    let dpe  = vel_ned.y;
    let dalt = -vel_ned.z; // NED Z is positive down; altitude is positive up

    Deriv { dphi, dtheta, dpsi, dpn, dpe, dalt, du, dv, dw, dp, dq, dr }
}

// ── Post-integration derived quantities ──────────────────────────────────────

/// Recompute auxiliary quantities (alpha, beta, Mach, etc.) from integrated state.
fn update_auxiliary(s: &mut State, prev_alpha: f64, dt: f64) {
    let atmo = atmo::atmosphere(s.altitude);

    // Angle of attack and sideslip (wind has no effect in this model,
    // body velocity IS the aerodynamic velocity).
    let u2w2 = s.u * s.u + s.w * s.w;
    let vt2  = u2w2 + s.v * s.v;
    s.vt    = vt2.sqrt();
    s.alpha = if u2w2 >= 1e-6 { s.w.atan2(s.u) } else { 0.0 };
    s.beta  = if s.vt > 0.001 { s.v.atan2(u2w2.sqrt()) } else { 0.0 };

    s.qbar  = 0.5 * atmo.density * vt2;
    s.mach  = s.vt / atmo.sound_speed;

    // Alpha-rate (finite difference from previous step)
    // (alpha_dot is passed into aero separately in lib.rs)

    // Stall hysteresis (from c172p.xml hysteresis_limits)
    const ALPHA_HYST_MAX: f64 = 0.36; // rad
    const ALPHA_HYST_MIN: f64 = 0.09; // rad
    if s.alpha > ALPHA_HYST_MAX {
        s.stall_hyst = 1.0;
    } else if s.alpha < ALPHA_HYST_MIN {
        s.stall_hyst = 0.0;
    }
    // Between limits: keep stall_hyst unchanged (hysteresis).

    // Normal load factor (positive up)
    // nz = −(Fz_body / (m·g))  but computed as body acceleration
    // Approximation: nz ≈ 1 − (theta_rate·vt / g)  (for small angles).
    // For display we use nz from lift: nz ≈ lift / weight.
    // We'll fill it in properly in lib.rs after aero is called.
    let _ = prev_alpha;
    let _ = dt;
}

// ── 4th-order Runge-Kutta integration ────────────────────────────────────────

/// Advance the state by one time step using RK4.
///
/// The caller supplies a closure that computes external loads as a function
/// of the current provisional state (needed for mid-step force evaluations).
pub fn rk4_step<F>(state: &State, dt: f64, loads_fn: F) -> State
where
    F: Fn(&State) -> ExternalLoads,
{
    let prev_alpha = state.alpha;

    // k1
    let l1 = loads_fn(state);
    let d1 = compute_deriv(state, &l1);

    // k2
    let s2 = add_state_deriv(state, &d1, dt * 0.5);
    let l2 = loads_fn(&s2);
    let d2 = compute_deriv(&s2, &l2);

    // k3
    let s3 = add_state_deriv(state, &d2, dt * 0.5);
    let l3 = loads_fn(&s3);
    let d3 = compute_deriv(&s3, &l3);

    // k4
    let s4 = add_state_deriv(state, &d3, dt);
    let l4 = loads_fn(&s4);
    let d4 = compute_deriv(&s4, &l4);

    // Weighted average
    let d_avg = scale_deriv(
        &add_derivs(
            &add_derivs(&d1, &scale_deriv(&d2, 2.0)),
            &add_derivs(&scale_deriv(&d3, 2.0), &d4),
        ),
        1.0 / 6.0,
    );

    let mut next = add_state_deriv(state, &d_avg, dt);
    next.t = state.t + dt;

    // Wrap heading to [0, 2π)
    next.psi = next.psi.rem_euclid(2.0 * std::f64::consts::PI);

    // Clamp altitude above ground (very basic – no gear/terrain model)
    if next.altitude < 0.0 {
        next.altitude = 0.0;
        if next.w > 0.0 {
            next.w = 0.0; // stop sinking through ground
        }
        if next.u < 0.0 {
            next.u = 0.0;
        }
    }

    update_auxiliary(&mut next, prev_alpha, dt);

    next
}

/// Simple Euler integration (faster, less accurate – useful for sub-steps).
pub fn euler_step<F>(state: &State, dt: f64, loads_fn: F) -> State
where
    F: Fn(&State) -> ExternalLoads,
{
    let prev_alpha = state.alpha;
    let loads = loads_fn(state);
    let d = compute_deriv(state, &loads);
    let mut next = add_state_deriv(state, &d, dt);
    next.t = state.t + dt;
    next.psi = next.psi.rem_euclid(2.0 * std::f64::consts::PI);
    if next.altitude < 0.0 {
        next.altitude = 0.0;
        if next.w > 0.0 { next.w = 0.0; }
    }
    update_auxiliary(&mut next, prev_alpha, dt);
    next
}
