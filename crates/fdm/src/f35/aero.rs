//! F-35A aerodynamic model.
//!
//! **Coordinate frame** (JSBSim convention, shared with the C172 module):
//! - Body:  X forward, Y right, Z down.
//! - Wind:  X along aerodynamic velocity, Y right, Z down-ish.
//! - Forces are assembled in DRAG / SIDE / LIFT wind-frame axes then
//!   transformed to body frame via [`crate::math::dcm_wind2body`].
//! - Moments are assembled directly in body-frame ROLL / PITCH / YAW.
//!
//! **Reference geometry (F-35A, public domain):**
//! - Wing area   S = 460 ft²
//! - Wingspan    b = 35.0 ft
//! - MAC         c̄ = 15.8 ft
//! - AR          = b²/S ≈ 2.66  (low-AR cranked-delta planform)
//!
//! **Data sources:**
//! Aerodynamic coefficients are estimated from open-source reports, DATCOM
//! estimates for similar delta-wing / blended-body fighters (F-16 JSBSim
//! reference, Nguyen et al. NASA TM-1979), and public-domain F-35 references.
//!
//! **Units:** Imperial (ft, slug, lbf, s, rad) throughout.

use crate::math::{dcm_wind2body, interp1, Vec3};

// ── Reference geometry ───────────────────────────────────────────────────────

pub const WING_AREA: f64 = 460.0; // ft²  – S
pub const WINGSPAN:  f64 =  35.0; // ft   – b
pub const CHORD:     f64 =  15.8; // ft   – c̄ (mean aerodynamic chord)

// ── Mach-varying CD0 (wave-drag rise through transonic regime) ───────────────

/// Base zero-lift drag vs. Mach number.
const CD0_MACH: &[(f64, f64)] = &[
    (0.0,  0.0240),
    (0.6,  0.0240),
    (0.85, 0.0265),
    (0.95, 0.0340),
    (1.05, 0.0420),
    (1.20, 0.0390),
    (1.60, 0.0340),
    (2.00, 0.0300),
];

// ── Lift curve (CL vs. alpha) clean ─────────────────────────────────────────
//
//  F-35A cranked-delta: powerful LERX delays stall to ~35° (0.61 rad).
//  CL_alpha ≈ 3.2 /rad, CL_max ≈ 1.6 at α ≈ 0.40 rad (23°).

/// Lift curve (CL vs alpha_rad), clean configuration.
pub const CL_ALPHA_TABLE: &[(f64, f64)] = &[
    (-0.26, -0.65),  // −15°
    (-0.17, -0.43),  // −10°
    (-0.09, -0.19),  //  −5°
    ( 0.00,  0.10),  //   0°  (asymmetric body – slight positive lift at 0 α)
    ( 0.09,  0.39),  //   5°
    ( 0.17,  0.65),  //  10°
    ( 0.26,  0.90),  //  15°
    ( 0.35,  1.15),  //  20°
    ( 0.44,  1.38),  //  25°
    ( 0.52,  1.52),  //  30°
    ( 0.61,  1.60),  //  35°  – near C_Lmax
    ( 0.70,  1.45),  //  40°  – post-stall
    ( 0.87,  1.10),  //  50°  – deep post-stall
];

// ── Induced-drag factor k ────────────────────────────────────────────────────
//
//  CDi = k · CL²   where k = 1/(π·AR·e)
//  AR = 2.66, e ≈ 0.80  →  k ≈ 0.150

const K_INDUCED: f64 = 0.150;

// ── Elevator (pitch-channel elevon) effects on CL and CD ────────────────────

const CL_DE: f64 = 0.25; // ΔCL per elevon radian
const CD_DE: f64 = 0.040; // ΔCD per |elevon| radian (drag from deflection)

// ── Side-force coefficients ──────────────────────────────────────────────────

const CY_BETA: f64 = -0.90; // /rad
const CY_DR:   f64 =  0.12; // /rad  rudder
const CY_P:    f64 = -0.08; // per (b/2Vt)·p
const CY_R:    f64 =  0.25; // per (b/2Vt)·r

// ── Pitch-moment coefficients ────────────────────────────────────────────────

const CM0:     f64 =  0.040; // zero-alpha / zero-elevator trim moment
const CM_ALPHA:f64 = -0.300; // /rad   (FBW provides stability augmentation)
const CM_Q:    f64 = -6.000; // per (c/2Vt)·q
const CM_ADOT: f64 = -3.000; // per (c/2Vt)·α̇
const CM_DE:   f64 = -0.900; // /rad  elevon to pitch moment

// ── Roll-moment coefficients ─────────────────────────────────────────────────

const CL_BETA: f64 = -0.060; // /rad  dihedral effect
const CL_P:    f64 = -0.350; // per (b/2Vt)·p  roll damping
const CL_R:    f64 =  0.060; // per (b/2Vt)·r
const CL_DA:   f64 =  0.060; // /rad  differential elevon
const CL_DR:   f64 =  0.005; // /rad  rudder cross-coupling

// ── Yaw-moment coefficients ──────────────────────────────────────────────────

const CN_BETA: f64 =  0.100; // /rad  directional stability
const CN_P:    f64 = -0.030; // per (b/2Vt)·p
const CN_R:    f64 = -0.120; // per (b/2Vt)·r  yaw damping
const CN_DA:   f64 = -0.010; // /rad  aileron adverse yaw
const CN_DR:   f64 = -0.060; // /rad  rudder

// ── Public API ───────────────────────────────────────────────────────────────

/// Aerodynamic output (forces and moments in the body frame).
#[derive(Debug, Clone, Copy, Default)]
pub struct AeroOut {
    /// Total aerodynamic force in body frame [Fx, Fy, Fz] (lbf).
    pub force_body: Vec3,
    /// Total aerodynamic moment in body frame [L, M, N] (lbf·ft).
    pub moment_body: Vec3,
    /// Dynamic pressure (psf).
    pub qbar: f64,
    /// Angle of attack (rad).
    pub alpha: f64,
    /// Sideslip angle (rad).
    pub beta: f64,
    /// True airspeed (ft/s).
    pub vt: f64,
}

/// Inputs to the F-35A aerodynamic model.
#[derive(Debug, Clone, Copy)]
pub struct AeroIn {
    /// Body-frame velocity relative to airmass (ft/s): [u, v, w].
    pub vel_aero: Vec3,
    /// Body angular rate (rad/s): [p, q, r].
    pub pqr: Vec3,
    /// Alpha-dot (rad/s) from the previous integration step.
    pub alpha_dot: f64,
    /// Air density (slug/ft³).
    pub rho: f64,
    /// Speed of sound (ft/s).
    pub sound_speed: f64,
    /// Elevon deflection for pitch (rad): positive = nose-up moment.
    pub elevator_rad: f64,
    /// Differential elevon for roll (rad): positive = roll right.
    pub aileron_rad: f64,
    /// Rudder deflection (rad): positive = yaw right.
    pub rudder_rad: f64,
}

/// Compute aerodynamic forces and moments for the F-35A.
pub fn aerodynamics(inp: &AeroIn) -> AeroOut {
    let u = inp.vel_aero.x;
    let v = inp.vel_aero.y;
    let w = inp.vel_aero.z;

    // ── Basic flow quantities ─────────────────────────────────────────────
    let u2w2 = u * u + w * w;
    let vt2  = u2w2 + v * v;
    let vt   = vt2.sqrt();

    let alpha = if u2w2 >= 1e-6 { w.atan2(u) } else { 0.0 };
    let beta  = if vt > 0.001   { v.atan2(u2w2.sqrt()) } else { 0.0 };

    let qbar = 0.5 * inp.rho * vt2;
    let mach = vt / inp.sound_speed.max(1.0);

    let p = inp.pqr.x;
    let q = inp.pqr.y;
    let r = inp.pqr.z;

    let two_vt = 2.0 * vt.max(1.0);
    let bi2vel = WINGSPAN / two_vt;  // b / (2 Vt)
    let ci2vel = CHORD   / two_vt;  // c̄ / (2 Vt)

    let s = WING_AREA;
    let b = WINGSPAN;
    let c = CHORD;

    // ── Drag ─────────────────────────────────────────────────────────────
    let cd0    = interp1(CD0_MACH, mach);
    let cl_raw = interp1(CL_ALPHA_TABLE, alpha); // for induced drag
    let cd_i   = K_INDUCED * cl_raw * cl_raw;
    let cd_de  = CD_DE * inp.elevator_rad.abs();
    let cd_beta = 0.15 * beta.abs();

    let drag = qbar * s * (cd0 + cd_i + cd_de + cd_beta);

    // ── Side force ────────────────────────────────────────────────────────
    let cy = CY_BETA * beta
           + CY_DR * inp.rudder_rad
           + bi2vel * (CY_P * p + CY_R * r);

    let side = qbar * s * cy;

    // ── Lift ─────────────────────────────────────────────────────────────
    let cl = interp1(CL_ALPHA_TABLE, alpha)
           + CL_DE  * inp.elevator_rad
           + ci2vel * inp.alpha_dot * 1.2  // α̇ lift
           + ci2vel * q * 4.5;             // pitch-rate lift

    let lift = qbar * s * cl;

    // ── Transform wind → body ─────────────────────────────────────────────
    let f_wind = Vec3::new(-drag, side, -lift);
    let tw2b   = dcm_wind2body(alpha, beta);
    let force_body = tw2b.mul_vec(f_wind);

    // ── Pitch moment ──────────────────────────────────────────────────────
    let cm = CM0
           + CM_ALPHA * alpha
           + CM_DE    * inp.elevator_rad
           + ci2vel   * (CM_Q * q + CM_ADOT * inp.alpha_dot);

    let pitch_moment = qbar * s * c * cm;

    // ── Roll moment ───────────────────────────────────────────────────────
    let cl_moment = CL_BETA * beta
                  + CL_DA   * inp.aileron_rad
                  + CL_DR   * inp.rudder_rad
                  + bi2vel  * (CL_P * p + CL_R * r);

    let roll_moment = qbar * s * b * cl_moment;

    // ── Yaw moment ────────────────────────────────────────────────────────
    let cn = CN_BETA * beta
           + CN_DA   * inp.aileron_rad
           + CN_DR   * inp.rudder_rad
           + bi2vel  * (CN_P * p + CN_R * r);

    let yaw_moment = qbar * s * b * cn;

    AeroOut {
        force_body,
        moment_body: Vec3::new(roll_moment, pitch_moment, yaw_moment),
        qbar,
        alpha,
        beta,
        vt,
    }
}
