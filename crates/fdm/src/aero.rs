//! C172p aerodynamics – translated from c172p.xml
//!
//! **Frame conventions (matching JSBSim):**
//! - Body:  X forward, Y right, Z down.
//! - Wind:  X along aerodynamic velocity, Y right, Z down-ish.
//! - Forces are first assembled in DRAG / SIDE / LIFT wind-frame axes
//!   (positive drag = magnitude of retarding force; positive lift = upward),
//!   then transformed to body frame via [`crate::math::dcm_wind2body`].
//! - Moments are assembled directly in body-frame ROLL / PITCH / YAW axes.
//!
//! **Reference geometry (from metrics block):**
//! - Wing area S = 174 ft²
//! - Wingspan  b = 35.8 ft
//! - Chord     c = 4.9 ft

use crate::math::{dcm_wind2body, interp1, interp2, Vec3};

// ── Aircraft geometry ─────────────────────────────────────────────────────────

pub const WING_AREA: f64 = 174.0; // ft²
pub const WINGSPAN: f64 = 35.8; // ft – b
pub const CHORD: f64 = 4.9; // ft – c̄

// ── Aerodynamic table data (c172p.xml) ───────────────────────────────────────

// --- Ground-effect correction tables ----------------------------------------

const KCDGE_TABLE: &[(f64, f64)] = &[
    (0.00, 0.480), (0.10, 0.515), (0.15, 0.629), (0.20, 0.709),
    (0.30, 0.815), (0.40, 0.882), (0.50, 0.928), (0.60, 0.962),
    (0.70, 0.988), (0.80, 1.000), (0.90, 1.000), (1.00, 1.000),
    (1.10, 1.000),
];

const KCLGE_TABLE: &[(f64, f64)] = &[
    (0.00, 1.203), (0.10, 1.127), (0.15, 1.090), (0.20, 1.073),
    (0.30, 1.046), (0.40, 1.055), (0.50, 1.019), (0.60, 1.013),
    (0.70, 1.008), (0.80, 1.006), (0.90, 1.003), (1.00, 1.002),
    (1.10, 1.000),
];

// --- DRAG: CDwbh 2-D table (rows=alpha_rad, cols=flap_deg) ------------------

const CDWBH_ALPHA: &[f64] = &[
    -0.0873, -0.0698, -0.0524, -0.0349, -0.0175,
     0.0000,  0.0175,  0.0349,  0.0524,  0.0698,
     0.0873,  0.1047,  0.1222,  0.1396,  0.1571,
     0.1745,  0.1920,  0.2094,  0.2269,  0.2443,
     0.2618,  0.2793,  0.2967,  0.3142,  0.3316,  0.3491,
];
const CDWBH_FLAP: &[f64] = &[0.0, 10.0, 20.0, 30.0];
const CDWBH_DATA: &[&[f64]] = &[
    &[0.0041, 0.0000, 0.0005, 0.0014],
    &[0.0013, 0.0004, 0.0025, 0.0041],
    &[0.0001, 0.0023, 0.0059, 0.0084],
    &[0.0003, 0.0057, 0.0108, 0.0141],
    &[0.0020, 0.0105, 0.0172, 0.0212],
    &[0.0052, 0.0168, 0.0251, 0.0299],
    &[0.0099, 0.0248, 0.0346, 0.0402],
    &[0.0162, 0.0342, 0.0457, 0.0521],
    &[0.0240, 0.0452, 0.0583, 0.0655],
    &[0.0334, 0.0577, 0.0724, 0.0804],
    &[0.0442, 0.0718, 0.0881, 0.0968],
    &[0.0566, 0.0874, 0.1053, 0.1148],
    &[0.0706, 0.1045, 0.1240, 0.1343],
    &[0.0860, 0.1232, 0.1442, 0.1554],
    &[0.0962, 0.1353, 0.1573, 0.1690],
    &[0.1069, 0.1479, 0.1708, 0.1830],
    &[0.1180, 0.1610, 0.1849, 0.1975],
    &[0.1298, 0.1746, 0.1995, 0.2126],
    &[0.1424, 0.1892, 0.2151, 0.2286],
    &[0.1565, 0.2054, 0.2323, 0.2464],
    &[0.1727, 0.2240, 0.2521, 0.2667],
    &[0.1782, 0.2302, 0.2587, 0.2735],
    &[0.1716, 0.2227, 0.2507, 0.2653],
    &[0.1618, 0.2115, 0.2388, 0.2531],
    &[0.1475, 0.1951, 0.2214, 0.2351],
    &[0.1097, 0.1512, 0.1744, 0.1866],
];

// CDDf 1-D (drag increment due to flaps)
const CDDF_TABLE: &[(f64, f64)] = &[
    (0.0, 0.0000), (10.0, 0.0070), (20.0, 0.0120), (30.0, 0.0180),
];

// --- SIDE: CYb 2-D (beta_rad × flap_deg) ------------------------------------

const CYB_BETA: &[f64] = &[-0.349, 0.0, 0.349];
const CYB_FLAP: &[f64] = &[0.0, 30.0];
const CYB_DATA: &[&[f64]] = &[
    &[0.137, 0.106],
    &[0.000, 0.000],
    &[-0.137, -0.106],
];

// CYp 2-D (alpha × flap)
const CYP_ALPHA: &[f64] = &[0.0, 0.094];
const CYP_FLAP: &[f64] = &[0.0, 30.0];
const CYP_DATA: &[&[f64]] = &[
    &[-0.0750, -0.1610],
    &[-0.1450, -0.2310],
];

// CYr 2-D (alpha × flap)
const CYR_ALPHA: &[f64] = &[0.0, 0.094];
const CYR_FLAP: &[f64] = &[0.0, 30.0];
const CYR_DATA: &[&[f64]] = &[
    &[0.2140, 0.1620],
    &[0.2670, 0.2150],
];

// --- LIFT: CLwbh 2-D (alpha × stall_hyst) -----------------------------------

const CLWBH_ALPHA: &[f64] = &[
    -0.09, 0.00, 0.09, 0.10, 0.12, 0.14, 0.16, 0.17,
     0.19, 0.21, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36,
];
const CLWBH_HYST: &[f64] = &[0.0, 1.0];
const CLWBH_DATA: &[&[f64]] = &[
    &[-0.22, -0.22],
    &[ 0.25,  0.25],
    &[ 0.73,  0.73],
    &[ 0.83,  0.78],
    &[ 0.92,  0.79],
    &[ 1.02,  0.81],
    &[ 1.08,  0.82],
    &[ 1.13,  0.83],
    &[ 1.19,  0.85],
    &[ 1.25,  0.86],
    &[ 1.35,  0.88],
    &[ 1.44,  0.90],
    &[ 1.47,  0.92],
    &[ 1.43,  0.95],
    &[ 1.38,  0.99],
    &[ 1.30,  1.05],
    &[ 1.15,  1.15],
];

// CLDf 1-D
const CLDF_TABLE: &[(f64, f64)] = &[
    (0.0, 0.00), (10.0, 0.20), (20.0, 0.30), (30.0, 0.35),
];

// --- ROLL: Clb 1-D (beta → coefficient) ------------------------------------

const CLB_TABLE: &[(f64, f64)] = &[
    (-0.349, 0.0322), (0.0, 0.0000), (0.349, -0.0322),
];

// Clr 2-D (alpha × flap)
const CLR_ALPHA: &[f64] = &[0.0, 0.094];
const CLR_FLAP: &[f64] = &[0.0, 30.0];
const CLR_DATA: &[&[f64]] = &[
    &[0.0798, 0.1246],
    &[0.1869, 0.2317],
];

// --- PITCH: Cmdf 1-D (flap → moment increment) ─────────────────────────────

const CMDF_TABLE: &[(f64, f64)] = &[
    (0.0, 0.0000), (10.0, -0.0654), (20.0, -0.0981), (30.0, -0.1140),
];

// --- YAW: Cnb 1-D (beta → coefficient) ─────────────────────────────────────

const CNB_TABLE: &[(f64, f64)] = &[
    (-0.349, -0.0205), (0.0, 0.0000), (0.349, 0.0205),
];

// ── Public API ────────────────────────────────────────────────────────────────

/// Aerodynamic output: forces and moments in the body frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct AeroOut {
    /// Total aerodynamic force in body frame [Fx, Fy, Fz] (lbf).
    /// Body: X forward, Y right, Z down.
    pub force_body: Vec3,
    /// Total aerodynamic moment in body frame [L, M, N] (lbf·ft).
    /// Positive L = roll right, positive M = pitch up, positive N = yaw right.
    pub moment_body: Vec3,
    /// Dynamic pressure (psf) – for information / other modules.
    pub qbar: f64,
    /// Angle of attack (rad).
    pub alpha: f64,
    /// Sideslip angle (rad).
    pub beta: f64,
    /// True airspeed (ft/s).
    pub vt: f64,
}

/// Inputs to the aerodynamic model.
#[derive(Debug, Clone, Copy)]
pub struct AeroIn {
    /// Body-frame velocity relative to airmass (ft/s): [u, v, w].
    pub vel_aero: Vec3,
    /// Body angular rate (rad/s): [p, q, r].
    pub pqr: Vec3,
    /// Alpha-dot from last step (rad/s) – used for pitch moment / lift damping.
    pub alpha_dot: f64,
    /// Air density (slug/ft³).
    pub rho: f64,
    /// Speed of sound (ft/s).
    pub sound_speed: f64,
    /// Elevator deflection (rad):  positive = trailing edge down = nose-up.
    pub elevator_rad: f64,
    /// Left aileron deflection (rad): positive = right roll.
    pub aileron_rad: f64,
    /// Rudder deflection (rad): positive = yaw right.
    pub rudder_rad: f64,
    /// Flap angle (deg): 0, 10, 20, or 30.
    pub flap_deg: f64,
    /// Propeller-induced velocity (ft/s) for tail-immersed dynamic pressure.
    pub v_induced: f64,
    /// Height above ground / MAC (dimensionless) for ground-effect tables.
    /// Set to >1.1 when airborne (no ground effect).
    pub hoverbmac: f64,
    /// Stall hysteresis flag (0 = no stall, 1 = in stall).
    pub stall_hyst: f64,
}

pub fn aerodynamics(inp: &AeroIn) -> AeroOut {
    let u = inp.vel_aero.x;
    let v = inp.vel_aero.y;
    let w = inp.vel_aero.z;

    // ── Angles & airspeed ─────────────────────────────────────────────────────
    let muw = u * u + w * w;
    let vt2 = muw + v * v;
    let vt = vt2.sqrt();

    let alpha = if muw >= 1e-6 { w.atan2(u) } else { 0.0 };
    let beta = if vt > 0.001 { v.atan2(muw.sqrt()) } else { 0.0 };

    let qbar = 0.5 * inp.rho * vt2;

    // Rate convenience
    let rho = inp.rho;
    let p = inp.pqr.x;
    let q = inp.pqr.y;
    let r = inp.pqr.z;
    let adot = inp.alpha_dot;

    let flap = inp.flap_deg;

    // b/(2Vt) and c/(2Vt) for dimensionless rate scaling
    let two_vt = 2.0 * vt.max(1.0);
    let bi2vel = WINGSPAN / two_vt;
    let ci2vel = CHORD / two_vt;

    // ── Ground-effect corrections ─────────────────────────────────────────────
    let h_mac = inp.hoverbmac;
    let k_cd_ge = interp1(KCDGE_TABLE, h_mac);
    let k_cl_ge = interp1(KCLGE_TABLE, h_mac);

    // ── Induced qbar (for tail surfaces in propwash) ──────────────────────────
    // From the XML: velocity_induced = u_aero + 2 * v_induced
    let v_ind = u + 2.0 * inp.v_induced;
    let qbar_induced = 0.5 * rho * v_ind * v_ind;

    let s = WING_AREA;
    let b = WINGSPAN;
    let c = CHORD;

    // ─────────────────────────────────────────────────────────────────────────
    // A S S E M B L E   W I N D - F R A M E   F O R C E S
    // Sign convention (before transformation):
    //   drag_raw  = positive magnitude (will be negated before Tw2b multiply)
    //   side_raw  = positive rightward
    //   lift_raw  = positive upward  (will be negated before Tw2b multiply)
    // ─────────────────────────────────────────────────────────────────────────

    // ── DRAG ─────────────────────────────────────────────────────────────────
    let cd_o = 0.027;
    let cd_df_coeff = interp1(CDDF_TABLE, flap);
    let cd_wbh_coeff = interp2(CDWBH_ALPHA, CDWBH_FLAP, CDWBH_DATA, alpha, flap);
    let cd_de = inp.elevator_rad.abs() * 0.06;
    let cd_beta = beta.abs() * 0.17;

    let drag = qbar * s * (cd_o
        + k_cd_ge * (cd_df_coeff + cd_wbh_coeff)
        + cd_de
        + cd_beta);

    // ── SIDE FORCE ───────────────────────────────────────────────────────────
    let cy_b = interp2(CYB_BETA, CYB_FLAP, CYB_DATA, beta, flap);
    let cy_dr = inp.rudder_rad * 0.187;
    let cy_p_coeff = interp2(CYP_ALPHA, CYP_FLAP, CYP_DATA, alpha, flap);
    let cy_r_coeff = interp2(CYR_ALPHA, CYR_FLAP, CYR_DATA, alpha, flap);

    let side = qbar * s * (cy_b + cy_dr + bi2vel * (cy_p_coeff * p + cy_r_coeff * r));

    // ── LIFT ──────────────────────────────────────────────────────────────────
    let cl_wbh = interp2(CLWBH_ALPHA, CLWBH_HYST, CLWBH_DATA, alpha, inp.stall_hyst);
    let cl_df = interp1(CLDF_TABLE, flap);
    let cl_de = inp.elevator_rad * 0.43;
    let cl_adot = adot * ci2vel * 1.7;
    let cl_q = q * ci2vel * 3.9;

    let lift = qbar * s * (k_cl_ge * (cl_wbh + cl_df) + cl_de + cl_adot + cl_q);

    // ── Transform wind → body ─────────────────────────────────────────────────
    // JSBSim: vFnative[Drag] *= -1; vFnative[Lift] *= -1; then Tw2b * vFnative.
    // With our sign (before negation): drag > 0, side is signed, lift > 0.
    let f_wind = Vec3::new(-drag, side, -lift); // after JSBSim sign flip
    let tw2b = dcm_wind2body(alpha, beta);
    let force_body = tw2b.mul_vec(f_wind);

    // ─────────────────────────────────────────────────────────────────────────
    // A S S E M B L E   B O D Y - F R A M E   M O M E N T S
    // Each moment term already has reference geometry multiplied in.
    // ─────────────────────────────────────────────────────────────────────────

    // ── ROLL moment L (lbf·ft) ───────────────────────────────────────────────
    let cl_b = interp1(CLB_TABLE, beta);
    let cl_p = bi2vel * p * (-0.484);
    let cl_r_coeff = interp2(CLR_ALPHA, CLR_FLAP, CLR_DATA, alpha, flap);
    let cl_r = bi2vel * r * cl_r_coeff;
    let cl_da = inp.aileron_rad * 0.229;
    let cl_dr = inp.rudder_rad * 0.0147;

    let roll_moment = qbar * s * b * (cl_b + cl_p + cl_r + cl_da + cl_dr);

    // ── PITCH moment M (lbf·ft) ──────────────────────────────────────────────
    let cm_o = 0.1;
    let cm_alpha = alpha * (-1.8);
    let cm_q = q * ci2vel * (-12.4);
    let cm_adot = adot * ci2vel * (-7.27);
    // Cmde uses qbar_induced and our sign: positive elevator_rad = nose up,
    // coefficient −1.122 means TE-down deflection creates nose-up moment.
    let cm_de = inp.elevator_rad * (-1.122);
    let cm_df = interp1(CMDF_TABLE, flap);

    let pitch_moment = qbar * s * c * (cm_o + cm_alpha + cm_q + cm_adot + cm_df)
        + qbar_induced * s * c * cm_de;

    // ── YAW moment N (lbf·ft) ────────────────────────────────────────────────
    let cn_b = interp1(CNB_TABLE, beta);
    let cn_p = bi2vel * p * (-0.0278);
    let cn_r = bi2vel * r * (-0.0937);
    let cn_da = inp.aileron_rad * (-0.0053);
    let cn_dr = inp.rudder_rad * (-0.043);

    let yaw_moment = qbar * s * b * (cn_b + cn_p + cn_r + cn_da)
        + qbar_induced * s * b * cn_dr;

    AeroOut {
        force_body,
        moment_body: Vec3::new(roll_moment, pitch_moment, yaw_moment),
        qbar,
        alpha,
        beta,
        vt,
    }
}
