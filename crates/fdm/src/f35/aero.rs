//! F-16A aerodynamic model.
//!
//! Data are taken verbatim from the JSBSim F-16 model
//! (`aircraft/f16/f16.xml`, Aeromatic v0.8 / Nguyen et al. NASA TM-1979).
//!
//! **Coordinate frame** (JSBSim convention):
//! - Body:  X forward, Y right, Z down.
//! - Wind frame forces: DRAG (+X), SIDE (+Y), LIFT (+Z-down).
//! - Forces assembled in wind frame then rotated to body via
//!   [`crate::math::dcm_wind2body`].
//! - Moments assembled directly in body frame (L, M, N).
//!
//! **Reference geometry (F-16A from JSBSim):**
//! - Wing area   S  = 300 ft²
//! - Wingspan    b  = 30.0 ft
//! - MAC         c̄  = 11.32 ft
//!
//! **Units:** Imperial (ft, slug, lbf, s, rad) throughout.

use crate::math::{dcm_wind2body, interp1, Vec3};

// ── Reference geometry ───────────────────────────────────────────────────────

pub const WING_AREA: f64 = 300.0;  // ft²  – S
pub const WINGSPAN:  f64 =  30.0;  // ft   – b
pub const CHORD:     f64 =  11.32; // ft   – c̄ (mean aerodynamic chord)

// ── Common alpha breakpoints (12 rows, shared by every table) ─────────────────
//
//  Matches the JSBSim XML: -10° … +45° in 5° steps.

const ALPHA_BREAKS: [f64; 12] = [
    -0.1750, -0.0870,  0.0000,  0.0870,
     0.1750,  0.2620,  0.3490,  0.4360,
     0.5240,  0.6110,  0.6980,  0.7850,
];

// ── Elevator breakpoints (5 columns) ─────────────────────────────────────────
//
//  Positive elevator = nose-down deflection (JSBSim sign).
//  Our Controls convention uses positive elevator = nose-UP command; the
//  caller negates before looking up.

const DE_BREAKS: [f64; 5] = [-0.4360, -0.2180, 0.0000, 0.2180, 0.4360];

// ── 2-D bilinear interpolation helper ────────────────────────────────────────

/// Bilinear table lookup for a row-major `[[f64; C]; R]` array.
fn bl<const R: usize, const C: usize>(
    row_keys: &[f64; R],
    col_keys: &[f64; C],
    data:     &[[f64; C]; R],
    rval:     f64,
    cval:     f64,
) -> f64 {
    let ri = {
        let mut i = 0;
        while i < R - 1 && row_keys[i + 1] < rval { i += 1; }
        i.min(R - 2)
    };
    let ci = {
        let mut i = 0;
        while i < C - 1 && col_keys[i + 1] < cval { i += 1; }
        i.min(C - 2)
    };
    let tr = if row_keys[ri + 1] != row_keys[ri] {
        ((rval - row_keys[ri]) / (row_keys[ri + 1] - row_keys[ri])).clamp(0.0, 1.0)
    } else { 0.0 };
    let tc = if col_keys[ci + 1] != col_keys[ci] {
        ((cval - col_keys[ci]) / (col_keys[ci + 1] - col_keys[ci])).clamp(0.0, 1.0)
    } else { 0.0 };
    let v0 = data[ri    ][ci] + tc * (data[ri    ][ci + 1] - data[ri    ][ci]);
    let v1 = data[ri + 1][ci] + tc * (data[ri + 1][ci + 1] - data[ri + 1][ci]);
    v0 + tr * (v1 - v0)
}

// ── LIFT:  CL(alpha, de)  ─────────────────────────────────────────────────────
//
//  Source: `aero/coefficient/CLDh`, JSBSim F-16 model.

const CL_TABLE: [[f64; 5]; 12] = [
    // de: -0.436  -0.218   0.000   0.218   0.436
    [-0.6590, -0.7090, -0.7540, -0.7920, -0.8250],  // α = -10°
    [-0.1510, -0.1960, -0.2380, -0.2780, -0.3160],  // α =  -5°
    [ 0.1830,  0.1410,  0.1000,  0.0590,  0.0170],  // α =   0°
    [ 0.4910,  0.4540,  0.4140,  0.3710,  0.3260],  // α =   5°
    [ 0.7970,  0.7630,  0.7250,  0.6800,  0.6300],  // α =  10°
    [ 1.1080,  1.0790,  1.0410,  0.9930,  0.9400],  // α =  15°
    [ 1.3950,  1.3660,  1.3270,  1.2740,  1.2140],  // α =  20°
    [ 1.6150,  1.5870,  1.5470,  1.4900,  1.4270],  // α =  25°
    [ 1.8040,  1.7770,  1.7370,  1.6740,  1.6100],  // α =  30°
    [ 1.9000,  1.8720,  1.8290,  1.7660,  1.6990],  // α =  35°
    [ 1.8980,  1.8690,  1.8220,  1.7570,  1.6890],  // α =  40°
    [ 1.7530,  1.7240,  1.6740,  1.6120,  1.5460],  // α =  45°
];

// ── DRAG:  CD(alpha, de)  ─────────────────────────────────────────────────────
//
//  Source: `aero/coefficient/CDDh`.

const CD_TABLE: [[f64; 5]; 12] = [
    // de: -0.436  -0.218   0.000   0.218   0.436
    [ 0.2170,  0.1740,  0.1560,  0.1810,  0.2300],
    [ 0.0940,  0.0550,  0.0410,  0.0620,  0.1010],
    [ 0.0810,  0.0400,  0.0210,  0.0390,  0.0760],
    [ 0.1060,  0.0610,  0.0400,  0.0570,  0.1010],
    [ 0.1660,  0.1190,  0.0960,  0.1140,  0.1580],
    [ 0.2520,  0.2030,  0.1820,  0.2020,  0.2400],
    [ 0.4040,  0.3620,  0.3470,  0.3710,  0.4160],
    [ 0.6280,  0.5880,  0.5770,  0.6010,  0.6370],
    [ 0.8750,  0.8400,  0.8260,  0.8520,  0.8800],
    [ 1.1270,  1.0950,  1.0840,  1.1020,  1.1250],
    [ 1.3650,  1.3340,  1.3260,  1.3380,  1.3560],
    [ 1.5170,  1.4870,  1.4780,  1.4820,  1.4890],
];

// ── PITCH MOMENT:  Cm(alpha, de)  ────────────────────────────────────────────
//
//  Source: `aero/coefficient/CmDh`.

const CM_TABLE: [[f64; 5]; 12] = [
    // de: -0.436  -0.218   0.000   0.218   0.436
    [ 0.2050,  0.0810, -0.0460, -0.1740, -0.2590],
    [ 0.1680,  0.0770, -0.0200, -0.1450, -0.2020],
    [ 0.1860,  0.1070, -0.0090, -0.1210, -0.1840],
    [ 0.1960,  0.1100, -0.0050, -0.1270, -0.1930],
    [ 0.2130,  0.1100, -0.0060, -0.1290, -0.1990],
    [ 0.2510,  0.1410,  0.0100, -0.1020, -0.1500],
    [ 0.2450,  0.1270,  0.0060, -0.0970, -0.1600],
    [ 0.2380,  0.1190, -0.0010, -0.1130, -0.1670],
    [ 0.2520,  0.1330,  0.0140, -0.0870, -0.1040],
    [ 0.2310,  0.1080,  0.0000, -0.0840, -0.0760],
    [ 0.1980,  0.0810, -0.0130, -0.0690, -0.0410],
    [ 0.1920,  0.0930,  0.0320, -0.0060, -0.0050],
];

// ── CD transonic wave-drag rise  ─────────────────────────────────────────────

const CD_MACH: &[(f64, f64)] = &[
    (0.00, 0.0000), (0.81, 0.0000), (1.10, 0.0230), (1.80, 0.0150),
];

// ── Pitch-rate lift:  CLq(alpha)  ─────────────────────────────────────────────

const CLQ_ALPHA: &[(f64, f64)] = &[
    (-0.1750,  8.7127), (-0.0870, 25.7114), ( 0.0000, 28.9000),
    ( 0.0870, 31.3973), ( 0.1750, 31.0872), ( 0.2620, 30.4071),
    ( 0.3490, 26.9735), ( 0.4360, 26.4242), ( 0.5240, 25.8647),
    ( 0.6110, 25.2654), ( 0.6980, 30.5158), ( 0.7850, 25.8165),
];

// ── Pitch-rate moment:  Cmq(alpha)  ──────────────────────────────────────────

const CMQ_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -7.2100), (-0.0870, -5.4000), ( 0.0000, -5.2300),
    ( 0.0870, -5.2600), ( 0.1750, -6.1100), ( 0.2620, -6.6400),
    ( 0.3490, -5.6900), ( 0.4360, -6.0000), ( 0.5240, -6.2000),
    ( 0.6110, -6.4000), ( 0.6980, -6.6000), ( 0.7850, -6.0000),
];

// ── Side-force scalars  ───────────────────────────────────────────────────────

const CY_BETA: f64 = -1.1460; // /rad  (CYb)
const CY_DA:   f64 = -0.0226; // /rad  (CYDa)
const CY_DR:   f64 =  0.0860; // /rad  (CYdr)

// ── Side-force rate derivatives  ─────────────────────────────────────────────

const CYP_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -0.1080), (-0.0870, -0.1080), ( 0.0000, -0.1880),
    ( 0.0870,  0.1100), ( 0.1750,  0.2580), ( 0.2620,  0.2260),
    ( 0.3490,  0.3440), ( 0.4360,  0.3620), ( 0.5240,  0.6110),
    ( 0.6110,  0.5290), ( 0.6980,  0.2980), ( 0.7850, -0.2270),
];

const CYR_ALPHA: &[(f64, f64)] = &[
    (-0.1750,  0.8820), (-0.0870,  0.8520), ( 0.0000,  0.8760),
    ( 0.0870,  0.9580), ( 0.1750,  0.9620), ( 0.2620,  0.9740),
    ( 0.3490,  0.8190), ( 0.4360,  0.4830), ( 0.5240,  0.5900),
    ( 0.6110,  1.2100), ( 0.6980, -0.4930), ( 0.7850, -1.0400),
];

// ── Roll-moment derivatives  ──────────────────────────────────────────────────

/// dCl/dβ slope (central difference of Clb(alpha,beta) table at β = ±0.087 rad).
const CLB_SLOPE: &[(f64, f64)] = &[
    (-0.1750, -0.0115), (-0.0870, -0.0460), ( 0.0000, -0.0920),
    ( 0.0870, -0.1380), ( 0.1750, -0.1840), ( 0.2620, -0.2530),
    ( 0.3490, -0.2530), ( 0.4360, -0.2410), ( 0.5240, -0.1720),
    ( 0.6110, -0.0920), ( 0.6980, -0.1490), ( 0.7850, -0.1720),
];

const CLP_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -0.3600), (-0.0870, -0.3590), ( 0.0000, -0.4430),
    ( 0.0870, -0.4200), ( 0.1750, -0.3830), ( 0.2620, -0.3750),
    ( 0.3490, -0.3290), ( 0.4360, -0.2940), ( 0.5240, -0.2300),
    ( 0.6110, -0.2100), ( 0.6980, -0.1200), ( 0.7850, -0.1000),
];

const CLR_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -0.1260), (-0.0870, -0.0260), ( 0.0000,  0.0630),
    ( 0.0870,  0.1130), ( 0.1750,  0.2080), ( 0.2620,  0.2300),
    ( 0.3490,  0.3190), ( 0.4360,  0.4370), ( 0.5240,  0.6800),
    ( 0.6110,  0.1000), ( 0.6980,  0.4470), ( 0.7850, -0.3300),
];

/// Aileron roll effectiveness at β = 0.
const CLDA_ALPHA: &[(f64, f64)] = &[
    (-0.1750,  0.0400), (-0.0870,  0.0520), ( 0.0000,  0.0510),
    ( 0.0870,  0.0520), ( 0.1750,  0.0480), ( 0.2620,  0.0480),
    ( 0.3490,  0.0420), ( 0.4360,  0.0370), ( 0.5240,  0.0310),
    ( 0.6110,  0.0260), ( 0.6980,  0.0170), ( 0.7850,  0.0120),
];

/// Rudder-to-roll cross-coupling at β = 0.
const CLDR_ALPHA: &[(f64, f64)] = &[
    (-0.1750,  0.0180), (-0.0870,  0.0150), ( 0.0000,  0.0150),
    ( 0.0870,  0.0140), ( 0.1750,  0.0140), ( 0.2620,  0.0140),
    ( 0.3490,  0.0140), ( 0.4360,  0.0150), ( 0.5240,  0.0130),
    ( 0.6110,  0.0110), ( 0.6980,  0.0060), ( 0.7850,  0.0010),
];

// ── Yaw-moment derivatives  ───────────────────────────────────────────────────

/// dCn/dβ slope (central difference of Cnb(alpha,beta) table at β = ±0.087 rad).
const CNB_SLOPE: &[(f64, f64)] = &[
    (-0.1750,  0.2069), (-0.0870,  0.2184), ( 0.0000,  0.2069),
    ( 0.0870,  0.2184), ( 0.1750,  0.2184), ( 0.2620,  0.2069),
    ( 0.3490,  0.1494), ( 0.4360,  0.0805), ( 0.5240,  0.0460),
    ( 0.6110, -0.1609), ( 0.6980, -0.1954), ( 0.7850, -0.3793),
];

const CNP_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -0.0610), (-0.0870, -0.0520), ( 0.0000, -0.0520),
    ( 0.0870,  0.0120), ( 0.1750,  0.0130), ( 0.2620,  0.0240),
    ( 0.3490, -0.0500), ( 0.4360, -0.1500), ( 0.5240, -0.1300),
    ( 0.6110, -0.1580), ( 0.6980, -0.2400), ( 0.7850, -0.1500),
];

const CNR_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -0.3800), (-0.0870, -0.3630), ( 0.0000, -0.3780),
    ( 0.0870, -0.3860), ( 0.1750, -0.3700), ( 0.2620, -0.4530),
    ( 0.3490, -0.5500), ( 0.4360, -0.5820), ( 0.5240, -0.5950),
    ( 0.6110, -0.6370), ( 0.6980, -1.0200), ( 0.7850, -0.8400),
];

/// Adverse yaw due to aileron at β = 0.
const CNDA_ALPHA: &[(f64, f64)] = &[
    (-0.1750,  0.0110), (-0.0870,  0.0110), ( 0.0000,  0.0100),
    ( 0.0870,  0.0090), ( 0.1750,  0.0080), ( 0.2620,  0.0060),
    ( 0.3490,  0.0000), ( 0.4360, -0.0040), ( 0.5240, -0.0070),
    ( 0.6110, -0.0100), ( 0.6980, -0.0040), ( 0.7850, -0.0100),
];

/// Rudder yaw authority at β = 0.
const CNDR_ALPHA: &[(f64, f64)] = &[
    (-0.1750, -0.0480), (-0.0870, -0.0450), ( 0.0000, -0.0450),
    ( 0.0870, -0.0450), ( 0.1750, -0.0440), ( 0.2620, -0.0450),
    ( 0.3490, -0.0470), ( 0.4360, -0.0480), ( 0.5240, -0.0490),
    ( 0.6110, -0.0450), ( 0.6980, -0.0330), ( 0.7850, -0.0160),
];

// ── CL at de = 0 (for trim look-up) ──────────────────────────────────────────
//
//  Extracted from the de = 0.000 column of CL_TABLE.

pub const CL_ALPHA_TABLE: &[(f64, f64)] = &[
    (-0.1750, -0.7540), (-0.0870, -0.2380), ( 0.0000,  0.1000),
    ( 0.0870,  0.4140), ( 0.1750,  0.7250), ( 0.2620,  1.0410),
    ( 0.3490,  1.3270), ( 0.4360,  1.5470), ( 0.5240,  1.7370),
    ( 0.6110,  1.8290), ( 0.6980,  1.8220), ( 0.7850,  1.6740),
];

// ── Public API ────────────────────────────────────────────────────────────────

/// Aerodynamic output (forces and moments in body frame).
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

/// Inputs to the F-16A aerodynamic model.
#[derive(Debug, Clone, Copy)]
pub struct AeroIn {
    /// Body-frame velocity relative to airmass (ft/s): [u, v, w].
    pub vel_aero: Vec3,
    /// Body angular rate (rad/s): [p, q, r].
    pub pqr: Vec3,
    /// Alpha-dot (rad/s) from the previous integration step (reserved).
    pub alpha_dot: f64,
    /// Air density (slug/ft³).
    pub rho: f64,
    /// Speed of sound (ft/s).
    pub sound_speed: f64,
    /// Elevator deflection (rad): positive = nose-UP moment (our convention;
    /// negated internally before the JSBSim-convention table lookup).
    pub elevator_rad: f64,
    /// Differential aileron (rad): positive = roll right.
    pub aileron_rad: f64,
    /// Rudder deflection (rad): positive = yaw right.
    pub rudder_rad: f64,
}

/// Compute aerodynamic forces and moments for the F-16A.
pub fn aerodynamics(inp: &AeroIn) -> AeroOut {
    let u = inp.vel_aero.x;
    let v = inp.vel_aero.y;
    let w = inp.vel_aero.z;

    // ── Basic flow quantities ──────────────────────────────────────────────
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
    let bi2vel = WINGSPAN / two_vt;
    let ci2vel = CHORD   / two_vt;

    let s = WING_AREA;
    let b = WINGSPAN;
    let c = CHORD;

    // Convert to JSBSim elevator convention (positive = nose-down trailing-edge-up).
    let de_jsb = -inp.elevator_rad;
    let da = inp.aileron_rad;
    let dr = inp.rudder_rad;

    // ── LIFT ──────────────────────────────────────────────────────────────
    let cl = bl(&ALPHA_BREAKS, &DE_BREAKS, &CL_TABLE, alpha, de_jsb)
           + interp1(CLQ_ALPHA, alpha) * ci2vel * q;

    // ── DRAG ──────────────────────────────────────────────────────────────
    let cd = bl(&ALPHA_BREAKS, &DE_BREAKS, &CD_TABLE, alpha, de_jsb)
           + interp1(CD_MACH, mach);

    // ── SIDE FORCE ────────────────────────────────────────────────────────
    let cy = CY_BETA * beta
           + CY_DA   * da
           + CY_DR   * dr
           + bi2vel  * (interp1(CYP_ALPHA, alpha) * p
                      + interp1(CYR_ALPHA, alpha) * r);

    // ── Transform wind → body ──────────────────────────────────────────────
    let f_wind     = Vec3::new(-cd, cy, -cl);
    let force_body = qbar * s * dcm_wind2body(alpha, beta).mul_vec(f_wind);

    // ── PITCH MOMENT ──────────────────────────────────────────────────────
    let cm = bl(&ALPHA_BREAKS, &DE_BREAKS, &CM_TABLE, alpha, de_jsb)
           + interp1(CMQ_ALPHA, alpha) * ci2vel * q;

    let pitch_moment = qbar * s * c * cm;

    // ── ROLL MOMENT ───────────────────────────────────────────────────────
    let cl_m = interp1(CLB_SLOPE,  alpha) * beta
             + interp1(CLDA_ALPHA, alpha) * da
             + interp1(CLDR_ALPHA, alpha) * dr
             + bi2vel * (interp1(CLP_ALPHA, alpha) * p
                        + interp1(CLR_ALPHA, alpha) * r);

    let roll_moment = qbar * s * b * cl_m;

    // ── YAW MOMENT ────────────────────────────────────────────────────────
    let cn = interp1(CNB_SLOPE,  alpha) * beta
           + interp1(CNDA_ALPHA, alpha) * da
           + interp1(CNDR_ALPHA, alpha) * dr
           + bi2vel * (interp1(CNP_ALPHA, alpha) * p
                      + interp1(CNR_ALPHA, alpha) * r);

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
