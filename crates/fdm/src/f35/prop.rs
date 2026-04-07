//! F100-PW-229 engine model.
//!
//! The General Electric / Pratt & Whitney F100-PW-229 is the turbofan used
//! in the F-16C/D Block 50/52.  Data from the JSBSim engine file
//! `engine/F100-PW-229.xml` (Aeromatic v0.8).
//!
//! Thrust ratings (sea-level static, ISA):
//!   Military (MIL) power  – dry   ≈ 17,800 lbf
//!   Afterburner  (MAX) power      ≈ 29,000 lbf
//!
//! The throttle is treated as continuous.
//!   throttle ∈ [0.00, 0.83) → scaled military power: 0 … 17,800 lbf
//!   throttle ∈ [0.83, 1.00] → scaled afterburner:    17,800 … 29,000 lbf
//!
//! Thrust is corrected for altitude (density ratio) and Mach number using
//! piecewise-linear tables derived from the JSBSim MilThrust / AugThrust
//! 2-D tables evaluated at sea level.
//!
//! No engine inertia or spool-up modelling is included.

use crate::atmo::SL_DENSITY;

// ── Engine constants ─────────────────────────────────────────────────────────

/// Military (dry) thrust at SL static (lbf).  Source: F100-PW-229.xml.
const T_MIL_SL: f64 = 17_800.0;

/// Afterburner thrust at SL static (lbf).  Source: F100-PW-229.xml.
const T_MAX_SL: f64 = 29_000.0;

/// Throttle setting at which afterburner lights.
const AB_THRESHOLD: f64 = 0.83;

/// Density-ratio exponent for altitude correction.
/// From F100 MilThrust table at Mach 0, SL→60 kft the thrust ratio
/// tracks ρ/ρ₀ closely (bypass ratio 0.4, predominantly turbofan).
const DENSITY_EXP: f64 = 1.00;

/// Military thrust Mach multiplier at SL (from JSBSim MilThrust table,
/// altitude = 0 ft column).
const MIL_MACH_TABLE: &[(f64, f64)] = &[
    (0.0, 1.0000),
    (0.2, 0.9340),
    (0.4, 0.9210),
    (0.6, 0.9510),
    (0.8, 1.0200),
    (1.0, 1.1200),
    (1.2, 1.2300),
    (1.4, 1.3400),
];

/// Afterburner thrust Mach multiplier at SL (from JSBSim AugThrust table,
/// altitude = 0 ft column).
const AUG_MACH_TABLE: &[(f64, f64)] = &[
    (0.0, 1.0000),
    (0.2, 0.9599),
    (0.4, 0.9474),
    (0.6, 0.9589),
    (0.8, 0.9942),
    (1.0, 1.0529),
    (1.2, 1.1254),
    (1.4, 1.2149),
    (1.6, 1.3260),
    (1.8, 1.4579),
    (2.0, 1.5700),
];

// ── Public types ──────────────────────────────────────────────────────────────

/// Propulsion output for the F-35A.
#[derive(Debug, Clone, Copy)]
pub struct PropState {
    /// Net thrust along the body X axis (forward), lbf.
    pub thrust_lbf: f64,
    /// Exhaust Mach number (for future exhaust plume / noise modelling).
    pub mach_exit: f64,
}

// ── Thrust computation ────────────────────────────────────────────────────────

/// Compute thrust given throttle position, airspeed and atmospheric conditions.
///
/// * `throttle`    – [0, 1]
/// * `mach`        – flight Mach number
/// * `rho`         – local air density (slug/ft³)
pub fn propulsion(throttle: f64, mach: f64, rho: f64) -> PropState {
    use crate::math::interp1;

    let thr = throttle.clamp(0.0, 1.0);

    // ── SL thrust (throttle-scaled) + Mach correction ─────────────────────
    let thrust_lbf = if thr < AB_THRESHOLD {
        let frac    = thr / AB_THRESHOLD;
        let t_gross = frac * T_MIL_SL;
        let mach_f  = interp1(MIL_MACH_TABLE, mach);
        let sigma   = (rho / SL_DENSITY).max(0.0).powf(DENSITY_EXP);
        t_gross * mach_f * sigma
    } else {
        let ab_frac = (thr - AB_THRESHOLD) / (1.0 - AB_THRESHOLD);
        let t_gross = T_MIL_SL + ab_frac * (T_MAX_SL - T_MIL_SL);
        let mach_f  = interp1(AUG_MACH_TABLE, mach);
        let sigma   = (rho / SL_DENSITY).max(0.0).powf(DENSITY_EXP);
        t_gross * mach_f * sigma
    };

    // Rough exit Mach (for reference; not used in EOM).
    let mach_exit = if thr > AB_THRESHOLD { 1.8 } else { 1.0 };

    PropState { thrust_lbf, mach_exit }
}
