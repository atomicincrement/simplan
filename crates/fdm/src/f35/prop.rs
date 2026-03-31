//! F135-PW-100 engine model.
//!
//! The Pratt & Whitney F135 is a low-bypass turbofan derived from the F119.
//! Thrust ratings (sea-level static, ISA):
//!   Military (MIL) power  – "dry"  ≈ 27,000 lbf
//!   Afterburner  (MAX) power        ≈ 43,000 lbf
//!
//! The throttle is treated as continuous.
//!   throttle ∈ [0.00, 0.83) → scaled military power: 0 … 27,000 lbf
//!   throttle ∈ [0.83, 1.00] → scaled afterburner:    27,000 … 43,000 lbf
//!
//! Thrust is corrected for altitude (density) and ram-recovery loss at speed.
//!
//! No engine inertia or spool-up modelling is included; throttle changes are
//! instantaneous.  This is adequate for initial flight-dynamics work.

use crate::atmo::SL_DENSITY;

// ── Engine constants ─────────────────────────────────────────────────────────

/// Military (dry) thrust at SL static (lbf).
const T_MIL_SL: f64 = 27_000.0;

/// Afterburner thrust at SL static (lbf).
const T_MAX_SL: f64 = 43_000.0;

/// Throttle setting at which afterburner lights.
const AB_THRESHOLD: f64 = 0.83;

/// Density-ratio exponent for altitude correction.
/// Turbofans: thrust ∝ (ρ/ρ₀)^0.9 approximately.
const DENSITY_EXP: f64 = 0.90;

/// Inlet ram-recovery model: thrust decreases with speed above ~Mach 0.5.
/// We model thrust as T_net = T_gross × ram_factor(Mach).
/// Simple piecewise linear (subsonic only – supersonic ram rise omitted for now).
const RAM_TABLE: &[(f64, f64)] = &[
    (0.0, 1.00),
    (0.4, 0.97),
    (0.8, 0.92),
    (1.2, 0.88),
    (1.6, 0.85),
    (2.0, 0.82),
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

    // ── Gross SL static thrust ────────────────────────────────────────────
    let t_sl = if thr < AB_THRESHOLD {
        // Military power range
        (thr / AB_THRESHOLD) * T_MIL_SL
    } else {
        // Afterburner range
        let ab_frac = (thr - AB_THRESHOLD) / (1.0 - AB_THRESHOLD);
        T_MIL_SL + ab_frac * (T_MAX_SL - T_MIL_SL)
    };

    // ── Altitude correction ───────────────────────────────────────────────
    let sigma = (rho / SL_DENSITY).max(0.0).powf(DENSITY_EXP);

    // ── Ram-recovery correction ───────────────────────────────────────────
    let ram = interp1(RAM_TABLE, mach);

    let thrust_lbf = t_sl * sigma * ram;

    // Rough exit Mach (for reference; not used in EOM).
    let mach_exit = if thr > AB_THRESHOLD { 1.8 } else { 1.0 };

    PropState { thrust_lbf, mach_exit }
}
