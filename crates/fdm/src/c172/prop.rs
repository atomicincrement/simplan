//! Simplified propulsion model for the C172p (Lycoming IO-320, 150 HP).
//!
//! The engine drives a fixed-pitch 75-inch (1.905 m) diameter propeller.
//! We use actuator-disk theory to compute the propeller-induced velocity,
//! which is needed by the aerodynamics module (induced dynamic pressure for
//! tail surfaces immersed in the slipstream).

use crate::atmo::SL_DENSITY;

/// Propeller disk radius (ft).  75 in = 6.25 ft diameter ÷ 2.
const PROP_RADIUS_FT: f64 = 75.0 / 24.0; // 3.125 ft
/// Propeller disk area (ft²).
pub const PROP_DISK_AREA: f64 = std::f64::consts::PI * PROP_RADIUS_FT * PROP_RADIUS_FT;

/// Maximum static thrust at sea level (lbf).  Approximate for IO-320 / C172.
/// In climb at Vy the propulsive force is closer to 600–700 lbf; we use the
/// static value and let the actuator-disk correction handle the speed effect.
const T_MAX_SL: f64 = 900.0; // lbf

/// Thrust-speed correction exponent (Gagg-Ferrar approximation for
/// propeller efficiency variation with airspeed).
/// T(V) ≈ T_static * (1 − kT * V_tas / V_ref)
/// For a fixed-pitch prop in the C172 operating envelope we use a
/// linear thrust falloff reaching ~0 at ≈220 kt (372 ft/s).
const V_ZERO_THRUST: f64 = 372.0; // ft/s  (speed at which thrust = 0)

/// Density altitude correction: thrust scales with air density.
const DENSITY_EXP: f64 = 0.8; // empirical (0.7 … 1.0)

#[derive(Debug, Clone, Copy)]
pub struct PropState {
    /// Net thrust along the body X axis (lbf).
    pub thrust_lbf: f64,
    /// Actuator-disk induced velocity (ft/s) in the slipstream.
    pub v_induced: f64,
}

/// Compute thrust and induced velocity.
///
/// * `throttle`   – [0, 1]
/// * `u_fwd`      – airspeed component along body-X (ft/s), positive forward
/// * `rho`        – local air density (slug/ft³)
pub fn propulsion(throttle: f64, u_fwd: f64, rho: f64) -> PropState {
    let thr = throttle.clamp(0.0, 1.0);

    // Density correction
    let density_ratio = (rho / SL_DENSITY).powf(DENSITY_EXP);

    // Static thrust available at this throttle and altitude
    let t_static = thr * T_MAX_SL * density_ratio;

    // Speed correction: linear falloff (simplified fixed-pitch prop model)
    let speed_factor = (1.0 - u_fwd.max(0.0) / V_ZERO_THRUST).clamp(0.0, 1.0);
    let thrust = t_static * speed_factor;

    // Induced velocity via actuator-disk momentum theory (static approximation).
    // T = 2 * rho * A * v_i²  →  v_i = sqrt(T / (2 * rho * A))
    let v_induced = if rho > 0.0 && thrust > 0.0 {
        (thrust / (2.0 * rho * PROP_DISK_AREA)).sqrt()
    } else {
        0.0
    };

    PropState {
        thrust_lbf: thrust,
        v_induced,
    }
}
