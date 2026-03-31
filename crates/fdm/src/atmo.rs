//! ISA 1976 Standard Atmosphere
//!
//! All quantities use Imperial units:
//!   altitude  – feet (geometric ≈ geopotential for low altitudes)
//!   temperature – degrees Rankine (°R)
//!   pressure    – lbf/ft²  (psf)
//!   density     – slug/ft³
//!   sound speed – ft/s
//!   kinematic viscosity – ft²/s

// ── Constants ─────────────────────────────────────────────────────────────────

/// Specific gas constant for dry air – ft²/(s²·°R)
const R_DRY: f64 = 1716.49;
/// Ratio of specific heats for air
const GAMMA: f64 = 1.4;
/// Standard gravity – ft/s²
pub const G0: f64 = 32.174;

// Standard sea-level values
pub const SL_TEMP_R: f64 = 518.67;
pub const SL_PRESSURE_PSF: f64 = 2116.22;
pub const SL_DENSITY: f64 = SL_PRESSURE_PSF / (R_DRY * SL_TEMP_R); // ≈ 0.002377 slug/ft³

// Geopotential altitude breakpoints (ft) and their lapse rates (°R/ft)
// Layers from U.S. Standard Atmosphere 1976, Table 1
//   Idx  GeoPot top (ft)   Lapse rate (°R/ft)
//   0    36089.2           −3.5663e-3  (troposphere)
//   1    65616.8            0.0         (lower stratosphere)
//   2   104986.9           +5.4864e-4
//   3   154199.5           +1.5360e-3
//   4   167322.8            0.0
//   5   232939.6           −1.5594e-3
//   6   278385.8           −2.0850e-3

const LAYER_TOPS: &[f64] = &[
    36_089.239,
    65_616.798,
    104_986.877,
    154_199.475,
    167_322.835,
    232_939.633,
    278_385.827,
    298_556.430, // dummy upper bound
];

const BASE_TEMPS: &[f64] = &[
    518.67,  // 0 ft
    389.97,  // 36089 ft
    389.97,  // 65617 ft
    411.57,  // 104987 ft
    487.17,  // 154199 ft
    487.17,  // 167323 ft
    386.37,  // 232940 ft
    336.50,  // 278386 ft
];

const LAPSE_RATES: &[f64] = &[
    -3.56620e-3,  // troposphere
    0.0,          // lower stratosphere
    5.48640e-4,
    1.53600e-3,
    0.0,
    -1.55940e-3,
    -2.08500e-3,
    0.0,
];

// Pressure at each layer base (computed once at startup, but we compute on the fly)
// Exponent for pressure ratio in gradient layers: g0 / (R * lapse)
fn pressure_exponent(lapse: f64) -> f64 {
    -G0 / (R_DRY * lapse)
}

// ── Public API ────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
pub struct AtmoState {
    pub temperature_r: f64,  // °R
    pub pressure_psf: f64,   // lbf/ft²
    pub density: f64,         // slug/ft³
    pub sound_speed: f64,     // ft/s
    pub kin_viscosity: f64,   // ft²/s  (Sutherland's law)
}

/// Calculate ISA 1976 atmosphere at a given geometric altitude (ft).
pub fn atmosphere(altitude_ft: f64) -> AtmoState {
    // Clamp to range
    let h = altitude_ft.clamp(0.0, 295_000.0);

    // Walk through layers to find temperature and pressure.
    let mut temp = BASE_TEMPS[0];
    let mut pressure = SL_PRESSURE_PSF;
    let mut layer_base = 0.0_f64;

    for i in 0..LAYER_TOPS.len() {
        let top = LAYER_TOPS[i];
        let lapse = LAPSE_RATES[i];
        let base_temp = BASE_TEMPS[i];

        let layer_altitude = h.min(top) - layer_base;

        if lapse.abs() < 1e-12 {
            // Isothermal layer
            temp = base_temp;
            let exponent = -G0 * layer_altitude / (R_DRY * temp);
            pressure *= exponent.exp();
        } else {
            // Gradient layer: T = T_base + lapse*dh
            temp = base_temp + lapse * layer_altitude;
            let exp = pressure_exponent(lapse);
            pressure *= (temp / base_temp).powf(exp);
        }

        if h <= top {
            break;
        }

        layer_base = top;
    }

    let density = pressure / (R_DRY * temp);
    let sound_speed = (GAMMA * R_DRY * temp).sqrt();

    // Kinematic viscosity via Sutherland's law (converted to Rankine/Imperial)
    // μ = 2.333e-8 * T^1.5 / (T + 198.6)  [slug/(ft·s)]  (Sutherland)
    let mu = 2.333e-8 * temp.powf(1.5) / (temp + 198.6);
    let kin_viscosity = mu / density;

    AtmoState {
        temperature_r: temp,
        pressure_psf: pressure,
        density,
        sound_speed,
        kin_viscosity,
    }
}
