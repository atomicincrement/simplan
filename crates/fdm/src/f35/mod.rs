//! F-35A Lightning II – Flight Dynamics Model
//!
//! A 6-DoF rigid-body flight model for the F-35A.
//! Shares the atmosphere, equations-of-motion, and math modules from the
//! parent crate; only aerodynamics and propulsion are aircraft-specific.
//!
//! # Quick start
//!
//! ```rust
//! use fdm::f35::{FlightModel, Controls};
//!
//! // 10 000 ft, Mach 0.5 (≈ 889 ft/s), heading East
//! let mut fdm = FlightModel::new(10_000.0, 889.0, std::f64::consts::FRAC_PI_2);
//!
//! let controls = Controls {
//!     throttle: 0.60,   // military power
//!     elevator: 0.0,
//!     aileron:  0.0,
//!     rudder:   0.0,
//! };
//!
//! for _ in 0..600 {          // 10 s at dt = 1/60 s
//!     fdm.step(1.0 / 60.0, &controls);
//! }
//! println!("alt = {:.0} ft, Mach = {:.3}", fdm.state.altitude, fdm.state.mach);
//! ```

pub mod aero;
pub mod prop;

use crate::atmo::{self};
use crate::eom::{rk4_step, ExternalLoads, MassProps, State};
use crate::math::Vec3;
use aero::{AeroIn, WING_AREA};

// ── F-35A mass properties ────────────────────────────────────────────────────
//
//  Based on public F-35A data:
//    Empty weight:  29,300 lb  (publicised)
//    Pilot:            200 lb
//    Internal fuel: 18,498 lb  (full tanks)
//    50 % fuel load:  9,249 lb
//  → Operating weight ≈ 38,749 lb → mass ≈ 1,204 slug
//
//  Moments of inertia are estimated from published geometry using mass-moment
//  distribution consistent with known F-35A external dimensions.

/// Operating mass (slug).  38 750 lbf / 32.174 ft/s² ≈ 1 204 slug.
pub const MASS: f64 = 38_750.0 / crate::atmo::G0;

/// Roll inertia (slug·ft²).
pub const IXX: f64 = 34_000.0;
/// Pitch inertia (slug·ft²).
pub const IYY: f64 = 128_000.0;
/// Yaw inertia (slug·ft²).
pub const IZZ: f64 = 140_000.0;
/// Inertia cross-product (slug·ft²).
pub const IXZ: f64 = 1_800.0;

/// F-35A mass properties (pre-computed once).
pub const F35_MASS_PROPS: MassProps = MassProps::new(
    MASS, IXX, IYY, IZZ, IXZ,
);

// ── Control surface limits ────────────────────────────────────────────────────

/// Elevon pitch authority: ±25°.
const ELEV_MAX_DEG: f64 = 25.0;
/// Differential elevon (roll) authority: ±20°.
const AIL_MAX_DEG: f64 = 20.0;
/// Rudder authority: ±30°.
const RUD_MAX_DEG: f64 = 30.0;

// ── Controls ──────────────────────────────────────────────────────────────────

/// Normalised pilot controls for the F-35A.
#[derive(Debug, Clone, Copy)]
pub struct Controls {
    /// Throttle [0, 1].  0.83 = military power; 1.0 = max afterburner.
    pub throttle: f64,
    /// Longitudinal stick: +1 = full aft (nose-up), −1 = full forward.
    pub elevator: f64,
    /// Lateral stick: +1 = right roll, −1 = left roll.
    pub aileron: f64,
    /// Rudder pedal: +1 = right yaw, −1 = left yaw.
    pub rudder: f64,
}

impl Default for Controls {
    fn default() -> Self {
        Controls {
            throttle: 0.50,
            elevator: 0.0,
            aileron:  0.0,
            rudder:   0.0,
        }
    }
}

impl Controls {
    fn to_surfaces(&self) -> Surfaces {
        let de = (-ELEV_MAX_DEG * self.elevator.clamp(-1.0, 1.0)).to_radians();
        let da = ( AIL_MAX_DEG  * self.aileron .clamp(-1.0, 1.0)).to_radians();
        let dr = (-RUD_MAX_DEG  * self.rudder  .clamp(-1.0, 1.0)).to_radians();
        Surfaces { elevator_rad: de, aileron_rad: da, rudder_rad: dr }
    }
}

/// Control surface deflections in radians.
struct Surfaces {
    elevator_rad: f64,
    aileron_rad:  f64,
    rudder_rad:   f64,
}

// ── FlightModel ───────────────────────────────────────────────────────────────

/// Top-level F-35A flight model.
pub struct FlightModel {
    /// Current 12-DoF state.
    pub state: State,
    /// Tracked for α̇ finite-difference computation.
    prev_alpha: f64,
}

impl FlightModel {
    /// Create a new model initialised to approximate level-flight trim.
    ///
    /// * `altitude_ft`   – initial altitude above MSL (ft)
    /// * `airspeed_fps`  – initial true airspeed (ft/s)
    /// * `heading_rad`   – initial heading (rad, 0 = North)
    pub fn new(altitude_ft: f64, airspeed_fps: f64, heading_rad: f64) -> Self {
        let state = trim_level_flight(altitude_ft, airspeed_fps, heading_rad);
        FlightModel {
            prev_alpha: state.alpha,
            state,
        }
    }

    /// Advance the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64, controls: &Controls) -> &State {
        let surfs     = controls.to_surfaces();
        let throttle  = controls.throttle.clamp(0.0, 1.0);
        let prev_alpha = self.prev_alpha;

        let loads_fn = |s: &State| -> ExternalLoads {
            build_loads(s, &surfs, throttle, prev_alpha, dt)
        };

        let mut next = rk4_step(&self.state, dt, &F35_MASS_PROPS, loads_fn);

        // Update load-factor (nz) from aero output at the new state.
        let atmo    = atmo::atmosphere(next.altitude);
        let alpha_dot = (next.alpha - self.state.alpha) / dt.max(1e-6);
        let aero_in = make_aero_in(&next, &surfs, atmo.density, atmo.sound_speed,
                                   alpha_dot);
        let aero_out = aero::aerodynamics(&aero_in);
        next.nz = -aero_out.force_body.z / (MASS * atmo::G0);

        self.prev_alpha = next.alpha;
        self.state = next;
        &self.state
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

fn build_loads(
    s:          &State,
    surfs:      &Surfaces,
    throttle:   f64,
    prev_alpha: f64,
    dt:         f64,
) -> ExternalLoads {
    let atmo     = atmo::atmosphere(s.altitude);
    let alpha_dot = (s.alpha - prev_alpha) / dt.max(1e-6);

    let aero_in  = make_aero_in(s, surfs, atmo.density, atmo.sound_speed,
                                alpha_dot);
    let aero_out = aero::aerodynamics(&aero_in);

    let mach  = s.vt / atmo.sound_speed.max(1.0);
    let prop  = prop::propulsion(throttle, mach, atmo.density);

    // Thrust acts along body X (forward).
    let thrust_body = Vec3::new(prop.thrust_lbf, 0.0, 0.0);

    ExternalLoads {
        force:  aero_out.force_body + thrust_body,
        moment: aero_out.moment_body,
    }
}

fn make_aero_in(
    s:          &State,
    surfs:      &Surfaces,
    rho:        f64,
    sound_speed:f64,
    alpha_dot:  f64,
) -> AeroIn {
    AeroIn {
        vel_aero:     Vec3::new(s.u, s.v, s.w),
        pqr:          Vec3::new(s.p, s.q, s.r),
        alpha_dot,
        rho,
        sound_speed,
        elevator_rad: surfs.elevator_rad,
        aileron_rad:  surfs.aileron_rad,
        rudder_rad:   surfs.rudder_rad,
    }
}

// ── Level-flight trim ─────────────────────────────────────────────────────────

/// Approximate trim for straight-and-level flight.
/// Uses the CL table to find the angle of attack that balances lift against
/// the F-35A operating weight.
fn trim_level_flight(altitude_ft: f64, airspeed_fps: f64, heading_rad: f64)
    -> State
{
    use crate::math::interp1;
    use aero::CL_ALPHA_TABLE;

    let atmo  = atmo::atmosphere(altitude_ft);
    let vt    = airspeed_fps;
    let qbar  = 0.5 * atmo.density * vt * vt;

    let weight   = MASS * atmo::G0;          // lbf
    let cl_needed = weight / (qbar * WING_AREA);

    // Invert CL table: build (CL, alpha) pairs and interpolate.
    // The table is monotonic in CL for the range of interest.
    let cl_alpha_inv: Vec<(f64, f64)> = CL_ALPHA_TABLE
        .iter()
        .map(|&(a, cl)| (cl, a))
        .collect();

    let cl_clamped = cl_needed.clamp(
        CL_ALPHA_TABLE.first().unwrap().1,
        CL_ALPHA_TABLE.last() .unwrap().1,
    );
    let alpha = interp1(&cl_alpha_inv, cl_clamped).clamp(-0.26, 0.50);

    let theta = alpha; // level flight: climb angle = 0 → θ = α
    let u = vt * theta.cos();
    let w = vt * theta.sin();

    let mach = vt / atmo.sound_speed;

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
        beta:  0.0,
        vt,
        qbar,
        mach,
        stall_hyst: 0.0,
        nz: 1.0,
    }
}

// Remove stale re-export – CL_ALPHA_TABLE is used only in trim_level_flight above.

