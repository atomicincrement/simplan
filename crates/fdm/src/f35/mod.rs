//! F-16A Lightning II – Flight Dynamics Model
//!
//! A 6-DoF rigid-body flight model for the F-16A.
//! Aerodynamic data come from the JSBSim F-16 model
//! (`aircraft/f16/f16.xml`, Nguyen et al. NASA TM-1979 wind-tunnel data).
//! Engine data come from `engine/F100-PW-229.xml` (F100-PW-229 turbofan).
//! Mass and inertia are from the JSBSim `<mass_balance>` block.
//!
//! # Quick start
//!
//! ```rust
//! use fdm::f35::{FlightModel, Controls};
//!
//! // 10 000 ft, Mach 0.5 (≈ 556 ft/s), heading East
//! let mut fdm = FlightModel::new(10_000.0, 556.0, std::f64::consts::FRAC_PI_2);
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

// ── F-16A mass properties ────────────────────────────────────────────────────
//
//  From JSBSim `aircraft/f16/f16.xml` <mass_balance> block
//  (negated_crossproduct_inertia = true, so stored ixz = −982 → actual Ixz = +982):
//    Empty weight:   17,400 lb
//    Pilot:             230 lb
//    Internal fuel:   3,000 lb  (2 × 1,500 lb tanks, ~50 % load)
//  → Operating weight = 20,630 lb → mass ≈ 641 slug

/// Operating mass (slug).  20 630 lbf / 32.174 ft/s² ≈ 641 slug.
pub const MASS: f64 = 20_630.0 / crate::atmo::G0;

/// Roll inertia (slug·ft²).  Source: F-16 JSBSim XML.
pub const IXX: f64 = 9_496.0;
/// Pitch inertia (slug·ft²).  Source: F-16 JSBSim XML.
pub const IYY: f64 = 55_814.0;
/// Yaw inertia (slug·ft²).  Source: F-16 JSBSim XML.
pub const IZZ: f64 = 63_100.0;
/// Inertia cross-product Ixz (slug·ft²).  Stored as −982 with negated flag → +982.
pub const IXZ: f64 = 982.0;

/// F-16A mass properties (pre-computed once).
pub const F35_MASS_PROPS: MassProps = MassProps::new(
    MASS, IXX, IYY, IZZ, IXZ,
);

// ── Control surface limits ────────────────────────────────────────────────────

/// Horizontal tail (elevator) authority: ±25°  (±0.436 rad — JSBSim table range).
const ELEV_MAX_DEG: f64 = 25.0;
/// Aileron authority: ±21.5° (F-16 FCS limit).
const AIL_MAX_DEG:  f64 = 21.5;
/// Rudder authority: ±30°.
const RUD_MAX_DEG:  f64 = 30.0;

// ── Controls ──────────────────────────────────────────────────────────────────

/// Normalised pilot controls for the F-16A.
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

/// Top-level F-16A flight model (aerodynamic data from JSBSim F-16 model).
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
/// Uses the de=0 CL table to find the angle of attack that balances lift
/// against the F-16A operating weight.
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
    let alpha = interp1(&cl_alpha_inv, cl_clamped).clamp(-0.175, 0.524);

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

