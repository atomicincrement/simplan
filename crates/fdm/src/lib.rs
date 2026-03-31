//! C172p Flight Dynamics Model
//!
//! A 6-DoF flight model for the Cessna 172p translated from JSBSim.
//!
//! # Quick start
//!
//! ```rust
//! use c172_fdm::{FlightModel, Controls};
//!
//! let mut fdm = FlightModel::new(4_000.0, 169.0, 0.0); // 4 000 ft, 100 kt, heading North
//! let controls = Controls {
//!     throttle: 0.65,
//!     elevator: 0.0,
//!     aileron:  0.0,
//!     rudder:   0.0,
//!     flap:     0.0,
//! };
//!
//! for _ in 0..600 {          // 10 s at dt = 1/60 s
//!     fdm.step(1.0 / 60.0, &controls);
//! }
//! println!("alt = {:.0} ft, vt = {:.1} kt",
//!          fdm.state.altitude, fdm.state.vt / 1.6878);
//! ```

pub mod aero;
pub mod atmo;
pub mod eom;
pub mod fcs;
pub mod math;
pub mod prop;
pub mod f35;

pub use eom::State;
pub use fcs::Controls;

use aero::{AeroIn, WINGSPAN};
use atmo::atmosphere;
use eom::{rk4_step, C172_MASS_PROPS, ExternalLoads};
use fcs::Surfaces;
use math::Vec3;
use prop::propulsion;

/// The top-level flight model integrating all subsystems.
pub struct FlightModel {
    pub state: State,
    /// Cached alpha-dot for aerodynamic damping terms.
    prev_alpha: f64,
}

impl FlightModel {
    /// Create a new model trimmed for level flight.
    ///
    /// * `altitude_ft`   – initial altitude above MSL (ft)
    /// * `airspeed_fps`  – initial true airspeed (ft/s); 100 kt ≈ 169 ft/s
    /// * `heading_rad`   – initial heading (rad, 0 = North)
    pub fn new(altitude_ft: f64, airspeed_fps: f64, heading_rad: f64) -> Self {
        let state = State::level_flight(altitude_ft, airspeed_fps, heading_rad);
        FlightModel {
            prev_alpha: state.alpha,
            state,
        }
    }

    /// Advance the simulation by `dt` seconds.
    ///
    /// Returns a shared reference to the updated state.
    pub fn step(&mut self, dt: f64, controls: &Controls) -> &State {
        // Snapshot controls so the RK4 closure can capture them.
        let surfs = controls.to_surfaces();
        let throttle = controls.throttle.clamp(0.0, 1.0);
        let prev_alpha = self.prev_alpha;

        // build closure that evaluates forces+moments for any provisional state
        let loads_fn = |s: &State| -> ExternalLoads {
            loads(s, &surfs, throttle, prev_alpha, dt)
        };

        let next = rk4_step(&self.state, dt, &C172_MASS_PROPS, loads_fn);

        // Update alpha-dot tracker
        let alpha_dot = if dt > 0.0 {
            (next.alpha - self.state.alpha) / dt
        } else {
            0.0
        };
        self.prev_alpha = next.alpha;

        // Update nz using the aero output at the new state
        let mut st = next;
        let atmo = atmosphere(st.altitude);
        let aero_in = build_aero_in(&st, &surfs, atmo.density, atmo.sound_speed, alpha_dot, 0.0);
        let aero_out = aero::aerodynamics(&aero_in);
        // nz ≈ −Fz_body / (m·g)  (convention: + 1g in straight-and-level)
        st.nz = -aero_out.force_body.z / (eom::MASS * atmo::G0);

        self.state = st;
        &self.state
    }
}

// ── Internal force/moment assembly ───────────────────────────────────────────

fn loads(
    s: &State,
    surfs: &Surfaces,
    throttle: f64,
    prev_alpha: f64,
    dt: f64,
) -> ExternalLoads {
    let atmo = atmosphere(s.altitude);

    // Alpha-dot estimate
    let alpha_dot = (s.alpha - prev_alpha) / dt.max(1e-6);

    // Height above ground / wing MAC for ground effect
    // Here we assume flat terrain at 0 ft MSL.
    let hoverbmac = s.altitude / WINGSPAN;

    // Propulsion
    let prop = propulsion(throttle, s.u, atmo.density);

    // Aerodynamics
    let aero_in = build_aero_in(
        s,
        surfs,
        atmo.density,
        atmo.sound_speed,
        alpha_dot,
        prop.v_induced,
    );
    let aero_in = AeroIn { hoverbmac, ..aero_in };

    let aero_out = aero::aerodynamics(&aero_in);

    // Thrust acts along body X axis (forward).
    let thrust_body = Vec3::new(prop.thrust_lbf, 0.0, 0.0);

    ExternalLoads {
        force:  aero_out.force_body + thrust_body,
        moment: aero_out.moment_body,
    }
}

fn build_aero_in(
    s: &State,
    surfs: &Surfaces,
    rho: f64,
    sound_speed: f64,
    alpha_dot: f64,
    v_induced: f64,
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
        flap_deg:     surfs.flap_deg,
        v_induced,
        hoverbmac:    1.5, // will be overridden in loads()
        stall_hyst:   s.stall_hyst,
    }
}
