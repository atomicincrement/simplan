//! Flight Control System for the C172p
//!
//! Translates pilot commands (normalised −1 … +1) into control surface
//! deflections in radians, matching the c172p.xml aerosurface_scale elements.
//!
//! Sign conventions (our Rust model):
//!   elevator_cmd = +1   aft stick  → nose-up elevator deflection
//!   aileron_cmd  = +1   right roll → positive aileron (right wing down)
//!   rudder_cmd   = +1   right yaw  → positive rudder deflection
//!   flap_cmd     = 0…1  flap position (0 = retracted, 1 = full = 30°)

/// Control surface deflections in radians (and flap angle in degrees).
#[derive(Debug, Clone, Copy, Default)]
pub struct Surfaces {
    /// Elevator deflection (rad).  Positive → trailing edge up → nose-up moment.
    pub elevator_rad: f64,
    /// Left aileron deflection (rad).  Positive → roll moment to the right.
    pub aileron_rad: f64,
    /// Rudder deflection (rad).  Positive → yaw moment to the right.
    pub rudder_rad: f64,
    /// Flap in degrees (0, 10, 20, or 30).
    pub flap_deg: f64,
}

/// Pilot control inputs, all normalised to [−1, +1] (or [0, 1] for flap/throttle).
#[derive(Debug, Clone, Copy)]
pub struct Controls {
    /// Longitudinal: +1 = full aft (nose up), −1 = full forward (nose down).
    pub elevator: f64,
    /// Lateral: +1 = right roll, −1 = left roll.
    pub aileron: f64,
    /// Directional: +1 = right yaw, −1 = left yaw.
    pub rudder: f64,
    /// Throttle [0, 1].
    pub throttle: f64,
    /// Flap position [0, 1] where 1 = 30° (full flap).
    pub flap: f64,
}

impl Default for Controls {
    fn default() -> Self {
        Controls {
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
            throttle: 0.5,
            flap: 0.0,
        }
    }
}

impl Controls {
    pub fn to_surfaces(&self) -> Surfaces {
        // ── Elevator ──────────────────────────────────────────────────────────
        // c172p.xml aerosurface_scale:  range min=−28°, max=+23°; gain=0.01745 rad/deg.
        // We define elevator_cmd = +1 as nose-up (aft stick).
        // JSBSim stores elevator_cmd = +1 as forward (nose-down), so we negate here
        // so that our public sign convention is intuitive, then account for the
        // asymmetric limits:
        //   elevator_cmd_jsbsim = −our_cmd
        //   deflection range: [−28°, +23°] mapped from [−1, +1] cmd
        //     at our +1 (jsbsim −1) → −28° * 0.01745 = −0.4886 rad (TE down → nose up)
        //     at our −1 (jsbsim +1) → +23° * 0.01745 = +0.4013 rad (TE up  → nose down)
        // To keep arithmetic simple we use the average travel ~25° and define
        // positive elevator_rad as (in the aero code) trailing-edge DOWN = nose-up.
        let elev_cmd = self.elevator.clamp(-1.0, 1.0);
        // Our sign: elevator_cmd = +1 means NOSE UP (aft stick).
        // JSBSim sign: +1 = forward / nose-down, so we negate.
        // We use the average travel of the asymmetric limits (≈25°) as the
        // symmetric range; the asymmetry just sets hard stops that the sim
        // would never exceed in normal flight.
        let elevator_rad = (-25.0 * elev_cmd).to_radians();

        // ── Aileron ───────────────────────────────────────────────────────────
        // Left aileron range min=−20°, max=+15°; gain=0.01745.
        // Positive aileron_cmd = right roll = positive roll moment.
        // Left aileron positive deflection (TE down) increases roll moment rightward.
        let ail_cmd = self.aileron.clamp(-1.0, 1.0);
        // Symmetric ±17° (average of +15° / −20° limits).
        // Positive aileron_cmd = right roll; ClDa > 0 = roll right ✓.
        let aileron_rad = (17.0 * ail_cmd).to_radians();

        // ── Rudder ────────────────────────────────────────────────────────────
        // Symmetric ±16°; gain=0.01745.
        // Our sign: rudder_cmd = +1 means NOSE RIGHT.
        // JSBSim Cndr = -0.043, so positive jsbsim_rudder_rad = nose LEFT.
        // Therefore: jsbsim_rudder_rad = −our_cmd × 16° × deg2rad.
        let rud_cmd = self.rudder.clamp(-1.0, 1.0);
        let rudder_rad = (-16.0 * rud_cmd).to_radians();

        // ── Flaps ─────────────────────────────────────────────────────────────
        // Discrete positions 0°, 10°, 20°, 30°, but for aerodynamic purposes
        // we use a continuous value; the kinematic element in the XML just
        // slows the transition rate. We go straight to the target angle.
        let flap_deg = (self.flap.clamp(0.0, 1.0) * 30.0).round(); // snap to nearest 10°
        let flap_deg = (flap_deg / 10.0).round() * 10.0;

        Surfaces {
            elevator_rad,
            aileron_rad,
            rudder_rad,
            flap_deg,
        }
    }
}
