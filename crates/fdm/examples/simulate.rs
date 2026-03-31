//! F-35A simulation with initial conditions matching the Bevy flight sim.
//!
//! Run with:
//!   cargo run -p fdm --example simulate

use fdm::f35::{Controls, FlightModel};

/// Same starting state as the Bevy sim:
///   altitude = 500 m  → 1 640.42 ft
///   airspeed = 100 m/s → 328.08  ft/s
///   heading  = 0 (north / −Z in Bevy world)
///   throttle = 0.60, zero control inputs
fn main() {
    const ALT_FT: f64 = 500.0 * 3.280_84; // 1 640.42 ft
    const VT_FPS: f64 = 100.0 * 3.280_84; // 328.08  ft/s

    let mut fdm = FlightModel::new(ALT_FT, VT_FPS, 0.0);

    let controls = Controls {
        throttle: 0.60,
        elevator: 0.0,
        aileron: 0.0,
        rudder: 0.0,
    };

    // 60 Hz – same as Bevy's default fixed-update rate.
    let dt = 1.0 / 60.0;
    let total_time = 30.0_f64;

    println!(
        "{:>7}  {:>9}  {:>9}  {:>10}  {:>10}  (fdm)",
        "t[s]", "alt[ft]", "vt[fps]", "pos_n[ft]", "pos_e[ft]"
    );

    let mut next_print = 0.0_f64;
    let mut t = 0.0_f64;

    while t <= total_time {
        if t >= next_print {
            let s = &fdm.state;
            println!(
                "{:7.1}  {:9.1}  {:9.2}  {:10.1}  {:10.1}",
                s.t, s.altitude, s.vt, s.pos_n, s.pos_e,
            );
            next_print += 1.0;
        }
        fdm.step(dt, &controls);
        t += dt;
    }
}
