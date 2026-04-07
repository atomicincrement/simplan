//! F-16A simulation – outputs CSV for comparison with JSBSim reference.
//!
//! Initial conditions match `scripts/jsbsim_f16_test.py`:
//!   altitude = 500 m  → 1 640.42 ft
//!   airspeed = 100 m/s → 328.08  ft/s
//!   heading  = 0 (north), throttle = 0.60, zero control surface inputs
//!
//! Run with:
//!   cargo run -p fdm --example simulate
//!
//! Redirect to CSV:
//!   cargo run -p fdm --example simulate > scripts/output/our_f16.csv

use fdm::f16::{Controls, FlightModel};

fn main() {
    const ALT_FT: f64 = 500.0 * 3.280_84;   // 1 640.42 ft
    const VT_FPS: f64 = 100.0 * 3.280_84;   // 328.08  ft/s

    let mut fdm = FlightModel::new(ALT_FT, VT_FPS, 0.0);

    let controls = Controls {
        throttle: 0.60,
        elevator: 0.0,
        aileron:  0.0,
        rudder:   0.0,
    };

    // 60 Hz – same as Bevy's default fixed-update rate, matches JSBSim test dt.
    let dt         = 1.0 / 60.0;
    let total_time = 30.0_f64;

    // CSV header – columns match jsbsim_f16_test.py output.
    println!("t_s,alt_ft,vt_fps,alpha_deg,mach,pos_n_ft,pos_e_ft");

    let mut next_print = 0.0_f64;
    let mut t          = 0.0_f64;

    while t <= total_time + dt / 2.0 {
        if t >= next_print - dt / 2.0 {
            let s = &fdm.state;
            println!(
                "{:.1},{:.2},{:.3},{:.4},{:.4},{:.1},{:.1}",
                s.t,
                s.altitude,
                s.vt,
                s.alpha.to_degrees(),
                s.mach,
                s.pos_n,
                s.pos_e,
            );
            next_print += 1.0;
        }
        fdm.step(dt, &controls);
        t += dt;
    }
}
