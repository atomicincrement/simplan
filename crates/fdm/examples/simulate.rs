//! Simple simulation example for the C172p FDM.
//!
//! Run with:
//!   cargo run -p fdm --example simulate

use fdm::c172::{Controls, FlightModel};

fn main() {
    // Initial conditions: 3 000 ft MSL, 81 kt TAS – the natural trim speed
    // for this weight/altitude with zero elevator input (Cm_alpha balances Cm0).
    // State::level_flight() finds the angle of attack that balances lift=weight.
    const KT_TO_FPS: f64 = 1.6878;
    let mut fdm = FlightModel::new(3_000.0, 81.0 * KT_TO_FPS, 0.0);

    // Integration step size: 50 Hz matches JSBSim default.
    let dt = 1.0 / 50.0;

    // Print header
    println!(
        "{:>7}  {:>7}  {:>7}  {:>7}  {:>7}  {:>7}  {:>7}  {:>7}  {:>7}",
        "t[s]", "alt[ft]", "Vt[kt]", "alpha[°]", "theta[°]",
        "phi[°]", "p[°/s]", "q[°/s]", "nz[g]"
    );

    let total_time = 60.0_f64;   // simulate 60 seconds
    let print_every = 1.0_f64;   // print every 1 s of sim time
    let mut next_print = 0.0_f64;

    let mut t = 0.0_f64;
    while t < total_time {
        // --- Control schedule -------------------------------------------
        // At 81 kt / 3 000 ft with zero elevator the model trims naturally
        // (Cm0 = 0.10 is balanced by Cm_alpha at alpha ≈ 3.2°).
        // Throttle ≈ 0.38 balances drag at this condition.
        //
        // 0–20 s: hold trim
        // 20–25 s: pull-up (elevator back +0.25)
        // 25–40 s: sustain back-pressure at +0.15
        // 40–45 s: recover (stick forward −0.15)
        // 45–60 s: re-trim
        let base_throttle = 0.38_f64;
        let trim_elevator = 0.00_f64;

        let controls = if t < 20.0 {
            Controls { throttle: base_throttle, elevator: trim_elevator, ..Default::default() }
        } else if t < 25.0 {
            Controls { throttle: base_throttle, elevator: trim_elevator + 0.25, ..Default::default() }
        } else if t < 40.0 {
            Controls { throttle: base_throttle, elevator: trim_elevator + 0.15, ..Default::default() }
        } else if t < 45.0 {
            Controls { throttle: base_throttle, elevator: trim_elevator - 0.15, ..Default::default() }
        } else {
            Controls { throttle: base_throttle, elevator: trim_elevator, ..Default::default() }
        };

        fdm.step(dt, &controls);
        t += dt;

        // Print telemetry every second
        if t >= next_print {
            let s = &fdm.state;
            println!(
                "{:7.1}  {:7.0}  {:7.1}  {:7.2}  {:7.2}  {:7.2}  {:7.2}  {:7.2}  {:7.3}",
                s.t,
                s.altitude,
                s.vt / KT_TO_FPS,
                s.alpha.to_degrees(),
                s.theta.to_degrees(),
                s.phi.to_degrees(),
                s.p.to_degrees(),
                s.q.to_degrees(),
                s.nz,
            );
            next_print += print_every;
        }
    }

    // --- Roll manoeuvre: 30° banked turn (last 20 s) ---
    println!("\n--- banked turn (30° right) ---");
    println!(
        "{:>7}  {:>7}  {:>7}  {:>7}  {:>7}  {:>7}",
        "t[s]", "alt[ft]", "Vt[kt]", "phi[°]", "psi[°]", "nz[g]"
    );

    let bank_time = 20.0;
    let mut t2 = 0.0_f64;
    next_print = 0.0;

    while t2 < bank_time {
        let controls = if t2 < 4.0 {
            // Roll into bank
            Controls { throttle: 0.42, aileron: 0.35, elevator: 0.05, ..Default::default() }
        } else if t2 < 16.0 {
            // Hold bank, add back-pressure to maintain altitude
            Controls { throttle: 0.42, aileron: 0.0, elevator: 0.12, ..Default::default() }
        } else {
            // Roll out
            Controls { throttle: 0.42, aileron: -0.35, elevator: 0.0, ..Default::default() }
        };

        fdm.step(dt, &controls);
        t2 += dt;

        if t2 >= next_print {
            let s = &fdm.state;
            println!(
                "{:7.1}  {:7.0}  {:7.1}  {:7.2}  {:7.2}  {:7.3}",
                s.t,
                s.altitude,
                s.vt / KT_TO_FPS,
                s.phi.to_degrees(),
                s.psi.to_degrees(),
                s.nz,
            );
            next_print += 1.0;
        }
    }
}
