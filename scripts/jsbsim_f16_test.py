#!/usr/bin/env python3
"""
JSBSim F-16A reference test.

Runs the JSBSim F-16 model (from vendor/jsbsim) at the same initial conditions
as `crates/fdm/examples/simulate.rs`:
    altitude = 500 m  (1 640.42 ft)
    airspeed = 100 m/s (328.08 ft/s)
    level flight, heading North, throttle = 0.60

JSBSim's built-in longitudinal trim (do_trim mode 1) finds the equilibrium
elevator deflection and angle of attack; the trim alpha is printed to stderr
so it can be compared with our FDM's trim result.  After trim the simulation
runs open-loop (controls held fixed at the trimmed values) for 30 seconds at
60 Hz.

Outputs CSV to stdout (or scripts/output/jsbsim_f16.csv when redirected).

Usage:
    python3 scripts/jsbsim_f16_test.py
    python3 scripts/jsbsim_f16_test.py > scripts/output/jsbsim_f16.csv

Note: JSBSim's F-16 includes a full FCS (pitch/roll/yaw SAS).  This means
elevator is not zero even in trimmed level flight, and the closed-loop dynamics
will differ from our open-loop aerodynamic model.  The primary quantity to
compare across the two models is therefore the trim angle-of-attack.
"""

import math
import os
import sys
import jsbsim

# ── paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT    = os.path.dirname(SCRIPT_DIR)
JSBSIM_ROOT  = os.path.join(REPO_ROOT, "vendor", "jsbsim")

# ── initial conditions (must match simulate.rs) ───────────────────────────────
ALT_FT   = 500.0 * 3.28084    # 1 640.42 ft
VT_FPS   = 100.0 * 3.28084    # 328.08  ft/s
THROTTLE = 0.60
DT       = 1.0 / 60.0         # 60 Hz – same as Bevy / simulate.rs
DURATION = 30.0                # seconds

# ── set up JSBSim ─────────────────────────────────────────────────────────────
fdm = jsbsim.FGFDMExec(JSBSIM_ROOT, None)
fdm.set_debug_level(0)   # suppress console chatter

fdm.load_model("f16")
fdm.set_dt(DT)

# Set inflight initial conditions.
fdm["ic/h-sl-ft"]      = ALT_FT
fdm["ic/vt-fps"]       = VT_FPS
fdm["ic/gamma-deg"]    = 0.0    # level flight (flight-path angle = 0)
fdm["ic/psi-true-deg"] = 0.0    # heading North
fdm["ic/phi-deg"]      = 0.0    # wings level

fdm.run_ic()

# Bring the engine up to a running state and apply throttle.
fdm["propulsion/engine[0]/set-running"] = 1
fdm["fcs/throttle-cmd-norm[0]"]         = THROTTLE

# ── longitudinal trim ─────────────────────────────────────────────────────────
# do_trim(1) = tLongitudinal: finds elevator / alpha for steady level flight
# at the given altitude, airspeed and throttle.
try:
    fdm.do_trim(1)
    trim_alpha = fdm["aero/alpha-deg"]
    trim_elev  = fdm["fcs/elevator-pos-deg"]
    print(
        f"# JSBSim trim: alpha={trim_alpha:.3f} deg  "
        f"elevator={trim_elev:.3f} deg  "
        f"(throttle={THROTTLE:.2f})",
        file=sys.stderr,
    )
except Exception as exc:
    print(f"# Trim failed ({exc}); running untrimmed from IC", file=sys.stderr)

# Freeze controls at the trimmed values for the open-loop run.
# (JSBSim's FCS remains active; any residual SAS activity will be visible
#  in the trajectory and should be noted in the comparison.)

# Capture origin lat/lon for a flat-earth north/east position readout.
lat0_deg = fdm["position/lat-geod-deg"]
lon0_deg = fdm["position/long-gc-deg"]
DEG_TO_FT_LAT = 364_000.0   # 1° lat ≈ 364 000 ft  (= 60 nmi)


def north_east_ft(fdm_exec, lat0: float, lon0: float) -> tuple[float, float]:
    lat   = fdm_exec["position/lat-geod-deg"]
    lon   = fdm_exec["position/long-gc-deg"]
    north = (lat - lat0) * DEG_TO_FT_LAT
    east  = (lon - lon0) * DEG_TO_FT_LAT * abs(math.cos(math.radians(lat0)))
    return north, east


# ── CSV output ────────────────────────────────────────────────────────────────
print("t_s,alt_ft,vt_fps,alpha_deg,mach,pos_n_ft,pos_e_ft")

next_print = 0.0

while fdm.get_sim_time() <= DURATION + DT / 2.0:
    t = fdm.get_sim_time()

    if t >= next_print - DT / 2.0:
        alt   = fdm["position/h-sl-ft"]
        vt    = fdm["velocities/vt-fps"]
        alpha = fdm["aero/alpha-deg"]
        mach  = fdm["velocities/mach"]
        north, east = north_east_ft(fdm, lat0_deg, lon0_deg)

        print(f"{t:.1f},{alt:.2f},{vt:.3f},{alpha:.4f},{mach:.4f},{north:.1f},{east:.1f}")
        next_print += 1.0

    fdm.run()
