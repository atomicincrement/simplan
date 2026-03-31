# Simplan – Conversation Log

## 2026-03-31

**User:** Using bevy create a planar world with a rectangle as a runway. Build a basic f35-like aircraft model or fetch one from an open source site. We will leave flight dynamics and control to later.

**Action:** 
- Added `bevy = "0.15"` to `Cargo.toml` with optimised dev profiles.
- Rewrote `src/main.rs` to:
  - Spawn a 800×800 m ground plane (green grass material).
  - Spawn a 30×200 m asphalt runway centred at the origin, with white centre-line dashes and threshold markings.
  - Build an F-35–like aircraft from Bevy primitive shapes (Cuboid + Cylinder) with:
    - Fuselage, nose section, nose tip, avionics spine
    - DSI ventral intake
    - Blended wing-body (LERX)
    - Large delta wings with swept leading-edge extensions and elevon panels
    - All-moving horizontal stabilisers
    - Single slightly-canted vertical tail
    - Bubble canopy (translucent)
    - Engine nozzle (two-stage cylinder)
    - Three-point landing gear stubs
  - Camera positioned to view the aircraft over the runway.
  - Directional sunlight + ambient fill.
---

**User:** Use the f35 model to drive the avian physics model. Use the avian position and velocity as inputs to aerodynamics(), convert the output to a force and torque to apply to the model.

**Action:**
- Added `fdm` crate imports to `crates/sim/src/main.rs`: `atmosphere`, `aerodynamics`/`AeroIn`, `propulsion`, `fdm::math::Vec3 as FdmVec3`.
- Added unit-conversion constants: `M_TO_FT = 3.28084`, `LBF_TO_N = 4.44822`, `LBFFT_TO_NM = 1.35582`.
- Added `PilotControls` resource (throttle=0.6, controls=0) and registered it with `.init_resource::<PilotControls>()`.
- Added `Mass(17_576.0)`, `ExternalForce::default()`, `ExternalTorque::default()` to the aircraft `spawn` bundle (Mass is F-35A combat weight: 38 750 lbf / 32.174 ft/s² × 14.594 kg/slug ≈ 17 576 kg).
- Implemented `apply_aerodynamics` `Update` system:
  - Reads `Transform`, `LinearVelocity`, `AngularVelocity` from the `Aircraft` entity.
  - Transforms world-frame velocity/angular-rate into aircraft local frame via `rotation.inverse() * vel`.
  - Maps local Bevy frame (nose = −Z) to FDM body frame (X fwd, Y right, Z down): u=−vz, v=vx, w=−vy; p=−ωz, q=ωx, r=−ωy; converts from m/s to ft/s.
  - Calls `atmosphere(altitude_ft)` for ISA density/sound-speed; clamps altitude to ≥ 0.
  - Calls `aerodynamics(&AeroIn{…})` for aero forces and moments; `propulsion(throttle, mach, rho)` for F135 thrust.
  - Adds thrust (lbf) to body-X force component.
  - Converts body-frame force to Bevy local frame: `(body_y, −body_z, −body_x) * LBF_TO_N`.
  - Converts body-frame moments to Bevy local frame: `(M, −N, −L) * LBFFT_TO_NM`.
  - Rotates both into world frame and writes to `ExternalForce`/`ExternalTorque` via `DerefMut`.
- Build clean; pushed to `atomicincrement/simplan` master.
---

**User:** Centre the stick / ignore mouse input for a clean FDM comparison.
**Action:**
- Simplified `mouse_controls` to always set `controls.elevator = 0.0` and `controls.aileron = 0.0`.
- Removed `window_q: Query<&Window>` parameter and all dead constants (`MAX_DEF`, `BOX_W/H`, `BOX_RIGHT/BOTTOM`).
- Build: clean.

---

**User:** (Implicit – physics correctness review before comparison run)
**Analysis:**
- Identified two bugs in physics integration:
  1. `apply_aerodynamics` ran in Bevy's `Update` schedule (variable-rate) instead of Avian's `PhysicsSchedule` (fixed-rate). Forces were computed from stale state and could miss physics substeps.
  2. `ExternalForce`/`ExternalTorque` default to `persistent: true`, meaning the last Update-computed force persisted across multiple physics steps without being refreshed.
**Action:**
- Moved `apply_aerodynamics` from `Update` to `PhysicsSchedule` with `.in_set(PhysicsStepSet::First)` — runs once at the start of each fixed physics step, before BroadPhase/Solver.
- Changed spawn to `ExternalForce::default().with_persistence(false)` and `ExternalTorque::default().with_persistence(false)` — forces are auto-cleared after each physics step so they can't accumulate stale values.
- Build: clean. Committed and pushed: `bbcbf55`.
