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
