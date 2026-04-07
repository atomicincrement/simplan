//! `fdm` – Flight Dynamics Models
//!
//! # Modules
//!
//! Shared (aircraft-agnostic):
//! - [`atmo`]  – ISA atmosphere model
//! - [`eom`]   – 6-DoF rigid-body equations of motion (RK4 integrator)
//! - [`math`]  – Vec3, Mat3, interpolation utilities
//!
//! Per-aircraft:
//! - [`c172`]  – Cessna 172p (JSBSim port)
//! - [`f16`]   – F-16A Fighting Falcon (JSBSim data, Nguyen NASA TM-1979)

// ── Shared modules ────────────────────────────────────────────────────────────
pub mod atmo;
pub mod eom;
pub mod math;

// ── Aircraft modules ──────────────────────────────────────────────────────────
pub mod c172;
pub mod f16;
