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
//! - [`f35`]   – F-35A Lightning II (estimated coefficients)

// ── Shared modules ────────────────────────────────────────────────────────────
pub mod atmo;
pub mod eom;
pub mod math;

// ── Aircraft modules ──────────────────────────────────────────────────────────
pub mod c172;
pub mod f35;
