// Sphere geometry helpers.
//
// Convention
// ----------
// The Earth is modelled as a sphere of radius R metres.
// Its centre sits at world position (0, -R, 0) so that world origin (0, 0, 0)
// lies on the surface and the surface normal there points straight up (+Y).
//
// A "flat" coordinate (fx, fz) is the horizontal offset from the origin in
// the tangent plane at y = 0.  Given (fx, fz) we can project onto the sphere
// to get the true world-space position and outward surface normal.

pub const EARTH_RADIUS: f32 = 6_371_000.0; // metres

/// Result of projecting a flat tangent-plane coordinate onto the sphere.
#[derive(Clone, Copy, Debug)]
pub struct SpherePoint {
    /// World-space position on the sphere surface (before terrain height is added).
    pub position: [f32; 3],
    /// Outward unit normal at this surface point.
    pub normal: [f32; 3],
}

/// Project the flat tangent-plane coordinate `(fx, fz)` onto the unit sphere
/// centred at `(0, -R, 0)`.
///
/// The direction from the sphere centre to the "flat" point `(fx, 0, fz)` is:
///   d = (fx, R, fz) / ‖(fx, R, fz)‖
///
/// The sphere surface position is:
///   P = (0, -R, 0) + R · d = (R·fx/D, R·R/D − R, R·fz/D)
///   where D = √(fx² + R² + fz²)
pub fn project(fx: f32, fz: f32) -> SpherePoint {
    let r = EARTH_RADIUS;
    let d = (fx * fx + r * r + fz * fz).sqrt();
    let inv_d = 1.0 / d;

    let nx = fx * inv_d;
    let ny = r  * inv_d;
    let nz = fz * inv_d;

    SpherePoint {
        position: [r * nx, r * ny - r, r * nz],
        normal:   [nx, ny, nz],
    }
}
