// Tile mesh builder.
//
// Produces a Bevy `Mesh` for a terrain tile centred at flat coordinates
// (cx, cz) with half-size `half`.  The tile is subdivided into GRID×GRID quads
// (GRID+1 vertices per side).
//
// Each vertex is:
//   1. Projected from flat (fx, fz) onto the sphere surface.
//   2. Displaced along the surface normal by a multi-octave Perlin height.
//   3. Coloured by world-space Y altitude (water → pasture → rock → snow).
//
// Smooth per-vertex normals are computed from finite differences of the height
// field using a one-step stencil.

use bevy::{
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};

use crate::{
    noise::fbm3,
    sphere::project,
    TerrainConfig,
};

/// Number of quads along each tile edge (65×65 vertex grid = 64×64 quads).
pub const GRID: usize = 64;
const VERTS: usize = GRID + 1;   // vertices per side = 65

// ── Altitude colour ramp ─────────────────────────────────────────────────────
//
// Input `y` is world-space height in metres.  `height_scale` is the FBM
// amplitude in metres so the palette is self-consistent regardless of scale.
//
// Stops (as fraction of height_scale):
//   < -0.05  deep ocean blue
//   -0.05..0 shallow / nearshore
//   0..0.10  sandy beach
//   0.10..0.5 green pasture
//   0.5..0.80 grey rock
//   0.80..1.0 snow / glaciers

fn lerp_col(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    [a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t, 1.0]
}

fn altitude_color(y: f32, hs: f32) -> [f32; 4] {
    let t = y / hs; // normalised 0 → 1 (heights below sea level are clamped to 0)
    const OCEAN:   [f32; 4] = [0.04, 0.18, 0.48, 1.0];
    const SHALLOW: [f32; 4] = [0.10, 0.38, 0.65, 1.0];
    const BEACH:   [f32; 4] = [0.82, 0.76, 0.56, 1.0];
    const PASTURE: [f32; 4] = [0.22, 0.52, 0.18, 1.0];
    const ROCK:    [f32; 4] = [0.46, 0.43, 0.38, 1.0];
    const SNOW:    [f32; 4] = [0.95, 0.95, 0.98, 1.0];
    if t < 0.005 {
        // Flat sea – ocean blue.
        lerp_col(OCEAN, SHALLOW, t / 0.005)
    } else if t < 0.02 {
        // Shallow water → sandy beach.
        lerp_col(SHALLOW, BEACH, (t - 0.005) / 0.015)
    } else if t < 0.05 {
        // Beach → pasture.
        lerp_col(BEACH, PASTURE, (t - 0.02) / 0.03)
    } else if t < 0.50 {
        // Pasture → rock.
        lerp_col(PASTURE, ROCK, (t - 0.05) / 0.45)
    } else if t < 0.80 {
        ROCK
    } else {
        // Rock → snow.
        lerp_col(ROCK, SNOW, (t - 0.80) / 0.20)
    }
}

/// Build a terrain tile mesh.
///
/// * `cx`, `cz` – flat tangent-plane centre of the tile (metres from origin)
/// * `half`     – half-size of the tile in metres (tile spans [cx±half, cz±half])
pub fn build_tile_mesh(cx: f32, cz: f32, half: f32, cfg: &TerrainConfig) -> Mesh {
    let step = (2.0 * half) / GRID as f32;
    // Vertex spacing used for finite-difference normal estimation.
    let nd = step * 0.5;

    let total_verts = VERTS * VERTS;
    let mut positions = Vec::with_capacity(total_verts);
    let mut normals   = Vec::with_capacity(total_verts);
    let mut colors    = Vec::with_capacity(total_verts);
    let mut uvs       = Vec::with_capacity(total_verts);

    // Helper: sample height at arbitrary flat (fx, fz).
    // Clamped to >= 0 so that below-sea-level noise produces flat water.
    let height = |fx: f32, fz: f32| -> f32 {
        let sp = project(fx, fz);
        let n = sp.normal;
        (fbm3(
            n[0] * cfg.noise_frequency,
            n[1] * cfg.noise_frequency,
            n[2] * cfg.noise_frequency,
            1.0,
            cfg.noise_octaves,
            2.0,
            0.5,
        ) * cfg.height_scale).max(0.0)
    };

    // Helper: world position of vertex at flat (fx, fz).
    let world_pos = |fx: f32, fz: f32| -> [f32; 3] {
        let sp = project(fx, fz);
        let h  = height(fx, fz);
        [
            sp.position[0] + sp.normal[0] * h,
            sp.position[1] + sp.normal[1] * h,
            sp.position[2] + sp.normal[2] * h,
        ]
    };

    for j in 0..VERTS {
        for i in 0..VERTS {
            let fx = cx - half + i as f32 * step;
            let fz = cz - half + j as f32 * step;

            // Position with height displacement.
            let pos = world_pos(fx, fz);
            let world_y = pos[1];
            positions.push(pos);

            // Altitude-based vertex colour.
            colors.push(altitude_color(world_y, cfg.height_scale));

            // UV: simple 0…1 across the tile.
            uvs.push([i as f32 / GRID as f32, j as f32 / GRID as f32]);

            // Surface normal via central-difference of the height field.
            let px = world_pos(fx + nd, fz);
            let mx = world_pos(fx - nd, fz);
            let pz = world_pos(fx, fz + nd);
            let mz = world_pos(fx, fz - nd);

            let tx = Vec3::new(px[0] - mx[0], px[1] - mx[1], px[2] - mx[2]);
            let tz = Vec3::new(pz[0] - mz[0], pz[1] - mz[1], pz[2] - mz[2]);

            // Normal = tz × tx (right-hand, pointing away from sphere).
            let n = tz.cross(tx).normalize();
            normals.push([n.x, n.y, n.z]);
        }
    }

    // Index buffer: two triangles per quad, wound counter-clockwise.
    let quad_count = GRID * GRID;
    let mut indices = Vec::with_capacity(quad_count * 6);
    for j in 0..GRID {
        for i in 0..GRID {
            let tl = (j * VERTS + i) as u32;
            let tr = tl + 1;
            let bl = tl + VERTS as u32;
            let br = bl + 1;
            // Upper-left triangle.
            indices.extend_from_slice(&[tl, bl, tr]);
            // Lower-right triangle.
            indices.extend_from_slice(&[tr, bl, br]);
        }
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL,   normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR,    colors);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0,     uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

/// Compute terrain height at a single flat (fx, fz) coordinate.
/// Used externally to query ground elevation for physics / HUD.
pub fn terrain_height_at(fx: f32, fz: f32, cfg: &TerrainConfig) -> f32 {
    let sp = project(fx, fz);
    let n = sp.normal;
    let noise = fbm3(
        n[0] * cfg.noise_frequency,
        n[1] * cfg.noise_frequency,
        n[2] * cfg.noise_frequency,
        1.0,
        cfg.noise_octaves,
        2.0,
        0.5,
    );
    // Sphere surface y + noise displacement (clamped to sea level).
    sp.position[1] + sp.normal[1] * (noise * cfg.height_scale).max(0.0)
}


