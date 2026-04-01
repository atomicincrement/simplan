// Tile mesh builder.
//
// DEBUG MODE: each tile is a single flat axis-aligned quad (4 verts, 2 triangles)
// at y = 0.  Elevation and sphere-projection are disabled so that satellite
// imagery mapping can be inspected in isolation.
//
// UV (0,0) = top-left corner (north-west), UV (1,1) = bottom-right (south-east),
// matching the row-major ESRI/slippy-map tile convention.

use bevy::{
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};

use crate::TerrainConfig;

/// Build a flat single-quad terrain tile mesh at y = 0.
///
/// * `cx`, `cz` – flat tangent-plane centre of the tile (metres from origin)
/// * `half`     – half-size of the tile in metres (tile spans [cx±half, cz±half])
pub fn build_tile_mesh(cx: f32, cz: f32, half: f32, _cfg: &TerrainConfig) -> Mesh {
    // Four corners: TL, TR, BL, BR  (x = east, z = south in Bevy)
    //   UV row 0 (v=0) = north edge (smaller z)
    //   UV row 1 (v=1) = south edge (larger  z)
    let positions: Vec<[f32; 3]> = vec![
        [cx - half, 0.0, cz - half], // 0 TL  uv(0,0)
        [cx + half, 0.0, cz - half], // 1 TR  uv(1,0)
        [cx - half, 0.0, cz + half], // 2 BL  uv(0,1)
        [cx + half, 0.0, cz + half], // 3 BR  uv(1,1)
    ];
    let normals: Vec<[f32; 3]> = vec![[0.0, 1.0, 0.0]; 4];
    let uvs: Vec<[f32; 2]> = vec![
        [0.0, 0.0], // TL
        [1.0, 0.0], // TR
        [0.0, 1.0], // BL
        [1.0, 1.0], // BR
    ];
    // Two CCW triangles viewed from above: TL-BL-TR, TR-BL-BR
    let indices = Indices::U32(vec![0, 2, 1,  1, 2, 3]);

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL,   normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0,     uvs);
    mesh.insert_indices(indices);
    mesh
}

/// Terrain height query — always returns 0 in flat/debug mode.
pub fn terrain_height_at(_fx: f32, _fz: f32, _cfg: &TerrainConfig) -> f32 {
    0.0
}


