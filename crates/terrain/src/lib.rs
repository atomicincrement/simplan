// terrain crate – Bevy plugin wiring quad-tree LOD terrain to the sim.
//
// Architecture
// ============
//  TerrainConfig  (Resource) – tweakable parameters
//  TerrainState   (Resource) – owns the QuadTree and the shared material handle
//  TerrainMarker  (Component)– marks all tile entities so they can be queried
//
// Every Update frame `update_terrain` runs:
//   1. Reads camera (or aircraft) world position.
//   2. Calls QuadTree::update() which diffs desired vs. current tile set.
//   3. Despawns removed tiles; builds and spawns new tiles.
//
// The flat-plane projection means tiles close to the origin (runway area) match
// the existing y = 0 physics collider perfectly up to curvature error
// (≈ 8 mm over 10 km, ≈ 78 m over 1 000 km).

pub mod noise;
pub mod sphere;
pub mod tile_mesh;
pub mod quadtree;

use bevy::prelude::*;

use quadtree::{DeltaOp, QuadTree};
use tile_mesh::build_tile_mesh;

// ── Configuration ────────────────────────────────────────────────────────────

/// Terrain generation parameters.  Insert as a resource before adding the
/// plugin, or the defaults will be used.
#[derive(Resource, Clone)]
pub struct TerrainConfig {
    /// Base spatial frequency of the noise evaluated on the unit sphere normal.
    /// noise_frequency / EARTH_RADIUS ≈ cycles per metre in the tangent plane.
    /// 400 → wavelength ≈ 16 km; dramatic mountains visible at 10–50 km range.
    pub noise_frequency: f32,
    /// Number of FBM octaves.  More octaves = more detail but slower meshing.
    pub noise_octaves: u32,
    /// Peak-to-trough height of the terrain in metres (+2 000 m / −2 000 m).
    pub height_scale: f32,
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            noise_frequency: 400.0,
            noise_octaves:   7,
            height_scale:    2_000.0,
        }
    }
}

// ── State ────────────────────────────────────────────────────────────────────

#[derive(Resource)]
pub struct TerrainState {
    pub tree:     QuadTree,
    pub material: Handle<StandardMaterial>,
}

// ── Marker component ─────────────────────────────────────────────────────────

#[derive(Component)]
pub struct TerrainTile;

// ── Plugin ───────────────────────────────────────────────────────────────────

pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TerrainConfig>()
            .add_systems(Startup, setup_terrain)
            .add_systems(Update, update_terrain);
    }
}

fn setup_terrain(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 0.9,
        reflectance: 0.1,
        ..default()
    });

    commands.insert_resource(TerrainState {
        tree:     QuadTree::new(),
        material,
    });
}

fn update_terrain(
    mut state:     ResMut<TerrainState>,
    cfg:           Res<TerrainConfig>,
    camera_q:      Query<&GlobalTransform, With<Camera3d>>,
    mut commands:  Commands,
    mut meshes:    ResMut<Assets<Mesh>>,
) {
    // Determine the observer position in flat (x, z) coordinates.
    // We use the camera; fall back to origin if no camera exists yet.
    let cam_pos = camera_q
        .get_single()
        .map(|gt| gt.translation())
        .unwrap_or(Vec3::ZERO);

    let cam_x = cam_pos.x;
    let cam_z = cam_pos.z;

    // Run the quad-tree update.
    let ops = state.tree.update(cam_x, cam_z);

    // We need to clone the material handle before the mutable borrow below.
    let mat_handle = state.material.clone();

    // Process operations.
    for op in ops {
        match op {
            DeltaOp::Despawn(entity) => {
                commands.entity(entity).despawn_recursive();
            }
            DeltaOp::Spawn { cx, cz, half, slot } => {
                let mesh = build_tile_mesh(cx, cz, half, &cfg);
                let mesh_handle = meshes.add(mesh);

                let entity = commands
                    .spawn((
                        Mesh3d(mesh_handle),
                        MeshMaterial3d(mat_handle.clone()),
                        Transform::default(),
                        TerrainTile,
                    ))
                    .id();

                state.tree.assign_entity(slot, entity);
            }
        }
    }
}
