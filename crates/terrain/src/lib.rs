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

use bevy::{
    prelude::*,
    tasks::{futures_lite::future, AsyncComputeTaskPool, Task},
};

use geodata::GeoCache;
use quadtree::{DeltaOp, QuadTree};
use tile_mesh::build_tile_mesh;

// ── Configuration ────────────────────────────────────────────────────────────

/// Terrain generation parameters.  Insert as a resource before adding the
/// plugin, or the defaults will be used.
#[derive(Resource, Clone)]
pub struct TerrainConfig {
    // ── Procedural noise (fallback when geo_cache is None) ────────────────
    pub noise_frequency: f32,
    pub noise_octaves: u32,
    pub height_scale: f32,
    // ── Real-world elevation ───────────────────────────────────────────────
    /// When Some, fetches real elevation from AWS Terrain Tiles on demand.
    pub geo_cache:  Option<GeoCache>,
    /// WGS-84 latitude of the world origin (degrees).
    pub centre_lat: f64,
    /// WGS-84 longitude of the world origin (degrees).
    pub centre_lon: f64,
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            noise_frequency: 400.0,
            noise_octaves:   7,
            height_scale:    2_000.0,
            geo_cache:   None,
            centre_lat:  geodata::CENTRE_LAT,
            centre_lon:  geodata::CENTRE_LON,
        }
    }
}

// ── State ────────────────────────────────────────────────────────────────────

#[derive(Resource)]
pub struct TerrainState {
    pub tree:              QuadTree,
    pub material:          Handle<StandardMaterial>,
    /// Last value of GeoCache::fetch_count() that triggered a tree rebuild.
    pub last_geodata_gen:  u32,
    /// Most recently observed fetch_count (may be ahead of last_geodata_gen).
    pending_geodata_gen:   u32,
    /// Frames for which pending_geodata_gen has been stable (debounce counter).
    geodata_stable_frames: u32,
}

// ── Marker component ─────────────────────────────────────────────────────────

#[derive(Component)]
pub struct TerrainTile;

/// Holds an in-flight async mesh-build task; replaced by the full tile once done.
#[derive(Component)]
pub struct TerrainBuildTask(pub Task<Mesh>);

/// Marks a tile slot (or task entity) that should be despawned once it is safe
/// to do so — i.e. after any in-flight build task has been resolved.
#[derive(Component)]
struct TileDespawnPending;

// ── Plugin ───────────────────────────────────────────────────────────────────

pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TerrainConfig>()
            .add_systems(Startup, setup_terrain)
            .add_systems(Update, (update_terrain, poll_terrain_tasks, despawn_stale_tiles, watch_geodata_loads));
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
        tree:                 QuadTree::new(),
        material,
        last_geodata_gen:     0,
        pending_geodata_gen:  0,
        geodata_stable_frames: 0,
    });
}

fn update_terrain(
    mut state:    ResMut<TerrainState>,
    cfg:          Res<TerrainConfig>,
    camera_q:     Query<&GlobalTransform, With<Camera3d>>,
    mut commands: Commands,
) {
    let cam_pos = camera_q
        .get_single()
        .map(|gt| gt.translation())
        .unwrap_or(Vec3::ZERO);

    let ops = state.tree.update(cam_pos.x, cam_pos.z);
    let pool = AsyncComputeTaskPool::get();

    for op in ops {
        match op {
            DeltaOp::Despawn(entity) => {
                // Don't despawn immediately — the entity may still have a
                // TerrainBuildTask in flight.  Mark it and let
                // poll_terrain_tasks / despawn_stale_tiles resolve it safely.
                commands.entity(entity).insert(TileDespawnPending);
            }
            DeltaOp::Spawn { cx, cz, half, slot } => {
                let cfg_clone = cfg.clone();
                let task: Task<Mesh> =
                    pool.spawn(async move { build_tile_mesh(cx, cz, half, &cfg_clone) });
                let entity = commands.spawn(TerrainBuildTask(task)).id();
                state.tree.assign_entity(slot, entity);
            }
        }
    }
}

/// Promote completed mesh-build tasks to full tile entities.
fn poll_terrain_tasks(
    state:        Res<TerrainState>,
    mut commands: Commands,
    mut meshes:   ResMut<Assets<Mesh>>,
    mut query:    Query<(Entity, &mut TerrainBuildTask, Option<&TileDespawnPending>)>,
) {
    for (entity, mut task, despawn_pending) in query.iter_mut() {
        if let Some(mesh) = future::block_on(future::poll_once(&mut task.0)) {
            if despawn_pending.is_some() {
                // The tile slot was evicted before the mesh finished — discard.
                commands.entity(entity).despawn_recursive();
            } else {
                commands
                    .entity(entity)
                    .remove::<TerrainBuildTask>()
                    .insert((
                        Mesh3d(meshes.add(mesh)),
                        MeshMaterial3d(state.material.clone()),
                        Transform::default(),
                        TerrainTile,
                    ));
            }
        }
    }
}

/// Despawn tile entities that have been marked for removal and are no longer
/// waiting on a build task.
fn despawn_stale_tiles(
    mut commands: Commands,
    query:        Query<Entity, (With<TileDespawnPending>, Without<TerrainBuildTask>)>,
) {
    for entity in query.iter() {
        commands.entity(entity).despawn_recursive();
    }
}

/// How many consecutive frames fetch_count must be unchanged before we
/// rebuild the terrain.  Prevents a reset→fetch→reset cascade.
const GEODATA_DEBOUNCE_FRAMES: u32 = 30; // ≈ 0.5 s at 60 fps

/// When new elevation tiles have been loaded into the GeoCache, reset the
/// quad-tree so all tiles rebuild with real heights.
fn watch_geodata_loads(
    cfg:          Res<TerrainConfig>,
    mut state:    ResMut<TerrainState>,
    mut commands: Commands,
) {
    let Some(ref cache) = cfg.geo_cache else { return; };
    let current = cache.fetch_count();

    if current == state.last_geodata_gen {
        // Nothing new since the last rebuild.
        state.geodata_stable_frames = 0;
        return;
    }

    if current != state.pending_geodata_gen {
        // New tiles just landed — reset the debounce counter.
        state.pending_geodata_gen   = current;
        state.geodata_stable_frames = 0;
        return;
    }

    // fetch_count has been stable for another frame.
    state.geodata_stable_frames += 1;
    if state.geodata_stable_frames < GEODATA_DEBOUNCE_FRAMES {
        return;
    }

    // Stable long enough — rebuild with real heights.
    state.last_geodata_gen     = current;
    state.geodata_stable_frames = 0;

    for op in state.tree.reset() {
        if let crate::quadtree::DeltaOp::Despawn(e) = op {
            commands.entity(e).insert(TileDespawnPending);
        }
    }
}
