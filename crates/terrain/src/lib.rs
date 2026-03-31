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
    pbr::wireframe::{Wireframe, WireframePlugin},
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
    /// Post-rebuild cooldown: no new rebuild until this reaches 0.
    rebuild_cooldown:      u32,
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

/// Frame countdown for visible tiles awaiting despawn.  Gives children time to
/// build so the parent mesh covers the gap until they appear.
#[derive(Component)]
struct TileDespawnDelay(u32);

/// Frames a visible parent tile waits before being removed after its children
/// are spawned.  Should comfortably exceed a typical mesh-build round-trip.
const TILE_DESPAWN_DELAY_FRAMES: u32 = 120; // ≈ 2 s at 60 fps

// ── Debug state ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
pub struct TerrainDebug {
    pub wireframe: bool,
}

// ── Plugin ───────────────────────────────────────────────────────────────────

pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(WireframePlugin)
            .init_resource::<TerrainConfig>()
            .init_resource::<TerrainDebug>()
            .add_systems(Startup, setup_terrain)
            .add_systems(Update, (update_terrain, poll_terrain_tasks, despawn_stale_tiles, watch_geodata_loads, toggle_wireframe));
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
        tree:                  QuadTree::new(),
        material,
        last_geodata_gen:      0,
        pending_geodata_gen:   0,
        geodata_stable_frames: 0,
        rebuild_cooldown:      0,
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
                // Keep visible mesh alive while children/parent builds, preventing gaps.
                commands.entity(entity).insert((TileDespawnPending, TileDespawnDelay(TILE_DESPAWN_DELAY_FRAMES)));
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
    debug:        Res<TerrainDebug>,
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
                let mut ec = commands.entity(entity);
                ec.remove::<TerrainBuildTask>().insert((
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(state.material.clone()),
                    Transform::default(),
                    TerrainTile,
                ));
                if debug.wireframe {
                    ec.insert(Wireframe);
                }
            }
        }
    }
}

/// Despawn tile entities that have been marked for removal.
///
/// * Orphaned build tasks (no mesh yet) — removed immediately.
/// * Visible tile meshes (TerrainTile) — kept alive for TILE_DESPAWN_DELAY_FRAMES
///   so the parent mesh bridges the gap while child tiles finish building.
fn despawn_stale_tiles(
    mut commands: Commands,
    // Orphaned tasks: despawn now.
    tasks:   Query<Entity, (With<TileDespawnPending>, Without<TerrainBuildTask>, Without<TerrainTile>)>,
    // Visible tiles: count down then despawn.
    mut visible: Query<(Entity, &mut TileDespawnDelay), (With<TileDespawnPending>, Without<TerrainBuildTask>, With<TerrainTile>)>,
) {
    for entity in tasks.iter() {
        commands.entity(entity).despawn_recursive();
    }
    for (entity, mut delay) in visible.iter_mut() {
        if delay.0 == 0 {
            commands.entity(entity).despawn_recursive();
        } else {
            delay.0 -= 1;
        }
    }
}

/// Consecutive stable frames required before a rebuild is triggered.
const GEODATA_DEBOUNCE_FRAMES: u32 = 120; // ≈ 2 s at 60 fps
/// Minimum frames between successive tree rebuilds.
const REBUILD_COOLDOWN_FRAMES: u32 = 300; // ≈ 5 s at 60 fps

/// When new elevation tiles have been loaded into the GeoCache, reset the
/// quad-tree so all tiles rebuild with real heights.  Only fires once the
/// tile build queue is empty so initial flat tiles are always visible first.
fn watch_geodata_loads(
    cfg:          Res<TerrainConfig>,
    mut state:    ResMut<TerrainState>,
    mut commands: Commands,
    building:     Query<&TerrainBuildTask>,
) {
    // Enforce post-rebuild cooldown.
    if state.rebuild_cooldown > 0 {
        state.rebuild_cooldown -= 1;
        return;
    }

    let Some(ref cache) = cfg.geo_cache else { return; };
    let current = cache.fetch_count();

    if current == state.last_geodata_gen {
        state.geodata_stable_frames = 0;
        return;
    }

    if current != state.pending_geodata_gen {
        state.pending_geodata_gen   = current;
        state.geodata_stable_frames = 0;
        return;
    }

    state.geodata_stable_frames += 1;
    if state.geodata_stable_frames < GEODATA_DEBOUNCE_FRAMES {
        return;
    }

    // Don't rebuild while tile meshes are still being built.
    if !building.is_empty() {
        return;
    }

    // Stable, idle, and cooled down — rebuild with real heights.
    state.last_geodata_gen      = current;
    state.geodata_stable_frames = 0;
    state.rebuild_cooldown      = REBUILD_COOLDOWN_FRAMES;

    for op in state.tree.reset() {
        if let crate::quadtree::DeltaOp::Despawn(e) = op {
            commands.entity(e).insert((TileDespawnPending, TileDespawnDelay(TILE_DESPAWN_DELAY_FRAMES)));
        }
    }
}

/// Ctrl+D toggles wireframe overlay on all terrain tiles.
fn toggle_wireframe(
    keys:         Res<ButtonInput<KeyCode>>,
    mut debug:    ResMut<TerrainDebug>,
    mut commands: Commands,
    tiles:        Query<Entity, With<TerrainTile>>,
) {
    let ctrl = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);
    if !(keys.just_pressed(KeyCode::KeyD) && ctrl) {
        return;
    }
    debug.wireframe = !debug.wireframe;
    for entity in tiles.iter() {
        if debug.wireframe {
            commands.entity(entity).insert(Wireframe);
        } else {
            commands.entity(entity).remove::<Wireframe>();
        }
    }
}
