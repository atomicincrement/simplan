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
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    tasks::{futures_lite::future, AsyncComputeTaskPool, Task},
};
use bevy::render::mesh::VertexAttributeValues;

use geodata::{
    GeoCache,
    tile::{flat_to_lat_lon, lat_lon_to_tile_xy, zoom_for_half},
};
use crate::quadtree::ROOT_HALF;
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
    /// When true, force the quad-tree to full subdivision and render only
    /// the highest-resolution leaf tiles (useful for image-alignment
    /// debugging with the top-down camera).
    pub force_max_res: bool,
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
            force_max_res: false,
        }
    }
}

// ── State ────────────────────────────────────────────────────────────────────

#[derive(Resource)]
pub struct TerrainState {
    pub tree:              QuadTree,
    /// Fallback material for tiles without a satellite texture.
    pub material:          Handle<StandardMaterial>,
    // ── Elevation debounce ─────────────────────────────────────────────────
    pub last_geodata_gen:  u32,
    pending_geodata_gen:   u32,
    geodata_stable_frames: u32,
    rebuild_cooldown:      u32,
    // ── Imagery debounce ─────────────────────────────────────────────────
    last_imagery_gen:      u32,
    pending_imagery_gen:   u32,
    imagery_stable_frames: u32,
    imagery_cooldown:      u32,
}

// ── Marker component ─────────────────────────────────────────────────────────

#[derive(Component)]
pub struct TerrainTile;

/// Holds an in-flight async task that builds the mesh and optionally fetches
/// a satellite imagery tile.  The `Vec<u8>` is 256×256 RGBA8 if available.
/// When present the imagery is returned alongside the Web-Mercator tile
/// coordinates `(tx, ty, z)` that were used to generate it.
#[derive(Component)]
pub struct TerrainBuildTask(pub Task<(Mesh, Option<(Vec<u8>, i32, i32, u32)>)>);

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
            .add_systems(Update, (
                update_terrain,
                poll_terrain_tasks,
                despawn_stale_tiles,
                watch_geodata_loads,
                watch_imagery_loads,
                toggle_wireframe,
            ));
    }
}

fn setup_terrain(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    cfg: Res<TerrainConfig>,
    mut debug: ResMut<TerrainDebug>,
) {
    let material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 0.9,
        reflectance: 0.1,
        ..default()
    });

    // Enable wireframe by default during debugging so tile boundaries show.
    debug.wireframe = true;

    let mut tree = QuadTree::new();
    if cfg.force_max_res {
        tree.force_max_depth();
    }

    commands.insert_resource(TerrainState {
        tree,
        material,
        last_geodata_gen:      0,
        pending_geodata_gen:   0,
        geodata_stable_frames: 0,
        rebuild_cooldown:      0,
        last_imagery_gen:      0,
        pending_imagery_gen:   0,
        imagery_stable_frames: 0,
        imagery_cooldown:      0,
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

    let ops = if cfg.force_max_res {
        state.tree.update_force_leaves()
    } else {
        state.tree.update(cam_pos.x, cam_pos.z)
    };
    let pool = AsyncComputeTaskPool::get();

    for op in ops {
        match op {
            DeltaOp::Despawn(entity) => {
                // Keep visible mesh alive while children/parent builds, preventing gaps.
                commands.entity(entity).insert((TileDespawnPending, TileDespawnDelay(TILE_DESPAWN_DELAY_FRAMES)));
            }
            DeltaOp::Spawn { cx, cz, half, slot } => {
                let cfg_clone = cfg.clone();
                let task: Task<(Mesh, Option<(Vec<u8>, i32, i32, u32)>)> = pool.spawn(async move {
                    let mesh    = build_tile_mesh(cx, cz, half, &cfg_clone);
                    let imagery = sample_imagery(cx, cz, half, &cfg_clone);
                    (mesh, imagery)
                });
                let entity = commands.spawn(TerrainBuildTask(task)).id();
                state.tree.assign_entity(slot, entity);
            }
        }
    }
}

/// Promote completed mesh-build tasks to full tile entities.
///
/// When the task also returned RGBA satellite imagery, creates a unique
/// `StandardMaterial` with a satellite texture for that tile.  Otherwise
/// falls back to the shared altitude-colour material.
fn poll_terrain_tasks(
    state:         Res<TerrainState>,
    debug:         Res<TerrainDebug>,
    mut commands:  Commands,
    mut meshes:    ResMut<Assets<Mesh>>,
    mut images:    ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut query:     Query<(Entity, &mut TerrainBuildTask, Option<&TileDespawnPending>)>,
) {
    for (entity, mut task, despawn_pending) in query.iter_mut() {
            if let Some((mesh, rgba_opt)) = future::block_on(future::poll_once(&mut task.0)) {
            if despawn_pending.is_some() {
                // The tile slot was evicted before the mesh finished — discard.
                commands.entity(entity).despawn_recursive();
            } else {
                // Log mesh vertex positions and UVs for debugging alignment.
                if let Some(pos_attr) = mesh.attribute(bevy::render::mesh::Mesh::ATTRIBUTE_POSITION) {
                    if let VertexAttributeValues::Float32x3(pts) = pos_attr {
                        let mut xmin = f32::INFINITY; let mut xmax = f32::NEG_INFINITY;
                        let mut ymin = f32::INFINITY; let mut ymax = f32::NEG_INFINITY;
                        let mut zmin = f32::INFINITY; let mut zmax = f32::NEG_INFINITY;
                        for p in pts.iter() {
                            xmin = xmin.min(p[0]); xmax = xmax.max(p[0]);
                            ymin = ymin.min(p[1]); ymax = ymax.max(p[1]);
                            zmin = zmin.min(p[2]); zmax = zmax.max(p[2]);
                        }
                        println!(
                            "[terrain] mesh pos bounds x=({:.2}..{:.2}) y=({:.2}..{:.2}) z=({:.2}..{:.2}) verts={}",
                            xmin, xmax, ymin, ymax, zmin, zmax, pts.len()
                        );
                    }
                }
                if let Some(uv_attr) = mesh.attribute(bevy::render::mesh::Mesh::ATTRIBUTE_UV_0) {
                    if let VertexAttributeValues::Float32x2(uvs) = uv_attr {
                        let sample_count = uvs.len().min(6);
                        let samples: Vec<String> = uvs.iter().take(sample_count).map(|uv| format!("({:.3},{:.3})", uv[0], uv[1])).collect();
                        println!("[terrain] mesh uv samples: {}", samples.join(", "));
                    }
                }

                let mat_handle = if let Some((rgba, tx, ty, z)) = rgba_opt {
                    // Build a per-tile material backed by the satellite texture.
                    let img = Image::new(
                        Extent3d { width: 256, height: 256, depth_or_array_layers: 1 },
                        TextureDimension::D2,
                        rgba,
                        TextureFormat::Rgba8UnormSrgb,
                        bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
                    );
                    let tex = images.add(img);
                    // Log the Web-Mercator tile coordinates applied to this tile.
                    println!("[terrain] applied imagery z={}/x={}/y={}", z, tx, ty);
                    materials.add(StandardMaterial {
                        base_color_texture: Some(tex),
                        // Make the satellite quad unlit and double-sided so it's
                        // clearly visible during top-down debug sessions.
                        unlit: true,
                        double_sided: true,
                        cull_mode: None,
                        perceptual_roughness: 0.9,
                        reflectance: 0.1,
                        ..default()
                    })
                } else {
                    // No imagery yet — use the shared altitude-colour material.
                    state.material.clone()
                };

                let mut ec = commands.entity(entity);
                ec.remove::<TerrainBuildTask>().insert((
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(mat_handle),
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

// ── Satellite imagery helper ───────────────────────────────────────────────────

/// Try to get satellite imagery for the terrain tile centred at `(cx, cz)`
/// with half-size `half`.  Returns the 256×256 RGBA8 bytes if the tile is
/// already in the memory cache; otherwise fires a background fetch and
/// returns `None` (the tree will rebuild when the fetch completes).
fn sample_imagery(cx: f32, cz: f32, half: f32, cfg: &TerrainConfig) -> Option<(Vec<u8>, i32, i32, u32)> {
    let cache = cfg.geo_cache.as_ref()?;
    // Choose zoom based on this tile's half-size so the selected Web-Mercator
    // zoom corresponds to the terrain tile resolution.  Matches the example
    // stitcher which anchors the global grid at the root origin.
    let zoom = zoom_for_half(half, cfg.centre_lat);

    // Anchor sampling at the tile centre.  This avoids column shifts when
    // corners fall near Web-Mercator tile boundaries and produces a
    // contiguous tile sequence for the mosaic (matches
    // `stitch_mosaic_center` example).
    let (lat, lon) = flat_to_lat_lon(cx, cz, cfg.centre_lat, cfg.centre_lon);
    let (tx, ty) = lat_lon_to_tile_xy(lat, lon, zoom);
    let tile = cache.get_imagery_tile_nonblocking(tx, ty, zoom)?;
    Some((tile.rgba.clone(), tx, ty, zoom))
}

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
    // In debug 'force max resolution' mode we don't want automatic resets
    // to collapse the tree back to a coarse parent — keep the forced
    // highest-resolution leaves intact until the mode is disabled.
    if cfg.force_max_res {
        return;
    }
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

    // If we're in debug 'force max resolution' mode, re-subdivide the
    // cleared tree so subsequent updates will spawn highest-resolution
    // leaf tiles instead of leaving a single root tile.
    if cfg.force_max_res {
        state.tree.force_max_depth();
    }

    // If we're in debug 'force max resolution' mode, re-subdivide the
    // cleared tree so subsequent updates will spawn highest-resolution
    // leaf tiles instead of leaving a single root tile.
    if cfg.force_max_res {
        state.tree.force_max_depth();
    }
}

/// Mirror of `watch_geodata_loads` for satellite imagery.
///
/// Once imagery tiles finish downloading (detected via `imagery_fetch_count`),
/// the quad-tree is reset so tiles are respawned with satellite textures.
fn watch_imagery_loads(
    cfg:          Res<TerrainConfig>,
    mut state:    ResMut<TerrainState>,
    mut commands: Commands,
    building:     Query<&TerrainBuildTask>,
) {
    if state.imagery_cooldown > 0 {
        state.imagery_cooldown -= 1;
        return;
    }

    let Some(ref cache) = cfg.geo_cache else { return; };
    // When `force_max_res` is enabled we deliberately avoid resetting the
    // quad-tree in response to imagery arrivals so the high-resolution
    // mosaic remains stable for visual comparison/debugging.
    if cfg.force_max_res {
        return;
    }
    let current = cache.imagery_fetch_count();

    if current == state.last_imagery_gen {
        state.imagery_stable_frames = 0;
        return;
    }

    if current != state.pending_imagery_gen {
        state.pending_imagery_gen   = current;
        state.imagery_stable_frames = 0;
        return;
    }

    state.imagery_stable_frames += 1;
    if state.imagery_stable_frames < GEODATA_DEBOUNCE_FRAMES {
        return;
    }

    if !building.is_empty() {
        return;
    }

    state.last_imagery_gen      = current;
    state.imagery_stable_frames = 0;
    state.imagery_cooldown      = REBUILD_COOLDOWN_FRAMES;

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
