use avian3d::prelude::*;
use bevy::input::gamepad::{Gamepad, GamepadAxis};
use bevy::input::mouse::MouseWheel;
use bevy::prelude::*;
use fdm::atmo::atmosphere;
use fdm::f35::aero::{aerodynamics, AeroIn};
use fdm::f35::prop::propulsion;
use fdm::math::Vec3 as FdmVec3;
use geodata::GeoCache;
use terrain::{TerrainConfig, TerrainPlugin};
use terrain::tile_mesh::build_tile_mesh;
use terrain::quadtree::ROOT_HALF;
use geodata::tile::{flat_to_lat_lon, lat_lon_to_tile_xy, zoom_for_half};
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

mod sky;
use sky::{SkyPlugin, SUN_DIR};

// ── Unit-conversion constants (imperial ↔ SI) ────────────────────────────────
const M_TO_FT: f32 = 3.280_84;
const LBF_TO_N: f32 = 4.448_22;
const LBFFT_TO_NM: f32 = 1.355_82;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Simplan – Flight Simulator".into(),
                ..default()
            }),
            ..default()
        }))
        // Minimal runtime for terrain-only debugging.
        .insert_resource(TerrainConfig {
            geo_cache: Some(GeoCache::new()),
            force_max_res: true,
            ..default()
        })
        // Don't add the TerrainPlugin for this diagnostic run — we spawn
        // labeled tiles directly from `spawn_labeled_tiles`.
        // .add_plugins(TerrainPlugin)
        .add_plugins(SkyPlugin)
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(PilotControls::default())
        .insert_resource(JoystickActive::default())
        .add_systems(Startup, (setup, spawn_labeled_tiles, setup_hud))
        .add_systems(Update, (
            follow_camera,
            joystick_controls,
            mouse_controls.after(joystick_controls),
            apply_aerodynamics,
            update_hud,
        ))
        .run();
}

// ── Marker components ───────────────────────────────────────────────────────

#[derive(Component)]
struct Aircraft;

#[derive(Component)]
struct FollowCamera;

// HUD element markers
#[derive(Component)] struct ThrottleFill;
#[derive(Component)] struct CompassDisplay;
#[derive(Component)] struct StickDot;
#[derive(Component)] struct AltimeterDisplay;
#[derive(Component)] struct AirspeedDisplay;

// ── Pilot inputs ─────────────────────────────────────────────────────────────

#[derive(Resource)]
struct PilotControls {
    /// 0 = idle, 1 = full afterburner
    throttle: f64,
    /// Elevator deflection in radians (+nose-up)
    elevator: f64,
    /// Aileron deflection in radians (+right-wing-down)
    aileron: f64,
    /// Rudder deflection in radians (+nose-right)
    rudder: f64,
}

impl Default for PilotControls {
    fn default() -> Self {
        Self {
            throttle: 0.6,
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
        }
    }
}

// ── Startup system ──────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ── Camera ────────────────────────────────────────────────────────────
    // Far plane set to 5 000 km so the 900 km skydome and distant terrain
    // tiles are never clipped.
    // Aircraft starts 5 km north of the map's north edge (Z = -10 000),
    // 2 km up, heading south (+Z).  Camera is 35 m aft (-Z world) and 8 m above.
    let cam_pos  = Vec3::new(0.0, 2008.0, -10035.0);
    let look_pos = Vec3::new(0.0, 2001.0,  -9950.0); // 50 m ahead of nose (south)
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            near: 0.5,
            far:  5_000_000.0,
            ..default()
        }),
        Transform::from_translation(cam_pos).looking_at(look_pos, Vec3::Y),
        FollowCamera,
    ));

    // ── Sun (early morning – eastern horizon, ~8° elevation) ─────────────
    //
    // Light rays travel in  −SUN_DIR  (from east, angled slightly downward).
    // DirectionalLight always emits along its local −Z axis, so we rotate
    // the identity transform so that local −Z = −SUN_DIR.
    let light_dir = -SUN_DIR.normalize();
    let sun_rotation = Quat::from_rotation_arc(Vec3::NEG_Z, light_dir);
    commands.spawn((
        DirectionalLight {
            // Soft, warm orange-gold sunrise illuminance (vs ~100 000 lux at noon).
            illuminance: 7_500.0,
            color: Color::srgb(1.0, 0.82, 0.60),
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(sun_rotation),
    ));

    // ── Ground physics collider (half-space at y = 0) ─────────────────────
    commands.spawn((
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
    ));

    // ── Aircraft ──────────────────────────────────────────────────────────
    spawn_aircraft(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, 2000.0, -10000.0),
    );

}

/// Spawn a deterministic GRID×GRID mosaic of debug quads at startup.
/// Each quad is a flat tile built with `build_tile_mesh`.  If the
/// `GeoCache` already contains imagery for the tile the texture is
/// attached non-blocking; otherwise a plain white unlit material is used.
fn spawn_mosaic(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    cfg: Res<TerrainConfig>,
) {
    const GRID: usize = 4;
    let n = GRID as f32;
    let half = ROOT_HALF / n; // tile half-size
    let tile_size = half * 2.0;

    for ix in 0..GRID {
        for iz in 0..GRID {
            let cx = -ROOT_HALF + (ix as f32 + 0.5) * tile_size;
            let cz = -ROOT_HALF + (iz as f32 + 0.5) * tile_size;
            let mesh = build_tile_mesh(cx, cz, half, &cfg);
            let mesh_handle = meshes.add(mesh);

            let mat = if let Some(cache) = cfg.geo_cache.as_ref() {
                let zoom = zoom_for_half(half, cfg.centre_lat);
                let (lat, lon) = flat_to_lat_lon(cx - half, cz - half, cfg.centre_lat, cfg.centre_lon);
                let (tx, ty) = lat_lon_to_tile_xy(lat, lon, zoom);
                if let Some(tile) = cache.get_imagery_tile_nonblocking(tx, ty, zoom) {
                    let rgba = tile.rgba.clone();
                    let img = Image::new(
                        Extent3d { width: 256, height: 256, depth_or_array_layers: 1 },
                        TextureDimension::D2,
                        rgba,
                        TextureFormat::Rgba8UnormSrgb,
                        bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
                    );
                    let tex = images.add(img);
                    materials.add(StandardMaterial {
                        base_color_texture: Some(tex),
                        unlit: true,
                        double_sided: true,
                        cull_mode: None,
                        perceptual_roughness: 0.9,
                        reflectance: 0.1,
                        ..default()
                    })
                } else {
                    materials.add(StandardMaterial {
                        base_color: Color::WHITE,
                        unlit: true,
                        double_sided: true,
                        cull_mode: None,
                        ..default()
                    })
                }
            } else {
                materials.add(StandardMaterial {
                    base_color: Color::WHITE,
                    unlit: true,
                    double_sided: true,
                    cull_mode: None,
                    ..default()
                })
            };

            commands.spawn((
                Mesh3d(mesh_handle),
                MeshMaterial3d(mat),
                Transform::default(),
            ));
        }
    }
}

/// Fetch a 4×4 grid of Web-Mercator tiles at zoom 13, label each image with
/// its tile coordinates, create textures and spawn flat meshes with those
/// textures.  This bypasses the `terrain` module and is intended for quick
/// visual verification.
fn spawn_labeled_tiles(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    cfg: Res<TerrainConfig>,
) {
    let Some(cache) = cfg.geo_cache.as_ref() else { return; };

    // Fixed request: 4×4 at zoom 13 anchored to ROOT_HALF origin
    let zoom: u32 = 13;
    const GRID: usize = 4;
    let n = GRID as f32;
    let half = ROOT_HALF / n; // per-tile half-size in metres
    let tile_meter = half * 2.0;

    // Compute origin Web-Mercator tile for root top-left corner.
    let (lat_origin, lon_origin) = flat_to_lat_lon(-ROOT_HALF, -ROOT_HALF, cfg.centre_lat, cfg.centre_lon);
    let (tx_origin, ty_origin) = lat_lon_to_tile_xy(lat_origin, lon_origin, zoom);

    for iz in 0..GRID {
        for ix in 0..GRID {
            let ix_i = ix as i32;
            let iz_i = iz as i32;
            let tx = tx_origin + ix_i;
            let ty = ty_origin + iz_i;

            // Blocking fetch (disk or network) to ensure texture exists.
            let tile = cache.get_imagery_tile(tx, ty, zoom);
            let mut rgba = tile.rgba.clone();

            // Draw a tiny label (tx/ty) into the top-left of the image.
            // draw_label_small(&mut rgba, 256, 256, &format!("{}/{}", tx, ty));

            let img = Image::new(
                Extent3d { width: 256, height: 256, depth_or_array_layers: 1 },
                TextureDimension::D2,
                rgba,
                TextureFormat::Rgba8UnormSrgb,
                bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
            );
            let tex = images.add(img);
            let mat = materials.add(StandardMaterial {
                base_color_texture: Some(tex),
                unlit: true,
                double_sided: true,
                cull_mode: None,
                perceptual_roughness: 0.9,
                reflectance: 0.1,
                alpha_mode: AlphaMode::Blend,
                ..default()
            });

            // Compute world centre for this tile and spawn a quad mesh.
            let cx = -ROOT_HALF + (ix as f32 + 0.5) * tile_meter;
            let cz = -ROOT_HALF + (iz as f32 + 0.5) * tile_meter;
            let mesh = build_tile_mesh(cx, cz, half, &cfg);
            let mesh_handle = meshes.add(mesh);
            commands.spawn((
                Mesh3d(mesh_handle),
                MeshMaterial3d(mat),
                Transform::default(),
            ));
        }
    }
}

/// Small 3x5 bitmap font renderer (scaled) to stamp short numeric labels
/// into a 256×256 RGBA image buffer.  Text is drawn at top-left with a tiny
/// scale so labels are visible but unobtrusive.
fn draw_label_small(buf: &mut [u8], w: u32, h: u32, text: &str) {
    // 3x5 digit font (rows from top to bottom). Each row is 3 bits (LSB unused).
    const DIGITS: [[[u8; 3]; 5]; 10] = [
        // 0
        [[1,1,1],[1,0,1],[1,0,1],[1,0,1],[1,1,1]],
        // 1
        [[0,1,0],[1,1,0],[0,1,0],[0,1,0],[1,1,1]],
        // 2
        [[1,1,1],[0,0,1],[1,1,1],[1,0,0],[1,1,1]],
        // 3
        [[1,1,1],[0,0,1],[1,1,1],[0,0,1],[1,1,1]],
        // 4
        [[1,0,1],[1,0,1],[1,1,1],[0,0,1],[0,0,1]],
        // 5
        [[1,1,1],[1,0,0],[1,1,1],[0,0,1],[1,1,1]],
        // 6
        [[1,1,1],[1,0,0],[1,1,1],[1,0,1],[1,1,1]],
        // 7
        [[1,1,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1]],
        // 8
        [[1,1,1],[1,0,1],[1,1,1],[1,0,1],[1,1,1]],
        // 9
        [[1,1,1],[1,0,1],[1,1,1],[0,0,1],[1,1,1]],
    ];

    let scale = 4; // scale each font pixel to 4x4
    let padding = 6;
    let mut cursor_x = padding as i32;
    let cursor_y = padding as i32;

    for ch in text.chars() {
        if ch >= '0' && ch <= '9' {
            let d = (ch as u8 - b'0') as usize;
            // draw 3x5 bitmap scaled
            for ry in 0..5 {
                for rx in 0..3 {
                    if DIGITS[d][ry][rx] != 0 {
                        let px = cursor_x + (rx as i32) * scale;
                        let py = cursor_y + (ry as i32) * scale;
                        draw_rect(buf, w, h, px as i32, py as i32, scale, scale, [255,255,0,200]);
                    }
                }
            }
            cursor_x += (3 * scale + 2) as i32;
        } else if ch == '/' || ch == ',' || ch == '-' {
            // small separator dot or slash
            if ch == '/' {
                draw_rect(buf, w, h, cursor_x, cursor_y + scale, scale/2, scale*3, [255,255,0,200]);
            } else {
                draw_rect(buf, w, h, cursor_x, cursor_y + scale*2, scale/2, scale/2, [255,255,0,200]);
            }
            cursor_x += (scale + 2) as i32;
        } else {
            cursor_x += (scale + 2) as i32;
        }
    }
}

fn draw_rect(buf: &mut [u8], w: u32, h: u32, x: i32, y: i32, rw: i32, rh: i32, color: [u8;4]) {
    for yy in 0..rh {
        for xx in 0..rw {
            let px = x + xx;
            let py = y + yy;
            if px < 0 || py < 0 { continue; }
            let px = px as u32;
            let py = py as u32;
            if px >= w || py >= h { continue; }
            let i = ((py * w + px) * 4) as usize;
            if i + 3 >= buf.len() { continue; }
            buf[i] = color[0]; buf[i+1] = color[1]; buf[i+2] = color[2]; buf[i+3] = color[3];
        }
    }
}

// ── F-35–like aircraft built from primitive shapes ───────────────────────────
//
//  Coordinate convention (aircraft body frame):
//    +Z  = aft  (tail direction from nose)
//    +X  = starboard (right wing)
//    +Y  = up (dorsal)
//
//  All dimensions are in rough metres scaled for visual clarity.

fn spawn_aircraft(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    position: Vec3,
) {
    // Shared material handles
    let body_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.53, 0.55, 0.58),
        metallic: 0.4,
        perceptual_roughness: 0.5,
        ..default()
    });
    let dark_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.18, 0.18, 0.20),
        perceptual_roughness: 0.8,
        ..default()
    });
    let canopy_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.25, 0.55, 0.75, 0.55),
        metallic: 0.6,
        perceptual_roughness: 0.1,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let exhaust_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.08, 0.08, 0.10),
        metallic: 0.9,
        perceptual_roughness: 0.3,
        ..default()
    });

    // Helper: add a Cuboid child
    macro_rules! cub {
        ($parent:expr, $sx:expr, $sy:expr, $sz:expr, $tx:expr, $ty:expr, $tz:expr, $ry:expr, $mat:expr) => {
            $parent.spawn((
                Mesh3d(meshes.add(Cuboid::new($sx, $sy, $sz))),
                MeshMaterial3d($mat),
                Transform {
                    translation: Vec3::new($tx, $ty, $tz),
                    rotation: Quat::from_rotation_y($ry),
                    ..default()
                },
            ));
        };
        ($parent:expr, $sx:expr, $sy:expr, $sz:expr, $tx:expr, $ty:expr, $tz:expr, $mat:expr) => {
            $parent.spawn((
                Mesh3d(meshes.add(Cuboid::new($sx, $sy, $sz))),
                MeshMaterial3d($mat),
                Transform::from_xyz($tx, $ty, $tz),
            ));
        };
    }

    // Bounding box that broadly covers the F-35 model:
    //   wingspan ≈ 11 m  |  height ≈ 3.5 m  |  length ≈ 11 m
    commands
        .spawn((
            Transform::from_translation(position)
                .with_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
            Visibility::default(),
            Aircraft,
            RigidBody::Dynamic,
            Collider::cuboid(11.0, 3.5, 11.0),
            // F-35A combat-weight mass: 38 750 lbf / 32.174 ft/s² ≈ 17 576 kg
            Mass(17_576.0_f32),
            LinearVelocity(Vec3::new(0.0, 0.0, 100.0)), // heading south (+Z)
            ExternalForce::default().with_persistence(false),
            ExternalTorque::default().with_persistence(false),
        ))
        .with_children(|p| {
            // ── Fuselage (main spine) ──────────────────────────────────────
            // Central body: long, wide-ish, shallow oval approximated by box
            cub!(p, 1.1, 0.75, 7.0, 0.0, 0.0, 0.0, body_mat.clone());

            // Forward chine / nose section (tapers forward)
            cub!(p, 0.78, 0.52, 2.2, 0.0, -0.06, -4.1, body_mat.clone());

            // Nose tip
            cub!(p, 0.42, 0.36, 0.9, 0.0, -0.10, -5.1, body_mat.clone());

            // Spine hump behind canopy (avionics bay)
            cub!(p, 0.90, 0.28, 2.0, 0.0, 0.50, -1.0, body_mat.clone());

            // ── DSI (Diverterless Supersonic Inlet) under nose ─────────────
            // Inlet duct – ventral, slightly forward of wing root
            cub!(p, 0.82, 0.32, 1.8, 0.0, -0.52, -2.4, dark_mat.clone());

            // ── Blended wing-body – inner wing / LERX ─────────────────────
            // Lifting body blend (wide flat section merging into wing root)
            cub!(p, 4.0, 0.18, 4.5, 0.0, -0.18, 0.5, body_mat.clone());

            // ── Delta wings ────────────────────────────────────────────────
            // Main delta panel (centred slightly aft, very thin)
            cub!(p, 8.8, 0.09, 3.8, 0.0, -0.22, 1.2, body_mat.clone());

            // Swept leading-edge extension – starboard
            cub!(p, 3.2, 0.07, 2.0, 3.6, -0.22, -0.6, -0.52, body_mat.clone());
            // Swept leading-edge extension – port
            cub!(p, 3.2, 0.07, 2.0, -3.6, -0.22, -0.6, 0.52, body_mat.clone());

            // Trailing-edge control surfaces (elevons) – starboard
            cub!(p, 2.6, 0.06, 0.85, 3.5, -0.23, 2.6, body_mat.clone());
            // – port
            cub!(p, 2.6, 0.06, 0.85, -3.5, -0.23, 2.6, body_mat.clone());

            // ── Horizontal tail (all-moving stabilisers) ───────────────────
            cub!(p, 3.2, 0.08, 1.25, 0.0, -0.12, 3.4, body_mat.clone());

            // ── Vertical tail (single, canted 2° – approximate as straight) ─
            cub!(p, 0.14, 2.2, 1.8, 0.0, 1.05, 2.8, body_mat.clone());

            // ── Canopy ─────────────────────────────────────────────────────
            cub!(p, 0.72, 0.38, 1.6, 0.0, 0.55, -1.9, canopy_mat.clone());

            // ── Engine nozzle (circular, aft) ──────────────────────────────
            p.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.38, 0.6))),
                MeshMaterial3d(exhaust_mat.clone()),
                Transform {
                    translation: Vec3::new(0.0, 0.05, 3.95),
                    rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
                    ..default()
                },
            ));

            // Engine nozzle inner (darker bore)
            p.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.28, 0.3))),
                MeshMaterial3d(dark_mat.clone()),
                Transform {
                    translation: Vec3::new(0.0, 0.05, 4.25),
                    rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
                    ..default()
                },
            ));

            // ── Landing gear stubs (3-point) ───────────────────────────────
            // Nose gear
            p.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.14, 0.5))),
                MeshMaterial3d(dark_mat.clone()),
                Transform::from_xyz(0.0, -0.87, -3.2),
            ));
            // Main gear – starboard
            p.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.18, 0.55))),
                MeshMaterial3d(dark_mat.clone()),
                Transform::from_xyz(1.6, -0.9, 0.8),
            ));
            // Main gear – port
            p.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.18, 0.55))),
                MeshMaterial3d(dark_mat.clone()),
                Transform::from_xyz(-1.6, -0.9, 0.8),
            ));
        });
}

// ── Following camera ─────────────────────────────────────────────────────────
//
// Sits 35 m behind and 8 m above the aircraft in its local frame, looking
// toward a point 50 m ahead of the nose.

fn follow_camera(
    aircraft_q: Query<&Transform, With<Aircraft>>,
    mut camera_q: Query<&mut Transform, (With<FollowCamera>, Without<Aircraft>)>,
) {
    let (Ok(ac), Ok(mut cam)) = (aircraft_q.get_single(), camera_q.get_single_mut()) else {
        return;
    };

    // Aircraft nose points –Z in Bevy; "behind" is +Z in local space.
    let cam_offset = ac.rotation * Vec3::new(0.0, 8.0, 35.0);
    let look_target = ac.translation + ac.rotation * Vec3::new(0.0, 1.0, -50.0);

    cam.translation = ac.translation + cam_offset;
    cam.look_at(look_target, Vec3::Y);
}

// ── Joystick controls (Thrustmaster T16000M) ─────────────────────────────────
//
// Axis mapping (gilrs / Linux SDL2 database for T16000M):
//   LeftStickX  → aileron  (stick roll,  right = +)
//   LeftStickY  → elevator (stick pitch, pull back = nose up)
//   RightZ      → rudder   (stick twist, right = +)
//   LeftZ       → throttle slider (-1 = full forward/max, +1 = back/idle)
//                 mapped to [0, 1]: throttle = (1 − raw) / 2
//
// When any gamepad is connected it has exclusive control of the stick axes;
// mouse / WASD still control the stick only if no joystick is present.
// Scroll wheel throttle works regardless.

#[derive(Resource, Default)]
struct JoystickActive(bool);

fn joystick_controls(
    gamepads: Query<&Gamepad>,
    mut controls: ResMut<PilotControls>,
    mut js_active: ResMut<JoystickActive>,
) {
    const MAX_AIL_DEF:  f64 = 0.3491; // ≈ 20° aileron
    const MAX_ELEV_DEF: f64 = 0.1745; // ≈ 10° elevator
    const MAX_RUD_DEF:  f64 = 0.2618; // ≈ 15° rudder
    const DEADZONE: f32 = 0.03;

    js_active.0 = false;

    let Some(gamepad) = gamepads.iter().next() else { return; };
    js_active.0 = true;

    // Helper: apply deadzone and scale to [-max, max].
    let scale = |raw: f32, max: f64| -> f64 {
        let v = if raw.abs() < DEADZONE { 0.0_f32 } else { raw };
        (v as f64 * max).clamp(-max, max)
    };

    if let Some(v) = gamepad.get(GamepadAxis::LeftStickX) {
        controls.aileron = scale(v, MAX_AIL_DEF);
    }
    // LeftStickY: pull back = nose up (positive elevator).
    if let Some(v) = gamepad.get(GamepadAxis::LeftStickY) {
        controls.elevator = scale(v, MAX_ELEV_DEF);
    }
    // Twist axis on T16000M maps to RightZ via gilrs SDL2 db.
    if let Some(v) = gamepad.get(GamepadAxis::RightZ) {
        controls.rudder = scale(-v, MAX_RUD_DEF);
    }
    // Throttle slider: LeftZ, range -1 (max) to +1 (idle).
    if let Some(v) = gamepad.get(GamepadAxis::LeftZ) {
        controls.throttle = ((1.0 - v) / 2.0) as f64;
    }
}

// ── Mouse controls ────────────────────────────────────────────────────────────
//
// When the cursor is inside the stick indicator box its position maps directly
// to elevator (Y) and aileron (X) deflection.  Moving outside centres the stick.
// Scroll wheel adjusts throttle.  Escape quits.

fn mouse_controls(
    mut mouse_wheel: EventReader<MouseWheel>,
    mut controls: ResMut<PilotControls>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    window_q: Query<&Window>,
    mut app_exit: EventWriter<AppExit>,
    js_active: Res<JoystickActive>,
) {
    const THROTTLE_STEP: f64 = 0.05;
    // Stick box layout (matches setup_hud): right:24, bottom:40, 80×80 px.
    const BOX_W: f32 = 80.0;
    const BOX_H: f32 = 80.0;
    const BOX_RIGHT: f32 = 24.0;
    const BOX_BOTTOM: f32 = 40.0;
    const MAX_AIL_DEF:  f64 = 0.3491; // ≈ 20° aileron  — matches FDM AIL_MAX_DEG
    const MAX_ELEV_DEF: f64 = 0.1745; // ≈ 10° elevator — comfortable pitch authority

    if keys.just_pressed(KeyCode::Escape) {
        app_exit.send(AppExit::Success);
    }

    // Joystick owns all axes when connected — skip mouse/WASD input.
    if js_active.0 { return; }

    for ev in mouse_wheel.read() {
        controls.throttle =
            (controls.throttle + ev.y as f64 * THROTTLE_STEP).clamp(0.0, 1.0);
    }

    // WASD: full-deflection digital stick input.
    // W/S = pitch up/down; A/D = roll left/right.
    let mut kb_ail:  f64 = 0.0;
    let mut kb_elev: f64 = 0.0;
    if keys.pressed(KeyCode::KeyA) { kb_ail  -= MAX_AIL_DEF; }
    if keys.pressed(KeyCode::KeyD) { kb_ail  += MAX_AIL_DEF; }
    if keys.pressed(KeyCode::KeyW) { kb_elev += MAX_ELEV_DEF; }
    if keys.pressed(KeyCode::KeyS) { kb_elev -= MAX_ELEV_DEF; }
    let kb_active = kb_ail != 0.0 || kb_elev != 0.0;

    // Drive the stick from cursor position inside the box only when LMB is held.
    if mouse_buttons.pressed(MouseButton::Left) {
        if let Ok(window) = window_q.get_single() {
            if let Some(cursor) = window.cursor_position() {
                let w = window.width();
                let h = window.height();
                // Box top-left corner in screen coords.
                let box_x = w - BOX_RIGHT - BOX_W;
                let box_y = h - BOX_BOTTOM - BOX_H;
                let rel_x = cursor.x - box_x;
                let rel_y = cursor.y - box_y;
                if rel_x >= 0.0 && rel_x <= BOX_W && rel_y >= 0.0 && rel_y <= BOX_H {
                    // Normalize to [-1, 1]; Y is inverted (screen down = pitch nose down).
                    let nx = ((rel_x / BOX_W) * 2.0 - 1.0) as f64;
                    let ny = -(((rel_y / BOX_H) * 2.0 - 1.0) as f64);
                    controls.aileron  = (nx * MAX_AIL_DEF).clamp(-MAX_AIL_DEF, MAX_AIL_DEF);
                    controls.elevator = (ny * MAX_ELEV_DEF).clamp(-MAX_ELEV_DEF, MAX_ELEV_DEF);
                    return;
                }
            }
        }
    }

    if kb_active {
        controls.aileron  = kb_ail;
        controls.elevator = kb_elev;
    } else {
        // Neither mouse nor keyboard — centre the stick.
        controls.elevator = 0.0;
        controls.aileron  = 0.0;
    }
}

// ── HUD ──────────────────────────────────────────────────────────────────────
//
// Layout (all positions in pixels from edges):
//   Top-center  : compass heading text
//   Bottom-left : throttle bar (vertical, fill from bottom)
//   Bottom-right: stick indicator (dot in 80×80 box)

fn setup_hud(mut commands: Commands) {
    // Full-screen transparent root.
    commands
        .spawn(Node {
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            ..default()
        })
        .with_children(|root| {
            // ── Compass ───────────────────────────────────────────────────
            root.spawn((
                Text::new("HDG 000°"),
                TextFont { font_size: 20.0, ..default() },
                TextColor(Color::srgba(0.1, 1.0, 0.4, 0.95)),
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(14.0),
                    left: Val::Percent(50.0),
                    ..default()
                },
                CompassDisplay,
            ));

            // ── Altimeter ───────────────────────────────────────────────────────────────
            root.spawn((
                Text::new("ALT 00000 ft"),
                TextFont { font_size: 20.0, ..default() },
                TextColor(Color::srgba(0.1, 1.0, 0.4, 0.95)),
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(40.0),
                    left: Val::Percent(50.0),
                    ..default()
                },
                AltimeterDisplay,
            ));

            // ── Airspeed indicator ─────────────────────────────────────────
            root.spawn((
                Text::new("IAS   0 kt"),
                TextFont { font_size: 20.0, ..default() },
                TextColor(Color::srgba(0.1, 1.0, 0.4, 0.95)),
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(66.0),
                    left: Val::Percent(50.0),
                    ..default()
                },
                AirspeedDisplay,
            ));

            // ── Throttle bar container ─────────────────────────────────────
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(40.0),
                    left: Val::Px(24.0),
                    width: Val::Px(22.0),
                    height: Val::Px(120.0),
                    border: UiRect::all(Val::Px(1.0)),
                    flex_direction: FlexDirection::ColumnReverse,
                    overflow: Overflow::clip(),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
                BorderColor(Color::srgba(0.1, 1.0, 0.4, 0.7)),
            ))
            .with_children(|bar| {
                bar.spawn((
                    Node {
                        width: Val::Percent(100.0),
                        height: Val::Percent(60.0), // updated each frame
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.05, 0.85, 0.25, 0.9)),
                    ThrottleFill,
                ));
            });

            // "THR" label below the bar
            root.spawn((
                Text::new("THR"),
                TextFont { font_size: 13.0, ..default() },
                TextColor(Color::srgba(0.1, 1.0, 0.4, 0.85)),
                Node {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(24.0),
                    left: Val::Px(22.0),
                    ..default()
                },
            ));

            // ── Stick indicator container ──────────────────────────────────
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(40.0),
                    right: Val::Px(24.0),
                    width: Val::Px(80.0),
                    height: Val::Px(80.0),
                    border: UiRect::all(Val::Px(1.0)),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
                BorderColor(Color::srgba(0.1, 1.0, 0.4, 0.7)),
            ))
            .with_children(|stick| {
                // Horizontal crosshair line
                stick.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(0.0),
                        top: Val::Px(38.0),
                        width: Val::Percent(100.0),
                        height: Val::Px(1.0),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.1, 1.0, 0.4, 0.25)),
                ));
                // Vertical crosshair line
                stick.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        top: Val::Px(0.0),
                        left: Val::Px(38.0),
                        width: Val::Px(1.0),
                        height: Val::Percent(100.0),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.1, 1.0, 0.4, 0.25)),
                ));
                // Stick dot (8×8 px)
                stick.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        width: Val::Px(8.0),
                        height: Val::Px(8.0),
                        left: Val::Px(36.0),
                        top: Val::Px(36.0),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(1.0, 0.3, 0.05, 0.95)),
                    StickDot,
                ));
            });

            // "STICK" label below the box
            root.spawn((
                Text::new("STICK"),
                TextFont { font_size: 13.0, ..default() },
                TextColor(Color::srgba(0.1, 1.0, 0.4, 0.85)),
                Node {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(24.0),
                    right: Val::Px(28.0),
                    ..default()
                },
            ));
        });
}

fn update_hud(
    controls: Res<PilotControls>,
    aircraft_q: Query<(&Transform, &LinearVelocity), With<Aircraft>>,
    mut throttle_q: Query<&mut Node, (With<ThrottleFill>, Without<StickDot>)>,
    mut compass_q: Query<&mut Text, (With<CompassDisplay>, Without<AltimeterDisplay>, Without<AirspeedDisplay>)>,
    mut altimeter_q: Query<&mut Text, (With<AltimeterDisplay>, Without<CompassDisplay>, Without<AirspeedDisplay>)>,
    mut airspeed_q: Query<&mut Text, (With<AirspeedDisplay>, Without<CompassDisplay>, Without<AltimeterDisplay>)>,
    mut stick_q: Query<&mut Node, (With<StickDot>, Without<ThrottleFill>)>,
) {
    // Throttle fill height.
    if let Ok(mut node) = throttle_q.get_single_mut() {
        node.height = Val::Percent((controls.throttle * 100.0) as f32);
    }

    if let Ok((xform, vel)) = aircraft_q.get_single() {
        let fwd = xform.rotation * Vec3::NEG_Z;
        let hdg = (f32::atan2(fwd.x, -fwd.z).to_degrees() + 360.0) % 360.0;
        if let Ok(mut text) = compass_q.get_single_mut() {
            **text = format!("HDG {:03.0}°", hdg);
        }
        let alt_ft = xform.translation.y * M_TO_FT;
        if let Ok(mut text) = altimeter_q.get_single_mut() {
            **text = format!("ALT {:5.0} ft", alt_ft);
        }
        // 1 m/s = 1.94384 knots
        let ias_kt = vel.0.length() * 1.943_84;
        if let Ok(mut text) = airspeed_q.get_single_mut() {
            **text = format!("IAS {:3.0} kt", ias_kt);
        }
    }

    // Stick dot position in the 80×80 box (usable range 0–72 with 8 px dot).
    if let Ok(mut node) = stick_q.get_single_mut() {
        let ail  = (controls.aileron  / 0.3491) as f32;
        let elev = (controls.elevator / 0.1745) as f32;
        let cx = (36.0 + ail  * 36.0).clamp(0.0, 72.0);
        let cy = (36.0 - elev * 36.0).clamp(0.0, 72.0);
        node.left = Val::Px(cx);
        node.top  = Val::Px(cy);
    }
}

// ── Telemetry logger ──────────────────────────────────────────────────────────
//
// Prints one row per second to stdout in a format that matches simulate.rs so
// the two runs can be diff-ed directly.
//
// Column definitions to match simulate.rs:
//   t[s]  alt[ft]  vt[fps]  pos_n[ft]  pos_e[ft]
//
// Aircraft starts at Bevy world (0, 500, 2000) m heading −Z (north):
//   pos_n = (2000 − z) × M_TO_FT
//   pos_e =        x   × M_TO_FT

// ── Aerodynamics system ───────────────────────────────────────────────────────
//
// Coordinate mappings (aircraft nose faces –Z in Bevy):
//   FDM body X (forward) ↔ Bevy local –Z
//   FDM body Y (right)   ↔ Bevy local +X
//   FDM body Z (down)    ↔ Bevy local –Y
//
// Velocity:  u =  –v_local.z * M_TO_FT
//            v =   v_local.x * M_TO_FT
//            w =  –v_local.y * M_TO_FT
//
// Rates:     p = –ω_local.z   (roll,  body X = –Z_local)
//            q =  ω_local.x   (pitch, body Y =  X_local)
//            r = –ω_local.y   (yaw,   body Z = –Y_local)
//
// Force body → Bevy local:  (body_y, –body_z, –body_x) * LBF_TO_N
// Torque body → Bevy local: (M, –N, –L) * LBFFT_TO_NM

fn apply_aerodynamics(
    mut query: Query<
        (
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
            &mut ExternalForce,
            &mut ExternalTorque,
        ),
        With<Aircraft>,
    >,
    controls: Res<PilotControls>,
) {
    for (transform, lin_vel, ang_vel, mut ext_force, mut ext_torque) in &mut query {
        let rotation = transform.rotation;
        let inv_rot = rotation.inverse();

        // Transform world-frame velocities into the aircraft's local frame.
        let v_local = inv_rot * lin_vel.0;
        let w_local = inv_rot * ang_vel.0;

        // Convert to FDM body-frame velocities (ft/s).
        let u = (-v_local.z * M_TO_FT) as f64;
        let v = (v_local.x * M_TO_FT) as f64;
        let w = (-v_local.y * M_TO_FT) as f64;

        // Angular rates in FDM body frame (rad/s).
        let p = (-w_local.z) as f64;
        let q = (w_local.x) as f64;
        let r = (-w_local.y) as f64;

        // Altitude (ft) — clamp to sea level so atmo model stays valid on the ground.
        let altitude_ft = (transform.translation.y * M_TO_FT).max(0.0) as f64;
        let atmo = atmosphere(altitude_ft);

        // Aerodynamic forces / moments.
        let aero = aerodynamics(&AeroIn {
            vel_aero: FdmVec3::new(u, v, w),
            pqr: FdmVec3::new(p, q, r),
            alpha_dot: 0.0,
            rho: atmo.density,
            sound_speed: atmo.sound_speed,
            elevator_rad: controls.elevator,
            aileron_rad: controls.aileron,
            rudder_rad: controls.rudder,
        });

        // Propulsion: thrust along body X (forward).
        let mach = aero.vt / atmo.sound_speed;
        let prop = propulsion(controls.throttle, mach, atmo.density);

        // Total body-frame forces (lbf): aero + thrust along X.
        let body_fx = aero.force_body.x + prop.thrust_lbf;
        let body_fy = aero.force_body.y;
        let body_fz = aero.force_body.z;

        // Map to Bevy local frame and convert to N.
        //   Bevy local X ← body Y
        //   Bevy local Y ← –body Z
        //   Bevy local Z ← –body X
        let f_local = Vec3::new(
            body_fy as f32 * LBF_TO_N,
            -body_fz as f32 * LBF_TO_N,
            -body_fx as f32 * LBF_TO_N,
        );

        // Map moments to Bevy local frame and convert to N·m.
        //   Bevy local X ← M (pitch)
        //   Bevy local Y ← –N (yaw)
        //   Bevy local Z ← –L (roll)
        let t_local = Vec3::new(
            aero.moment_body.y as f32 * LBFFT_TO_NM,
            -aero.moment_body.z as f32 * LBFFT_TO_NM,
            -aero.moment_body.x as f32 * LBFFT_TO_NM,
        );

        // Rotate into world frame and apply.
        **ext_force = rotation * f_local;
        **ext_torque = rotation * t_local;
    }
}
