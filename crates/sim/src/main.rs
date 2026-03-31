use avian3d::prelude::*;
use bevy::input::mouse::MouseWheel;
use bevy::prelude::*;
use fdm::atmo::atmosphere;
use fdm::f35::aero::{aerodynamics, AeroIn};
use fdm::f35::prop::propulsion;
use fdm::math::Vec3 as FdmVec3;
use terrain::TerrainPlugin;

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
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(TerrainPlugin)
        .add_plugins(SkyPlugin)
        .insert_resource(ClearColor(Color::BLACK))
        .init_resource::<PilotControls>()
        .init_resource::<SimClock>()
        .insert_resource(AmbientLight {
            // Dim, slightly blue-warm: scattered morning skylight.
            color: Color::srgb(0.65, 0.72, 0.90),
            brightness: 180.0,
        })
        .add_systems(Startup, (setup, setup_hud))
        .add_systems(PhysicsSchedule, apply_aerodynamics.in_set(PhysicsStepSet::First))
        .add_systems(Update, (mouse_controls, follow_camera, update_hud, log_telemetry))
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

// ── Simulation clock ─────────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SimClock {
    elapsed:   f64,
    next_log:  f64,
    logged_header: bool,
}

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
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            near: 0.5,
            far:  5_000_000.0,
            ..default()
        }),
        Transform::from_xyz(-30.0, 20.0, 350.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
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

    // ── Runway ────────────────────────────────────────────────────────────
    // Main asphalt strip (30 m wide × 1 000 m long, along Z)
    let asphalt_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.22, 0.22),
        perceptual_roughness: 0.95,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(30.0, 1_000.0))),
        MeshMaterial3d(asphalt_mat),
        Transform::from_xyz(0.0, 0.001, 0.0),
    ));

    // Centre-line dashes
    let white_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        ..default()
    });
    for i in -24..=24 {
        commands.spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(1.0, 8.0))),
            MeshMaterial3d(white_mat.clone()),
            Transform::from_xyz(0.0, 0.003, i as f32 * 20.0),
        ));
    }

    // Threshold bars (both ends)
    for &z_sign in &[-1.0_f32, 1.0] {
        for stripe in -3..=3 {
            commands.spawn((
                Mesh3d(meshes.add(Plane3d::default().mesh().size(3.0, 3.0))),
                MeshMaterial3d(white_mat.clone()),
                Transform::from_xyz(stripe as f32 * 4.0, 0.003, z_sign * 495.0),
            ));
        }
    }

    // ── Aircraft ──────────────────────────────────────────────────────────
    // Start 2 km from the runway centre at 500 m altitude, heading north.
    spawn_aircraft(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, 500.0, 2000.0),
    );
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
            Transform::from_translation(position),
            Visibility::default(),
            Aircraft,
            RigidBody::Dynamic,
            Collider::cuboid(11.0, 3.5, 11.0),
            // F-35A combat-weight mass: 38 750 lbf / 32.174 ft/s² ≈ 17 576 kg
            Mass(17_576.0_f32),
            LinearVelocity(Vec3::new(0.0, 0.0, -100.0)),
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
) {
    const THROTTLE_STEP: f64 = 0.05;
    // Stick box layout (matches setup_hud): right:24, bottom:40, 80×80 px.
    const BOX_W: f32 = 80.0;
    const BOX_H: f32 = 80.0;
    const BOX_RIGHT: f32 = 24.0;
    const BOX_BOTTOM: f32 = 40.0;
    const MAX_DEF: f64 = 0.0872; // ≈ 5° max deflection (rad) — 5× reduced sensitivity

    if keys.just_pressed(KeyCode::Escape) {
        app_exit.send(AppExit::Success);
    }

    for ev in mouse_wheel.read() {
        controls.throttle =
            (controls.throttle + ev.y as f64 * THROTTLE_STEP).clamp(0.0, 1.0);
    }

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
                    controls.aileron  = (nx * MAX_DEF).clamp(-MAX_DEF, MAX_DEF);
                    controls.elevator = (ny * MAX_DEF).clamp(-MAX_DEF, MAX_DEF);
                    return;
                }
            }
        }
    }

    // LMB not held, or cursor outside box — centre the stick.
    controls.elevator = 0.0;
    controls.aileron  = 0.0;
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
        const MAX_DEF: f64 = 0.0872;
        let ail  = (controls.aileron  / MAX_DEF) as f32;
        let elev = (controls.elevator / MAX_DEF) as f32;
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

fn log_telemetry(
    time: Res<Time>,
    mut clock: ResMut<SimClock>,
    query: Query<(&Transform, &LinearVelocity), With<Aircraft>>,
) {
    clock.elapsed += time.delta_secs_f64();

    let Ok((xform, vel)) = query.get_single() else { return; };

    if !clock.logged_header {
        println!(
            "{:>7}  {:>9}  {:>9}  {:>10}  {:>10}  (bevy)",
            "t[s]", "alt[ft]", "vt[fps]", "pos_n[ft]", "pos_e[ft]"
        );
        clock.logged_header = true;
    }

    if clock.elapsed >= clock.next_log {
        let alt_ft  = (xform.translation.y * M_TO_FT) as f64;
        let vt_fps  = (vel.0.length() * M_TO_FT) as f64;
        let pos_n   = ((2000.0 - xform.translation.z) * M_TO_FT) as f64;
        let pos_e   = (xform.translation.x * M_TO_FT) as f64;
        println!(
            "{:7.1}  {:9.1}  {:9.2}  {:10.1}  {:10.1}",
            clock.elapsed, alt_ft, vt_fps, pos_n, pos_e,
        );
        clock.next_log += 1.0;
    }
}

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
