use avian3d::prelude::*;
use bevy::prelude::*;

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
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 400.0,
        })
        .add_systems(Startup, setup)
        .run();
}

// ── Marker component ────────────────────────────────────────────────────────

#[derive(Component)]
struct Aircraft;

// ── Startup system ──────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ── Camera ────────────────────────────────────────────────────────────
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 20.0, 50.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
    ));

    // ── Sun ───────────────────────────────────────────────────────────────
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            -std::f32::consts::FRAC_PI_4,
            std::f32::consts::FRAC_PI_4,
            0.0,
        )),
    ));

    // ── Ground ────────────────────────────────────────────────────────────
    let ground_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.48, 0.16),
        perceptual_roughness: 1.0,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(800.0, 800.0))),
        MeshMaterial3d(ground_mat),
        Transform::default(),
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
    ));

    // ── Runway ────────────────────────────────────────────────────────────
    // Main asphalt strip (30 m wide × 200 m long, along Z)
    let asphalt_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.22, 0.22),
        perceptual_roughness: 0.95,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(30.0, 200.0))),
        MeshMaterial3d(asphalt_mat),
        Transform::from_xyz(0.0, 0.001, 0.0),
    ));

    // Centre-line dashes
    let white_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        ..default()
    });
    for i in -9..=9 {
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
                Transform::from_xyz(stripe as f32 * 4.0, 0.003, z_sign * 95.0),
            ));
        }
    }

    // ── Aircraft ──────────────────────────────────────────────────────────
    spawn_aircraft(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, 6.0, 0.0),
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
            LinearVelocity(Vec3::new(0.0, 0.0, 100.0)),
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
