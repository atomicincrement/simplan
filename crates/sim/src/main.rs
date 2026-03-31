use avian3d::prelude::*;
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::CursorGrabMode;
use fdm::atmo::atmosphere;
use fdm::f35::aero::{aerodynamics, AeroIn};
use fdm::f35::prop::propulsion;
use fdm::math::Vec3 as FdmVec3;

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
        .init_resource::<PilotControls>()
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 400.0,
        })
        .add_systems(Startup, (setup, grab_cursor))
        .add_systems(Update, (apply_aerodynamics, mouse_controls, follow_camera))
        .run();
}

// ── Marker components ───────────────────────────────────────────────────────

#[derive(Component)]
struct Aircraft;

#[derive(Component)]
struct FollowCamera;

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
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 20.0, 350.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
        FollowCamera,
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
        Vec3::new(0.0, 6.0, 250.0),
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
            ExternalForce::default(),
            ExternalTorque::default(),
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

// ── Cursor grab ──────────────────────────────────────────────────────────────

fn grab_cursor(mut window_q: Query<&mut Window>) {
    if let Ok(mut window) = window_q.get_single_mut() {
        window.cursor_options.grab_mode = CursorGrabMode::Locked;
        window.cursor_options.visible = false;
    }
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
// Mouse Y (up)   → elevator +  (nose up)
// Mouse X (right) → aileron +  (right wing down / roll right)
// Escape releases the cursor grab.

fn mouse_controls(
    mut mouse_motion: EventReader<MouseMotion>,
    mut controls: ResMut<PilotControls>,
    keys: Res<ButtonInput<KeyCode>>,
    mut window_q: Query<&mut Window>,
) {
    const ELEV_SENS: f64 = 0.003;
    const AIL_SENS: f64 = 0.003;
    const MAX_DEF: f64 = 0.436; // ≈ 25 °

    // Escape releases the cursor so the window can be closed.
    if keys.just_pressed(KeyCode::Escape) {
        if let Ok(mut window) = window_q.get_single_mut() {
            window.cursor_options.grab_mode = CursorGrabMode::None;
            window.cursor_options.visible = true;
        }
    }

    for ev in mouse_motion.read() {
        // Invert Y so pulling mouse back raises the nose.
        controls.elevator =
            (controls.elevator - ev.delta.y as f64 * ELEV_SENS).clamp(-MAX_DEF, MAX_DEF);
        controls.aileron =
            (controls.aileron + ev.delta.x as f64 * AIL_SENS).clamp(-MAX_DEF, MAX_DEF);
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
