//! Procedural atmospheric sky – simplified Rayleigh + Mie single-scatter model.
//!
//! A UV-sphere skydome is built at startup with per-vertex colours derived from
//! a physically-motivated scattering formula.  The dome and the sun disc both
//! follow the camera every frame so they always surround the viewer.
//!
//! Time of day is early morning: sun ~8° above the eastern (+X) horizon.

use std::f32::consts::PI;

use bevy::{
    pbr::{NotShadowCaster, NotShadowReceiver},
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};

// ── Sun direction ─────────────────────────────────────────────────────────────
//
// Direction FROM the scene origin TOWARD the sun.
// NNE direction, ~15° above horizon – squarely in the initial camera view
// (camera starts looking north along −Z) and low enough for morning colour.
pub const SUN_DIR: Vec3 = Vec3::new(0.30, 0.27, -0.92);

// ── Dome geometry ─────────────────────────────────────────────────────────────
const SKY_RADIUS: f32 = 50_000.0;  // 50 km – stays well within float depth precision
const RINGS:    usize = 48;
const SECTORS:  usize = 96;

// ── Sun disc ──────────────────────────────────────────────────────────────────
// 3° angular radius (real sun is 0.27°; enlarged for clear in-game visibility).
const SUN_ANG_DEG:  f32 = 3.0;
// Place disc at 0.90× dome radius so it is clearly in front of the dome surface.
const SUN_DIST:     f32 = SKY_RADIUS * 0.90;
const SUN_DISC_R:   f32 = SUN_DIST * (SUN_ANG_DEG * PI / 180.0);
const SUN_SECTORS:  usize = 64;

// ── Component markers ─────────────────────────────────────────────────────────
#[derive(Component)]
struct SkydomeMarker;

#[derive(Component)]
struct SunDiscMarker;

// ── Plugin ────────────────────────────────────────────────────────────────────
pub struct SkyPlugin;

impl Plugin for SkyPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_sky)
            .add_systems(Update, follow_sky);
    }
}

// ── Scattering model ──────────────────────────────────────────────────────────
//
// Single-scatter Rayleigh + Mie model evaluated per vertex.
//
// Rayleigh:  I ∝ (1 + cos²θ) / λ⁴
//   Wavelengths: R = 700 nm, G = 550 nm, B = 440 nm
//   Relative λ⁻⁴:  1 / 0.70⁴ : 1 / 0.55⁴ : 1 / 0.44⁴  ≈  4.2 : 10.9 : 26.7
//   Normalised to R = 1:  1.0 : 2.6 : 6.35
const RAY_R: f32 = 1.0;
const RAY_G: f32 = 2.6;
const RAY_B: f32 = 6.35;

// Rayleigh phase function (unnormalised, folding constant into scatter_strength).
#[inline]
fn rayleigh_phase(cos_t: f32) -> f32 {
    0.75 * (1.0 + cos_t * cos_t)
}

// Mie phase function (Henyey-Greenstein), g ≈ 0.76 for aerosols.
#[inline]
fn mie_phase(cos_t: f32) -> f32 {
    const G: f32 = 0.76;
    const G2: f32 = G * G;
    let denom = (1.0 + G2 - 2.0 * G * cos_t).powf(1.5);
    (1.0 - G2) / ((2.0 + G2) * denom) * (1.0 + cos_t * cos_t)
}

// Simplified optical depth along direction `dir` through an exponential
// atmosphere.  Larger near the horizon than at zenith.
#[inline]
fn optical_depth(dir: Vec3) -> f32 {
    1.0 / (dir.y.max(0.0) + 0.035)
}

// Linear interpolation helper.
#[inline]
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

/// Compute the sky colour for a vertex whose outward direction is `dir`.
///
/// Returns linear `[r, g, b, 1.0]` in a range suitable for `ATTRIBUTE_COLOR`.
fn sky_color(dir: Vec3) -> [f32; 4] {
    let dir = dir.normalize();
    let sun = SUN_DIR.normalize();
    let cos_t = dir.dot(sun).clamp(-1.0, 1.0);

    let ray   = rayleigh_phase(cos_t);
    let mie_v = mie_phase(cos_t);

    // Optical depth along the view ray.
    let od_view = optical_depth(dir);

    // Optical depth along the sun ray (path the sunlight travels before
    // being scattered toward the viewer).  This is what makes the horizon
    // orange at sunrise: the long, near-horizontal sun path attenuates blue.
    let od_sun  = optical_depth(sun) * 2.5;

    // Per-channel transmittance of incoming sunlight.
    let sun_r = (-RAY_R * 0.07 * od_sun).exp();
    let sun_g = (-RAY_G * 0.07 * od_sun).exp();
    let sun_b = (-RAY_B * 0.07 * od_sun).exp();

    // Rayleigh scatter.
    const RS: f32 = 0.7; // overall Rayleigh strength
    let rr = RAY_R * ray * RS * sun_r;
    let rg = RAY_G * ray * RS * sun_g;
    let rb = RAY_B * ray * RS * sun_b;

    // Mie scatter (white-ish halo near the sun).
    const MS: f32 = 0.08;
    let mr = mie_v * MS * sun_r;
    let mg = mie_v * MS * sun_g;
    let mb = mie_v * MS * sun_b;

    // Combined scatter, attenuated along the view path.
    let tr = (-RAY_R * 0.07 * od_view).exp();
    let tg = (-RAY_G * 0.07 * od_view).exp();
    let tb = (-RAY_B * 0.07 * od_view).exp();

    let mut r = (rr + mr) * tr;
    let mut g = (rg + mg) * tg;
    let mut b = (rb + mb) * tb;

    // Fade to black below the horizon.
    let fade = (dir.y * 50.0 + 1.0).clamp(0.0, 1.0);
    r *= fade;
    g *= fade;
    b *= fade;

    // Horizon ground-glow: a thin warm band just below the horizon adds warm
    // pre-dawn colour bleed without requiring sub-horizon geometry.
    let glow = (1.0 - (dir.y * 30.0).abs().clamp(0.0, 1.0)) * 0.25;
    r += glow * 0.9 * sun_r;
    g += glow * 0.45 * sun_g;
    b += glow * 0.08 * sun_b;

    // Reinhard tone-map so values stay in [0, 1].
    let lum = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    let tm = if lum > 0.001 { 1.0 / (lum + 1.0) } else { 1.0 };
    let scale = lerp(tm, 1.0, 0.35); // partial tone-map for better saturation
    [(r * scale).clamp(0.0, 1.0),
     (g * scale).clamp(0.0, 1.0),
     (b * scale).clamp(0.0, 1.0),
     1.0]
}

// ── Mesh builders ─────────────────────────────────────────────────────────────

fn build_skydome_mesh() -> Mesh {
    let total = (RINGS + 1) * (SECTORS + 1);
    let mut positions = Vec::with_capacity(total);
    let mut colors    = Vec::with_capacity(total);
    let mut normals   = Vec::with_capacity(total);
    let mut uvs       = Vec::with_capacity(total);

    for r in 0..=RINGS {
        let phi = PI * r as f32 / RINGS as f32;
        let sp = phi.sin();
        let cp = phi.cos();
        for s in 0..=SECTORS {
            let theta = 2.0 * PI * s as f32 / SECTORS as f32;
            let x = sp * theta.cos();
            let y = cp;
            let z = sp * theta.sin();
            let dir = Vec3::new(x, y, z);
            positions.push([x * SKY_RADIUS, y * SKY_RADIUS, z * SKY_RADIUS]);
            colors.push(sky_color(dir));
            normals.push([-x, -y, -z]);
            uvs.push([s as f32 / SECTORS as f32, r as f32 / RINGS as f32]);
        }
    }

    let mut indices: Vec<u32> = Vec::new();
    let w = (SECTORS + 1) as u32;
    for r in 0..RINGS as u32 {
        for s in 0..SECTORS as u32 {
            let tl = r * w + s;
            let tr = tl + 1;
            let bl = tl + w;
            let br = bl + 1;
            indices.extend_from_slice(&[tl, tr, bl]);
            indices.extend_from_slice(&[tr, br, bl]);
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

fn build_sun_disc_mesh() -> Mesh {
    let fwd   = SUN_DIR.normalize();
    let right = Vec3::Y.cross(fwd).normalize();
    let up    = fwd.cross(right).normalize();
    let n = SUN_SECTORS;
    let r = SUN_DISC_R;

    let mut positions = Vec::with_capacity(n + 1);
    let mut colors    = Vec::with_capacity(n + 1);
    let mut normals   = Vec::with_capacity(n + 1);
    let mut uvs       = Vec::with_capacity(n + 1);

    let inward = [-fwd.x, -fwd.y, -fwd.z]; // normal facing the viewer inside the dome

    // Centre.
    positions.push([0.0_f32, 0.0, 0.0]);
    colors.push([1.0_f32, 0.96, 0.80, 1.0]); // warm solar white
    normals.push(inward);
    uvs.push([0.5_f32, 0.5]);

    for i in 0..n {
        let angle = 2.0 * PI * i as f32 / n as f32;
        let p = (right * angle.cos() + up * angle.sin()) * r;
        positions.push([p.x, p.y, p.z]);
        colors.push([1.0, 0.85, 0.60, 1.0]); // limb slightly dimmer + warmer (sunrise)
        normals.push(inward);
        uvs.push([angle.cos() * 0.5 + 0.5, angle.sin() * 0.5 + 0.5]);
    }

    let mut indices: Vec<u32> = Vec::new();
    for i in 0..n as u32 {
        let next = (i + 1) % n as u32;
        indices.extend_from_slice(&[0, i + 1, next + 1]);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL,   normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR,    colors);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0,     uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

// ── Systems ───────────────────────────────────────────────────────────────────

fn setup_sky(
    mut commands: Commands,
    mut meshes:   ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Skydome: unlit, cull disabled, vertex colours carry the Rayleigh/Mie sky.
    let sky_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        double_sided: true,
        cull_mode: None,
        fog_enabled: false,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(build_skydome_mesh())),
        MeshMaterial3d(sky_mat),
        Transform::default(),
        SkydomeMarker,
        NotShadowCaster,
        NotShadowReceiver,
    ));

    // Sun disc: bright, unlit, no culling so it's visible from both sides.
    let sun_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.92, 0.65),
        unlit: true,
        double_sided: true,
        fog_enabled: false,
        ..default()
    });

    let sun_pos = SUN_DIR.normalize() * SUN_DIST;
    commands.spawn((
        Mesh3d(meshes.add(build_sun_disc_mesh())),
        MeshMaterial3d(sun_mat),
        Transform::from_translation(sun_pos),
        SunDiscMarker,
        NotShadowCaster,
        NotShadowReceiver,
    ));
}

/// Centre the skydome and sun disc on the camera every frame.
fn follow_sky(
    camera_q: Query<&GlobalTransform, With<Camera3d>>,
    mut dome_q: Query<&mut Transform, (With<SkydomeMarker>, Without<SunDiscMarker>)>,
    mut sun_q:  Query<&mut Transform, (With<SunDiscMarker>,  Without<SkydomeMarker>)>,
) {
    let Ok(cam_gt) = camera_q.get_single() else { return };
    let cam = cam_gt.translation();

    for mut t in dome_q.iter_mut() {
        t.translation = cam;
    }
    for mut t in sun_q.iter_mut() {
        t.translation = cam + SUN_DIR.normalize() * SUN_DIST;
    }
}
