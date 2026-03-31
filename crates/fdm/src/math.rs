//! Minimal linear-algebra types used throughout the FDM.
//!
//! All units are Imperial (ft, slugs, lbf, seconds, radians) to match JSBSim.

use std::ops::{Add, AddAssign, Mul, Neg, Sub};

// ── Vec3 ─────────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }

    pub fn dot(self, rhs: Vec3) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Scale by scalar.
    pub fn scale(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Vec3) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        self.scale(s)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        v.scale(self)
    }
}

// ── Mat3 ─────────────────────────────────────────────────────────────────────

/// Row-major 3×3 matrix.  `m[i][j]` is row *i*, column *j*.
#[derive(Clone, Copy, Debug, Default)]
pub struct Mat3 {
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    pub fn from_rows(r0: Vec3, r1: Vec3, r2: Vec3) -> Self {
        Mat3 {
            m: [
                [r0.x, r0.y, r0.z],
                [r1.x, r1.y, r1.z],
                [r2.x, r2.y, r2.z],
            ],
        }
    }

    /// Rotation about X by angle (rad).
    pub fn rot_x(a: f64) -> Self {
        let (s, c) = a.sin_cos();
        Mat3::from_rows(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, c, -s),
            Vec3::new(0.0, s, c),
        )
    }

    /// Rotation about Y by angle (rad).
    pub fn rot_y(a: f64) -> Self {
        let (s, c) = a.sin_cos();
        Mat3::from_rows(
            Vec3::new(c, 0.0, s),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-s, 0.0, c),
        )
    }

    /// Rotation about Z by angle (rad).
    pub fn rot_z(a: f64) -> Self {
        let (s, c) = a.sin_cos();
        Mat3::from_rows(
            Vec3::new(c, -s, 0.0),
            Vec3::new(s, c, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
    }

    pub fn transpose(self) -> Self {
        let m = self.m;
        Mat3 {
            m: [
                [m[0][0], m[1][0], m[2][0]],
                [m[0][1], m[1][1], m[2][1]],
                [m[0][2], m[1][2], m[2][2]],
            ],
        }
    }

    /// Matrix-vector product.
    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        let m = self.m;
        Vec3 {
            x: m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            y: m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            z: m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        }
    }

    /// Matrix multiplication.
    pub fn mul_mat(self, rhs: Mat3) -> Mat3 {
        let mut out = Mat3::default();
        for i in 0..3 {
            for j in 0..3 {
                out.m[i][j] = (0..3).map(|k| self.m[i][k] * rhs.m[k][j]).sum();
            }
        }
        out
    }
}

// ── NED ↔ Body transform from  Euler angles ──────────────────────────────────

/// Build the Direction Cosine Matrix from NED to body.
///
/// Convention: Z-Y-X (Tait-Bryan) Euler sequence.
/// - psi: heading / yaw  (rot about NED-Z)
/// - theta: pitch         (rot about intermediate Y)
/// - phi: roll            (rot about body X)
///
/// The resulting matrix transforms vectors FROM NED INTO the body frame.
pub fn dcm_ned2body(phi: f64, theta: f64, psi: f64) -> Mat3 {
    // Tnb = Rx(phi) * Ry(theta) * Rz(psi)
    Mat3::rot_x(phi)
        .mul_mat(Mat3::rot_y(theta))
        .mul_mat(Mat3::rot_z(psi))
}

/// Wind-to-body rotation matrix.
///
/// The wind frame has X along the aerodynamic velocity vector.
/// Derive via stability frame as intermediate:
///   Tw2b = Ts2b · Tw2s  =  Ry(alpha) · Rz(beta)
///
/// Result rows:
///   [cosα·cosβ,  −cosα·sinβ,  sinα ]
///   [sinβ,        cosβ,        0   ]
///   [−sinα·cosβ,  sinα·sinβ,  cosα ]
pub fn dcm_wind2body(alpha: f64, beta: f64) -> Mat3 {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    Mat3::from_rows(
        Vec3::new(ca * cb, -ca * sb, sa),
        Vec3::new(sb, cb, 0.0),
        Vec3::new(-sa * cb, sa * sb, ca),
    )
}

// ── Table interpolation ───────────────────────────────────────────────────────

/// 1-D linear interpolation over a table of (x, y) pairs sorted by x.
pub fn interp1(table: &[(f64, f64)], x: f64) -> f64 {
    if x <= table.first().unwrap().0 {
        return table.first().unwrap().1;
    }
    if x >= table.last().unwrap().0 {
        return table.last().unwrap().1;
    }
    for i in 0..table.len() - 1 {
        let (x0, y0) = table[i];
        let (x1, y1) = table[i + 1];
        if x <= x1 {
            let t = (x - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    table.last().unwrap().1
}

/// Bilinear interpolation.
///
/// `row_keys` and `col_keys` are the breakpoints.
/// `values[r][c]` is the value at `(row_keys[r], col_keys[c])`.
pub fn interp2(
    row_keys: &[f64],
    col_keys: &[f64],
    values: &[&[f64]],
    row_x: f64,
    col_x: f64,
) -> f64 {
    // Find row bracket
    let ri = bracket(row_keys, row_x);
    // Find col bracket
    let ci = bracket(col_keys, col_x);

    let r0 = ri;
    let r1 = (ri + 1).min(row_keys.len() - 1);
    let c0 = ci;
    let c1 = (ci + 1).min(col_keys.len() - 1);

    let tr = if row_keys[r1] != row_keys[r0] {
        (row_x - row_keys[r0]) / (row_keys[r1] - row_keys[r0])
    } else {
        0.0
    };
    let tc = if col_keys[c1] != col_keys[c0] {
        (col_x - col_keys[c0]) / (col_keys[c1] - col_keys[c0])
    } else {
        0.0
    };

    let v00 = values[r0][c0];
    let v01 = values[r0][c1];
    let v10 = values[r1][c0];
    let v11 = values[r1][c1];

    let v0 = v00 + tc * (v01 - v00);
    let v1 = v10 + tc * (v11 - v10);
    v0 + tr * (v1 - v0)
}

fn bracket(keys: &[f64], x: f64) -> usize {
    if x <= keys[0] {
        return 0;
    }
    if x >= keys[keys.len() - 1] {
        return keys.len() - 1 - 1; // keep r1 = r0+1 valid
    }
    for i in 0..keys.len() - 1 {
        if x <= keys[i + 1] {
            return i;
        }
    }
    keys.len() - 2
}
