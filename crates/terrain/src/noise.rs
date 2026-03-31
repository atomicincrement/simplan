// 3-D gradient (Perlin) noise — Ken Perlin's improved noise, scalar output in [-1, 1].
//
// Reference: "Improving Noise" (K. Perlin, 2002).

/// 256-entry permutation table (two copies to avoid bounds checks).
const PERM: [u8; 512] = {
    const P: [u8; 256] = [
        151, 160, 137,  91,  90,  15, 131,  13, 201,  95,  96,  53, 194, 233,   7, 225,
        140,  36, 103,  30,  69, 142,   8,  99,  37, 240,  21,  10,  23, 190,   6, 148,
        247, 120, 234,  75,   0,  26, 197,  62,  94, 252, 219, 203, 117,  35,  11,  32,
         57, 177,  33,  88, 237, 149,  56,  87, 174,  20, 125, 136, 171, 168,  68, 175,
         74, 165,  71, 134, 139,  48,  27, 166,  77, 146, 158, 231,  83, 111, 229, 122,
         60, 211, 133, 230, 220, 105,  92,  41,  55,  46, 245,  40, 244, 102, 143,  54,
         65,  25,  63, 161,   1, 216,  80,  73, 209,  76, 132, 187, 208,  89,  18, 169,
        200, 196, 135, 130, 116, 188, 159,  86, 164, 100, 109, 198, 173, 186,   3,  64,
         52, 217, 226, 250, 124, 123,   5, 202,  38, 147, 118, 126, 255,  82,  85, 212,
        207, 206,  59, 227,  47,  16,  58,  17, 182, 189,  28,  42, 223, 183, 170, 213,
        119, 248, 152,   2,  44, 154, 163,  70, 221, 153, 101, 155, 167,  43, 172,   9,
        129,  22,  39, 253,  19,  98, 108, 110,  79, 113, 224, 232, 178, 185, 112, 104,
        218, 246,  97, 228, 251,  34, 242, 193, 238, 210, 144,  12, 191, 179, 162, 241,
         81,  51, 145, 235, 249,  14, 239, 107,  49, 192, 214,  31, 181, 199, 106, 157,
        184,  84, 204, 176, 115, 121,  50,  45, 127,   4, 150, 254, 138, 236, 205,  93,
        222, 114,  67,  29,  24,  72, 243, 141, 128, 195,  78,  66, 215,  61, 156, 180,
    ];
    let mut out = [0u8; 512];
    let mut i = 0usize;
    while i < 256 { out[i] = P[i]; out[i + 256] = P[i]; i += 1; }
    out
};

/// Quintic fade: 6t⁵ − 15t⁴ + 10t³
#[inline(always)]
fn fade(t: f32) -> f32 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

#[inline(always)]
fn lerp(t: f32, a: f32, b: f32) -> f32 {
    a + t * (b - a)
}

/// Map hash to gradient dot product.
#[inline(always)]
fn grad3(hash: u8, x: f32, y: f32, z: f32) -> f32 {
    match hash & 0xF {
        0x0 =>  x + y,
        0x1 => -x + y,
        0x2 =>  x - y,
        0x3 => -x - y,
        0x4 =>  x + z,
        0x5 => -x + z,
        0x6 =>  x - z,
        0x7 => -x - z,
        0x8 =>  y + z,
        0x9 => -y + z,
        0xA =>  y - z,
        0xB => -y - z,
        0xC =>  y + x,
        0xD => -y + z,
        0xE =>  y - x,
        _   => -y - z,
    }
}

/// Classic Perlin 3-D noise. Returns a value in approximately [-1, 1].
pub fn perlin3(x: f32, y: f32, z: f32) -> f32 {
    let xi = x.floor() as i32;
    let yi = y.floor() as i32;
    let zi = z.floor() as i32;

    let xf = x - xi as f32;
    let yf = y - yi as f32;
    let zf = z - zi as f32;

    let u = fade(xf);
    let v = fade(yf);
    let w = fade(zf);

    // Wrap to [0, 255]
    let x0 = (xi & 0xFF) as usize;
    let y0 = (yi & 0xFF) as usize;
    let z0 = (zi & 0xFF) as usize;
    let x1 = (x0 + 1) & 0xFF;
    let y1 = (y0 + 1) & 0xFF;
    let z1 = (z0 + 1) & 0xFF;

    let aaa = PERM[PERM[PERM[x0] as usize + y0] as usize + z0];
    let aba = PERM[PERM[PERM[x0] as usize + y1] as usize + z0];
    let aab = PERM[PERM[PERM[x0] as usize + y0] as usize + z1];
    let abb = PERM[PERM[PERM[x0] as usize + y1] as usize + z1];
    let baa = PERM[PERM[PERM[x1] as usize + y0] as usize + z0];
    let bba = PERM[PERM[PERM[x1] as usize + y1] as usize + z0];
    let bab = PERM[PERM[PERM[x1] as usize + y0] as usize + z1];
    let bbb = PERM[PERM[PERM[x1] as usize + y1] as usize + z1];

    let x1f = xf - 1.0;
    let y1f = yf - 1.0;
    let z1f = zf - 1.0;

    lerp(w,
        lerp(v,
            lerp(u, grad3(aaa, xf,  yf,  zf  ), grad3(baa, x1f, yf,  zf  )),
            lerp(u, grad3(aba, xf,  y1f, zf  ), grad3(bba, x1f, y1f, zf  ))),
        lerp(v,
            lerp(u, grad3(aab, xf,  yf,  z1f), grad3(bab, x1f, yf,  z1f)),
            lerp(u, grad3(abb, xf,  y1f, z1f), grad3(bbb, x1f, y1f, z1f))))
}

/// Fractional Brownian Motion layering `octaves` octaves of `perlin3`.
///
/// - `freq`       : base spatial frequency (noise cycles per unit)
/// - `lacunarity` : frequency multiplier per octave (typically 2.0)
/// - `gain`       : amplitude multiplier per octave (typically 0.5)
///
/// Returns a value in approximately [-1, 1].
pub fn fbm3(x: f32, y: f32, z: f32, freq: f32, octaves: u32, lacunarity: f32, gain: f32) -> f32 {
    let mut value = 0.0f32;
    let mut amplitude = 1.0f32;
    let mut frequency = freq;
    let mut max_amp = 0.0f32;
    for _ in 0..octaves {
        value   += perlin3(x * frequency, y * frequency, z * frequency) * amplitude;
        max_amp += amplitude;
        frequency *= lacunarity;
        amplitude *= gain;
    }
    value / max_amp
}
