//! Terrarium elevation tile fetch and decode.
//!
//! Terrarium encodes height as:
//!   height_m = R * 256 + G + B / 256 − 32768
//!
//! Source: https://github.com/tilezen/joerd (AWS Terrain Tiles)

use std::{fmt, io::Read};

/// 256×256 elevation grid decoded from one Terrarium PNG tile.
/// Row-major: index = row * 256 + col, row 0 = north.
pub struct ElevationTile {
    pub heights: Box<[f32; 256 * 256]>,
}

impl ElevationTile {
    /// All-zero (sea level) placeholder used when a fetch fails.
    pub fn flat() -> Self {
        Self { heights: Box::new([0.0_f32; 256 * 256]) }
    }

    /// Bilinear sample at fractional tile coordinates (fx, fy) ∈ [0, 1).
    /// fy=0 = north edge, fy=1 = south edge.
    pub fn sample(&self, fx: f32, fy: f32) -> f32 {
        let px = (fx * 255.999).clamp(0.0, 255.0);
        let py = (fy * 255.999).clamp(0.0, 255.0);
        let ix = px as usize;
        let iy = py as usize;
        let tx = px - ix as f32;
        let ty = py - iy as f32;
        let ix1 = (ix + 1).min(255);
        let iy1 = (iy + 1).min(255);
        let h00 = self.heights[iy  * 256 + ix ];
        let h10 = self.heights[iy  * 256 + ix1];
        let h01 = self.heights[iy1 * 256 + ix ];
        let h11 = self.heights[iy1 * 256 + ix1];
        (h00 + (h10 - h00) * tx) * (1.0 - ty)
            + (h01 + (h11 - h01) * tx) * ty
    }

    /// Serialize to raw little-endian f32 bytes for disk cache.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(256 * 256 * 4);
        for &h in self.heights.iter() {
            out.extend_from_slice(&h.to_le_bytes());
        }
        out
    }

    /// Deserialize from raw bytes (matches `to_bytes`).
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 256 * 256 * 4 {
            return None;
        }
        let mut heights = Box::new([0.0_f32; 256 * 256]);
        for (i, chunk) in bytes.chunks_exact(4).enumerate() {
            heights[i] = f32::from_le_bytes(chunk.try_into().unwrap());
        }
        Some(Self { heights })
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Debug)]
pub struct FetchError(pub String);

impl fmt::Display for FetchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<std::io::Error> for FetchError {
    fn from(e: std::io::Error) -> Self { FetchError(e.to_string()) }
}

// ── Fetch ─────────────────────────────────────────────────────────────────────

/// Download and decode a Terrarium elevation tile.
/// This is a blocking call; run it on a worker thread.
pub fn fetch_elevation_tile(x: i32, y: i32, zoom: u32) -> Result<ElevationTile, FetchError> {
    let url = format!(
        "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{x}/{y}.png"
    );
    let response = ureq::get(&url)
        .call()
        .map_err(|e| FetchError(e.to_string()))?;

    let mut bytes = Vec::new();
    response
        .into_reader()
        .read_to_end(&mut bytes)
        .map_err(|e| FetchError(e.to_string()))?;

    decode_terrarium(&bytes)
}

fn decode_terrarium(png_bytes: &[u8]) -> Result<ElevationTile, FetchError> {
    use png::ColorType;

    let decoder = png::Decoder::new(std::io::Cursor::new(png_bytes));
    let mut reader = decoder
        .read_info()
        .map_err(|e| FetchError(e.to_string()))?;

    let mut buf = vec![0u8; reader.output_buffer_size()];
    let info = reader
        .next_frame(&mut buf)
        .map_err(|e| FetchError(e.to_string()))?;

    let bytes = &buf[..info.buffer_size()];
    let w = info.width as usize;
    let h = info.height as usize;

    if w < 256 || h < 256 {
        return Err(FetchError(format!("tile too small: {}×{}", w, h)));
    }

    let mut heights = Box::new([0.0f32; 256 * 256]);
    match info.color_type {
        ColorType::Rgb => {
            for (i, chunk) in bytes.chunks_exact(3).enumerate().take(256 * 256) {
                heights[i] = chunk[0] as f32 * 256.0
                    + chunk[1] as f32
                    + chunk[2] as f32 / 256.0
                    - 32768.0;
            }
        }
        ColorType::Rgba => {
            for (i, chunk) in bytes.chunks_exact(4).enumerate().take(256 * 256) {
                heights[i] = chunk[0] as f32 * 256.0
                    + chunk[1] as f32
                    + chunk[2] as f32 / 256.0
                    - 32768.0;
            }
        }
        other => {
            return Err(FetchError(format!("unsupported PNG colour type: {other:?}")));
        }
    }

    Ok(ElevationTile { heights })
}
