//! Satellite imagery tile fetch and decode.
//!
//! Source: ESRI World Imagery – free, no API key required.
//!   URL: https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}
//!
//! Note: ESRI uses {z}/{y}/{x} ordering (y and x swapped vs. Terrarium).
//! Tiles are returned as JPEG and decoded to raw RGBA8 for Bevy.

use std::{fmt, io::Read};

/// 256×256 RGBA satellite image decoded from one Web-Mercator imagery tile.
/// Row-major: pixel (col, row) starts at byte (row * 256 + col) * 4.
/// Row 0 = north edge, row 255 = south edge.
pub struct ImageryTile {
    /// Raw RGBA8 bytes, always exactly 256 * 256 * 4 = 262 144 bytes.
    pub rgba: Vec<u8>,
}

impl ImageryTile {
    /// Mid-grey "no data" placeholder used when a fetch fails.
    pub fn blank() -> Self {
        Self { rgba: vec![128u8; 256 * 256 * 4] }
    }

    /// Borrow as raw bytes for disk persistence.
    pub fn to_bytes(&self) -> &[u8] {
        &self.rgba
    }

    /// Reconstruct from raw bytes saved by `to_bytes`.
    pub fn from_bytes(bytes: Vec<u8>) -> Option<Self> {
        if bytes.len() == 256 * 256 * 4 {
            Some(Self { rgba: bytes })
        } else {
            None
        }
    }
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Debug)]
pub struct ImageryFetchError(pub String);

impl fmt::Display for ImageryFetchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<std::io::Error> for ImageryFetchError {
    fn from(e: std::io::Error) -> Self {
        ImageryFetchError(e.to_string())
    }
}

// ── Fetch ─────────────────────────────────────────────────────────────────────

/// Download and decode an ESRI World Imagery tile into 256×256 RGBA8.
/// Blocking – call from a worker thread.
pub fn fetch_imagery_tile(x: i32, y: i32, zoom: u32) -> Result<ImageryTile, ImageryFetchError> {
    // ESRI tiles use {z}/{y}/{x} (note reversed y/x).
    let url = format!(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}"
    );

    let response = ureq::get(&url)
        .call()
        .map_err(|e| ImageryFetchError(e.to_string()))?;

    let mut bytes = Vec::new();
    response
        .into_reader()
        .read_to_end(&mut bytes)
        .map_err(|e| ImageryFetchError(e.to_string()))?;

    decode_imagery_tile(&bytes)
}

fn decode_imagery_tile(bytes: &[u8]) -> Result<ImageryTile, ImageryFetchError> {
    let img = image::load_from_memory(bytes)
        .map_err(|e| ImageryFetchError(e.to_string()))?;

    // Ensure exactly 256×256, converting any edge-case sizes the server might
    // return (e.g. 512×512 hi-res) down to the standard tile size.
    let img = if img.width() != 256 || img.height() != 256 {
        image::DynamicImage::ImageRgba8(image::imageops::resize(
            &img.to_rgba8(),
            256,
            256,
            image::imageops::FilterType::Lanczos3,
        ))
    } else {
        img
    };

    let rgba = img.to_rgba8().into_raw();
    debug_assert_eq!(rgba.len(), 256 * 256 * 4);
    Ok(ImageryTile { rgba })
}
