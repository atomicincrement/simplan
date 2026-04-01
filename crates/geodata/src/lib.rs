//! Real-world elevation data for simplan.
//!
//! Elevation tiles are fetched from AWS Terrain Tiles (Terrarium PNG format),
//! cached to disk under `$HOME/.cache/simplan/elevation/`, and decoded into
//! 256×256 f32 height grids that the terrain crate can sample.
//!
//! No API key is required.  Source:
//!   https://registry.opendata.aws/terrain-tiles/

pub mod cache;
pub mod elevation;
pub mod imagery;
pub mod tile;

pub use cache::GeoCache;
pub use imagery::ImageryTile;

// ── Default scene centre: Ålesund Airport, Vigra (ENAL) ──────────────────────
// 62.5625°N, 6.1097°E – fjords, sea, and 1 700 m peaks within 20 km.
pub const CENTRE_LAT: f64 = 62.5625;
pub const CENTRE_LON: f64 = 6.1097;

pub const EARTH_RADIUS_M: f64 = 6_371_000.0;
