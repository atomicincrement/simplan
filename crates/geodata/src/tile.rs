//! Slippy-map tile coordinate arithmetic (EPSG:3857 / Web Mercator).

use std::f64::consts::PI;

/// Convert WGS-84 (lat, lon) to the integer tile (x, y) at a given zoom.
pub fn lat_lon_to_tile_xy(lat_deg: f64, lon_deg: f64, zoom: u32) -> (i32, i32) {
    let lat_r = lat_deg.to_radians();
    let n = (1u64 << zoom) as f64;
    let x = ((lon_deg + 180.0) / 360.0 * n).floor() as i32;
    let y = ((1.0 - (lat_r.tan() + 1.0 / lat_r.cos()).ln() / PI) / 2.0 * n).floor() as i32;
    (x, y)
}

/// Top-left corner (lat, lon) of tile (x, y) at the given zoom.
pub fn tile_top_left(x: i32, y: i32, zoom: u32) -> (f64, f64) {
    let n = (1u64 << zoom) as f64;
    let lon = x as f64 / n * 360.0 - 180.0;
    let lat = (PI * (1.0 - 2.0 * y as f64 / n)).sinh().atan().to_degrees();
    (lat, lon)
}

/// Fractional position (fx, fy in [0, 1)) of (lat, lon) within its tile at
/// the given zoom.  (0, 0) = top-left, (1, 1) = bottom-right.
pub fn lat_lon_frac_in_tile(lat_deg: f64, lon_deg: f64, zoom: u32) -> (f64, f64) {
    let lat_r = lat_deg.to_radians();
    let n = (1u64 << zoom) as f64;
    let px = (lon_deg + 180.0) / 360.0 * n;
    let py = (1.0 - (lat_r.tan() + 1.0 / lat_r.cos()).ln() / PI) / 2.0 * n;
    (px.fract(), py.fract())
}

/// Convert flat tangent-plane offset in metres to WGS-84 (lat, lon).
///
/// In Bevy's coord system: +X = east, +Y = up, -Z = north.
/// So `fx` = east metres from origin, `fz` = Bevy-Z metres from origin
/// (positive fz → south, negative fz → north).
pub fn flat_to_lat_lon(fx: f32, fz: f32, centre_lat: f64, centre_lon: f64) -> (f64, f64) {
    let lat = centre_lat + (-fz as f64) / crate::EARTH_RADIUS_M * (180.0 / PI);
    let lon = centre_lon
        + (fx as f64) / (crate::EARTH_RADIUS_M * centre_lat.to_radians().cos())
            * (180.0 / PI);
    (lat, lon)
}

/// Choose a reasonable Terrarium zoom level for a terrain tile with the given
/// half-size (metres).  Returns values in [5, 14].
///
/// We aim for roughly 4 elevation samples per terrain-mesh quad, so:
///   wanted_pixel_m = tile_width_m / (GRID * 4)
///   → z such that Terrarium pixel ≈ wanted_pixel_m at the given latitude.
pub fn zoom_for_half(half: f32, lat_deg: f64) -> u32 {
    let tile_width_m = half as f64 * 2.0;
    // At this latitude, the circumference of a parallel in metres.
    let circ = 2.0 * PI * crate::EARTH_RADIUS_M * lat_deg.to_radians().cos();
    // We want ~128 elevation pixels across the tile.
    let wanted_tile_width_m = tile_width_m;   // one Terrarium tile per terrain tile
    let z = (circ / (256.0 * wanted_tile_width_m)).log2().round() as i32;
    z.clamp(5, 14) as u32
}
