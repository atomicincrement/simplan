//! Thread-safe two-level cache (memory + disk) for elevation and imagery tiles.

use std::{
    collections::{HashMap, HashSet},
    fs,
    path::PathBuf,
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
};

use crate::{
    elevation::{ElevationTile, fetch_elevation_tile},
    imagery::{ImageryTile, fetch_imagery_tile},
    tile::{lat_lon_frac_in_tile, lat_lon_to_tile_xy},
};

// ── GeoCache ──────────────────────────────────────────────────────────────────

/// Thread-safe, clone-cheap cache for elevation tiles.
///
/// Cheap to clone (wraps an `Arc`), safe to share across threads/tasks.
#[derive(Clone)]
pub struct GeoCache {
    inner: Arc<Inner>,
}

struct Inner {
    // ── Elevation ─────────────────────────────────────────────────────────
    elevation:    Mutex<HashMap<(i32, i32, u32), Arc<ElevationTile>>>,
    inflight:     Mutex<HashSet<(i32, i32, u32)>>,
    fetch_count:  AtomicU32,
    cache_dir:    PathBuf,
    // ── Imagery ───────────────────────────────────────────────────────────
    imagery:         Mutex<HashMap<(i32, i32, u32), Arc<ImageryTile>>>,
    imagery_inflight: Mutex<HashSet<(i32, i32, u32)>>,
    imagery_count:   AtomicU32,
    imagery_dir:     PathBuf,
}

impl GeoCache {
    pub fn new() -> Self {
        let base = {
            #[cfg(unix)]
            {
                std::env::var("HOME")
                    .map(|h| PathBuf::from(h).join(".cache").join("simplan"))
                    .unwrap_or_else(|_| PathBuf::from(".simplan_cache"))
            }
            #[cfg(not(unix))]
            {
                std::env::var("APPDATA")
                    .map(|h| PathBuf::from(h).join("simplan"))
                    .unwrap_or_else(|_| PathBuf::from(".simplan_cache"))
            }
        };

        let cache_dir   = base.join("elevation");
        let imagery_dir = base.join("imagery");
        let _ = fs::create_dir_all(&cache_dir);
        let _ = fs::create_dir_all(&imagery_dir);

        Self {
            inner: Arc::new(Inner {
                elevation:        Mutex::new(HashMap::new()),
                inflight:         Mutex::new(HashSet::new()),
                fetch_count:      AtomicU32::new(0),
                cache_dir,
                imagery:          Mutex::new(HashMap::new()),
                imagery_inflight: Mutex::new(HashSet::new()),
                imagery_count:    AtomicU32::new(0),
                imagery_dir,
            }),
        }
    }

    /// Sample elevation at (lat_deg, lon_deg) using tiles at the given zoom.
    /// Fetches and caches the tile on first access.  Blocking.
    pub fn elevation_at(&self, lat_deg: f64, lon_deg: f64, zoom: u32) -> f32 {
        let (tx, ty) = lat_lon_to_tile_xy(lat_deg, lon_deg, zoom);
        let tile = self.get_tile(tx, ty, zoom);
        let (fx, fy) = lat_lon_frac_in_tile(lat_deg, lon_deg, zoom);
        tile.sample(fx as f32, fy as f32)
    }

    /// Non-blocking elevation sample.  Returns the cached value immediately if
    /// the tile is in memory, otherwise returns 0.0 and spawns a background
    /// thread to fetch the tile.  Call `fetch_count()` to detect when new
    /// tiles have arrived so callers can request a rebuild.
    pub fn elevation_at_nonblocking(&self, lat_deg: f64, lon_deg: f64, zoom: u32) -> f32 {
        let (tx, ty) = lat_lon_to_tile_xy(lat_deg, lon_deg, zoom);

        // Fast path: already cached in memory.
        {
            let map = self.inner.elevation.lock().unwrap();
            if let Some(t) = map.get(&(tx, ty, zoom)) {
                let (fx, fy) = lat_lon_frac_in_tile(lat_deg, lon_deg, zoom);
                return t.sample(fx as f32, fy as f32);
            }
        }

        // Tile not cached — kick off a background fetch if not already in flight.
        {
            let mut inflight = self.inner.inflight.lock().unwrap();
            if inflight.insert((tx, ty, zoom)) {
                let cache = self.clone();
                std::thread::spawn(move || {
                    cache.get_tile(tx, ty, zoom); // loads, caches, increments counter
                    cache.inner.inflight.lock().unwrap().remove(&(tx, ty, zoom));
                });
            }
        }

        0.0
    }

    /// Number of elevation tiles that have been loaded into the memory cache.
    /// Use as a generation counter to detect when new elevation data is available.
    pub fn fetch_count(&self) -> u32 {
        self.inner.fetch_count.load(Ordering::Relaxed)
    }

    // ── Imagery ─────────────────────────────────────────────────────────────

    /// Non-blocking imagery tile access.
    ///
    /// Returns the tile immediately if it is already in the memory cache.
    /// Otherwise fires a background thread to load/fetch it and returns `None`.
    /// Poll `imagery_fetch_count()` to know when new imagery has arrived.
    pub fn get_imagery_tile_nonblocking(&self, x: i32, y: i32, zoom: u32) -> Option<Arc<ImageryTile>> {
        // Fast path.
        {
            let map = self.inner.imagery.lock().unwrap();
            if let Some(t) = map.get(&(x, y, zoom)) {
                return Some(Arc::clone(t));
            }
        }

        // Kick off a background fetch if not already in flight.
        {
            let mut inflight = self.inner.imagery_inflight.lock().unwrap();
            if inflight.insert((x, y, zoom)) {
                let cache = self.clone();
                std::thread::spawn(move || {
                    cache.get_imagery_tile(x, y, zoom);
                    cache.inner.imagery_inflight.lock().unwrap().remove(&(x, y, zoom));
                });
            }
        }

        None
    }

    /// Blocking imagery tile access.  Loads from disk cache or network.
    pub fn get_imagery_tile(&self, x: i32, y: i32, zoom: u32) -> Arc<ImageryTile> {
        // Fast path.
        {
            let map = self.inner.imagery.lock().unwrap();
            if let Some(t) = map.get(&(x, y, zoom)) {
                return Arc::clone(t);
            }
        }

        let tile = self.load_or_fetch_imagery(x, y, zoom);
        let tile = Arc::new(tile);
        self.inner.imagery.lock().unwrap().insert((x, y, zoom), Arc::clone(&tile));
        self.inner.imagery_count.fetch_add(1, Ordering::Relaxed);
        tile
    }

    /// Number of imagery tiles loaded into the memory cache.
    /// Use as a generation counter to detect when satellite textures are ready.
    pub fn imagery_fetch_count(&self) -> u32 {
        self.inner.imagery_count.load(Ordering::Relaxed)
    }

    // ── Internal ─────────────────────────────────────────────────────────────

    fn get_tile(&self, x: i32, y: i32, zoom: u32) -> Arc<ElevationTile> {
        // Fast path: already in memory.
        {
            let map = self.inner.elevation.lock().unwrap();
            if let Some(t) = map.get(&(x, y, zoom)) {
                return Arc::clone(t);
            }
        }

        // Try disk, then network.
        let tile = self.load_or_fetch(x, y, zoom);
        let tile = Arc::new(tile);
        self.inner.elevation.lock().unwrap().insert((x, y, zoom), Arc::clone(&tile));
        self.inner.fetch_count.fetch_add(1, Ordering::Relaxed);
        tile
    }

    fn load_or_fetch_imagery(&self, x: i32, y: i32, zoom: u32) -> ImageryTile {
        let path = self.inner.imagery_dir.join(format!("{zoom}_{x}_{y}.rgba"));

        if path.exists() {
            if let Ok(bytes) = fs::read(&path) {
                if let Some(t) = ImageryTile::from_bytes(bytes) {
                    eprintln!("[geodata] imagery disk  {zoom}/{x}/{y}");
                    return t;
                }
            }
        }

        eprintln!("[geodata] imagery fetch {zoom}/{x}/{y} …");
        let tile = match fetch_imagery_tile(x, y, zoom) {
            Ok(t) => {
                eprintln!("[geodata] imagery fetch {zoom}/{x}/{y} OK");
                t
            }
            Err(e) => {
                eprintln!("[geodata] imagery fetch {zoom}/{x}/{y} FAILED: {e}");
                ImageryTile::blank()
            }
        };

        let _ = fs::write(&path, tile.to_bytes());
        tile
    }

    fn load_or_fetch(&self, x: i32, y: i32, zoom: u32) -> ElevationTile {
        let path = self.inner.cache_dir.join(format!("{zoom}_{x}_{y}.bin"));

        // Try disk cache first.
        if path.exists() {
            if let Ok(bytes) = fs::read(&path) {
                if let Some(t) = ElevationTile::from_bytes(&bytes) {
                    eprintln!("[geodata] disk  {zoom}/{x}/{y}");
                    return t;
                }
            }
        }

        // Fetch from AWS Terrain Tiles.
        eprintln!("[geodata] fetch {zoom}/{x}/{y} …");
        let tile = match fetch_elevation_tile(x, y, zoom) {
            Ok(t) => {
                eprintln!("[geodata] fetch {zoom}/{x}/{y} OK");
                t
            }
            Err(e) => {
                eprintln!("[geodata] fetch {zoom}/{x}/{y} FAILED: {e}");
                ElevationTile::flat()
            }
        };

        // Persist to disk.
        let _ = fs::write(&path, tile.to_bytes());

        tile
    }
}

impl Default for GeoCache {
    fn default() -> Self {
        Self::new()
    }
}
