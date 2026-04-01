// Quad-tree LOD manager.
//
// Tiles are parameterised in the tangent-plane (flat x / flat z) projected from
// world origin onto the sphere.  Each node covers a square region
// [cx − half, cx + half] × [cz − half, cz + half].
//
// Split rule  : tile is too coarse  → split when nearest_dist < half * SPLIT_FACTOR
// Merge rule  : tile is fine enough → no children needed when nearest_dist ≥ half * SPLIT_FACTOR
//
// Only leaf nodes (no children) contribute a visible mesh.
// A minimum half size (MIN_HALF) prevents infinite subdivision.

use bevy::prelude::Entity;

/// Split a tile when the camera's nearest distance to its bounding square is
/// less than `half * SPLIT_FACTOR`.
const SPLIT_FACTOR: f32 = 4.0;

/// Finest tile half-size in metres (smallest tiles that will be meshed).
/// With ROOT_HALF = 5 000 m this allows at most 2 splits:
///   5 000 → 2 500 → 1 250  (1 250 is not > MIN_HALF, so no further split).
const MIN_HALF: f32 = 1_250.0;

// ── Data types ───────────────────────────────────────────────────────────────

/// A node in the quad-tree.
pub struct QuadNode {
    /// Flat tangent-plane centre x (metres from world origin).
    pub cx: f32,
    /// Flat tangent-plane centre z (metres from world origin).
    pub cz: f32,
    /// Half-size: tile spans [cx−half, cx+half] × [cz−half, cz+half].
    pub half: f32,
    /// Children, when this node has been split.
    pub children: Option<Box<[QuadNode; 4]>>,
    /// Bevy entity that owns this tile's mesh (only for leaf nodes).
    pub entity: Option<Entity>,
}

impl QuadNode {
    pub fn new(cx: f32, cz: f32, half: f32) -> Self {
        Self { cx, cz, half, children: None, entity: None }
    }

    /// Minimum squared distance from the camera xz position to the tile AABB.
    fn nearest_dist_sq(&self, cam_x: f32, cam_z: f32) -> f32 {
        let dx = (cam_x - self.cx).abs() - self.half;
        let dz = (cam_z - self.cz).abs() - self.half;
        let dx = dx.max(0.0);
        let dz = dz.max(0.0);
        dx * dx + dz * dz
    }

    /// Recursively update the quadtree from the given camera flat position.
    ///
    /// Returns a list of `DeltaOp`s describing entities to spawn or despawn.
    pub fn update(&mut self, cam_x: f32, cam_z: f32, ops: &mut Vec<DeltaOp>) {
        let dist_sq = self.nearest_dist_sq(cam_x, cam_z);
        let split_dist = self.half * SPLIT_FACTOR;
        let should_split = dist_sq < split_dist * split_dist && self.half > MIN_HALF;

        if should_split {
            // Need children — create them if they don't exist yet.
            if self.children.is_none() {
                // Despawn this node's own mesh; children will carry the geometry.
                if let Some(e) = self.entity.take() {
                    ops.push(DeltaOp::Despawn(e));
                }
                let h = self.half * 0.5;
                self.children = Some(Box::new([
                    QuadNode::new(self.cx - h, self.cz - h, h), // SW
                    QuadNode::new(self.cx + h, self.cz - h, h), // SE
                    QuadNode::new(self.cx - h, self.cz + h, h), // NW
                    QuadNode::new(self.cx + h, self.cz + h, h), // NE
                ]));
            }
            // Recurse into children.
            if let Some(children) = &mut self.children {
                for child in children.iter_mut() {
                    child.update(cam_x, cam_z, ops);
                }
            }
        } else {
            // This node should be a leaf.  Collapse children if present.
            if self.children.is_some() {
                let mut orphans = self.children.take().unwrap();
                for child in orphans.iter_mut() {
                    child.collect_entities(ops);
                }
            }
            // Ensure we have a mesh entity queued.
            if self.entity.is_none() {
                ops.push(DeltaOp::Spawn {
                    cx:   self.cx,
                    cz:   self.cz,
                    half: self.half,
                    // slot filled in by the plugin after spawning
                    slot: self as *mut QuadNode as usize,
                });
            }
        }
    }

    /// Recursively collect all entities in the subtree for despawning.
    fn collect_entities(&mut self, ops: &mut Vec<DeltaOp>) {
        if let Some(e) = self.entity.take() {
            ops.push(DeltaOp::Despawn(e));
        }
        if let Some(children) = &mut self.children {
            for child in children.iter_mut() {
                child.collect_entities(ops);
            }
        }
        self.children = None;
    }

    /// Walk the tree and collect visible (leaf) tiles.
    pub fn collect_leaves<'a>(&'a self, out: &mut Vec<&'a QuadNode>) {
        if let Some(children) = &self.children {
            for child in children.iter() {
                child.collect_leaves(out);
            }
        } else {
            out.push(self);
        }
    }

    /// Find the node whose `slot` pointer matches, return a mutable ref.
    pub fn find_slot_mut(&mut self, slot: usize) -> Option<&mut QuadNode> {
        if self as *mut QuadNode as usize == slot {
            return Some(self);
        }
        if let Some(children) = &mut self.children {
            for child in children.iter_mut() {
                if let Some(found) = child.find_slot_mut(slot) {
                    return Some(found);
                }
            }
        }
        None
    }
}

// ── Delta operations requested by the tree update ────────────────────────────

/// Operations returned by `QuadNode::update` for the Bevy plugin to execute.
pub enum DeltaOp {
    /// Spawn a new tile mesh at these coordinates.
    Spawn {
        cx:   f32,
        cz:   f32,
        half: f32,
        /// Raw pointer (as usize) to the `QuadNode` slot that needs its
        /// `entity` field set once the entity is spawned.
        slot: usize,
    },
    /// Despawn an existing tile entity.
    Despawn(Entity),
}

// ── Root wrapper ─────────────────────────────────────────────────────────────

/// Root of the terrain quad-tree.
///
/// The root tile covers ±ROOT_HALF metres in flat x and z.
pub const ROOT_HALF: f32 = 5_000.0; // 10 km × 10 km

pub struct QuadTree {
    pub root: QuadNode,
}

impl QuadTree {
    pub fn new() -> Self {
        Self { root: QuadNode::new(0.0, 0.0, ROOT_HALF) }
    }

    /// Force full subdivision down to `MIN_HALF` so the tree contains the
    /// highest-resolution leaf nodes everywhere. Useful for debugging.
    pub fn force_max_depth(&mut self) {
        fn recurse(node: &mut QuadNode) {
            if node.half <= MIN_HALF {
                // already at or below minimum
                node.children = None;
                return;
            }
            // create children if missing
            if node.children.is_none() {
                let h = node.half * 0.5;
                node.children = Some(Box::new([
                    QuadNode::new(node.cx - h, node.cz - h, h), // SW
                    QuadNode::new(node.cx + h, node.cz - h, h), // SE
                    QuadNode::new(node.cx - h, node.cz + h, h), // NW
                    QuadNode::new(node.cx + h, node.cz + h, h), // NE
                ]));
            }
            if let Some(children) = &mut node.children {
                for child in children.iter_mut() {
                    recurse(child);
                }
            }
        }
        recurse(&mut self.root);
    }

    /// Update the tree from the camera's flat (x, z) position and return a
    /// list of spawn/despawn operations to execute.
    pub fn update(&mut self, cam_x: f32, cam_z: f32) -> Vec<DeltaOp> {
        let mut ops = Vec::new();
        self.root.update(cam_x, cam_z, &mut ops);
        ops
    }

    /// Force the tree into a state where all leaf nodes are the finest
    /// resolution (children exist down to MIN_HALF) and return spawn ops
    /// for any leaf nodes that need entities, and despawn ops for any
    /// non-leaf nodes that currently own entities. This is used when the
    /// application requests rendering only the highest-resolution tiles.
    pub fn update_force_leaves(&mut self) -> Vec<DeltaOp> {
        let mut ops = Vec::new();

        fn recurse(node: &mut QuadNode, ops: &mut Vec<DeltaOp>) {
            if let Some(children) = &mut node.children {
                // This is an internal node: if it has an entity, request despawn.
                if let Some(e) = node.entity.take() {
                    ops.push(DeltaOp::Despawn(e));
                }
                // Recurse into children.
                for child in children.iter_mut() {
                    recurse(child, ops);
                }
            } else {
                // Leaf node: ensure it has a mesh entity queued.
                if node.entity.is_none() {
                    ops.push(DeltaOp::Spawn {
                        cx:   node.cx,
                        cz:   node.cz,
                        half: node.half,
                        slot: node as *mut QuadNode as usize,
                    });
                }
            }
        }

        recurse(&mut self.root, &mut ops);
        ops
    }

    /// Tear down the entire tree, returning despawn ops for every live entity.
    /// The tree is reset to a fresh root so the next `update` rebuilds from scratch.
    pub fn reset(&mut self) -> Vec<DeltaOp> {
        let mut ops = Vec::new();
        self.root.collect_entities(&mut ops);
        self.root = QuadNode::new(0.0, 0.0, ROOT_HALF);
        ops
    }

    /// Record the Bevy entity assigned to a freshly-spawned tile (identified
    /// by the `slot` pointer from a `DeltaOp::Spawn`).
    pub fn assign_entity(&mut self, slot: usize, entity: Entity) {
        if let Some(node) = self.root.find_slot_mut(slot) {
            node.entity = Some(entity);
        }
    }
}
