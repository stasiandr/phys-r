//! Bounding volume hierarchy for static level geometry and query acceleration.
//!
//! Built with a deterministic binned SAH split (falls back to a median split
//! for degenerate inputs); nodes are stored in a flat array in depth-first
//! order for cache-friendly traversal.

use crate::fixed::Fx;
use crate::geometry::{Aabb, Ray};

const LEAF_SIZE: usize = 4;
const BINS: usize = 12;
const STACK: usize = 64;

#[derive(Clone, Debug)]
struct Node {
    aabb: Aabb,
    /// Leaf: first index into `items`. Inner: index of the right child (the
    /// left child always immediately follows its parent).
    offset: u32,
    /// Number of items for a leaf, 0 for inner nodes.
    count: u32,
}

#[derive(Clone, Debug, Default)]
pub struct Bvh {
    nodes: Vec<Node>,
    items: Vec<u32>,
    item_aabbs: Vec<Aabb>,
}

impl Bvh {
    /// Build from item bounding boxes; item ids are their indices.
    pub fn build(aabbs: &[Aabb]) -> Bvh {
        let ids: Vec<u32> = (0..aabbs.len() as u32).collect();
        Bvh::build_with_ids(aabbs, ids)
    }

    /// Build from `(aabb, id)` pairs where `aabbs[i]` belongs to `ids[i]`.
    pub fn build_with_ids(aabbs: &[Aabb], ids: Vec<u32>) -> Bvh {
        let mut bvh = Bvh {
            nodes: Vec::with_capacity(aabbs.len() / 2 + 1),
            items: Vec::with_capacity(aabbs.len()),
            item_aabbs: Vec::with_capacity(aabbs.len()),
        };
        if aabbs.is_empty() {
            return bvh;
        }
        let mut work: Vec<(u32, Aabb)> = ids.into_iter().zip(aabbs.iter().copied()).collect();
        bvh.build_rec(&mut work);
        bvh
    }

    fn build_rec(&mut self, work: &mut [(u32, Aabb)]) -> u32 {
        let mut bounds = Aabb::EMPTY;
        let mut cbounds = Aabb::EMPTY;
        for (_, a) in work.iter() {
            bounds = bounds.union(a);
            let c = a.center();
            cbounds.min = cbounds.min.min(c);
            cbounds.max = cbounds.max.max(c);
        }
        let node_idx = self.nodes.len() as u32;
        if work.len() <= LEAF_SIZE {
            work.sort_unstable_by_key(|(id, _)| *id);
            let first = self.items.len() as u32;
            self.items.extend(work.iter().map(|(id, _)| *id));
            self.item_aabbs.extend(work.iter().map(|(_, a)| *a));
            self.nodes.push(Node { aabb: bounds, offset: first, count: work.len() as u32 });
            return node_idx;
        }
        let mid = match sah_split(work, &cbounds) {
            Some(m) => m,
            None => {
                // Degenerate centroids: median split by the total order
                // (centroid, id), a pure function of the input.
                let ext = cbounds.max - cbounds.min;
                let axis = if ext.x >= ext.y && ext.x >= ext.z {
                    0
                } else if ext.y >= ext.z {
                    1
                } else {
                    2
                };
                let mid = work.len() / 2;
                work.select_nth_unstable_by_key(mid, |(id, a)| ((a.min.get(axis) + a.max.get(axis)).0, *id));
                mid
            }
        };
        self.nodes.push(Node { aabb: bounds, offset: 0, count: 0 });
        let (l, r) = work.split_at_mut(mid);
        self.build_rec(l);
        let right = self.build_rec(r);
        self.nodes[node_idx as usize].offset = right;
        node_idx
    }

    /// Update item boxes in place and refit node bounds bottom-up, keeping
    /// the tree topology. Much cheaper than a rebuild for moving items.
    pub fn refit(&mut self, mut aabb_of: impl FnMut(u32) -> Aabb) {
        for (id, a) in self.items.iter().zip(self.item_aabbs.iter_mut()) {
            *a = aabb_of(*id);
        }
        // Children always have larger indices than their parent.
        for i in (0..self.nodes.len()).rev() {
            let n = &self.nodes[i];
            let b = if n.count > 0 {
                let s = n.offset as usize;
                self.item_aabbs[s..s + n.count as usize].iter().fold(Aabb::EMPTY, |acc, a| acc.union(a))
            } else {
                self.nodes[i + 1].aabb.union(&self.nodes[n.offset as usize].aabb)
            };
            self.nodes[i].aabb = b;
        }
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    pub fn bounds(&self) -> Aabb {
        self.nodes.first().map(|n| n.aabb).unwrap_or(Aabb::EMPTY)
    }

    /// Calls `f(id)` for every item whose box overlaps `aabb`, in a
    /// deterministic order.
    #[inline]
    pub fn query_aabb(&self, aabb: &Aabb, mut f: impl FnMut(u32)) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let idx = stack[sp];
            let node = &self.nodes[idx as usize];
            if !node.aabb.overlaps(aabb) {
                continue;
            }
            if node.count > 0 {
                let s = node.offset as usize;
                let e = s + node.count as usize;
                for (id, a) in self.items[s..e].iter().zip(&self.item_aabbs[s..e]) {
                    if a.overlaps(aabb) {
                        f(*id);
                    }
                }
            } else if sp + 2 <= STACK {
                stack[sp] = node.offset;
                stack[sp + 1] = idx + 1;
                sp += 2;
            }
        }
    }

    /// Ray traversal. `f(id, enter_t, max_t)` is called for items whose box
    /// the ray enters before `max_t` (`enter_t` is that entry distance) and
    /// returns `Some(t)` to shrink the ray. Children are visited near-first and
    /// every box is tested exactly once.
    #[inline]
    pub fn raycast(&self, ray: &Ray, mut max_t: Fx, mut f: impl FnMut(u32, Fx, Fx) -> Option<Fx>) -> Fx {
        if self.nodes.is_empty() {
            return max_t;
        }
        let Some(t_root) = self.nodes[0].aabb.ray_test(ray, max_t) else { return max_t };
        let mut stack = [(0u32, Fx::ZERO); STACK];
        stack[0] = (0, t_root);
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let (idx, t_enter) = stack[sp];
            // The ray may have been shortened since this node was pushed.
            if t_enter > max_t {
                continue;
            }
            let node = &self.nodes[idx as usize];
            if node.count > 0 {
                let s = node.offset as usize;
                let e = s + node.count as usize;
                for (&id, a) in self.items[s..e].iter().zip(&self.item_aabbs[s..e]) {
                    let Some(enter) = a.ray_test(ray, max_t) else { continue };
                    if let Some(t) = f(id, enter, max_t) {
                        if t < max_t {
                            max_t = t;
                        }
                    }
                }
            } else if sp + 2 <= STACK {
                let l = idx + 1;
                let r = node.offset;
                let tl = self.nodes[l as usize].aabb.ray_test(ray, max_t);
                let tr = self.nodes[r as usize].aabb.ray_test(ray, max_t);
                match (tl, tr) {
                    (Some(a), Some(b)) => {
                        // Push the farther child first so the nearer pops first.
                        if a <= b {
                            stack[sp] = (r, b);
                            stack[sp + 1] = (l, a);
                        } else {
                            stack[sp] = (l, a);
                            stack[sp + 1] = (r, b);
                        }
                        sp += 2;
                    }
                    (Some(a), None) => {
                        stack[sp] = (l, a);
                        sp += 1;
                    }
                    (None, Some(b)) => {
                        stack[sp] = (r, b);
                        sp += 1;
                    }
                    (None, None) => {}
                }
            }
        }
        max_t
    }
}

/// Half surface area as a raw integer (enough for comparing SAH costs).
#[inline]
fn half_area(a: &Aabb) -> i128 {
    let d = a.max - a.min;
    let (x, y, z) = (d.x.0.max(0) as i128, d.y.0.max(0) as i128, d.z.0.max(0) as i128);
    x * y + y * z + z * x
}

/// Binned SAH over all three axes. Partitions `work` and returns the split
/// index, or `None` when no split beats the median heuristic's input
/// (all centroids coincide). Ties break toward the lowest axis/bin, and the
/// partition orders by (bin, centroid, id), so the result is deterministic.
fn sah_split(work: &mut [(u32, Aabb)], cb: &Aabb) -> Option<usize> {
    let n = work.len();
    let mut best: Option<(i128, usize, usize)> = None; // (cost, axis, split bin)
    for axis in 0..3 {
        let lo = cb.min.get(axis).0;
        let ext = cb.max.get(axis).0 - lo;
        if ext <= 0 {
            continue;
        }
        let width = ext / BINS as i64 + 1;
        let bin_of = |a: &Aabb| -> usize {
            let c = (a.min.get(axis).0 >> 1) + (a.max.get(axis).0 >> 1) - lo;
            ((c.max(0) / width) as usize).min(BINS - 1)
        };
        let mut counts = [0usize; BINS];
        let mut boxes = [Aabb::EMPTY; BINS];
        for (_, a) in work.iter() {
            let b = bin_of(a);
            counts[b] += 1;
            boxes[b] = boxes[b].union(a);
        }
        // Prefix sweep from the left, suffix from the right.
        let mut left_area = [0i128; BINS];
        let mut left_n = [0usize; BINS];
        let mut acc = Aabb::EMPTY;
        let mut cnt = 0;
        for i in 0..BINS - 1 {
            acc = acc.union(&boxes[i]);
            cnt += counts[i];
            left_area[i] = if cnt > 0 { half_area(&acc) } else { 0 };
            left_n[i] = cnt;
        }
        let mut acc = Aabb::EMPTY;
        let mut cnt = 0;
        for i in (1..BINS).rev() {
            acc = acc.union(&boxes[i]);
            cnt += counts[i];
            let (ln, rn) = (left_n[i - 1], cnt);
            if ln == 0 || rn == 0 {
                continue;
            }
            let cost = left_area[i - 1] * ln as i128 + half_area(&acc) * rn as i128;
            if best.is_none_or(|(bc, _, _)| cost < bc) {
                best = Some((cost, axis, i));
            }
        }
    }
    let (_, axis, split) = best?;
    let lo = cb.min.get(axis).0;
    let ext = cb.max.get(axis).0 - lo;
    let width = ext / BINS as i64 + 1;
    let key = |a: &Aabb| -> (usize, i64) {
        let c = (a.min.get(axis).0 >> 1) + (a.max.get(axis).0 >> 1);
        ((((c - lo).max(0) / width) as usize).min(BINS - 1), c)
    };
    work.sort_unstable_by_key(|(id, a)| (key(a), *id));
    let mid = work.iter().position(|(_, a)| key(a).0 >= split).unwrap_or(n / 2);
    if mid == 0 || mid == n {
        return None;
    }
    Some(mid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    #[test]
    fn query_finds_all() {
        let boxes: Vec<Aabb> = (0..100).map(|i| Aabb::from_center_half(Vec3::from_int(i * 2, 0, 0), Vec3::splat(Fx::HALF))).collect();
        let bvh = Bvh::build(&boxes);
        let mut hits = Vec::new();
        bvh.query_aabb(&Aabb::new(Vec3::from_int(9, -1, -1), Vec3::from_int(21, 1, 1)), |id| hits.push(id));
        hits.sort();
        assert_eq!(hits, vec![5, 6, 7, 8, 9, 10]);
    }
}
