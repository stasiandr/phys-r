//! Bounding volume hierarchy for static level geometry and query acceleration.
//!
//! Built with a deterministic binned SAH split (falls back to a median split
//! for degenerate inputs); nodes are stored in a flat array in depth-first
//! order for cache-friendly traversal.

use crate::fixed::Fx;
use crate::geometry::{Aabb, Ray};
use crate::simd::{Box4, F4};

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
    /// 4-wide f32 copy of the tree used for traversal (see [`WNode`]).
    wide: Vec<WNode>,
    /// Leaves of the wide tree: up to four conservative item boxes each.
    leaves: Vec<Leaf>,
}

/// A 4-wide node with f32 bounds in SoA layout: one node test is a handful
/// of packed SSE / NEON instructions (see [`crate::simd`]).
///
/// The f32 boxes are inflated outward, so they only ever *cull*
/// conservatively: every item the exact fixed-point test could accept
/// survives, and the exact test then decides. Results are therefore
/// bit-identical to a pure fixed-point traversal on every platform.
#[derive(Clone, Copy, Debug)]
struct WNode {
    b: Box4,
    /// Inner lane: wide node index. Leaf lane: leaf index. Empty: unused.
    child: [u32; 4],
    /// Bit `l` set when lane `l` is a leaf.
    leaf_mask: u32,
    /// Bit `l` set when lane `l` is used.
    used_mask: u32,
}

#[derive(Clone, Copy, Debug)]
struct Leaf {
    b: Box4,
    start: u32,
    count: u32,
}

#[inline]
fn f_lo(v: Fx) -> f32 {
    let x = v.to_f64();
    (x - (1e-3 + x.abs() * 1e-6)) as f32
}

#[inline]
fn f_hi(v: Fx) -> f32 {
    let x = v.to_f64();
    (x + (1e-3 + x.abs() * 1e-6)) as f32
}

#[inline]
fn f_box(a: &Aabb) -> [f32; 6] {
    [f_lo(a.min.x), f_lo(a.min.y), f_lo(a.min.z), f_hi(a.max.x), f_hi(a.max.y), f_hi(a.max.z)]
}

/// Ray in f32 for culling.
struct FRay {
    o: [F4; 3],
    inv: [F4; 3],
}

impl FRay {
    #[inline]
    fn new(r: &Ray) -> FRay {
        let f = |i: usize| {
            let d = r.dir.get(i).to_f64();
            let inv = if d.abs() < 1e-12 { 1e30f32.copysign(if d < 0.0 { -1.0 } else { 1.0 }) } else { (1.0 / d) as f32 };
            (F4::splat(r.origin.get(i).to_f64() as f32), F4::splat(inv))
        };
        let (a, b, c) = (f(0), f(1), f(2));
        FRay { o: [a.0, b.0, c.0], inv: [a.1, b.1, c.1] }
    }
}

/// Conservative f32 bound for an exact distance.
#[inline]
fn t_bound(t: Fx) -> f32 {
    let x = t.to_f64();
    (x + 1e-3 + x.abs() * 1e-5) as f32
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
            wide: Vec::new(),
            leaves: Vec::new(),
        };
        if aabbs.is_empty() {
            return bvh;
        }
        let mut work: Vec<(u32, Aabb)> = ids.into_iter().zip(aabbs.iter().copied()).collect();
        bvh.build_rec(&mut work);
        bvh.widen();
        bvh
    }

    /// (Re)build the 4-wide f32 tree from the binary tree.
    fn widen(&mut self) {
        self.wide.clear();
        self.leaves.clear();
        if !self.nodes.is_empty() {
            self.wide_rec(0);
        }
    }

    fn wide_rec(&mut self, bin: u32) -> u32 {
        let idx = self.wide.len() as u32;
        self.wide.push(WNode { b: Box4::EMPTY, child: [0; 4], leaf_mask: 0, used_mask: 0 });
        // Collapse up to two binary levels into four lanes.
        let mut lanes: [u32; 4] = [0; 4];
        let mut n;
        let root = &self.nodes[bin as usize];
        if root.count > 0 {
            lanes[0] = bin;
            n = 1;
        } else {
            lanes[0] = bin + 1;
            lanes[1] = root.offset;
            n = 2;
            while n < 4 {
                let Some(k) = (0..n).find(|&k| self.nodes[lanes[k] as usize].count == 0) else { break };
                let inner = lanes[k];
                lanes[k] = inner + 1;
                lanes[n] = self.nodes[inner as usize].offset;
                n += 1;
            }
        }
        let mut w = WNode { b: Box4::EMPTY, child: [0; 4], leaf_mask: 0, used_mask: (1 << n) - 1 };
        for (l, &b) in lanes[..n].iter().enumerate() {
            let node = self.nodes[b as usize].clone();
            w.b.set(l, &f_box(&node.aabb));
            if node.count > 0 {
                let mut leaf = Leaf { b: Box4::EMPTY, start: node.offset, count: node.count };
                for k in 0..node.count as usize {
                    leaf.b.set(k, &f_box(&self.item_aabbs[node.offset as usize + k]));
                }
                w.child[l] = self.leaves.len() as u32;
                w.leaf_mask |= 1 << l;
                self.leaves.push(leaf);
            } else {
                w.child[l] = self.wide_rec(b);
            }
        }
        self.wide[idx as usize] = w;
        idx
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
        self.widen();
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

    /// Calls `f(id)` for every item whose exact box overlaps `aabb`. The
    /// visiting order is deterministic but unspecified; callers sort.
    #[inline]
    pub fn query_aabb(&self, aabb: &Aabb, mut f: impl FnMut(u32)) {
        self.query_aabb_slots(aabb, |k| f(self.items[k as usize]));
    }

    /// Item ids in slot order (slots are what the `*_slots` queries report;
    /// spatially close items have close slots).
    #[inline]
    pub fn ids(&self) -> &[u32] {
        &self.items
    }

    /// Like [`Bvh::query_aabb`] but reports item slots.
    #[inline]
    pub fn query_aabb_slots(&self, aabb: &Aabb, mut f: impl FnMut(u32)) {
        if self.wide.is_empty() {
            return;
        }
        let fb = f_box(aabb);
        let q = [F4::splat(fb[0]), F4::splat(fb[1]), F4::splat(fb[2]), F4::splat(fb[3]), F4::splat(fb[4]), F4::splat(fb[5])];
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let node = &self.wide[stack[sp] as usize];
            let mut m = node.b.overlap(&q) & node.used_mask;
            while m != 0 {
                let l = m.trailing_zeros() as usize;
                m &= m - 1;
                if node.leaf_mask & (1 << l) != 0 {
                    let leaf = &self.leaves[node.child[l] as usize];
                    let mut lm = leaf.b.overlap(&q) & ((1 << leaf.count) - 1);
                    while lm != 0 {
                        let k = leaf.start as usize + lm.trailing_zeros() as usize;
                        lm &= lm - 1;
                        if self.item_aabbs[k].overlaps(aabb) {
                            f(k as u32);
                        }
                    }
                } else if sp < STACK {
                    stack[sp] = node.child[l];
                    sp += 1;
                }
            }
        }
    }

    /// Ray traversal. `f(id, max_t)` is called for candidate items (their
    /// conservative box is hit before `max_t`) and returns `Some(t)` to shrink
    /// the ray. Lanes are visited near-first.
    #[inline]
    pub fn raycast(&self, ray: &Ray, max_t: Fx, mut f: impl FnMut(u32, Fx) -> Option<Fx>) -> Fx {
        self.raycast_slots(ray, max_t, |k, t| f(self.items[k as usize], t))
    }

    /// Like [`Bvh::raycast`] but reports item slots.
    #[inline]
    pub fn raycast_slots(&self, ray: &Ray, mut max_t: Fx, mut f: impl FnMut(u32, Fx) -> Option<Fx>) -> Fx {
        if self.wide.is_empty() {
            return max_t;
        }
        let r = FRay::new(ray);
        let mut tmax = t_bound(max_t);
        let mut stack = [(0u32, 0f32); STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let (ni, tn_node) = stack[sp];
            if tn_node > tmax {
                continue;
            }
            let node = &self.wide[ni as usize];
            let (m, tn) = node.b.ray(&r.o, &r.inv, tmax);
            let m = m & node.used_mask;
            if m == 0 {
                continue;
            }
            // Hit lanes sorted near-first (tiny insertion sort).
            let mut order = [0usize; 4];
            let mut k = 0;
            let mut mm = m;
            while mm != 0 {
                let l = mm.trailing_zeros() as usize;
                mm &= mm - 1;
                let mut j = k;
                while j > 0 && tn[order[j - 1]] > tn[l] {
                    order[j] = order[j - 1];
                    j -= 1;
                }
                order[j] = l;
                k += 1;
            }
            // Leaves now (near-first); inner lanes pushed far-first.
            for &l in &order[..k] {
                if node.leaf_mask & (1 << l) == 0 || tn[l] > tmax {
                    continue;
                }
                let leaf = &self.leaves[node.child[l] as usize];
                let (lm, _) = leaf.b.ray(&r.o, &r.inv, tmax);
                let mut lm = lm & ((1 << leaf.count) - 1);
                while lm != 0 {
                    let i = leaf.start as usize + lm.trailing_zeros() as usize;
                    lm &= lm - 1;
                    if let Some(t) = f(i as u32, max_t) {
                        if t < max_t {
                            max_t = t;
                            tmax = t_bound(max_t);
                        }
                    }
                }
            }
            for &l in order[..k].iter().rev() {
                if node.leaf_mask & (1 << l) == 0 && sp < STACK {
                    stack[sp] = (node.child[l], tn[l]);
                    sp += 1;
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

    /// The f32 culling must never lose an item the exact test accepts.
    #[test]
    fn wide_matches_brute_force() {
        let mut seed = 0x1234_5678u64;
        let mut rnd = |lo: i64, hi: i64| {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            Fx(lo + ((seed >> 20) as i64).rem_euclid(hi - lo))
        };
        let one = Fx::ONE.0;
        let boxes: Vec<Aabb> = (0..3000)
            .map(|_| {
                let c = Vec3::new(rnd(-200 * one, 200 * one), rnd(-20 * one, 20 * one), rnd(-200 * one, 200 * one));
                // Includes degenerate (flat) boxes touching exactly.
                let h = Vec3::new(rnd(0, 4 * one), rnd(0, 4 * one), rnd(0, 4 * one));
                Aabb::from_center_half(c, h)
            })
            .collect();
        let bvh = Bvh::build(&boxes);
        for _ in 0..2000 {
            let o = Vec3::new(rnd(-250 * one, 250 * one), rnd(-30 * one, 30 * one), rnd(-250 * one, 250 * one));
            let d = Vec3::new(rnd(-one, one), rnd(-one / 4, one / 4), rnd(-one, one));
            let d = d.normalize_or(Vec3::X);
            let ray = Ray::new(o, d);
            let max_t = Fx::from_int(300);
            let brute = boxes.iter().enumerate().filter_map(|(i, b)| b.ray_test(&ray, max_t).map(|t| (t, i as u32))).min();
            let mut best: Option<(Fx, u32)> = None;
            bvh.raycast(&ray, max_t, |id, cur| {
                let t = boxes[id as usize].ray_test(&ray, cur)?;
                if best.is_none_or(|b| (t, id) < b) {
                    best = Some((t, id));
                }
                Some(t)
            });
            assert_eq!(best, brute);
            let q = Aabb::from_center_half(o, Vec3::new(rnd(0, 10 * one), rnd(0, 10 * one), rnd(0, 10 * one)));
            let mut a = Vec::new();
            bvh.query_aabb(&q, |id| a.push(id));
            a.sort();
            let b: Vec<u32> = (0..boxes.len() as u32).filter(|&i| boxes[i as usize].overlaps(&q)).collect();
            assert_eq!(a, b);
        }
    }
}
