//! Bounding boxes and closest-point primitives.

use crate::fixed::Fx;
use crate::fx;
use crate::math::Vec3;

/// Axis-aligned bounding box.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb { min: Vec3::splat(Fx::MAX), max: Vec3::splat(Fx::MIN) };

    #[inline(always)]
    pub const fn new(min: Vec3, max: Vec3) -> Aabb {
        Aabb { min, max }
    }

    #[inline]
    pub fn from_center_half(c: Vec3, h: Vec3) -> Aabb {
        Aabb { min: c - h, max: c + h }
    }

    #[inline]
    pub fn from_points(pts: &[Vec3]) -> Aabb {
        let mut a = Aabb::EMPTY;
        for p in pts {
            a.min = a.min.min(*p);
            a.max = a.max.max(*p);
        }
        a
    }

    #[inline(always)]
    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && self.max.x >= o.min.x
            && self.min.y <= o.max.y
            && self.max.y >= o.min.y
            && self.min.z <= o.max.z
            && self.max.z >= o.min.z
    }

    #[inline(always)]
    pub fn contains_point(&self, p: Vec3) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y && p.z >= self.min.z && p.z <= self.max.z
    }

    #[inline(always)]
    pub fn union(&self, o: &Aabb) -> Aabb {
        Aabb { min: self.min.min(o.min), max: self.max.max(o.max) }
    }

    #[inline(always)]
    pub fn expand(&self, m: Fx) -> Aabb {
        let v = Vec3::splat(m);
        Aabb { min: self.min - v, max: self.max + v }
    }

    /// Grow to contain the box swept along `d`.
    #[inline]
    pub fn sweep(&self, d: Vec3) -> Aabb {
        Aabb { min: self.min.min(self.min + d), max: self.max.max(self.max + d) }
    }

    #[inline(always)]
    pub fn center(&self) -> Vec3 {
        (self.min + self.max).half()
    }

    #[inline(always)]
    pub fn half_extents(&self) -> Vec3 {
        (self.max - self.min).half()
    }

    /// Slab test. `inv_d` is from [`Ray::new`]. Returns entry distance.
    #[inline]
    pub fn ray_test(&self, ray: &Ray, max_t: Fx) -> Option<Fx> {
        let mut t0 = Fx::ZERO;
        let mut t1 = max_t;
        for i in 0..3 {
            let o = ray.origin.get(i);
            let lo = self.min.get(i);
            let hi = self.max.get(i);
            if ray.parallel[i] {
                if o < lo || o > hi {
                    return None;
                }
            } else {
                let inv = ray.inv_dir.get(i);
                let mut a = (lo - o) * inv;
                let mut b = (hi - o) * inv;
                if a > b {
                    core::mem::swap(&mut a, &mut b);
                }
                t0 = t0.max(a);
                t1 = t1.min(b);
                if t0 > t1 {
                    return None;
                }
            }
        }
        Some(t0)
    }
}

/// Ray with a unit direction and precomputed reciprocal.
#[derive(Copy, Clone, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
    pub inv_dir: Vec3,
    pub parallel: [bool; 3],
}

impl Ray {
    /// `dir` is normalised internally.
    pub fn new(origin: Vec3, dir: Vec3) -> Ray {
        let dir = dir.normalize_or(Vec3::Z);
        let mut inv = Vec3::ZERO;
        let mut par = [false; 3];
        for i in 0..3 {
            let d = dir.get(i);
            if d.abs().0 < 64 {
                par[i] = true;
            } else {
                inv.set(i, Fx::ONE / d);
            }
        }
        Ray { origin, dir, inv_dir: inv, parallel: par }
    }

    #[inline(always)]
    pub fn at(&self, t: Fx) -> Vec3 {
        self.origin + self.dir.scale(t)
    }
}

const PAR_EPS: Fx = fx!(1e-6);

/// Closest point on segment `ab` to `p`. Returns `(t, point)`.
#[inline]
pub fn closest_on_segment(p: Vec3, a: Vec3, b: Vec3) -> (Fx, Vec3) {
    let ab = b - a;
    let denom = ab.length_sq();
    if denom.0 <= 0 {
        return (Fx::ZERO, a);
    }
    let t = ((p - a).dot(ab) / denom).clamp(Fx::ZERO, Fx::ONE);
    (t, a + ab.scale(t))
}

/// Closest points between segments `p1q1` and `p2q2` (Ericson 5.1.9).
/// Returns `(s, t, c1, c2)`.
pub fn closest_segment_segment(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Fx, Fx, Vec3, Vec3) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.length_sq();
    let e = d2.length_sq();
    let f = d2.dot(r);
    let (s, t);
    if a <= PAR_EPS && e <= PAR_EPS {
        return (Fx::ZERO, Fx::ZERO, p1, p2);
    }
    if a <= PAR_EPS {
        s = Fx::ZERO;
        t = (f / e).clamp(Fx::ZERO, Fx::ONE);
    } else {
        let c = d1.dot(r);
        if e <= PAR_EPS {
            t = Fx::ZERO;
            s = (-c / a).clamp(Fx::ZERO, Fx::ONE);
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > PAR_EPS * a.max(e) { ((b * f - c * e) / denom).clamp(Fx::ZERO, Fx::ONE) } else { Fx::ZERO };
            let mut t0 = (b * s0 + f) / e;
            if t0 < Fx::ZERO {
                t0 = Fx::ZERO;
                s0 = (-c / a).clamp(Fx::ZERO, Fx::ONE);
            } else if t0 > Fx::ONE {
                t0 = Fx::ONE;
                s0 = ((b - c) / a).clamp(Fx::ZERO, Fx::ONE);
            }
            s = s0;
            t = t0;
        }
    }
    (s, t, p1 + d1.scale(s), p2 + d2.scale(t))
}

/// Clip segment `pq` against box `[-h, h]`. Returns the parameter interval
/// inside the box, if any.
pub fn segment_box_clip(p: Vec3, q: Vec3, h: Vec3) -> Option<(Fx, Fx)> {
    let d = q - p;
    let mut t0 = Fx::ZERO;
    let mut t1 = Fx::ONE;
    for i in 0..3 {
        let o = p.get(i);
        let di = d.get(i);
        let hi = h.get(i);
        if di.abs().0 < 16 {
            if o < -hi || o > hi {
                return None;
            }
        } else {
            let mut a = (-hi - o) / di;
            let mut b = (hi - o) / di;
            if a > b {
                core::mem::swap(&mut a, &mut b);
            }
            t0 = t0.max(a);
            t1 = t1.min(b);
            if t0 > t1 {
                return None;
            }
        }
    }
    Some((t0, t1))
}

/// Closest points between segment `pq` and box `[-h, h]` (box local space).
/// Returns `(on_segment, on_box, dist_sq)`; distance is zero if they intersect.
///
/// The squared distance from `p + t (q - p)` to the box is a convex,
/// piecewise-quadratic function of `t` whose pieces change only where a
/// coordinate crosses a slab boundary, so the exact minimum is found by
/// checking those breakpoints and the analytic minimum of the adjacent pieces.
pub fn closest_segment_box(p: Vec3, q: Vec3, h: Vec3) -> (Vec3, Vec3, Fx) {
    if let Some((t0, t1)) = segment_box_clip(p, q, h) {
        let x = p + (q - p).scale((t0 + t1).shr(1));
        return (x, x, Fx::ZERO);
    }
    let d = q - p;
    let dist2 = |t: Fx| -> Fx {
        let x = p + d.scale(t);
        (x - x.clamp(-h, h)).length_sq()
    };
    let mut ts = [Fx::ZERO; 8];
    let mut n = 0;
    ts[n] = Fx::ZERO;
    n += 1;
    ts[n] = Fx::ONE;
    n += 1;
    for i in 0..3 {
        let di = d.get(i);
        if di.abs().0 < 16 {
            continue;
        }
        for bound in [-h.get(i), h.get(i)] {
            let t = (bound - p.get(i)) / di;
            if t > Fx::ZERO && t < Fx::ONE {
                ts[n] = t;
                n += 1;
            }
        }
    }
    let ts = &mut ts[..n];
    ts.sort_unstable();
    let mut best_k = 0;
    let mut best = Fx::MAX;
    for (k, &t) in ts.iter().enumerate() {
        let v = dist2(t);
        if v < best {
            best = v;
            best_k = k;
        }
    }
    let mut best_t = ts[best_k];
    // Minimum of the quadratic piece on each side of the best breakpoint.
    for (lo, hi) in [(best_k.wrapping_sub(1), best_k), (best_k, best_k + 1)] {
        if lo >= ts.len() || hi >= ts.len() {
            continue;
        }
        let (tl, th) = (ts[lo], ts[hi]);
        let mid = p + d.scale((tl + th).shr(1));
        // Active axes (outside the slab) and their nearest face on this piece.
        let mut num = Fx::ZERO;
        let mut den = Fx::ZERO;
        for i in 0..3 {
            let xi = mid.get(i);
            let hi_ = h.get(i);
            let face = if xi > hi_ {
                hi_
            } else if xi < -hi_ {
                -hi_
            } else {
                continue;
            };
            num += (p.get(i) - face) * d.get(i);
            den += d.get(i) * d.get(i);
        }
        if den.0 > 0 {
            let t = (-num / den).clamp(tl, th);
            let v = dist2(t);
            if v < best {
                best = v;
                best_t = t;
            }
        }
    }
    let x = p + d.scale(best_t);
    (x, x.clamp(-h, h), best)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_segment_crossing() {
        let (_, _, c1, c2) =
            closest_segment_segment(Vec3::from_int(-1, 0, 0), Vec3::from_int(1, 0, 0), Vec3::from_int(0, 1, -1), Vec3::from_int(0, 1, 1));
        assert!((c1 - Vec3::ZERO).length() < fx!(1e-6));
        assert!((c2 - Vec3::Y).length() < fx!(1e-6));
    }

    #[test]
    fn segment_box_matches_brute_force() {
        let h = Vec3::from_f32(0.5, 1.0, 0.25);
        let mut seed = 12345u32;
        let mut rnd = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 * 6.0 - 3.0
        };
        for _ in 0..500 {
            let p = Vec3::from_f32(rnd(), rnd(), rnd());
            let q = Vec3::from_f32(rnd(), rnd(), rnd());
            let (_, _, d) = closest_segment_box(p, q, h);
            let mut brute = Fx::MAX;
            for k in 0..=2000 {
                let x = Vec3::lerp(p, q, Fx::from_ratio(k, 2000));
                brute = brute.min((x - x.clamp(-h, h)).length_sq());
            }
            assert!(d <= brute + fx!(1e-5), "{d:?} vs {brute:?}");
            assert!((d - brute).abs() < fx!(1e-3), "{d:?} vs {brute:?}");
        }
    }

    #[test]
    fn segment_box() {
        let h = Vec3::ONE;
        let (s, b, d) = closest_segment_box(Vec3::from_int(-3, 2, 0), Vec3::from_int(3, 2, 0), h);
        assert!((d - Fx::ONE).abs() < fx!(1e-5));
        assert_eq!(s.y, Fx::from_int(2));
        assert_eq!(b.y, Fx::ONE);
    }
}
