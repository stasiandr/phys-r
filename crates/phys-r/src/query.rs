//! Scene queries: raycasts, overlaps and shape sweeps.

use crate::body::BodyHandle;
use crate::collide::{collide_convex, rounded_core, rounded_distance, Contacts};
use crate::fixed::Fx;
use crate::fx;
use crate::geometry::{Aabb, Ray};
use crate::math::{Iso, Quat, Vec3};
use crate::shape::Shape;
use crate::world::World;

/// Query filter.
#[derive(Copy, Clone, Debug)]
pub struct QueryFilter {
    /// Layers (bitmask) the query can hit.
    pub mask: u32,
    /// Body to ignore (e.g. the shooter).
    pub exclude: BodyHandle,
    pub include_sensors: bool,
    pub include_static: bool,
    pub include_moving: bool,
}

impl Default for QueryFilter {
    fn default() -> Self {
        QueryFilter { mask: u32::MAX, exclude: BodyHandle::INVALID, include_sensors: false, include_static: true, include_moving: true }
    }
}

impl QueryFilter {
    pub fn mask(mask: u32) -> QueryFilter {
        QueryFilter { mask, ..Default::default() }
    }

    pub fn excluding(mut self, h: BodyHandle) -> QueryFilter {
        self.exclude = h;
        self
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RayHit {
    pub body: BodyHandle,
    /// Distance along the (normalised) ray.
    pub distance: Fx,
    pub point: Vec3,
    pub normal: Vec3,
    /// Hit point in the body's local space. For hitscan hit zones: on a
    /// character capsule `local_point.y` is the height above the capsule
    /// centre, so e.g. `y > 0.55` can count as a headshot.
    pub local_point: Vec3,
}

/// Result of a shape sweep.
#[derive(Copy, Clone, Debug)]
pub struct SweepHit {
    pub body: BodyHandle,
    /// Fraction of the displacement travelled before the hit (0..1).
    pub fraction: Fx,
    /// Surface normal pointing toward the swept shape.
    pub normal: Vec3,
}

/// A gathered sweep obstacle with its pose cached, so repeated sweeps during
/// one character move can cheaply reject by bounding box.
#[derive(Copy, Clone, Debug)]
pub(crate) struct ObRef {
    pub body: u32,
    pub shape: u32,
    pub iso: Iso,
    pub aabb: Aabb,
}

// ---------------------------------------------------------------------------
// Ray vs primitive
// ---------------------------------------------------------------------------

/// Ray vs sphere: returns `(t, normal)`.
pub fn ray_sphere(o: Vec3, d: Vec3, c: Vec3, r: Fx, max_t: Fx) -> Option<(Fx, Vec3)> {
    let m = o - c;
    let b = m.dot(d);
    let cc = m.length_sq() - r * r;
    if cc > Fx::ZERO && b > Fx::ZERO {
        return None;
    }
    let disc = b * b - cc;
    if disc < Fx::ZERO {
        return None;
    }
    let t = (-b - disc.sqrt()).max(Fx::ZERO);
    if t > max_t {
        return None;
    }
    let p = o + d.scale(t);
    Some((t, (p - c).normalize_or(-d)))
}

/// Ray vs capsule with core segment `a..b`.
pub fn ray_capsule(o: Vec3, d: Vec3, a: Vec3, b: Vec3, r: Fx, max_t: Fx) -> Option<(Fx, Vec3)> {
    let mut best: Option<(Fx, Vec3)> = None;
    let mut consider = |h: Option<(Fx, Vec3)>| {
        if let Some((t, n)) = h {
            if best.is_none_or(|(bt, _)| t < bt) {
                best = Some((t, n));
            }
        }
    };
    consider(ray_sphere(o, d, a, r, max_t));
    consider(ray_sphere(o, d, b, r, max_t));
    // Finite cylinder (Ericson 5.3.7).
    let ax = b - a;
    let m = o - a;
    let md = m.dot(ax);
    let nd = d.dot(ax);
    let dd = ax.length_sq();
    if dd.0 > 0 {
        let mn = m.dot(d);
        let aa = dd - nd * nd;
        let k = m.length_sq() - r * r;
        let c = dd * k - md * md;
        if aa.abs() > fx!(1e-6) {
            let bb = dd * mn - nd * md;
            let disc = bb * bb - aa * c;
            if disc >= Fx::ZERO {
                let t = (-bb - disc.sqrt()) / aa;
                let s = md + t * nd;
                if t >= Fx::ZERO && t <= max_t && s >= Fx::ZERO && s <= dd {
                    let p = o + d.scale(t);
                    let axis_pt = a + ax.scale(s / dd);
                    consider(Some((t, (p - axis_pt).normalize_or(-d))));
                } else if c <= Fx::ZERO && md >= Fx::ZERO && md <= dd {
                    // Origin inside the cylinder.
                    consider(Some((Fx::ZERO, -d)));
                }
            }
        }
    }
    best
}

/// Ray vs box `[-h, h]` in local space.
pub fn ray_box_local(o: Vec3, d: Vec3, h: Vec3, max_t: Fx) -> Option<(Fx, Vec3)> {
    let mut t0 = Fx::ZERO;
    let mut t1 = max_t;
    let mut n = -d;
    for i in 0..3 {
        let oi = o.get(i);
        let di = d.get(i);
        let hi = h.get(i);
        if di.abs().0 < 64 {
            if oi < -hi || oi > hi {
                return None;
            }
        } else {
            let mut a = (-hi - oi) / di;
            let mut b = (hi - oi) / di;
            let mut s = Fx::NEG_ONE;
            if a > b {
                core::mem::swap(&mut a, &mut b);
                s = Fx::ONE;
            }
            if a > t0 {
                t0 = a;
                n = Vec3::ZERO;
                n.set(i, s);
            }
            t1 = t1.min(b);
            if t0 > t1 {
                return None;
            }
        }
    }
    Some((t0, n))
}

/// Ray vs a shape at `iso`. Returns `(t, normal)`.
#[inline]
pub fn ray_shape(shape: &Shape, iso: &Iso, ray: &Ray, max_t: Fx) -> Option<(Fx, Vec3)> {
    match shape {
        Shape::Sphere { radius } => ray_sphere(ray.origin, ray.dir, iso.pos, *radius, max_t),
        Shape::Capsule { .. } => {
            let (a, b, r) = rounded_core(shape, iso);
            ray_capsule(ray.origin, ray.dir, a, b, r, max_t)
        }
        Shape::Box { half_extents } => {
            let o = iso.inverse_transform_point(ray.origin);
            let d = iso.inverse_transform_vector(ray.dir);
            ray_box_local(o, d, *half_extents, max_t).map(|(t, n)| (t, iso.transform_vector(n)))
        }
    }
}

impl World {
    #[inline]
    pub(crate) fn filter_accepts(&self, idx: u32, f: &QueryFilter) -> bool {
        let b = &self.bodies[idx as usize];
        b.alive
            && (f.mask & (1u32 << b.layer)) != 0
            && (f.include_sensors || !b.is_sensor())
            && !(f.exclude.generation != 0 && f.exclude.index == idx && f.exclude.generation == b.generation)
    }

    /// Closest hit along a ray. `dir` need not be normalised.
    pub fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: Fx, filter: &QueryFilter) -> Option<RayHit> {
        let ray = Ray::new(origin, dir);
        let mut best: Option<(Fx, u32, Vec3)> = None;
        let mut max_t = max_dist;
        let test = |idx: u32, cur: Fx, best: &mut Option<(Fx, u32, Vec3)>| -> Option<Fx> {
            let b = &self.bodies[idx as usize];
            let enter = b.aabb.ray_test(&ray, cur)?;
            // Re-base the ray near the shape to keep fixed-point magnitudes small.
            let base = (enter - fx!(0.5)).max(Fx::ZERO);
            let local = Ray { origin: ray.at(base), ..ray };
            let (t, n) = ray_shape(&self.shapes[b.shape.0 as usize], &b.iso(), &local, cur - base)?;
            let t = t + base;
            if best.is_none_or(|(bt, bi, _)| t < bt || (t == bt && idx < bi)) {
                *best = Some((t, idx, n));
            }
            Some(t)
        };
        if filter.include_static {
            max_t = self.static_bvh.raycast(&ray, max_t, |idx, cur| {
                if !self.filter_accepts(idx, filter) {
                    return None;
                }
                test(idx, cur, &mut best)
            });
        }
        if filter.include_moving {
            self.raycast_movers(&ray, max_t, |idx, cur| {
                if !self.filter_accepts(idx, filter) {
                    return None;
                }
                test(idx, cur, &mut best)
            });
        }
        best.map(|(t, idx, n)| self.make_hit(idx, &self.bodies[idx as usize].iso(), &ray, t, n))
    }

    #[inline]
    pub(crate) fn make_hit(&self, idx: u32, iso: &Iso, ray: &Ray, t: Fx, normal: Vec3) -> RayHit {
        let point = ray.at(t);
        RayHit { body: self.handle_of(idx), distance: t, point, normal, local_point: iso.inverse_transform_point(point) }
    }

    /// All bodies whose shapes overlap a sphere. Results are appended to `out`
    /// in body-index order.
    pub fn overlap_sphere(&self, center: Vec3, radius: Fx, filter: &QueryFilter, out: &mut Vec<BodyHandle>) {
        let shape = Shape::sphere(radius);
        self.overlap_shape(&shape, &Iso::new(center, Quat::IDENTITY), filter, out)
    }

    /// All bodies overlapping a convex query shape.
    pub fn overlap_shape(&self, shape: &Shape, iso: &Iso, filter: &QueryFilter, out: &mut Vec<BodyHandle>) {
        let aabb = shape.aabb(iso);
        let mut found: Vec<u32> = Vec::new();
        let mut c = Contacts::default();
        let mut check = |idx: u32, found: &mut Vec<u32>| {
            let b = &self.bodies[idx as usize];
            if !b.aabb.overlaps(&aabb) {
                return;
            }
            let bs = &self.shapes[b.shape.0 as usize];
            let biso = b.iso();
            let hit = collide_convex(shape, iso, bs, &biso, Fx::ZERO, &mut c) && c.points[..c.count].iter().any(|p| p.depth >= Fx::ZERO);
            if hit {
                found.push(idx);
            }
        };
        if filter.include_static {
            self.static_bvh.query_aabb(&aabb, |idx| {
                if self.filter_accepts(idx, filter) {
                    check(idx, &mut found);
                }
            });
        }
        if filter.include_moving {
            self.query_movers(&aabb, |idx| {
                if self.filter_accepts(idx, filter) {
                    check(idx, &mut found);
                }
            });
        }
        found.sort_unstable();
        out.extend(found.into_iter().map(|i| self.handle_of(i)));
    }

    /// Gather sweep obstacles overlapping `aabb` into `out` (sorted).
    pub(crate) fn gather_obstacles(&self, aabb: &Aabb, filter: &QueryFilter, out: &mut Vec<ObRef>) {
        out.clear();
        let push = |idx: u32, out: &mut Vec<ObRef>| {
            let b = &self.bodies[idx as usize];
            if !b.aabb.overlaps(aabb) {
                return;
            }
            out.push(ObRef { body: idx, shape: b.shape.0, iso: b.iso(), aabb: b.aabb });
        };
        if filter.include_static {
            self.static_bvh.query_aabb(aabb, |idx| {
                if self.filter_accepts(idx, filter) {
                    push(idx, out);
                }
            });
        }
        if filter.include_moving {
            self.query_movers(aabb, |idx| {
                if self.filter_accepts(idx, filter) {
                    push(idx, out);
                }
            });
        }
        out.sort_unstable_by_key(|o| o.body);
    }

    /// Penetration of a rounded segment into an obstacle: `(depth, push_dir)`.
    pub(crate) fn rounded_penetration(&self, a0: Vec3, a1: Vec3, r: Fx, shape: &Shape, siso: &Iso) -> Option<(Fx, Vec3)> {
        let probe = if a0 == a1 { Shape::sphere(r) } else { Shape::capsule((a1 - a0).length().shr(1), r) };
        let mid = (a0 + a1).half();
        let up = (a1 - a0).normalize_or(Vec3::Y);
        let iso = Iso::new(mid, rotation_from_y(up));
        let mut c = Contacts::default();
        if !collide_convex(&probe, &iso, shape, siso, Fx::ZERO, &mut c) {
            return None;
        }
        let d = c.deepest()?.depth;
        if d <= Fx::ZERO {
            return None;
        }
        Some((d, -c.normal))
    }

    /// Sweep a rounded segment (sphere when `a0 == a1`, capsule otherwise)
    /// along `disp` against pre-gathered obstacles. Stops `skin` short of
    /// contact. Uses conservative advancement, which is exact for convex
    /// obstacles (distance along a translation is a convex function of time).
    /// Returns `(fraction, normal, obstacle, contact_point)`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn sweep_rounded(
        &self,
        obstacles: &[ObRef],
        a0: Vec3,
        a1: Vec3,
        r: Fx,
        disp: Vec3,
        skin: Fx,
        skip: u32,
    ) -> Option<(Fx, Vec3, ObRef, Vec3)> {
        if disp.is_zero() {
            return None;
        }
        let mut best: Option<(Fx, Vec3, ObRef, Vec3)> = None;
        let swept = Aabb::new(a0.min(a1), a0.max(a1)).expand(r + skin + skin).sweep(disp);
        for &orf in obstacles {
            if orf.body == skip || !orf.aabb.overlaps(&swept) {
                continue;
            }
            let shape = &self.shapes[orf.shape as usize];
            let limit = best.map_or(Fx::ONE, |b| b.0);
            let mut t = Fx::ZERO;
            let mut hit = None;
            for _ in 0..12 {
                let off = disp.scale(t);
                match rounded_distance(a0 + off, a1 + off, r, shape, &orf.iso) {
                    None => {
                        // Cores overlap: use the penetration direction.
                        if let Some((_, n)) = self.rounded_penetration(a0 + off, a1 + off, r, shape, &orf.iso) {
                            if disp.dot(n) < Fx::ZERO {
                                hit = Some((t, n, (a0 + a1).half() + off - n.scale(r)));
                            }
                        }
                        break;
                    }
                    Some((d, n, core)) => {
                        let closing = -disp.dot(n);
                        if d <= skin {
                            if closing > Fx::ZERO {
                                hit = Some((t, n, core - n.scale(r)));
                            }
                            break;
                        }
                        if closing <= Fx::ZERO {
                            break;
                        }
                        let step = (d - skin) / closing;
                        t += step.max(Fx(1));
                        if t > limit {
                            break;
                        }
                        if step < fx!(1e-4) {
                            hit = Some((t, n, core + disp.scale(step) - n.scale(r)));
                            break;
                        }
                    }
                }
            }
            if let Some((t, n, p)) = hit {
                if t <= limit && best.is_none_or(|b| t < b.0) {
                    best = Some((t, n, orf, p));
                }
            }
        }
        best
    }

    /// Sweep a sphere through the world (e.g. grenade trajectories, thick
    /// bullets). Returns the first hit.
    pub fn sphere_cast(&mut self, center: Vec3, radius: Fx, disp: Vec3, filter: &QueryFilter) -> Option<SweepHit> {
        self.shape_cast_rounded(center, center, radius, disp, filter)
    }

    /// Sweep a Y-axis capsule (centre, half height of the inner segment).
    pub fn capsule_cast(&mut self, center: Vec3, half_height: Fx, radius: Fx, disp: Vec3, filter: &QueryFilter) -> Option<SweepHit> {
        let h = Vec3::new(Fx::ZERO, half_height, Fx::ZERO);
        self.shape_cast_rounded(center - h, center + h, radius, disp, filter)
    }

    fn shape_cast_rounded(&mut self, a0: Vec3, a1: Vec3, r: Fx, disp: Vec3, filter: &QueryFilter) -> Option<SweepHit> {
        let aabb = Aabb::new(a0.min(a1), a0.max(a1)).expand(r + fx!(0.01)).sweep(disp);
        let mut obs = core::mem::take(&mut self.scratch_obstacles);
        self.gather_obstacles(&aabb, filter, &mut obs);
        let res = self.sweep_rounded(&obs, a0, a1, r, disp, fx!(0.001), u32::MAX);
        self.scratch_obstacles = obs;
        res.map(|(t, n, o, _)| SweepHit { body: self.handle_of(o.body), fraction: t, normal: n })
    }
}

/// Rotation taking local +Y onto unit vector `up`.
pub(crate) fn rotation_from_y(up: Vec3) -> Quat {
    let d = Vec3::Y.dot(up);
    if d > fx!(0.99999) {
        return Quat::IDENTITY;
    }
    if d < fx!(-0.99999) {
        return Quat::new(Fx::ONE, Fx::ZERO, Fx::ZERO, Fx::ZERO);
    }
    let axis = Vec3::Y.cross(up);
    Quat::new(axis.x, axis.y, axis.z, Fx::ONE + d).normalize()
}
