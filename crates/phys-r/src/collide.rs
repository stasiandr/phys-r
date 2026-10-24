//! Narrow-phase contact generation.
//!
//! Every function produces a manifold of up to four points with a single
//! normal pointing from shape A to shape B. `depth > 0` means penetration;
//! negative depths (within `margin`) are speculative contacts.

use crate::fixed::Fx;
use crate::fx;
use crate::geometry::{closest_on_segment, closest_segment_box, closest_segment_segment};
use crate::math::{Iso, Vec3};
use crate::shape::Shape;

pub const MAX_POINTS: usize = 4;

#[derive(Copy, Clone, Default, Debug)]
pub struct ContactPoint {
    /// World-space contact point (midway between the surfaces).
    pub point: Vec3,
    /// Penetration depth (negative = separated by that distance).
    pub depth: Fx,
    /// Feature id used to match points between frames for warm starting.
    pub id: u32,
}

#[derive(Copy, Clone, Default, Debug)]
pub struct Contacts {
    /// Unit normal from A to B.
    pub normal: Vec3,
    pub points: [ContactPoint; MAX_POINTS],
    pub count: usize,
}

impl Contacts {
    #[inline]
    fn clear(&mut self) {
        self.count = 0;
    }

    #[inline]
    fn push(&mut self, point: Vec3, depth: Fx, id: u32) {
        if self.count < MAX_POINTS {
            self.points[self.count] = ContactPoint { point, depth, id };
            self.count += 1;
        }
    }

    #[inline]
    fn has_point_near(&self, p: Vec3, tol: Fx) -> bool {
        let t2 = tol * tol;
        self.points[..self.count].iter().any(|c| (c.point - p).length_sq() < t2)
    }

    /// Deepest point, if any.
    pub fn deepest(&self) -> Option<&ContactPoint> {
        self.points[..self.count].iter().max_by_key(|p| p.depth)
    }

    #[inline]
    fn flip(&mut self) {
        self.normal = -self.normal;
    }
}

const TINY: Fx = fx!(1e-5);
const DISTINCT: Fx = fx!(0.01);
const ALIGN: Fx = fx!(0.95);

/// Contacts between two convex shapes (anything but triangle meshes).
pub fn collide_convex(sa: &Shape, ia: &Iso, sb: &Shape, ib: &Iso, margin: Fx, out: &mut Contacts) -> bool {
    out.clear();
    match (sa, sb) {
        (Shape::Box { half_extents: ha }, Shape::Box { half_extents: hb }) => box_box(*ha, ia, *hb, ib, margin, out),
        (Shape::Box { .. }, _) => {
            let r = collide_convex(sb, ib, sa, ia, margin, out);
            out.flip();
            r
        }
        (_, Shape::Box { half_extents }) => {
            let (a0, a1, r) = rounded_core(sa, ia);
            rounded_box(a0, a1, r, *half_extents, ib, margin, out)
        }
        _ => {
            let (a0, a1, ra) = rounded_core(sa, ia);
            let (b0, b1, rb) = rounded_core(sb, ib);
            rounded_rounded(a0, a1, ra, b0, b1, rb, margin, out)
        }
    }
}

/// Inner segment (world space) and radius of a sphere or capsule.
#[inline]
pub fn rounded_core(s: &Shape, iso: &Iso) -> (Vec3, Vec3, Fx) {
    match s {
        Shape::Sphere { radius } => (iso.pos, iso.pos, *radius),
        Shape::Capsule { half_height, radius } => {
            let a = iso.rot.rotate(Vec3::new(Fx::ZERO, *half_height, Fx::ZERO));
            (iso.pos - a, iso.pos + a, *radius)
        }
        _ => (iso.pos, iso.pos, Fx::ZERO),
    }
}

/// Sphere/capsule vs sphere/capsule.
#[allow(clippy::too_many_arguments)]
pub fn rounded_rounded(a0: Vec3, a1: Vec3, ra: Fx, b0: Vec3, b1: Vec3, rb: Fx, margin: Fx, out: &mut Contacts) -> bool {
    let (_, _, ca, cb) = closest_segment_segment(a0, a1, b0, b1);
    let rsum = ra + rb;
    let (n, dist) = (cb - ca).normalize_with_len();
    if dist > rsum + margin {
        return false;
    }
    let n = if dist <= TINY {
        let mid_a = (a0 + a1).half();
        let mid_b = (b0 + b1).half();
        (mid_b - mid_a).normalize_or(Vec3::Y)
    } else {
        n
    };
    out.normal = n;
    let depth = rsum - dist;
    out.push(ca + n.scale(ra - depth.shr(1)), depth, 0);

    // Parallel capsules: add endpoint contacts so they rest stably side by side.
    let limit = rsum + margin;
    if a0 != a1 {
        for (k, e) in [a0, a1].into_iter().enumerate() {
            let (_, q) = closest_on_segment(e, b0, b1);
            let (ne, de) = (q - e).normalize_with_len();
            if de > TINY && de <= limit && ne.dot(n) >= ALIGN && (e - ca).length() > DISTINCT {
                let d = rsum - de;
                let p = e + n.scale(ra - d.shr(1));
                if !out.has_point_near(p, DISTINCT) {
                    out.push(p, d, 1 + k as u32);
                }
            }
        }
    }
    if b0 != b1 {
        for (k, e) in [b0, b1].into_iter().enumerate() {
            let (_, q) = closest_on_segment(e, a0, a1);
            let (ne, de) = (e - q).normalize_with_len();
            if de > TINY && de <= limit && ne.dot(n) >= ALIGN {
                let d = rsum - de;
                let p = q + n.scale(ra - d.shr(1));
                if !out.has_point_near(p, DISTINCT) {
                    out.push(p, d, 3 + k as u32);
                }
            }
        }
    }
    true
}

/// Sphere/capsule (A) vs box (B).
pub fn rounded_box(a0w: Vec3, a1w: Vec3, r: Fx, h: Vec3, ib: &Iso, margin: Fx, out: &mut Contacts) -> bool {
    let p0 = ib.inverse_transform_point(a0w);
    let p1 = ib.inverse_transform_point(a1w);
    let is_seg = p0 != p1;
    let (ps, pb, d2) = if is_seg {
        closest_segment_box(p0, p1, h)
    } else {
        let q = p0.clamp(-h, h);
        (p0, q, (p0 - q).length_sq())
    };
    let limit = r + margin;
    if d2 > limit * limit {
        return false;
    }
    let (n_out, dist) = (ps - pb).normalize_with_len();
    if dist > TINY {
        let depth = r - dist;
        out.push(ib.transform_point(pb + n_out.scale(depth.shr(1))), depth, 0);
        if is_seg {
            for (k, e) in [p0, p1].into_iter().enumerate() {
                let q = e.clamp(-h, h);
                let (ne, de) = (e - q).normalize_with_len();
                if de > TINY && de <= limit && ne.dot(n_out) >= ALIGN && (e - ps).length() > DISTINCT {
                    let d = r - de;
                    let p = ib.transform_point(q + n_out.scale(d.shr(1)));
                    if !out.has_point_near(p, DISTINCT) {
                        out.push(p, d, 1 + k as u32);
                    }
                }
            }
        }
        out.normal = -ib.transform_vector(n_out);
        return true;
    }

    // Core intersects the box: separating-axis search for the exit direction.
    let mut axes = [Vec3::ZERO; 6];
    let mut na = 3;
    axes[0] = Vec3::X;
    axes[1] = Vec3::Y;
    axes[2] = Vec3::Z;
    if is_seg {
        let sd = p1 - p0;
        for basis in [Vec3::X, Vec3::Y, Vec3::Z] {
            let (c, l) = sd.cross(basis).normalize_with_len();
            if l > fx!(1e-3) {
                axes[na] = c;
                na += 1;
            }
        }
    }
    let mut best_pen = Fx::MAX;
    let mut best_n = Vec3::Y;
    let mut best_hb = Fx::ZERO;
    for a in &axes[..na] {
        let hb = a.abs().dot(h);
        let s0 = p0.dot(*a).min(p1.dot(*a));
        let s1 = p0.dot(*a).max(p1.dot(*a));
        let pen_pos = hb + r - s0;
        let pen_neg = s1 + hb + r;
        if pen_pos < best_pen {
            best_pen = pen_pos;
            best_n = *a;
            best_hb = hb;
        }
        if pen_neg < best_pen {
            best_pen = pen_neg;
            best_n = -*a;
            best_hb = hb;
        }
    }
    let ends: &[Vec3] = if is_seg { &[p0, p1] } else { &[p0] };
    let mut deepest = (Fx::MIN, Vec3::ZERO);
    for (k, e) in ends.iter().enumerate() {
        let depth = best_hb + r - e.dot(best_n);
        if depth > deepest.0 {
            deepest = (depth, *e);
        }
        if depth > -margin {
            let p = *e - best_n.scale(r - depth.shr(1));
            out.push(ib.transform_point(p), depth, 8 + k as u32);
        }
    }
    if out.count == 0 {
        let p = deepest.1 - best_n.scale(r - deepest.0.shr(1));
        out.push(ib.transform_point(p), deepest.0, 8);
    }
    out.normal = -ib.transform_vector(best_n);
    true
}

// ---------------------------------------------------------------------------
// Polygon clipping (shared by box-box and box-triangle)
// ---------------------------------------------------------------------------

const POLY_MAX: usize = 16;

#[derive(Copy, Clone)]
struct Poly {
    v: [Vec3; POLY_MAX],
    id: [u32; POLY_MAX],
    n: usize,
}

impl Poly {
    fn new() -> Poly {
        Poly { v: [Vec3::ZERO; POLY_MAX], id: [0; POLY_MAX], n: 0 }
    }

    #[inline]
    fn push(&mut self, v: Vec3, id: u32) {
        if self.n < POLY_MAX {
            self.v[self.n] = v;
            self.id[self.n] = id;
            self.n += 1;
        }
    }

    /// Keep the part with `dot(p, pn) <= off` (Sutherland–Hodgman).
    fn clip(&self, pn: Vec3, off: Fx, plane_id: u32) -> Poly {
        let mut out = Poly::new();
        if self.n == 0 {
            return out;
        }
        let mut prev = self.n - 1;
        let mut dprev = self.v[prev].dot(pn) - off;
        for i in 0..self.n {
            let d = self.v[i].dot(pn) - off;
            if (dprev <= Fx::ZERO) != (d <= Fx::ZERO) {
                let t = dprev / (dprev - d);
                let p = self.v[prev] + (self.v[i] - self.v[prev]).scale(t);
                out.push(p, 0x100 | (plane_id << 4) | (self.id[prev] & 0xF));
            }
            if d <= Fx::ZERO {
                out.push(self.v[i], self.id[i]);
            }
            prev = i;
            dprev = d;
        }
        out
    }
}

/// Collect clipped incident points below the reference plane and reduce to
/// at most four points.
fn emit_clipped(poly: &Poly, ref_point: Vec3, n: Vec3, margin: Fx, id_base: u32, out: &mut Contacts) {
    let mut pts = [ContactPoint::default(); POLY_MAX];
    let mut cnt = 0;
    for i in 0..poly.n {
        let p = poly.v[i];
        let sep = (p - ref_point).dot(n);
        if sep <= margin {
            pts[cnt] = ContactPoint { point: p - n.scale(sep.shr(1)), depth: -sep, id: id_base | poly.id[i] };
            cnt += 1;
        }
    }
    reduce_points(&pts[..cnt], n, out);
}

/// Pick up to four points that keep the deepest one and maximise area.
pub(crate) fn reduce_points(pts: &[ContactPoint], n: Vec3, out: &mut Contacts) {
    if pts.len() <= MAX_POINTS {
        for p in pts {
            out.push(p.point, p.depth, p.id);
        }
        return;
    }
    let mut used = [false; 64];
    let mut i0 = 0;
    for (i, p) in pts.iter().enumerate() {
        if p.depth > pts[i0].depth {
            i0 = i;
        }
    }
    used[i0] = true;
    let p0 = pts[i0].point;
    let mut i1 = usize::MAX;
    let mut best = Fx::MIN;
    for (i, p) in pts.iter().enumerate() {
        let d = (p.point - p0).length_sq();
        if !used[i] && d > best {
            best = d;
            i1 = i;
        }
    }
    used[i1] = true;
    let p1 = pts[i1].point;
    let mut i2 = usize::MAX;
    let mut best_area = Fx::MIN;
    for (i, p) in pts.iter().enumerate() {
        let a = (p1 - p0).cross(p.point - p0).dot(n);
        if !used[i] && a > best_area {
            best_area = a;
            i2 = i;
        }
    }
    let mut i3 = usize::MAX;
    let mut worst_area = Fx::MAX;
    for (i, p) in pts.iter().enumerate() {
        let a = (p1 - p0).cross(p.point - p0).dot(n);
        if !used[i] && i != i2 && a < worst_area {
            worst_area = a;
            i3 = i;
        }
    }
    for i in [i0, i1, i2, i3] {
        if i != usize::MAX {
            out.push(pts[i].point, pts[i].depth, pts[i].id);
        }
    }
}

// ---------------------------------------------------------------------------
// Box vs box
// ---------------------------------------------------------------------------

const FACE_TOL: Fx = fx!(0.005);
const EDGE_TOL: Fx = fx!(0.01);

/// Box A vs box B (SAT with reference-face clipping).
pub fn box_box(ha: Vec3, ia: &Iso, hb: Vec3, ib: &Iso, margin: Fx, out: &mut Contacts) -> bool {
    let ma = ia.rot.to_mat3();
    let mb = ib.rot.to_mat3();
    let ax = [ma.col(0), ma.col(1), ma.col(2)];
    let bx = [mb.col(0), mb.col(1), mb.col(2)];
    let d = ib.pos - ia.pos;
    let eps = Fx(32);
    let mut absc = [[Fx::ZERO; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            absc[i][j] = ax[i].dot(bx[j]).abs() + eps;
        }
    }

    let mut sep_a = (Fx::MIN, 0usize);
    for i in 0..3 {
        let rb = hb.x * absc[i][0] + hb.y * absc[i][1] + hb.z * absc[i][2];
        let s = d.dot(ax[i]).abs() - (ha.get(i) + rb);
        if s > margin {
            return false;
        }
        if s > sep_a.0 {
            sep_a = (s, i);
        }
    }
    let mut sep_b = (Fx::MIN, 0usize);
    for j in 0..3 {
        let ra = ha.x * absc[0][j] + ha.y * absc[1][j] + ha.z * absc[2][j];
        let s = d.dot(bx[j]).abs() - (hb.get(j) + ra);
        if s > margin {
            return false;
        }
        if s > sep_b.0 {
            sep_b = (s, j);
        }
    }
    let mut sep_e = (Fx::MIN, 0usize, 0usize, Vec3::ZERO);
    for i in 0..3 {
        for j in 0..3 {
            let (n, len) = ax[i].cross(bx[j]).normalize_with_len();
            if len < fx!(1e-3) {
                continue;
            }
            let ra = ha.x * ax[0].dot(n).abs() + ha.y * ax[1].dot(n).abs() + ha.z * ax[2].dot(n).abs();
            let rb = hb.x * bx[0].dot(n).abs() + hb.y * bx[1].dot(n).abs() + hb.z * bx[2].dot(n).abs();
            let dn = d.dot(n);
            let s = dn.abs() - (ra + rb);
            if s > margin {
                return false;
            }
            if s > sep_e.0 {
                sep_e = (s, i, j, if dn < Fx::ZERO { -n } else { n });
            }
        }
    }

    let face_best = if sep_b.0 > sep_a.0 + FACE_TOL { sep_b.0 } else { sep_a.0 };
    if sep_e.0 > face_best + EDGE_TOL {
        // Edge-edge contact.
        let (_, i, j, n) = sep_e;
        let mut ca = ia.pos;
        for k in 0..3 {
            if k != i {
                ca += ax[k].scale(ha.get(k) * ax[k].dot(n).sign_nonzero());
            }
        }
        let mut cb = ib.pos;
        for k in 0..3 {
            if k != j {
                cb -= bx[k].scale(hb.get(k) * bx[k].dot(n).sign_nonzero());
            }
        }
        let ea = ax[i].scale(ha.get(i));
        let eb = bx[j].scale(hb.get(j));
        let (_, _, pa, pb) = closest_segment_segment(ca - ea, ca + ea, cb - eb, cb + eb);
        let depth = (pa - pb).dot(n);
        out.normal = n;
        out.push((pa + pb).half(), depth, 0x2000 | ((i as u32) << 4) | j as u32);
        return true;
    }

    if sep_b.0 > sep_a.0 + FACE_TOL {
        let j = sep_b.1;
        let n_ref = bx[j].scale(-d.dot(bx[j]).sign_nonzero());
        face_contact(ib.pos, &bx, hb, j, n_ref, ia.pos, &ax, ha, margin, 0x1000 | ((j as u32) << 8), out);
        out.normal = -n_ref;
    } else {
        let i = sep_a.1;
        let n_ref = ax[i].scale(d.dot(ax[i]).sign_nonzero());
        face_contact(ia.pos, &ax, ha, i, n_ref, ib.pos, &bx, hb, margin, (i as u32) << 8, out);
        out.normal = n_ref;
    }
    out.count > 0
}

#[allow(clippy::too_many_arguments)]
fn face_contact(
    rc: Vec3,
    rax: &[Vec3; 3],
    rh: Vec3,
    ri: usize,
    n: Vec3,
    ic: Vec3,
    iax: &[Vec3; 3],
    ih: Vec3,
    margin: Fx,
    id_base: u32,
    out: &mut Contacts,
) {
    // Incident face: the face of the other box most anti-parallel to n.
    let mut k = 0;
    let mut best = Fx::MIN;
    for (m, a) in iax.iter().enumerate() {
        let v = a.dot(n).abs();
        if v > best {
            best = v;
            k = m;
        }
    }
    let s = if iax[k].dot(n) > Fx::ZERO { Fx::NEG_ONE } else { Fx::ONE };
    let fc = ic + iax[k].scale(ih.get(k) * s);
    let u = iax[(k + 1) % 3].scale(ih.get((k + 1) % 3));
    let v = iax[(k + 2) % 3].scale(ih.get((k + 2) % 3));
    let fid = ((k as u32) << 2) | if s > Fx::ZERO { 0 } else { 0x40 };
    let mut poly = Poly::new();
    poly.push(fc + u + v, fid);
    poly.push(fc - u + v, fid | 1);
    poly.push(fc - u - v, fid | 2);
    poly.push(fc + u - v, fid | 3);

    let mut plane = 0;
    for a in [(ri + 1) % 3, (ri + 2) % 3] {
        let ax = rax[a];
        let c = ax.dot(rc);
        let h = rh.get(a);
        poly = poly.clip(ax, c + h, plane);
        poly = poly.clip(-ax, -c + h, plane + 1);
        plane += 2;
    }
    let ref_point = rc + n.scale(rh.get(ri));
    emit_clipped(&poly, ref_point, n, margin, id_base, out);
}

// ---------------------------------------------------------------------------
// Distance queries (used by character sweeps)
// ---------------------------------------------------------------------------

/// Separation between a world-space rounded segment (capsule core `a0..a1`,
/// radius `r`) and a convex obstacle at `iso`. Returns `(distance, normal,
/// core_point)` where the normal points from the obstacle toward the capsule
/// and `core_point` is the closest point on the capsule's core segment, or
/// `None` if the cores overlap (use contact generation for that case).
#[inline]
pub fn rounded_distance(a0: Vec3, a1: Vec3, r: Fx, shape: &Shape, iso: &Iso) -> Option<(Fx, Vec3, Vec3)> {
    match shape {
        Shape::Sphere { .. } | Shape::Capsule { .. } => {
            let (b0, b1, rb) = rounded_core(shape, iso);
            let (_, _, ca, cb) = closest_segment_segment(a0, a1, b0, b1);
            let (n, d) = (ca - cb).normalize_with_len();
            if d <= TINY {
                return None;
            }
            Some((d - r - rb, n, ca))
        }
        Shape::Box { half_extents } => {
            let p0 = iso.inverse_transform_point(a0);
            let p1 = iso.inverse_transform_point(a1);
            let (ps, pb, d2) = if p0 == p1 {
                let q = p0.clamp(-*half_extents, *half_extents);
                (p0, q, (p0 - q).length_sq())
            } else {
                closest_segment_box(p0, p1, *half_extents)
            };
            if d2.0 <= 0 {
                return None;
            }
            let (n, d) = (ps - pb).normalize_with_len();
            if d <= TINY {
                return None;
            }
            Some((d - r, iso.transform_vector(n), iso.transform_point(ps)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quat;

    #[test]
    fn sphere_on_box() {
        let s = Shape::sphere(Fx::HALF);
        let b = Shape::cuboid(Vec3::from_int(5, 1, 5));
        let mut c = Contacts::default();
        let hit = collide_convex(&s, &Iso::new(Vec3::from_f32(0.0, 1.4, 0.0), Quat::IDENTITY), &b, &Iso::IDENTITY, fx!(0.02), &mut c);
        assert!(hit);
        assert_eq!(c.count, 1);
        assert!((c.normal - Vec3::DOWN).length() < fx!(1e-4));
        assert!((c.points[0].depth - fx!(0.1)).abs() < fx!(1e-4));
    }

    #[test]
    fn box_on_box_face() {
        let a = Shape::cuboid(Vec3::splat(Fx::HALF));
        let g = Shape::cuboid(Vec3::from_int(10, 1, 10));
        let mut c = Contacts::default();
        let hit = collide_convex(&a, &Iso::new(Vec3::from_f32(0.0, 1.49, 0.0), Quat::IDENTITY), &g, &Iso::IDENTITY, fx!(0.02), &mut c);
        assert!(hit);
        assert_eq!(c.count, 4);
        assert!((c.normal - Vec3::DOWN).length() < fx!(1e-4), "{:?}", c.normal);
        for p in &c.points[..4] {
            assert!((p.depth - fx!(0.01)).abs() < fx!(1e-4), "{:?}", p.depth);
        }
    }

    #[test]
    fn capsule_lying_on_box_has_two_points() {
        let cap = Shape::capsule(Fx::ONE, fx!(0.3));
        let g = Shape::cuboid(Vec3::from_int(10, 1, 10));
        let rot = Quat::from_axis_angle(Vec3::Z, Fx::HALF_PI);
        let mut c = Contacts::default();
        assert!(collide_convex(&cap, &Iso::new(Vec3::from_f32(0.0, 1.29, 0.0), rot), &g, &Iso::IDENTITY, fx!(0.02), &mut c));
        assert_eq!(c.count, 2);
    }
}
