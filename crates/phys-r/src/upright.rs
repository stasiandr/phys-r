//! Exact closed-form queries for an **upright** capsule (inner segment along
//! world Y, as used by characters) against primitives whose local Y axis is
//! vertical (identity or yaw-only rotation: almost all level geometry).
//!
//! An upright capsule (centre `c`, half segment `hh`, radius `r`) against a
//! yaw-aligned box of half extents `h` is the point `c` against the Minkowski
//! sum: a "tall" box `(h.x, h.y + hh, h.z)` rounded by `r`. Sweeping becomes a
//! ray against a rounded box (Ericson, RTCD 5.5.7) and penetration a point vs
//! box test, with no iteration. Upright capsules against each other reduce to
//! a point against a vertical capsule the same way.
//!
//! Callers fall back to the general routines when a shape is not upright
//! (`None` / [`Sweep::Unsupported`]).

use crate::fixed::Fx;
use crate::fx;
use crate::geometry::closest_on_segment;
use crate::math::{Iso, Quat, Vec3};
use crate::query::{ray_capsule, ray_sphere};
use crate::shape::Shape;

const TINY: Fx = fx!(1e-5);

/// Precomputed sweep parameters (shared by every obstacle of one sweep).
#[derive(Copy, Clone)]
pub(crate) struct UpSweep {
    pub c: Vec3,
    pub hh: Fx,
    pub r: Fx,
    /// Unit direction and length of `disp`.
    pub dir: Vec3,
    pub len: Fx,
    pub skin: Fx,
}

impl UpSweep {
    /// `a0`/`a1` must form a vertical segment (`a0` below `a1`).
    #[inline]
    pub fn new(a0: Vec3, a1: Vec3, r: Fx, disp: Vec3, skin: Fx) -> Option<UpSweep> {
        if a0.x != a1.x || a0.z != a1.z {
            return None;
        }
        let (dir, len) = disp.normalize_with_len();
        if len.0 <= 16 {
            return None;
        }
        let hh = (a1.y - a0.y).abs().shr(1);
        Some(UpSweep { c: (a0 + a1).half(), hh, r, dir, len, skin })
    }
}

pub(crate) enum Sweep {
    Miss,
    /// `(fraction of disp, normal toward the capsule, contact point)`.
    Hit(Fx, Vec3, Vec3),
    Unsupported,
}

#[inline(always)]
fn yaw_only(q: &Quat) -> bool {
    q.x.0 == 0 && q.z.0 == 0
}

/// Upright-capsule obstacle data in the obstacle's local frame.
enum Local {
    /// Tall box half extents and the real box half extents.
    Box { tall: Vec3, h: Vec3 },
    /// Vertical segment half length and combined core radius.
    Seg { half: Fx, rad: Fx },
}

#[inline]
fn local_shape(shape: &Shape, iso: &Iso, hh: Fx) -> Option<Local> {
    match shape {
        Shape::Box { half_extents } if yaw_only(&iso.rot) => {
            let h = *half_extents;
            Some(Local::Box { tall: Vec3::new(h.x, h.y + hh, h.z), h })
        }
        Shape::Capsule { half_height, radius } if yaw_only(&iso.rot) => Some(Local::Seg { half: *half_height + hh, rad: *radius }),
        Shape::Sphere { radius } => Some(Local::Seg { half: hh, rad: *radius }),
        _ => None,
    }
}

/// Distance from the capsule surface to the obstacle: `(distance, normal
/// toward the capsule, closest point on the capsule core)`, or `Some(None)`
/// when the cores overlap; `None` if not supported.
#[inline]
pub(crate) fn distance(c: Vec3, hh: Fx, r: Fx, shape: &Shape, iso: &Iso) -> Option<Option<(Fx, Vec3, Vec3)>> {
    let l = local_shape(shape, iso, hh)?;
    let o = iso.inverse_transform_point(c);
    Some(match l {
        Local::Box { tall, .. } => {
            let q = o.clamp(-tall, tall);
            let (n, d) = (o - q).normalize_with_len();
            if d <= TINY {
                None
            } else {
                let core = Vec3::new(o.x, o.y - (o.y - q.y).clamp(-hh, hh), o.z);
                Some((d - r, iso.transform_vector(n), iso.transform_point(core)))
            }
        }
        Local::Seg { half, rad } => {
            let (_, q) = closest_on_segment(o, Vec3::new(Fx::ZERO, -half, Fx::ZERO), Vec3::new(Fx::ZERO, half, Fx::ZERO));
            let (n, d) = (o - q).normalize_with_len();
            if d <= TINY {
                None
            } else {
                let core = Vec3::new(o.x, o.y - (o.y - q.y).clamp(-hh, hh), o.z);
                Some((d - r - rad, iso.transform_vector(n), iso.transform_point(core)))
            }
        }
    })
}

/// Penetration `(depth, push direction)` of the capsule into the obstacle;
/// `Some(None)` if separated, `None` if not supported.
#[inline]
pub(crate) fn penetration(c: Vec3, hh: Fx, r: Fx, shape: &Shape, iso: &Iso) -> Option<Option<(Fx, Vec3)>> {
    let l = local_shape(shape, iso, hh)?;
    let o = iso.inverse_transform_point(c);
    Some(match l {
        Local::Box { tall, .. } => {
            let q = o.clamp(-tall, tall);
            let (n, d) = (o - q).normalize_with_len();
            if d > TINY {
                if d >= r {
                    None
                } else {
                    Some((r - d, iso.transform_vector(n)))
                }
            } else {
                // Centre inside the tall box: exit through the nearest face.
                let (axis, pen) = min_face(o, tall);
                let mut n = Vec3::ZERO;
                n.set(axis, o.get(axis).sign_nonzero());
                Some((r + pen, iso.transform_vector(n)))
            }
        }
        Local::Seg { half, rad } => {
            let (_, q) = closest_on_segment(o, Vec3::new(Fx::ZERO, -half, Fx::ZERO), Vec3::new(Fx::ZERO, half, Fx::ZERO));
            let rr = r + rad;
            let (n, d) = (o - q).normalize_with_len();
            if d >= rr {
                None
            } else {
                let n = if d <= TINY { Vec3::X } else { n };
                Some((rr - d, iso.transform_vector(n)))
            }
        }
    })
}

#[inline]
fn min_face(o: Vec3, tall: Vec3) -> (usize, Fx) {
    let mut best = (0, Fx::MAX);
    for i in 0..3 {
        let p = tall.get(i) - o.get(i).abs();
        if p < best.1 {
            best = (i, p);
        }
    }
    best
}

/// Sweep against one obstacle, stopping `skin` short of contact. Matches the
/// conservative-advancement semantics: when already within `skin`, it is a
/// hit at `t = 0` only if moving toward the obstacle.
#[inline]
pub(crate) fn sweep(s: &UpSweep, shape: &Shape, iso: &Iso) -> Sweep {
    let Some(l) = local_shape(shape, iso, s.hh) else { return Sweep::Unsupported };
    let o = iso.inverse_transform_point(s.c);
    let d = iso.inverse_transform_vector(s.dir);
    let big_r = s.r + s.skin;
    match l {
        Local::Box { tall, h } => {
            // Start state.
            let q = o.clamp(-tall, tall);
            let (n0, d0) = (o - q).normalize_with_len();
            if d0 <= big_r {
                let n = if d0 > TINY {
                    n0
                } else {
                    let (axis, _) = min_face(o, tall);
                    let mut n = Vec3::ZERO;
                    n.set(axis, o.get(axis).sign_nonzero());
                    n
                };
                return if d.dot(n) < Fx::ZERO {
                    Sweep::Hit(Fx::ZERO, iso.transform_vector(n), contact(iso, o, n, s.r, h))
                } else {
                    Sweep::Miss
                };
            }
            // Ray vs the box expanded by R.
            let e = tall + Vec3::splat(big_r);
            let mut t0 = Fx::ZERO;
            let mut t1 = s.len;
            let mut axis = 0usize;
            for i in 0..3 {
                let oi = o.get(i);
                let di = d.get(i);
                let ei = e.get(i);
                if di.abs().0 < 64 {
                    if oi < -ei || oi > ei {
                        return Sweep::Miss;
                    }
                    continue;
                }
                let mut a = (-ei - oi) / di;
                let mut b = (ei - oi) / di;
                if a > b {
                    core::mem::swap(&mut a, &mut b);
                }
                if a > t0 {
                    t0 = a;
                    axis = i;
                }
                t1 = t1.min(b);
                if t0 > t1 {
                    return Sweep::Miss;
                }
            }
            let p = o + d.scale(t0);
            let mut out = [false; 3];
            let mut count = 0;
            for i in 0..3 {
                if p.get(i).abs() > tall.get(i) {
                    out[i] = true;
                    count += 1;
                }
            }
            if count <= 1 {
                let mut n = Vec3::ZERO;
                n.set(axis, p.get(axis).sign_nonzero());
                if d.dot(n) >= Fx::ZERO {
                    return Sweep::Miss;
                }
                return Sweep::Hit(t0 / s.len, iso.transform_vector(n), contact(iso, p, n, s.r, h));
            }
            // Edge / corner region: the rounded part is the union of edge
            // capsules; test the edge (or the three edges at a corner).
            let mut best: Option<(Fx, Vec3)> = None;
            let corner = Vec3::new(p.x.sign_nonzero() * tall.x, p.y.sign_nonzero() * tall.y, p.z.sign_nonzero() * tall.z);
            for k in 0..3 {
                // Edge along axis k: needs the other two axes outside.
                let (u, w) = ((k + 1) % 3, (k + 2) % 3);
                if !(out[u] && out[w]) {
                    continue;
                }
                let mut a = corner;
                let mut b = corner;
                a.set(k, -tall.get(k));
                b.set(k, tall.get(k));
                if let Some((t, _)) = ray_capsule(o, d, a, b, big_r, s.len) {
                    if best.is_none_or(|(bt, _)| t < bt) {
                        let hp = o + d.scale(t);
                        let (_, q) = closest_on_segment(hp, a, b);
                        best = Some((t, (hp - q).normalize_or(-d)));
                    }
                }
            }
            match best {
                // Moving away from (or along) the surface is never a hit; this
                // also resolves borderline starts exactly at the skin distance.
                Some((t, n)) if d.dot(n) < Fx::ZERO => {
                    Sweep::Hit(t / s.len, iso.transform_vector(n), contact(iso, o + d.scale(t), n, s.r, h))
                }
                _ => Sweep::Miss,
            }
        }
        Local::Seg { half, rad } => {
            let a = Vec3::new(Fx::ZERO, -half, Fx::ZERO);
            let b = Vec3::new(Fx::ZERO, half, Fx::ZERO);
            let rr = big_r + rad;
            let (_, q) = closest_on_segment(o, a, b);
            let (n0, d0) = (o - q).normalize_with_len();
            if d0 <= rr {
                let n = if d0 > TINY { n0 } else { Vec3::X };
                return if d.dot(n) < Fx::ZERO {
                    Sweep::Hit(Fx::ZERO, iso.transform_vector(n), iso.transform_point(o - n.scale(s.r)))
                } else {
                    Sweep::Miss
                };
            }
            let hit = if half.0 == 0 { ray_sphere(o, d, a, rr, s.len) } else { ray_capsule(o, d, a, b, rr, s.len) };
            match hit {
                Some((t, n)) if d.dot(n) < Fx::ZERO => {
                    let hp = o + d.scale(t);
                    Sweep::Hit(t / s.len, iso.transform_vector(n), iso.transform_point(hp - n.scale(s.r)))
                }
                _ => Sweep::Miss,
            }
        }
    }
}

/// Contact point on the real box for a capsule whose Minkowski centre is `p`.
#[inline]
fn contact(iso: &Iso, p: Vec3, n: Vec3, r: Fx, h: Vec3) -> Vec3 {
    let q = p - n.scale(r);
    iso.transform_point(q.clamp(-h, h))
}
