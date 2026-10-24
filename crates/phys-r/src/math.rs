//! Fixed-point vector, quaternion and matrix types.
//!
//! Coordinate conventions follow Unity: Y is up, left-handed, and a capsule's
//! axis is its local Y axis. The math itself is handedness agnostic, so data can
//! be passed straight through from Unity without conversion.

use crate::fixed::{isqrt_u128, Fx, FRAC_BITS};
use core::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

const HALF: i128 = 1 << (FRAC_BITS - 1);

#[inline(always)]
fn shr_round(v: i128) -> Fx {
    Fx(((v + HALF) >> FRAC_BITS) as i64)
}

#[derive(Copy, Clone, Default, PartialEq, Eq, Hash, Debug)]
#[repr(C)]
pub struct Vec3 {
    pub x: Fx,
    pub y: Fx,
    pub z: Fx,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3::new(Fx::ZERO, Fx::ZERO, Fx::ZERO);
    pub const ONE: Vec3 = Vec3::new(Fx::ONE, Fx::ONE, Fx::ONE);
    pub const X: Vec3 = Vec3::new(Fx::ONE, Fx::ZERO, Fx::ZERO);
    pub const Y: Vec3 = Vec3::new(Fx::ZERO, Fx::ONE, Fx::ZERO);
    pub const Z: Vec3 = Vec3::new(Fx::ZERO, Fx::ZERO, Fx::ONE);
    pub const UP: Vec3 = Vec3::Y;
    pub const DOWN: Vec3 = Vec3::new(Fx::ZERO, Fx::NEG_ONE, Fx::ZERO);

    #[inline(always)]
    pub const fn new(x: Fx, y: Fx, z: Fx) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[inline(always)]
    pub const fn splat(v: Fx) -> Vec3 {
        Vec3 { x: v, y: v, z: v }
    }

    pub fn from_f32(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3::new(Fx::from_f32(x), Fx::from_f32(y), Fx::from_f32(z))
    }

    pub fn from_int(x: i32, y: i32, z: i32) -> Vec3 {
        Vec3::new(Fx::from_int(x), Fx::from_int(y), Fx::from_int(z))
    }

    pub fn to_f32(self) -> [f32; 3] {
        [self.x.to_f32(), self.y.to_f32(), self.z.to_f32()]
    }

    #[inline(always)]
    pub fn get(self, i: usize) -> Fx {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }

    #[inline(always)]
    pub fn set(&mut self, i: usize, v: Fx) {
        match i {
            0 => self.x = v,
            1 => self.y = v,
            _ => self.z = v,
        }
    }

    /// Dot product, accumulated at full precision and rounded once.
    #[inline(always)]
    pub fn dot(self, o: Vec3) -> Fx {
        shr_round(self.x.0 as i128 * o.x.0 as i128 + self.y.0 as i128 * o.y.0 as i128 + self.z.0 as i128 * o.z.0 as i128)
    }

    #[inline(always)]
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            shr_round(self.y.0 as i128 * o.z.0 as i128 - self.z.0 as i128 * o.y.0 as i128),
            shr_round(self.z.0 as i128 * o.x.0 as i128 - self.x.0 as i128 * o.z.0 as i128),
            shr_round(self.x.0 as i128 * o.y.0 as i128 - self.y.0 as i128 * o.x.0 as i128),
        )
    }

    #[inline(always)]
    pub fn length_sq(self) -> Fx {
        self.dot(self)
    }

    #[inline(always)]
    fn raw_len_sq(self) -> u128 {
        let x = self.x.0 as i128;
        let y = self.y.0 as i128;
        let z = self.z.0 as i128;
        (x * x + y * y + z * z) as u128
    }

    /// Exact length (floor at 2^-24), precise even for tiny vectors.
    #[inline]
    pub fn length(self) -> Fx {
        Fx(isqrt_u128(self.raw_len_sq()) as i64)
    }

    #[inline]
    pub fn distance(self, o: Vec3) -> Fx {
        (self - o).length()
    }

    #[inline]
    pub fn distance_sq(self, o: Vec3) -> Fx {
        (self - o).length_sq()
    }

    /// Normalised copy, or zero if the vector is (nearly) zero.
    #[inline]
    pub fn normalize_or_zero(self) -> Vec3 {
        self.normalize_with_len().0
    }

    /// Returns `(unit_vector, length)`; unit vector is zero if length is tiny.
    #[inline]
    pub fn normalize_with_len(self) -> (Vec3, Fx) {
        let len = self.length();
        if len.0 <= 16 {
            return (Vec3::ZERO, len);
        }
        (Vec3::new(self.x / len, self.y / len, self.z / len), len)
    }

    #[inline]
    pub fn normalize_or(self, fallback: Vec3) -> Vec3 {
        let (n, len) = self.normalize_with_len();
        if len.0 <= 16 {
            fallback
        } else {
            n
        }
    }

    #[inline(always)]
    pub fn scale(self, s: Fx) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }

    #[inline(always)]
    pub fn mul_elem(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }

    #[inline(always)]
    pub fn min(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    #[inline(always)]
    pub fn max(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    #[inline(always)]
    pub fn abs(self) -> Vec3 {
        Vec3::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    #[inline(always)]
    pub fn clamp(self, lo: Vec3, hi: Vec3) -> Vec3 {
        self.max(lo).min(hi)
    }

    #[inline]
    pub fn lerp(a: Vec3, b: Vec3, t: Fx) -> Vec3 {
        a + (b - a).scale(t)
    }

    #[inline(always)]
    pub fn half(self) -> Vec3 {
        Vec3::new(self.x.shr(1), self.y.shr(1), self.z.shr(1))
    }

    #[inline(always)]
    pub fn is_zero(self) -> bool {
        self.x.0 == 0 && self.y.0 == 0 && self.z.0 == 0
    }

    #[inline(always)]
    pub fn max_abs_component(self) -> Fx {
        self.x.abs().max(self.y.abs()).max(self.z.abs())
    }

    /// Two unit vectors orthogonal to `self` (which must be unit length).
    /// Pure function of the input, so it is deterministic.
    #[inline]
    pub fn orthonormal_basis(self) -> (Vec3, Vec3) {
        let t1 = if self.x.abs() >= Fx::from_f64_const(0.57735) {
            Vec3::new(self.y, -self.x, Fx::ZERO)
        } else {
            Vec3::new(Fx::ZERO, self.z, -self.y)
        }
        .normalize_or_zero();
        let t2 = self.cross(t1);
        (t1, t2)
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}
impl Mul<Fx> for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn mul(self, s: Fx) -> Vec3 {
        self.scale(s)
    }
}
impl AddAssign for Vec3 {
    #[inline(always)]
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
impl SubAssign for Vec3 {
    #[inline(always)]
    fn sub_assign(&mut self, o: Vec3) {
        *self = *self - o;
    }
}

/// Unit quaternion (x, y, z, w).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
#[repr(C)]
pub struct Quat {
    pub x: Fx,
    pub y: Fx,
    pub z: Fx,
    pub w: Fx,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { x: Fx::ZERO, y: Fx::ZERO, z: Fx::ZERO, w: Fx::ONE };

    #[inline(always)]
    pub const fn new(x: Fx, y: Fx, z: Fx, w: Fx) -> Quat {
        Quat { x, y, z, w }
    }

    #[inline(always)]
    pub fn xyz(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    /// Rotation of `angle` radians around unit `axis`.
    pub fn from_axis_angle(axis: Vec3, angle: Fx) -> Quat {
        let (s, c) = angle.shr(1).sin_cos();
        let a = axis.normalize_or(Vec3::Y);
        Quat::new(a.x * s, a.y * s, a.z * s, c)
    }

    /// Rotation around the world Y axis (FPS yaw).
    pub fn from_yaw(yaw: Fx) -> Quat {
        let (s, c) = yaw.shr(1).sin_cos();
        Quat::new(Fx::ZERO, s, Fx::ZERO, c)
    }

    /// Euler angles in radians using Unity's convention (Z, then X, then Y).
    pub fn from_euler(x: Fx, y: Fx, z: Fx) -> Quat {
        let qx = Quat::from_axis_angle(Vec3::X, x);
        let qy = Quat::from_axis_angle(Vec3::Y, y);
        let qz = Quat::from_axis_angle(Vec3::Z, z);
        qy.mul(qx).mul(qz).normalize()
    }

    #[inline(always)]
    pub fn conjugate(self) -> Quat {
        Quat::new(-self.x, -self.y, -self.z, self.w)
    }

    #[inline]
    pub fn dot(self, o: Quat) -> Fx {
        shr_round(
            self.x.0 as i128 * o.x.0 as i128
                + self.y.0 as i128 * o.y.0 as i128
                + self.z.0 as i128 * o.z.0 as i128
                + self.w.0 as i128 * o.w.0 as i128,
        )
    }

    #[inline]
    #[allow(clippy::should_implement_trait)]
    pub fn mul(self, o: Quat) -> Quat {
        let (ax, ay, az, aw) = (self.x.0 as i128, self.y.0 as i128, self.z.0 as i128, self.w.0 as i128);
        let (bx, by, bz, bw) = (o.x.0 as i128, o.y.0 as i128, o.z.0 as i128, o.w.0 as i128);
        Quat::new(
            shr_round(aw * bx + ax * bw + ay * bz - az * by),
            shr_round(aw * by - ax * bz + ay * bw + az * bx),
            shr_round(aw * bz + ax * by - ay * bx + az * bw),
            shr_round(aw * bw - ax * bx - ay * by - az * bz),
        )
    }

    /// Rotate a vector.
    #[inline]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        // Identity fast path (axis-aligned level geometry); bit-identical to
        // the general formula, which reduces to `v` when xyz == 0.
        if (self.x.0 | self.y.0 | self.z.0) == 0 {
            return v;
        }
        let q = self.xyz();
        let t = q.cross(v);
        let t = t + t;
        v + t.scale(self.w) + q.cross(t)
    }

    #[inline]
    pub fn inverse_rotate(self, v: Vec3) -> Vec3 {
        self.conjugate().rotate(v)
    }

    pub fn normalize(self) -> Quat {
        let x = self.x.0 as i128;
        let y = self.y.0 as i128;
        let z = self.z.0 as i128;
        let w = self.w.0 as i128;
        let len = Fx(isqrt_u128((x * x + y * y + z * z + w * w) as u128) as i64);
        if len.0 <= 16 {
            return Quat::IDENTITY;
        }
        Quat::new(self.x / len, self.y / len, self.z / len, self.w / len)
    }

    /// Integrate angular velocity `w` (rad/s, world space) over `dt`.
    #[inline]
    pub fn integrate(self, w: Vec3, dt: Fx) -> Quat {
        if w.is_zero() {
            return self;
        }
        let h = dt.shr(1);
        let dq = Quat::new(w.x * h, w.y * h, w.z * h, Fx::ZERO).mul(self);
        Quat::new(self.x + dq.x, self.y + dq.y, self.z + dq.z, self.w + dq.w).normalize()
    }

    /// Normalised linear interpolation along the shortest arc.
    pub fn nlerp(a: Quat, b: Quat, t: Fx) -> Quat {
        let b = if a.dot(b).is_negative() { Quat::new(-b.x, -b.y, -b.z, -b.w) } else { b };
        Quat::new(Fx::lerp(a.x, b.x, t), Fx::lerp(a.y, b.y, t), Fx::lerp(a.z, b.z, t), Fx::lerp(a.w, b.w, t)).normalize()
    }

    pub fn to_mat3(self) -> Mat3 {
        let (x, y, z, w) = (self.x, self.y, self.z, self.w);
        let (x2, y2, z2) = (x + x, y + y, z + z);
        let (xx, yy, zz) = (x * x2, y * y2, z * z2);
        let (xy, xz, yz) = (x * y2, x * z2, y * z2);
        let (wx, wy, wz) = (w * x2, w * y2, w * z2);
        Mat3 {
            r: [
                Vec3::new(Fx::ONE - (yy + zz), xy - wz, xz + wy),
                Vec3::new(xy + wz, Fx::ONE - (xx + zz), yz - wx),
                Vec3::new(xz - wy, yz + wx, Fx::ONE - (xx + yy)),
            ],
        }
    }
}

/// Row-major 3x3 matrix.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct Mat3 {
    pub r: [Vec3; 3],
}

impl Mat3 {
    pub const ZERO: Mat3 = Mat3 { r: [Vec3::ZERO; 3] };
    pub const IDENTITY: Mat3 = Mat3 { r: [Vec3::X, Vec3::Y, Vec3::Z] };

    #[inline(always)]
    pub fn mul_vec(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.r[0].dot(v), self.r[1].dot(v), self.r[2].dot(v))
    }

    #[inline]
    pub fn col(&self, i: usize) -> Vec3 {
        Vec3::new(self.r[0].get(i), self.r[1].get(i), self.r[2].get(i))
    }

    pub fn transpose(&self) -> Mat3 {
        Mat3 { r: [self.col(0), self.col(1), self.col(2)] }
    }

    /// `R * diag(d) * R^T` — world-space inverse inertia from a local diagonal.
    pub fn rotate_diagonal(rot: &Mat3, d: Vec3) -> Mat3 {
        let mut m = Mat3::ZERO;
        for i in 0..3 {
            let ri = rot.r[i].mul_elem(d);
            for j in i..3 {
                let v = ri.dot(rot.r[j]);
                m.r[i].set(j, v);
                m.r[j].set(i, v);
            }
        }
        m
    }
}

/// Rigid transform (rotation then translation).
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[repr(C)]
pub struct Iso {
    pub pos: Vec3,
    pub rot: Quat,
}

impl Iso {
    pub const IDENTITY: Iso = Iso { pos: Vec3::ZERO, rot: Quat::IDENTITY };

    #[inline(always)]
    pub const fn new(pos: Vec3, rot: Quat) -> Iso {
        Iso { pos, rot }
    }

    #[inline(always)]
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        self.rot.rotate(p) + self.pos
    }

    #[inline(always)]
    pub fn inverse_transform_point(&self, p: Vec3) -> Vec3 {
        self.rot.inverse_rotate(p - self.pos)
    }

    #[inline(always)]
    pub fn transform_vector(&self, v: Vec3) -> Vec3 {
        self.rot.rotate(v)
    }

    #[inline(always)]
    pub fn inverse_transform_vector(&self, v: Vec3) -> Vec3 {
        self.rot.inverse_rotate(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fx;

    #[test]
    fn rotate_and_back() {
        let q = Quat::from_euler(fx!(0.3), fx!(1.2), fx!(-0.7));
        let v = Vec3::from_f32(1.0, 2.0, 3.0);
        let r = q.rotate(v);
        let b = q.inverse_rotate(r);
        assert!((b - v).length() < fx!(1e-5));
        let m = q.to_mat3();
        assert!((m.mul_vec(v) - r).length() < fx!(1e-5));
    }

    #[test]
    fn yaw_matches_unity() {
        // Unity: Quaternion.Euler(0, 90, 0) * Vector3.forward == Vector3.right
        let q = Quat::from_yaw(Fx::HALF_PI);
        let f = q.rotate(Vec3::Z);
        assert!((f - Vec3::X).length() < fx!(1e-5), "{f:?}");
    }

    #[test]
    fn tiny_lengths_are_precise() {
        let v = Vec3::new(Fx(3), Fx(4), Fx(0));
        assert_eq!(v.length(), Fx(5));
    }
}
