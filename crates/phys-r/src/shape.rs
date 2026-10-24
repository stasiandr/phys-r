//! Collision shapes: convex primitives only (sphere, capsule, box).
//!
//! Levels are built from primitives too: static bodies live in a BVH, so a
//! map made of a few hundred boxes is cheaper to query than a triangle mesh
//! and every collision pair has a fast dedicated routine.

use crate::fixed::Fx;
use crate::fx;
use crate::geometry::Aabb;
use crate::math::{Iso, Vec3};

/// Index into the world's shape registry. Shapes are immutable and shared.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Default)]
pub struct ShapeId(pub u32);

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Shape {
    Sphere {
        radius: Fx,
    },
    /// Capsule along the local Y axis. `half_height` is half the length of the
    /// inner segment (total height = 2 * (half_height + radius)).
    Capsule {
        half_height: Fx,
        radius: Fx,
    },
    Box {
        half_extents: Vec3,
    },
}

impl Shape {
    pub fn sphere(radius: Fx) -> Shape {
        Shape::Sphere { radius }
    }

    pub fn capsule(half_height: Fx, radius: Fx) -> Shape {
        Shape::Capsule { half_height, radius }
    }

    pub fn cuboid(half_extents: Vec3) -> Shape {
        Shape::Box { half_extents }
    }

    /// World-space bounding box at `iso`.
    #[inline]
    pub fn aabb(&self, iso: &Iso) -> Aabb {
        match self {
            Shape::Sphere { radius } => Aabb::from_center_half(iso.pos, Vec3::splat(*radius)),
            Shape::Capsule { half_height, radius } => {
                let a = iso.rot.rotate(Vec3::new(Fx::ZERO, *half_height, Fx::ZERO)).abs();
                Aabb::from_center_half(iso.pos, a + Vec3::splat(*radius))
            }
            Shape::Box { half_extents } => {
                let m = iso.rot.to_mat3();
                let h = Vec3::new(m.r[0].abs().dot(*half_extents), m.r[1].abs().dot(*half_extents), m.r[2].abs().dot(*half_extents));
                Aabb::from_center_half(iso.pos, h)
            }
        }
    }

    /// Diagonal of the inverse inertia tensor for a body of `mass`.
    pub fn inverse_inertia(&self, mass: Fx) -> Vec3 {
        if mass.0 <= 0 {
            return Vec3::ZERO;
        }
        let i = match self {
            Shape::Sphere { radius } => Vec3::splat(fx!(0.4) * mass * *radius * *radius),
            Shape::Capsule { half_height, radius } => {
                // Cylinder + two hemispheres, mass split by volume.
                let r = *radius;
                let h = half_height.shl(1);
                let vc = h; // pi r^2 h  (common factor pi r^2 dropped)
                let vs = fx!(4.0 / 3.0) * r; // 4/3 pi r^3 / (pi r^2)
                let total = vc + vs;
                let mc = if total.0 > 0 { mass * vc / total } else { Fx::ZERO };
                let ms = mass - mc;
                let r2 = r * r;
                let iy = mc * r2 * Fx::HALF + ms * r2 * fx!(0.4);
                let ixz = mc * (r2 * fx!(0.25) + h * h / Fx::from_int(12))
                    + ms * (r2 * fx!(0.4) + (*half_height) * (*half_height) + fx!(0.375) * r * h);
                Vec3::new(ixz, iy, ixz)
            }
            Shape::Box { half_extents: e } => {
                let k = mass / Fx::from_int(3);
                Vec3::new(k * (e.y * e.y + e.z * e.z), k * (e.x * e.x + e.z * e.z), k * (e.x * e.x + e.y * e.y))
            }
        };
        let inv = |v: Fx| if v.0 > 0 { Fx::ONE / v } else { Fx::ZERO };
        Vec3::new(inv(i.x), inv(i.y), inv(i.z))
    }
}
