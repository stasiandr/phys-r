//! Rigid bodies.

use crate::fixed::Fx;
use crate::fx;
use crate::geometry::Aabb;
use crate::math::{Iso, Quat, Vec3};
use crate::shape::ShapeId;

/// Generational handle to a body. Encodes as a `u64` for FFI
/// (`generation << 32 | index`); `0` is never a valid handle.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Default)]
pub struct BodyHandle {
    pub index: u32,
    pub generation: u32,
}

impl BodyHandle {
    pub const INVALID: BodyHandle = BodyHandle { index: 0, generation: 0 };

    #[inline]
    pub fn to_bits(self) -> u64 {
        ((self.generation as u64) << 32) | self.index as u64
    }

    #[inline]
    pub fn from_bits(b: u64) -> BodyHandle {
        BodyHandle { index: b as u32, generation: (b >> 32) as u32 }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[repr(u8)]
pub enum BodyKind {
    /// Never moves. Lives in the static BVH.
    #[default]
    Static = 0,
    /// Simulated.
    Dynamic = 1,
    /// Moved by the user (velocity or target pose); pushes dynamic bodies.
    Kinematic = 2,
}

impl BodyKind {
    pub fn from_u8(v: u8) -> BodyKind {
        match v {
            1 => BodyKind::Dynamic,
            2 => BodyKind::Kinematic,
            _ => BodyKind::Static,
        }
    }
}

/// Body behaviour flags.
pub mod flags {
    /// Trigger volume: reports overlaps, no collision response.
    pub const SENSOR: u32 = 1 << 0;
    /// Expand the speculative margin by the body's velocity (fast projectiles).
    pub const CCD: u32 = 1 << 1;
    /// Record poses in the lag-compensation history.
    pub const HISTORY: u32 = 1 << 2;
    /// Never fall asleep.
    pub const NO_SLEEP: u32 = 1 << 3;
    /// Lock rotation (infinite inertia), e.g. for upright props.
    pub const LOCK_ROTATION: u32 = 1 << 4;
    /// Report contact begin/end events for this body.
    pub const CONTACT_EVENTS: u32 = 1 << 5;
    /// Internal: body is owned by a character controller.
    pub const CHARACTER: u32 = 1 << 30;
}

/// Parameters for creating a body.
#[derive(Clone, Debug)]
pub struct BodyDesc {
    pub kind: BodyKind,
    pub shape: ShapeId,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    /// Mass in kg (dynamic bodies only).
    pub mass: Fx,
    pub friction: Fx,
    pub restitution: Fx,
    pub linear_damping: Fx,
    pub angular_damping: Fx,
    pub gravity_scale: Fx,
    /// Collision layer (0..31).
    pub layer: u8,
    /// Bitmask of layers this body collides with.
    pub mask: u32,
    pub flags: u32,
    pub user_data: u64,
}

impl Default for BodyDesc {
    fn default() -> Self {
        BodyDesc {
            kind: BodyKind::Dynamic,
            shape: ShapeId(0),
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: Fx::ONE,
            friction: fx!(0.6),
            restitution: Fx::ZERO,
            linear_damping: fx!(0.01),
            angular_damping: fx!(0.05),
            gravity_scale: Fx::ONE,
            layer: 0,
            mask: u32::MAX,
            flags: 0,
            user_data: 0,
        }
    }
}

impl BodyDesc {
    pub fn new_static(shape: ShapeId, position: Vec3) -> BodyDesc {
        BodyDesc { kind: BodyKind::Static, shape, position, ..Default::default() }
    }

    pub fn new_dynamic(shape: ShapeId, position: Vec3, mass: Fx) -> BodyDesc {
        BodyDesc { kind: BodyKind::Dynamic, shape, position, mass, ..Default::default() }
    }

    pub fn new_kinematic(shape: ShapeId, position: Vec3) -> BodyDesc {
        BodyDesc { kind: BodyKind::Kinematic, shape, position, ..Default::default() }
    }
}

/// Simulation state of one body. All fields that influence the simulation are
/// included in snapshots.
#[derive(Clone, Debug, Default)]
pub struct Body {
    pub(crate) generation: u32,
    pub(crate) alive: bool,
    pub kind: BodyKind,
    pub shape: ShapeId,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub(crate) force: Vec3,
    pub(crate) torque: Vec3,
    pub(crate) mass: Fx,
    pub(crate) inv_mass: Fx,
    pub(crate) inv_inertia_local: Vec3,
    pub friction: Fx,
    pub restitution: Fx,
    pub linear_damping: Fx,
    pub angular_damping: Fx,
    pub gravity_scale: Fx,
    pub layer: u8,
    pub mask: u32,
    pub flags: u32,
    pub(crate) sleeping: bool,
    pub(crate) sleep_time: Fx,
    pub user_data: u64,
    /// Kinematic target pose reached exactly at the end of the next step.
    pub(crate) has_target: bool,
    pub(crate) target: Iso,
    // Derived data (recomputed every step, not part of snapshots).
    pub(crate) aabb: Aabb,
}

impl Body {
    #[inline]
    pub fn iso(&self) -> Iso {
        Iso::new(self.position, self.rotation)
    }

    #[inline]
    pub fn is_dynamic(&self) -> bool {
        self.kind == BodyKind::Dynamic
    }

    #[inline]
    pub fn is_static(&self) -> bool {
        self.kind == BodyKind::Static
    }

    #[inline]
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    #[inline]
    pub fn is_sensor(&self) -> bool {
        self.flags & flags::SENSOR != 0
    }

    #[inline]
    pub fn mass(&self) -> Fx {
        self.mass
    }

    #[inline]
    pub fn aabb(&self) -> Aabb {
        self.aabb
    }

    #[inline]
    pub fn handle(&self, index: u32) -> BodyHandle {
        BodyHandle { index, generation: self.generation }
    }
}

/// Do two bodies' layer filters accept each other?
#[inline(always)]
pub fn layers_interact(la: u8, ma: u32, lb: u8, mb: u32) -> bool {
    (ma & (1u32 << (lb & 31))) != 0 && (mb & (1u32 << (la & 31))) != 0
}
