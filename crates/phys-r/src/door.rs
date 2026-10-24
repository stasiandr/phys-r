//! Deterministic doors: hinged or sliding kinematic bodies driven toward an
//! open/closed target at a fixed speed.
//!
//! Door state (current opening, target, blocked flag) lives in the world and
//! is part of snapshots, so doors roll back and replay with everything else.
//! Opening a door is just `set_door_open(h, true)` from game logic (e.g. on a
//! "use" input); the motion itself is simulated identically on every peer.
//!
//! Doors push dynamic props (they are kinematic) and push characters out of
//! the way. With `block_on_characters` a door instead stops when its next
//! pose would overlap a character, like most FPS doors.

use crate::body::{BodyDesc, BodyHandle, BodyKind};
use crate::collide::{collide_convex, Contacts};
use crate::fixed::Fx;
use crate::fx;
use crate::math::{Iso, Quat, Vec3};
use crate::shape::ShapeId;
use crate::world::{World, WorldError};

/// Generational handle to a door.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Default)]
pub struct DoorHandle {
    pub index: u32,
    pub generation: u32,
}

impl DoorHandle {
    pub fn to_bits(self) -> u64 {
        ((self.generation as u64) << 32) | self.index as u64
    }
    pub fn from_bits(b: u64) -> Self {
        DoorHandle { index: b as u32, generation: (b >> 32) as u32 }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[repr(u8)]
pub enum DoorKind {
    /// Rotates about `axis` through `pivot`; opening amount in radians.
    #[default]
    Hinge = 0,
    /// Translates along `axis`; opening amount in metres.
    Slide = 1,
}

impl DoorKind {
    pub fn from_u8(v: u8) -> DoorKind {
        if v == 1 {
            DoorKind::Slide
        } else {
            DoorKind::Hinge
        }
    }
}

#[derive(Clone, Debug)]
pub struct DoorDesc {
    pub shape: ShapeId,
    pub kind: DoorKind,
    /// Hinge: a point on the hinge axis. Slide: the closed body centre.
    pub pivot: Vec3,
    /// Hinge axis or slide direction (normalised internally).
    pub axis: Vec3,
    /// Body rotation when closed.
    pub closed_rotation: Quat,
    /// Hinge only: closed body centre relative to `pivot`.
    pub offset: Vec3,
    /// Fully-open amount (radians or metres; sign picks the direction).
    pub open_amount: Fx,
    /// Opening speed (rad/s or m/s).
    pub speed: Fx,
    /// Stop instead of pushing characters.
    pub block_on_characters: bool,
    pub layer: u8,
    pub mask: u32,
    pub friction: Fx,
    pub user_data: u64,
}

impl Default for DoorDesc {
    fn default() -> Self {
        DoorDesc {
            shape: ShapeId(0),
            kind: DoorKind::Hinge,
            pivot: Vec3::ZERO,
            axis: Vec3::Y,
            closed_rotation: Quat::IDENTITY,
            offset: Vec3::ZERO,
            open_amount: Fx::HALF_PI,
            speed: fx!(2.0),
            block_on_characters: true,
            layer: 0,
            mask: u32::MAX,
            friction: fx!(0.6),
            user_data: 0,
        }
    }
}

/// Door state (part of snapshots).
#[derive(Clone, Debug)]
pub struct Door {
    pub(crate) alive: bool,
    pub(crate) generation: u32,
    pub(crate) body: BodyHandle,
    pub desc: DoorDesc,
    /// Current opening (0 = closed, `open_amount` = open).
    pub amount: Fx,
    /// Opening the door is moving toward.
    pub target: Fx,
    /// A character stopped the door last tick.
    pub blocked: bool,
}

impl Door {
    pub fn body(&self) -> BodyHandle {
        self.body
    }

    /// 0 = closed .. 1 = open.
    pub fn open_fraction(&self) -> Fx {
        if self.desc.open_amount.0 == 0 {
            Fx::ZERO
        } else {
            self.amount / self.desc.open_amount
        }
    }

    pub fn is_moving(&self) -> bool {
        self.amount != self.target
    }

    /// Body pose for an opening `amount`.
    pub fn pose_at(&self, amount: Fx) -> Iso {
        let d = &self.desc;
        match d.kind {
            DoorKind::Hinge => {
                let q = Quat::from_axis_angle(d.axis, amount);
                Iso::new(d.pivot + q.rotate(d.offset), q.mul(d.closed_rotation).normalize())
            }
            DoorKind::Slide => Iso::new(d.pivot + d.axis.scale(amount), d.closed_rotation),
        }
    }
}

impl World {
    pub fn create_door(&mut self, desc: &DoorDesc) -> Result<DoorHandle, WorldError> {
        let mut desc = desc.clone();
        desc.axis = desc.axis.normalize_or(Vec3::Y);
        desc.speed = desc.speed.abs();
        let mut door =
            Door { alive: true, generation: 0, body: BodyHandle::INVALID, desc, amount: Fx::ZERO, target: Fx::ZERO, blocked: false };
        let pose = door.pose_at(Fx::ZERO);
        door.body = self.create_body(&BodyDesc {
            kind: BodyKind::Kinematic,
            shape: door.desc.shape,
            position: pose.pos,
            rotation: pose.rot,
            layer: door.desc.layer,
            mask: door.desc.mask,
            friction: door.desc.friction,
            user_data: door.desc.user_data,
            ..Default::default()
        })?;
        let index = match self.door_free.pop() {
            Some(i) => i,
            None => {
                self.doors.push(Door { alive: false, ..door.clone() });
                self.doors.len() as u32 - 1
            }
        };
        let generation = self.doors[index as usize].generation.wrapping_add(1).max(1);
        self.doors[index as usize] = Door { generation, ..door };
        Ok(DoorHandle { index, generation })
    }

    pub fn destroy_door(&mut self, h: DoorHandle) -> bool {
        let Some(d) = self.door(h) else { return false };
        let body = d.body;
        self.destroy_body(body);
        self.doors[h.index as usize].alive = false;
        self.door_free.push(h.index);
        true
    }

    pub fn door(&self, h: DoorHandle) -> Option<&Door> {
        match self.doors.get(h.index as usize) {
            Some(d) if d.alive && d.generation == h.generation => Some(d),
            _ => None,
        }
    }

    fn door_mut(&mut self, h: DoorHandle) -> Option<&mut Door> {
        match self.doors.get_mut(h.index as usize) {
            Some(d) if d.alive && d.generation == h.generation => Some(d),
            _ => None,
        }
    }

    /// Start opening (`true`) or closing (`false`).
    pub fn set_door_open(&mut self, h: DoorHandle, open: bool) -> bool {
        match self.door_mut(h) {
            Some(d) => {
                d.target = if open { d.desc.open_amount } else { Fx::ZERO };
                true
            }
            None => false,
        }
    }

    /// Move toward a partial opening (0 = closed .. 1 = open).
    pub fn set_door_target(&mut self, h: DoorHandle, fraction: Fx) -> bool {
        match self.door_mut(h) {
            Some(d) => {
                d.target = d.desc.open_amount * fraction.clamp(Fx::ZERO, Fx::ONE);
                true
            }
            None => false,
        }
    }

    /// Snap a door to an opening instantly (map load, admin commands).
    pub fn set_door_amount(&mut self, h: DoorHandle, fraction: Fx) -> bool {
        let Some(d) = self.door_mut(h) else { return false };
        d.amount = d.desc.open_amount * fraction.clamp(Fx::ZERO, Fx::ONE);
        d.target = d.amount;
        let (body, pose) = (d.body, d.pose_at(d.amount));
        self.set_pose(body, pose.pos, pose.rot);
        self.set_velocity(body, Vec3::ZERO, Vec3::ZERO);
        true
    }

    #[inline(never)]
    pub(crate) fn step_doors(&mut self, dt: Fx) {
        let mut c = Contacts::default();
        for i in 0..self.doors.len() {
            let d = &self.doors[i];
            if !d.alive {
                continue;
            }
            let body = d.body;
            if d.amount == d.target {
                if let Some(b) = self.body_mut(body) {
                    b.linear_velocity = Vec3::ZERO;
                    b.angular_velocity = Vec3::ZERO;
                }
                self.doors[i].blocked = false;
                continue;
            }
            let step = d.desc.speed * dt;
            let next = if d.target > d.amount { (d.amount + step).min(d.target) } else { (d.amount - step).max(d.target) };
            let pose = d.pose_at(next);
            let mut blocked = false;
            if d.desc.block_on_characters {
                let shape = &self.shapes[d.desc.shape.0 as usize];
                let aabb = shape.aabb(&pose);
                for ch in self.characters.iter().filter(|c| c.alive) {
                    let Some(cb) = self.body(ch.body) else { continue };
                    if !cb.aabb.overlaps(&aabb) {
                        continue;
                    }
                    let cs = &self.shapes[cb.shape.0 as usize];
                    // Stop a skin width short of touching, so the player is
                    // never pushed (no depenetration nudge either).
                    let skin = ch.desc.skin;
                    if collide_convex(shape, &pose, cs, &Iso::new(ch.position, Quat::IDENTITY), skin, &mut c)
                        && c.points[..c.count].iter().any(|p| p.depth >= -skin)
                    {
                        blocked = true;
                        break;
                    }
                }
            }
            if blocked {
                self.doors[i].blocked = true;
                if let Some(b) = self.body_mut(body) {
                    b.linear_velocity = Vec3::ZERO;
                    b.angular_velocity = Vec3::ZERO;
                }
                continue;
            }
            let door = &mut self.doors[i];
            door.amount = next;
            door.blocked = false;
            self.set_kinematic_target(body, pose.pos, pose.rot);
        }
    }
}
