//! Binary snapshots of the full simulation state, plus a state hash for
//! desync detection.
//!
//! A snapshot contains everything that influences future steps: bodies,
//! free lists, contact caches (warm-start impulses), sensor/touch pairs and
//! characters. It does NOT contain shapes or settings — those are
//! configuration and must be set up identically on every peer before
//! restoring. The lag-compensation history is not included either.

use crate::body::{Body, BodyHandle, BodyKind};
use crate::character::{Character, CharacterDesc, CharacterInput, MoveMode, MovementSettings};
use crate::door::{Door, DoorDesc, DoorKind};
use crate::fixed::Fx;
use crate::geometry::Aabb;
use crate::math::{Iso, Quat, Vec3};
use crate::shape::ShapeId;
use crate::solver::{Manifold, ManifoldPoint};
use crate::world::World;

const MAGIC: u32 = 0x3152_4850; // "PHR1"
const VERSION: u32 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapshotError {
    BadMagic,
    BadVersion,
    Truncated,
    /// The snapshot references a shape that is not registered.
    MissingShape,
}

struct W<'a>(&'a mut Vec<u8>);

impl W<'_> {
    #[inline]
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    #[inline]
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    #[inline]
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    #[inline]
    fn fx(&mut self, v: Fx) {
        self.0.extend_from_slice(&v.0.to_le_bytes());
    }
    #[inline]
    fn v3(&mut self, v: Vec3) {
        self.fx(v.x);
        self.fx(v.y);
        self.fx(v.z);
    }
    #[inline]
    fn q(&mut self, q: Quat) {
        self.fx(q.x);
        self.fx(q.y);
        self.fx(q.z);
        self.fx(q.w);
    }
    #[inline]
    fn b(&mut self, v: bool) {
        self.u8(v as u8);
    }
}

struct R<'a> {
    d: &'a [u8],
    p: usize,
}

impl R<'_> {
    #[inline]
    fn take<const N: usize>(&mut self) -> Result<[u8; N], SnapshotError> {
        if self.p + N > self.d.len() {
            return Err(SnapshotError::Truncated);
        }
        let mut a = [0u8; N];
        a.copy_from_slice(&self.d[self.p..self.p + N]);
        self.p += N;
        Ok(a)
    }
    fn u8(&mut self) -> Result<u8, SnapshotError> {
        Ok(self.take::<1>()?[0])
    }
    fn u32(&mut self) -> Result<u32, SnapshotError> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    fn u64(&mut self) -> Result<u64, SnapshotError> {
        Ok(u64::from_le_bytes(self.take()?))
    }
    fn fx(&mut self) -> Result<Fx, SnapshotError> {
        Ok(Fx(i64::from_le_bytes(self.take()?)))
    }
    fn v3(&mut self) -> Result<Vec3, SnapshotError> {
        Ok(Vec3::new(self.fx()?, self.fx()?, self.fx()?))
    }
    fn q(&mut self) -> Result<Quat, SnapshotError> {
        Ok(Quat::new(self.fx()?, self.fx()?, self.fx()?, self.fx()?))
    }
    fn b(&mut self) -> Result<bool, SnapshotError> {
        Ok(self.u8()? != 0)
    }
    fn len(&mut self) -> Result<usize, SnapshotError> {
        let n = self.u32()? as usize;
        // Every element is at least one byte: reject absurd lengths early.
        if n > self.d.len() - self.p {
            return Err(SnapshotError::Truncated);
        }
        Ok(n)
    }
}

fn write_pairs(w: &mut W, p: &[(u32, u32)]) {
    w.u32(p.len() as u32);
    for &(a, b) in p {
        w.u32(a);
        w.u32(b);
    }
}

fn read_pairs(r: &mut R) -> Result<Vec<(u32, u32)>, SnapshotError> {
    let n = r.len()?;
    let mut v = Vec::with_capacity(n);
    for _ in 0..n {
        v.push((r.u32()?, r.u32()?));
    }
    Ok(v)
}

impl World {
    /// Serialise the simulation state into `out` (cleared first).
    pub fn save_snapshot(&self, out: &mut Vec<u8>) {
        out.clear();
        let mut w = W(out);
        w.u32(MAGIC);
        w.u32(VERSION);
        w.u32(self.tick);

        w.u32(self.bodies.len() as u32);
        for b in &self.bodies {
            w.u32(b.generation);
            w.b(b.alive);
            if !b.alive {
                continue;
            }
            w.u8(b.kind as u8);
            w.u32(b.shape.0);
            w.v3(b.position);
            w.q(b.rotation);
            w.v3(b.linear_velocity);
            w.v3(b.angular_velocity);
            w.v3(b.force);
            w.v3(b.torque);
            w.fx(b.mass);
            w.fx(b.inv_mass);
            w.v3(b.inv_inertia_local);
            w.fx(b.friction);
            w.fx(b.restitution);
            w.fx(b.linear_damping);
            w.fx(b.angular_damping);
            w.fx(b.gravity_scale);
            w.u8(b.layer);
            w.u32(b.mask);
            w.u32(b.flags);
            w.b(b.sleeping);
            w.fx(b.sleep_time);
            w.u64(b.user_data);
            w.b(b.has_target);
            w.v3(b.target.pos);
            w.q(b.target.rot);
        }
        w.u32(self.free.len() as u32);
        for &f in &self.free {
            w.u32(f);
        }

        w.u32(self.manifolds.len() as u32);
        for m in &self.manifolds {
            w.u32(m.a);
            w.u32(m.b);
            w.u32(m.sub);
            w.v3(m.normal);
            w.u8(m.count);
            for p in m.points() {
                w.v3(p.point);
                w.fx(p.depth);
                w.u32(p.id);
                w.fx(p.normal_impulse);
            }
            w.fx(m.friction_impulse[0]);
            w.fx(m.friction_impulse[1]);
            w.fx(m.twist_impulse);
        }
        write_pairs(&mut w, &self.touching);
        write_pairs(&mut w, &self.sensor_pairs);

        w.u32(self.characters.len() as u32);
        for c in &self.characters {
            w.u32(c.generation);
            w.b(c.alive);
            if !c.alive {
                continue;
            }
            w.u64(c.body.to_bits());
            w.u32(c.stand_shape.0);
            w.u32(c.crouch_shape.0);
            let d = &c.desc;
            for v in [d.radius, d.height, d.crouch_height, d.step_height, d.max_slope_cos, d.skin, d.snap_distance, d.mass, d.push_strength]
            {
                w.fx(v);
            }
            w.u8(d.layer);
            w.u32(d.mask);
            w.u32(d.body_flags);
            w.u64(d.user_data);
            let m = &d.movement;
            for v in [
                m.walk_speed,
                m.sprint_speed,
                m.crouch_speed,
                m.ground_accel,
                m.air_accel,
                m.air_speed_cap,
                m.friction,
                m.stop_speed,
                m.gravity,
                m.jump_speed,
                m.max_fall_speed,
            ] {
                w.fx(v);
            }
            w.b(m.jump_requires_release);
            w.u32(m.coyote_ticks);
            w.v3(c.position);
            w.v3(c.velocity);
            w.b(c.grounded);
            w.v3(c.ground_normal);
            w.u64(c.ground_body.to_bits());
            w.v3(c.ground_velocity);
            w.b(c.crouched);
            w.u8(c.mode as u8);
            w.u32(c.collision_flags);
            w.fx(c.landing_speed);
            w.u32(c.air_ticks);
            w.fx(c.input.move_x);
            w.fx(c.input.move_y);
            w.fx(c.input.move_z);
            w.fx(c.input.yaw);
            w.u8((c.input.jump as u8) | ((c.input.sprint as u8) << 1) | ((c.input.crouch as u8) << 2));
            w.b(c.jump_latched);
        }
        w.u32(self.char_free.len() as u32);
        for &f in &self.char_free {
            w.u32(f);
        }

        w.u32(self.doors.len() as u32);
        for d in &self.doors {
            w.u32(d.generation);
            w.b(d.alive);
            if !d.alive {
                continue;
            }
            w.u64(d.body.to_bits());
            let c = &d.desc;
            w.u32(c.shape.0);
            w.u8(c.kind as u8);
            w.v3(c.pivot);
            w.v3(c.axis);
            w.q(c.closed_rotation);
            w.v3(c.offset);
            w.fx(c.open_amount);
            w.fx(c.speed);
            w.b(c.block_on_characters);
            w.u8(c.layer);
            w.u32(c.mask);
            w.fx(c.friction);
            w.u64(c.user_data);
            w.fx(d.amount);
            w.fx(d.target);
            w.b(d.blocked);
        }
        w.u32(self.door_free.len() as u32);
        for &f in &self.door_free {
            w.u32(f);
        }
    }

    /// Restore state from a snapshot produced by [`World::save_snapshot`].
    /// On error the world is left unchanged.
    pub fn load_snapshot(&mut self, data: &[u8]) -> Result<(), SnapshotError> {
        let mut r = R { d: data, p: 0 };
        if r.u32()? != MAGIC {
            return Err(SnapshotError::BadMagic);
        }
        if r.u32()? != VERSION {
            return Err(SnapshotError::BadVersion);
        }
        let tick = r.u32()?;
        let nb = r.len()?;
        let mut bodies = Vec::with_capacity(nb);
        for _ in 0..nb {
            let generation = r.u32()?;
            let alive = r.b()?;
            if !alive {
                bodies.push(Body { generation, alive: false, ..Default::default() });
                continue;
            }
            let kind = BodyKind::from_u8(r.u8()?);
            let shape = ShapeId(r.u32()?);
            if shape.0 as usize >= self.shapes.len() {
                return Err(SnapshotError::MissingShape);
            }
            let b = Body {
                generation,
                alive,
                kind,
                shape,
                position: r.v3()?,
                rotation: r.q()?,
                linear_velocity: r.v3()?,
                angular_velocity: r.v3()?,
                force: r.v3()?,
                torque: r.v3()?,
                mass: r.fx()?,
                inv_mass: r.fx()?,
                inv_inertia_local: r.v3()?,
                friction: r.fx()?,
                restitution: r.fx()?,
                linear_damping: r.fx()?,
                angular_damping: r.fx()?,
                gravity_scale: r.fx()?,
                layer: r.u8()? & 31,
                mask: r.u32()?,
                flags: r.u32()?,
                sleeping: r.b()?,
                sleep_time: r.fx()?,
                user_data: r.u64()?,
                has_target: r.b()?,
                target: Iso::new(r.v3()?, r.q()?),
                aabb: Aabb::EMPTY,
            };
            bodies.push(b);
        }
        let nf = r.len()?;
        let mut free = Vec::with_capacity(nf);
        for _ in 0..nf {
            free.push(r.u32()?);
        }
        let nm = r.len()?;
        let mut manifolds = Vec::with_capacity(nm);
        for _ in 0..nm {
            let mut m = Manifold { a: r.u32()?, b: r.u32()?, sub: r.u32()?, normal: r.v3()?, count: r.u8()?, ..Default::default() };
            m.count = m.count.min(4);
            for k in 0..m.count as usize {
                m.points[k] = ManifoldPoint { point: r.v3()?, depth: r.fx()?, id: r.u32()?, normal_impulse: r.fx()? };
            }
            m.friction_impulse = [r.fx()?, r.fx()?];
            m.twist_impulse = r.fx()?;
            manifolds.push(m);
        }
        let touching = read_pairs(&mut r)?;
        let sensor_pairs = read_pairs(&mut r)?;

        let nc = r.len()?;
        let mut characters = Vec::with_capacity(nc);
        for _ in 0..nc {
            let generation = r.u32()?;
            let alive = r.b()?;
            if !alive {
                characters.push(Character {
                    alive: false,
                    generation,
                    body: BodyHandle::INVALID,
                    stand_shape: ShapeId(0),
                    crouch_shape: ShapeId(0),
                    desc: CharacterDesc::default(),
                    position: Vec3::ZERO,
                    velocity: Vec3::ZERO,
                    grounded: false,
                    ground_normal: Vec3::Y,
                    ground_body: BodyHandle::INVALID,
                    ground_velocity: Vec3::ZERO,
                    crouched: false,
                    mode: MoveMode::Walk,
                    collision_flags: 0,
                    landing_speed: Fx::ZERO,
                    air_ticks: u32::MAX,
                    input: CharacterInput::default(),
                    jump_latched: false,
                });
                continue;
            }
            let body = BodyHandle::from_bits(r.u64()?);
            let stand_shape = ShapeId(r.u32()?);
            let crouch_shape = ShapeId(r.u32()?);
            if stand_shape.0 as usize >= self.shapes.len() || crouch_shape.0 as usize >= self.shapes.len() {
                return Err(SnapshotError::MissingShape);
            }
            let mut d = CharacterDesc {
                position: Vec3::ZERO,
                radius: r.fx()?,
                height: r.fx()?,
                crouch_height: r.fx()?,
                step_height: r.fx()?,
                max_slope_cos: r.fx()?,
                skin: r.fx()?,
                snap_distance: r.fx()?,
                mass: r.fx()?,
                push_strength: r.fx()?,
                layer: r.u8()?,
                mask: r.u32()?,
                body_flags: r.u32()?,
                user_data: r.u64()?,
                movement: MovementSettings::default(),
            };
            let m = &mut d.movement;
            m.walk_speed = r.fx()?;
            m.sprint_speed = r.fx()?;
            m.crouch_speed = r.fx()?;
            m.ground_accel = r.fx()?;
            m.air_accel = r.fx()?;
            m.air_speed_cap = r.fx()?;
            m.friction = r.fx()?;
            m.stop_speed = r.fx()?;
            m.gravity = r.fx()?;
            m.jump_speed = r.fx()?;
            m.max_fall_speed = r.fx()?;
            m.jump_requires_release = r.b()?;
            m.coyote_ticks = r.u32()?;
            let position = r.v3()?;
            d.position = position;
            let velocity = r.v3()?;
            let grounded = r.b()?;
            let ground_normal = r.v3()?;
            let ground_body = BodyHandle::from_bits(r.u64()?);
            let ground_velocity = r.v3()?;
            let crouched = r.b()?;
            let mode = MoveMode::from_u8(r.u8()?);
            let collision_flags = r.u32()?;
            let landing_speed = r.fx()?;
            let air_ticks = r.u32()?;
            let move_x = r.fx()?;
            let move_y = r.fx()?;
            let move_z = r.fx()?;
            let yaw = r.fx()?;
            let bits = r.u8()?;
            let jump_latched = r.b()?;
            characters.push(Character {
                alive,
                generation,
                body,
                stand_shape,
                crouch_shape,
                desc: d,
                position,
                velocity,
                grounded,
                ground_normal,
                ground_body,
                ground_velocity,
                crouched,
                mode,
                collision_flags,
                landing_speed,
                air_ticks,
                input: CharacterInput { move_x, move_y, move_z, yaw, jump: bits & 1 != 0, sprint: bits & 2 != 0, crouch: bits & 4 != 0 },
                jump_latched,
            });
        }
        let ncf = r.len()?;
        let mut char_free = Vec::with_capacity(ncf);
        for _ in 0..ncf {
            char_free.push(r.u32()?);
        }

        let nd = r.len()?;
        let mut doors = Vec::with_capacity(nd);
        for _ in 0..nd {
            let generation = r.u32()?;
            let alive = r.b()?;
            if !alive {
                doors.push(Door {
                    alive: false,
                    generation,
                    body: BodyHandle::INVALID,
                    desc: DoorDesc::default(),
                    amount: Fx::ZERO,
                    target: Fx::ZERO,
                    blocked: false,
                });
                continue;
            }
            let body = BodyHandle::from_bits(r.u64()?);
            let desc = DoorDesc {
                shape: ShapeId(r.u32()?),
                kind: DoorKind::from_u8(r.u8()?),
                pivot: r.v3()?,
                axis: r.v3()?,
                closed_rotation: r.q()?,
                offset: r.v3()?,
                open_amount: r.fx()?,
                speed: r.fx()?,
                block_on_characters: r.b()?,
                layer: r.u8()?,
                mask: r.u32()?,
                friction: r.fx()?,
                user_data: r.u64()?,
            };
            if desc.shape.0 as usize >= self.shapes.len() {
                return Err(SnapshotError::MissingShape);
            }
            doors.push(Door { alive, generation, body, desc, amount: r.fx()?, target: r.fx()?, blocked: r.b()? });
        }
        let ndf = r.len()?;
        let mut door_free = Vec::with_capacity(ndf);
        for _ in 0..ndf {
            door_free.push(r.u32()?);
        }

        // Validate indices before committing.
        let nbodies = bodies.len() as u32;
        if manifolds.iter().any(|m| m.a >= nbodies || m.b >= nbodies)
            || touching.iter().chain(sensor_pairs.iter()).any(|p| p.0 >= nbodies || p.1 >= nbodies)
            || free.iter().any(|&f| f >= nbodies)
        {
            return Err(SnapshotError::Truncated);
        }

        self.tick = tick;
        self.bodies = bodies;
        self.free = free;
        self.manifolds = manifolds;
        self.touching = touching;
        self.sensor_pairs = sensor_pairs;
        self.characters = characters;
        self.char_free = char_free;
        self.doors = doors;
        self.door_free = door_free;
        self.events.clear();
        // Derived data.
        for b in self.bodies.iter_mut() {
            if b.alive {
                // Must match what a continuous run holds: movers keep the
                // margin-expanded bounds from the end of the last step.
                b.aabb = self.shapes[b.shape.0 as usize].aabb(&b.iso());
                if !b.is_static() {
                    b.aabb = b.aabb.expand(self.settings.contact_margin);
                }
            }
        }
        self.static_dirty = true;
        self.rebuild_static();
        self.mark_movers_dirty();
        self.update_mover_tree();
        Ok(())
    }

    /// 64-bit FNV-1a hash of the full simulation state. Compare between
    /// peers each tick (or every N ticks) to detect desyncs.
    pub fn state_hash(&mut self) -> u64 {
        let mut buf = core::mem::take(&mut self.snapshot_buf);
        self.save_snapshot(&mut buf);
        let h = fnv1a(&buf);
        self.snapshot_buf = buf;
        h
    }
}

pub fn fnv1a(data: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in data {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}
