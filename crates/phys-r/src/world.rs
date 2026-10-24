//! The physics world: body storage, broad phase, narrow phase and stepping.

use crate::body::{flags, layers_interact, Body, BodyDesc, BodyHandle, BodyKind};
use crate::bvh::Bvh;
use crate::character::Character;
use crate::collide::{collide_convex, Contacts};
use crate::fixed::Fx;
use crate::fx;
use crate::geometry::Aabb;
use crate::history::History;
use crate::math::{Iso, Mat3, Quat, Vec3};
use crate::shape::{Shape, ShapeId};
use crate::solver::{Manifold, ManifoldPoint, Solver, SolverParams};

/// Global simulation parameters. These are configuration, not state: both
/// sides of a networked game must use identical settings.
#[derive(Clone, Debug)]
pub struct WorldSettings {
    pub gravity: Vec3,
    /// Fixed time step in seconds.
    pub dt: Fx,
    pub velocity_iterations: u32,
    /// Split-impulse position correction iterations.
    pub position_iterations: u32,
    /// Distance at which speculative contacts are created.
    pub contact_margin: Fx,
    /// Allowed penetration before position correction kicks in.
    pub slop: Fx,
    /// Fraction of penetration corrected per step.
    pub baumgarte: Fx,
    /// Max separation velocity from position correction (m/s).
    pub max_correction_velocity: Fx,
    /// Impact speed below which restitution is ignored (m/s).
    pub restitution_threshold: Fx,
    pub sleep_linear_threshold: Fx,
    pub sleep_angular_threshold: Fx,
    /// Seconds of rest before an island falls asleep.
    pub time_to_sleep: Fx,
    pub enable_sleep: bool,
    /// Number of ticks kept for lag compensation.
    pub history_ticks: u32,
}

impl Default for WorldSettings {
    fn default() -> Self {
        WorldSettings {
            gravity: Vec3::new(Fx::ZERO, fx!(-9.81), Fx::ZERO),
            dt: Fx::from_ratio(1, 60),
            velocity_iterations: 8,
            position_iterations: 3,
            contact_margin: fx!(0.02),
            slop: fx!(0.005),
            baumgarte: fx!(0.2),
            max_correction_velocity: fx!(3.0),
            restitution_threshold: fx!(1.0),
            sleep_linear_threshold: fx!(0.05),
            sleep_angular_threshold: fx!(0.08),
            time_to_sleep: fx!(0.5),
            enable_sleep: true,
            history_ticks: 64,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum EventKind {
    ContactBegin = 0,
    ContactEnd = 1,
    SensorEnter = 2,
    SensorExit = 3,
}

/// Simulation event produced by [`World::step`].
#[derive(Copy, Clone, Debug)]
pub struct Event {
    pub kind: EventKind,
    pub a: BodyHandle,
    pub b: BodyHandle,
    /// Contact point / normal (zero for sensor and end events).
    pub point: Vec3,
    pub normal: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorldError {
    InvalidShape,
    InvalidHandle,
}

/// Physics world.
pub struct World {
    pub settings: WorldSettings,
    pub(crate) shapes: Vec<Shape>,
    pub(crate) bodies: Vec<Body>,
    pub(crate) free: Vec<u32>,
    pub(crate) tick: u32,

    // Persistent simulation state (snapshotted).
    pub(crate) manifolds: Vec<Manifold>,
    pub(crate) touching: Vec<(u32, u32)>,
    pub(crate) sensor_pairs: Vec<(u32, u32)>,
    pub(crate) characters: Vec<Character>,
    pub(crate) char_free: Vec<u32>,
    pub(crate) doors: Vec<crate::door::Door>,
    pub(crate) door_free: Vec<u32>,

    // Derived / scratch state.
    pub(crate) static_bvh: Bvh,
    /// Subset of `static_bvh` holding only static sensors.
    static_sensor_bvh: Bvh,
    pub(crate) static_dirty: bool,
    pub(crate) movers: Vec<u32>,
    movers_dirty: bool,
    /// BVH over moving bodies for queries; refit lazily on first use after a
    /// step and rebuilt periodically. Query results never depend on its shape.
    mover_bvh: core::cell::RefCell<Bvh>,
    mover_bvh_state: core::cell::Cell<u8>,
    pairs: Vec<(u32, u32)>,
    new_manifolds: Vec<Manifold>,
    scratch_pairs: Vec<(u32, u32)>,
    solver: Solver,
    solver_index: Vec<u32>,
    uf_parent: Vec<u32>,
    uf_sleep: Vec<Fx>,
    pseudo: Vec<(Vec3, Vec3)>,
    solver_touched: Vec<u32>,
    pub(crate) events: Vec<Event>,
    pub(crate) history: History,
    pub(crate) scratch_obstacles: Vec<crate::query::ObRef>,
    pub(crate) snapshot_buf: Vec<u8>,
}

const NO_INDEX: u32 = u32::MAX;
const BVH_FRESH: u8 = 0;
const BVH_REFIT: u8 = 1;
const BVH_REBUILD: u8 = 2;
/// Below this many moving bodies queries just scan them linearly.
const MOVER_BVH_MIN: usize = 24;

impl World {
    pub fn new(settings: WorldSettings) -> World {
        let history = History::new(settings.history_ticks as usize);
        World {
            settings,
            shapes: Vec::new(),
            bodies: Vec::new(),
            free: Vec::new(),
            tick: 0,
            manifolds: Vec::new(),
            touching: Vec::new(),
            sensor_pairs: Vec::new(),
            characters: Vec::new(),
            char_free: Vec::new(),
            doors: Vec::new(),
            door_free: Vec::new(),
            static_bvh: Bvh::default(),
            static_sensor_bvh: Bvh::default(),
            static_dirty: true,
            movers: Vec::new(),
            movers_dirty: true,
            mover_bvh: core::cell::RefCell::new(Bvh::default()),
            mover_bvh_state: core::cell::Cell::new(BVH_REBUILD),
            pairs: Vec::new(),
            new_manifolds: Vec::new(),
            scratch_pairs: Vec::new(),
            solver: Solver::default(),
            solver_index: Vec::new(),
            uf_parent: Vec::new(),
            uf_sleep: Vec::new(),
            pseudo: Vec::new(),
            solver_touched: Vec::new(),
            events: Vec::new(),
            history,
            scratch_obstacles: Vec::new(),
            snapshot_buf: Vec::new(),
        }
    }

    /// Current tick (number of completed steps).
    #[inline]
    pub fn tick(&self) -> u32 {
        self.tick
    }

    // -----------------------------------------------------------------------
    // Shapes
    // -----------------------------------------------------------------------

    /// Register a shape. Shapes are configuration (like level data): they are
    /// not part of snapshots, so register them in the same order everywhere.
    pub fn add_shape(&mut self, shape: Shape) -> ShapeId {
        self.shapes.push(shape);
        ShapeId(self.shapes.len() as u32 - 1)
    }

    #[inline]
    pub fn shape(&self, id: ShapeId) -> Option<&Shape> {
        self.shapes.get(id.0 as usize)
    }

    pub fn shape_count(&self) -> usize {
        self.shapes.len()
    }

    // -----------------------------------------------------------------------
    // Bodies
    // -----------------------------------------------------------------------

    pub fn create_body(&mut self, d: &BodyDesc) -> Result<BodyHandle, WorldError> {
        let shape = self.shapes.get(d.shape.0 as usize).ok_or(WorldError::InvalidShape)?;
        let (mass, inv_mass, inv_i) = if d.kind == BodyKind::Dynamic {
            let m = if d.mass.0 > 0 { d.mass } else { Fx::ONE };
            let ii = if d.flags & flags::LOCK_ROTATION != 0 { Vec3::ZERO } else { shape.inverse_inertia(m) };
            (m, Fx::ONE / m, ii)
        } else {
            (Fx::ZERO, Fx::ZERO, Vec3::ZERO)
        };
        let rotation = d.rotation.normalize();
        let iso = Iso::new(d.position, rotation);
        let aabb = if d.kind == BodyKind::Static { shape.aabb(&iso) } else { shape.aabb(&iso).expand(self.settings.contact_margin) };
        let index = match self.free.pop() {
            Some(i) => i,
            None => {
                self.bodies.push(Body::default());
                self.bodies.len() as u32 - 1
            }
        };
        let slot = &mut self.bodies[index as usize];
        let generation = slot.generation.wrapping_add(1).max(1);
        *slot = Body {
            generation,
            alive: true,
            kind: d.kind,
            shape: d.shape,
            position: d.position,
            rotation,
            linear_velocity: if d.kind == BodyKind::Static { Vec3::ZERO } else { d.linear_velocity },
            angular_velocity: if d.kind == BodyKind::Static { Vec3::ZERO } else { d.angular_velocity },
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            mass,
            inv_mass,
            inv_inertia_local: inv_i,
            friction: d.friction,
            restitution: d.restitution,
            linear_damping: d.linear_damping,
            angular_damping: d.angular_damping,
            gravity_scale: d.gravity_scale,
            layer: d.layer & 31,
            mask: d.mask,
            flags: d.flags,
            sleeping: false,
            sleep_time: Fx::ZERO,
            user_data: d.user_data,
            has_target: false,
            target: Iso::IDENTITY,
            aabb,
        };
        if d.kind == BodyKind::Static {
            self.static_dirty = true;
            self.wake_in_aabb(&aabb);
        } else {
            self.movers_dirty = true;
        }
        Ok(BodyHandle { index, generation })
    }

    pub fn destroy_body(&mut self, h: BodyHandle) -> bool {
        if self.body(h).is_none() {
            return false;
        }
        let i = h.index;
        let aabb = self.bodies[i as usize].aabb;
        let was_static = self.bodies[i as usize].is_static();
        // Wake everything that was touching it.
        for k in 0..self.manifolds.len() {
            let m = self.manifolds[k];
            if m.a == i {
                self.wake_index(m.b);
            } else if m.b == i {
                self.wake_index(m.a);
            }
        }
        self.manifolds.retain(|m| m.a != i && m.b != i);
        self.touching.retain(|p| p.0 != i && p.1 != i);
        self.sensor_pairs.retain(|p| p.0 != i && p.1 != i);
        let b = &mut self.bodies[i as usize];
        b.alive = false;
        b.flags = 0;
        self.free.push(i);
        if was_static {
            self.static_dirty = true;
            self.wake_in_aabb(&aabb);
        } else {
            self.movers_dirty = true;
        }
        true
    }

    #[inline]
    pub fn body(&self, h: BodyHandle) -> Option<&Body> {
        match self.bodies.get(h.index as usize) {
            Some(b) if b.alive && b.generation == h.generation => Some(b),
            _ => None,
        }
    }

    /// Mutable access. Changing `position`/`rotation` of a static body here
    /// will not update the static BVH; use [`World::set_pose`] instead.
    #[inline]
    pub fn body_mut(&mut self, h: BodyHandle) -> Option<&mut Body> {
        match self.bodies.get_mut(h.index as usize) {
            Some(b) if b.alive && b.generation == h.generation => Some(b),
            _ => None,
        }
    }

    #[inline]
    pub(crate) fn handle_of(&self, index: u32) -> BodyHandle {
        BodyHandle { index, generation: self.bodies[index as usize].generation }
    }

    /// Iterate over live bodies with their handles (index order).
    pub fn bodies(&self) -> impl Iterator<Item = (BodyHandle, &Body)> {
        self.bodies.iter().enumerate().filter(|(_, b)| b.alive).map(|(i, b)| (b.handle(i as u32), b))
    }

    pub fn body_count(&self) -> usize {
        self.bodies.iter().filter(|b| b.alive).count()
    }

    /// Teleport a body.
    pub fn set_pose(&mut self, h: BodyHandle, position: Vec3, rotation: Quat) -> bool {
        let shapes = &self.shapes;
        let Some(b) = (match self.bodies.get_mut(h.index as usize) {
            Some(b) if b.alive && b.generation == h.generation => Some(b),
            _ => None,
        }) else {
            return false;
        };
        b.position = position;
        b.rotation = rotation.normalize();
        let old = b.aabb;
        b.aabb = shapes[b.shape.0 as usize].aabb(&b.iso());
        if !b.is_static() {
            b.aabb = b.aabb.expand(self.settings.contact_margin);
        }
        let new = b.aabb;
        if b.is_static() {
            self.static_dirty = true;
            self.wake_in_aabb(&old.union(&new));
        } else {
            b.sleeping = false;
            b.sleep_time = Fx::ZERO;
            if self.mover_bvh_state.get() == BVH_FRESH {
                self.mover_bvh_state.set(BVH_REFIT);
            }
        }
        true
    }

    /// Change body flags (keeps internal flags). Static bodies are re-indexed
    /// so e.g. turning a collider into a trigger takes effect.
    pub fn set_body_flags(&mut self, h: BodyHandle, flags: u32) -> bool {
        let Some(b) = self.body_mut(h) else { return false };
        b.flags = (b.flags & flags::CHARACTER) | (flags & !flags::CHARACTER);
        if b.is_static() {
            self.static_dirty = true;
        }
        true
    }

    pub fn set_velocity(&mut self, h: BodyHandle, linear: Vec3, angular: Vec3) -> bool {
        match self.body_mut(h) {
            Some(b) if !b.is_static() => {
                b.linear_velocity = linear;
                b.angular_velocity = angular;
                b.sleeping = false;
                b.sleep_time = Fx::ZERO;
                true
            }
            _ => false,
        }
    }

    /// Kinematic bodies: reach exactly this pose at the end of the next step
    /// (the implied velocity pushes dynamic bodies). Call every tick for
    /// continuous motion; without a new target the body stops.
    pub fn set_kinematic_target(&mut self, h: BodyHandle, position: Vec3, rotation: Quat) -> bool {
        let inv_dt = Fx::ONE / self.settings.dt;
        match self.body_mut(h) {
            Some(b) if b.kind == BodyKind::Kinematic => {
                // Velocities let the solver push props; the exact pose is
                // snapped at the end of the step (no integration drift).
                b.has_target = true;
                b.target = Iso::new(position, rotation.normalize());
                b.linear_velocity = (position - b.position).scale(inv_dt);
                // Angular velocity from the relative rotation (small-angle).
                let mut dq = rotation.normalize().mul(b.rotation.conjugate());
                if dq.w < Fx::ZERO {
                    dq = Quat::new(-dq.x, -dq.y, -dq.z, -dq.w);
                }
                b.angular_velocity = dq.xyz().scale(inv_dt.shl(1));
                true
            }
            _ => false,
        }
    }

    pub fn apply_impulse(&mut self, h: BodyHandle, impulse: Vec3, point: Option<Vec3>) -> bool {
        match self.body_mut(h) {
            Some(b) if b.is_dynamic() => {
                b.linear_velocity += impulse.scale(b.inv_mass);
                if let Some(p) = point {
                    let r = p - b.position;
                    let ii = Mat3::rotate_diagonal(&b.rotation.to_mat3(), b.inv_inertia_local);
                    b.angular_velocity += ii.mul_vec(r.cross(impulse));
                }
                b.sleeping = false;
                b.sleep_time = Fx::ZERO;
                true
            }
            _ => false,
        }
    }

    /// Force applied during the next step only.
    pub fn apply_force(&mut self, h: BodyHandle, force: Vec3, point: Option<Vec3>) -> bool {
        match self.body_mut(h) {
            Some(b) if b.is_dynamic() => {
                b.force += force;
                if let Some(p) = point {
                    b.torque += (p - b.position).cross(force);
                }
                b.sleeping = false;
                b.sleep_time = Fx::ZERO;
                true
            }
            _ => false,
        }
    }

    pub fn apply_torque(&mut self, h: BodyHandle, torque: Vec3) -> bool {
        match self.body_mut(h) {
            Some(b) if b.is_dynamic() => {
                b.torque += torque;
                b.sleeping = false;
                b.sleep_time = Fx::ZERO;
                true
            }
            _ => false,
        }
    }

    pub fn wake(&mut self, h: BodyHandle) {
        if self.body(h).is_some() {
            self.wake_index(h.index);
        }
    }

    #[inline]
    pub(crate) fn wake_index(&mut self, i: u32) {
        let b = &mut self.bodies[i as usize];
        if b.sleeping {
            b.sleeping = false;
            b.sleep_time = Fx::ZERO;
        }
    }

    fn wake_in_aabb(&mut self, aabb: &Aabb) {
        // Only moving bodies can sleep (keeps level loading O(n)).
        self.rebuild_movers();
        for &i in &self.movers {
            let b = &mut self.bodies[i as usize];
            if b.is_dynamic() && b.sleeping && b.aabb.overlaps(aabb) {
                b.sleeping = false;
                b.sleep_time = Fx::ZERO;
            }
        }
    }

    /// Events generated by the last step.
    pub fn events(&self) -> &[Event] {
        &self.events
    }

    /// Current contact manifolds (sorted by body pair).
    pub fn manifolds(&self) -> &[Manifold] {
        &self.manifolds
    }

    // -----------------------------------------------------------------------
    // Stepping
    // -----------------------------------------------------------------------

    /// Advance the simulation by one fixed tick.
    pub fn step(&mut self) {
        self.events.clear();
        let dt = self.settings.dt;

        // 1. Doors pick their next pose (reached exactly at the end of the
        //    step), then characters move (kinematic, query-based).
        self.step_doors(dt);
        self.step_characters(dt);

        // 2. Integrate velocities. (Per-tick loops only visit moving bodies, so
        // thousands of static colliders cost nothing here.)
        self.rebuild_movers();
        let g = self.settings.gravity;
        for &i in &self.movers {
            let b = &mut self.bodies[i as usize];
            if !b.is_dynamic() || b.sleeping {
                continue;
            }
            let mut v = b.linear_velocity + (g.scale(b.gravity_scale) + b.force.scale(b.inv_mass)).scale(dt);
            let mut w = b.angular_velocity;
            if !b.torque.is_zero() {
                let ii = Mat3::rotate_diagonal(&b.rotation.to_mat3(), b.inv_inertia_local);
                w += ii.mul_vec(b.torque).scale(dt);
            }
            // Pade damping: v *= 1 / (1 + c*dt)
            if b.linear_damping.0 > 0 {
                v = v.scale(Fx::ONE / (Fx::ONE + b.linear_damping * dt));
            }
            if b.angular_damping.0 > 0 {
                w = w.scale(Fx::ONE / (Fx::ONE + b.angular_damping * dt));
            }
            b.linear_velocity = v;
            b.angular_velocity = w;
        }

        // 3. Collision detection.
        self.update_bounds();
        self.find_pairs();
        self.narrow_phase();

        // 4. Solve.
        self.solve(dt);

        // 5. Integrate positions.
        for &i in &self.movers {
            let b = &mut self.bodies[i as usize];
            if b.sleeping {
                continue;
            }
            let (vp, wp) = self.pseudo[i as usize];
            b.position += (b.linear_velocity + vp).scale(dt);
            b.rotation = b.rotation.integrate(b.angular_velocity + wp, dt);
            b.force = Vec3::ZERO;
            b.torque = Vec3::ZERO;
        }
        for &i in &self.movers {
            let b = &mut self.bodies[i as usize];
            if b.has_target {
                // Targets are per tick: arrive exactly, then stop unless a
                // new target is set before the next step.
                b.position = b.target.pos;
                b.rotation = b.target.rot;
                b.has_target = false;
                b.linear_velocity = Vec3::ZERO;
                b.angular_velocity = Vec3::ZERO;
            }
        }
        self.sync_character_bodies();

        // 6. Sleep.
        if self.settings.enable_sleep {
            self.update_sleep(dt);
        }

        self.tick = self.tick.wrapping_add(1);
        self.record_history();
        // Bodies moved: refresh query bounds, refit the query tree lazily.
        self.refresh_mover_bounds();
        if self.mover_bvh_state.get() != BVH_REBUILD {
            self.mover_bvh_state.set(if self.tick % 64 == 0 { BVH_REBUILD } else { BVH_REFIT });
        }
    }

    #[inline(never)]
    pub(crate) fn update_bounds(&mut self) {
        self.rebuild_movers();
        let margin = self.settings.contact_margin;
        let dt = self.settings.dt;
        for &i in &self.movers {
            let b = &mut self.bodies[i as usize];
            // Sleeping bodies do not move: their bounds are still valid.
            if b.sleeping {
                continue;
            }
            let shape = &self.shapes[b.shape.0 as usize];
            let mut aabb = shape.aabb(&b.iso()).expand(margin);
            if b.flags & flags::CCD != 0 || b.kind == BodyKind::Kinematic {
                aabb = aabb.sweep(b.linear_velocity.scale(dt));
            }
            b.aabb = aabb;
        }
        if self.static_dirty {
            self.rebuild_static();
        }
    }

    #[inline(never)]
    /// Tight (unexpanded) bounds for moving bodies after integration, so
    /// queries between steps see current poses.
    fn refresh_mover_bounds(&mut self) {
        let margin = self.settings.contact_margin;
        for &i in &self.movers {
            let b = &mut self.bodies[i as usize];
            if !b.sleeping {
                b.aabb = self.shapes[b.shape.0 as usize].aabb(&b.iso()).expand(margin);
            }
        }
    }

    pub(crate) fn rebuild_static(&mut self) {
        let mut aabbs = Vec::new();
        let mut ids = Vec::new();
        let mut sensor_aabbs = Vec::new();
        let mut sensor_ids = Vec::new();
        for (i, b) in self.bodies.iter_mut().enumerate() {
            if b.alive && b.is_static() {
                b.aabb = self.shapes[b.shape.0 as usize].aabb(&b.iso());
                aabbs.push(b.aabb);
                ids.push(i as u32);
                if b.is_sensor() {
                    sensor_aabbs.push(b.aabb);
                    sensor_ids.push(i as u32);
                }
            }
        }
        self.static_bvh = Bvh::build_with_ids(&aabbs, ids);
        self.static_sensor_bvh = Bvh::build_with_ids(&sensor_aabbs, sensor_ids);
        self.static_dirty = false;
    }

    pub(crate) fn mark_movers_dirty(&mut self) {
        self.movers_dirty = true;
        self.mover_bvh_state.set(BVH_REBUILD);
    }

    /// Visit moving bodies whose box overlaps `aabb` (any order).
    pub(crate) fn query_movers(&self, aabb: &Aabb, mut f: impl FnMut(u32)) {
        if self.movers.len() < MOVER_BVH_MIN {
            for &i in &self.movers {
                if self.bodies[i as usize].aabb.overlaps(aabb) {
                    f(i);
                }
            }
            return;
        }
        self.ensure_mover_bvh();
        self.mover_bvh.borrow().query_aabb(aabb, f);
    }

    /// Ray traversal over moving bodies; `f(id, max_t)` may shrink the ray.
    pub(crate) fn raycast_movers(&self, ray: &crate::geometry::Ray, max_t: Fx, mut f: impl FnMut(u32, Fx, Fx) -> Option<Fx>) -> Fx {
        if self.movers.len() < MOVER_BVH_MIN {
            let mut t = max_t;
            for &i in &self.movers {
                let Some(enter) = self.bodies[i as usize].aabb.ray_test(ray, t) else { continue };
                if let Some(h) = f(i, enter, t) {
                    t = t.min(h);
                }
            }
            return t;
        }
        self.ensure_mover_bvh();
        self.mover_bvh.borrow().raycast(ray, max_t, f)
    }

    fn ensure_mover_bvh(&self) {
        match self.mover_bvh_state.get() {
            BVH_FRESH => {}
            BVH_REFIT => {
                self.mover_bvh.borrow_mut().refit(|i| self.bodies[i as usize].aabb);
                self.mover_bvh_state.set(BVH_FRESH);
            }
            _ => {
                let aabbs: Vec<Aabb> = self.movers.iter().map(|&i| self.bodies[i as usize].aabb).collect();
                *self.mover_bvh.borrow_mut() = Bvh::build_with_ids(&aabbs, self.movers.clone());
                self.mover_bvh_state.set(BVH_FRESH);
            }
        }
    }

    pub(crate) fn rebuild_movers(&mut self) {
        if self.movers_dirty {
            self.movers.clear();
            for (i, b) in self.bodies.iter().enumerate() {
                if b.alive && !b.is_static() {
                    self.movers.push(i as u32);
                }
            }
            self.movers_dirty = false;
            self.mover_bvh_state.set(BVH_REBUILD);
        }
    }

    #[inline(never)]
    fn find_pairs(&mut self) {
        self.rebuild_movers();
        // Sort-and-sweep along X. The key (min.x, index) is a total order, so
        // the sorted result depends only on the current state. Insertion sort
        // is ~O(n) thanks to frame-to-frame coherence.
        let bodies = &self.bodies;
        let movers = &mut self.movers;
        for i in 1..movers.len() {
            let cur = movers[i];
            let key = (bodies[cur as usize].aabb.min.x, cur);
            let mut j = i;
            while j > 0 {
                let p = movers[j - 1];
                if (bodies[p as usize].aabb.min.x, p) <= key {
                    break;
                }
                movers[j] = p;
                j -= 1;
            }
            movers[j] = cur;
        }

        self.pairs.clear();
        for i in 0..movers.len() {
            let ia = movers[i];
            let a = &bodies[ia as usize];
            for &ib in &movers[i + 1..] {
                let b = &bodies[ib as usize];
                if b.aabb.min.x > a.aabb.max.x {
                    break;
                }
                if !a.aabb.overlaps(&b.aabb) {
                    continue;
                }
                if !layers_interact(a.layer, a.mask, b.layer, b.mask) {
                    continue;
                }
                let sensor = a.is_sensor() || b.is_sensor();
                let a_active = a.is_dynamic() && !a.sleeping;
                let b_active = b.is_dynamic() && !b.sleeping;
                if sensor {
                    if a.is_sensor() && b.is_sensor() {
                        continue;
                    }
                } else if !(a_active || b_active) {
                    // Needs at least one awake dynamic body.
                    continue;
                }
                self.pairs.push(if ia < ib { (ia, ib) } else { (ib, ia) });
            }
        }

        // Moving bodies vs static geometry.
        for &ia in movers.iter() {
            let a = &bodies[ia as usize];
            if a.sleeping {
                continue;
            }
            let (la, ma) = (a.layer, a.mask);
            let a_sensor = a.is_sensor();
            let a_dyn = a.is_dynamic();
            let pairs = &mut self.pairs;
            // Kinematic bodies (characters, doors) only care about static
            // triggers; skip the full level tree for them.
            let tree = if a_dyn || a_sensor { &self.static_bvh } else { &self.static_sensor_bvh };
            tree.query_aabb(&a.aabb, |is| {
                let s = &bodies[is as usize];
                if !s.aabb.overlaps(&a.aabb) || !layers_interact(la, ma, s.layer, s.mask) {
                    return;
                }
                if s.is_sensor() {
                    if !a_sensor {
                        pairs.push((ia, is));
                    }
                } else if a_dyn || a_sensor {
                    pairs.push((ia, is));
                }
            });
        }
        // Canonical order for deterministic downstream processing.
        self.pairs.sort_unstable();
        self.pairs.dedup();
    }

    #[inline(never)]
    fn narrow_phase(&mut self) {
        let margin = self.settings.contact_margin;
        let dt = self.settings.dt;
        let old = core::mem::take(&mut self.manifolds);
        let mut new = core::mem::take(&mut self.new_manifolds);
        new.clear();
        let mut sensors = core::mem::take(&mut self.scratch_pairs);
        sensors.clear();
        let mut c = Contacts::default();

        for &(ia, ib) in &self.pairs {
            let a = &self.bodies[ia as usize];
            let b = &self.bodies[ib as usize];
            if a.is_sensor() || b.is_sensor() {
                if self.overlap_bodies(ia, ib) {
                    let (s, o) = if a.is_sensor() { (ia, ib) } else { (ib, ia) };
                    sensors.push((s, o));
                }
                continue;
            }
            let sa = &self.shapes[a.shape.0 as usize];
            let sb = &self.shapes[b.shape.0 as usize];
            let mut m = margin;
            if (a.flags | b.flags) & flags::CCD != 0 {
                m += (a.linear_velocity - b.linear_velocity).length() * dt;
            }
            if collide_convex(sa, &a.iso(), sb, &b.iso(), m, &mut c) && c.count > 0 {
                new.push(make_manifold(ia, ib, 0, &c, &old));
            }
        }
        // Pairs are sorted and yield at most one manifold each, so `new` is
        // already in key order (needed for warm-start lookups next step).
        debug_assert!(new.windows(2).all(|w| w[0].key() < w[1].key()));

        // Wake sleeping bodies touched by awake ones.
        for k in 0..new.len() {
            let (a, b) = (new[k].a, new[k].b);
            let sa = self.bodies[a as usize].sleeping;
            let sb = self.bodies[b as usize].sleeping;
            if sa && !sb {
                self.wake_index(a);
            } else if sb && !sa {
                self.wake_index(b);
            }
        }

        // Contact events (pair level, only for bodies that asked for them).
        let mut now: Vec<((u32, u32), Vec3, Vec3)> = Vec::new();
        for m in &new {
            let fa = self.bodies[m.a as usize].flags;
            let fb = self.bodies[m.b as usize].flags;
            if (fa | fb) & flags::CONTACT_EVENTS == 0 {
                continue;
            }
            if let Some(p) = m.points().iter().find(|p| p.depth >= Fx::ZERO) {
                now.push(((m.a.min(m.b), m.a.max(m.b)), p.point, m.normal));
            }
        }
        // Resting pairs that were not re-tested because they are asleep keep
        // touching (otherwise sleeping would emit spurious end events).
        let resting = |b: &Body| b.alive && (b.sleeping || b.is_static());
        for &(a, b) in &self.touching {
            if resting(&self.bodies[a as usize]) && resting(&self.bodies[b as usize]) {
                now.push(((a, b), Vec3::ZERO, Vec3::ZERO));
            }
        }
        // Stable sort keeps the first (real) point for each pair.
        now.sort_by_key(|e| e.0);
        now.dedup_by_key(|e| e.0);
        let mut touching = core::mem::take(&mut self.touching);
        let keys: Vec<(u32, u32)> = now.iter().map(|e| e.0).collect();
        diff_sorted(&touching, &keys, |p, began| {
            let (point, normal) = if began {
                let k = keys.binary_search(p).unwrap_or(0);
                (now[k].1, now[k].2)
            } else {
                (Vec3::ZERO, Vec3::ZERO)
            };
            self.events.push(Event {
                kind: if began { EventKind::ContactBegin } else { EventKind::ContactEnd },
                a: BodyHandle { index: p.0, generation: self.bodies[p.0 as usize].generation },
                b: BodyHandle { index: p.1, generation: self.bodies[p.1 as usize].generation },
                point,
                normal,
            });
        });
        touching.clear();
        touching.extend_from_slice(&keys);
        self.touching = touching;

        // Sensor events. Static sensors are not re-tested against sleeping
        // bodies, so carry those pairs over.
        let old_sensors = core::mem::take(&mut self.sensor_pairs);
        for &(sn, o) in &old_sensors {
            let (bs, bo) = (&self.bodies[sn as usize], &self.bodies[o as usize]);
            if bs.alive && bo.alive && bs.is_static() && bo.sleeping {
                sensors.push((sn, o));
            }
        }
        sensors.sort_unstable();
        sensors.dedup();
        diff_sorted(&old_sensors, &sensors, |p, began| {
            self.events.push(Event {
                kind: if began { EventKind::SensorEnter } else { EventKind::SensorExit },
                a: BodyHandle { index: p.0, generation: self.bodies[p.0 as usize].generation },
                b: BodyHandle { index: p.1, generation: self.bodies[p.1 as usize].generation },
                point: Vec3::ZERO,
                normal: Vec3::ZERO,
            });
        });
        self.sensor_pairs = sensors;
        self.scratch_pairs = old_sensors;

        self.manifolds = new;
        self.new_manifolds = old;
    }

    /// Exact overlap test between two bodies (used for sensors).
    pub(crate) fn overlap_bodies(&self, ia: u32, ib: u32) -> bool {
        let a = &self.bodies[ia as usize];
        let b = &self.bodies[ib as usize];
        let mut c = Contacts::default();
        collide_convex(&self.shapes[a.shape.0 as usize], &a.iso(), &self.shapes[b.shape.0 as usize], &b.iso(), Fx::ZERO, &mut c)
            && c.points[..c.count].iter().any(|p| p.depth >= Fx::ZERO)
    }

    #[inline(never)]
    fn solve(&mut self, dt: Fx) {
        let params = SolverParams {
            inv_dt: Fx::ONE / dt,
            slop: self.settings.slop,
            baumgarte: self.settings.baumgarte,
            max_correction: self.settings.max_correction_velocity,
            restitution_threshold: self.settings.restitution_threshold,
        };
        self.solver.reset();
        // Sparse reset: only entries touched last step are dirty.
        let n = self.bodies.len();
        if self.pseudo.len() != n {
            self.pseudo.clear();
            self.pseudo.resize(n, (Vec3::ZERO, Vec3::ZERO));
            self.solver_index.clear();
            self.solver_index.resize(n, NO_INDEX);
        } else {
            for &i in &self.solver_touched {
                self.pseudo[i as usize] = (Vec3::ZERO, Vec3::ZERO);
                self.solver_index[i as usize] = NO_INDEX;
            }
        }
        self.solver_touched.clear();
        for mi in 0..self.manifolds.len() {
            let m = self.manifolds[mi];
            let sa = self.solver_body(m.a);
            let sb = self.solver_body(m.b);
            if sa == 0 && sb == 0 {
                continue;
            }
            let ba = &self.bodies[m.a as usize];
            let bb = &self.bodies[m.b as usize];
            let friction = (ba.friction * bb.friction).sqrt();
            let restitution = ba.restitution.max(bb.restitution);
            let (xa, xb) = (ba.position, bb.position);
            self.solver.add(mi as u32, &m, sa, sb, xa, xb, friction, restitution, &params);
        }
        self.solver.warm_start();
        self.solver.solve(self.settings.velocity_iterations);
        self.solver.solve_positions(self.settings.position_iterations);
        self.solver.store_impulses(&mut self.manifolds);
        // Write velocities back.
        for &i in &self.solver_touched {
            let i = i as usize;
            let si = self.solver_index[i];
            if si != NO_INDEX && si != 0 {
                let b = &mut self.bodies[i];
                if b.is_dynamic() {
                    let sv = self.solver.vel[si as usize];
                    let pv = self.solver.pvel[si as usize];
                    b.linear_velocity = sv.v;
                    b.angular_velocity = sv.w;
                    // Pseudo velocities only move the body this step.
                    self.pseudo[i] = (pv.v, pv.w);
                }
            }
        }
    }

    fn solver_body(&mut self, i: u32) -> u32 {
        let existing = self.solver_index[i as usize];
        if existing != NO_INDEX {
            return existing;
        }
        let b = &self.bodies[i as usize];
        let si = match b.kind {
            BodyKind::Static => 0,
            BodyKind::Kinematic => self.solver.push_body(b.linear_velocity, b.angular_velocity, Fx::ZERO, Mat3::ZERO),
            BodyKind::Dynamic => {
                let ii = Mat3::rotate_diagonal(&b.rotation.to_mat3(), b.inv_inertia_local);
                self.solver.push_body(b.linear_velocity, b.angular_velocity, b.inv_mass, ii)
            }
        };
        self.solver_index[i as usize] = si;
        self.solver_touched.push(i);
        si
    }

    #[inline(never)]
    fn update_sleep(&mut self, dt: Fx) {
        let n = self.bodies.len();
        if self.uf_parent.len() != n {
            self.uf_parent.resize(n, 0);
            self.uf_sleep.resize(n, Fx::MAX);
        }
        let lin2 = self.settings.sleep_linear_threshold * self.settings.sleep_linear_threshold;
        let ang2 = self.settings.sleep_angular_threshold * self.settings.sleep_angular_threshold;
        // Union-find over moving bodies only. Roots end up as the minimum
        // index of each island regardless of visiting order (deterministic).
        for &i in &self.movers {
            self.uf_parent[i as usize] = i;
            self.uf_sleep[i as usize] = Fx::MAX;
            let b = &mut self.bodies[i as usize];
            if !b.is_dynamic() || b.sleeping {
                continue;
            }
            if b.flags & flags::NO_SLEEP == 0 && b.linear_velocity.length_sq() < lin2 && b.angular_velocity.length_sq() < ang2 {
                b.sleep_time += dt;
            } else {
                b.sleep_time = Fx::ZERO;
            }
        }
        for mi in 0..self.manifolds.len() {
            let m = &self.manifolds[mi];
            let (a, b) = (m.a, m.b);
            let ka = self.bodies[a as usize].kind;
            let kb = self.bodies[b as usize].kind;
            if ka == BodyKind::Dynamic && kb == BodyKind::Dynamic {
                uf_union(&mut self.uf_parent, a, b);
            } else {
                // Touching a moving kinematic body keeps a body awake.
                let (dynb, other) = if ka == BodyKind::Dynamic { (a, b) } else { (b, a) };
                let o = &self.bodies[other as usize];
                if o.kind == BodyKind::Kinematic && !(o.linear_velocity.is_zero() && o.angular_velocity.is_zero()) {
                    self.bodies[dynb as usize].sleep_time = Fx::ZERO;
                }
            }
        }
        for k in 0..self.movers.len() {
            let i = self.movers[k];
            let b = &self.bodies[i as usize];
            if b.is_dynamic() && !b.sleeping {
                let r = uf_find(&mut self.uf_parent, i) as usize;
                self.uf_sleep[r] = self.uf_sleep[r].min(b.sleep_time);
            }
        }
        let tts = self.settings.time_to_sleep;
        for k in 0..self.movers.len() {
            let i = self.movers[k];
            let r = uf_find(&mut self.uf_parent, i) as usize;
            let b = &mut self.bodies[i as usize];
            if b.is_dynamic() && !b.sleeping && self.uf_sleep[r] >= tts {
                b.sleeping = true;
                b.linear_velocity = Vec3::ZERO;
                b.angular_velocity = Vec3::ZERO;
            }
        }
    }
}

fn make_manifold(a: u32, b: u32, sub: u32, c: &Contacts, old: &[Manifold]) -> Manifold {
    let mut m = Manifold { a, b, sub, normal: c.normal, count: c.count as u8, ..Default::default() };
    let prev = old
        .binary_search_by_key(&(a, b, sub), |m| m.key())
        .ok()
        .map(|i| &old[i])
        // Only reuse impulses if the normal barely changed.
        .filter(|pm| pm.normal.dot(c.normal) > fx!(0.95));
    let mut used = [false; crate::collide::MAX_POINTS];
    for k in 0..c.count {
        let cp = &c.points[k];
        m.points[k] = ManifoldPoint { point: cp.point, depth: cp.depth, id: cp.id, ..Default::default() };
    }
    let Some(pm) = prev else { return m };
    m.friction_impulse = pm.friction_impulse;
    m.twist_impulse = pm.twist_impulse;
    // Match each new point to an unused old point: same feature id within the
    // tolerance first, otherwise the nearest one within the tolerance. Ids alone
    // are not trusted because clipped features can swap ids between frames.
    let tol2 = WARM_MATCH_DIST * WARM_MATCH_DIST;
    for k in 0..c.count {
        let mut best = (Fx::MAX, usize::MAX);
        for (j, p) in pm.points().iter().enumerate() {
            if used[j] {
                continue;
            }
            let d = (p.point - c.points[k].point).length_sq();
            if d >= tol2 {
                continue;
            }
            // Same-id points win ties against any proximity match.
            let score = if p.id == c.points[k].id { Fx::MIN } else { d };
            if score < best.0 {
                best = (score, j);
            }
        }
        if best.1 != usize::MAX {
            used[best.1] = true;
            m.points[k].normal_impulse = pm.points[best.1].normal_impulse;
        }
    }
    m
}

const WARM_MATCH_DIST: Fx = fx!(0.05);

/// Merge-walk two sorted lists, reporting additions (`true`) and removals.
fn diff_sorted(old: &[(u32, u32)], new: &[(u32, u32)], mut f: impl FnMut(&(u32, u32), bool)) {
    let (mut i, mut j) = (0, 0);
    while i < old.len() || j < new.len() {
        if j >= new.len() || (i < old.len() && old[i] < new[j]) {
            f(&old[i], false);
            i += 1;
        } else if i >= old.len() || new[j] < old[i] {
            f(&new[j], true);
            j += 1;
        } else {
            i += 1;
            j += 1;
        }
    }
}

fn uf_find(p: &mut [u32], mut x: u32) -> u32 {
    while p[x as usize] != x {
        let gp = p[p[x as usize] as usize];
        p[x as usize] = gp;
        x = gp;
    }
    x
}

fn uf_union(p: &mut [u32], a: u32, b: u32) {
    let ra = uf_find(p, a);
    let rb = uf_find(p, b);
    if ra != rb {
        // Smaller index becomes root: deterministic.
        if ra < rb {
            p[rb as usize] = ra;
        } else {
            p[ra as usize] = rb;
        }
    }
}
