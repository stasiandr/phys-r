//! Deterministic kinematic character controller for first-person games.
//!
//! A character is a capsule driven by [`CharacterInput`]:
//!
//! * Quake-style ground/air acceleration, friction, gravity, jumping (with
//!   coyote time and optional release-to-rejump), walk/sprint/crouch speeds.
//! * Collide-and-slide with crease handling, step climbing, slope limits,
//!   ground snapping (walking down stairs/slopes stays grounded), sliding
//!   down steep slopes, ceiling hits.
//! * Real crouching: the capsule shrinks (feet stay planted on the ground,
//!   head stays put in the air), and standing up is refused under ceilings.
//! * Moving platforms: carried by kinematic/dynamic ground, momentum is
//!   inherited when jumping or walking off.
//! * Fly mode (ladders, swimming, spectators with collision) and noclip.
//! * Knockback via [`World::add_character_velocity`], landing speed for fall
//!   damage, collision flags (below/above/sides).
//! * Owns a kinematic capsule body, so characters block each other, push
//!   dynamic props, are hit by raycasts and are rewound for lag compensation.
//!
//! The controller is a pure function of world state and input, so prediction
//! and server reconciliation are exact: replaying the same inputs from the
//! same snapshot reproduces the same positions bit for bit.

use crate::body::{flags, BodyDesc, BodyHandle, BodyKind};
use crate::fixed::Fx;
use crate::fx;
use crate::geometry::Aabb;
use crate::math::{Quat, Vec3};
use crate::query::{ObRef, QueryFilter};
use crate::shape::{Shape, ShapeId};
use crate::world::{World, WorldError};

/// Generational handle to a character.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Default)]
pub struct CharacterHandle {
    pub index: u32,
    pub generation: u32,
}

impl CharacterHandle {
    pub fn to_bits(self) -> u64 {
        ((self.generation as u64) << 32) | self.index as u64
    }
    pub fn from_bits(b: u64) -> Self {
        CharacterHandle { index: b as u32, generation: (b >> 32) as u32 }
    }
}

/// Collision flags reported for the last tick ([`Character::collision_flags`]).
pub mod collision {
    /// Standing on walkable ground.
    pub const BELOW: u32 = 1 << 0;
    /// Bumped a ceiling.
    pub const ABOVE: u32 = 1 << 1;
    /// Touched a wall / steep surface.
    pub const SIDES: u32 = 1 << 2;
    /// Landed this tick (see [`super::Character::landing_speed`]).
    pub const LANDED: u32 = 1 << 3;
    /// Jumped this tick.
    pub const JUMPED: u32 = 1 << 4;
    /// Standing up was blocked by geometry (still crouched).
    pub const STAND_BLOCKED: u32 = 1 << 5;
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[repr(u8)]
pub enum MoveMode {
    /// Normal walking with gravity.
    #[default]
    Walk = 0,
    /// Free 3D movement with collision, no gravity (ladders, water, spectators).
    Fly = 1,
    /// Free movement without collision.
    Noclip = 2,
}

impl MoveMode {
    pub fn from_u8(v: u8) -> MoveMode {
        match v {
            1 => MoveMode::Fly,
            2 => MoveMode::Noclip,
            _ => MoveMode::Walk,
        }
    }
}

/// Movement tuning (all values in SI units).
#[derive(Clone, Debug)]
pub struct MovementSettings {
    pub walk_speed: Fx,
    pub sprint_speed: Fx,
    pub crouch_speed: Fx,
    pub ground_accel: Fx,
    pub air_accel: Fx,
    /// Max wish speed while airborne (Quake air control).
    pub air_speed_cap: Fx,
    pub friction: Fx,
    pub stop_speed: Fx,
    pub gravity: Fx,
    pub jump_speed: Fx,
    pub max_fall_speed: Fx,
    /// Require releasing jump between jumps (disables auto bunny-hop).
    pub jump_requires_release: bool,
    /// Ticks after leaving an edge during which a jump is still allowed.
    pub coyote_ticks: u32,
}

impl Default for MovementSettings {
    fn default() -> Self {
        MovementSettings {
            walk_speed: fx!(6.0),
            sprint_speed: fx!(9.0),
            crouch_speed: fx!(3.0),
            ground_accel: fx!(10.0),
            air_accel: fx!(2.0),
            air_speed_cap: fx!(1.0),
            friction: fx!(6.0),
            stop_speed: fx!(1.5),
            gravity: fx!(20.0),
            jump_speed: fx!(7.0),
            max_fall_speed: fx!(50.0),
            jump_requires_release: true,
            coyote_ticks: 6,
        }
    }
}

#[derive(Clone, Debug)]
pub struct CharacterDesc {
    /// Capsule centre.
    pub position: Vec3,
    pub radius: Fx,
    /// Standing capsule height (including the hemispheres).
    pub height: Fx,
    /// Crouched capsule height.
    pub crouch_height: Fx,
    pub step_height: Fx,
    /// Cosine of the steepest walkable slope.
    pub max_slope_cos: Fx,
    pub skin: Fx,
    pub snap_distance: Fx,
    /// Mass used when pushing dynamic bodies.
    pub mass: Fx,
    /// Fraction of the character's momentum transferred into pushed props.
    pub push_strength: Fx,
    pub layer: u8,
    /// What the character collides with.
    pub mask: u32,
    /// Extra body flags for the character's capsule (e.g. HISTORY).
    pub body_flags: u32,
    pub movement: MovementSettings,
    pub user_data: u64,
}

impl Default for CharacterDesc {
    fn default() -> Self {
        CharacterDesc {
            position: Vec3::ZERO,
            radius: fx!(0.4),
            height: fx!(1.8),
            crouch_height: fx!(1.2),
            step_height: fx!(0.35),
            max_slope_cos: fx!(core::f64::consts::FRAC_1_SQRT_2), // 45 degrees
            skin: fx!(0.01),
            snap_distance: fx!(0.3),
            mass: fx!(80.0),
            push_strength: fx!(0.5),
            layer: 1,
            mask: u32::MAX,
            body_flags: flags::HISTORY,
            movement: MovementSettings::default(),
            user_data: 0,
        }
    }
}

/// Per-tick input. Quantise these on the network (e.g. `i8` axes, `u16` yaw)
/// and convert with `Fx::from_ratio` so every peer sees identical values.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct CharacterInput {
    /// Strafe axis, -1 (left) .. 1 (right).
    pub move_x: Fx,
    /// Vertical axis for fly mode (ladders, swimming), -1 .. 1.
    pub move_y: Fx,
    /// Forward axis, -1 (back) .. 1 (forward).
    pub move_z: Fx,
    /// View yaw in radians (rotation about +Y; 0 looks down +Z).
    pub yaw: Fx,
    pub jump: bool,
    pub sprint: bool,
    pub crouch: bool,
}

/// Character state (part of snapshots).
#[derive(Clone, Debug)]
pub struct Character {
    pub(crate) alive: bool,
    pub(crate) generation: u32,
    pub(crate) body: BodyHandle,
    pub(crate) stand_shape: ShapeId,
    pub(crate) crouch_shape: ShapeId,
    pub desc: CharacterDesc,
    /// Capsule centre.
    pub position: Vec3,
    pub velocity: Vec3,
    pub grounded: bool,
    pub ground_normal: Vec3,
    /// Body we stand on (invalid when airborne).
    pub ground_body: BodyHandle,
    /// Velocity of the ground under our feet (moving platforms).
    pub ground_velocity: Vec3,
    pub crouched: bool,
    pub mode: MoveMode,
    /// [`collision`] flags from the last tick.
    pub collision_flags: u32,
    /// Downward speed at the moment of landing (only set on the landing tick).
    pub landing_speed: Fx,
    pub(crate) air_ticks: u32,
    pub input: CharacterInput,
    pub(crate) jump_latched: bool,
}

impl Character {
    /// Current capsule height (depends on crouching).
    #[inline]
    pub fn height(&self) -> Fx {
        if self.crouched {
            self.desc.crouch_height
        } else {
            self.desc.height
        }
    }

    /// Half the inner segment length of the current capsule.
    #[inline]
    pub fn half_height(&self) -> Fx {
        (self.height().shr(1) - self.desc.radius).max(Fx::ZERO)
    }

    pub fn body(&self) -> BodyHandle {
        self.body
    }

    /// Feet position (bottom of the capsule).
    pub fn feet(&self) -> Vec3 {
        self.position - Vec3::new(Fx::ZERO, self.height().shr(1), Fx::ZERO)
    }

    /// Top of the capsule.
    pub fn head(&self) -> Vec3 {
        self.position + Vec3::new(Fx::ZERO, self.height().shr(1), Fx::ZERO)
    }
}

/// Outcome of a collide-and-slide move.
#[derive(Copy, Clone, Debug, Default)]
pub struct MoveResult {
    pub position: Vec3,
    /// Up to four surfaces touched during the move (normals toward the capsule).
    pub hit_normals: [Vec3; 4],
    pub hit_count: u32,
}

impl MoveResult {
    fn add_hit(&mut self, n: Vec3) {
        if (self.hit_count as usize) < 4 {
            self.hit_normals[self.hit_count as usize] = n;
            self.hit_count += 1;
        }
    }
}

/// Result of [`World::compute_character`].
#[derive(Copy, Clone, Debug)]
pub(crate) struct CharOut {
    position: Vec3,
    velocity: Vec3,
    grounded: bool,
    ground_normal: Vec3,
    ground_body: BodyHandle,
    ground_velocity: Vec3,
    crouched: bool,
    collision_flags: u32,
    landing_speed: Fx,
    air_ticks: u32,
    jump_latched: bool,
    pushes: [(BodyHandle, Vec3); 4],
    np: usize,
}

/// One character's slot in a (parallel) character phase.
#[derive(Default)]
pub(crate) struct CharJob {
    index: usize,
    scratch: Vec<ObRef>,
    out: Option<CharOut>,
}

#[derive(Copy, Clone)]
struct Capsule {
    hh: Fx,
    r: Fx,
}

impl Capsule {
    #[inline]
    fn seg(&self, p: Vec3) -> (Vec3, Vec3) {
        let h = Vec3::new(Fx::ZERO, self.hh, Fx::ZERO);
        (p - h, p + h)
    }
}

const NO_SKIP: u32 = u32::MAX;

impl World {
    fn capsule_shape(&mut self, hh: Fx, r: Fx) -> ShapeId {
        // Reuse an identical capsule shape so the registry stays stable.
        let probe = Shape::capsule(hh, r);
        match self.shapes.iter().position(|s| *s == probe) {
            Some(i) => ShapeId(i as u32),
            None => self.add_shape(probe),
        }
    }

    pub fn create_character(&mut self, desc: &CharacterDesc) -> Result<CharacterHandle, WorldError> {
        let mut desc = desc.clone();
        desc.crouch_height = desc.crouch_height.clamp(desc.radius.shl(1), desc.height);
        let stand_shape = self.capsule_shape((desc.height.shr(1) - desc.radius).max(Fx::ZERO), desc.radius);
        let crouch_shape = self.capsule_shape((desc.crouch_height.shr(1) - desc.radius).max(Fx::ZERO), desc.radius);
        let body = self.create_body(&BodyDesc {
            kind: BodyKind::Kinematic,
            shape: stand_shape,
            position: desc.position,
            layer: desc.layer,
            mask: desc.mask,
            flags: desc.body_flags | flags::CHARACTER,
            friction: Fx::ZERO,
            user_data: desc.user_data,
            ..Default::default()
        })?;
        let ch = Character {
            alive: true,
            generation: 0,
            body,
            stand_shape,
            crouch_shape,
            position: desc.position,
            desc,
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
        };
        let index = match self.char_free.pop() {
            Some(i) => i,
            None => {
                self.characters.push(Character { alive: false, ..ch.clone() });
                self.characters.len() as u32 - 1
            }
        };
        let generation = self.characters[index as usize].generation.wrapping_add(1).max(1);
        self.characters[index as usize] = Character { generation, ..ch };
        Ok(CharacterHandle { index, generation })
    }

    pub fn destroy_character(&mut self, h: CharacterHandle) -> bool {
        let Some(ch) = self.character(h) else { return false };
        let body = ch.body;
        self.destroy_body(body);
        self.characters[h.index as usize].alive = false;
        self.char_free.push(h.index);
        true
    }

    pub fn character(&self, h: CharacterHandle) -> Option<&Character> {
        match self.characters.get(h.index as usize) {
            Some(c) if c.alive && c.generation == h.generation => Some(c),
            _ => None,
        }
    }

    pub fn character_mut(&mut self, h: CharacterHandle) -> Option<&mut Character> {
        match self.characters.get_mut(h.index as usize) {
            Some(c) if c.alive && c.generation == h.generation => Some(c),
            _ => None,
        }
    }

    /// Find the character owning a body (e.g. from a raycast hit).
    pub fn character_of_body(&self, body: BodyHandle) -> Option<CharacterHandle> {
        let b = self.body(body)?;
        if b.flags & flags::CHARACTER == 0 {
            return None;
        }
        self.characters
            .iter()
            .position(|c| c.alive && c.body == body)
            .map(|i| CharacterHandle { index: i as u32, generation: self.characters[i].generation })
    }

    /// Input consumed by the next [`World::step`].
    pub fn set_character_input(&mut self, h: CharacterHandle, input: CharacterInput) -> bool {
        match self.character_mut(h) {
            Some(c) => {
                c.input = input;
                true
            }
            None => false,
        }
    }

    pub fn set_character_mode(&mut self, h: CharacterHandle, mode: MoveMode) -> bool {
        match self.character_mut(h) {
            Some(c) => {
                c.mode = mode;
                if mode != MoveMode::Walk {
                    c.grounded = false;
                }
                true
            }
            None => false,
        }
    }

    /// Knockback / explosions / jump pads. An upward kick un-grounds the
    /// character so ground snapping does not cancel it.
    pub fn add_character_velocity(&mut self, h: CharacterHandle, dv: Vec3) -> bool {
        match self.character_mut(h) {
            Some(c) => {
                c.velocity += dv;
                if dv.y > Fx::ZERO {
                    c.grounded = false;
                    c.air_ticks = u32::MAX;
                }
                true
            }
            None => false,
        }
    }

    pub fn set_character_velocity(&mut self, h: CharacterHandle, v: Vec3) -> bool {
        match self.character_mut(h) {
            Some(c) => {
                if v.y > Fx::ZERO {
                    c.grounded = false;
                    c.air_ticks = u32::MAX;
                }
                c.velocity = v;
                true
            }
            None => false,
        }
    }

    pub fn teleport_character(&mut self, h: CharacterHandle, position: Vec3) -> bool {
        let Some(c) = self.character_mut(h) else { return false };
        c.position = position;
        c.velocity = Vec3::ZERO;
        c.grounded = false;
        c.ground_body = BodyHandle::INVALID;
        c.ground_velocity = Vec3::ZERO;
        let body = c.body;
        self.set_pose(body, position, Quat::IDENTITY);
        true
    }

    /// Advance a single character immediately (outside of `step`). Use this
    /// for local-player prediction / input replay without simulating the rest
    /// of the world.
    pub fn simulate_character(&mut self, h: CharacterHandle, input: CharacterInput, dt: Fx) -> bool {
        if self.character(h).is_none() {
            return false;
        }
        self.characters[h.index as usize].input = input;
        self.update_mover_tree();
        if self.static_dirty {
            self.rebuild_static();
        }
        self.update_character(h.index as usize, dt);
        self.sync_character_body(h.index as usize);
        let body = self.characters[h.index as usize].body;
        let (p, r) = (self.characters[h.index as usize].position, Quat::IDENTITY);
        self.set_pose(body, p, r);
        true
    }

    #[inline(never)]
    pub(crate) fn step_characters(&mut self, dt: Fx) {
        if self.characters.iter().all(|c| !c.alive) {
            return;
        }
        // Mover bounds were refreshed at the end of the previous step.
        self.update_mover_tree();
        if self.static_dirty {
            self.rebuild_static();
        }
        let inv_dt = Fx::ONE / dt;
        // Compute every move from the same start-of-tick world (in parallel
        // when a pool is configured), then apply in index order.
        let mut jobs = core::mem::take(&mut self.char_jobs);
        jobs.retain(|_| false);
        for i in 0..self.characters.len() {
            if self.characters[i].alive {
                let scratch = self.char_scratch_pool.pop().unwrap_or_default();
                jobs.push(CharJob { index: i, scratch, out: None });
            }
        }
        #[cfg(not(target_family = "wasm"))]
        let mut pool = self.pool.take();
        {
            let me = &*self;
            let run = |_: usize, j: &mut CharJob| {
                j.out = Some(me.compute_character(j.index, dt, &mut j.scratch));
            };
            #[cfg(not(target_family = "wasm"))]
            match pool.as_mut() {
                Some(p) if jobs.len() > 1 => p.for_each_mut(&mut jobs, &run),
                _ => jobs.iter_mut().enumerate().for_each(|(k, j)| run(k, j)),
            }
            #[cfg(target_family = "wasm")]
            jobs.iter_mut().enumerate().for_each(|(k, j)| run(k, j));
        }
        #[cfg(not(target_family = "wasm"))]
        {
            self.pool = pool;
        }
        for j in jobs.iter_mut() {
            let i = j.index;
            let start = self.characters[i].position;
            if let Some(out) = j.out.take() {
                self.apply_character(i, &out);
            }
            // Drive the kinematic body so the solver pushes props; the exact
            // pose is restored in `sync_character_bodies`.
            let c = &self.characters[i];
            let v = (c.position - start).scale(inv_dt);
            let body = c.body;
            if let Some(b) = self.body_mut(body) {
                b.linear_velocity = v;
                b.angular_velocity = Vec3::ZERO;
            }
            self.char_scratch_pool.push(core::mem::take(&mut j.scratch));
        }
        jobs.clear();
        self.char_jobs = jobs;
    }

    fn sync_character_body(&mut self, i: usize) {
        let c = &self.characters[i];
        let (body, pos, shape) = (c.body, c.position, if c.crouched { c.crouch_shape } else { c.stand_shape });
        if let Some(b) = self.body_mut(body) {
            b.position = pos;
            b.rotation = Quat::IDENTITY;
            b.shape = shape;
        }
    }

    pub(crate) fn sync_character_bodies(&mut self) {
        for i in 0..self.characters.len() {
            if self.characters[i].alive {
                self.sync_character_body(i);
            }
        }
    }

    /// Advance one character (serial path).
    fn update_character(&mut self, i: usize, dt: Fx) {
        let mut obs = core::mem::take(&mut self.scratch_obstacles);
        let out = self.compute_character(i, dt, &mut obs);
        self.scratch_obstacles = obs;
        self.apply_character(i, &out);
    }

    /// Write a computed move back (state and deferred prop pushes).
    pub(crate) fn apply_character(&mut self, i: usize, o: &CharOut) {
        for (h, imp) in o.pushes.iter().take(o.np) {
            self.apply_impulse(*h, *imp, None);
        }
        let ch = &mut self.characters[i];
        ch.position = o.position;
        ch.velocity = o.velocity;
        ch.grounded = o.grounded;
        ch.ground_normal = o.ground_normal;
        ch.ground_body = o.ground_body;
        ch.ground_velocity = o.ground_velocity;
        ch.crouched = o.crouched;
        ch.collision_flags = o.collision_flags;
        ch.landing_speed = o.landing_speed;
        ch.air_ticks = o.air_ticks;
        ch.jump_latched = o.jump_latched;
    }

    /// Compute one character's move from the start-of-tick world without
    /// mutating anything, so all characters can run in parallel. Characters
    /// see each other's start-of-tick bodies and prop pushes are applied
    /// afterwards in index order: the result is independent of scheduling.
    #[inline(never)]
    pub(crate) fn compute_character(&self, i: usize, dt: Fx, obs: &mut Vec<ObRef>) -> CharOut {
        let c = &self.characters[i];
        let mv = &c.desc.movement;
        let input = c.input;
        let skin = c.desc.skin;
        let r = c.desc.radius;
        let stand = Capsule { hh: (c.desc.height.shr(1) - r).max(Fx::ZERO), r };
        let crouch = Capsule { hh: (c.desc.crouch_height.shr(1) - r).max(Fx::ZERO), r };
        let filter = QueryFilter { mask: c.desc.mask, exclude: c.body, ..Default::default() };
        let mut flags_out = 0u32;

        // --- Noclip: no collision at all ---
        if c.mode == MoveMode::Noclip {
            let mut v = c.velocity;
            let wish = wish_vector(&input, true);
            let (dir, amt) = wish.normalize_with_len();
            friction(&mut v, mv, dt, true);
            accelerate(&mut v, dir, top_speed(mv, &input) * amt.min(Fx::ONE), mv.ground_accel, dt);
            return CharOut {
                position: c.position + v.scale(dt),
                velocity: v,
                grounded: false,
                ground_normal: c.ground_normal,
                ground_body: BodyHandle::INVALID,
                ground_velocity: Vec3::ZERO,
                crouched: c.crouched,
                collision_flags: 0,
                landing_speed: Fx::ZERO,
                air_ticks: c.air_ticks,
                jump_latched: c.jump_latched,
                pushes: [(BodyHandle::INVALID, Vec3::ZERO); 4],
                np: 0,
            };
        }

        // --- Gather obstacles once for the whole move ---
        let disp_bound = c.velocity.length() * dt + mv.jump_speed * dt + c.ground_velocity.length() * dt;
        let reach = disp_bound + c.desc.step_height + c.desc.snap_distance + skin * Fx::from_int(4) + (stand.hh - crouch.hh).abs();
        let (s0, s1) = stand.seg(c.position);
        let aabb = Aabb::new(s0.min(s1), s0.max(s1)).expand(reach + r);
        self.gather_obstacles(&aabb, &filter, obs);
        let obs: &[ObRef] = obs;

        let mut pos = c.position;
        let mut crouched = c.crouched;
        let grounded_before = c.grounded && c.mode == MoveMode::Walk;

        // --- Crouch / stand (changes the capsule) ---
        let delta = (stand.hh - crouch.hh).shl(1); // full height difference
        if input.crouch && !crouched {
            crouched = true;
            // Ground: keep feet planted. Air: keep the head where it is
            // (tucking the legs, as in crouch-jumping).
            let h = Vec3::new(Fx::ZERO, delta.shr(1), Fx::ZERO);
            pos = if grounded_before { pos - h } else { pos + h };
        } else if !input.crouch && crouched {
            let h = Vec3::new(Fx::ZERO, delta.shr(1), Fx::ZERO);
            let candidates = if grounded_before { [pos + h, pos + h] } else { [pos - h, pos + h] };
            let mut stood = false;
            for cand in candidates {
                if self.capsule_fits(obs, &stand, cand, skin) {
                    pos = cand;
                    crouched = false;
                    stood = true;
                    break;
                }
            }
            if !stood {
                flags_out |= crate::character::collision::STAND_BLOCKED;
            }
        }
        let cap = if crouched { crouch } else { stand };

        // --- Velocity from input (Quake-style) ---
        let fly = c.mode == MoveMode::Fly;
        let wish = wish_vector(&input, fly);
        let (wish_dir, wish_amount) = wish.normalize_with_len();
        let wish_speed = top_speed_crouch(mv, &input, crouched) * wish_amount.min(Fx::ONE);

        let mut v = c.velocity;
        let mut grounded = grounded_before;
        let mut jump_latched = c.jump_latched && input.jump;
        let mut jumped = false;
        let mut air_ticks = c.air_ticks;
        let mut ground_vel = c.ground_velocity;
        if fly {
            friction(&mut v, mv, dt, true);
            accelerate(&mut v, wish_dir, wish_speed, mv.ground_accel, dt);
            grounded = false;
        } else {
            let can_jump = grounded || air_ticks <= mv.coyote_ticks;
            if grounded {
                friction(&mut v, mv, dt, false);
                accelerate(&mut v, wish_dir, wish_speed, mv.ground_accel, dt);
                v.y = Fx::ZERO;
            } else {
                accelerate(&mut v, wish_dir, wish_speed.min(mv.air_speed_cap), mv.air_accel, dt);
                v.y = (v.y - mv.gravity * dt).max(-mv.max_fall_speed);
            }
            if input.jump && can_jump && !(mv.jump_requires_release && jump_latched) {
                // Inherit the platform's motion when leaving it.
                v += ground_vel;
                v.y = v.y.max(Fx::ZERO) + mv.jump_speed;
                grounded = false;
                jumped = true;
                jump_latched = true;
                air_ticks = u32::MAX;
                ground_vel = Vec3::ZERO;
                flags_out |= collision::JUMPED;
            }
        }
        if !input.jump {
            jump_latched = false;
        }

        let ground_idx = if grounded && c.ground_body.generation != 0 { c.ground_body.index } else { NO_SKIP };

        // --- Depenetration (pushed by movers, spawned inside geometry) ---
        pos = self.depenetrate(obs, &cap, pos, skin);

        // --- Moving platform carry (ignoring the platform itself, which has
        // not moved yet this tick) ---
        if grounded && !ground_vel.is_zero() {
            let carry = self.slide(obs, &cap, pos, ground_vel.scale(dt), false, &c.desc, ground_idx);
            pos = carry.position;
        }

        // --- Move (pass A: plain slide) ---
        let disp = v.scale(dt);
        let mut res = self.slide(obs, &cap, pos, disp, grounded, &c.desc, NO_SKIP);
        // --- Pass B: step up and over obstacles ---
        if grounded && c.desc.step_height.0 > 0 && res.hit_count > 0 {
            let hd = Vec3::new(disp.x, Fx::ZERO, disp.z);
            let want = hd.length();
            let got_a = horiz_len(res.position - pos);
            if want.0 > 0 && got_a < want - fx!(0.001) {
                if let Some(rb) = self.try_step(obs, &cap, pos, hd, &c.desc, &filter) {
                    if horiz_len(rb.position - pos) > got_a + fx!(0.001) {
                        res = rb;
                    }
                }
            }
        }
        pos = res.position;

        // --- Ground probe / snap ---
        let was_grounded = grounded;
        grounded = false;
        let mut ground_normal = Vec3::Y;
        let mut ground_body = BodyHandle::INVALID;
        let mut new_ground_vel = Vec3::ZERO;
        if !fly && !jumped && v.y <= fx!(0.001) {
            let probe_len = if was_grounded { c.desc.snap_distance } else { skin * Fx::from_int(3) };
            let down = Vec3::new(Fx::ZERO, -(probe_len + skin), Fx::ZERO);
            let (a0, a1) = cap.seg(pos);
            if let Some((t, n, orf, p)) = self.sweep_rounded(obs, a0, a1, cap.r, down, skin, NO_SKIP) {
                if let Some(gn) = self.walkable(&filter, p, n, c.desc.max_slope_cos) {
                    grounded = true;
                    ground_normal = gn;
                    pos += down.scale(t);
                    let gb = &self.bodies[orf.body as usize];
                    ground_body = self.handle_of(orf.body);
                    if !gb.is_static() && gb.flags & flags::CHARACTER == 0 {
                        new_ground_vel = gb.linear_velocity + gb.angular_velocity.cross(p - gb.position);
                    }
                }
            }
        }

        // Walking off a moving platform keeps its momentum.
        if was_grounded && !grounded && !jumped {
            v += ground_vel;
        }

        // --- Velocity clipping and collision flags ---
        for k in 0..res.hit_count as usize {
            let n = res.hit_normals[k];
            if n.y >= c.desc.max_slope_cos {
                flags_out |= collision::BELOW;
            } else if n.y <= fx!(-0.5) {
                flags_out |= collision::ABOVE;
            } else {
                flags_out |= collision::SIDES;
            }
            let vn = v.dot(n);
            if vn < Fx::ZERO {
                v -= n.scale(vn);
            }
        }
        let mut landing_speed = Fx::ZERO;
        if grounded {
            flags_out |= collision::BELOW;
            if !c.grounded {
                flags_out |= collision::LANDED;
                landing_speed = (-c.velocity.y).max(Fx::ZERO);
            }
            if v.y < Fx::ZERO {
                v.y = Fx::ZERO;
            }
            air_ticks = 0;
        } else if air_ticks != u32::MAX {
            air_ticks = air_ticks.saturating_add(1);
        }

        // --- Push dynamic bodies we ran into ---
        let mut pushes: [(BodyHandle, Vec3); 4] = [(BodyHandle::INVALID, Vec3::ZERO); 4];
        let mut np = 0;
        if c.desc.push_strength.0 > 0 && !disp.is_zero() {
            let (a0, a1) = cap.seg(pos);
            for orf in obs.iter() {
                if np >= 4 {
                    break;
                }
                let b = &self.bodies[orf.body as usize];
                if !b.is_dynamic() {
                    continue;
                }
                let shape = &self.shapes[orf.shape as usize];
                let dist = crate::upright::distance(pos, cap.hh, cap.r, shape, &orf.iso)
                    .unwrap_or_else(|| crate::collide::rounded_distance(a0, a1, cap.r, shape, &orf.iso));
                if let Some((d, n, _)) = dist {
                    let speed = c.velocity.dot(-n);
                    if d <= skin * Fx::from_int(3) && speed > Fx::ZERO {
                        let imp = (-n).scale(speed * c.desc.mass * c.desc.push_strength * dt);
                        pushes[np] = (self.handle_of(orf.body), imp);
                        np += 1;
                    }
                }
            }
        }
        CharOut {
            position: pos,
            velocity: v,
            grounded,
            ground_normal,
            ground_body,
            ground_velocity: new_ground_vel,
            crouched,
            collision_flags: flags_out,
            landing_speed,
            air_ticks,
            jump_latched,
            pushes,
            np,
        }
    }

    /// Does a capsule at `pos` stay clear of every obstacle?
    fn capsule_fits(&self, obs: &[ObRef], cap: &Capsule, pos: Vec3, skin: Fx) -> bool {
        let (a0, a1) = cap.seg(pos);
        let probe = Aabb::new(a0.min(a1), a0.max(a1)).expand(cap.r);
        obs.iter().all(|o| {
            !o.aabb.overlaps(&probe)
                || self.rounded_penetration(a0, a1, cap.r, &self.shapes[o.shape as usize], &o.iso).is_none_or(|(d, _)| d <= skin.shr(1))
        })
    }

    fn depenetrate(&self, obs: &[ObRef], cap: &Capsule, mut pos: Vec3, skin: Fx) -> Vec3 {
        for _ in 0..4 {
            let mut moved = false;
            let (a0, a1) = cap.seg(pos);
            let probe = Aabb::new(a0.min(a1), a0.max(a1)).expand(cap.r);
            for o in obs {
                if !o.aabb.overlaps(&probe) {
                    continue;
                }
                let (a0, a1) = cap.seg(pos);
                if let Some((depth, n)) = self.rounded_penetration(a0, a1, cap.r, &self.shapes[o.shape as usize], &o.iso) {
                    if depth > Fx(64) {
                        pos += n.scale(depth + skin.shr(1));
                        moved = true;
                    }
                }
            }
            if !moved {
                break;
            }
        }
        pos
    }

    #[allow(clippy::too_many_arguments)]
    fn slide(
        &self,
        obs: &[ObRef],
        cap: &Capsule,
        mut pos: Vec3,
        disp: Vec3,
        grounded: bool,
        desc: &CharacterDesc,
        skip: u32,
    ) -> MoveResult {
        let mut res = MoveResult::default();
        let mut remaining = disp;
        let mut planes = [Vec3::ZERO; 4];
        let mut np = 0;
        for _ in 0..4 {
            if remaining.length_sq() < fx!(1e-10) {
                break;
            }
            let (a0, a1) = cap.seg(pos);
            let Some((t, n, _, _)) = self.sweep_rounded(obs, a0, a1, cap.r, remaining, desc.skin, skip) else {
                pos += remaining;
                break;
            };
            pos += remaining.scale(t);
            remaining = remaining.scale(Fx::ONE - t);
            let mut n = n;
            // Walls (non-walkable) block horizontally only while grounded, so
            // steep slopes cannot be climbed and we are not pushed into the floor.
            if grounded && n.y < desc.max_slope_cos && n.y > fx!(-0.5) {
                let h = Vec3::new(n.x, Fx::ZERO, n.z).normalize_or_zero();
                if !h.is_zero() {
                    n = h;
                }
            }
            res.add_hit(n);
            let rn = remaining.dot(n);
            if rn < Fx::ZERO {
                remaining -= n.scale(rn);
            }
            // Crease between two planes: slide along their intersection.
            for p in planes.iter().take(np) {
                if remaining.dot(*p) < Fx::ZERO {
                    let dir = p.cross(n).normalize_or_zero();
                    remaining = dir.scale(remaining.dot(dir));
                    break;
                }
            }
            if np < 4 {
                planes[np] = n;
                np += 1;
            }
        }
        res.position = pos;
        res
    }

    /// Is a sweep hit with normal `n` at `contact` standable? Rounded capsule
    /// bottoms report tilted normals on convex edges (e.g. stair noses), so
    /// for those the real surface just past the edge is probed with a ray.
    fn walkable(&self, filter: &QueryFilter, contact: Vec3, n: Vec3, slope_cos: Fx) -> Option<Vec3> {
        if n.y >= slope_cos {
            return Some(n);
        }
        if n.y <= fx!(0.05) {
            return None;
        }
        let inward = Vec3::new(-n.x, Fx::ZERO, -n.z).normalize_or_zero().scale(fx!(0.02));
        let origin = contact + inward + Vec3::new(Fx::ZERO, fx!(0.05), Fx::ZERO);
        let hit = self.raycast(origin, Vec3::DOWN, fx!(0.1), filter)?;
        if hit.normal.y >= slope_cos {
            Some(hit.normal)
        } else {
            None
        }
    }

    fn try_step(
        &self,
        obs: &[ObRef],
        cap: &Capsule,
        pos: Vec3,
        hd: Vec3,
        desc: &CharacterDesc,
        filter: &QueryFilter,
    ) -> Option<MoveResult> {
        // Up.
        let up = Vec3::new(Fx::ZERO, desc.step_height, Fx::ZERO);
        let (a0, a1) = cap.seg(pos);
        let t_up = self.sweep_rounded(obs, a0, a1, cap.r, up, desc.skin, NO_SKIP).map_or(Fx::ONE, |h| h.0);
        let raised = pos + up.scale(t_up);
        // Across.
        let mut res = self.slide(obs, cap, raised, hd, true, desc, NO_SKIP);
        // Down.
        let down = Vec3::new(Fx::ZERO, -(desc.step_height * t_up + desc.skin * Fx::from_int(2)), Fx::ZERO);
        let (b0, b1) = cap.seg(res.position);
        let (t, n, _, p) = self.sweep_rounded(obs, b0, b1, cap.r, down, desc.skin, NO_SKIP)?;
        self.walkable(filter, p, n, desc.max_slope_cos)?;
        res.position += down.scale(t);
        Some(res)
    }
}

/// World-space wish vector from input (length <= 1).
#[inline]
fn wish_vector(input: &CharacterInput, fly: bool) -> Vec3 {
    let (sy, cy) = input.yaw.sin_cos();
    let forward = Vec3::new(sy, Fx::ZERO, cy);
    let right = Vec3::new(cy, Fx::ZERO, -sy);
    let mut wish = right.scale(input.move_x.clamp(Fx::NEG_ONE, Fx::ONE)) + forward.scale(input.move_z.clamp(Fx::NEG_ONE, Fx::ONE));
    if fly {
        wish.y = input.move_y.clamp(Fx::NEG_ONE, Fx::ONE);
    }
    let wl = wish.length();
    if wl > Fx::ONE {
        wish = wish.scale(Fx::ONE / wl);
    }
    wish
}

#[inline]
fn top_speed(mv: &MovementSettings, input: &CharacterInput) -> Fx {
    top_speed_crouch(mv, input, input.crouch)
}

#[inline]
fn top_speed_crouch(mv: &MovementSettings, input: &CharacterInput, crouched: bool) -> Fx {
    if crouched {
        mv.crouch_speed
    } else if input.sprint {
        mv.sprint_speed
    } else {
        mv.walk_speed
    }
}

/// Ground friction (horizontal only unless `all_axes`).
#[inline]
fn friction(v: &mut Vec3, mv: &MovementSettings, dt: Fx, all_axes: bool) {
    let hv = if all_axes { *v } else { Vec3::new(v.x, Fx::ZERO, v.z) };
    let speed = hv.length();
    if speed.0 <= 0 {
        return;
    }
    let drop = speed.max(mv.stop_speed) * mv.friction * dt;
    let k = (speed - drop).max(Fx::ZERO) / speed;
    v.x *= k;
    v.z *= k;
    if all_axes {
        v.y *= k;
    }
}

#[inline]
fn horiz_len(v: Vec3) -> Fx {
    Vec3::new(v.x, Fx::ZERO, v.z).length()
}

#[inline]
fn accelerate(v: &mut Vec3, dir: Vec3, wish_speed: Fx, accel: Fx, dt: Fx) {
    if dir.is_zero() {
        return;
    }
    let current = v.dot(dir);
    let add = wish_speed - current;
    if add <= Fx::ZERO {
        return;
    }
    let a = (accel * dt * wish_speed).min(add);
    *v += dir.scale(a);
}
