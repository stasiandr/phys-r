//! C ABI for phys-r. Consumed by the Unity package via P/Invoke (and usable
//! from any language with a C FFI). See `include/phys_r.h`.
//!
//! Conventions:
//! * Fixed-point values are raw `int64_t` Q40.24 (`1.0 == 1 << 24`).
//! * Handles are `uint64_t`; `0` is the invalid handle.
//! * Functions never unwind: invalid handles / null pointers return 0/false.
//! * A world must only be used from one thread at a time.

#![allow(clippy::missing_safety_doc)]

use phys_r::body::{BodyDesc, BodyHandle, BodyKind};
use phys_r::character::{CharacterDesc, CharacterHandle, CharacterInput, MoveMode, MovementSettings};
use phys_r::door::{DoorDesc, DoorHandle, DoorKind};
use phys_r::query::QueryFilter;
use phys_r::{Fx, Quat, Shape, ShapeId, Vec3, World, WorldSettings};

pub const PR_VERSION: u32 = 1;
pub const PR_INVALID_SHAPE: u32 = u32::MAX;

// ---------------------------------------------------------------------------
// Plain data types (layouts mirrored in phys_r.h and PhysRNative.cs)
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrVec3 {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrQuat {
    pub x: i64,
    pub y: i64,
    pub z: i64,
    pub w: i64,
}

impl From<PrVec3> for Vec3 {
    #[inline]
    fn from(v: PrVec3) -> Vec3 {
        Vec3::new(Fx(v.x), Fx(v.y), Fx(v.z))
    }
}
impl From<Vec3> for PrVec3 {
    #[inline]
    fn from(v: Vec3) -> PrVec3 {
        PrVec3 { x: v.x.0, y: v.y.0, z: v.z.0 }
    }
}
impl From<PrQuat> for Quat {
    #[inline]
    fn from(q: PrQuat) -> Quat {
        let q = Quat::new(Fx(q.x), Fx(q.y), Fx(q.z), Fx(q.w));
        if q.x.0 == 0 && q.y.0 == 0 && q.z.0 == 0 && q.w.0 == 0 {
            Quat::IDENTITY
        } else {
            q
        }
    }
}
impl From<Quat> for PrQuat {
    #[inline]
    fn from(q: Quat) -> PrQuat {
        PrQuat { x: q.x.0, y: q.y.0, z: q.z.0, w: q.w.0 }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrWorldSettings {
    pub gravity: PrVec3,
    pub dt: i64,
    pub contact_margin: i64,
    pub slop: i64,
    pub baumgarte: i64,
    pub max_correction_velocity: i64,
    pub restitution_threshold: i64,
    pub sleep_linear_threshold: i64,
    pub sleep_angular_threshold: i64,
    pub time_to_sleep: i64,
    pub velocity_iterations: u32,
    pub position_iterations: u32,
    pub history_ticks: u32,
    pub enable_sleep: u32,
}

impl From<&WorldSettings> for PrWorldSettings {
    fn from(s: &WorldSettings) -> Self {
        PrWorldSettings {
            gravity: s.gravity.into(),
            dt: s.dt.0,
            contact_margin: s.contact_margin.0,
            slop: s.slop.0,
            baumgarte: s.baumgarte.0,
            max_correction_velocity: s.max_correction_velocity.0,
            restitution_threshold: s.restitution_threshold.0,
            sleep_linear_threshold: s.sleep_linear_threshold.0,
            sleep_angular_threshold: s.sleep_angular_threshold.0,
            time_to_sleep: s.time_to_sleep.0,
            velocity_iterations: s.velocity_iterations,
            position_iterations: s.position_iterations,
            history_ticks: s.history_ticks,
            enable_sleep: s.enable_sleep as u32,
        }
    }
}

impl From<&PrWorldSettings> for WorldSettings {
    fn from(s: &PrWorldSettings) -> Self {
        WorldSettings {
            gravity: s.gravity.into(),
            dt: if s.dt > 0 { Fx(s.dt) } else { Fx::from_ratio(1, 60) },
            velocity_iterations: s.velocity_iterations.clamp(1, 64),
            position_iterations: s.position_iterations.min(64),
            contact_margin: Fx(s.contact_margin),
            slop: Fx(s.slop),
            baumgarte: Fx(s.baumgarte),
            max_correction_velocity: Fx(s.max_correction_velocity),
            restitution_threshold: Fx(s.restitution_threshold),
            sleep_linear_threshold: Fx(s.sleep_linear_threshold),
            sleep_angular_threshold: Fx(s.sleep_angular_threshold),
            time_to_sleep: Fx(s.time_to_sleep),
            enable_sleep: s.enable_sleep != 0,
            history_ticks: s.history_ticks.min(4096),
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrBodyDesc {
    /// 0 = static, 1 = dynamic, 2 = kinematic.
    pub kind: u32,
    pub shape: u32,
    pub position: PrVec3,
    pub rotation: PrQuat,
    pub linear_velocity: PrVec3,
    pub angular_velocity: PrVec3,
    pub mass: i64,
    pub friction: i64,
    pub restitution: i64,
    pub linear_damping: i64,
    pub angular_damping: i64,
    pub gravity_scale: i64,
    pub layer: u32,
    pub mask: u32,
    pub flags: u32,
    pub _pad: u32,
    pub user_data: u64,
}

impl From<&BodyDesc> for PrBodyDesc {
    fn from(d: &BodyDesc) -> Self {
        PrBodyDesc {
            kind: d.kind as u32,
            shape: d.shape.0,
            position: d.position.into(),
            rotation: d.rotation.into(),
            linear_velocity: d.linear_velocity.into(),
            angular_velocity: d.angular_velocity.into(),
            mass: d.mass.0,
            friction: d.friction.0,
            restitution: d.restitution.0,
            linear_damping: d.linear_damping.0,
            angular_damping: d.angular_damping.0,
            gravity_scale: d.gravity_scale.0,
            layer: d.layer as u32,
            mask: d.mask,
            flags: d.flags,
            _pad: 0,
            user_data: d.user_data,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrBodyState {
    pub position: PrVec3,
    pub rotation: PrQuat,
    pub linear_velocity: PrVec3,
    pub angular_velocity: PrVec3,
    pub sleeping: u32,
    pub kind: u32,
    pub user_data: u64,
}

/// Float transform for rendering (never feed back into the simulation).
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrTransformF {
    pub px: f32,
    pub py: f32,
    pub pz: f32,
    pub rx: f32,
    pub ry: f32,
    pub rz: f32,
    pub rw: f32,
    /// 1 if the handle was valid.
    pub valid: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrQueryFilter {
    pub mask: u32,
    /// bit 0: include sensors, bit 1: exclude static, bit 2: exclude moving.
    pub flags: u32,
    pub exclude: u64,
}

impl From<&PrQueryFilter> for QueryFilter {
    fn from(f: &PrQueryFilter) -> Self {
        QueryFilter {
            mask: f.mask,
            exclude: BodyHandle::from_bits(f.exclude),
            include_sensors: f.flags & 1 != 0,
            include_static: f.flags & 2 == 0,
            include_moving: f.flags & 4 == 0,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrRayHit {
    pub body: u64,
    pub distance: i64,
    pub point: PrVec3,
    pub normal: PrVec3,
    /// Hit point in the body's local space (hit zones / headshots).
    pub local_point: PrVec3,
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrSweepHit {
    pub body: u64,
    /// Fraction of the displacement (0..1, raw Q40.24).
    pub fraction: i64,
    pub normal: PrVec3,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrMovementSettings {
    pub walk_speed: i64,
    pub sprint_speed: i64,
    pub crouch_speed: i64,
    pub ground_accel: i64,
    pub air_accel: i64,
    pub air_speed_cap: i64,
    pub friction: i64,
    pub stop_speed: i64,
    pub gravity: i64,
    pub jump_speed: i64,
    pub max_fall_speed: i64,
    pub jump_requires_release: u32,
    pub coyote_ticks: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrCharacterDesc {
    pub position: PrVec3,
    pub radius: i64,
    pub height: i64,
    pub crouch_height: i64,
    pub step_height: i64,
    pub max_slope_cos: i64,
    pub skin: i64,
    pub snap_distance: i64,
    pub mass: i64,
    pub push_strength: i64,
    pub layer: u32,
    pub mask: u32,
    pub body_flags: u32,
    pub _pad: u32,
    pub user_data: u64,
    pub movement: PrMovementSettings,
}

impl From<&CharacterDesc> for PrCharacterDesc {
    fn from(d: &CharacterDesc) -> Self {
        let m = &d.movement;
        PrCharacterDesc {
            position: d.position.into(),
            radius: d.radius.0,
            height: d.height.0,
            crouch_height: d.crouch_height.0,
            step_height: d.step_height.0,
            max_slope_cos: d.max_slope_cos.0,
            skin: d.skin.0,
            snap_distance: d.snap_distance.0,
            mass: d.mass.0,
            push_strength: d.push_strength.0,
            layer: d.layer as u32,
            mask: d.mask,
            body_flags: d.body_flags,
            _pad: 0,
            user_data: d.user_data,
            movement: PrMovementSettings {
                walk_speed: m.walk_speed.0,
                sprint_speed: m.sprint_speed.0,
                crouch_speed: m.crouch_speed.0,
                ground_accel: m.ground_accel.0,
                air_accel: m.air_accel.0,
                air_speed_cap: m.air_speed_cap.0,
                friction: m.friction.0,
                stop_speed: m.stop_speed.0,
                gravity: m.gravity.0,
                jump_speed: m.jump_speed.0,
                max_fall_speed: m.max_fall_speed.0,
                jump_requires_release: m.jump_requires_release as u32,
                coyote_ticks: m.coyote_ticks,
            },
        }
    }
}

impl From<&PrCharacterDesc> for CharacterDesc {
    fn from(d: &PrCharacterDesc) -> Self {
        let m = &d.movement;
        CharacterDesc {
            position: d.position.into(),
            radius: Fx(d.radius),
            height: Fx(d.height),
            crouch_height: Fx(d.crouch_height),
            step_height: Fx(d.step_height),
            max_slope_cos: Fx(d.max_slope_cos),
            skin: Fx(d.skin),
            snap_distance: Fx(d.snap_distance),
            mass: Fx(d.mass),
            push_strength: Fx(d.push_strength),
            layer: (d.layer & 31) as u8,
            mask: d.mask,
            body_flags: d.body_flags,
            user_data: d.user_data,
            movement: MovementSettings {
                walk_speed: Fx(m.walk_speed),
                sprint_speed: Fx(m.sprint_speed),
                crouch_speed: Fx(m.crouch_speed),
                ground_accel: Fx(m.ground_accel),
                air_accel: Fx(m.air_accel),
                air_speed_cap: Fx(m.air_speed_cap),
                friction: Fx(m.friction),
                stop_speed: Fx(m.stop_speed),
                gravity: Fx(m.gravity),
                jump_speed: Fx(m.jump_speed),
                max_fall_speed: Fx(m.max_fall_speed),
                jump_requires_release: m.jump_requires_release != 0,
                coyote_ticks: m.coyote_ticks,
            },
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrCharacterInput {
    pub move_x: i64,
    /// Vertical axis for fly mode (ladders / swimming).
    pub move_y: i64,
    pub move_z: i64,
    pub yaw: i64,
    /// bit 0: jump, bit 1: sprint, bit 2: crouch.
    pub buttons: u32,
    pub _pad: u32,
}

impl From<&PrCharacterInput> for CharacterInput {
    fn from(i: &PrCharacterInput) -> Self {
        CharacterInput {
            move_x: Fx(i.move_x),
            move_y: Fx(i.move_y),
            move_z: Fx(i.move_z),
            yaw: Fx(i.yaw),
            jump: i.buttons & 1 != 0,
            sprint: i.buttons & 2 != 0,
            crouch: i.buttons & 4 != 0,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrCharacterState {
    /// Capsule centre.
    pub position: PrVec3,
    pub velocity: PrVec3,
    pub ground_normal: PrVec3,
    /// Velocity of the moving platform under the feet.
    pub ground_velocity: PrVec3,
    pub body: u64,
    /// Body stood on (0 when airborne).
    pub ground_body: u64,
    /// Downward speed on the landing tick (fall damage), else 0.
    pub landing_speed: i64,
    /// Current capsule height (crouch aware).
    pub height: i64,
    pub grounded: u32,
    pub crouched: u32,
    /// 0 walk, 1 fly, 2 noclip.
    pub mode: u32,
    /// PR_COLLIDED_* flags from the last tick.
    pub collision_flags: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrEvent {
    /// 0 contact begin, 1 contact end, 2 sensor enter, 3 sensor exit.
    pub kind: u32,
    pub _pad: u32,
    pub a: u64,
    pub b: u64,
    pub point: PrVec3,
    pub normal: PrVec3,
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#[inline]
unsafe fn world<'a>(w: *mut World) -> Option<&'a mut World> {
    w.as_mut()
}

#[inline]
unsafe fn read<T: Copy>(p: *const T) -> Option<T> {
    if p.is_null() {
        None
    } else {
        Some(*p)
    }
}

#[inline]
unsafe fn write<T>(p: *mut T, v: T) {
    if !p.is_null() {
        *p = v;
    }
}

#[inline]
fn b(v: bool) -> u32 {
    v as u32
}

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

#[no_mangle]
pub extern "C" fn pr_version() -> u32 {
    PR_VERSION
}

#[no_mangle]
pub unsafe extern "C" fn pr_world_settings_default(out: *mut PrWorldSettings) {
    write(out, PrWorldSettings::from(&WorldSettings::default()));
}

/// Create a world. `settings` may be null for defaults.
#[no_mangle]
pub unsafe extern "C" fn pr_world_create(settings: *const PrWorldSettings) -> *mut World {
    let s = read(settings).map(|s| WorldSettings::from(&s)).unwrap_or_default();
    Box::into_raw(Box::new(World::new(s)))
}

#[no_mangle]
pub unsafe extern "C" fn pr_world_destroy(w: *mut World) {
    if !w.is_null() {
        drop(Box::from_raw(w));
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_world_step(w: *mut World, steps: u32) {
    if let Some(w) = world(w) {
        for _ in 0..steps.max(1) {
            w.step();
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_world_tick(w: *mut World) -> u32 {
    world(w).map_or(0, |w| w.tick())
}

#[no_mangle]
pub unsafe extern "C" fn pr_world_set_gravity(w: *mut World, g: *const PrVec3) {
    if let (Some(w), Some(g)) = (world(w), read(g)) {
        w.settings.gravity = g.into();
    }
}

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn pr_shape_sphere(w: *mut World, radius: i64) -> u32 {
    world(w).map_or(PR_INVALID_SHAPE, |w| w.add_shape(Shape::sphere(Fx(radius))).0)
}

/// Capsule along local Y; `half_height` is half the inner segment length.
#[no_mangle]
pub unsafe extern "C" fn pr_shape_capsule(w: *mut World, half_height: i64, radius: i64) -> u32 {
    world(w).map_or(PR_INVALID_SHAPE, |w| w.add_shape(Shape::capsule(Fx(half_height), Fx(radius))).0)
}

#[no_mangle]
pub unsafe extern "C" fn pr_shape_box(w: *mut World, half_extents: *const PrVec3) -> u32 {
    match (world(w), read(half_extents)) {
        (Some(w), Some(h)) => w.add_shape(Shape::cuboid(h.into())).0,
        _ => PR_INVALID_SHAPE,
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_shape_count(w: *mut World) -> u32 {
    world(w).map_or(0, |w| w.shape_count() as u32)
}

// ---------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn pr_body_desc_default(out: *mut PrBodyDesc) {
    write(out, PrBodyDesc::from(&BodyDesc::default()));
}

/// Returns the new body handle, or 0 on failure (bad shape, mesh on a
/// dynamic body).
#[no_mangle]
pub unsafe extern "C" fn pr_body_create(w: *mut World, desc: *const PrBodyDesc) -> u64 {
    let (Some(w), Some(d)) = (world(w), read(desc)) else { return 0 };
    let bd = BodyDesc {
        kind: BodyKind::from_u8(d.kind as u8),
        shape: ShapeId(d.shape),
        position: d.position.into(),
        rotation: d.rotation.into(),
        linear_velocity: d.linear_velocity.into(),
        angular_velocity: d.angular_velocity.into(),
        mass: Fx(d.mass),
        friction: Fx(d.friction),
        restitution: Fx(d.restitution),
        linear_damping: Fx(d.linear_damping),
        angular_damping: Fx(d.angular_damping),
        gravity_scale: Fx(d.gravity_scale),
        layer: (d.layer & 31) as u8,
        mask: d.mask,
        flags: d.flags,
        user_data: d.user_data,
    };
    w.create_body(&bd).map_or(0, |h| h.to_bits())
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_destroy(w: *mut World, h: u64) -> u32 {
    world(w).map_or(0, |w| b(w.destroy_body(BodyHandle::from_bits(h))))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_is_valid(w: *mut World, h: u64) -> u32 {
    world(w).map_or(0, |w| b(w.body(BodyHandle::from_bits(h)).is_some()))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_get_state(w: *mut World, h: u64, out: *mut PrBodyState) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let Some(bd) = w.body(BodyHandle::from_bits(h)) else { return 0 };
    write(
        out,
        PrBodyState {
            position: bd.position.into(),
            rotation: bd.rotation.into(),
            linear_velocity: bd.linear_velocity.into(),
            angular_velocity: bd.angular_velocity.into(),
            sleeping: b(bd.is_sleeping()),
            kind: bd.kind as u32,
            user_data: bd.user_data,
        },
    );
    1
}

/// Batch read of float transforms for rendering. Returns the number of
/// valid handles.
#[no_mangle]
pub unsafe extern "C" fn pr_body_get_transforms_f32(w: *mut World, handles: *const u64, count: u32, out: *mut PrTransformF) -> u32 {
    let Some(w) = world(w) else { return 0 };
    if handles.is_null() || out.is_null() {
        return 0;
    }
    let hs = core::slice::from_raw_parts(handles, count as usize);
    let os = core::slice::from_raw_parts_mut(out, count as usize);
    let mut valid = 0;
    for (h, o) in hs.iter().zip(os.iter_mut()) {
        *o = match w.body(BodyHandle::from_bits(*h)) {
            Some(bd) => {
                valid += 1;
                PrTransformF {
                    px: bd.position.x.to_f32(),
                    py: bd.position.y.to_f32(),
                    pz: bd.position.z.to_f32(),
                    rx: bd.rotation.x.to_f32(),
                    ry: bd.rotation.y.to_f32(),
                    rz: bd.rotation.z.to_f32(),
                    rw: bd.rotation.w.to_f32(),
                    valid: 1,
                }
            }
            None => PrTransformF::default(),
        };
    }
    valid
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_set_pose(w: *mut World, h: u64, position: *const PrVec3, rotation: *const PrQuat) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let h = BodyHandle::from_bits(h);
    let Some(bd) = w.body(h) else { return 0 };
    let p = read(position).map_or(bd.position, |p| p.into());
    let r = read(rotation).map_or(bd.rotation, |r| r.into());
    b(w.set_pose(h, p, r))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_set_velocity(w: *mut World, h: u64, linear: *const PrVec3, angular: *const PrVec3) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let h = BodyHandle::from_bits(h);
    let Some(bd) = w.body(h) else { return 0 };
    let l = read(linear).map_or(bd.linear_velocity, |v| v.into());
    let a = read(angular).map_or(bd.angular_velocity, |v| v.into());
    b(w.set_velocity(h, l, a))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_set_kinematic_target(w: *mut World, h: u64, position: *const PrVec3, rotation: *const PrQuat) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let h = BodyHandle::from_bits(h);
    let Some(bd) = w.body(h) else { return 0 };
    let p = read(position).map_or(bd.position, |p| p.into());
    let r = read(rotation).map_or(bd.rotation, |r| r.into());
    b(w.set_kinematic_target(h, p, r))
}

/// `point` may be null to apply at the centre of mass.
#[no_mangle]
pub unsafe extern "C" fn pr_body_apply_impulse(w: *mut World, h: u64, impulse: *const PrVec3, point: *const PrVec3) -> u32 {
    let (Some(w), Some(i)) = (world(w), read(impulse)) else { return 0 };
    b(w.apply_impulse(BodyHandle::from_bits(h), i.into(), read(point).map(Into::into)))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_apply_force(w: *mut World, h: u64, force: *const PrVec3, point: *const PrVec3) -> u32 {
    let (Some(w), Some(f)) = (world(w), read(force)) else { return 0 };
    b(w.apply_force(BodyHandle::from_bits(h), f.into(), read(point).map(Into::into)))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_apply_torque(w: *mut World, h: u64, torque: *const PrVec3) -> u32 {
    let (Some(w), Some(t)) = (world(w), read(torque)) else { return 0 };
    b(w.apply_torque(BodyHandle::from_bits(h), t.into()))
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_wake(w: *mut World, h: u64) {
    if let Some(w) = world(w) {
        w.wake(BodyHandle::from_bits(h));
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_set_filter(w: *mut World, h: u64, layer: u32, mask: u32) -> u32 {
    let Some(w) = world(w) else { return 0 };
    match w.body_mut(BodyHandle::from_bits(h)) {
        Some(bd) => {
            bd.layer = (layer & 31) as u8;
            bd.mask = mask;
            1
        }
        None => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_set_material(w: *mut World, h: u64, friction: i64, restitution: i64) -> u32 {
    let Some(w) = world(w) else { return 0 };
    match w.body_mut(BodyHandle::from_bits(h)) {
        Some(bd) => {
            bd.friction = Fx(friction);
            bd.restitution = Fx(restitution);
            1
        }
        None => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_body_set_flags(w: *mut World, h: u64, flags: u32) -> u32 {
    world(w).map_or(0, |w| b(w.set_body_flags(BodyHandle::from_bits(h), flags)))
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn pr_raycast(
    w: *mut World,
    origin: *const PrVec3,
    dir: *const PrVec3,
    max_dist: i64,
    filter: *const PrQueryFilter,
    out: *mut PrRayHit,
) -> u32 {
    let (Some(w), Some(o), Some(d)) = (world(w), read(origin), read(dir)) else { return 0 };
    let f = read(filter).map(|f| QueryFilter::from(&f)).unwrap_or_default();
    match w.raycast(o.into(), d.into(), Fx(max_dist), &f) {
        Some(h) => {
            write(
                out,
                PrRayHit {
                    body: h.body.to_bits(),
                    distance: h.distance.0,
                    point: h.point.into(),
                    normal: h.normal.into(),
                    local_point: h.local_point.into(),
                },
            );
            1
        }
        None => 0,
    }
}

/// Lag-compensated raycast against poses recorded at `tick + alpha`.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn pr_raycast_rewind(
    w: *mut World,
    tick: u32,
    alpha: i64,
    origin: *const PrVec3,
    dir: *const PrVec3,
    max_dist: i64,
    filter: *const PrQueryFilter,
    out: *mut PrRayHit,
) -> u32 {
    let (Some(w), Some(o), Some(d)) = (world(w), read(origin), read(dir)) else { return 0 };
    let f = read(filter).map(|f| QueryFilter::from(&f)).unwrap_or_default();
    match w.raycast_rewind(tick, Fx(alpha), o.into(), d.into(), Fx(max_dist), &f) {
        Some(h) => {
            write(
                out,
                PrRayHit {
                    body: h.body.to_bits(),
                    distance: h.distance.0,
                    point: h.point.into(),
                    normal: h.normal.into(),
                    local_point: h.local_point.into(),
                },
            );
            1
        }
        None => 0,
    }
}

/// Writes up to `capacity` handles; returns the total number found.
#[no_mangle]
pub unsafe extern "C" fn pr_overlap_sphere(
    w: *mut World,
    center: *const PrVec3,
    radius: i64,
    filter: *const PrQueryFilter,
    out: *mut u64,
    capacity: u32,
) -> u32 {
    let (Some(w), Some(c)) = (world(w), read(center)) else { return 0 };
    let f = read(filter).map(|f| QueryFilter::from(&f)).unwrap_or_default();
    let mut found = Vec::new();
    w.overlap_sphere(c.into(), Fx(radius), &f, &mut found);
    if !out.is_null() {
        let os = core::slice::from_raw_parts_mut(out, capacity as usize);
        for (o, h) in os.iter_mut().zip(found.iter()) {
            *o = h.to_bits();
        }
    }
    found.len() as u32
}

#[no_mangle]
pub unsafe extern "C" fn pr_sphere_cast(
    w: *mut World,
    center: *const PrVec3,
    radius: i64,
    disp: *const PrVec3,
    filter: *const PrQueryFilter,
    out: *mut PrSweepHit,
) -> u32 {
    let (Some(w), Some(c), Some(d)) = (world(w), read(center), read(disp)) else { return 0 };
    let f = read(filter).map(|f| QueryFilter::from(&f)).unwrap_or_default();
    match w.sphere_cast(c.into(), Fx(radius), d.into(), &f) {
        Some(h) => {
            write(out, PrSweepHit { body: h.body.to_bits(), fraction: h.fraction.0, normal: h.normal.into() });
            1
        }
        None => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_capsule_cast(
    w: *mut World,
    center: *const PrVec3,
    half_height: i64,
    radius: i64,
    disp: *const PrVec3,
    filter: *const PrQueryFilter,
    out: *mut PrSweepHit,
) -> u32 {
    let (Some(w), Some(c), Some(d)) = (world(w), read(center), read(disp)) else { return 0 };
    let f = read(filter).map(|f| QueryFilter::from(&f)).unwrap_or_default();
    match w.capsule_cast(c.into(), Fx(half_height), Fx(radius), d.into(), &f) {
        Some(h) => {
            write(out, PrSweepHit { body: h.body.to_bits(), fraction: h.fraction.0, normal: h.normal.into() });
            1
        }
        None => 0,
    }
}

// ---------------------------------------------------------------------------
// Characters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn pr_character_desc_default(out: *mut PrCharacterDesc) {
    write(out, PrCharacterDesc::from(&CharacterDesc::default()));
}

#[no_mangle]
pub unsafe extern "C" fn pr_character_create(w: *mut World, desc: *const PrCharacterDesc) -> u64 {
    let (Some(w), Some(d)) = (world(w), read(desc)) else { return 0 };
    w.create_character(&CharacterDesc::from(&d)).map_or(0, |h| h.to_bits())
}

#[no_mangle]
pub unsafe extern "C" fn pr_character_destroy(w: *mut World, h: u64) -> u32 {
    world(w).map_or(0, |w| b(w.destroy_character(CharacterHandle::from_bits(h))))
}

/// Input consumed by the next `pr_world_step`.
#[no_mangle]
pub unsafe extern "C" fn pr_character_set_input(w: *mut World, h: u64, input: *const PrCharacterInput) -> u32 {
    let (Some(w), Some(i)) = (world(w), read(input)) else { return 0 };
    b(w.set_character_input(CharacterHandle::from_bits(h), CharacterInput::from(&i)))
}

/// Advance only this character by `dt` (client-side prediction / replay).
#[no_mangle]
pub unsafe extern "C" fn pr_character_simulate(w: *mut World, h: u64, input: *const PrCharacterInput, dt: i64) -> u32 {
    let (Some(w), Some(i)) = (world(w), read(input)) else { return 0 };
    let dt = if dt > 0 { Fx(dt) } else { w.settings.dt };
    b(w.simulate_character(CharacterHandle::from_bits(h), CharacterInput::from(&i), dt))
}

#[no_mangle]
pub unsafe extern "C" fn pr_character_get_state(w: *mut World, h: u64, out: *mut PrCharacterState) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let Some(c) = w.character(CharacterHandle::from_bits(h)) else { return 0 };
    write(
        out,
        PrCharacterState {
            position: c.position.into(),
            velocity: c.velocity.into(),
            ground_normal: c.ground_normal.into(),
            ground_velocity: c.ground_velocity.into(),
            body: c.body().to_bits(),
            ground_body: c.ground_body.to_bits(),
            landing_speed: c.landing_speed.0,
            height: c.height().0,
            grounded: b(c.grounded),
            crouched: b(c.crouched),
            mode: c.mode as u32,
            collision_flags: c.collision_flags,
        },
    );
    1
}

#[no_mangle]
pub unsafe extern "C" fn pr_character_teleport(w: *mut World, h: u64, position: *const PrVec3) -> u32 {
    let (Some(w), Some(p)) = (world(w), read(position)) else { return 0 };
    b(w.teleport_character(CharacterHandle::from_bits(h), p.into()))
}

#[no_mangle]
pub unsafe extern "C" fn pr_character_set_velocity(w: *mut World, h: u64, velocity: *const PrVec3) -> u32 {
    let (Some(w), Some(v)) = (world(w), read(velocity)) else { return 0 };
    b(w.set_character_velocity(CharacterHandle::from_bits(h), v.into()))
}

/// Knockback / explosions / jump pads (an upward kick un-grounds).
#[no_mangle]
pub unsafe extern "C" fn pr_character_add_velocity(w: *mut World, h: u64, dv: *const PrVec3) -> u32 {
    let (Some(w), Some(v)) = (world(w), read(dv)) else { return 0 };
    b(w.add_character_velocity(CharacterHandle::from_bits(h), v.into()))
}

/// 0 walk, 1 fly (ladders / swimming), 2 noclip.
#[no_mangle]
pub unsafe extern "C" fn pr_character_set_mode(w: *mut World, h: u64, mode: u32) -> u32 {
    let Some(w) = world(w) else { return 0 };
    b(w.set_character_mode(CharacterHandle::from_bits(h), MoveMode::from_u8(mode.min(255) as u8)))
}

/// Character owning `body` (e.g. from a raycast hit), or 0.
#[no_mangle]
pub unsafe extern "C" fn pr_character_of_body(w: *mut World, body: u64) -> u64 {
    world(w).and_then(|w| w.character_of_body(BodyHandle::from_bits(body))).map_or(0, |h| h.to_bits())
}

// ---------------------------------------------------------------------------
// Doors
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Copy, Clone)]
pub struct PrDoorDesc {
    pub shape: u32,
    /// 0 hinge (amount in radians), 1 slide (amount in metres).
    pub kind: u32,
    /// Hinge: point on the axis. Slide: closed body centre.
    pub pivot: PrVec3,
    pub axis: PrVec3,
    pub closed_rotation: PrQuat,
    /// Hinge: closed body centre relative to pivot.
    pub offset: PrVec3,
    pub open_amount: i64,
    pub speed: i64,
    pub friction: i64,
    pub layer: u32,
    pub mask: u32,
    /// Stop instead of pushing characters.
    pub block_on_characters: u32,
    pub _pad: u32,
    pub user_data: u64,
}

impl From<&DoorDesc> for PrDoorDesc {
    fn from(d: &DoorDesc) -> Self {
        PrDoorDesc {
            shape: d.shape.0,
            kind: d.kind as u32,
            pivot: d.pivot.into(),
            axis: d.axis.into(),
            closed_rotation: d.closed_rotation.into(),
            offset: d.offset.into(),
            open_amount: d.open_amount.0,
            speed: d.speed.0,
            friction: d.friction.0,
            layer: d.layer as u32,
            mask: d.mask,
            block_on_characters: b(d.block_on_characters),
            _pad: 0,
            user_data: d.user_data,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct PrDoorState {
    pub body: u64,
    /// Current / target opening (radians or metres).
    pub amount: i64,
    pub target: i64,
    /// 0 = closed .. 1 = open.
    pub open_fraction: i64,
    pub blocked: u32,
    pub moving: u32,
}

#[no_mangle]
pub unsafe extern "C" fn pr_door_desc_default(out: *mut PrDoorDesc) {
    write(out, PrDoorDesc::from(&DoorDesc::default()));
}

#[no_mangle]
pub unsafe extern "C" fn pr_door_create(w: *mut World, desc: *const PrDoorDesc) -> u64 {
    let (Some(w), Some(d)) = (world(w), read(desc)) else { return 0 };
    let dd = DoorDesc {
        shape: ShapeId(d.shape),
        kind: DoorKind::from_u8(d.kind.min(255) as u8),
        pivot: d.pivot.into(),
        axis: d.axis.into(),
        closed_rotation: d.closed_rotation.into(),
        offset: d.offset.into(),
        open_amount: Fx(d.open_amount),
        speed: Fx(d.speed),
        friction: Fx(d.friction),
        layer: (d.layer & 31) as u8,
        mask: d.mask,
        block_on_characters: d.block_on_characters != 0,
        user_data: d.user_data,
    };
    w.create_door(&dd).map_or(0, |h| h.to_bits())
}

#[no_mangle]
pub unsafe extern "C" fn pr_door_destroy(w: *mut World, h: u64) -> u32 {
    world(w).map_or(0, |w| b(w.destroy_door(DoorHandle::from_bits(h))))
}

/// Start opening (open != 0) or closing.
#[no_mangle]
pub unsafe extern "C" fn pr_door_set_open(w: *mut World, h: u64, open: u32) -> u32 {
    world(w).map_or(0, |w| b(w.set_door_open(DoorHandle::from_bits(h), open != 0)))
}

/// Move toward a partial opening (0 = closed .. 1 = open).
#[no_mangle]
pub unsafe extern "C" fn pr_door_set_target(w: *mut World, h: u64, fraction: i64) -> u32 {
    world(w).map_or(0, |w| b(w.set_door_target(DoorHandle::from_bits(h), Fx(fraction))))
}

/// Snap instantly to an opening (0..1).
#[no_mangle]
pub unsafe extern "C" fn pr_door_set_amount(w: *mut World, h: u64, fraction: i64) -> u32 {
    world(w).map_or(0, |w| b(w.set_door_amount(DoorHandle::from_bits(h), Fx(fraction))))
}

#[no_mangle]
pub unsafe extern "C" fn pr_door_get_state(w: *mut World, h: u64, out: *mut PrDoorState) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let Some(d) = w.door(DoorHandle::from_bits(h)) else { return 0 };
    write(
        out,
        PrDoorState {
            body: d.body().to_bits(),
            amount: d.amount.0,
            target: d.target.0,
            open_fraction: d.open_fraction().0,
            blocked: b(d.blocked),
            moving: b(d.is_moving()),
        },
    );
    1
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn pr_world_event_count(w: *mut World) -> u32 {
    world(w).map_or(0, |w| w.events().len() as u32)
}

/// Copies up to `capacity` events from the last step; returns the total.
#[no_mangle]
pub unsafe extern "C" fn pr_world_get_events(w: *mut World, out: *mut PrEvent, capacity: u32) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let ev = w.events();
    if !out.is_null() {
        let os = core::slice::from_raw_parts_mut(out, capacity as usize);
        for (o, e) in os.iter_mut().zip(ev.iter()) {
            *o = PrEvent {
                kind: e.kind as u32,
                _pad: 0,
                a: e.a.to_bits(),
                b: e.b.to_bits(),
                point: e.point.into(),
                normal: e.normal.into(),
            };
        }
    }
    ev.len() as u32
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// Serialise the world. If `capacity` is too small nothing is written; the
/// return value is always the required size in bytes.
#[no_mangle]
pub unsafe extern "C" fn pr_world_save(w: *mut World, out: *mut u8, capacity: u32) -> u32 {
    let Some(w) = world(w) else { return 0 };
    let mut buf = Vec::new();
    w.save_snapshot(&mut buf);
    if !out.is_null() && buf.len() <= capacity as usize {
        core::ptr::copy_nonoverlapping(buf.as_ptr(), out, buf.len());
    }
    buf.len() as u32
}

/// Restore a snapshot. Returns 0 on success, otherwise:
/// 1 bad magic, 2 bad version, 3 truncated/corrupt, 4 missing shape, 5 bad args.
#[no_mangle]
pub unsafe extern "C" fn pr_world_load(w: *mut World, data: *const u8, len: u32) -> u32 {
    let Some(w) = world(w) else { return 5 };
    if data.is_null() {
        return 5;
    }
    let d = core::slice::from_raw_parts(data, len as usize);
    match w.load_snapshot(d) {
        Ok(()) => 0,
        Err(phys_r::SnapshotError::BadMagic) => 1,
        Err(phys_r::SnapshotError::BadVersion) => 2,
        Err(phys_r::SnapshotError::Truncated) => 3,
        Err(phys_r::SnapshotError::MissingShape) => 4,
    }
}

/// 64-bit hash of the full simulation state (desync detection).
#[no_mangle]
pub unsafe extern "C" fn pr_world_hash(w: *mut World) -> u64 {
    world(w).map_or(0, |w| w.state_hash())
}

#[no_mangle]
pub unsafe extern "C" fn pr_world_clear_history(w: *mut World) {
    if let Some(w) = world(w) {
        w.clear_history();
    }
}

// ---------------------------------------------------------------------------
// Deterministic math helpers (so managed code never needs float math for
// simulation inputs).
// ---------------------------------------------------------------------------

#[no_mangle]
pub extern "C" fn pr_fx_from_f32(v: f32) -> i64 {
    Fx::from_f32(v).0
}

#[no_mangle]
pub extern "C" fn pr_fx_from_f64(v: f64) -> i64 {
    Fx::from_f64(v).0
}

#[no_mangle]
pub extern "C" fn pr_fx_mul(a: i64, b: i64) -> i64 {
    (Fx(a) * Fx(b)).0
}

#[no_mangle]
pub extern "C" fn pr_fx_div(a: i64, b: i64) -> i64 {
    (Fx(a) / Fx(b)).0
}

#[no_mangle]
pub extern "C" fn pr_fx_sqrt(a: i64) -> i64 {
    Fx(a).sqrt().0
}

#[no_mangle]
pub extern "C" fn pr_fx_sin(a: i64) -> i64 {
    Fx(a).sin().0
}

#[no_mangle]
pub extern "C" fn pr_fx_cos(a: i64) -> i64 {
    Fx(a).cos().0
}

#[no_mangle]
pub extern "C" fn pr_fx_atan2(y: i64, x: i64) -> i64 {
    Fx::atan2(Fx(y), Fx(x)).0
}

#[no_mangle]
pub unsafe extern "C" fn pr_quat_from_euler(x: i64, y: i64, z: i64, out: *mut PrQuat) {
    write(out, Quat::from_euler(Fx(x), Fx(y), Fx(z)).into());
}

#[no_mangle]
pub unsafe extern "C" fn pr_quat_from_axis_angle(axis: *const PrVec3, angle: i64, out: *mut PrQuat) {
    if let Some(a) = read(axis) {
        write(out, Quat::from_axis_angle(a.into(), Fx(angle)).into());
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_quat_mul(a: *const PrQuat, b: *const PrQuat, out: *mut PrQuat) {
    if let (Some(a), Some(b)) = (read(a), read(b)) {
        write(out, Quat::from(a).mul(Quat::from(b)).into());
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_quat_rotate(q: *const PrQuat, v: *const PrVec3, out: *mut PrVec3) {
    if let (Some(q), Some(v)) = (read(q), read(v)) {
        write(out, Quat::from(q).rotate(v.into()).into());
    }
}

#[no_mangle]
pub unsafe extern "C" fn pr_vec3_normalize(v: *const PrVec3, out: *mut PrVec3) -> i64 {
    match read(v) {
        Some(v) => {
            let (n, len) = Vec3::from(v).normalize_with_len();
            write(out, n.into());
            len.0
        }
        None => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layouts_are_stable() {
        // Mirrored in C and C#: changing these breaks the ABI.
        assert_eq!(core::mem::size_of::<PrVec3>(), 24);
        assert_eq!(core::mem::size_of::<PrQuat>(), 32);
        assert_eq!(core::mem::size_of::<PrWorldSettings>(), 24 + 9 * 8 + 4 * 4);
        assert_eq!(core::mem::size_of::<PrBodyDesc>(), 8 + 24 + 32 + 24 + 24 + 6 * 8 + 16 + 8);
        assert_eq!(core::mem::size_of::<PrBodyState>(), 24 + 32 + 24 + 24 + 8 + 8);
        assert_eq!(core::mem::size_of::<PrTransformF>(), 32);
        assert_eq!(core::mem::size_of::<PrQueryFilter>(), 16);
        assert_eq!(core::mem::size_of::<PrRayHit>(), 8 + 8 + 24 * 3);
        assert_eq!(core::mem::size_of::<PrSweepHit>(), 8 + 8 + 24);
        assert_eq!(core::mem::size_of::<PrMovementSettings>(), 11 * 8 + 8);
        assert_eq!(core::mem::size_of::<PrCharacterDesc>(), 24 + 9 * 8 + 16 + 8 + 96);
        assert_eq!(core::mem::size_of::<PrCharacterInput>(), 40);
        assert_eq!(core::mem::size_of::<PrCharacterState>(), 24 * 4 + 8 * 4 + 16);
        assert_eq!(core::mem::size_of::<PrEvent>(), 8 + 16 + 48);
        assert_eq!(core::mem::size_of::<PrDoorDesc>(), 8 + 24 * 3 + 32 + 3 * 8 + 4 * 4 + 8);
        assert_eq!(core::mem::size_of::<PrDoorState>(), 8 * 4 + 8);
    }

    #[test]
    fn end_to_end() {
        unsafe {
            let w = pr_world_create(core::ptr::null());
            let g = pr_shape_box(w, &PrVec3 { x: 50 << 24, y: 1 << 24, z: 50 << 24 });
            let mut d = core::mem::zeroed::<PrBodyDesc>();
            pr_body_desc_default(&mut d);
            d.kind = 0;
            d.shape = g;
            d.position = PrVec3 { x: 0, y: -(1 << 24), z: 0 };
            assert_ne!(pr_body_create(w, &d), 0);
            let s = pr_shape_sphere(w, 1 << 23);
            d.kind = 1;
            d.shape = s;
            d.position = PrVec3 { x: 0, y: 3 << 24, z: 0 };
            let ball = pr_body_create(w, &d);
            assert_ne!(ball, 0);
            pr_world_step(w, 120);
            let mut st = core::mem::zeroed::<PrBodyState>();
            assert_eq!(pr_body_get_state(w, ball, &mut st), 1);
            assert!((Fx(st.position.y).to_f64() - 0.5).abs() < 0.02);

            let mut hit = PrRayHit::default();
            let f = PrQueryFilter { mask: u32::MAX, flags: 0, exclude: 0 };
            assert_eq!(
                pr_raycast(w, &PrVec3 { x: 0, y: 5 << 24, z: 0 }, &PrVec3 { x: 0, y: -(1 << 24), z: 0 }, 100 << 24, &f, &mut hit),
                1
            );
            assert_eq!(hit.body, ball);

            let size = pr_world_save(w, core::ptr::null_mut(), 0);
            let mut buf = vec![0u8; size as usize];
            assert_eq!(pr_world_save(w, buf.as_mut_ptr(), size), size);
            let h1 = pr_world_hash(w);
            pr_world_step(w, 10);
            assert_eq!(pr_world_load(w, buf.as_ptr(), size), 0);
            assert_eq!(pr_world_hash(w), h1);
            pr_world_destroy(w);
        }
    }
}
