/*
 * phys-r — deterministic fixed-point physics for networked mobile FPS games.
 * C API. Layouts must stay in sync with crates/phys-r-ffi/src/lib.rs and the
 * Unity bindings (PhysRNative.cs).
 *
 * Conventions:
 *  - pr_fx is a raw Q40.24 fixed-point value (1.0 == 1 << 24).
 *  - Handles are uint64_t; 0 is invalid.
 *  - Functions never abort on bad input: invalid handles return 0.
 *  - A world must only be used by one thread at a time.
 */
#ifndef PHYS_R_H
#define PHYS_R_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int64_t pr_fx;
typedef struct PrWorld PrWorld;

#define PR_FX_ONE ((pr_fx)1 << 24)
#define PR_INVALID_SHAPE 0xFFFFFFFFu

/* Body kinds */
#define PR_STATIC 0u
#define PR_DYNAMIC 1u
#define PR_KINEMATIC 2u

/* Body flags */
#define PR_FLAG_SENSOR (1u << 0)
#define PR_FLAG_CCD (1u << 1)
#define PR_FLAG_HISTORY (1u << 2)
#define PR_FLAG_NO_SLEEP (1u << 3)
#define PR_FLAG_LOCK_ROTATION (1u << 4)
#define PR_FLAG_CONTACT_EVENTS (1u << 5)

/* Query filter flags */
#define PR_QUERY_INCLUDE_SENSORS (1u << 0)
#define PR_QUERY_EXCLUDE_STATIC (1u << 1)
#define PR_QUERY_EXCLUDE_MOVING (1u << 2)

/* Character buttons */
#define PR_BUTTON_JUMP (1u << 0)
#define PR_BUTTON_SPRINT (1u << 1)
#define PR_BUTTON_CROUCH (1u << 2)

/* Character movement modes */
#define PR_MODE_WALK 0u
#define PR_MODE_FLY 1u
#define PR_MODE_NOCLIP 2u

/* Character collision flags (PrCharacterState.collision_flags) */
#define PR_COLLIDED_BELOW (1u << 0)
#define PR_COLLIDED_ABOVE (1u << 1)
#define PR_COLLIDED_SIDES (1u << 2)
#define PR_COLLIDED_LANDED (1u << 3)
#define PR_COLLIDED_JUMPED (1u << 4)
#define PR_COLLIDED_STAND_BLOCKED (1u << 5)

/* Event kinds */
#define PR_EVENT_CONTACT_BEGIN 0u
#define PR_EVENT_CONTACT_END 1u
#define PR_EVENT_SENSOR_ENTER 2u
#define PR_EVENT_SENSOR_EXIT 3u

typedef struct { pr_fx x, y, z; } PrVec3;
typedef struct { pr_fx x, y, z, w; } PrQuat;

typedef struct {
    PrVec3 gravity;
    pr_fx dt;
    pr_fx contact_margin;
    pr_fx slop;
    pr_fx baumgarte;
    pr_fx max_correction_velocity;
    pr_fx restitution_threshold;
    pr_fx sleep_linear_threshold;
    pr_fx sleep_angular_threshold;
    pr_fx time_to_sleep;
    uint32_t velocity_iterations;
    uint32_t position_iterations;
    uint32_t history_ticks;
    uint32_t enable_sleep;
    uint32_t threads; /* 0 = auto, 1 = single-threaded; results identical */
    uint32_t _pad;
} PrWorldSettings;

typedef struct {
    uint32_t kind;
    uint32_t shape;
    PrVec3 position;
    PrQuat rotation;
    PrVec3 linear_velocity;
    PrVec3 angular_velocity;
    pr_fx mass;
    pr_fx friction;
    pr_fx restitution;
    pr_fx linear_damping;
    pr_fx angular_damping;
    pr_fx gravity_scale;
    uint32_t layer;
    uint32_t mask;
    uint32_t flags;
    uint32_t _pad;
    uint64_t user_data;
} PrBodyDesc;

typedef struct {
    PrVec3 position;
    PrQuat rotation;
    PrVec3 linear_velocity;
    PrVec3 angular_velocity;
    uint32_t sleeping;
    uint32_t kind;
    uint64_t user_data;
} PrBodyState;

typedef struct {
    float px, py, pz;
    float rx, ry, rz, rw;
    uint32_t valid;
} PrTransformF;

typedef struct {
    uint32_t mask;
    uint32_t flags;
    uint64_t exclude;
} PrQueryFilter;

typedef struct {
    PrVec3 origin;
    PrVec3 dir;
    pr_fx max_dist;
    PrQueryFilter filter;
} PrRayQuery;

typedef struct {
    uint64_t body;
    pr_fx distance;
    PrVec3 point;
    PrVec3 normal;
    PrVec3 local_point; /* hit point in body space (hit zones) */
} PrRayHit;

typedef struct {
    uint64_t body;
    pr_fx fraction;
    PrVec3 normal;
} PrSweepHit;

typedef struct {
    pr_fx walk_speed;
    pr_fx sprint_speed;
    pr_fx crouch_speed;
    pr_fx ground_accel;
    pr_fx air_accel;
    pr_fx air_speed_cap;
    pr_fx friction;
    pr_fx stop_speed;
    pr_fx gravity;
    pr_fx jump_speed;
    pr_fx max_fall_speed;
    uint32_t jump_requires_release;
    uint32_t coyote_ticks;
} PrMovementSettings;

typedef struct {
    PrVec3 position;
    pr_fx radius;
    pr_fx height;
    pr_fx crouch_height;
    pr_fx step_height;
    pr_fx max_slope_cos;
    pr_fx skin;
    pr_fx snap_distance;
    pr_fx mass;
    pr_fx push_strength;
    uint32_t layer;
    uint32_t mask;
    uint32_t body_flags;
    uint32_t _pad;
    uint64_t user_data;
    PrMovementSettings movement;
} PrCharacterDesc;

typedef struct {
    pr_fx move_x;
    pr_fx move_y; /* fly mode: ladders / swimming */
    pr_fx move_z;
    pr_fx yaw;
    uint32_t buttons;
    uint32_t _pad;
} PrCharacterInput;

typedef struct {
    PrVec3 position; /* capsule centre */
    PrVec3 velocity;
    PrVec3 ground_normal;
    PrVec3 ground_velocity; /* moving platform under the feet */
    uint64_t body;
    uint64_t ground_body; /* 0 when airborne */
    pr_fx landing_speed;  /* downward speed on the landing tick (fall damage) */
    pr_fx height;         /* current capsule height (crouch aware) */
    uint32_t grounded;
    uint32_t crouched;
    uint32_t mode;            /* PR_MODE_* */
    uint32_t collision_flags; /* PR_COLLIDED_* */
} PrCharacterState;

typedef struct {
    uint32_t kind;
    uint32_t _pad;
    uint64_t a;
    uint64_t b;
    PrVec3 point;
    PrVec3 normal;
} PrEvent;

typedef struct {
    uint32_t shape;
    uint32_t kind;  /* PR_DOOR_HINGE (amount in radians) / PR_DOOR_SLIDE (metres) */
    PrVec3 pivot;   /* hinge: point on axis; slide: closed body centre */
    PrVec3 axis;    /* hinge axis / slide direction */
    PrQuat closed_rotation;
    PrVec3 offset;  /* hinge: closed body centre relative to pivot */
    pr_fx open_amount;
    pr_fx speed;
    pr_fx friction;
    uint32_t layer;
    uint32_t mask;
    uint32_t block_on_characters; /* stop instead of pushing players */
    uint32_t _pad;
    uint64_t user_data;
} PrDoorDesc;

typedef struct {
    uint64_t body;
    pr_fx amount;
    pr_fx target;
    pr_fx open_fraction; /* 0 closed .. 1 open */
    uint32_t blocked;
    uint32_t moving;
} PrDoorState;

#define PR_DOOR_HINGE 0u
#define PR_DOOR_SLIDE 1u

uint32_t pr_version(void);

/* World */
void pr_world_settings_default(PrWorldSettings *out);
PrWorld *pr_world_create(const PrWorldSettings *settings /* nullable */);
void pr_world_destroy(PrWorld *w);
void pr_world_step(PrWorld *w, uint32_t steps);
uint32_t pr_world_tick(PrWorld *w);
void pr_world_set_threads(PrWorld *w, uint32_t threads);
void pr_world_set_gravity(PrWorld *w, const PrVec3 *g);

/* Shapes: convex primitives only (registered in order; not part of snapshots).
   Build levels from boxes: static bodies live in a BVH. */
uint32_t pr_shape_sphere(PrWorld *w, pr_fx radius);
uint32_t pr_shape_capsule(PrWorld *w, pr_fx half_height, pr_fx radius);
uint32_t pr_shape_box(PrWorld *w, const PrVec3 *half_extents);
uint32_t pr_shape_count(PrWorld *w);

/* Bodies */
void pr_body_desc_default(PrBodyDesc *out);
uint64_t pr_body_create(PrWorld *w, const PrBodyDesc *desc);
uint32_t pr_body_destroy(PrWorld *w, uint64_t body);
uint32_t pr_body_is_valid(PrWorld *w, uint64_t body);
uint32_t pr_body_get_state(PrWorld *w, uint64_t body, PrBodyState *out);
uint32_t pr_body_get_transforms_f32(PrWorld *w, const uint64_t *bodies, uint32_t count, PrTransformF *out);
uint32_t pr_body_set_pose(PrWorld *w, uint64_t body, const PrVec3 *position /* nullable */, const PrQuat *rotation /* nullable */);
uint32_t pr_body_set_velocity(PrWorld *w, uint64_t body, const PrVec3 *linear /* nullable */, const PrVec3 *angular /* nullable */);
uint32_t pr_body_set_kinematic_target(PrWorld *w, uint64_t body, const PrVec3 *position, const PrQuat *rotation);
uint32_t pr_body_apply_impulse(PrWorld *w, uint64_t body, const PrVec3 *impulse, const PrVec3 *point /* nullable */);
uint32_t pr_body_apply_force(PrWorld *w, uint64_t body, const PrVec3 *force, const PrVec3 *point /* nullable */);
uint32_t pr_body_apply_torque(PrWorld *w, uint64_t body, const PrVec3 *torque);
void pr_body_wake(PrWorld *w, uint64_t body);
uint32_t pr_body_set_filter(PrWorld *w, uint64_t body, uint32_t layer, uint32_t mask);
uint32_t pr_body_set_material(PrWorld *w, uint64_t body, pr_fx friction, pr_fx restitution);
uint32_t pr_body_set_flags(PrWorld *w, uint64_t body, uint32_t flags);

/* Queries (filter may be null = hit everything) */
uint32_t pr_raycast(PrWorld *w, const PrVec3 *origin, const PrVec3 *dir, pr_fx max_dist, const PrQueryFilter *filter, PrRayHit *out);
/* Casts n rays in parallel; out[i].body == 0 on a miss. Returns the hit count. */
uint32_t pr_raycast_batch(PrWorld *w, const PrRayQuery *rays, uint32_t n, PrRayHit *out);
uint32_t pr_raycast_rewind(PrWorld *w, uint32_t tick, pr_fx alpha, const PrVec3 *origin, const PrVec3 *dir, pr_fx max_dist, const PrQueryFilter *filter, PrRayHit *out);
uint32_t pr_overlap_sphere(PrWorld *w, const PrVec3 *center, pr_fx radius, const PrQueryFilter *filter, uint64_t *out, uint32_t capacity);
uint32_t pr_sphere_cast(PrWorld *w, const PrVec3 *center, pr_fx radius, const PrVec3 *disp, const PrQueryFilter *filter, PrSweepHit *out);
uint32_t pr_capsule_cast(PrWorld *w, const PrVec3 *center, pr_fx half_height, pr_fx radius, const PrVec3 *disp, const PrQueryFilter *filter, PrSweepHit *out);

/* Characters */
void pr_character_desc_default(PrCharacterDesc *out);
uint64_t pr_character_create(PrWorld *w, const PrCharacterDesc *desc);
uint32_t pr_character_destroy(PrWorld *w, uint64_t character);
uint32_t pr_character_set_input(PrWorld *w, uint64_t character, const PrCharacterInput *input);
uint32_t pr_character_simulate(PrWorld *w, uint64_t character, const PrCharacterInput *input, pr_fx dt /* <= 0: world dt */);
uint32_t pr_character_get_state(PrWorld *w, uint64_t character, PrCharacterState *out);
uint32_t pr_character_teleport(PrWorld *w, uint64_t character, const PrVec3 *position);
uint32_t pr_character_set_velocity(PrWorld *w, uint64_t character, const PrVec3 *velocity);
uint32_t pr_character_add_velocity(PrWorld *w, uint64_t character, const PrVec3 *dv);
uint32_t pr_character_set_mode(PrWorld *w, uint64_t character, uint32_t mode);
uint64_t pr_character_of_body(PrWorld *w, uint64_t body);

/* Doors (state is part of snapshots) */
void pr_door_desc_default(PrDoorDesc *out);
uint64_t pr_door_create(PrWorld *w, const PrDoorDesc *desc);
uint32_t pr_door_destroy(PrWorld *w, uint64_t door);
uint32_t pr_door_set_open(PrWorld *w, uint64_t door, uint32_t open);
uint32_t pr_door_set_target(PrWorld *w, uint64_t door, pr_fx fraction);
uint32_t pr_door_set_amount(PrWorld *w, uint64_t door, pr_fx fraction);
uint32_t pr_door_get_state(PrWorld *w, uint64_t door, PrDoorState *out);

/* Events from the last step */
uint32_t pr_world_event_count(PrWorld *w);
uint32_t pr_world_get_events(PrWorld *w, PrEvent *out, uint32_t capacity);

/* Snapshots / desync detection */
uint32_t pr_world_save(PrWorld *w, uint8_t *out /* nullable */, uint32_t capacity);
uint32_t pr_world_load(PrWorld *w, const uint8_t *data, uint32_t len);
uint64_t pr_world_hash(PrWorld *w);
void pr_world_clear_history(PrWorld *w);

/* Deterministic math helpers */
pr_fx pr_fx_from_f32(float v);
pr_fx pr_fx_from_f64(double v);
pr_fx pr_fx_mul(pr_fx a, pr_fx b);
pr_fx pr_fx_div(pr_fx a, pr_fx b);
pr_fx pr_fx_sqrt(pr_fx a);
pr_fx pr_fx_sin(pr_fx a);
pr_fx pr_fx_cos(pr_fx a);
pr_fx pr_fx_atan2(pr_fx y, pr_fx x);
void pr_quat_from_euler(pr_fx x, pr_fx y, pr_fx z, PrQuat *out);
void pr_quat_from_axis_angle(const PrVec3 *axis, pr_fx angle, PrQuat *out);
void pr_quat_mul(const PrQuat *a, const PrQuat *b, PrQuat *out);
void pr_quat_rotate(const PrQuat *q, const PrVec3 *v, PrVec3 *out);
pr_fx pr_vec3_normalize(const PrVec3 *v, PrVec3 *out);

#ifdef __cplusplus
}
#endif

#endif /* PHYS_R_H */
