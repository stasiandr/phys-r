# phys-r

Fast, **deterministic** 3D physics in Rust for **networked first-person shooters on mobile**, shipped as a Unity package.

- **Bit-identical everywhere.** The whole simulation runs in Q40.24 fixed point (`Fx`, an `i64`). There are no floats in the simulation path, so x86_64, i686, ARM64 and ARMv7 produce exactly the same state, in debug and release builds. A golden-hash test verifies this on all four.
- **Built for FPS.** It includes a deterministic character controller, hitscan raycasts with hit zones, lag compensation (rewound raycasts), snapshots/rollback and a state hash for desync detection.
- **Fast.** No dependencies, no allocations in the steady-state step, and sleeping islands. Level geometry sits in a SAH BVH that is traversed as a 4-wide tree with SIMD (SSE / NEON) box tests. Characters and batched raycasts run on a small worker pool. Upright capsules against level boxes use closed-form sweeps. The solver uses precomputed Jacobians.

## Features

| Area | What you get |
|---|---|
| Shapes | Sphere, capsule, box. Convex primitives only: levels are built from boxes in a BVH (no triangle meshes; they are slow by design). |
| Rigid bodies | Static / dynamic / kinematic bodies, mass and inertia, friction, restitution, damping, gravity scale, rotation lock, collision layers and masks |
| Solver | Sequential impulse with warm starting, central friction plus twist friction, split-impulse position correction (stacks stay quiet), speculative contacts, optional velocity-expanded CCD |
| Sleeping | Island-based, deterministic union-find |
| Events | Contact begin/end, sensor (trigger) enter/exit |
| Queries | Raycast (closest hit, with body-local hit point for headshots), parallel `raycast_batch` for all shots of a tick, sphere cast, capsule cast, sphere/shape overlap, layer masks, exclusion |
| Character | See below |
| Doors | Hinged or sliding, open/close/partial targets at a fixed speed, exact kinematic poses, stop on players or push them; state is in snapshots |
| Netcode | `save_snapshot` / `load_snapshot`, `state_hash`, single-character `simulate_character` for prediction/replay, pose history plus `raycast_rewind` for lag compensation |

### FPS character controller

A kinematic capsule driven by per-tick input (`move_x/y/z`, `yaw`, jump/sprint/crouch):

- **Movement.** Quake-style ground/air acceleration, friction, air control cap, gravity and jumping. Release-to-rejump is optional, and **coyote time** allows a jump shortly after walking off an edge.
- **Collide-and-slide.** Crease handling, **step climbing**, slope limit, **ground snapping** (going down stairs and slopes stays grounded), sliding down steep slopes, and ceiling bumps.
- **Crouch.** The capsule really shrinks: feet stay planted on the ground and the head stays put in the air. Standing up is refused under a ceiling (`STAND_BLOCKED`).
- **Moving platforms.** The character is carried by kinematic or dynamic ground and inherits the platform's momentum when jumping or walking off.
- **Modes.** `Walk`, `Fly` (ladders, water, spectators with collision) and `Noclip`.
- **Feedback.** `add_character_velocity` handles knockback, explosions and jump pads. `landing_speed` supports fall damage. Collision flags report below, above, sides, landed, jumped and stand-blocked.
- **Interaction.** Characters block each other and push dynamic props. They are raycastable, `character_of_body` maps a hit body to its character, and they are recorded for lag compensation.

The controller is a pure function of world state and input. Replaying the same inputs from a snapshot reproduces positions bit for bit, so client prediction and server reconciliation are exact.

## Layout

```
crates/phys-r        core engine (no dependencies)
crates/phys-r-ffi    C ABI (cdylib + staticlib) + include/phys_r.h
unity/com.physr.engine   Unity package (C# bindings, components, sample)
scripts/             plugin build script, Unity .meta generator
tools/csharp-check   compiles the Unity package against a stub and runs the
                     bindings test on Mono against the native library
```

## Rust quick start

```rust
use phys_r::*;

let mut world = World::new(WorldSettings::default()); // 60 Hz
let floor = world.add_shape(Shape::cuboid(Vec3::from_int(50, 1, 50)));
world.create_body(&BodyDesc::new_static(floor, Vec3::from_int(0, -1, 0)))?;

let crate_shape = world.add_shape(Shape::cuboid(Vec3::splat(fx!(0.4))));
world.create_body(&BodyDesc::new_dynamic(crate_shape, Vec3::from_int(0, 3, 0), fx!(10.0)))?;

let player = world.create_character(&CharacterDesc { position: Vec3::from_int(2, 1, 0), ..Default::default() })?;

// Each tick: apply inputs, step.
world.set_character_input(player, CharacterInput { move_z: Fx::ONE, yaw: Fx::from_ratio(157, 100), ..Default::default() });
world.step();

// Hitscan with headshots.
if let Some(hit) = world.raycast(eye, dir, fx!(200.0), &QueryFilter::default().excluding(shooter_body)) {
    if let Some(victim) = world.character_of_body(hit.body) {
        let headshot = hit.local_point.y > fx!(0.55);
    }
}

// Rollback / desync detection.
let mut snap = Vec::new();
world.save_snapshot(&mut snap);
let hash = world.state_hash();
world.load_snapshot(&snap)?;
```

## Unity

1. Build the native plugins: `scripts/build-unity-plugins.sh android ios` (or `macos`, `windows`, `linux`, `host`). The script lists per-platform requirements (NDK + `cargo-ndk`, Xcode, …). Binaries go to `unity/com.physr.engine/Plugins/` together with `.meta` import settings.
2. Add the package: Package Manager → *Add package from disk* → `unity/com.physr.engine/package.json`.
3. In the scene:
   - **`PhysRWorld`**: one per scene. It owns the world, runs the fixed tick (or set `autoSimulate = false` and call `Tick()` from your netcode) and interpolates visuals.
   - **`PhysRStaticGeometry`**: put the level under it. It bakes every Box/Sphere/Capsule collider (triggers become sensors) into static bodies.
   - **`PhysRBody`**: dynamic and kinematic props.
   - **`PhysRDoor`**: doors (hinge side or slide). Call `Open()`/`Close()` from simulated game logic.
   - **`PhysRCharacter`**: players. Set `NextInput` every tick. See `Samples~/FpsBasic/SimpleFpsPlayer.cs` for quantized network input, crouch-aware eye height and headshot detection.

Determinism rules on the C# side:

- Keep gameplay values in `FP` / `FPVector3`. Their `+ - * /` match the native engine bit for bit; this is verified against the native library on 200k random pairs.
- Never feed Unity floats from runtime math into the simulation.
- Convert authoring data with `FP.FromFloat` (done natively, so it is exact).
- World poses are composed in fixed point from serialized local transforms. Unity's float world matrices can differ between ARM and x86 builds because of fused multiply-add.
- Quantize inputs on the wire and rebuild them with `FP.FromRatio`.
- Shapes and settings are configuration: register them identically on every peer. Bodies are created sorted by `(creationOrder, hierarchy path)`.

## Performance

### Target map: 5000 static colliders, 20 doors, 16 players

Run `cargo run --release -p phys-r --example fps_map` (`THREADS=1` forces single-threaded). The scene has 400 floor tiles plus 4600 walls, crates, pillars, capsules and ramps (a third of them rotated), 20 hinged doors opening and closing, 16 players running, jumping and crouching, and 32 hitscan rays per tick cast with `raycast_batch`.

Results on a shared 4-vCPU x86_64 cloud VM (noisy; best of several runs):

| | before | 1 thread | 4 threads |
|---|---:|---:|---:|
| Step (doors + 16 players) | ~40 µs | ~28 µs | ~21 µs |
| Raycast across the map | ~0.9 µs | ~0.6 µs | ~0.33 µs |
| Whole tick (step + 32 shots) | ~69 µs | ~47 µs | ~32 µs |

Thousands of static colliders cost almost nothing per tick. They sit in an SAH BVH, and every per-tick loop only visits moving bodies. Each character gathers nearby statics once per tick from a compact, spatially ordered copy of the level, then runs about four sweeps against a dozen obstacles.

### Threads and determinism

`WorldSettings::threads` (`0` = auto, up to 4; also `World::set_threads`, `pr_world_set_threads`) sets the worker count. Character moves are computed in parallel from the same start-of-tick state and applied in index order. Batched rays only read the world. The result is bit-identical for every thread count, which the golden test checks. Items keep their thread across ticks (character `i` and shot `i` prefer the same core), so each core stays warm on its part of the level.

The SIMD box tests are f32, and they only cull conservatively: boxes are inflated outward, and every surviving candidate is decided by the exact fixed-point test. Rounding differences between SSE, NEON and scalar code therefore never change a result.

### Prop-heavy worst case

Run `cargo run --release -p phys-r --example bench`. The scene is a 32×32 tiled box level with 300 cover boxes, N props, 16–32 characters (moving and jumping), 64–128 raycasts per tick, and sleeping disabled (worst case).

Results on a shared x86_64 cloud container (noisy, roughly 2× run-to-run variance):

| props | players | step |
|---:|---:|---:|
| 0 | 16 | ~17–20 µs |
| 50 | 16 | ~70–120 µs |
| 200 | 16 | ~0.35–0.7 ms |

Raycasts cost about 1 µs each against the level. Expect roughly 2–3× these times on a recent phone core. With sleeping enabled (the default), props at rest cost close to nothing.

## Testing

```
cargo test --workspace                 # unit + simulation + golden hash + FFI
tools/csharp-check/run.sh              # Unity package compile check + C# bindings test (needs mono)
```

Cross-architecture determinism: run the golden test on other targets, for example under qemu:

```
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_RUNNER="qemu-aarch64 -L /usr/aarch64-linux-gnu" \
cargo test --release -p phys-r --target aarch64-unknown-linux-gnu
```

If you intentionally change simulation behavior, update `GOLDEN` in `crates/phys-r/tests/golden.rs`.

## Limitations

- No joints or constraints besides contacts yet.
- No convex hulls or meshes. Model the level with boxes, capsules and spheres.
- Only characters and batched raycasts run in parallel; the rigid-body solver is single-threaded (it is cheap when props sleep).
