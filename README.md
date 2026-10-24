# phys-r

Fast, **deterministic** 3D physics in Rust for **networked first-person shooters on mobile**, shipped as a Unity package.

- **Bit-identical everywhere.** The whole simulation runs in Q40.24 fixed point (`Fx`, an `i64`). There are no floats in the simulation path, so x86_64, i686, ARM64 and ARMv7 produce exactly the same state, in debug and release builds. A golden-hash test verifies this on all four.
- **Built for FPS.** It includes a deterministic character controller, hitscan raycasts with hit zones, lag compensation (rewound raycasts), snapshots/rollback and a state hash for desync detection.
- **Fast.** No dependencies, no allocations in the steady-state step, sleeping islands, and a SAH BVH for level geometry. Mover queries use a refit BVH. The solver uses precomputed Jacobians.

## Features

| Area | What you get |
|---|---|
| Shapes | Sphere, capsule, box. Convex primitives only: levels are built from boxes in a BVH (no triangle meshes; they are slow by design). |
| Rigid bodies | Static / dynamic / kinematic bodies, mass and inertia, friction, restitution, damping, gravity scale, rotation lock, collision layers and masks |
| Solver | Sequential impulse with warm starting, central friction plus twist friction, split-impulse position correction (stacks stay quiet), speculative contacts, optional velocity-expanded CCD |
| Sleeping | Island-based, deterministic union-find |
| Events | Contact begin/end, sensor (trigger) enter/exit |
| Queries | Raycast (closest hit, with body-local hit point for headshots), sphere cast, capsule cast, sphere/shape overlap, layer masks, exclusion |
| Character | See below |
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
   - **`PhysRCharacter`**: players. Set `NextInput` every tick. See `Samples~/FpsBasic/SimpleFpsPlayer.cs` for quantized network input, crouch-aware eye height and headshot detection.

Determinism rules on the C# side:

- Keep gameplay values in `FP` / `FPVector3`. Their `+ - * /` match the native engine bit for bit; this is verified against the native library on 200k random pairs.
- Never feed Unity floats from runtime math into the simulation.
- Convert authoring data with `FP.FromFloat` (done natively, so it is exact).
- World poses are composed in fixed point from serialized local transforms. Unity's float world matrices can differ between ARM and x86 builds because of fused multiply-add.
- Quantize inputs on the wire and rebuild them with `FP.FromRatio`.
- Shapes and settings are configuration: register them identically on every peer. Bodies are created sorted by `(creationOrder, hierarchy path)`.

## Performance

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
- Single-threaded per world. Run several worlds on different threads if needed.
