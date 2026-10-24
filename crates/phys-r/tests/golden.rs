//! Cross-platform determinism check.
//!
//! Runs a scenario exercising every subsystem (tiled box level, stacked boxes,
//! spheres, capsules, CCD, sensors, hinged and sliding doors, characters with jumping / strafing /
//! steps, raycasts, rewinds, snapshots) and compares the final state hash
//! against a golden value. Every supported target (x86_64, i686, aarch64,
//! armv7, wasm, ...) must produce exactly this hash; CI runs this test on
//! several architectures.
//!
//! If you intentionally change simulation behaviour, update GOLDEN with the
//! value printed by `cargo test --test golden -- --nocapture`.

use phys_r::*;

const GOLDEN: u64 = 0xa5e5_555d_d8fc_c9a4;

fn f(v: f64) -> Fx {
    Fx::from_f64(v)
}

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(f(x), f(y), f(z))
}

fn scenario() -> (u64, Vec<u64>) {
    let mut w = World::new(WorldSettings::default());
    // Level floor: tiles of slightly different heights (small ledges).
    let tile = w.add_shape(Shape::cuboid(v(2.0, 1.0, 2.0)));
    for i in 0..12 {
        for j in 0..12 {
            let y = ((i * 7 + j * 13) % 5) as f64 * 0.04 - 1.0;
            w.create_body(&BodyDesc::new_static(tile, v(-24.0 + 2.0 + i as f64 * 4.0, y, -24.0 + 2.0 + j as f64 * 4.0))).unwrap();
        }
    }
    let wall = w.add_shape(Shape::cuboid(v(3.0, 1.5, 0.25)));
    w.create_body(&BodyDesc { rotation: Quat::from_yaw(f(0.4)), ..BodyDesc::new_static(wall, v(4.0, 1.5, 6.0)) }).unwrap();
    let step = w.add_shape(Shape::cuboid(v(1.5, 0.15, 0.5)));
    for i in 0..3 {
        w.create_body(&BodyDesc::new_static(step, v(-6.0, 0.15 + 0.3 * i as f64, 2.0 + i as f64))).unwrap();
    }
    let trig = w.add_shape(Shape::cuboid(v(2.0, 2.0, 2.0)));
    w.create_body(&BodyDesc { flags: flags::SENSOR, ..BodyDesc::new_static(trig, v(0.0, 2.0, -6.0)) }).unwrap();

    let bx = w.add_shape(Shape::cuboid(v(0.4, 0.4, 0.4)));
    let sp = w.add_shape(Shape::sphere(f(0.3)));
    let cp = w.add_shape(Shape::capsule(f(0.35), f(0.2)));
    for i in 0..5 {
        w.create_body(&BodyDesc::new_dynamic(bx, v(2.0, 0.6 + i as f64 * 0.81, -2.0), f(2.0))).unwrap();
    }
    for i in 0..12 {
        let s = [bx, sp, cp][i % 3];
        w.create_body(&BodyDesc {
            rotation: Quat::from_euler(Fx::from_ratio(i as i64, 3), Fx::from_ratio(i as i64, 5), Fx::ZERO),
            restitution: Fx::from_ratio((i % 3) as i64, 4),
            linear_velocity: v(0.0, 0.0, (i % 4) as f64 - 1.5),
            flags: if i % 4 == 0 { flags::CONTACT_EVENTS } else { 0 },
            ..BodyDesc::new_dynamic(s, v(-2.0 + (i % 4) as f64, 2.0 + (i / 4) as f64, -6.0 + (i % 3) as f64), f(1.0 + (i % 3) as f64))
        })
        .unwrap();
    }
    let bullet = w
        .create_body(&BodyDesc {
            flags: flags::CCD,
            linear_velocity: v(0.0, 0.0, 60.0),
            gravity_scale: Fx::ZERO,
            ..BodyDesc::new_dynamic(sp, v(4.0, 1.0, 0.0), f(0.1))
        })
        .unwrap();
    let _ = bullet;

    let slab = w.add_shape(Shape::cuboid(v(0.6, 1.1, 0.05)));
    let doors = [
        w.create_door(&DoorDesc { shape: slab, pivot: v(-3.0, 1.2, 1.0), offset: v(0.6, 0.0, 0.0), ..Default::default() }).unwrap(),
        w.create_door(&DoorDesc {
            shape: slab,
            kind: DoorKind::Slide,
            pivot: v(1.0, 1.2, -4.0),
            axis: Vec3::X,
            open_amount: f(1.2),
            speed: f(1.5),
            block_on_characters: false,
            ..Default::default()
        })
        .unwrap(),
    ];

    let mut chars = Vec::new();
    for i in 0..4 {
        chars.push(w.create_character(&CharacterDesc { position: v(-6.0 + i as f64 * 1.5, 1.2, -1.0), ..Default::default() }).unwrap());
    }

    let mut trace = Vec::new();
    let mut snap = Vec::new();
    for t in 0..360u32 {
        for (k, c) in chars.iter().enumerate() {
            let input = CharacterInput {
                move_x: Fx::from_ratio(((t / 30 + k as u32) % 3) as i64 - 1, 1),
                move_z: Fx::from_ratio(((t / 45 + k as u32) % 3) as i64 - 1, 1),
                yaw: Fx::from_ratio((t as i64 * 3 + k as i64 * 50) % 628, 100),
                jump: (t + k as u32 * 17) % 90 == 0,
                sprint: k == 1,
                crouch: k == 3 && t % 120 < 60,
                move_y: Fx::ZERO,
            };
            w.set_character_input(*c, input);
        }
        for (k, d) in doors.iter().enumerate() {
            w.set_door_open(*d, (t / (70 + k as u32 * 25)) % 2 == 0);
        }
        w.step();
        if t == 120 {
            w.save_snapshot(&mut snap);
        }
        if t % 60 == 59 {
            let mut acc = w.state_hash();
            // Fold in some queries too.
            for r in 0..16 {
                let a = Fx::from_ratio(r * 39, 100);
                let (s, c) = a.sin_cos();
                if let Some(h) = w.raycast(v(0.0, 1.0, 0.0), Vec3::new(s, f(-0.1), c), f(60.0), &QueryFilter::default()) {
                    acc = acc.rotate_left(7) ^ (h.distance.0 as u64) ^ h.body.to_bits();
                }
                if let Some(h) = w.raycast_rewind(
                    w.tick().wrapping_sub(10),
                    Fx::HALF,
                    v(0.0, 1.0, 0.0),
                    Vec3::new(s, Fx::ZERO, c),
                    f(60.0),
                    &QueryFilter::default(),
                ) {
                    acc = acc.rotate_left(5) ^ (h.distance.0 as u64);
                }
            }
            trace.push(acc);
        }
    }
    // Rollback and resimulate must reproduce the same end state.
    let end = w.state_hash();
    w.load_snapshot(&snap).unwrap();
    for t in 121..360u32 {
        for (k, c) in chars.iter().enumerate() {
            let input = CharacterInput {
                move_x: Fx::from_ratio(((t / 30 + k as u32) % 3) as i64 - 1, 1),
                move_z: Fx::from_ratio(((t / 45 + k as u32) % 3) as i64 - 1, 1),
                yaw: Fx::from_ratio((t as i64 * 3 + k as i64 * 50) % 628, 100),
                jump: (t + k as u32 * 17) % 90 == 0,
                sprint: k == 1,
                crouch: k == 3 && t % 120 < 60,
                move_y: Fx::ZERO,
            };
            w.set_character_input(*c, input);
        }
        for (k, d) in doors.iter().enumerate() {
            w.set_door_open(*d, (t / (70 + k as u32 * 25)) % 2 == 0);
        }
        w.step();
    }
    assert_eq!(w.state_hash(), end, "rollback diverged");
    (end, trace)
}

#[test]
fn golden_state_hash() {
    let (h, trace) = scenario();
    let folded = trace.iter().fold(h, |a, x| a.rotate_left(13) ^ x);
    println!("GOLDEN = 0x{folded:016x}");
    for (i, t) in trace.iter().enumerate() {
        println!("  checkpoint {i}: {t:016x}");
    }
    if GOLDEN != 0 {
        assert_eq!(folded, GOLDEN, "simulation is not bit-identical to the golden run on this platform");
    }
}
