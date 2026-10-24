//! Target scenario: ~5000 static colliders, 20 doors, 16 players.
//!
//! cargo run --release -p phys-r --example fps_map

use phys_r::*;
use std::time::Instant;

fn f(v: f64) -> Fx {
    Fx::from_f64(v)
}

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(f(x), f(y), f(z))
}

fn main() {
    let t0 = Instant::now();
    let threads: u32 = std::env::var("THREADS").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let mut w = World::new(WorldSettings { threads, ..Default::default() });
    // Floor tiles (20 x 20 of 8 m).
    let tile = w.add_shape(Shape::cuboid(v(4.0, 0.5, 4.0)));
    for i in 0..20 {
        for j in 0..20 {
            w.create_body(&BodyDesc::new_static(tile, v(-80.0 + 4.0 + i as f64 * 8.0, -0.5, -80.0 + 4.0 + j as f64 * 8.0))).unwrap();
        }
    }
    // Walls, crates, pillars, ramps: fill up to 5000 statics.
    let shapes = [
        w.add_shape(Shape::cuboid(v(3.0, 1.5, 0.2))),
        w.add_shape(Shape::cuboid(v(0.5, 0.5, 0.5))),
        w.add_shape(Shape::cuboid(v(0.3, 2.0, 0.3))),
        w.add_shape(Shape::capsule(f(1.0), f(0.3))),
        w.add_shape(Shape::cuboid(v(1.0, 0.1, 2.0))),
    ];
    let mut n = 400;
    let mut k = 0u64;
    while n < 5000 {
        k += 1;
        let x = ((k * 7919) % 1500) as f64 / 10.0 - 75.0;
        let z = ((k * 104729) % 1500) as f64 / 10.0 - 75.0;
        if x.abs() < 6.0 && z.abs() < 6.0 {
            continue;
        }
        let s = shapes[(k % 5) as usize];
        let rot = if k % 3 == 0 { Quat::from_yaw(Fx::from_ratio((k % 628) as i64, 100)) } else { Quat::IDENTITY };
        let y = if k % 5 == 4 {
            0.4
        } else if k % 5 == 1 {
            0.5
        } else {
            1.5
        };
        w.create_body(&BodyDesc { rotation: rot, ..BodyDesc::new_static(s, v(x, y, z)) }).unwrap();
        n += 1;
    }
    // Doors: hinged slabs that stop on players.
    let door_shape = w.add_shape(Shape::cuboid(v(0.6, 1.1, 0.05)));
    let mut doors = Vec::new();
    for i in 0..20 {
        let x = -60.0 + (i % 5) as f64 * 30.0;
        let z = -45.0 + (i / 5) as f64 * 30.0;
        doors.push(
            w.create_door(&DoorDesc { shape: door_shape, pivot: v(x - 0.6, 1.1, z), offset: v(0.6, 0.0, 0.0), ..Default::default() })
                .unwrap(),
        );
    }
    let mut players = Vec::new();
    for i in 0..16 {
        let a = i as f64 / 16.0 * std::f64::consts::TAU;
        players.push(w.create_character(&CharacterDesc { position: v(a.cos() * 4.0, 1.0, a.sin() * 4.0), ..Default::default() }).unwrap());
    }
    w.step();
    let build = t0.elapsed();
    println!(
        "statics {} doors {} players {}  threads {}  build {:.1} ms",
        n,
        doors.len(),
        players.len(),
        w.threads(),
        build.as_secs_f64() * 1000.0
    );

    let ticks: usize = std::env::var("TICKS").ok().and_then(|s| s.parse().ok()).unwrap_or(1200);
    let mut times = Vec::with_capacity(ticks);
    let mut ray_ns = 0u128;
    let mut rays = Vec::with_capacity(32);
    let mut hits = vec![None; 32];
    let mut hash = 0u64;
    for t in 0..ticks {
        for (i, d) in doors.iter().enumerate() {
            w.set_door_open(*d, ((t + i * 13) / 120) % 2 == 0);
        }
        for (i, c) in players.iter().enumerate() {
            let yaw = Fx::from_ratio(((t / 40 + i * 37) % 628) as i64, 100);
            w.set_character_input(
                *c,
                CharacterInput { move_z: Fx::ONE, yaw, jump: (t + i * 7) % 90 == 0, crouch: (t / 200 + i) % 5 == 0, ..Default::default() },
            );
        }
        let s = Instant::now();
        w.step();
        times.push(s.elapsed().as_nanos());
        // 16 players x 2 shots per tick, cast as one batch.
        rays.clear();
        for (i, c) in players.iter().enumerate() {
            let ch = w.character(*c).unwrap();
            let (sn, cs) = Fx::from_ratio(((t * 3 + i * 50) % 628) as i64, 100).sin_cos();
            let filter = QueryFilter::default().excluding(ch.body());
            let origin = ch.position + Vec3::new(Fx::ZERO, fx!(0.7), Fx::ZERO);
            for k in 0..2 {
                rays.push(RayQuery { origin, dir: Vec3::new(sn, f(-0.02 - 0.01 * k as f64), cs), max_dist: f(150.0), filter });
            }
        }
        let s = Instant::now();
        w.raycast_batch(&rays, &mut hits);
        ray_ns += s.elapsed().as_nanos();
        for h in &hits {
            hash = hash.rotate_left(5) ^ h.map_or(0, |h| h.distance.0 as u64);
        }
    }
    times.sort_unstable();
    let avg = times.iter().sum::<u128>() as f64 / ticks as f64 / 1000.0;
    println!(
        "step avg {avg:.1} us  p50 {:.1} us  p99 {:.1} us  max {:.1} us   raycast {:.0} ns",
        times[ticks / 2] as f64 / 1000.0,
        times[ticks * 99 / 100] as f64 / 1000.0,
        times[ticks - 1] as f64 / 1000.0,
        ray_ns as f64 / (ticks * 32) as f64
    );
    println!("hash {:016x}  rays {:016x}", w.state_hash(), hash);
    let s = Instant::now();
    for _ in 0..2000 {
        w.raycast_batch(&rays, &mut hits);
    }
    println!("raycast_batch, warm caches: {:.0} ns/ray", s.elapsed().as_nanos() as f64 / 2000.0 / rays.len() as f64);
}
