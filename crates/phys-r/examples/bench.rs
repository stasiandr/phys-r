//! Benchmark of a typical small-FPS server tick:
//! tiled box level + cover + crates/props + 16 characters + hitscan traffic.
//!
//! cargo run --release -p phys-r --example bench

use phys_r::*;
use std::time::Instant;

fn f(v: f64) -> Fx {
    Fx::from_f64(v)
}

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(f(x), f(y), f(z))
}

fn build(props: usize, players: usize) -> (World, Vec<CharacterHandle>) {
    let mut w = World::new(WorldSettings { enable_sleep: false, ..Default::default() });
    // Level: 32x32 floor tiles (4 m) with small height steps, plus cover.
    let tile = w.add_shape(Shape::cuboid(v(2.0, 1.0, 2.0)));
    for i in 0..32 {
        for j in 0..32 {
            let y = ((i * 7 + j * 13) % 4) as f64 * 0.05 - 1.0;
            w.create_body(&BodyDesc::new_static(tile, v(-64.0 + 2.0 + i as f64 * 4.0, y, -64.0 + 2.0 + j as f64 * 4.0))).unwrap();
        }
    }
    let cover = w.add_shape(Shape::cuboid(v(0.6, 0.6, 0.6)));
    for i in 0..300 {
        let x = ((i * 37) % 120) as f64 - 60.0;
        let z = ((i * 53) % 120) as f64 - 60.0;
        if x.abs() < 8.0 && z.abs() < 8.0 {
            continue;
        }
        w.create_body(&BodyDesc::new_static(cover, v(x, 0.6, z))).unwrap();
    }
    // Some walls / cover.
    let wall = w.add_shape(Shape::cuboid(v(4.0, 2.0, 0.3)));
    for i in 0..40 {
        let a = i as f64 * 0.7;
        w.create_body(&BodyDesc {
            rotation: Quat::from_yaw(f(a)),
            ..BodyDesc::new_static(wall, v((i % 8) as f64 * 12.0 - 42.0, 2.0, (i / 8) as f64 * 14.0 - 28.0))
        })
        .unwrap();
    }
    let crate_s = w.add_shape(Shape::cuboid(v(0.4, 0.4, 0.4)));
    let barrel = w.add_shape(Shape::capsule(f(0.3), f(0.3)));
    let ball = w.add_shape(Shape::sphere(f(0.25)));
    for i in 0..props {
        let s = [crate_s, barrel, ball][i % 3];
        let x = (i % 10) as f64 * 1.1 - 5.0;
        let z = ((i / 10) % 10) as f64 * 1.1 - 5.0;
        let y = 1.0 + (i / 100) as f64 * 1.0;
        w.create_body(&BodyDesc::new_dynamic(s, v(x, y, z), f(10.0))).unwrap();
    }
    let mut chars = Vec::new();
    for i in 0..players {
        let a = i as f64 / players as f64 * std::f64::consts::TAU;
        chars.push(w.create_character(&CharacterDesc { position: v(a.cos() * 20.0, 1.5, a.sin() * 20.0), ..Default::default() }).unwrap());
    }
    (w, chars)
}

fn run(props: usize, players: usize, rays_per_tick: usize, ticks: usize) {
    let (mut w, chars) = build(props, players);
    // Warm up.
    for _ in 0..60 {
        w.step();
    }
    let mut step_ns = Vec::with_capacity(ticks);
    let mut ray_ns = 0u128;
    let mut hits = 0;
    for t in 0..ticks {
        for (i, c) in chars.iter().enumerate() {
            let yaw = Fx::from_ratio(((t * 3 + i * 37) % 628) as i64, 100);
            w.set_character_input(
                *c,
                CharacterInput {
                    move_z: Fx::ONE,
                    move_x: Fx::from_ratio((i % 3) as i64 - 1, 1),
                    yaw,
                    jump: t % 50 == i,
                    ..Default::default()
                },
            );
        }
        let s = Instant::now();
        w.step();
        step_ns.push(s.elapsed().as_nanos());
        let s = Instant::now();
        for r in 0..rays_per_tick {
            let a = Fx::from_ratio(((r * 97 + t) % 628) as i64, 100);
            let (sn, cs) = a.sin_cos();
            let o = v(0.0, 1.5, 0.0);
            if w.raycast(o, Vec3::new(sn, f(-0.05), cs), f(200.0), &QueryFilter::default()).is_some() {
                hits += 1;
            }
        }
        ray_ns += s.elapsed().as_nanos();
    }
    if std::env::var("STATS").is_ok() {
        let pts: usize = w.manifolds().iter().map(|m| m.count as usize).sum();
        println!("manifolds {} points {}", w.manifolds().len(), pts);
    }
    step_ns.sort_unstable();
    let avg = step_ns.iter().sum::<u128>() as f64 / ticks as f64 / 1000.0;
    let p99 = step_ns[ticks * 99 / 100] as f64 / 1000.0;
    let ray = ray_ns as f64 / (ticks * rays_per_tick.max(1)) as f64;
    println!(
        "props={props:4} players={players:3}  step avg {avg:8.1} us  p99 {p99:8.1} us   raycast {ray:6.0} ns  (hits {hits})  hash {:016x}",
        w.state_hash()
    );
}

fn main() {
    if let Ok(spec) = std::env::var("BENCH") {
        // e.g. BENCH=200,16,0,100 (props, players, rays, ticks)
        let a: Vec<usize> = spec.split(',').map(|x| x.parse().unwrap()).collect();
        run(a[0], a[1], a[2], a[3]);
        return;
    }
    run(0, 16, 64, 600);
    run(50, 16, 64, 600);
    run(200, 16, 64, 600);
    run(200, 32, 128, 600);
    run(500, 32, 128, 300);
}
