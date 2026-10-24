use phys_r::*;

fn f(v: f64) -> Fx {
    Fx::from_f64(v)
}

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(f(x), f(y), f(z))
}

/// Flat ground box with its top face at y = 0.
fn ground(w: &mut World, half: f64) -> BodyHandle {
    let s = w.add_shape(Shape::cuboid(v(half, 1.0, half)));
    w.create_body(&BodyDesc::new_static(s, v(0.0, -1.0, 0.0))).unwrap()
}

fn ground_box(w: &mut World) -> BodyHandle {
    let s = w.add_shape(Shape::cuboid(v(50.0, 1.0, 50.0)));
    w.create_body(&BodyDesc::new_static(s, v(0.0, -1.0, 0.0))).unwrap()
}

#[test]
fn sphere_rests_on_ground_and_sleeps() {
    let mut w = World::new(WorldSettings::default());
    ground_box(&mut w);
    let s = w.add_shape(Shape::sphere(f(0.5)));
    let b = w.create_body(&BodyDesc::new_dynamic(s, v(0.0, 3.0, 0.0), f(1.0))).unwrap();
    for _ in 0..240 {
        w.step();
    }
    let body = w.body(b).unwrap();
    let y = body.position.y.to_f64();
    assert!((y - 0.5).abs() < 0.02, "y = {y}");
    assert!(body.is_sleeping(), "should sleep");
}

#[test]
fn sphere_rests_on_mesh() {
    let mut w = World::new(WorldSettings::default());
    ground(&mut w, 20.0);
    let s = w.add_shape(Shape::sphere(f(0.5)));
    let b = w.create_body(&BodyDesc::new_dynamic(s, v(0.3, 2.0, 0.2), f(1.0))).unwrap();
    for _ in 0..180 {
        w.step();
    }
    let y = w.body(b).unwrap().position.y.to_f64();
    assert!((y - 0.5).abs() < 0.02, "y = {y}");
}

#[test]
fn box_stack_is_stable() {
    let mut w = World::new(WorldSettings::default());
    ground_box(&mut w);
    let s = w.add_shape(Shape::cuboid(v(0.5, 0.5, 0.5)));
    let mut boxes = Vec::new();
    for i in 0..6 {
        boxes.push(w.create_body(&BodyDesc::new_dynamic(s, v(0.0, 0.5 + i as f64 * 1.0, 0.0), f(1.0))).unwrap());
    }
    for _ in 0..600 {
        w.step();
    }
    for (i, b) in boxes.iter().enumerate() {
        let p = w.body(*b).unwrap().position;
        let ey = 0.5 + i as f64;
        assert!((p.y.to_f64() - ey).abs() < 0.06, "box {i} y={:?}", p.y);
        assert!(p.x.to_f64().abs() < 0.05 && p.z.to_f64().abs() < 0.05, "box {i} drifted {p:?}");
    }
}

#[test]
fn box_rests_flat_on_mesh() {
    let mut w = World::new(WorldSettings::default());
    ground(&mut w, 20.0);
    let s = w.add_shape(Shape::cuboid(v(0.5, 0.25, 0.5)));
    let b = w
        .create_body(&BodyDesc { rotation: Quat::from_euler(f(0.3), f(0.5), f(0.2)), ..BodyDesc::new_dynamic(s, v(1.0, 2.0, 1.0), f(2.0)) })
        .unwrap();
    for _ in 0..300 {
        w.step();
    }
    let body = w.body(b).unwrap();
    let y = body.position.y.to_f64();
    assert!((y - 0.25).abs() < 0.03, "y = {y}");
    let up = body.rotation.rotate(Vec3::Y);
    assert!(up.y.abs().to_f64() > 0.99, "not flat: {up:?}");
}

#[test]
fn capsule_falls_over_and_rests() {
    let mut w = World::new(WorldSettings::default());
    ground_box(&mut w);
    let s = w.add_shape(Shape::capsule(f(0.5), f(0.25)));
    let b = w
        .create_body(&BodyDesc { rotation: Quat::from_euler(f(0.0), f(0.0), f(1.2)), ..BodyDesc::new_dynamic(s, v(0.0, 1.5, 0.0), f(1.0)) })
        .unwrap();
    for _ in 0..400 {
        w.step();
    }
    let y = w.body(b).unwrap().position.y.to_f64();
    assert!((y - 0.25).abs() < 0.03, "y = {y}");
}

#[test]
fn restitution_bounces() {
    let mut w = World::new(WorldSettings::default());
    ground_box(&mut w);
    let s = w.add_shape(Shape::sphere(f(0.5)));
    let b = w.create_body(&BodyDesc { restitution: f(0.8), ..BodyDesc::new_dynamic(s, v(0.0, 5.0, 0.0), f(1.0)) }).unwrap();
    let mut max_after_bounce = 0.0f64;
    let mut bounced = false;
    for _ in 0..240 {
        w.step();
        let body = w.body(b).unwrap();
        if body.linear_velocity.y > Fx::ZERO {
            bounced = true;
        }
        if bounced {
            max_after_bounce = max_after_bounce.max(body.position.y.to_f64());
        }
    }
    assert!(bounced);
    assert!(max_after_bounce > 2.0, "bounce height {max_after_bounce}");
}

fn build_scene(w: &mut World) -> Vec<BodyHandle> {
    ground(w, 30.0);
    let sb = w.add_shape(Shape::cuboid(v(0.4, 0.4, 0.4)));
    let ss = w.add_shape(Shape::sphere(f(0.35)));
    let sc = w.add_shape(Shape::capsule(f(0.3), f(0.2)));
    let mut out = Vec::new();
    for i in 0..60 {
        let x = (i % 5) as f64 * 0.9 - 2.0;
        let z = ((i / 5) % 4) as f64 * 0.9 - 1.5;
        let y = 1.0 + (i / 20) as f64 * 1.2 + (i % 3) as f64 * 0.05;
        let shape = [sb, ss, sc][i % 3];
        let d = BodyDesc {
            rotation: Quat::from_euler(f(i as f64 * 0.1), f(i as f64 * 0.37), f(0.0)),
            linear_velocity: v((i % 7) as f64 * 0.1 - 0.3, 0.0, 0.0),
            ..BodyDesc::new_dynamic(shape, v(x, y, z), f(1.0 + (i % 4) as f64))
        };
        out.push(w.create_body(&d).unwrap());
    }
    out
}

#[test]
fn deterministic_across_runs() {
    let mut a = World::new(WorldSettings::default());
    let mut b = World::new(WorldSettings::default());
    build_scene(&mut a);
    build_scene(&mut b);
    for t in 0..300 {
        a.step();
        b.step();
        if t % 50 == 0 {
            assert_eq!(a.state_hash(), b.state_hash(), "diverged at tick {t}");
        }
    }
    assert_eq!(a.state_hash(), b.state_hash());
}

#[test]
fn snapshot_rollback_reproduces_future() {
    let mut w = World::new(WorldSettings::default());
    build_scene(&mut w);
    for _ in 0..90 {
        w.step();
    }
    let mut snap = Vec::new();
    w.save_snapshot(&mut snap);
    for _ in 0..120 {
        w.step();
    }
    let expected = w.state_hash();

    // Restore into the same world.
    w.load_snapshot(&snap).unwrap();
    for _ in 0..120 {
        w.step();
    }
    assert_eq!(w.state_hash(), expected);

    // Restore into a fresh world with the same shapes registered.
    let mut fresh = World::new(WorldSettings::default());
    build_scene(&mut fresh);
    fresh.load_snapshot(&snap).unwrap();
    for _ in 0..120 {
        fresh.step();
    }
    assert_eq!(fresh.state_hash(), expected);
}

#[test]
fn snapshot_rejects_garbage() {
    let mut w = World::new(WorldSettings::default());
    assert!(w.load_snapshot(&[1, 2, 3]).is_err());
    let mut snap = Vec::new();
    build_scene(&mut w);
    w.save_snapshot(&mut snap);
    snap.truncate(snap.len() / 2);
    assert!(w.load_snapshot(&snap).is_err());
}

#[test]
fn raycasts_hit_correct_surfaces() {
    let mut w = World::new(WorldSettings::default());
    ground(&mut w, 10.0);
    let sb = w.add_shape(Shape::cuboid(v(1.0, 1.0, 1.0)));
    let bx = w.create_body(&BodyDesc::new_static(sb, v(0.0, 1.0, 5.0))).unwrap();
    let ss = w.add_shape(Shape::sphere(f(0.5)));
    let sp = w.create_body(&BodyDesc::new_kinematic(ss, v(3.0, 1.0, 0.0))).unwrap();
    let sc = w.add_shape(Shape::capsule(f(0.5), f(0.3)));
    let cp = w.create_body(&BodyDesc::new_kinematic(sc, v(-3.0, 1.0, 0.0))).unwrap();
    w.step();

    let q = QueryFilter::default();
    let h = w.raycast(v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0), f(100.0), &q).unwrap();
    assert_eq!(h.body, bx);
    assert!((h.distance.to_f64() - 4.0).abs() < 1e-4, "{:?}", h.distance);
    assert!((h.normal - v(0.0, 0.0, -1.0)).length() < f(1e-4));

    let h = w.raycast(v(0.0, 1.0, 0.0), v(1.0, 0.0, 0.0), f(100.0), &q).unwrap();
    assert_eq!(h.body, sp);
    assert!((h.distance.to_f64() - 2.5).abs() < 1e-4);

    let h = w.raycast(v(0.0, 1.2, 0.0), v(-1.0, 0.0, 0.0), f(100.0), &q).unwrap();
    assert_eq!(h.body, cp);
    assert!((h.distance.to_f64() - 2.7).abs() < 1e-4, "{:?}", h.distance);

    let h = w.raycast(v(2.0, 5.0, -2.0), v(0.0, -1.0, 0.0), f(100.0), &q).unwrap();
    assert!((h.distance.to_f64() - 5.0).abs() < 1e-4);
    assert!((h.normal - Vec3::Y).length() < f(1e-4));

    // Mask filtering & exclusion.
    let q2 = QueryFilter::default().excluding(sp);
    let h = w.raycast(v(0.0, 1.0, 0.0), v(1.0, 0.0, 0.0), f(100.0), &q2);
    assert!(h.is_none_or(|h| h.body != sp));
    assert!(w.raycast(v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0), f(100.0), &QueryFilter::mask(0)).is_none());
}

#[test]
fn overlap_and_sweeps() {
    let mut w = World::new(WorldSettings::default());
    ground_box(&mut w);
    let sb = w.add_shape(Shape::cuboid(v(0.5, 0.5, 0.5)));
    let b = w.create_body(&BodyDesc::new_static(sb, v(0.0, 0.5, 4.0))).unwrap();
    w.step();
    let mut out = Vec::new();
    w.overlap_sphere(v(0.0, 0.5, 3.2), f(0.4), &QueryFilter::default(), &mut out);
    assert!(out.contains(&b));
    out.clear();
    w.overlap_sphere(v(0.0, 0.5, 2.0), f(0.4), &QueryFilter::default(), &mut out);
    assert!(!out.contains(&b));

    let hit = w.sphere_cast(v(0.0, 0.5, 0.0), f(0.25), v(0.0, 0.0, 10.0), &QueryFilter::default()).unwrap();
    assert_eq!(hit.body, b);
    // Contact when the centre reaches z = 4 - 0.5 - 0.25 = 3.25.
    let z = hit.fraction.to_f64() * 10.0;
    assert!((z - 3.25).abs() < 0.01, "z = {z}");
    assert!((hit.normal - v(0.0, 0.0, -1.0)).length() < f(1e-3));
}

#[test]
fn sensors_report_enter_and_exit() {
    let mut w = World::new(WorldSettings::default());
    ground_box(&mut w);
    let st = w.add_shape(Shape::cuboid(v(1.0, 1.0, 1.0)));
    let trig = w.create_body(&BodyDesc { flags: flags::SENSOR, ..BodyDesc::new_static(st, v(0.0, 1.0, 0.0)) }).unwrap();
    let ss = w.add_shape(Shape::sphere(f(0.25)));
    let ball = w.create_body(&BodyDesc::new_dynamic(ss, v(0.0, 5.0, 0.0), f(1.0))).unwrap();
    let mut entered = false;
    for _ in 0..120 {
        w.step();
        for e in w.events() {
            if e.kind == EventKind::SensorEnter && e.a == trig && e.b == ball {
                entered = true;
            }
        }
    }
    assert!(entered);
    w.set_pose(ball, v(10.0, 0.25, 0.0), Quat::IDENTITY);
    w.step();
    assert!(w.events().iter().any(|e| e.kind == EventKind::SensorExit && e.a == trig && e.b == ball));
}

#[test]
fn contact_events() {
    let mut w = World::new(WorldSettings::default());
    let g = ground_box(&mut w);
    let ss = w.add_shape(Shape::sphere(f(0.5)));
    let _ball = w.create_body(&BodyDesc { flags: flags::CONTACT_EVENTS, ..BodyDesc::new_dynamic(ss, v(0.0, 2.0, 0.0), f(1.0)) }).unwrap();
    let mut began = 0;
    for _ in 0..120 {
        w.step();
        began += w.events().iter().filter(|e| e.kind == EventKind::ContactBegin).count();
    }
    assert_eq!(began, 1);
    let _ = g;
}

fn fps_world() -> World {
    let mut w = World::new(WorldSettings::default());
    ground(&mut w, 50.0);
    w
}

#[test]
fn character_walks_and_is_blocked_by_wall() {
    let mut w = fps_world();
    let wall = w.add_shape(Shape::cuboid(v(5.0, 2.0, 0.25)));
    w.create_body(&BodyDesc::new_static(wall, v(0.0, 2.0, 5.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    // Settle.
    for _ in 0..10 {
        w.step();
    }
    assert!(w.character(c).unwrap().grounded);
    let input = CharacterInput { move_z: Fx::ONE, ..Default::default() };
    for _ in 0..180 {
        w.set_character_input(c, input);
        w.step();
    }
    let ch = w.character(c).unwrap();
    // Wall front face is at z = 4.75, capsule radius 0.4.
    let z = ch.position.z.to_f64();
    assert!(z < 4.36 && z > 4.2, "z = {z}");
    assert!(ch.grounded);
    assert!((ch.feet().y.to_f64()).abs() < 0.05, "feet {:?}", ch.feet().y);
}

#[test]
fn character_slides_along_wall() {
    let mut w = fps_world();
    let wall = w.add_shape(Shape::cuboid(v(20.0, 2.0, 0.25)));
    w.create_body(&BodyDesc::new_static(wall, v(0.0, 2.0, 2.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    let yaw = Fx::from_f64(std::f64::consts::FRAC_PI_4); // diagonal into the wall
    for _ in 0..120 {
        w.set_character_input(c, CharacterInput { move_z: Fx::ONE, yaw, ..Default::default() });
        w.step();
    }
    let p = w.character(c).unwrap().position;
    assert!(p.x.to_f64() > 3.0, "did not slide: {p:?}");
    assert!(p.z.to_f64() < 1.36, "{p:?}");
}

#[test]
fn character_climbs_steps_but_not_walls() {
    let mut w = fps_world();
    let step = w.add_shape(Shape::cuboid(v(2.0, 0.15, 0.5)));
    for i in 0..4 {
        w.create_body(&BodyDesc::new_static(step, v(0.0, 0.15 + 0.3 * i as f64, 3.0 + i as f64 * 1.0))).unwrap();
    }
    // Make steps "solid" below each tread.
    let fill = w.add_shape(Shape::cuboid(v(2.0, 0.5, 2.5)));
    w.create_body(&BodyDesc::new_static(fill, v(0.0, -0.5 + 0.15, 5.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    let mut peak = 0.0f64;
    let mut airborne = 0;
    for _ in 0..70 {
        w.set_character_input(c, CharacterInput { move_z: Fx::ONE, ..Default::default() });
        w.step();
        let ch = w.character(c).unwrap();
        peak = peak.max(ch.feet().y.to_f64());
        if !ch.grounded {
            airborne += 1;
        }
    }
    let ch = w.character(c).unwrap();
    assert!(ch.position.z.to_f64() > 6.0, "stuck at {:?}", ch.position);
    assert!((peak - 1.2).abs() < 0.02, "did not climb: peak {peak}");
    assert!(airborne <= 2, "airborne for {airborne} ticks on stairs");

    // A tall block cannot be stepped onto.
    let mut w = fps_world();
    let block = w.add_shape(Shape::cuboid(v(2.0, 0.5, 0.5)));
    w.create_body(&BodyDesc::new_static(block, v(0.0, 0.5, 3.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    for _ in 0..180 {
        w.set_character_input(c, CharacterInput { move_z: Fx::ONE, ..Default::default() });
        w.step();
    }
    assert!(w.character(c).unwrap().position.z.to_f64() < 2.2);
}

#[test]
fn character_jumps_and_lands() {
    let mut w = fps_world();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    for _ in 0..5 {
        w.step();
    }
    w.set_character_input(c, CharacterInput { jump: true, ..Default::default() });
    w.step();
    let mut peak = 0.0f64;
    for _ in 0..90 {
        w.set_character_input(c, CharacterInput { jump: true, ..Default::default() });
        w.step();
        peak = peak.max(w.character(c).unwrap().feet().y.to_f64());
    }
    // v^2 / 2g = 49 / 40 = 1.225
    assert!((peak - 1.225).abs() < 0.1, "peak {peak}");
    let ch = w.character(c).unwrap();
    assert!(ch.grounded, "holding jump must not re-jump (requires release)");
    assert!(ch.feet().y.to_f64().abs() < 0.05);
}

#[test]
fn character_walks_down_slope_and_stays_grounded() {
    let mut w = World::new(WorldSettings::default());
    // 20 degree ramp descending along +Z.
    let a = (20.0f64).to_radians();
    let len = 20.0;
    let drop = len * a.tan();
    let _ = (len, drop);
    // Ramp = box rotated about X; its top normal tilts toward +Z.
    let s = w.add_shape(Shape::cuboid(v(5.0, 0.5, 12.0)));
    w.create_body(&BodyDesc { rotation: Quat::from_axis_angle(Vec3::X, f(a)), ..BodyDesc::new_static(s, v(0.0, -3.0, 10.0)) }).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 2.7, 1.0), ..Default::default() }).unwrap();
    for _ in 0..40 {
        w.step();
    }
    let mut airborne = 0;
    for _ in 0..120 {
        w.set_character_input(c, CharacterInput { move_z: Fx::ONE, ..Default::default() });
        w.step();
        if !w.character(c).unwrap().grounded {
            airborne += 1;
        }
    }
    assert!(airborne <= 1, "lost ground {airborne} ticks");
}

#[test]
fn character_pushes_prop_and_blocks_other_character() {
    let mut w = fps_world();
    let a = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    let b = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 3.0), ..Default::default() }).unwrap();
    for _ in 0..120 {
        w.set_character_input(a, CharacterInput { move_z: Fx::ONE, ..Default::default() });
        w.step();
    }
    let za = w.character(a).unwrap().position.z.to_f64();
    let zb = w.character(b).unwrap().position.z.to_f64();
    assert!(zb - za > 0.79, "overlapping characters: {za} {zb}");
    assert!((zb - 3.0).abs() < 1e-6, "kinematic characters are not pushed");
}

#[test]
fn lag_compensated_raycast() {
    let mut w = fps_world();
    let target = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 10.0), ..Default::default() }).unwrap();
    let tb = w.character(target).unwrap().body();
    for _ in 0..5 {
        w.step();
    }
    let shot_tick = w.tick();
    // Target strafes away for 30 ticks.
    for _ in 0..30 {
        w.set_character_input(target, CharacterInput { move_x: Fx::ONE, ..Default::default() });
        w.step();
    }
    let q = QueryFilter::default();
    let origin = v(0.0, 1.0, 0.0);
    let dir = v(0.0, 0.0, 1.0);
    let now = w.raycast(origin, dir, f(100.0), &q);
    assert!(now.is_none_or(|h| h.body != tb), "target moved away");
    let past = w.raycast_rewind(shot_tick, Fx::ZERO, origin, dir, f(100.0), &q).unwrap();
    assert_eq!(past.body, tb);
    assert!((past.distance.to_f64() - 9.6).abs() < 0.02, "{:?}", past.distance);
}

#[test]
fn prediction_replay_matches() {
    // Client-side prediction: replaying the same inputs from a snapshot via
    // `simulate_character` gives exactly the same result.
    let mut w = fps_world();
    let wall = w.add_shape(Shape::cuboid(v(3.0, 2.0, 0.25)));
    w.create_body(&BodyDesc::new_static(wall, v(1.0, 2.0, 4.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    let dt = w.settings.dt;
    let inputs: Vec<CharacterInput> = (0..120)
        .map(|i| CharacterInput {
            move_x: Fx::from_ratio((i % 7) as i64 - 3, 3),
            move_z: Fx::ONE,
            yaw: Fx::from_ratio(i as i64, 40),
            jump: i % 40 == 10,
            ..Default::default()
        })
        .collect();
    let mut snap = Vec::new();
    w.save_snapshot(&mut snap);
    for inp in &inputs {
        w.simulate_character(c, *inp, dt);
    }
    let p1 = w.character(c).unwrap().position;
    w.load_snapshot(&snap).unwrap();
    for inp in &inputs {
        w.simulate_character(c, *inp, dt);
    }
    assert_eq!(w.character(c).unwrap().position, p1);
}

#[test]
fn box_holds_on_ramp_with_static_friction() {
    let mut w = World::new(WorldSettings::default());
    let a = 20.0f64.to_radians();
    let rot = Quat::from_axis_angle(Vec3::X, f(a));
    let rs = w.add_shape(Shape::cuboid(v(10.0, 0.5, 10.0)));
    w.create_body(&BodyDesc { rotation: rot, ..BodyDesc::new_static(rs, Vec3::ZERO) }).unwrap();
    let bs = w.add_shape(Shape::cuboid(v(0.5, 0.5, 0.5)));
    let start = rot.rotate(v(0.0, 1.0, 0.0));
    let b = w.create_body(&BodyDesc { rotation: rot, ..BodyDesc::new_dynamic(bs, start, f(5.0)) }).unwrap();
    for _ in 0..300 {
        w.step();
    }
    let p = w.body(b).unwrap().position;
    assert!((p - start).length() < f(0.05), "slid from {start:?} to {p:?}");

    // Frictionless body slides away.
    let mut w2 = World::new(WorldSettings::default());
    w2.add_shape(Shape::sphere(f(1.0)));
    let rs = w2.add_shape(Shape::cuboid(v(10.0, 0.5, 10.0)));
    w2.create_body(&BodyDesc { rotation: rot, friction: Fx::ZERO, ..BodyDesc::new_static(rs, Vec3::ZERO) }).unwrap();
    let bs = w2.add_shape(Shape::cuboid(v(0.5, 0.5, 0.5)));
    let b = w2.create_body(&BodyDesc { rotation: rot, friction: Fx::ZERO, ..BodyDesc::new_dynamic(bs, start, f(5.0)) }).unwrap();
    for _ in 0..60 {
        w2.step();
    }
    assert!((w2.body(b).unwrap().position - start).length() > f(0.5));
}

// ---------------------------------------------------------------------------
// FPS controller features
// ---------------------------------------------------------------------------

fn walk(w: &mut World, c: CharacterHandle, input: CharacterInput, ticks: usize) {
    for _ in 0..ticks {
        w.set_character_input(c, input);
        w.step();
    }
}

#[test]
fn crouch_under_ceiling_and_stand_up_after() {
    let mut w = fps_world();
    // Ceiling slab from z=3 to z=7 with its underside at y=1.5.
    let slab = w.add_shape(Shape::cuboid(v(3.0, 0.25, 2.0)));
    w.create_body(&BodyDesc::new_static(slab, v(0.0, 1.75, 5.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    walk(&mut w, c, CharacterInput::default(), 5);
    // Standing: blocked by the slab edge.
    walk(&mut w, c, CharacterInput { move_z: Fx::ONE, ..Default::default() }, 90);
    assert!(w.character(c).unwrap().position.z.to_f64() < 2.7);
    // Crouched: fits underneath.
    walk(&mut w, c, CharacterInput { move_z: Fx::ONE, crouch: true, ..Default::default() }, 50);
    let ch = w.character(c).unwrap();
    assert!(ch.crouched && ch.position.z.to_f64() > 4.0, "{:?}", ch.position);
    assert!(ch.feet().y.to_f64().abs() < 0.05);
    // Releasing crouch under the slab keeps us crouched.
    walk(&mut w, c, CharacterInput::default(), 5);
    let ch = w.character(c).unwrap();
    assert!(ch.crouched);
    assert!(ch.collision_flags & collision::STAND_BLOCKED != 0);
    // Walk out: stands up automatically.
    walk(&mut w, c, CharacterInput { move_z: Fx::ONE, ..Default::default() }, 60);
    let ch = w.character(c).unwrap();
    assert!(!ch.crouched && ch.position.z.to_f64() > 7.5, "{:?}", ch.position);
    assert!(ch.feet().y.to_f64().abs() < 0.05);
    // The body's capsule follows the crouch state.
    let body = w.body(ch.body()).unwrap();
    assert_eq!(w.shape(body.shape), Some(&Shape::capsule(f(0.5), f(0.4))));
}

#[test]
fn rides_moving_platform_and_keeps_momentum() {
    let mut w = fps_world();
    let ps = w.add_shape(Shape::cuboid(v(1.5, 0.25, 1.5)));
    let plat = w.create_body(&BodyDesc::new_kinematic(ps, v(0.0, 1.25, 10.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 2.45, 10.0), ..Default::default() }).unwrap();
    walk(&mut w, c, CharacterInput::default(), 10);
    assert!(w.character(c).unwrap().grounded);
    assert_eq!(w.character(c).unwrap().ground_body, plat);
    w.set_velocity(plat, v(3.0, 0.0, 0.0), Vec3::ZERO);
    walk(&mut w, c, CharacterInput::default(), 60);
    let px = w.body(plat).unwrap().position.x.to_f64();
    let ch = w.character(c).unwrap();
    assert!((px - 3.0).abs() < 0.01);
    assert!((ch.position.x.to_f64() - px).abs() < 0.06, "char {:?} plat {px}", ch.position);
    assert!(ch.grounded);
    // Elevator up.
    w.set_velocity(plat, v(0.0, 2.0, 0.0), Vec3::ZERO);
    walk(&mut w, c, CharacterInput::default(), 30);
    let top = w.body(plat).unwrap().position.y.to_f64() + 0.25;
    let ch = w.character(c).unwrap();
    assert!((ch.feet().y.to_f64() - top).abs() < 0.1, "feet {:?} top {top}", ch.feet().y);
    // Jumping off a sideways-moving platform inherits its velocity.
    w.set_velocity(plat, v(4.0, 0.0, 0.0), Vec3::ZERO);
    walk(&mut w, c, CharacterInput::default(), 5);
    walk(&mut w, c, CharacterInput { jump: true, ..Default::default() }, 1);
    let ch = w.character(c).unwrap();
    assert!(ch.collision_flags & collision::JUMPED != 0);
    assert!(ch.velocity.x.to_f64() > 3.5, "{:?}", ch.velocity);
}

#[test]
fn landing_reports_impact_speed() {
    let mut w = fps_world();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 4.9, 0.0), ..Default::default() }).unwrap();
    let mut landed = None;
    for _ in 0..120 {
        w.step();
        let ch = w.character(c).unwrap();
        if ch.collision_flags & collision::LANDED != 0 {
            landed = Some(ch.landing_speed.to_f64());
        }
    }
    // Fell 4 m with g = 20: v = sqrt(2 * 20 * 4) ~ 12.6
    let s = landed.expect("never landed");
    assert!((s - 12.6).abs() < 0.6, "landing speed {s}");
}

#[test]
fn coyote_time_allows_late_jump() {
    let mut w = World::new(WorldSettings::default());
    let s = w.add_shape(Shape::cuboid(v(2.0, 1.0, 2.0)));
    w.create_body(&BodyDesc::new_static(s, v(0.0, -1.0, 0.0))).unwrap();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    walk(&mut w, c, CharacterInput::default(), 5);
    // Run off the edge (z = 2) and press jump a few ticks after leaving it.
    let mut left_at = None;
    for t in 0..120 {
        let ch = w.character(c).unwrap();
        if left_at.is_none() && !ch.grounded {
            left_at = Some(t);
        }
        let jump = left_at.is_some_and(|l| t >= l + 2);
        w.set_character_input(c, CharacterInput { move_z: Fx::ONE, jump, ..Default::default() });
        w.step();
        if w.character(c).unwrap().collision_flags & collision::JUMPED != 0 {
            return;
        }
    }
    panic!("coyote jump did not trigger");
}

#[test]
fn fly_mode_and_knockback() {
    let mut w = fps_world();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 0.0), ..Default::default() }).unwrap();
    walk(&mut w, c, CharacterInput::default(), 5);
    // Knockback up launches us.
    w.add_character_velocity(c, v(0.0, 8.0, 0.0));
    walk(&mut w, c, CharacterInput::default(), 10);
    assert!(!w.character(c).unwrap().grounded);
    assert!(w.character(c).unwrap().feet().y.to_f64() > 0.5);
    walk(&mut w, c, CharacterInput::default(), 60);
    assert!(w.character(c).unwrap().grounded);
    // Fly (ladder): climb without gravity.
    w.set_character_mode(c, MoveMode::Fly);
    walk(&mut w, c, CharacterInput { move_y: Fx::ONE, ..Default::default() }, 60);
    let y = w.character(c).unwrap().feet().y.to_f64();
    assert!(y > 3.0, "{y}");
    walk(&mut w, c, CharacterInput::default(), 60);
    let y1 = w.character(c).unwrap().feet().y;
    walk(&mut w, c, CharacterInput::default(), 30);
    // No gravity in fly mode: friction brings us to a stop in mid-air.
    assert!(y1.to_f64() >= y && w.character(c).unwrap().feet().y == y1, "drifting in fly mode");
    w.set_character_mode(c, MoveMode::Walk);
    walk(&mut w, c, CharacterInput::default(), 90);
    assert!(w.character(c).unwrap().grounded);
}

#[test]
fn hitscan_hit_zones() {
    let mut w = fps_world();
    let c = w.create_character(&CharacterDesc { position: v(0.0, 0.95, 5.0), ..Default::default() }).unwrap();
    walk(&mut w, c, CharacterInput::default(), 5);
    let body = w.character(c).unwrap().body();
    let head = w.raycast(v(0.0, 1.65, 0.0), Vec3::Z, f(50.0), &QueryFilter::default()).unwrap();
    assert_eq!(head.body, body);
    assert_eq!(w.character_of_body(head.body), Some(c));
    assert!(head.local_point.y.to_f64() > 0.6, "{:?}", head.local_point);
    let legs = w.raycast(v(0.0, 0.4, 0.0), Vec3::Z, f(50.0), &QueryFilter::default()).unwrap();
    assert_eq!(legs.body, body);
    assert!(legs.local_point.y.to_f64() < -0.4);
}
