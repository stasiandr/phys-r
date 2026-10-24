using System;
using System.Numerics;
using PhysR;

static class Test {
    static int fails = 0;
    static void Check(bool c, string m) { if (!c) { fails++; Console.WriteLine("FAIL: " + m); } }
    static void Main() {
        Console.WriteLine("native version " + PhysRNative.pr_version());
        var rng = new Random(42);
        // Mul/Div exactness vs native.
        for (int i = 0; i < 200000; i++) {
            long a = NextLong(rng, i), b = NextLong(rng, i * 7 + 3);
            long m = FP.MulRaw(a, b), mn = PhysRNative.pr_fx_mul(a, b);
            if (m != mn) { Check(false, "mul " + a + " * " + b + " = " + m + " native " + mn); if (fails > 10) return; }
            long d = FP.DivRaw(a, b), dn = PhysRNative.pr_fx_div(a, b);
            if (d != dn) { Check(false, "div " + a + " / " + b + " = " + d + " native " + dn); if (fails > 10) return; }
        }
        // Dot vs BigInteger reference.
        for (int i = 0; i < 20000; i++) {
            var v = new FPVector3(FP.FromRaw(NextLong(rng, 1)), FP.FromRaw(NextLong(rng, 2)), FP.FromRaw(NextLong(rng, 3)));
            var w = new FPVector3(FP.FromRaw(NextLong(rng, 4)), FP.FromRaw(NextLong(rng, 5)), FP.FromRaw(NextLong(rng, 6)));
            BigInteger s = (BigInteger)v.X.Raw * w.X.Raw + (BigInteger)v.Y.Raw * w.Y.Raw + (BigInteger)v.Z.Raw * w.Z.Raw;
            long refv = (long)(ulong)((s + (1 << 23)) >> 24 & ulong.MaxValue);
            Check(FPVector3.Dot(v, w).Raw == refv, "dot");
        }
        Check(FP.FromFloat(1.5f).Raw == 3L << 23, "fromfloat");
        Check(FP.Sqrt(FP.FromInt(4)) == FP.FromInt(2), "sqrt");
        Check(FP.Pi.Raw == PhysRNative.pr_fx_atan2(0, -FP.OneRaw), "pi");

        // World round trip.
        using (var world = new PhysWorld()) {
            var g = world.AddBox(new FPVector3(50, 1, 50));
            var d = BodyDesc.Default; d.Kind = BodyKind.Static; d.Shape = g; d.Position = new FPVector3(0, -1, 0);
            Check(world.CreateBody(d).IsValid, "static body");
            var s = world.AddSphere(FP.Half);
            d = BodyDesc.Default; d.Shape = s; d.Position = new FPVector3(0, 3, 0); d.Flags = BodyFlags.ContactEvents;
            var ball = world.CreateBody(d);
            var cd = CharacterDesc.Default; cd.Position = new FPVector3(3, 1, 0);
            var ch = world.CreateCharacter(cd);
            Check(ch.IsValid, "character");
            var events = new PhysEvent[16];
            int began = 0;
            for (int i = 0; i < 120; i++) {
                world.SetInput(ch, new CharacterInput { MoveZ = FP.One, Yaw = FP.Zero });
                world.Step();
                int n = world.GetEvents(events);
                for (int k = 0; k < n && k < events.Length; k++) if (events[k].Kind == EventKind.ContactBegin) began++;
            }
            var st = world.GetState(ball);
            Check(Math.Abs(st.Position.Y.ToFloat() - 0.5f) < 0.02f, "ball rest " + st.Position);
            Check(began == 1, "contact begin " + began);
            var cs = world.GetState(ch);
            Check(cs.Grounded == 1 && cs.Position.Z > FP.FromInt(10), "char " + cs.Position);
            Check(FP.Abs(cs.Height - FP.FromRatio(18, 10)) <= FP.FromRaw(1) && cs.Collision.HasFlag(CollisionFlags.Below), "char state " + cs.Height + " " + cs.Collision);
            // Crouch shrinks the capsule; headshot zones via LocalPoint.
            for (int i = 0; i < 5; i++) { world.SetInput(ch, new CharacterInput { Buttons = CharacterButtons.Crouch }); world.Step(); }
            cs = world.GetState(ch);
            Check(cs.IsCrouched && FP.Abs(cs.Height - FP.FromRatio(12, 10)) <= FP.FromRaw(1), "crouch " + cs.Height);
            RayHit hh;
            var eye = new FPVector3(cs.Position.X, cs.Position.Y + FP.FromRatio(4, 10), cs.Position.Z - FP.FromInt(5));
            Check(world.Raycast(eye, FPVector3.Forward, FP.FromInt(20), out hh) && world.CharacterOfBody(hh.Body) == ch && hh.LocalPoint.Y > FP.FromRatio(3, 10), "hit zone " + hh.LocalPoint);
            Check(world.AddVelocity(ch, new FPVector3(0, 8, 0)) && world.SetMode(ch, MoveMode.Walk), "knockback");
            RayHit hit;
            Check(world.Raycast(new FPVector3(0, 5, 0), FPVector3.Down, FP.FromInt(100), out hit) && hit.Body == ball, "raycast");
            var batch = new RayQuery[] { new RayQuery(new FPVector3(0, 5, 0), FPVector3.Down, FP.FromInt(100), QueryFilter.All), new RayQuery(new FPVector3(0, 5, 0), FPVector3.Up, FP.FromInt(100), QueryFilter.All) };
            var batchHits = new RayHit[2];
            Check(world.RaycastBatch(batch, batchHits, 2) == 1 && batchHits[0].Body == ball && batchHits[0].Distance == hit.Distance && !batchHits[1].Body.IsValid, "raycast batch");
            var snap = world.SaveSnapshot();
            ulong h = world.StateHash();
            world.Step(30);
            world.LoadSnapshot(snap);
            Check(world.StateHash() == h, "snapshot hash");
            int len = world.SaveSnapshotNoAlloc();
            Check(len == snap.Length, "noalloc len");
            var ids = new ulong[] { ball.Value, 0 };
            var tf = new TransformF[2];
            Check(world.GetTransforms(ids, tf, 2) == 1 && tf[0].Valid == 1 && tf[1].Valid == 0, "transforms");
            var buf = new ulong[8];
            Check(world.OverlapSphere(new FPVector3(0, 0, 0), FP.FromInt(1), QueryFilter.All, buf) >= 2, "overlap");
            var q = FPQuaternion.FromYaw(FP.HalfPi);
            var f = q * FPVector3.Forward;
            Check(FP.Abs(f.X - FP.One) < FP.FromRatio(1, 10000), "yaw rotate " + f);
            // Doors: hinge opens to exactly its open pose; state round-trips via snapshot.
            var dd = DoorDesc.Default;
            dd.Shape = world.AddBox(new FPVector3(FP.FromRatio(6, 10), FP.FromRatio(11, 10), FP.FromRatio(5, 100)));
            dd.Pivot = new FPVector3(10, 2, 10);
            dd.Offset = new FPVector3(FP.FromRatio(6, 10), FP.Zero, FP.Zero);
            var door = world.CreateDoor(dd);
            Check(door.IsValid, "door create");
            world.SetDoorOpen(door, true);
            var dsnap = world.SaveSnapshot();
            world.Step(60);
            var ds = world.GetState(door);
            Check(ds.Moving == 0 && ds.OpenFraction == FP.One, "door open " + ds.OpenFraction);
            ulong dh = world.StateHash();
            world.LoadSnapshot(dsnap);
            world.Step(60);
            Check(world.StateHash() == dh, "door rollback");
            Console.WriteLine("hash " + h.ToString("x16"));
        }
        Console.WriteLine(fails == 0 ? "ALL OK" : ("FAILURES: " + fails));
        Environment.Exit(fails == 0 ? 0 : 1);
    }
    static long NextLong(Random r, int salt) {
        var b = new byte[8]; r.NextBytes(b);
        long v = BitConverter.ToInt64(b, 0);
        switch ((salt & 0x7fffffff) % 5) {
            case 0: return v;                         // full range
            case 1: return v >> 20;                   // big values
            case 2: return v >> 36;                   // typical (< 2^28)
            case 3: return v >> 50;                   // small
            default: return (v >> 40) | 1;            // non-zero mid
        }
    }
}
