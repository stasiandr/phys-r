using System;

namespace PhysR
{
    /// <summary>
    /// Managed wrapper around a native phys-r world. Not thread safe: use each
    /// world from one thread. Engine-agnostic (no UnityEngine dependency), so
    /// it also runs on a headless .NET game server.
    /// </summary>
    public sealed class PhysWorld : IDisposable
    {
        IntPtr ptr;
        byte[] snapshotBuffer = new byte[0];

        public PhysWorld() : this(WorldSettings.Default) { }

        public PhysWorld(WorldSettings settings)
        {
            ptr = PhysRNative.pr_world_create(ref settings);
            if (ptr == IntPtr.Zero) throw new InvalidOperationException("phys-r: failed to create world");
            Dt = settings.Dt;
        }

        ~PhysWorld() { Dispose(); }

        public void Dispose()
        {
            if (ptr != IntPtr.Zero)
            {
                PhysRNative.pr_world_destroy(ptr);
                ptr = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        public bool IsDisposed { get { return ptr == IntPtr.Zero; } }
        public IntPtr NativePtr { get { return ptr; } }

        /// <summary>Fixed tick duration.</summary>
        public FP Dt { get; private set; }

        /// <summary>Number of completed ticks.</summary>
        public uint Tick { get { return PhysRNative.pr_world_tick(Ptr); } }

        IntPtr Ptr
        {
            get
            {
                if (ptr == IntPtr.Zero) throw new ObjectDisposedException("PhysWorld");
                return ptr;
            }
        }

        public void Step(int steps = 1) { PhysRNative.pr_world_step(Ptr, (uint)Math.Max(1, steps)); }

        public void SetGravity(FPVector3 g) { PhysRNative.pr_world_set_gravity(Ptr, ref g); }

        // ------------------------------------------------------------------
        // Shapes: convex primitives (configuration: register identically on
        // every peer). Build levels from boxes; static bodies live in a BVH.
        // ------------------------------------------------------------------

        public ShapeId AddSphere(FP radius) { return new ShapeId(PhysRNative.pr_shape_sphere(Ptr, radius.Raw)); }

        /// <summary>Y-axis capsule. halfHeight = half the inner segment length.</summary>
        public ShapeId AddCapsule(FP halfHeight, FP radius) { return new ShapeId(PhysRNative.pr_shape_capsule(Ptr, halfHeight.Raw, radius.Raw)); }

        public ShapeId AddBox(FPVector3 halfExtents) { return new ShapeId(PhysRNative.pr_shape_box(Ptr, ref halfExtents)); }

        public int ShapeCount { get { return (int)PhysRNative.pr_shape_count(Ptr); } }

        // ------------------------------------------------------------------
        // Bodies
        // ------------------------------------------------------------------

        public BodyHandle CreateBody(BodyDesc desc) { return new BodyHandle(PhysRNative.pr_body_create(Ptr, ref desc)); }
        public bool DestroyBody(BodyHandle h) { return PhysRNative.pr_body_destroy(Ptr, h.Value) != 0; }
        public bool IsValid(BodyHandle h) { return PhysRNative.pr_body_is_valid(Ptr, h.Value) != 0; }

        public bool TryGetState(BodyHandle h, out BodyState state) { return PhysRNative.pr_body_get_state(Ptr, h.Value, out state) != 0; }

        public BodyState GetState(BodyHandle h)
        {
            BodyState s;
            PhysRNative.pr_body_get_state(Ptr, h.Value, out s);
            return s;
        }

        /// <summary>Batch read of float transforms for rendering.</summary>
        public int GetTransforms(ulong[] handles, TransformF[] output, int count)
        {
            if (count > handles.Length || count > output.Length) throw new ArgumentException("count exceeds buffers");
            return (int)PhysRNative.pr_body_get_transforms_f32(Ptr, handles, (uint)count, output);
        }

        public bool SetPose(BodyHandle h, FPVector3 position, FPQuaternion rotation) { return PhysRNative.pr_body_set_pose(Ptr, h.Value, ref position, ref rotation) != 0; }
        public bool SetVelocity(BodyHandle h, FPVector3 linear, FPVector3 angular) { return PhysRNative.pr_body_set_velocity(Ptr, h.Value, ref linear, ref angular) != 0; }
        public bool SetKinematicTarget(BodyHandle h, FPVector3 position, FPQuaternion rotation) { return PhysRNative.pr_body_set_kinematic_target(Ptr, h.Value, ref position, ref rotation) != 0; }
        public bool ApplyImpulse(BodyHandle h, FPVector3 impulse) { return PhysRNative.pr_body_apply_impulse(Ptr, h.Value, ref impulse, IntPtr.Zero) != 0; }
        public bool ApplyImpulse(BodyHandle h, FPVector3 impulse, FPVector3 point) { return PhysRNative.pr_body_apply_impulse_at(Ptr, h.Value, ref impulse, ref point) != 0; }
        public bool ApplyForce(BodyHandle h, FPVector3 force) { return PhysRNative.pr_body_apply_force(Ptr, h.Value, ref force, IntPtr.Zero) != 0; }
        public bool ApplyForce(BodyHandle h, FPVector3 force, FPVector3 point) { return PhysRNative.pr_body_apply_force_at(Ptr, h.Value, ref force, ref point) != 0; }
        public bool ApplyTorque(BodyHandle h, FPVector3 torque) { return PhysRNative.pr_body_apply_torque(Ptr, h.Value, ref torque) != 0; }
        public void Wake(BodyHandle h) { PhysRNative.pr_body_wake(Ptr, h.Value); }
        public bool SetFilter(BodyHandle h, int layer, uint mask) { return PhysRNative.pr_body_set_filter(Ptr, h.Value, (uint)layer, mask) != 0; }
        public bool SetMaterial(BodyHandle h, FP friction, FP restitution) { return PhysRNative.pr_body_set_material(Ptr, h.Value, friction.Raw, restitution.Raw) != 0; }
        public bool SetFlags(BodyHandle h, BodyFlags flags) { return PhysRNative.pr_body_set_flags(Ptr, h.Value, (uint)flags) != 0; }

        // ------------------------------------------------------------------
        // Queries
        // ------------------------------------------------------------------

        public bool Raycast(FPVector3 origin, FPVector3 direction, FP maxDistance, out RayHit hit)
        {
            QueryFilter f = QueryFilter.All;
            return PhysRNative.pr_raycast(Ptr, ref origin, ref direction, maxDistance.Raw, ref f, out hit) != 0;
        }

        public bool Raycast(FPVector3 origin, FPVector3 direction, FP maxDistance, QueryFilter filter, out RayHit hit)
        {
            return PhysRNative.pr_raycast(Ptr, ref origin, ref direction, maxDistance.Raw, ref filter, out hit) != 0;
        }

        /// <summary>
        /// Lag-compensated raycast: bodies flagged History are tested at their
        /// poses from <paramref name="tick"/> + <paramref name="alpha"/>.
        /// </summary>
        public bool RaycastRewind(uint tick, FP alpha, FPVector3 origin, FPVector3 direction, FP maxDistance, QueryFilter filter, out RayHit hit)
        {
            return PhysRNative.pr_raycast_rewind(Ptr, tick, alpha.Raw, ref origin, ref direction, maxDistance.Raw, ref filter, out hit) != 0;
        }

        /// <summary>Fills <paramref name="results"/>; returns the total number of overlaps found.</summary>
        public int OverlapSphere(FPVector3 center, FP radius, QueryFilter filter, ulong[] results)
        {
            return (int)PhysRNative.pr_overlap_sphere(Ptr, ref center, radius.Raw, ref filter, results, results == null ? 0u : (uint)results.Length);
        }

        public bool SphereCast(FPVector3 center, FP radius, FPVector3 displacement, QueryFilter filter, out SweepHit hit)
        {
            return PhysRNative.pr_sphere_cast(Ptr, ref center, radius.Raw, ref displacement, ref filter, out hit) != 0;
        }

        public bool CapsuleCast(FPVector3 center, FP halfHeight, FP radius, FPVector3 displacement, QueryFilter filter, out SweepHit hit)
        {
            return PhysRNative.pr_capsule_cast(Ptr, ref center, halfHeight.Raw, radius.Raw, ref displacement, ref filter, out hit) != 0;
        }

        // ------------------------------------------------------------------
        // Characters
        // ------------------------------------------------------------------

        public CharacterHandle CreateCharacter(CharacterDesc desc) { return new CharacterHandle(PhysRNative.pr_character_create(Ptr, ref desc)); }
        public bool DestroyCharacter(CharacterHandle h) { return PhysRNative.pr_character_destroy(Ptr, h.Value) != 0; }

        /// <summary>Input consumed by the next <see cref="Step"/>.</summary>
        public bool SetInput(CharacterHandle h, CharacterInput input) { return PhysRNative.pr_character_set_input(Ptr, h.Value, ref input) != 0; }

        /// <summary>Advance one character immediately (local prediction / replay).</summary>
        public bool Simulate(CharacterHandle h, CharacterInput input) { return PhysRNative.pr_character_simulate(Ptr, h.Value, ref input, 0) != 0; }

        public bool Simulate(CharacterHandle h, CharacterInput input, FP dt) { return PhysRNative.pr_character_simulate(Ptr, h.Value, ref input, dt.Raw) != 0; }

        public bool TryGetState(CharacterHandle h, out CharacterState state) { return PhysRNative.pr_character_get_state(Ptr, h.Value, out state) != 0; }

        public CharacterState GetState(CharacterHandle h)
        {
            CharacterState s;
            PhysRNative.pr_character_get_state(Ptr, h.Value, out s);
            return s;
        }

        public bool Teleport(CharacterHandle h, FPVector3 position) { return PhysRNative.pr_character_teleport(Ptr, h.Value, ref position) != 0; }
        public bool SetVelocity(CharacterHandle h, FPVector3 velocity) { return PhysRNative.pr_character_set_velocity(Ptr, h.Value, ref velocity) != 0; }

        /// <summary>Knockback / explosions / jump pads (an upward kick un-grounds).</summary>
        public bool AddVelocity(CharacterHandle h, FPVector3 dv) { return PhysRNative.pr_character_add_velocity(Ptr, h.Value, ref dv) != 0; }

        public bool SetMode(CharacterHandle h, MoveMode mode) { return PhysRNative.pr_character_set_mode(Ptr, h.Value, (uint)mode) != 0; }

        /// <summary>Character owning a body (e.g. from a raycast hit), or invalid.</summary>
        public CharacterHandle CharacterOfBody(BodyHandle body) { return new CharacterHandle(PhysRNative.pr_character_of_body(Ptr, body.Value)); }

        // ------------------------------------------------------------------
        // Doors (state is part of snapshots)
        // ------------------------------------------------------------------

        public DoorHandle CreateDoor(DoorDesc desc) { return new DoorHandle(PhysRNative.pr_door_create(Ptr, ref desc)); }
        public bool DestroyDoor(DoorHandle h) { return PhysRNative.pr_door_destroy(Ptr, h.Value) != 0; }
        /// <summary>Start opening (true) or closing (false).</summary>
        public bool SetDoorOpen(DoorHandle h, bool open) { return PhysRNative.pr_door_set_open(Ptr, h.Value, open ? 1u : 0u) != 0; }
        /// <summary>Move toward a partial opening (0 = closed .. 1 = open).</summary>
        public bool SetDoorTarget(DoorHandle h, FP fraction) { return PhysRNative.pr_door_set_target(Ptr, h.Value, fraction.Raw) != 0; }
        /// <summary>Snap instantly to an opening (0..1).</summary>
        public bool SetDoorAmount(DoorHandle h, FP fraction) { return PhysRNative.pr_door_set_amount(Ptr, h.Value, fraction.Raw) != 0; }

        public DoorState GetState(DoorHandle h)
        {
            DoorState s;
            PhysRNative.pr_door_get_state(Ptr, h.Value, out s);
            return s;
        }

        // ------------------------------------------------------------------
        // Events
        // ------------------------------------------------------------------

        public int EventCount { get { return (int)PhysRNative.pr_world_event_count(Ptr); } }

        /// <summary>Copies events from the last step; returns the total available.</summary>
        public int GetEvents(PhysEvent[] buffer)
        {
            return (int)PhysRNative.pr_world_get_events(Ptr, buffer, buffer == null ? 0u : (uint)buffer.Length);
        }

        // ------------------------------------------------------------------
        // Snapshots / desync detection
        // ------------------------------------------------------------------

        /// <summary>Serialise the full simulation state into a new array.</summary>
        public byte[] SaveSnapshot()
        {
            uint size = PhysRNative.pr_world_save(Ptr, null, 0);
            var data = new byte[size];
            PhysRNative.pr_world_save(Ptr, data, size);
            return data;
        }

        /// <summary>
        /// Serialise into a reusable internal buffer (no allocation once warm).
        /// Returns the number of valid bytes in <see cref="LastSnapshot"/>.
        /// </summary>
        public int SaveSnapshotNoAlloc()
        {
            uint size = PhysRNative.pr_world_save(Ptr, snapshotBuffer, (uint)snapshotBuffer.Length);
            if (size > snapshotBuffer.Length)
            {
                snapshotBuffer = new byte[size + size / 2];
                size = PhysRNative.pr_world_save(Ptr, snapshotBuffer, (uint)snapshotBuffer.Length);
            }
            return (int)size;
        }

        public byte[] LastSnapshot { get { return snapshotBuffer; } }

        public void LoadSnapshot(byte[] data) { LoadSnapshot(data, data.Length); }

        public void LoadSnapshot(byte[] data, int length)
        {
            uint r = PhysRNative.pr_world_load(Ptr, data, (uint)length);
            if (r != 0) throw new InvalidOperationException("phys-r: snapshot load failed (code " + r + ")");
        }

        /// <summary>64-bit hash of the full state; compare between peers to detect desyncs.</summary>
        public ulong StateHash() { return PhysRNative.pr_world_hash(Ptr); }

        public void ClearHistory() { PhysRNative.pr_world_clear_history(Ptr); }
    }
}
