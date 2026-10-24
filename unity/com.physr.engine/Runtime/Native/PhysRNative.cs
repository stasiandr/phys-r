using System;
using System.Runtime.InteropServices;

namespace PhysR
{
    public enum BodyKind : uint { Static = 0, Dynamic = 1, Kinematic = 2 }

    [Flags]
    public enum BodyFlags : uint
    {
        None = 0,
        /// <summary>Trigger volume: reports enter/exit, no collision response.</summary>
        Sensor = 1 << 0,
        /// <summary>Velocity-expanded speculative contacts for fast bodies.</summary>
        Ccd = 1 << 1,
        /// <summary>Record poses for lag-compensated raycasts.</summary>
        History = 1 << 2,
        NoSleep = 1 << 3,
        LockRotation = 1 << 4,
        ContactEvents = 1 << 5,
    }

    [Flags]
    public enum QueryFlags : uint
    {
        None = 0,
        IncludeSensors = 1 << 0,
        ExcludeStatic = 1 << 1,
        ExcludeMoving = 1 << 2,
    }

    [Flags]
    public enum CharacterButtons : uint
    {
        None = 0,
        Jump = 1 << 0,
        Sprint = 1 << 1,
        Crouch = 1 << 2,
    }

    public enum EventKind : uint { ContactBegin = 0, ContactEnd = 1, SensorEnter = 2, SensorExit = 3 }

    public enum MoveMode : uint
    {
        Walk = 0,
        /// <summary>Free 3D movement with collision, no gravity (ladders, swimming, spectators).</summary>
        Fly = 1,
        /// <summary>No collision.</summary>
        Noclip = 2,
    }

    [Flags]
    public enum CollisionFlags : uint
    {
        None = 0,
        Below = 1 << 0,
        Above = 1 << 1,
        Sides = 1 << 2,
        /// <summary>Landed this tick (see CharacterState.LandingSpeed).</summary>
        Landed = 1 << 3,
        Jumped = 1 << 4,
        /// <summary>Wanted to stand up but a ceiling is in the way.</summary>
        StandBlocked = 1 << 5,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WorldSettings
    {
        public FPVector3 Gravity;
        public FP Dt;
        public FP ContactMargin;
        public FP Slop;
        public FP Baumgarte;
        public FP MaxCorrectionVelocity;
        public FP RestitutionThreshold;
        public FP SleepLinearThreshold;
        public FP SleepAngularThreshold;
        public FP TimeToSleep;
        public uint VelocityIterations;
        public uint PositionIterations;
        public uint HistoryTicks;
        public uint EnableSleep;

        public static WorldSettings Default
        {
            get { WorldSettings s; PhysRNative.pr_world_settings_default(out s); return s; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BodyDesc
    {
        public BodyKind Kind;
        public ShapeId Shape;
        public FPVector3 Position;
        public FPQuaternion Rotation;
        public FPVector3 LinearVelocity;
        public FPVector3 AngularVelocity;
        public FP Mass;
        public FP Friction;
        public FP Restitution;
        public FP LinearDamping;
        public FP AngularDamping;
        public FP GravityScale;
        public uint Layer;
        public uint Mask;
        public BodyFlags Flags;
        uint _pad;
        public ulong UserData;

        public static BodyDesc Default
        {
            get { BodyDesc d; PhysRNative.pr_body_desc_default(out d); return d; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BodyState
    {
        public FPVector3 Position;
        public FPQuaternion Rotation;
        public FPVector3 LinearVelocity;
        public FPVector3 AngularVelocity;
        public uint Sleeping;
        public BodyKind Kind;
        public ulong UserData;
    }

    /// <summary>Float pose for rendering only.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct TransformF
    {
        public float PX, PY, PZ;
        public float RX, RY, RZ, RW;
        public uint Valid;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct QueryFilter
    {
        public uint Mask;
        public QueryFlags Flags;
        public BodyHandle Exclude;

        public static QueryFilter All { get { return new QueryFilter { Mask = uint.MaxValue }; } }

        public QueryFilter(uint mask, BodyHandle exclude = default(BodyHandle), QueryFlags flags = QueryFlags.None)
        {
            Mask = mask;
            Exclude = exclude;
            Flags = flags;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RayHit
    {
        public BodyHandle Body;
        public FP Distance;
        public FPVector3 Point;
        public FPVector3 Normal;
        /// <summary>
        /// Hit point in the body's local space. On a character capsule
        /// LocalPoint.Y is the height above the capsule centre (hit zones).
        /// </summary>
        public FPVector3 LocalPoint;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SweepHit
    {
        public BodyHandle Body;
        /// <summary>Fraction (0..1) of the displacement travelled before contact.</summary>
        public FP Fraction;
        public FPVector3 Normal;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct MovementSettings
    {
        public FP WalkSpeed;
        public FP SprintSpeed;
        public FP CrouchSpeed;
        public FP GroundAccel;
        public FP AirAccel;
        public FP AirSpeedCap;
        public FP Friction;
        public FP StopSpeed;
        public FP Gravity;
        public FP JumpSpeed;
        public FP MaxFallSpeed;
        public uint JumpRequiresRelease;
        /// <summary>Ticks after leaving an edge during which jumping still works.</summary>
        public uint CoyoteTicks;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CharacterDesc
    {
        /// <summary>Capsule centre.</summary>
        public FPVector3 Position;
        public FP Radius;
        /// <summary>Standing height including hemispheres.</summary>
        public FP Height;
        public FP CrouchHeight;
        public FP StepHeight;
        public FP MaxSlopeCos;
        public FP Skin;
        public FP SnapDistance;
        public FP Mass;
        public FP PushStrength;
        public uint Layer;
        public uint Mask;
        public BodyFlags BodyFlags;
        uint _pad;
        public ulong UserData;
        public MovementSettings Movement;

        public static CharacterDesc Default
        {
            get { CharacterDesc d; PhysRNative.pr_character_desc_default(out d); return d; }
        }
    }

    /// <summary>
    /// One tick of player input. Quantise on the wire (e.g. sbyte axes, ushort
    /// yaw) and rebuild with FP.FromRatio so every peer gets identical values.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CharacterInput
    {
        public FP MoveX;
        /// <summary>Vertical axis for fly mode (ladders, swimming).</summary>
        public FP MoveY;
        public FP MoveZ;
        /// <summary>View yaw in radians about +Y (0 looks down +Z).</summary>
        public FP Yaw;
        public CharacterButtons Buttons;
        uint _pad;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CharacterState
    {
        /// <summary>Capsule centre.</summary>
        public FPVector3 Position;
        public FPVector3 Velocity;
        public FPVector3 GroundNormal;
        /// <summary>Velocity of the moving platform under the feet.</summary>
        public FPVector3 GroundVelocity;
        public BodyHandle Body;
        /// <summary>Body stood on (invalid when airborne).</summary>
        public BodyHandle GroundBody;
        /// <summary>Downward speed on the landing tick (fall damage), else zero.</summary>
        public FP LandingSpeed;
        /// <summary>Current capsule height (crouch aware).</summary>
        public FP Height;
        public uint Grounded;
        public uint Crouched;
        public MoveMode Mode;
        public CollisionFlags Collision;

        public bool IsGrounded { get { return Grounded != 0; } }
        public bool IsCrouched { get { return Crouched != 0; } }
        public FPVector3 Feet { get { return Position - FPVector3.Up * (Height / 2); } }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysEvent
    {
        public EventKind Kind;
        uint _pad;
        public BodyHandle A;
        public BodyHandle B;
        public FPVector3 Point;
        public FPVector3 Normal;
    }

    public enum DoorKind : uint
    {
        /// <summary>Rotates about an axis; amounts in radians.</summary>
        Hinge = 0,
        /// <summary>Translates along an axis; amounts in metres.</summary>
        Slide = 1,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct DoorDesc
    {
        public ShapeId Shape;
        public DoorKind Kind;
        /// <summary>Hinge: point on the axis. Slide: closed body centre.</summary>
        public FPVector3 Pivot;
        /// <summary>Hinge axis / slide direction.</summary>
        public FPVector3 Axis;
        public FPQuaternion ClosedRotation;
        /// <summary>Hinge: closed body centre relative to Pivot.</summary>
        public FPVector3 Offset;
        /// <summary>Fully open amount (radians or metres; sign = direction).</summary>
        public FP OpenAmount;
        public FP Speed;
        public FP Friction;
        public uint Layer;
        public uint Mask;
        /// <summary>1 = stop on players instead of pushing them.</summary>
        public uint BlockOnCharacters;
        uint _pad;
        public ulong UserData;

        public static DoorDesc Default
        {
            get { DoorDesc d; PhysRNative.pr_door_desc_default(out d); return d; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct DoorState
    {
        public BodyHandle Body;
        public FP Amount;
        public FP Target;
        /// <summary>0 = closed .. 1 = open.</summary>
        public FP OpenFraction;
        public uint Blocked;
        public uint Moving;
    }

    /// <summary>Raw P/Invoke declarations. Prefer the <see cref="PhysWorld"/> wrapper.</summary>
    public static class PhysRNative
    {
#if (UNITY_IOS || UNITY_WEBGL) && !UNITY_EDITOR
        const string Lib = "__Internal";
#else
        const string Lib = "physr";
#endif
        const CallingConvention CC = CallingConvention.Cdecl;

        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_version();

        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_world_settings_default(out WorldSettings s);
        [DllImport(Lib, CallingConvention = CC)] public static extern IntPtr pr_world_create(ref WorldSettings s);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_world_destroy(IntPtr w);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_world_step(IntPtr w, uint steps);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_world_tick(IntPtr w);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_world_set_gravity(IntPtr w, ref FPVector3 g);

        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_shape_sphere(IntPtr w, long radius);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_shape_capsule(IntPtr w, long halfHeight, long radius);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_shape_box(IntPtr w, ref FPVector3 halfExtents);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_shape_count(IntPtr w);

        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_body_desc_default(out BodyDesc d);
        [DllImport(Lib, CallingConvention = CC)] public static extern ulong pr_body_create(IntPtr w, ref BodyDesc d);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_destroy(IntPtr w, ulong h);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_is_valid(IntPtr w, ulong h);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_get_state(IntPtr w, ulong h, out BodyState s);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_get_transforms_f32(IntPtr w, ulong[] handles, uint count, [Out] TransformF[] output);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_set_pose(IntPtr w, ulong h, ref FPVector3 position, ref FPQuaternion rotation);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_set_velocity(IntPtr w, ulong h, ref FPVector3 linear, ref FPVector3 angular);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_set_kinematic_target(IntPtr w, ulong h, ref FPVector3 position, ref FPQuaternion rotation);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_apply_impulse(IntPtr w, ulong h, ref FPVector3 impulse, IntPtr pointNull);
        [DllImport(Lib, CallingConvention = CC, EntryPoint = "pr_body_apply_impulse")] public static extern uint pr_body_apply_impulse_at(IntPtr w, ulong h, ref FPVector3 impulse, ref FPVector3 point);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_apply_force(IntPtr w, ulong h, ref FPVector3 force, IntPtr pointNull);
        [DllImport(Lib, CallingConvention = CC, EntryPoint = "pr_body_apply_force")] public static extern uint pr_body_apply_force_at(IntPtr w, ulong h, ref FPVector3 force, ref FPVector3 point);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_apply_torque(IntPtr w, ulong h, ref FPVector3 torque);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_body_wake(IntPtr w, ulong h);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_set_filter(IntPtr w, ulong h, uint layer, uint mask);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_set_material(IntPtr w, ulong h, long friction, long restitution);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_body_set_flags(IntPtr w, ulong h, uint flags);

        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_raycast(IntPtr w, ref FPVector3 origin, ref FPVector3 dir, long maxDist, ref QueryFilter filter, out RayHit hit);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_raycast_rewind(IntPtr w, uint tick, long alpha, ref FPVector3 origin, ref FPVector3 dir, long maxDist, ref QueryFilter filter, out RayHit hit);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_overlap_sphere(IntPtr w, ref FPVector3 center, long radius, ref QueryFilter filter, [Out] ulong[] output, uint capacity);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_sphere_cast(IntPtr w, ref FPVector3 center, long radius, ref FPVector3 disp, ref QueryFilter filter, out SweepHit hit);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_capsule_cast(IntPtr w, ref FPVector3 center, long halfHeight, long radius, ref FPVector3 disp, ref QueryFilter filter, out SweepHit hit);

        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_character_desc_default(out CharacterDesc d);
        [DllImport(Lib, CallingConvention = CC)] public static extern ulong pr_character_create(IntPtr w, ref CharacterDesc d);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_destroy(IntPtr w, ulong h);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_set_input(IntPtr w, ulong h, ref CharacterInput input);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_simulate(IntPtr w, ulong h, ref CharacterInput input, long dt);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_get_state(IntPtr w, ulong h, out CharacterState s);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_teleport(IntPtr w, ulong h, ref FPVector3 position);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_set_velocity(IntPtr w, ulong h, ref FPVector3 velocity);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_add_velocity(IntPtr w, ulong h, ref FPVector3 dv);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_character_set_mode(IntPtr w, ulong h, uint mode);
        [DllImport(Lib, CallingConvention = CC)] public static extern ulong pr_character_of_body(IntPtr w, ulong body);

        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_door_desc_default(out DoorDesc d);
        [DllImport(Lib, CallingConvention = CC)] public static extern ulong pr_door_create(IntPtr w, ref DoorDesc d);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_door_destroy(IntPtr w, ulong h);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_door_set_open(IntPtr w, ulong h, uint open);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_door_set_target(IntPtr w, ulong h, long fraction);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_door_set_amount(IntPtr w, ulong h, long fraction);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_door_get_state(IntPtr w, ulong h, out DoorState s);

        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_world_event_count(IntPtr w);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_world_get_events(IntPtr w, [Out] PhysEvent[] output, uint capacity);

        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_world_save(IntPtr w, [Out] byte[] output, uint capacity);
        [DllImport(Lib, CallingConvention = CC)] public static extern uint pr_world_load(IntPtr w, byte[] data, uint len);
        [DllImport(Lib, CallingConvention = CC)] public static extern ulong pr_world_hash(IntPtr w);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_world_clear_history(IntPtr w);

        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_from_f32(float v);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_from_f64(double v);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_mul(long a, long b);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_div(long a, long b);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_sqrt(long a);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_sin(long a);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_cos(long a);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_fx_atan2(long y, long x);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_quat_from_euler(long x, long y, long z, out FPQuaternion q);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_quat_from_axis_angle(ref FPVector3 axis, long angle, out FPQuaternion q);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_quat_mul(ref FPQuaternion a, ref FPQuaternion b, out FPQuaternion q);
        [DllImport(Lib, CallingConvention = CC)] public static extern void pr_quat_rotate(ref FPQuaternion q, ref FPVector3 v, out FPVector3 r);
        [DllImport(Lib, CallingConvention = CC)] public static extern long pr_vec3_normalize(ref FPVector3 v, out FPVector3 n);
    }
}
