using UnityEngine;

namespace PhysR
{
    /// <summary>
    /// Deterministic first-person character (kinematic capsule with Quake-style
    /// movement, steps, slopes and ground snapping). The transform is the
    /// capsule centre. Set <see cref="NextInput"/> every tick (or call
    /// <see cref="PhysWorld.SetInput"/> yourself from netcode).
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class PhysRCharacter : MonoBehaviour
    {
        [Header("Capsule")]
        public float radius = 0.4f;
        public float height = 1.8f;
        public float crouchHeight = 1.2f;
        public float stepHeight = 0.35f;
        [Range(0f, 89f)] public float maxSlopeDegrees = 45f;
        public float skinWidth = 0.01f;
        public float snapDistance = 0.3f;

        [Header("Movement")]
        public float walkSpeed = 6f;
        public float sprintSpeed = 9f;
        public float crouchSpeed = 3f;
        public float groundAcceleration = 10f;
        public float airAcceleration = 2f;
        public float airSpeedCap = 1f;
        public float friction = 6f;
        public float stopSpeed = 1.5f;
        public float gravity = 20f;
        public float jumpSpeed = 7f;
        public float maxFallSpeed = 50f;
        public bool jumpRequiresRelease = true;
        [Tooltip("Ticks after walking off an edge during which jumping still works.")]
        public int coyoteTicks = 6;

        [Header("Interaction")]
        public float mass = 80f;
        [Range(0f, 1f)] public float pushStrength = 0.5f;
        [Range(0, 31)] public int layer = 1;
        public int collisionMask = -1;
        [Tooltip("Record poses for lag-compensated hitscan.")]
        public bool recordHistory = true;

        [Header("Determinism / visuals")]
        public int creationOrder = 0;
        public bool interpolate = true;

        /// <summary>Input applied on the next tick.</summary>
        [System.NonSerialized] public CharacterInput NextInput;

        public CharacterHandle Handle { get; private set; }
        public BodyHandle Body { get; private set; }
        public PhysRWorld Owner { get; private set; }

        string hierarchyPath;
        internal string HierarchyPath
        {
            get
            {
                if (hierarchyPath == null)
                {
                    hierarchyPath = name;
                    for (Transform p = transform.parent; p != null; p = p.parent) hierarchyPath = p.name + "/" + hierarchyPath;
                    hierarchyPath += "#" + transform.GetSiblingIndex();
                }
                return hierarchyPath;
            }
        }

        void OnEnable()
        {
            if (PhysRWorld.Instance != null) PhysRWorld.Instance.Register(this);
        }

        void Start()
        {
            if (!Handle.IsValid && PhysRWorld.Instance != null) PhysRWorld.Instance.Register(this);
        }

        void OnDisable()
        {
            if (Owner != null) Owner.Unregister(this);
            else if (PhysRWorld.Instance != null) PhysRWorld.Instance.Unregister(this);
            Handle = CharacterHandle.Invalid;
            Body = BodyHandle.Invalid;
            Owner = null;
        }

        internal bool CreateIn(PhysRWorld w)
        {
            FPVector3 pos, scale;
            FPQuaternion rot;
            PhysRConvert.WorldPose(transform, out pos, out rot, out scale);
            var d = CharacterDesc.Default;
            d.Position = pos;
            d.Radius = radius.ToFP();
            d.Height = height.ToFP();
            d.CrouchHeight = crouchHeight.ToFP();
            d.StepHeight = stepHeight.ToFP();
            // Cosine via the deterministic native trig.
            d.MaxSlopeCos = FP.Cos(maxSlopeDegrees.ToFP() * FP.Deg2Rad);
            d.Skin = skinWidth.ToFP();
            d.SnapDistance = snapDistance.ToFP();
            d.Mass = mass.ToFP();
            d.PushStrength = pushStrength.ToFP();
            d.Layer = (uint)layer;
            d.Mask = unchecked((uint)collisionMask);
            d.BodyFlags = recordHistory ? BodyFlags.History : BodyFlags.None;
            d.UserData = (ulong)(uint)GetInstanceID();
            d.Movement.WalkSpeed = walkSpeed.ToFP();
            d.Movement.SprintSpeed = sprintSpeed.ToFP();
            d.Movement.CrouchSpeed = crouchSpeed.ToFP();
            d.Movement.GroundAccel = groundAcceleration.ToFP();
            d.Movement.AirAccel = airAcceleration.ToFP();
            d.Movement.AirSpeedCap = airSpeedCap.ToFP();
            d.Movement.Friction = friction.ToFP();
            d.Movement.StopSpeed = stopSpeed.ToFP();
            d.Movement.Gravity = gravity.ToFP();
            d.Movement.JumpSpeed = jumpSpeed.ToFP();
            d.Movement.MaxFallSpeed = maxFallSpeed.ToFP();
            d.Movement.JumpRequiresRelease = jumpRequiresRelease ? 1u : 0u;
            d.Movement.CoyoteTicks = (uint)Mathf.Max(0, coyoteTicks);
            Handle = w.World.CreateCharacter(d);
            Owner = w;
            if (!Handle.IsValid)
            {
                Debug.LogError("phys-r: failed to create character '" + name + "'.", this);
                return false;
            }
            Body = w.World.GetState(Handle).Body;
            return true;
        }

        internal void PushInput(PhysWorld w)
        {
            if (Handle.IsValid) w.SetInput(Handle, NextInput);
        }

        PhysWorld W
        {
            get
            {
                if (Owner == null || !Handle.IsValid) throw new System.InvalidOperationException("phys-r: character '" + name + "' is not registered yet");
                return Owner.World;
            }
        }

        public CharacterState State { get { return W.GetState(Handle); } }
        public FPVector3 Position { get { return State.Position; } }
        public FPVector3 Velocity { get { return State.Velocity; } set { W.SetVelocity(Handle, value); } }
        public bool Grounded { get { return State.Grounded != 0; } }
        public bool Crouched { get { return State.Crouched != 0; } }
        /// <summary>Current capsule height (crouch aware).</summary>
        public FP Height { get { return State.Height; } }
        public FPVector3 Feet { get { return State.Feet; } }
        public CollisionFlags Collision { get { return State.Collision; } }

        /// <summary>Knockback, explosions, jump pads.</summary>
        public void AddVelocity(FPVector3 dv) { W.AddVelocity(Handle, dv); }

        /// <summary>Walk / Fly (ladders, swimming) / Noclip.</summary>
        public void SetMode(MoveMode mode) { W.SetMode(Handle, mode); }

        public void Teleport(FPVector3 position) { W.Teleport(Handle, position); }

        /// <summary>Advance just this character now (client prediction / input replay).</summary>
        public void Simulate(CharacterInput input) { W.Simulate(Handle, input); }

#if UNITY_EDITOR
        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.magenta;
            float hh = Mathf.Max(0f, height * 0.5f - radius);
            Vector3 c = transform.position;
            Gizmos.DrawWireSphere(c + Vector3.up * hh, radius);
            Gizmos.DrawWireSphere(c - Vector3.up * hh, radius);
        }
#endif
    }
}
