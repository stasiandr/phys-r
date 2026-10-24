using System;
using UnityEngine;

namespace PhysR
{
    public enum PhysRShapeType { Box, Sphere, Capsule }

    /// <summary>
    /// A phys-r rigid body. The shape is centred on this transform (capsules
    /// along local Y). Sizes are scaled by the transform's scale in fixed point.
    /// For level geometry built from Unity colliders see PhysRStaticGeometry.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class PhysRBody : MonoBehaviour
    {
        [Header("Body")]
        public BodyKind kind = BodyKind.Dynamic;
        public PhysRShapeType shape = PhysRShapeType.Box;
        [Tooltip("Box: full size. Ignored for other shapes.")]
        public Vector3 size = Vector3.one;
        [Tooltip("Sphere/capsule radius.")]
        public float radius = 0.5f;
        [Tooltip("Capsule total height (including hemispheres).")]
        public float height = 2f;

        [Header("Material")]
        public float mass = 1f;
        public float friction = 0.6f;
        public float restitution = 0f;
        public float linearDamping = 0.01f;
        public float angularDamping = 0.05f;
        public float gravityScale = 1f;

        [Header("Filtering")]
        [Range(0, 31)] public int layer = 0;
        [Tooltip("Bitmask of phys-r layers this body collides with (-1 = all).")]
        public int collisionMask = -1;
        public BodyFlags flags = BodyFlags.None;

        [Header("Determinism / visuals")]
        [Tooltip("Bodies are created sorted by (creationOrder, hierarchy path).")]
        public int creationOrder = 0;
        [Tooltip("Interpolate the transform between ticks (dynamic/kinematic).")]
        public bool interpolate = true;

        public BodyHandle Handle { get; private set; }
        public PhysRWorld Owner { get; private set; }

        /// <summary>(self, other, point, normal). Requires BodyFlags.ContactEvents on either body.</summary>
        public event Action<PhysRBody, PhysRBody, Vector3, Vector3> ContactBegin;
        public event Action<PhysRBody, PhysRBody> ContactEnd;
        /// <summary>Raised on the sensor and on the other body.</summary>
        public event Action<PhysRBody, PhysRBody> SensorEnter;
        public event Action<PhysRBody, PhysRBody> SensorExit;

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
            var w = PhysRWorld.Instance;
            if (w != null) w.Register(this);
        }

        void Start()
        {
            // The world may have been created after us.
            if (!Handle.IsValid && PhysRWorld.Instance != null) PhysRWorld.Instance.Register(this);
        }

        void OnDisable()
        {
            if (Owner != null) Owner.Unregister(this);
            else if (PhysRWorld.Instance != null) PhysRWorld.Instance.Unregister(this);
            Handle = BodyHandle.Invalid;
            Owner = null;
        }

        internal bool CreateIn(PhysRWorld w)
        {
            FPVector3 pos, scale;
            FPQuaternion rot;
            PhysRConvert.WorldPose(transform, out pos, out rot, out scale);
            ShapeId sid;
            switch (shape)
            {
                case PhysRShapeType.Sphere:
                    sid = w.GetSphere(radius.ToFP() * PhysRConvert.MaxAbs(scale));
                    break;
                case PhysRShapeType.Capsule:
                {
                    FP sr = FP.Max(FP.Abs(scale.X), FP.Abs(scale.Z));
                    FP r = radius.ToFP() * sr;
                    FP hh = FP.Max(FP.Zero, (height.ToFP() * FP.Abs(scale.Y)) * FP.Half - r);
                    sid = w.GetCapsule(hh, r);
                    break;
                }
                default:
                {
                    FPVector3 full = FPVector3.Scale(size.ToFP(), scale);
                    sid = w.GetBox(new FPVector3(FP.Abs(full.X) * FP.Half, FP.Abs(full.Y) * FP.Half, FP.Abs(full.Z) * FP.Half));
                    break;
                }
            }
            var d = BodyDesc.Default;
            d.Kind = kind;
            d.Shape = sid;
            d.Position = pos;
            d.Rotation = rot;
            d.Mass = mass.ToFP();
            d.Friction = friction.ToFP();
            d.Restitution = restitution.ToFP();
            d.LinearDamping = linearDamping.ToFP();
            d.AngularDamping = angularDamping.ToFP();
            d.GravityScale = gravityScale.ToFP();
            d.Layer = (uint)layer;
            d.Mask = unchecked((uint)collisionMask);
            d.Flags = flags;
            d.UserData = (ulong)(uint)GetInstanceID();
            Handle = w.World.CreateBody(d);
            Owner = w;
            if (!Handle.IsValid)
            {
                Debug.LogError("phys-r: failed to create body for '" + name + "'.", this);
                return false;
            }
            return true;
        }

        internal void RaiseEvent(PhysEvent e, PhysRBody other, bool isB)
        {
            switch (e.Kind)
            {
                case EventKind.ContactBegin:
                    if (ContactBegin != null)
                    {
                        // Normals point from A to B; flip so they point away from `other`.
                        Vector3 n = e.Normal.ToUnity();
                        ContactBegin(this, other, e.Point.ToUnity(), isB ? n : -n);
                    }
                    break;
                case EventKind.ContactEnd:
                    if (ContactEnd != null) ContactEnd(this, other);
                    break;
                case EventKind.SensorEnter:
                    if (SensorEnter != null) SensorEnter(this, other);
                    break;
                case EventKind.SensorExit:
                    if (SensorExit != null) SensorExit(this, other);
                    break;
            }
        }

        PhysWorld W
        {
            get
            {
                if (Owner == null || !Handle.IsValid) throw new InvalidOperationException("phys-r: body '" + name + "' is not registered yet");
                return Owner.World;
            }
        }

        public BodyState State { get { return W.GetState(Handle); } }
        public FPVector3 Position { get { return State.Position; } }
        public FPQuaternion Rotation { get { return State.Rotation; } }

        public FPVector3 LinearVelocity
        {
            get { return State.LinearVelocity; }
            set { var s = State; W.SetVelocity(Handle, value, s.AngularVelocity); }
        }

        public FPVector3 AngularVelocity
        {
            get { return State.AngularVelocity; }
            set { var s = State; W.SetVelocity(Handle, s.LinearVelocity, value); }
        }

        public void AddImpulse(FPVector3 impulse) { W.ApplyImpulse(Handle, impulse); }
        public void AddImpulseAtPoint(FPVector3 impulse, FPVector3 point) { W.ApplyImpulse(Handle, impulse, point); }
        public void AddForce(FPVector3 force) { W.ApplyForce(Handle, force); }
        public void AddTorque(FPVector3 torque) { W.ApplyTorque(Handle, torque); }
        public void Teleport(FPVector3 position, FPQuaternion rotation) { W.SetPose(Handle, position, rotation); }
        public void MoveKinematic(FPVector3 position, FPQuaternion rotation) { W.SetKinematicTarget(Handle, position, rotation); }
        public void Wake() { W.Wake(Handle); }

#if UNITY_EDITOR
        void OnDrawGizmosSelected()
        {
            Gizmos.color = kind == BodyKind.Static ? new Color(0.5f, 0.5f, 0.5f) : kind == BodyKind.Kinematic ? Color.cyan : Color.green;
            if ((flags & BodyFlags.Sensor) != 0) Gizmos.color = Color.yellow;
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
            Vector3 s = transform.lossyScale;
            switch (shape)
            {
                case PhysRShapeType.Box:
                    Gizmos.DrawWireCube(Vector3.zero, Vector3.Scale(size, s));
                    break;
                case PhysRShapeType.Sphere:
                    Gizmos.DrawWireSphere(Vector3.zero, radius * Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y), Mathf.Abs(s.z)));
                    break;
                case PhysRShapeType.Capsule:
                {
                    float r = radius * Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.z));
                    float hh = Mathf.Max(0f, height * Mathf.Abs(s.y) * 0.5f - r);
                    Gizmos.DrawWireSphere(Vector3.up * hh, r);
                    Gizmos.DrawWireSphere(Vector3.down * hh, r);
                    Gizmos.DrawLine(new Vector3(r, hh, 0), new Vector3(r, -hh, 0));
                    Gizmos.DrawLine(new Vector3(-r, hh, 0), new Vector3(-r, -hh, 0));
                    Gizmos.DrawLine(new Vector3(0, hh, r), new Vector3(0, -hh, r));
                    Gizmos.DrawLine(new Vector3(0, hh, -r), new Vector3(0, -hh, -r));
                    break;
                }
            }
        }
#endif
    }
}
