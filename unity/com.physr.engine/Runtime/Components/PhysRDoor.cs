using UnityEngine;

namespace PhysR
{
    public enum PhysRHingeSide { Left, Right }

    /// <summary>
    /// Deterministic door. The GameObject is the door slab (box of `size`,
    /// centred on the transform). Hinge doors rotate about the vertical edge
    /// on `hingeSide` (local ±X); sliding doors move along local X.
    ///
    /// Door state lives in the native world and is part of snapshots, so
    /// call Open/Close from simulated game logic (e.g. a "use" input handled
    /// in PhysRWorld.BeforeTick) and rollback/replay stays exact.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class PhysRDoor : MonoBehaviour
    {
        public DoorKind kind = DoorKind.Hinge;
        [Tooltip("Full size of the door slab.")]
        public Vector3 size = new Vector3(1.2f, 2.2f, 0.1f);
        public PhysRHingeSide hingeSide = PhysRHingeSide.Left;
        [Tooltip("Hinge: opening angle in degrees (sign = direction).")]
        public float openAngle = 90f;
        [Tooltip("Slide: opening distance in metres along local X (sign = direction).")]
        public float slideDistance = 1.2f;
        [Tooltip("Degrees per second (hinge) or metres per second (slide).")]
        public float speed = 120f;
        [Tooltip("Stop when a player is in the way instead of pushing them.")]
        public bool blockOnCharacters = true;
        public bool startOpen = false;
        [Range(0, 31)] public int layer = 0;
        public int collisionMask = -1;
        public int creationOrder = 0;

        public DoorHandle Handle { get; private set; }
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
            Handle = DoorHandle.Invalid;
            Body = BodyHandle.Invalid;
            Owner = null;
        }

        internal bool CreateIn(PhysRWorld w)
        {
            FPVector3 pos, scale;
            FPQuaternion rot;
            PhysRConvert.WorldPose(transform, out pos, out rot, out scale);
            FPVector3 full = FPVector3.Scale(size.ToFP(), scale);
            FPVector3 half = new FPVector3(FP.Abs(full.X) * FP.Half, FP.Abs(full.Y) * FP.Half, FP.Abs(full.Z) * FP.Half);
            var d = DoorDesc.Default;
            d.Shape = w.GetBox(half);
            d.Kind = kind;
            d.ClosedRotation = rot;
            d.Layer = (uint)layer;
            d.Mask = unchecked((uint)collisionMask);
            d.BlockOnCharacters = blockOnCharacters ? 1u : 0u;
            d.UserData = (ulong)(uint)GetInstanceID();
            if (kind == DoorKind.Hinge)
            {
                FP side = hingeSide == PhysRHingeSide.Left ? FP.NegOne : FP.One;
                FPVector3 edge = rot * new FPVector3(half.X * side, FP.Zero, FP.Zero);
                d.Pivot = pos + edge;
                d.Offset = -edge;
                d.Axis = rot * FPVector3.Up;
                d.OpenAmount = openAngle.ToFP() * FP.Deg2Rad;
                d.Speed = FP.Abs(speed.ToFP() * FP.Deg2Rad);
            }
            else
            {
                d.Pivot = pos;
                d.Axis = rot * FPVector3.Right;
                d.OpenAmount = slideDistance.ToFP();
                d.Speed = FP.Abs(speed.ToFP());
            }
            Handle = w.World.CreateDoor(d);
            Owner = w;
            if (!Handle.IsValid)
            {
                Debug.LogError("phys-r: failed to create door '" + name + "'.", this);
                return false;
            }
            if (startOpen) w.World.SetDoorAmount(Handle, FP.One);
            Body = w.World.GetState(Handle).Body;
            return true;
        }

        PhysWorld W
        {
            get
            {
                if (Owner == null || !Handle.IsValid) throw new System.InvalidOperationException("phys-r: door '" + name + "' is not registered yet");
                return Owner.World;
            }
        }

        public DoorState State { get { return W.GetState(Handle); } }
        public bool IsOpenOrOpening { get { return State.Target > FP.Zero || State.Target < FP.Zero; } }
        public bool Blocked { get { return State.Blocked != 0; } }

        public void Open() { W.SetDoorOpen(Handle, true); }
        public void Close() { W.SetDoorOpen(Handle, false); }
        public void Toggle() { W.SetDoorOpen(Handle, !IsOpenOrOpening); }
        /// <summary>Partial opening, 0 = closed .. 1 = open.</summary>
        public void SetTarget(FP fraction) { W.SetDoorTarget(Handle, fraction); }

#if UNITY_EDITOR
        void OnDrawGizmosSelected()
        {
            Gizmos.color = new Color(1f, 0.6f, 0.1f);
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
            Vector3 s = Vector3.Scale(size, transform.lossyScale);
            Gizmos.DrawWireCube(Vector3.zero, s);
            if (kind == DoorKind.Hinge)
            {
                float x = (hingeSide == PhysRHingeSide.Left ? -0.5f : 0.5f) * s.x;
                Gizmos.DrawLine(new Vector3(x, -0.5f * s.y, 0f), new Vector3(x, 0.5f * s.y, 0f));
            }
        }
#endif
    }
}
