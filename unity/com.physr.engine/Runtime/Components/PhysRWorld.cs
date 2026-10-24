using System;
using System.Collections.Generic;
using UnityEngine;

namespace PhysR
{
    /// <summary>
    /// Scene entry point: owns a <see cref="PhysWorld"/>, runs the fixed tick,
    /// creates bodies for registered components in a deterministic order and
    /// interpolates their visuals between ticks.
    ///
    /// Networking: disable <see cref="autoSimulate"/> and call <see cref="Tick"/>
    /// from your netcode (one call per simulation tick, after applying inputs).
    /// </summary>
    [DefaultExecutionOrder(-1000)]
    [DisallowMultipleComponent]
    public sealed class PhysRWorld : MonoBehaviour
    {
        public static PhysRWorld Instance { get; private set; }

        [Header("Simulation")]
        [Tooltip("Fixed ticks per second (e.g. 30, 60, 64, 128).")]
        [SerializeField] int tickRate = 60;
        [SerializeField] Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        [SerializeField, Range(1, 32)] int velocityIterations = 8;
        [SerializeField, Range(0, 16)] int positionIterations = 3;
        [SerializeField] bool enableSleep = true;
        [Tooltip("Ticks of pose history kept for lag-compensated raycasts.")]
        [SerializeField, Range(0, 1024)] int historyTicks = 64;

        [Header("Driving")]
        [Tooltip("Run ticks from Update with an accumulator. Disable when netcode calls Tick().")]
        public bool autoSimulate = true;
        [SerializeField, Range(1, 16)] int maxTicksPerFrame = 4;

        PhysWorld world;
        float accumulator;
        bool flushing;

        readonly List<PhysRBody> pending = new List<PhysRBody>();
        readonly List<PhysRCharacter> pendingCharacters = new List<PhysRCharacter>();
        readonly List<PhysRBody> bodies = new List<PhysRBody>();
        readonly List<PhysRCharacter> characters = new List<PhysRCharacter>();
        readonly Dictionary<ulong, PhysRBody> byHandle = new Dictionary<ulong, PhysRBody>();
        readonly Dictionary<string, ShapeId> shapeCache = new Dictionary<string, ShapeId>();

        // Interpolation buffers (rendering only).
        ulong[] handles = new ulong[64];
        TransformF[] prev = new TransformF[64];
        TransformF[] curr = new TransformF[64];
        readonly List<Transform> visuals = new List<Transform>();
        int visualCount;
        bool visualsDirty = true;
        PhysEvent[] events = new PhysEvent[64];

        /// <summary>Raised before each tick (apply inputs here).</summary>
        public event Action<PhysRWorld> BeforeTick;
        /// <summary>Raised after each tick (read results, fire gameplay).</summary>
        public event Action<PhysRWorld> AfterTick;

        public PhysWorld World { get { return world; } }
        public FP Dt { get { return world.Dt; } }
        public float DtSeconds { get { return 1f / tickRate; } }
        public uint CurrentTick { get { return world.Tick; } }
        /// <summary>Render interpolation factor between the last two ticks.</summary>
        public float InterpolationAlpha { get; private set; }

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Debug.LogWarning("phys-r: more than one PhysRWorld in the scene; using the first one.");
                enabled = false;
                return;
            }
            Instance = this;
            var s = WorldSettings.Default;
            s.Gravity = gravity.ToFP();
            s.Dt = FP.FromRatio(1, Mathf.Max(1, tickRate));
            s.VelocityIterations = (uint)velocityIterations;
            s.PositionIterations = (uint)positionIterations;
            s.EnableSleep = enableSleep ? 1u : 0u;
            s.HistoryTicks = (uint)historyTicks;
            world = new PhysWorld(s);
        }

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
            if (world != null)
            {
                world.Dispose();
                world = null;
            }
        }

        void Update()
        {
            if (!autoSimulate || world == null) return;
            accumulator += Time.deltaTime;
            float dt = DtSeconds;
            int n = 0;
            while (accumulator >= dt && n < maxTicksPerFrame)
            {
                Tick();
                accumulator -= dt;
                n++;
            }
            if (n == maxTicksPerFrame && accumulator > dt) accumulator = 0f; // don't spiral
            InterpolationAlpha = Mathf.Clamp01(accumulator / dt);
        }

        /// <summary>Run exactly one simulation tick.</summary>
        public void Tick()
        {
            FlushPending();
            if (BeforeTick != null) BeforeTick(this);
            for (int i = 0; i < characters.Count; i++) characters[i].PushInput(world);
            world.Step();
            CapturePoses();
            DispatchEvents();
            if (AfterTick != null) AfterTick(this);
        }

        /// <summary>Set the render interpolation factor (when netcode drives ticks).</summary>
        public void SetInterpolationAlpha(float alpha) { InterpolationAlpha = Mathf.Clamp01(alpha); }

        void LateUpdate()
        {
            if (world == null) return;
            if (visualsDirty) RebuildVisuals();
            float a = InterpolationAlpha;
            for (int i = 0; i < visualCount; i++)
            {
                if (curr[i].Valid == 0 || visuals[i] == null) continue;
                Vector3 p = Vector3.LerpUnclamped(prev[i].Position(), curr[i].Position(), a);
                Quaternion r = Quaternion.SlerpUnclamped(prev[i].Rotation(), curr[i].Rotation(), a);
                visuals[i].SetPositionAndRotation(p, r);
            }
        }

        // ------------------------------------------------------------------
        // Registration
        // ------------------------------------------------------------------

        internal void Register(PhysRBody b)
        {
            if (!pending.Contains(b) && !bodies.Contains(b)) pending.Add(b);
        }

        internal void Register(PhysRCharacter c)
        {
            if (!pendingCharacters.Contains(c) && !characters.Contains(c)) pendingCharacters.Add(c);
        }

        internal void Unregister(PhysRBody b)
        {
            pending.Remove(b);
            if (bodies.Remove(b))
            {
                if (world != null && b.Handle.IsValid) world.DestroyBody(b.Handle);
                byHandle.Remove(b.Handle.Value);
                visualsDirty = true;
            }
        }

        internal void Unregister(PhysRCharacter c)
        {
            pendingCharacters.Remove(c);
            if (characters.Remove(c))
            {
                if (world != null && c.Handle.IsValid) world.DestroyCharacter(c.Handle);
                byHandle.Remove(c.Body.Value);
                visualsDirty = true;
            }
        }

        /// <summary>
        /// Create native objects for everything registered since the last tick.
        /// Order is (creationOrder, hierarchy path) so every peer creates
        /// bodies identically regardless of Unity's OnEnable order.
        /// </summary>
        public void FlushPending()
        {
            if (flushing || world == null) return;
            if (pending.Count == 0 && pendingCharacters.Count == 0) return;
            flushing = true;
            pending.Sort(CompareBodies);
            pendingCharacters.Sort(CompareCharacters);
            for (int i = 0; i < pending.Count; i++)
            {
                var b = pending[i];
                if (b == null) continue;
                if (b.CreateIn(this))
                {
                    bodies.Add(b);
                    byHandle[b.Handle.Value] = b;
                }
            }
            for (int i = 0; i < pendingCharacters.Count; i++)
            {
                var c = pendingCharacters[i];
                if (c == null) continue;
                if (c.CreateIn(this))
                {
                    characters.Add(c);
                }
            }
            pending.Clear();
            pendingCharacters.Clear();
            visualsDirty = true;
            flushing = false;
        }

        static int CompareBodies(PhysRBody a, PhysRBody b)
        {
            int c = a.creationOrder.CompareTo(b.creationOrder);
            return c != 0 ? c : string.CompareOrdinal(a.HierarchyPath, b.HierarchyPath);
        }

        static int CompareCharacters(PhysRCharacter a, PhysRCharacter b)
        {
            int c = a.creationOrder.CompareTo(b.creationOrder);
            return c != 0 ? c : string.CompareOrdinal(a.HierarchyPath, b.HierarchyPath);
        }

        public PhysRBody FindBody(BodyHandle h)
        {
            PhysRBody b;
            return byHandle.TryGetValue(h.Value, out b) ? b : null;
        }

        // ------------------------------------------------------------------
        // Shapes (deduplicated, created in deterministic order)
        // ------------------------------------------------------------------

        public ShapeId GetSphere(FP radius)
        {
            string key = "s:" + radius.Raw;
            ShapeId id;
            if (!shapeCache.TryGetValue(key, out id)) shapeCache[key] = id = world.AddSphere(radius);
            return id;
        }

        public ShapeId GetCapsule(FP halfHeight, FP radius)
        {
            string key = "c:" + halfHeight.Raw + ":" + radius.Raw;
            ShapeId id;
            if (!shapeCache.TryGetValue(key, out id)) shapeCache[key] = id = world.AddCapsule(halfHeight, radius);
            return id;
        }

        public ShapeId GetBox(FPVector3 halfExtents)
        {
            string key = "b:" + halfExtents.X.Raw + ":" + halfExtents.Y.Raw + ":" + halfExtents.Z.Raw;
            ShapeId id;
            if (!shapeCache.TryGetValue(key, out id)) shapeCache[key] = id = world.AddBox(halfExtents);
            return id;
        }

        // ------------------------------------------------------------------
        // Visual interpolation and events
        // ------------------------------------------------------------------

        void RebuildVisuals()
        {
            visuals.Clear();
            int n = 0;
            for (int i = 0; i < bodies.Count; i++)
            {
                var b = bodies[i];
                if (b.kind == BodyKind.Static || !b.interpolate) continue;
                EnsureCapacity(n + 1);
                handles[n++] = b.Handle.Value;
                visuals.Add(b.transform);
            }
            for (int i = 0; i < characters.Count; i++)
            {
                var c = characters[i];
                if (!c.interpolate) continue;
                EnsureCapacity(n + 1);
                handles[n++] = c.Body.Value;
                visuals.Add(c.transform);
            }
            visualCount = n;
            world.GetTransforms(handles, curr, n);
            Array.Copy(curr, prev, n);
            visualsDirty = false;
        }

        void EnsureCapacity(int n)
        {
            if (n <= handles.Length) return;
            int cap = Mathf.NextPowerOfTwo(n);
            Array.Resize(ref handles, cap);
            Array.Resize(ref prev, cap);
            Array.Resize(ref curr, cap);
        }

        void CapturePoses()
        {
            if (visualsDirty) RebuildVisuals();
            var t = prev;
            prev = curr;
            curr = t;
            if (visualCount > 0) world.GetTransforms(handles, curr, visualCount);
        }

        void DispatchEvents()
        {
            int n = world.GetEvents(events);
            if (n > events.Length)
            {
                events = new PhysEvent[Mathf.NextPowerOfTwo(n)];
                n = world.GetEvents(events);
            }
            for (int i = 0; i < n; i++)
            {
                var e = events[i];
                PhysRBody a = FindBody(e.A), b = FindBody(e.B);
                if (a != null) a.RaiseEvent(e, b, false);
                if (b != null) b.RaiseEvent(e, a, true);
            }
        }
    }
}
