using System.Collections.Generic;
using UnityEngine;

namespace PhysR
{
    /// <summary>
    /// Bakes every BoxCollider / SphereCollider / CapsuleCollider under this
    /// GameObject into static phys-r bodies. Author your level with Unity's
    /// primitive colliders (no mesh colliders: primitives in a BVH are far
    /// faster), put them under this component, done.
    ///
    /// Poses are composed in fixed point from serialized local transforms, so
    /// the baked level is bit-identical on every platform.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class PhysRStaticGeometry : MonoBehaviour
    {
        [Range(0, 31)] public int layer = 0;
        public int collisionMask = -1;
        public float friction = 0.6f;
        public float restitution = 0f;
        [Tooltip("Only include colliders on these Unity layers.")]
        public LayerMask includeLayers = ~0;
        public bool includeInactive = false;
        [Tooltip("Treat colliders marked 'Is Trigger' as phys-r sensors.")]
        public bool triggersAsSensors = true;
        [Tooltip("Disable the Unity colliders after baking (avoid double physics).")]
        public bool disableUnityColliders = true;

        readonly List<BodyHandle> handles = new List<BodyHandle>();
        public IReadOnlyList<BodyHandle> Handles { get { return handles; } }

        void Start()
        {
            var w = PhysRWorld.Instance;
            if (w == null)
            {
                Debug.LogError("phys-r: PhysRStaticGeometry needs a PhysRWorld in the scene.", this);
                return;
            }
            Bake(w);
        }

        void OnDestroy()
        {
            var w = PhysRWorld.Instance;
            if (w == null || w.World == null) return;
            foreach (var h in handles) w.World.DestroyBody(h);
            handles.Clear();
        }

        public void Bake(PhysRWorld w)
        {
            var colliders = new List<Collider>(GetComponentsInChildren<Collider>(includeInactive));
            // Deterministic order independent of Unity's traversal details.
            colliders.Sort((a, b) =>
            {
                int c = string.CompareOrdinal(Path(a.transform), Path(b.transform));
                return c != 0 ? c : string.CompareOrdinal(a.GetType().Name, b.GetType().Name);
            });
            foreach (var col in colliders)
            {
                if ((includeLayers.value & (1 << col.gameObject.layer)) == 0) continue;
                FPVector3 pos, scale;
                FPQuaternion rot;
                PhysRConvert.WorldPose(col.transform, out pos, out rot, out scale);
                FPVector3 absScale = new FPVector3(FP.Abs(scale.X), FP.Abs(scale.Y), FP.Abs(scale.Z));
                ShapeId shape;
                FPVector3 center;
                FPQuaternion shapeRot = rot;
                var box = col as BoxCollider;
                var sphere = col as SphereCollider;
                var capsule = col as CapsuleCollider;
                if (box != null)
                {
                    center = box.center.ToFP();
                    shape = w.GetBox(FPVector3.Scale(box.size.ToFP(), absScale) * FP.Half);
                }
                else if (sphere != null)
                {
                    center = sphere.center.ToFP();
                    shape = w.GetSphere(sphere.radius.ToFP() * PhysRConvert.MaxAbs(scale));
                }
                else if (capsule != null)
                {
                    center = capsule.center.ToFP();
                    // Unity capsules may run along X (0), Y (1) or Z (2).
                    FP along, across;
                    if (capsule.direction == 0)
                    {
                        along = absScale.X; across = FP.Max(absScale.Y, absScale.Z);
                        shapeRot = rot * FPQuaternion.AngleAxis(-FP.HalfPi, FPVector3.Forward);
                    }
                    else if (capsule.direction == 2)
                    {
                        along = absScale.Z; across = FP.Max(absScale.X, absScale.Y);
                        shapeRot = rot * FPQuaternion.AngleAxis(FP.HalfPi, FPVector3.Right);
                    }
                    else
                    {
                        along = absScale.Y; across = FP.Max(absScale.X, absScale.Z);
                    }
                    FP r = capsule.radius.ToFP() * across;
                    FP hh = FP.Max(FP.Zero, capsule.height.ToFP() * along * FP.Half - r);
                    shape = w.GetCapsule(hh, r);
                }
                else
                {
                    Debug.LogWarning("phys-r: skipping unsupported collider " + col.GetType().Name + " on '" + col.name + "' (use box/sphere/capsule).", col);
                    continue;
                }
                var d = BodyDesc.Default;
                d.Kind = BodyKind.Static;
                d.Shape = shape;
                d.Position = pos + rot * FPVector3.Scale(center, scale);
                d.Rotation = shapeRot;
                d.Layer = (uint)layer;
                d.Mask = unchecked((uint)collisionMask);
                d.Friction = friction.ToFP();
                d.Restitution = restitution.ToFP();
                if (triggersAsSensors && col.isTrigger) d.Flags = BodyFlags.Sensor;
                d.UserData = (ulong)(uint)col.GetInstanceID();
                var h = w.World.CreateBody(d);
                if (h.IsValid) handles.Add(h);
                if (disableUnityColliders) col.enabled = false;
            }
        }

        static string Path(Transform t)
        {
            string p = t.name + "#" + t.GetSiblingIndex();
            for (Transform q = t.parent; q != null; q = q.parent) p = q.name + "#" + q.GetSiblingIndex() + "/" + p;
            return p;
        }
    }
}
