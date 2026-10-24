using UnityEngine;

namespace PhysR
{
    /// <summary>
    /// Conversions between Unity float types and phys-r fixed-point types.
    ///
    /// Float -> FP conversion is done natively and is exact, so serialized
    /// authoring data (positions, sizes) converts identically everywhere.
    /// World-space poses are composed in fixed point from local transforms:
    /// Unity's own float world matrices may differ between ARM and x86 builds
    /// (fused multiply-add), so they must never feed the simulation.
    /// </summary>
    public static class PhysRConvert
    {
        public static FP ToFP(this float v) { return FP.FromFloat(v); }
        public static FPVector3 ToFP(this Vector3 v) { return FPVector3.FromFloat(v.x, v.y, v.z); }
        public static FPQuaternion ToFP(this Quaternion q) { return FPQuaternion.FromFloat(q.x, q.y, q.z, q.w); }

        public static Vector3 ToUnity(this FPVector3 v) { return new Vector3(v.X.ToFloat(), v.Y.ToFloat(), v.Z.ToFloat()); }
        public static Quaternion ToUnity(this FPQuaternion q) { return new Quaternion(q.X.ToFloat(), q.Y.ToFloat(), q.Z.ToFloat(), q.W.ToFloat()); }

        public static Vector3 Position(this TransformF t) { return new Vector3(t.PX, t.PY, t.PZ); }
        public static Quaternion Rotation(this TransformF t) { return new Quaternion(t.RX, t.RY, t.RZ, t.RW); }

        /// <summary>
        /// Deterministic world pose of <paramref name="t"/> composed in fixed
        /// point from serialized local position/rotation/scale (skew ignored).
        /// </summary>
        public static void WorldPose(Transform t, out FPVector3 position, out FPQuaternion rotation, out FPVector3 scale)
        {
            position = t.localPosition.ToFP();
            rotation = t.localRotation.ToFP();
            scale = t.localScale.ToFP();
            for (Transform p = t.parent; p != null; p = p.parent)
            {
                FPVector3 pp = p.localPosition.ToFP();
                FPQuaternion pr = p.localRotation.ToFP();
                FPVector3 ps = p.localScale.ToFP();
                position = pr * FPVector3.Scale(ps, position) + pp;
                rotation = pr * rotation;
                scale = FPVector3.Scale(ps, scale);
            }
        }

        /// <summary>Largest absolute scale component (for uniform-only shapes).</summary>
        public static FP MaxAbs(FPVector3 s)
        {
            return FP.Max(FP.Abs(s.X), FP.Max(FP.Abs(s.Y), FP.Abs(s.Z)));
        }
    }
}
