using System;
using System.Runtime.InteropServices;

namespace PhysR
{
    /// <summary>Fixed-point unit quaternion (layout-compatible with the native PrQuat).</summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct FPQuaternion : IEquatable<FPQuaternion>
    {
        public FP X, Y, Z, W;

        public static readonly FPQuaternion Identity = new FPQuaternion(FP.Zero, FP.Zero, FP.Zero, FP.One);

        public FPQuaternion(FP x, FP y, FP z, FP w) { X = x; Y = y; Z = z; W = w; }

        /// <summary>Euler angles in radians, Unity order (Z, X, Y).</summary>
        public static FPQuaternion Euler(FP x, FP y, FP z)
        {
            FPQuaternion q;
            PhysRNative.pr_quat_from_euler(x.Raw, y.Raw, z.Raw, out q);
            return q;
        }

        /// <summary>Rotation about +Y (FPS yaw, radians).</summary>
        public static FPQuaternion FromYaw(FP yaw) { return Euler(FP.Zero, yaw, FP.Zero); }

        public static FPQuaternion AngleAxis(FP radians, FPVector3 axis)
        {
            FPQuaternion q;
            PhysRNative.pr_quat_from_axis_angle(ref axis, radians.Raw, out q);
            return q;
        }

        /// <summary>Deterministic conversion of a Unity rotation (authoring data only).</summary>
        public static FPQuaternion FromFloat(float x, float y, float z, float w)
        {
            return new FPQuaternion(FP.FromFloat(x), FP.FromFloat(y), FP.FromFloat(z), FP.FromFloat(w));
        }

        public static FPQuaternion operator *(FPQuaternion a, FPQuaternion b)
        {
            FPQuaternion q;
            PhysRNative.pr_quat_mul(ref a, ref b, out q);
            return q;
        }

        public static FPVector3 operator *(FPQuaternion q, FPVector3 v)
        {
            FPVector3 r;
            PhysRNative.pr_quat_rotate(ref q, ref v, out r);
            return r;
        }

        public FPQuaternion Conjugate { get { return new FPQuaternion(-X, -Y, -Z, W); } }

        public static bool operator ==(FPQuaternion a, FPQuaternion b) { return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W; }
        public static bool operator !=(FPQuaternion a, FPQuaternion b) { return !(a == b); }
        public bool Equals(FPQuaternion o) { return this == o; }
        public override bool Equals(object o) { return o is FPQuaternion && (FPQuaternion)o == this; }
        public override int GetHashCode() { return X.GetHashCode() ^ (Y.GetHashCode() << 2) ^ (Z.GetHashCode() >> 2) ^ (W.GetHashCode() << 4); }
        public override string ToString() { return "(" + X + ", " + Y + ", " + Z + ", " + W + ")"; }
    }
}
