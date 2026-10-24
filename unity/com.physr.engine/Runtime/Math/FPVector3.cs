using System;
using System.Runtime.InteropServices;

namespace PhysR
{
    /// <summary>Fixed-point 3D vector (layout-compatible with the native PrVec3).</summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct FPVector3 : IEquatable<FPVector3>
    {
        public FP X, Y, Z;

        public static readonly FPVector3 Zero = new FPVector3();
        public static readonly FPVector3 One = new FPVector3(FP.One, FP.One, FP.One);
        public static readonly FPVector3 Up = new FPVector3(FP.Zero, FP.One, FP.Zero);
        public static readonly FPVector3 Down = new FPVector3(FP.Zero, FP.NegOne, FP.Zero);
        public static readonly FPVector3 Forward = new FPVector3(FP.Zero, FP.Zero, FP.One);
        public static readonly FPVector3 Right = new FPVector3(FP.One, FP.Zero, FP.Zero);

        public FPVector3(FP x, FP y, FP z) { X = x; Y = y; Z = z; }
        public FPVector3(int x, int y, int z) { X = x; Y = y; Z = z; }

        public static FPVector3 FromFloat(float x, float y, float z)
        {
            return new FPVector3(FP.FromFloat(x), FP.FromFloat(y), FP.FromFloat(z));
        }

        public static FPVector3 operator +(FPVector3 a, FPVector3 b) { return new FPVector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        public static FPVector3 operator -(FPVector3 a, FPVector3 b) { return new FPVector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        public static FPVector3 operator -(FPVector3 a) { return new FPVector3(-a.X, -a.Y, -a.Z); }
        public static FPVector3 operator *(FPVector3 a, FP s) { return new FPVector3(a.X * s, a.Y * s, a.Z * s); }
        public static FPVector3 operator *(FP s, FPVector3 a) { return new FPVector3(a.X * s, a.Y * s, a.Z * s); }
        public static FPVector3 operator /(FPVector3 a, FP s) { return new FPVector3(a.X / s, a.Y / s, a.Z / s); }
        public static bool operator ==(FPVector3 a, FPVector3 b) { return a.X == b.X && a.Y == b.Y && a.Z == b.Z; }
        public static bool operator !=(FPVector3 a, FPVector3 b) { return !(a == b); }

        /// <summary>Dot product rounded once, exactly like the native engine.</summary>
        public static FP Dot(FPVector3 a, FPVector3 b)
        {
            I128 s = I128.Mul(a.X.Raw, b.X.Raw);
            s = s.Add(I128.Mul(a.Y.Raw, b.Y.Raw));
            s = s.Add(I128.Mul(a.Z.Raw, b.Z.Raw));
            return FP.FromRaw(s.RoundShr24());
        }

        public static FPVector3 Cross(FPVector3 a, FPVector3 b)
        {
            return new FPVector3(
                FP.FromRaw(I128.Mul(a.Y.Raw, b.Z.Raw).Sub(I128.Mul(a.Z.Raw, b.Y.Raw)).RoundShr24()),
                FP.FromRaw(I128.Mul(a.Z.Raw, b.X.Raw).Sub(I128.Mul(a.X.Raw, b.Z.Raw)).RoundShr24()),
                FP.FromRaw(I128.Mul(a.X.Raw, b.Y.Raw).Sub(I128.Mul(a.Y.Raw, b.X.Raw)).RoundShr24()));
        }

        public FP SqrMagnitude { get { return Dot(this, this); } }

        /// <summary>Exact length (computed natively).</summary>
        public FP Magnitude
        {
            get
            {
                FPVector3 n, v = this;
                return FP.FromRaw(PhysRNative.pr_vec3_normalize(ref v, out n));
            }
        }

        /// <summary>Unit vector, or zero for (near) zero vectors.</summary>
        public FPVector3 Normalized
        {
            get
            {
                FPVector3 n, v = this;
                PhysRNative.pr_vec3_normalize(ref v, out n);
                return n;
            }
        }

        public static FPVector3 Lerp(FPVector3 a, FPVector3 b, FP t) { return a + (b - a) * t; }
        public static FPVector3 Scale(FPVector3 a, FPVector3 b) { return new FPVector3(a.X * b.X, a.Y * b.Y, a.Z * b.Z); }

        public bool Equals(FPVector3 o) { return this == o; }
        public override bool Equals(object o) { return o is FPVector3 && (FPVector3)o == this; }
        public override int GetHashCode() { return X.GetHashCode() ^ (Y.GetHashCode() << 2) ^ (Z.GetHashCode() >> 2); }
        public override string ToString() { return "(" + X + ", " + Y + ", " + Z + ")"; }
    }

    /// <summary>Minimal signed 128-bit integer for exact fixed-point products.</summary>
    internal struct I128
    {
        public ulong Lo;
        public ulong Hi;

        public static I128 Mul(long a, long b)
        {
            unchecked
            {
                bool neg = (a < 0) != (b < 0);
                ulong ua = a < 0 ? (ulong)(-a) : (ulong)a;
                ulong ub = b < 0 ? (ulong)(-b) : (ulong)b;
                ulong aL = (uint)ua, aH = ua >> 32, bL = (uint)ub, bH = ub >> 32;
                ulong ll = aL * bL, lh = aL * bH, hl = aH * bL, hh = aH * bH;
                ulong mid = (ll >> 32) + (uint)lh + (uint)hl;
                I128 r;
                r.Lo = (mid << 32) | (uint)ll;
                r.Hi = hh + (lh >> 32) + (hl >> 32) + (mid >> 32);
                return neg ? r.Neg() : r;
            }
        }

        public I128 Neg()
        {
            unchecked
            {
                I128 r;
                r.Lo = ~Lo + 1UL;
                r.Hi = ~Hi + (r.Lo == 0 ? 1UL : 0UL);
                return r;
            }
        }

        public I128 Add(I128 o)
        {
            unchecked
            {
                I128 r;
                r.Lo = Lo + o.Lo;
                r.Hi = Hi + o.Hi + (r.Lo < Lo ? 1UL : 0UL);
                return r;
            }
        }

        public I128 Sub(I128 o) { return Add(o.Neg()); }

        /// <summary>(this + 2^23) >> 24, truncated to 64 bits.</summary>
        public long RoundShr24()
        {
            unchecked
            {
                ulong lo2 = Lo + (1UL << 23);
                ulong hi2 = Hi + (lo2 < Lo ? 1UL : 0UL);
                return (long)((lo2 >> 24) | (hi2 << 40));
            }
        }
    }
}
