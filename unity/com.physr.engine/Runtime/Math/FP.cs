using System;
using System.Runtime.InteropServices;

namespace PhysR
{
    /// <summary>
    /// Deterministic Q40.24 fixed-point number, bit-compatible with the native
    /// engine's <c>Fx</c>. Addition, subtraction, multiplication, comparison and
    /// most divisions run in managed code with exactly the same results as Rust;
    /// sqrt/trig/large divisions call into the native library so every platform
    /// produces identical bits.
    ///
    /// Use FP (never float) for anything that feeds the simulation: inputs,
    /// gameplay values, spawn positions. Convert to float only for rendering.
    /// </summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct FP : IEquatable<FP>, IComparable<FP>
    {
        public const int FracBits = 24;
        public const long OneRaw = 1L << FracBits;
        const long HalfRaw = 1L << (FracBits - 1);

        /// <summary>Raw Q40.24 value. Serialise this over the network.</summary>
        public long Raw;

        public static readonly FP Zero = new FP { Raw = 0 };
        public static readonly FP One = new FP { Raw = OneRaw };
        public static readonly FP NegOne = new FP { Raw = -OneRaw };
        public static readonly FP Half = new FP { Raw = OneRaw / 2 };
        public static readonly FP Two = new FP { Raw = OneRaw * 2 };
        public static readonly FP MaxValue = new FP { Raw = long.MaxValue };
        public static readonly FP MinValue = new FP { Raw = -long.MaxValue };
        public static readonly FP Epsilon = new FP { Raw = 1 };
        // Same rounding as the Rust constants (round half away from zero).
        public static readonly FP Pi = new FP { Raw = 52707179 };
        public static readonly FP HalfPi = new FP { Raw = 26353589 };
        public static readonly FP TwoPi = new FP { Raw = 105414357 };
        public static readonly FP Deg2Rad = new FP { Raw = 292818 };
        public static readonly FP Rad2Deg = new FP { Raw = 961263669 };

        public static FP FromRaw(long raw) { return new FP { Raw = raw }; }
        public static FP FromInt(int v) { return new FP { Raw = (long)v << FracBits }; }

        /// <summary>Exact num / den (|num| must be below 2^39).</summary>
        public static FP FromRatio(long num, long den)
        {
            if (den == 0) return Zero;
            return new FP { Raw = (num << FracBits) / den };
        }

        /// <summary>Deterministic float conversion (done natively, IEEE exact).</summary>
        public static FP FromFloat(float v) { return new FP { Raw = PhysRNative.pr_fx_from_f32(v) }; }
        public static FP FromDouble(double v) { return new FP { Raw = PhysRNative.pr_fx_from_f64(v) }; }

        public float ToFloat() { return (float)((double)Raw / OneRaw); }
        public double ToDouble() { return (double)Raw / OneRaw; }
        /// <summary>Floor to integer.</summary>
        public long ToInt() { return Raw >> FracBits; }

        public static FP operator +(FP a, FP b) { return new FP { Raw = unchecked(a.Raw + b.Raw) }; }
        public static FP operator -(FP a, FP b) { return new FP { Raw = unchecked(a.Raw - b.Raw) }; }
        public static FP operator -(FP a) { return new FP { Raw = unchecked(-a.Raw) }; }
        public static FP operator *(FP a, FP b) { return new FP { Raw = MulRaw(a.Raw, b.Raw) }; }
        public static FP operator /(FP a, FP b) { return new FP { Raw = DivRaw(a.Raw, b.Raw) }; }
        public static FP operator *(FP a, int b) { return new FP { Raw = unchecked(a.Raw * b) }; }
        public static FP operator *(int a, FP b) { return new FP { Raw = unchecked(a * b.Raw) }; }
        public static FP operator /(FP a, int b) { return b == 0 ? Zero : new FP { Raw = a.Raw / b }; }

        public static bool operator ==(FP a, FP b) { return a.Raw == b.Raw; }
        public static bool operator !=(FP a, FP b) { return a.Raw != b.Raw; }
        public static bool operator <(FP a, FP b) { return a.Raw < b.Raw; }
        public static bool operator >(FP a, FP b) { return a.Raw > b.Raw; }
        public static bool operator <=(FP a, FP b) { return a.Raw <= b.Raw; }
        public static bool operator >=(FP a, FP b) { return a.Raw >= b.Raw; }

        public static implicit operator FP(int v) { return FromInt(v); }

        public static FP Abs(FP a) { return new FP { Raw = a.Raw < 0 ? unchecked(-a.Raw) : a.Raw }; }
        public static FP Min(FP a, FP b) { return a.Raw < b.Raw ? a : b; }
        public static FP Max(FP a, FP b) { return a.Raw > b.Raw ? a : b; }
        public static FP Clamp(FP v, FP lo, FP hi) { return Min(Max(v, lo), hi); }
        public static FP Lerp(FP a, FP b, FP t) { return a + (b - a) * t; }
        public static FP Sign(FP a) { return a.Raw > 0 ? One : a.Raw < 0 ? NegOne : Zero; }
        public static FP Floor(FP a) { return new FP { Raw = a.Raw & ~(OneRaw - 1) }; }

        public static FP Sqrt(FP a) { return new FP { Raw = PhysRNative.pr_fx_sqrt(a.Raw) }; }
        public static FP Sin(FP a) { return new FP { Raw = PhysRNative.pr_fx_sin(a.Raw) }; }
        public static FP Cos(FP a) { return new FP { Raw = PhysRNative.pr_fx_cos(a.Raw) }; }
        public static FP Atan2(FP y, FP x) { return new FP { Raw = PhysRNative.pr_fx_atan2(y.Raw, x.Raw) }; }

        /// <summary>
        /// Rust: <c>((a as i128 * b as i128 + 2^23) >> 24) as i64</c>, reproduced
        /// with 64-bit halves (no Int128 on Unity's runtime).
        /// </summary>
        public static long MulRaw(long a, long b)
        {
            unchecked
            {
                bool neg = (a < 0) != (b < 0);
                ulong ua = a < 0 ? (ulong)(-a) : (ulong)a;
                ulong ub = b < 0 ? (ulong)(-b) : (ulong)b;
                ulong hi, lo;
                MulU64(ua, ub, out hi, out lo);
                if (neg)
                {
                    lo = ~lo + 1UL;
                    hi = ~hi + (lo == 0 ? 1UL : 0UL);
                }
                ulong lo2 = lo + (ulong)HalfRaw;
                if (lo2 < lo) hi += 1UL;
                return (long)((lo2 >> FracBits) | (hi << (64 - FracBits)));
            }
        }

        /// <summary>Rust: truncating division with saturation on divide-by-zero.</summary>
        public static long DivRaw(long a, long b)
        {
            if (b == 0) return a >= 0 ? long.MaxValue : -long.MaxValue;
            ulong ua = a < 0 ? unchecked((ulong)(-(a + 1)) + 1UL) : (ulong)a;
            if (ua < (1UL << (63 - FracBits)))
                return (a << FracBits) / b; // C# long division truncates toward zero, like Rust.
            return PhysRNative.pr_fx_div(a, b);
        }

        static void MulU64(ulong a, ulong b, out ulong hi, out ulong lo)
        {
            unchecked
            {
                ulong aL = (uint)a, aH = a >> 32;
                ulong bL = (uint)b, bH = b >> 32;
                ulong ll = aL * bL;
                ulong lh = aL * bH;
                ulong hl = aH * bL;
                ulong hh = aH * bH;
                ulong mid = (ll >> 32) + (uint)lh + (uint)hl;
                lo = (mid << 32) | (uint)ll;
                hi = hh + (lh >> 32) + (hl >> 32) + (mid >> 32);
            }
        }

        public bool Equals(FP o) { return Raw == o.Raw; }
        public override bool Equals(object o) { return o is FP && ((FP)o).Raw == Raw; }
        public override int GetHashCode() { return Raw.GetHashCode(); }
        public int CompareTo(FP o) { return Raw.CompareTo(o.Raw); }
        public override string ToString() { return ToDouble().ToString("0.######"); }
    }
}
