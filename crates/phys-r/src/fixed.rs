//! Deterministic Q40.24 fixed-point scalar.
//!
//! Every simulation value in the engine is an [`Fx`]. All arithmetic is plain
//! integer math, so results are bit-identical on every platform (ARMv7, ARM64,
//! x86, x86_64), every compiler version and every optimisation level.
//!
//! * Resolution: 2^-24 ≈ 6e-8 (comparable to `f32` near 1.0, but constant
//!   everywhere instead of degrading far from the origin).
//! * Range: ±5.4e11.
//! * Multiplication rounds to nearest, division truncates toward zero.
//! * Overflow wraps (identically in debug and release builds).

use core::fmt;
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Number of fractional bits.
pub const FRAC_BITS: u32 = 24;
const ONE_RAW: i64 = 1 << FRAC_BITS;
const HALF_RAW: i128 = 1 << (FRAC_BITS - 1);

/// Q40.24 fixed-point number.
#[derive(Copy, Clone, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Fx(pub i64);

impl Fx {
    pub const ZERO: Fx = Fx(0);
    pub const ONE: Fx = Fx(ONE_RAW);
    pub const NEG_ONE: Fx = Fx(-ONE_RAW);
    pub const TWO: Fx = Fx(ONE_RAW * 2);
    pub const HALF: Fx = Fx(ONE_RAW / 2);
    /// Smallest representable positive value.
    pub const EPSILON: Fx = Fx(1);
    pub const MAX: Fx = Fx(i64::MAX);
    pub const MIN: Fx = Fx(-i64::MAX);
    pub const PI: Fx = Fx::from_f64_const(core::f64::consts::PI);
    pub const TWO_PI: Fx = Fx::from_f64_const(core::f64::consts::TAU);
    pub const HALF_PI: Fx = Fx::from_f64_const(core::f64::consts::FRAC_PI_2);
    pub const DEG_TO_RAD: Fx = Fx::from_f64_const(core::f64::consts::PI / 180.0);
    pub const RAD_TO_DEG: Fx = Fx::from_f64_const(180.0 / core::f64::consts::PI);

    #[inline(always)]
    pub const fn from_raw(raw: i64) -> Fx {
        Fx(raw)
    }

    #[inline(always)]
    pub const fn raw(self) -> i64 {
        self.0
    }

    #[inline(always)]
    pub const fn from_int(v: i32) -> Fx {
        Fx((v as i64) << FRAC_BITS)
    }

    /// `num / den` computed exactly in integer math. Handy for deterministic
    /// constants such as `Fx::from_ratio(1, 60)`.
    #[inline]
    pub const fn from_ratio(num: i64, den: i64) -> Fx {
        if den == 0 {
            return Fx::ZERO;
        }
        Fx((((num as i128) << FRAC_BITS) / den as i128) as i64)
    }

    /// Compile-time conversion from `f64`. Constant evaluation is exact IEEE
    /// arithmetic, so the resulting constant is identical on every target.
    pub const fn from_f64_const(v: f64) -> Fx {
        let s = v * ONE_RAW as f64;
        Fx(if s >= 0.0 { (s + 0.5) as i64 } else { (s - 0.5) as i64 })
    }

    /// Runtime conversion from `f64` (round half away from zero).
    ///
    /// IEEE multiplication by a power of two and rounding are exact, so this is
    /// deterministic too; still, prefer converting authoring data once at load
    /// time and keeping everything in `Fx` afterwards.
    #[inline]
    pub fn from_f64(v: f64) -> Fx {
        if v.is_nan() {
            return Fx::ZERO;
        }
        let s = v * ONE_RAW as f64;
        // `as` saturates on overflow.
        Fx(if s >= 0.0 { (s + 0.5) as i64 } else { (s - 0.5) as i64 })
    }

    #[inline]
    pub fn from_f32(v: f32) -> Fx {
        Fx::from_f64(v as f64)
    }

    /// Lossy conversion for rendering / debugging. Never feed the result back
    /// into the simulation.
    #[inline]
    pub fn to_f32(self) -> f32 {
        (self.0 as f64 / ONE_RAW as f64) as f32
    }

    #[inline]
    pub fn to_f64(self) -> f64 {
        self.0 as f64 / ONE_RAW as f64
    }

    /// Floor to integer.
    #[inline]
    pub const fn to_int(self) -> i64 {
        self.0 >> FRAC_BITS
    }

    #[inline(always)]
    pub const fn abs(self) -> Fx {
        Fx(self.0.wrapping_abs())
    }

    #[inline(always)]
    pub const fn is_negative(self) -> bool {
        self.0 < 0
    }

    #[inline(always)]
    pub const fn signum(self) -> Fx {
        if self.0 > 0 {
            Fx::ONE
        } else if self.0 < 0 {
            Fx::NEG_ONE
        } else {
            Fx::ZERO
        }
    }

    /// `1` for `>= 0`, `-1` otherwise.
    #[inline(always)]
    pub const fn sign_nonzero(self) -> Fx {
        if self.0 >= 0 {
            Fx::ONE
        } else {
            Fx::NEG_ONE
        }
    }

    #[inline(always)]
    pub fn min(self, o: Fx) -> Fx {
        if self.0 < o.0 {
            self
        } else {
            o
        }
    }

    #[inline(always)]
    pub fn max(self, o: Fx) -> Fx {
        if self.0 > o.0 {
            self
        } else {
            o
        }
    }

    #[inline(always)]
    pub fn clamp(self, lo: Fx, hi: Fx) -> Fx {
        self.max(lo).min(hi)
    }

    #[inline]
    pub const fn floor(self) -> Fx {
        Fx(self.0 & !(ONE_RAW - 1))
    }

    #[inline]
    pub const fn ceil(self) -> Fx {
        Fx((self.0.wrapping_add(ONE_RAW - 1)) & !(ONE_RAW - 1))
    }

    #[inline]
    pub const fn fract(self) -> Fx {
        Fx(self.0 & (ONE_RAW - 1))
    }

    /// Multiply by an integer (exact, no rounding).
    #[inline(always)]
    pub const fn mul_int(self, v: i64) -> Fx {
        Fx(self.0.wrapping_mul(v))
    }

    /// Divide by an integer (truncating).
    #[inline(always)]
    pub const fn div_int(self, v: i64) -> Fx {
        if v == 0 {
            return Fx::ZERO;
        }
        Fx(self.0.wrapping_div(v))
    }

    /// Multiply by `2^shift` (`shift` may be negative).
    #[inline(always)]
    pub const fn shl(self, shift: u32) -> Fx {
        Fx(self.0.wrapping_shl(shift))
    }

    #[inline(always)]
    pub const fn shr(self, shift: u32) -> Fx {
        Fx(self.0 >> shift)
    }

    #[inline]
    pub fn recip(self) -> Fx {
        Fx::ONE / self
    }

    #[inline]
    pub fn lerp(a: Fx, b: Fx, t: Fx) -> Fx {
        a + (b - a) * t
    }

    /// Square root (exact floor of the true root at 2^-24 resolution).
    /// Returns zero for non-positive inputs.
    #[inline]
    pub fn sqrt(self) -> Fx {
        if self.0 <= 0 {
            return Fx::ZERO;
        }
        let v = self.0 as u64;
        if v < (1u64 << (64 - FRAC_BITS)) {
            Fx(isqrt_u64(v << FRAC_BITS) as i64)
        } else {
            Fx(isqrt_u128((v as u128) << FRAC_BITS) as i64)
        }
    }

    /// Deterministic sine (absolute error < 1.5e-7 over the full range).
    pub fn sin(self) -> Fx {
        // Reduce to [-pi, pi).
        let two_pi = Fx::TWO_PI.0;
        let mut x = self.0.rem_euclid(two_pi);
        if x >= Fx::PI.0 {
            x -= two_pi;
        }
        // Fold to [-pi/2, pi/2].
        if x > Fx::HALF_PI.0 {
            x = Fx::PI.0 - x;
        } else if x < -Fx::HALF_PI.0 {
            x = -Fx::PI.0 - x;
        }
        // Evaluate the Taylor polynomial in Q30 for extra headroom.
        let xq = x << (30 - FRAC_BITS);
        let x2 = mul_q30(xq, xq);
        let mut p = SIN_C[5];
        p = SIN_C[4] + mul_q30(x2, p);
        p = SIN_C[3] + mul_q30(x2, p);
        p = SIN_C[2] + mul_q30(x2, p);
        p = SIN_C[1] + mul_q30(x2, p);
        p = SIN_C[0] + mul_q30(x2, p);
        let r = mul_q30(xq, p);
        Fx((r + (1 << (30 - FRAC_BITS - 1))) >> (30 - FRAC_BITS))
    }

    #[inline]
    pub fn cos(self) -> Fx {
        (self + Fx::HALF_PI).sin()
    }

    /// Returns `(sin, cos)`.
    #[inline]
    pub fn sin_cos(self) -> (Fx, Fx) {
        (self.sin(), self.cos())
    }

    /// Deterministic four-quadrant arctangent (absolute error < 2e-5 rad).
    pub fn atan2(y: Fx, x: Fx) -> Fx {
        if x.0 == 0 && y.0 == 0 {
            return Fx::ZERO;
        }
        let ax = x.abs();
        let ay = y.abs();
        let (num, den, swapped) = if ay > ax { (ax, ay, true) } else { (ay, ax, false) };
        let z = num / den; // in [0, 1]
        let z2 = z * z;
        let mut p = ATAN_C[4];
        p = ATAN_C[3] + z2 * p;
        p = ATAN_C[2] + z2 * p;
        p = ATAN_C[1] + z2 * p;
        p = ATAN_C[0] + z2 * p;
        let mut a = z * p;
        if swapped {
            a = Fx::HALF_PI - a;
        }
        if x.0 < 0 {
            a = Fx::PI - a;
        }
        if y.0 < 0 {
            a = -a;
        }
        a
    }

    /// Arc-cosine, input clamped to [-1, 1].
    pub fn acos(self) -> Fx {
        let x = self.clamp(Fx::NEG_ONE, Fx::ONE);
        let s = (Fx::ONE - x * x).sqrt();
        Fx::atan2(s, x)
    }
}

const fn q30(v: f64) -> i64 {
    let s = v * (1u64 << 30) as f64;
    if s >= 0.0 {
        (s + 0.5) as i64
    } else {
        (s - 0.5) as i64
    }
}

const SIN_C: [i64; 6] = [q30(1.0), q30(-1.0 / 6.0), q30(1.0 / 120.0), q30(-1.0 / 5040.0), q30(1.0 / 362880.0), q30(-1.0 / 39916800.0)];

const ATAN_C: [Fx; 5] = [
    Fx::from_f64_const(0.999_866_0),
    Fx::from_f64_const(-0.330_299_5),
    Fx::from_f64_const(0.180_141_0),
    Fx::from_f64_const(-0.085_133_0),
    Fx::from_f64_const(0.020_835_1),
];

#[inline(always)]
fn mul_q30(a: i64, b: i64) -> i64 {
    // Inputs are bounded by ~2.5 in Q30, so the product fits in i64.
    (a.wrapping_mul(b) + (1 << 29)) >> 30
}

/// Exact integer square root (floor).
#[inline]
pub fn isqrt_u64(n: u64) -> u64 {
    if n == 0 {
        return 0;
    }
    // A correctly rounded f64 sqrt is an excellent first guess; the integer
    // fix-up below makes the result exact regardless of the guess.
    let mut r = (n as f64).sqrt() as u64;
    let n128 = n as u128;
    while (r as u128) * (r as u128) > n128 {
        r -= 1;
    }
    while ((r + 1) as u128) * ((r + 1) as u128) <= n128 {
        r += 1;
    }
    r
}

/// Exact integer square root (floor) for 128-bit inputs below 2^126.
#[inline]
pub fn isqrt_u128(n: u128) -> u128 {
    if n == 0 {
        return 0;
    }
    if n < (1u128 << 64) {
        return isqrt_u64(n as u64) as u128;
    }
    let mut r = (n as f64).sqrt() as u128;
    while r * r > n {
        r -= 1;
    }
    while (r + 1) * (r + 1) <= n {
        r += 1;
    }
    r
}

impl Add for Fx {
    type Output = Fx;
    #[inline(always)]
    fn add(self, o: Fx) -> Fx {
        Fx(self.0.wrapping_add(o.0))
    }
}

impl Sub for Fx {
    type Output = Fx;
    #[inline(always)]
    fn sub(self, o: Fx) -> Fx {
        Fx(self.0.wrapping_sub(o.0))
    }
}

impl Neg for Fx {
    type Output = Fx;
    #[inline(always)]
    fn neg(self) -> Fx {
        Fx(self.0.wrapping_neg())
    }
}

impl Mul for Fx {
    type Output = Fx;
    #[inline(always)]
    fn mul(self, o: Fx) -> Fx {
        Fx(((self.0 as i128 * o.0 as i128 + HALF_RAW) >> FRAC_BITS) as i64)
    }
}

impl Div for Fx {
    type Output = Fx;
    #[inline]
    fn div(self, o: Fx) -> Fx {
        let a = self.0;
        let b = o.0;
        if b == 0 {
            return if a >= 0 { Fx::MAX } else { Fx::MIN };
        }
        if a.unsigned_abs() < (1u64 << (63 - FRAC_BITS)) {
            // Fast path: the shifted dividend fits in 64 bits (|a| < 32768),
            // which covers almost every division the solver performs.
            Fx((a << FRAC_BITS) / b)
        } else {
            let q = ((a as i128) << FRAC_BITS) / b as i128;
            Fx(q.clamp(-(i64::MAX as i128), i64::MAX as i128) as i64)
        }
    }
}

impl AddAssign for Fx {
    #[inline(always)]
    fn add_assign(&mut self, o: Fx) {
        *self = *self + o;
    }
}
impl SubAssign for Fx {
    #[inline(always)]
    fn sub_assign(&mut self, o: Fx) {
        *self = *self - o;
    }
}
impl MulAssign for Fx {
    #[inline(always)]
    fn mul_assign(&mut self, o: Fx) {
        *self = *self * o;
    }
}
impl DivAssign for Fx {
    #[inline(always)]
    fn div_assign(&mut self, o: Fx) {
        *self = *self / o;
    }
}

impl fmt::Debug for Fx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.6}", self.to_f64())
    }
}

impl fmt::Display for Fx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.to_f64())
    }
}

/// Shorthand for building constants: `fx!(1.5)`, `fx!(9.81)`.
///
/// Evaluated at compile time, so it is deterministic.
#[macro_export]
macro_rules! fx {
    ($v:expr) => {
        $crate::Fx::from_f64_const($v as f64)
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Fx, b: f64, tol: f64) -> bool {
        (a.to_f64() - b).abs() <= tol
    }

    #[test]
    fn basic_ops() {
        let a = Fx::from_f64(1.5);
        let b = Fx::from_f64(-2.25);
        assert!(close(a + b, -0.75, 1e-7));
        assert!(close(a * b, -3.375, 1e-7));
        assert!(close(a / b, 1.5 / -2.25, 1e-7));
        assert!(close(Fx::from_int(40000) / Fx::from_int(3), 40000.0 / 3.0, 1e-6));
        assert_eq!(Fx::ONE / Fx::ZERO, Fx::MAX);
    }

    #[test]
    fn sqrt_exact() {
        for v in [0.0001, 0.5, 1.0, 2.0, 9.0, 1234.5678, 1.0e6, 3.0e9] {
            assert!(close(Fx::from_f64(v).sqrt(), v.sqrt(), 1e-6 * v.sqrt().max(1.0)), "{v}");
        }
        assert_eq!(Fx::from_int(4).sqrt(), Fx::from_int(2));
        assert_eq!(Fx::from_int(-4).sqrt(), Fx::ZERO);
    }

    #[test]
    fn trig() {
        let mut x = -20.0f64;
        while x < 20.0 {
            let f = Fx::from_f64(x);
            assert!(close(f.sin(), x.sin(), 2e-7), "sin {x}");
            assert!(close(f.cos(), x.cos(), 2e-7), "cos {x}");
            x += 0.0137;
        }
        let mut a = -3.1f64;
        while a < 3.1 {
            let (y, xx) = (a.sin() * 3.0, a.cos() * 3.0);
            let r = Fx::atan2(Fx::from_f64(y), Fx::from_f64(xx));
            assert!(close(r, y.atan2(xx), 3e-5), "atan2 {a}");
            a += 0.01;
        }
        assert!(close(Fx::from_f64(0.3).acos(), 0.3f64.acos(), 5e-5));
    }
}
