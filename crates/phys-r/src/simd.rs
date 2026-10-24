//! Four-lane f32 vectors for BVH culling: SSE on x86 / x86_64, NEON on
//! aarch64, plain arrays elsewhere (e.g. ARMv7 without NEON in the target).
//!
//! Only conservative culling uses these (see `bvh.rs`): the f32 results never
//! decide a simulation outcome, so differences in rounding between
//! instruction sets cannot break determinism.

#![allow(unsafe_code)]

#[cfg(all(any(target_arch = "x86_64", target_arch = "x86"), target_feature = "sse2"))]
mod imp {
    #[cfg(target_arch = "x86")]
    use core::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    use core::arch::x86_64::*;

    #[derive(Copy, Clone)]
    #[repr(transparent)]
    pub struct F4(__m128);

    impl F4 {
        #[inline(always)]
        pub fn load(a: &[f32; 4]) -> F4 {
            // SAFETY: 16 readable bytes; unaligned load.
            F4(unsafe { _mm_loadu_ps(a.as_ptr()) })
        }
        #[inline(always)]
        pub fn splat(x: f32) -> F4 {
            F4(unsafe { _mm_set1_ps(x) })
        }
        #[inline(always)]
        pub fn sub(self, o: F4) -> F4 {
            F4(unsafe { _mm_sub_ps(self.0, o.0) })
        }
        #[inline(always)]
        pub fn mul(self, o: F4) -> F4 {
            F4(unsafe { _mm_mul_ps(self.0, o.0) })
        }
        #[inline(always)]
        pub fn min(self, o: F4) -> F4 {
            F4(unsafe { _mm_min_ps(self.0, o.0) })
        }
        #[inline(always)]
        pub fn max(self, o: F4) -> F4 {
            F4(unsafe { _mm_max_ps(self.0, o.0) })
        }
        /// Bit `l` set where `self[l] <= o[l]`.
        #[inline(always)]
        pub fn le_mask(self, o: F4) -> u32 {
            unsafe { _mm_movemask_ps(_mm_cmple_ps(self.0, o.0)) as u32 }
        }
        #[inline(always)]
        pub fn store(self) -> [f32; 4] {
            let mut r = [0f32; 4];
            unsafe { _mm_storeu_ps(r.as_mut_ptr(), self.0) };
            r
        }
    }
}

#[cfg(all(target_arch = "aarch64", target_feature = "neon"))]
mod imp {
    use core::arch::aarch64::*;

    #[derive(Copy, Clone)]
    #[repr(transparent)]
    pub struct F4(float32x4_t);

    impl F4 {
        #[inline(always)]
        pub fn load(a: &[f32; 4]) -> F4 {
            // SAFETY: 16 readable bytes.
            F4(unsafe { vld1q_f32(a.as_ptr()) })
        }
        #[inline(always)]
        pub fn splat(x: f32) -> F4 {
            F4(unsafe { vdupq_n_f32(x) })
        }
        #[inline(always)]
        pub fn sub(self, o: F4) -> F4 {
            F4(unsafe { vsubq_f32(self.0, o.0) })
        }
        #[inline(always)]
        pub fn mul(self, o: F4) -> F4 {
            F4(unsafe { vmulq_f32(self.0, o.0) })
        }
        #[inline(always)]
        pub fn min(self, o: F4) -> F4 {
            F4(unsafe { vminq_f32(self.0, o.0) })
        }
        #[inline(always)]
        pub fn max(self, o: F4) -> F4 {
            F4(unsafe { vmaxq_f32(self.0, o.0) })
        }
        #[inline(always)]
        pub fn le_mask(self, o: F4) -> u32 {
            unsafe {
                let m = vcleq_f32(self.0, o.0);
                let bits: [u32; 4] = [1, 2, 4, 8];
                vaddvq_u32(vandq_u32(m, vld1q_u32(bits.as_ptr())))
            }
        }
        #[inline(always)]
        pub fn store(self) -> [f32; 4] {
            let mut r = [0f32; 4];
            unsafe { vst1q_f32(r.as_mut_ptr(), self.0) };
            r
        }
    }
}

#[cfg(not(any(
    all(any(target_arch = "x86_64", target_arch = "x86"), target_feature = "sse2"),
    all(target_arch = "aarch64", target_feature = "neon")
)))]
mod imp {
    #[derive(Copy, Clone)]
    pub struct F4([f32; 4]);

    impl F4 {
        #[inline(always)]
        pub fn load(a: &[f32; 4]) -> F4 {
            F4(*a)
        }
        #[inline(always)]
        pub fn splat(x: f32) -> F4 {
            F4([x; 4])
        }
        #[inline(always)]
        fn zip(self, o: F4, f: impl Fn(f32, f32) -> f32) -> F4 {
            F4([f(self.0[0], o.0[0]), f(self.0[1], o.0[1]), f(self.0[2], o.0[2]), f(self.0[3], o.0[3])])
        }
        #[inline(always)]
        pub fn sub(self, o: F4) -> F4 {
            self.zip(o, |a, b| a - b)
        }
        #[inline(always)]
        pub fn mul(self, o: F4) -> F4 {
            self.zip(o, |a, b| a * b)
        }
        #[inline(always)]
        pub fn min(self, o: F4) -> F4 {
            self.zip(o, |a, b| if a < b { a } else { b })
        }
        #[inline(always)]
        pub fn max(self, o: F4) -> F4 {
            self.zip(o, |a, b| if a > b { a } else { b })
        }
        #[inline(always)]
        pub fn le_mask(self, o: F4) -> u32 {
            (0..4).fold(0, |m, l| m | (((self.0[l] <= o.0[l]) as u32) << l))
        }
        #[inline(always)]
        pub fn store(self) -> [f32; 4] {
            self.0
        }
    }
}

pub(crate) use imp::F4;

/// Four boxes in SoA layout.
#[derive(Clone, Copy, Debug)]
#[repr(C, align(16))]
pub(crate) struct Box4 {
    pub lo: [[f32; 4]; 3],
    pub hi: [[f32; 4]; 3],
}

impl Box4 {
    /// Empty lanes never pass any test.
    pub const EMPTY: Box4 = Box4 { lo: [[f32::MAX; 4]; 3], hi: [[-f32::MAX; 4]; 3] };

    #[inline]
    pub fn set(&mut self, l: usize, b: &[f32; 6]) {
        for a in 0..3 {
            self.lo[a][l] = b[a];
            self.hi[a][l] = b[a + 3];
        }
    }

    /// Slab test of four boxes; returns the hit mask and entry distances.
    #[inline(always)]
    pub fn ray(&self, o: &[F4; 3], inv: &[F4; 3], tmax: f32) -> (u32, [f32; 4]) {
        let mut tn = F4::splat(0.0);
        let mut tf = F4::splat(tmax);
        for a in 0..3 {
            let t1 = F4::load(&self.lo[a]).sub(o[a]).mul(inv[a]);
            let t2 = F4::load(&self.hi[a]).sub(o[a]).mul(inv[a]);
            tn = tn.max(t1.min(t2));
            tf = tf.min(t1.max(t2));
        }
        (tn.le_mask(tf), tn.store())
    }

    /// Overlap mask with the box `q` (`[min xyz, max xyz]`).
    #[inline(always)]
    pub fn overlap(&self, q: &[F4; 6]) -> u32 {
        let mut m = 0xF;
        for a in 0..3 {
            m &= F4::load(&self.lo[a]).le_mask(q[a + 3]) & q[a].le_mask(F4::load(&self.hi[a]));
        }
        m
    }
}
