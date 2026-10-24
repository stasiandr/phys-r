//! Minimal persistent thread pool (std only, no dependencies).
//!
//! Items have **affinity**: thread `t` of `T` first runs the items
//! `i ≡ t (mod T)`, then steals whatever is left. Callers keep item indices
//! stable between ticks (character `i`, shot `i`), so the same core keeps
//! working on the same part of the level and finds it in its own cache.
//!
//! Each item is claimed with one compare-and-swap on a per-slot epoch tag
//! (`epoch - 1` → `epoch`). Slots are reset to `epoch - 1` before a job is
//! published, so a worker that wakes late for an old job can never claim an
//! item of a newer one. Between jobs workers spin briefly (a step issues
//! several parallel phases back to back), then park on a condvar.
//!
//! Every parallel phase in the engine is written so that its result does not
//! depend on how items are distributed: items only read shared state and
//! write their own output slot, and outputs are applied in index order.
//! Simulation results are therefore identical for any thread count.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

type Task = *const (dyn Fn(usize) + Sync);

struct Job {
    epoch: u32,
    task: Option<SendPtr>,
    n: usize,
}

#[derive(Copy, Clone)]
struct SendPtr(Task);
// SAFETY: the pointee is `Sync` and outlives every use (see `Pool::run`).
unsafe impl Send for SendPtr {}

/// Items per published job; larger runs are split into windows.
const SLOTS: usize = 1024;

#[repr(align(64))]
struct Padded<T>(T);

struct Shared {
    job: Mutex<Job>,
    cv: Condvar,
    /// Latest published epoch (for the spin phase).
    epoch: Padded<AtomicU32>,
    done: Padded<AtomicUsize>,
    shutdown: AtomicBool,
    threads: usize,
    claims: Box<[AtomicU32]>,
}

pub(crate) struct Pool {
    shared: Arc<Shared>,
    handles: Vec<JoinHandle<()>>,
    epoch: u32,
}

/// Spin iterations before a worker parks (roughly tens of microseconds).
const SPIN: u32 = 1 << 14;

impl Pool {
    /// `threads` total workers including the calling thread (`<= 1`: none).
    pub fn new(threads: usize) -> Pool {
        let threads = threads.max(1);
        let shared = Arc::new(Shared {
            job: Mutex::new(Job { epoch: 0, task: None, n: 0 }),
            cv: Condvar::new(),
            epoch: Padded(AtomicU32::new(0)),
            done: Padded(AtomicUsize::new(0)),
            shutdown: AtomicBool::new(false),
            threads,
            claims: (0..SLOTS).map(|_| AtomicU32::new(0)).collect(),
        });
        let mut handles = Vec::new();
        for k in 1..threads {
            let sh = shared.clone();
            let h = std::thread::Builder::new().name(format!("phys-r-{k}")).spawn(move || worker(sh, k));
            match h {
                Ok(h) => handles.push(h),
                Err(_) => break,
            }
        }
        // Thread ids must be dense: if spawning failed, run with fewer.
        if handles.len() + 1 != threads {
            let n = handles.len() + 1;
            drop(Pool { shared, handles, epoch: 0 });
            return Pool::new(n);
        }
        Pool { shared, handles, epoch: 0 }
    }

    pub fn threads(&self) -> usize {
        self.handles.len() + 1
    }

    /// Run `f(0..n)` across the pool (the caller participates) and return
    /// when every item has finished.
    pub fn run(&mut self, n: usize, f: &(dyn Fn(usize) + Sync)) {
        if self.handles.is_empty() || n <= 1 {
            for i in 0..n {
                f(i);
            }
            return;
        }
        if n <= SLOTS {
            self.run_window(n, f);
            return;
        }
        let mut base = 0;
        while base < n {
            let m = SLOTS.min(n - base);
            self.run_window(m, &|i| f(base + i));
            base += m;
        }
    }

    fn run_window(&mut self, n: usize, f: &(dyn Fn(usize) + Sync)) {
        self.epoch = self.epoch.wrapping_add(1);
        let epoch = self.epoch;
        let sh = &*self.shared;
        for c in &sh.claims[..n] {
            c.store(epoch.wrapping_sub(1), Ordering::Relaxed);
        }
        // SAFETY: erase the lifetime; `f` stays alive until `done == n`
        // below, and no worker dereferences the task after its epoch ends.
        let task: Task = unsafe { core::mem::transmute::<&(dyn Fn(usize) + Sync), &'static (dyn Fn(usize) + Sync)>(f) };
        {
            let mut j = sh.job.lock().unwrap_or_else(|e| e.into_inner());
            j.epoch = epoch;
            j.task = Some(SendPtr(task));
            j.n = n;
            sh.done.0.store(0, Ordering::Relaxed);
            sh.epoch.0.store(epoch, Ordering::Release);
        }
        sh.cv.notify_all();
        let did = drain(sh, 0, epoch, n, task);
        if did > 0 {
            sh.done.0.fetch_add(did, Ordering::AcqRel);
        }
        let mut spins = 0u32;
        while sh.done.0.load(Ordering::Acquire) < n {
            spins = spins.wrapping_add(1);
            if spins < SPIN {
                std::hint::spin_loop();
            } else {
                std::thread::yield_now();
            }
        }
        let mut j = sh.job.lock().unwrap_or_else(|e| e.into_inner());
        j.task = None;
    }

    /// Run `f(i, &mut items[i])` for every item in parallel.
    pub fn for_each_mut<T: Send>(&mut self, items: &mut [T], f: &(dyn Fn(usize, &mut T) + Sync)) {
        let base = SyncPtr(items.as_mut_ptr());
        let n = items.len();
        self.run(n, &move |i| {
            let b = base;
            // SAFETY: each index is handed out exactly once, so the
            // `&mut` borrows are disjoint; `items` outlives `run`.
            let item = unsafe { &mut *b.0.add(i) };
            f(i, item);
        });
    }

    /// Run `f(start, &mut items[start..start + chunk])` for consecutive
    /// chunks in parallel (the last one may be shorter).
    pub fn for_each_chunk_mut<T: Send>(&mut self, items: &mut [T], chunk: usize, f: &(dyn Fn(usize, &mut [T]) + Sync)) {
        let chunk = chunk.max(1);
        let base = SyncPtr(items.as_mut_ptr());
        let len = items.len();
        self.run(len.div_ceil(chunk), &move |c| {
            let b = base;
            let start = c * chunk;
            let n = chunk.min(len - start);
            // SAFETY: chunks are disjoint and each is handed out once;
            // `items` outlives `run`.
            let s = unsafe { core::slice::from_raw_parts_mut(b.0.add(start), n) };
            f(start, s);
        });
    }
}

struct SyncPtr<T>(*mut T);
impl<T> Clone for SyncPtr<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Copy for SyncPtr<T> {}
unsafe impl<T: Send> Sync for SyncPtr<T> {}
unsafe impl<T: Send> Send for SyncPtr<T> {}

/// Run the items of `epoch` this thread can claim: its own residue class
/// first, then any left over. Returns how many it ran.
fn drain(sh: &Shared, me: usize, epoch: u32, n: usize, task: Task) -> usize {
    let prev = epoch.wrapping_sub(1);
    let claim = |i: usize| {
        let c = &sh.claims[i];
        c.load(Ordering::Relaxed) == prev && c.compare_exchange(prev, epoch, Ordering::AcqRel, Ordering::Relaxed).is_ok()
    };
    let t = sh.threads;
    let mut did = 0;
    for k in 0..t {
        let mut i = (me + k) % t;
        while i < n {
            if claim(i) {
                // SAFETY: the job of `epoch` is still running (it cannot
                // finish before this item is counted as done).
                unsafe { (*task)(i) };
                did += 1;
            }
            i += t;
        }
    }
    did
}

fn worker(sh: Arc<Shared>, me: usize) {
    let mut seen: u32 = 0;
    loop {
        // Spin briefly for the next job, then park.
        let mut spins = 0;
        while sh.epoch.0.load(Ordering::Acquire) == seen && spins < SPIN && !sh.shutdown.load(Ordering::Relaxed) {
            std::hint::spin_loop();
            spins += 1;
        }
        let (epoch, task, n) = {
            let mut j = sh.job.lock().unwrap_or_else(|e| e.into_inner());
            while (j.epoch == seen || j.task.is_none()) && !sh.shutdown.load(Ordering::Relaxed) {
                j = sh.cv.wait(j).unwrap_or_else(|e| e.into_inner());
            }
            if sh.shutdown.load(Ordering::Relaxed) {
                return;
            }
            (j.epoch, j.task.unwrap(), j.n)
        };
        seen = epoch;
        let did = drain(&sh, me, epoch, n, task.0);
        if did > 0 {
            sh.done.0.fetch_add(did, Ordering::AcqRel);
        }
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Ordering::Release);
        {
            let _j = self.shared.job.lock().unwrap_or_else(|e| e.into_inner());
        }
        self.shared.cv.notify_all();
        for h in self.handles.drain(..) {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU64;

    #[test]
    fn runs_every_item_once() {
        let mut p = Pool::new(4);
        for round in 0..300u64 {
            let n = [1, 2, 5, 16, 37, 1500, 2048][round as usize % 7];
            let hits: Vec<AtomicU64> = (0..n).map(|_| AtomicU64::new(0)).collect();
            p.run(n, &|i| {
                hits[i].fetch_add(round + 1, Ordering::Relaxed);
            });
            assert!(hits.iter().all(|h| h.load(Ordering::Relaxed) == round + 1));
        }
        let mut v = vec![0u32; 100];
        p.for_each_mut(&mut v, &|i, x| *x = i as u32 * 2);
        assert!(v.iter().enumerate().all(|(i, x)| *x == i as u32 * 2));
        let mut v = vec![0u32; 103];
        p.for_each_chunk_mut(&mut v, 4, &|s, xs| xs.iter_mut().enumerate().for_each(|(k, x)| *x = (s + k) as u32));
        assert!(v.iter().enumerate().all(|(i, x)| *x == i as u32));
    }
}
