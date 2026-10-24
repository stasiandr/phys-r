//! Lag compensation: a ring buffer of past poses for bodies flagged
//! [`flags::HISTORY`](crate::body::flags::HISTORY), plus rewound raycasts.
//!
//! Typical server usage: when a client fires at its render tick `T + alpha`,
//! call [`World::raycast_rewind`] with that tick to test against where the
//! targets were on the shooter's screen.

use crate::body::{flags, BodyHandle};
use crate::fixed::Fx;
use crate::geometry::Ray;
use crate::math::{Iso, Quat, Vec3};
use crate::query::{ray_shape, QueryFilter, RayHit};
use crate::world::World;

#[derive(Copy, Clone, Debug)]
struct Entry {
    index: u32,
    generation: u32,
    pos: Vec3,
    rot: Quat,
}

#[derive(Clone, Debug, Default)]
struct Frame {
    tick: u32,
    valid: bool,
    entries: Vec<Entry>,
}

#[derive(Clone, Debug, Default)]
pub struct History {
    frames: Vec<Frame>,
}

impl History {
    pub fn new(capacity: usize) -> History {
        History { frames: vec![Frame::default(); capacity] }
    }

    fn frame(&self, tick: u32) -> Option<&Frame> {
        if self.frames.is_empty() {
            return None;
        }
        let f = &self.frames[tick as usize % self.frames.len()];
        if f.valid && f.tick == tick {
            Some(f)
        } else {
            None
        }
    }

    pub fn clear(&mut self) {
        for f in self.frames.iter_mut() {
            f.valid = false;
            f.entries.clear();
        }
    }
}

impl World {
    pub(crate) fn record_history(&mut self) {
        if self.history.frames.is_empty() {
            return;
        }
        let tick = self.tick;
        let n = self.history.frames.len();
        let frame = &mut self.history.frames[tick as usize % n];
        frame.tick = tick;
        frame.valid = true;
        frame.entries.clear();
        // Only moving bodies change pose; static ones are tested at their
        // (unchanging) current pose by `raycast_rewind`.
        for &i in &self.movers {
            let b = &self.bodies[i as usize];
            if b.flags & flags::HISTORY != 0 {
                frame.entries.push(Entry { index: i, generation: b.generation, pos: b.position, rot: b.rotation });
            }
        }
        // `movers` is in sweep order; rewind lookups binary-search by index.
        frame.entries.sort_unstable_by_key(|e| e.index);
    }

    /// Pose of a recorded body at `tick + alpha` (alpha in [0, 1)).
    pub fn historic_pose(&self, body: BodyHandle, tick: u32, alpha: Fx) -> Option<Iso> {
        let f0 = self.history.frame(tick)?;
        let e0 = find(f0, body)?;
        let f1 = if alpha.0 > 0 { self.history.frame(tick.wrapping_add(1)) } else { None };
        Some(match f1.and_then(|f| find(f, body)) {
            Some(e1) => Iso::new(Vec3::lerp(e0.pos, e1.pos, alpha), Quat::nlerp(e0.rot, e1.rot, alpha)),
            None => Iso::new(e0.pos, e0.rot),
        })
    }

    /// Raycast where history-tracked bodies are tested at their poses from
    /// `tick + alpha` and everything else (static world, untracked bodies) at
    /// its current pose. Falls back to a normal raycast if the tick is no
    /// longer in the buffer.
    pub fn raycast_rewind(&self, tick: u32, alpha: Fx, origin: Vec3, dir: Vec3, max_dist: Fx, filter: &QueryFilter) -> Option<RayHit> {
        let Some(f0) = self.history.frame(tick) else {
            return self.raycast(origin, dir, max_dist, filter);
        };
        let f1 = if alpha.0 > 0 { self.history.frame(tick.wrapping_add(1)) } else { None };

        // Everything except tracked bodies, at current poses.
        let mut best = {
            let mut f = *filter;
            f.include_moving = false;
            self.raycast(origin, dir, max_dist, &f)
        };
        let ray = Ray::new(origin, dir);
        let mut max_t = best.map_or(max_dist, |h| h.distance);
        if filter.include_moving {
            // Untracked moving bodies at current pose.
            for &idx in &self.movers {
                let b = &self.bodies[idx as usize];
                if b.flags & flags::HISTORY != 0 || !self.filter_accepts(idx, filter) {
                    continue;
                }
                let iso = b.iso();
                if let Some((t, n)) = ray_shape(&self.shapes[b.shape.0 as usize], &iso, &ray, max_t) {
                    if t < max_t {
                        max_t = t;
                        best = Some(self.make_hit(idx, &iso, &ray, t, n));
                    }
                }
            }
            // Tracked bodies at their historic poses.
            for e in &f0.entries {
                let Some(b) = self.bodies.get(e.index as usize) else { continue };
                if !b.alive || b.generation != e.generation || !self.filter_accepts(e.index, filter) {
                    continue;
                }
                let iso = match f1.and_then(|f| find(f, BodyHandle { index: e.index, generation: e.generation })) {
                    Some(e1) => Iso::new(Vec3::lerp(e.pos, e1.pos, alpha), Quat::nlerp(e.rot, e1.rot, alpha)),
                    None => Iso::new(e.pos, e.rot),
                };
                if let Some((t, n)) = ray_shape(&self.shapes[b.shape.0 as usize], &iso, &ray, max_t) {
                    if t < max_t {
                        max_t = t;
                        // `local_point` is relative to the rewound pose (hit zones).
                        best = Some(self.make_hit(e.index, &iso, &ray, t, n));
                    }
                }
            }
        }
        best
    }

    /// Drop all recorded history (e.g. after a full state resync).
    pub fn clear_history(&mut self) {
        self.history.clear();
    }
}

fn find(f: &Frame, h: BodyHandle) -> Option<&Entry> {
    f.entries.binary_search_by_key(&h.index, |e| e.index).ok().map(|i| &f.entries[i]).filter(|e| e.generation == h.generation)
}
