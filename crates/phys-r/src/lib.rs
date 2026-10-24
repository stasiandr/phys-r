//! # phys-r
//!
//! A small, fast, **deterministic** 3D physics engine aimed at networked
//! first-person shooters on mobile devices.
//!
//! * Fixed-point math ([`Fx`], Q40.24): bit-identical results on every
//!   platform, compiler and optimisation level — required for client-side
//!   prediction, rollback and server reconciliation.
//! * Rigid bodies (sphere, capsule, box) with a warm-started sequential
//!   impulse solver, speculative contacts, friction, restitution, sleeping.
//! * Static level geometry built from primitives in a BVH (no triangle meshes:
//!   boxes are faster and every pair has a dedicated routine).
//! * Kinematic FPS character controller (move & slide, steps, slopes, snap).
//! * Raycasts, sphere/capsule casts, overlaps, sensors and contact events.
//! * Lag compensation (pose history + rewound raycasts).
//! * Snapshots / restore and state hashing for desync detection.
//!
//! No dependencies, no allocation in the steady-state step.

// Index loops over x/y/z axes read more clearly than iterator chains here.
#![allow(clippy::needless_range_loop)]

pub mod body;
pub mod bvh;
pub mod character;
pub mod collide;
pub mod fixed;
pub mod geometry;
pub mod history;
pub mod math;
pub mod query;
pub mod shape;
pub mod snapshot;
pub mod solver;
pub mod world;

pub use body::{flags, Body, BodyDesc, BodyHandle, BodyKind};
pub use character::{collision, Character, CharacterDesc, CharacterHandle, CharacterInput, MoveMode, MoveResult, MovementSettings};
pub use fixed::Fx;
pub use geometry::Aabb;
pub use math::{Iso, Mat3, Quat, Vec3};
pub use query::{QueryFilter, RayHit, SweepHit};
pub use shape::{Shape, ShapeId};
pub use snapshot::SnapshotError;
pub use solver::Manifold;
pub use world::{Event, EventKind, World, WorldError, WorldSettings};
