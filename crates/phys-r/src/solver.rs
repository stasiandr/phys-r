//! Sequential-impulse contact solver with warm starting, central friction
//! and split-impulse position correction.
//!
//! Jacobians (`r x n` and `I^-1 (r x n)`) are precomputed once per step so
//! each solver iteration only needs a handful of dot products per point.

use crate::collide::MAX_POINTS;
use crate::fixed::Fx;
use crate::math::{Mat3, Vec3};

/// Persistent contact manifold between two bodies (`a < b` for body pairs;
/// for static pairs `a` is the moving body). `sub` distinguishes per-triangle
/// manifolds of mesh pairs.
#[derive(Copy, Clone, Debug, Default)]
pub struct Manifold {
    pub a: u32,
    pub b: u32,
    pub sub: u32,
    /// Unit normal from `a` to `b`.
    pub normal: Vec3,
    pub count: u8,
    pub points: [ManifoldPoint; MAX_POINTS],
    /// Accumulated friction impulse at the manifold centre (two tangents).
    pub friction_impulse: [Fx; 2],
    /// Accumulated twist friction impulse about the normal.
    pub twist_impulse: Fx,
}

impl Manifold {
    #[inline]
    pub fn key(&self) -> (u32, u32, u32) {
        (self.a, self.b, self.sub)
    }

    #[inline]
    pub fn points(&self) -> &[ManifoldPoint] {
        &self.points[..self.count as usize]
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ManifoldPoint {
    pub point: Vec3,
    pub depth: Fx,
    pub id: u32,
    pub normal_impulse: Fx,
}

/// Per-body solver data, split into hot (velocities) and cold (mass) arrays.
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct Vel {
    pub v: Vec3,
    pub w: Vec3,
}

#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct MassProps {
    pub inv_mass: Fx,
    pub inv_i: Mat3,
}

/// One scalar velocity constraint row along `dir` with anchors `ra`, `rb`.
#[derive(Copy, Clone, Debug, Default)]
struct Row {
    /// `ra x dir`, `rb x dir`.
    ja: Vec3,
    jb: Vec3,
    /// `I_a^-1 (ra x dir)`, `I_b^-1 (rb x dir)`.
    ia: Vec3,
    ib: Vec3,
    mass: Fx,
}

impl Row {
    #[inline]
    fn new(ba: &MassProps, bb: &MassProps, ra: Vec3, rb: Vec3, dir: Vec3) -> Row {
        let ja = ra.cross(dir);
        let jb = rb.cross(dir);
        let ia = ba.inv_i.mul_vec(ja);
        let ib = bb.inv_i.mul_vec(jb);
        let k = ba.inv_mass + bb.inv_mass + ja.dot(ia) + jb.dot(ib);
        Row { ja, jb, ia, ib, mass: if k.0 > 0 { Fx::ONE / k } else { Fx::ZERO } }
    }

    /// Relative velocity along the row.
    #[inline(always)]
    fn velocity(&self, dir: Vec3, a: &Vel, b: &Vel) -> Fx {
        (b.v - a.v).dot(dir) + b.w.dot(self.jb) - a.w.dot(self.ja)
    }
}

#[derive(Copy, Clone, Debug, Default)]
struct PointC {
    row: Row,
    /// Velocity bias (speculative gap closing / restitution).
    bias: Fx,
    /// Position-correction bias for the split-impulse pass.
    pbias: Fx,
    ni: Fx,
    pi: Fx,
}

#[derive(Copy, Clone, Debug, Default)]
struct Constraint {
    a: u32,
    b: u32,
    manifold: u32,
    normal: Vec3,
    t: [Vec3; 2],
    friction: Fx,
    count: u8,
    ma: Fx,
    mb: Fx,
    points: [PointC; MAX_POINTS],
    // Central friction: rows at the manifold centre plus twist about n.
    trow: [Row; 2],
    twist_ia: Vec3,
    twist_ib: Vec3,
    twist_mass: Fx,
    /// Average distance of the points from the centre (twist lever arm).
    radius: Fx,
    fi: [Fx; 2],
    twi: Fx,
}

pub(crate) struct SolverParams {
    pub inv_dt: Fx,
    pub slop: Fx,
    pub baumgarte: Fx,
    pub max_correction: Fx,
    pub restitution_threshold: Fx,
}

/// Contact solver scratch storage (reused across steps; no per-step allocs).
#[derive(Default)]
pub(crate) struct Solver {
    pub vel: Vec<Vel>,
    /// Split-impulse pseudo velocities (position correction only).
    pub pvel: Vec<Vel>,
    mass: Vec<MassProps>,
    constraints: Vec<Constraint>,
}

impl Solver {
    /// Index 0 of `bodies` is the immovable dummy body.
    pub fn reset(&mut self) {
        self.vel.clear();
        self.pvel.clear();
        self.mass.clear();
        self.constraints.clear();
        self.push_body(Vec3::ZERO, Vec3::ZERO, Fx::ZERO, Mat3::ZERO);
    }

    /// Returns the solver index of the new body.
    pub fn push_body(&mut self, v: Vec3, w: Vec3, inv_mass: Fx, inv_i: Mat3) -> u32 {
        self.vel.push(Vel { v, w });
        self.pvel.push(Vel::default());
        self.mass.push(MassProps { inv_mass, inv_i });
        self.vel.len() as u32 - 1
    }

    /// Add a constraint for manifold `mi` between solver bodies `sa`/`sb`.
    #[allow(clippy::too_many_arguments)]
    #[inline(never)]
    pub fn add(&mut self, mi: u32, m: &Manifold, sa: u32, sb: u32, xa: Vec3, xb: Vec3, friction: Fx, restitution: Fx, p: &SolverParams) {
        let n = m.normal;
        let (t1, t2) = n.orthonormal_basis();
        let ba = self.mass[sa as usize];
        let bb = self.mass[sb as usize];
        let (va, vb) = (self.vel[sa as usize], self.vel[sb as usize]);
        let mut c = Constraint {
            a: sa,
            b: sb,
            manifold: mi,
            normal: n,
            t: [t1, t2],
            friction,
            count: m.count,
            ma: ba.inv_mass,
            mb: bb.inv_mass,
            ..Default::default()
        };
        let mut center = Vec3::ZERO;
        for (k, mp) in m.points().iter().enumerate() {
            center += mp.point;
            let row = Row::new(&ba, &bb, mp.point - xa, mp.point - xb, n);
            let s = -mp.depth + p.slop;
            // Speculative contacts may close their gap this step. Penetration
            // is resolved separately with split impulses so that position
            // correction never adds kinetic energy (keeps stacks quiet).
            let mut bias = s.max(Fx::ZERO) * p.inv_dt;
            let pbias = if s < Fx::ZERO { (p.baumgarte * p.inv_dt * s).max(-p.max_correction) } else { Fx::ZERO };
            if restitution > Fx::ZERO {
                let vn = row.velocity(n, &va, &vb);
                if vn < -p.restitution_threshold {
                    bias = bias.min(restitution * vn);
                }
            }
            c.points[k] = PointC { row, bias, pbias, ni: mp.normal_impulse, pi: Fx::ZERO };
        }
        let count = Fx::from_int(m.count.max(1) as i32);
        center = Vec3::new(center.x / count, center.y / count, center.z / count);
        let mut radius = Fx::ZERO;
        for mp in m.points() {
            radius += (mp.point - center).length();
        }
        c.radius = radius / count;
        let (ca, cb) = (center - xa, center - xb);
        c.trow = [Row::new(&ba, &bb, ca, cb, t1), Row::new(&ba, &bb, ca, cb, t2)];
        c.twist_ia = ba.inv_i.mul_vec(n);
        c.twist_ib = bb.inv_i.mul_vec(n);
        let kt = n.dot(c.twist_ia) + n.dot(c.twist_ib);
        c.twist_mass = if kt.0 > 0 { Fx::ONE / kt } else { Fx::ZERO };
        // Friction is warm started at half strength: full-strength friction
        // warm starting sets up a slow rocking limit cycle in tall stacks.
        c.fi = [m.friction_impulse[0].shr(1), m.friction_impulse[1].shr(1)];
        c.twi = m.twist_impulse.shr(1);
        self.constraints.push(c);
    }

    #[inline(always)]
    #[allow(clippy::too_many_arguments)]
    fn apply(vel: &mut [Vel], c_a: usize, c_b: usize, ma: Fx, mb: Fx, dir: Vec3, row: &Row, lambda: Fx) {
        let imp = dir.scale(lambda);
        if c_a != 0 {
            let ba = &mut vel[c_a];
            ba.v -= imp.scale(ma);
            ba.w -= row.ia.scale(lambda);
        }
        if c_b != 0 {
            let bb = &mut vel[c_b];
            bb.v += imp.scale(mb);
            bb.w += row.ib.scale(lambda);
        }
    }

    #[inline(always)]
    fn apply_twist(vel: &mut [Vel], a: usize, b: usize, c: &Constraint, lambda: Fx) {
        if a != 0 {
            vel[a].w -= c.twist_ia.scale(lambda);
        }
        if b != 0 {
            vel[b].w += c.twist_ib.scale(lambda);
        }
    }

    #[inline(never)]
    pub fn warm_start(&mut self) {
        let vel = &mut self.vel;
        for c in &self.constraints {
            let (a, b) = (c.a as usize, c.b as usize);
            for p in &c.points[..c.count as usize] {
                if p.ni.0 != 0 {
                    Self::apply(vel, a, b, c.ma, c.mb, c.normal, &p.row, p.ni);
                }
            }
            for k in 0..2 {
                if c.fi[k].0 != 0 {
                    Self::apply(vel, a, b, c.ma, c.mb, c.t[k], &c.trow[k], c.fi[k]);
                }
            }
            if c.twi.0 != 0 {
                Self::apply_twist(vel, a, b, c, c.twi);
            }
        }
    }

    #[inline(never)]
    pub fn solve(&mut self, iterations: u32) {
        let vel = &mut self.vel;
        for _ in 0..iterations {
            for c in self.constraints.iter_mut() {
                let (a, b) = (c.a as usize, c.b as usize);
                // Friction at the manifold centre, bounded by the total normal
                // impulse (more stable for stacks than per-point friction).
                let mut total = Fx::ZERO;
                for p in &c.points[..c.count as usize] {
                    total += p.ni;
                }
                let max_f = c.friction * total;
                // Untouched speculative contacts carry no friction: skip
                // (identical result, since the clamp would yield zero).
                if max_f.0 != 0 || (c.fi[0].0 | c.fi[1].0 | c.twi.0) != 0 {
                    for k in 0..2 {
                        let vt = c.trow[k].velocity(c.t[k], &vel[a], &vel[b]);
                        let old = c.fi[k];
                        c.fi[k] = (old - c.trow[k].mass * vt).clamp(-max_f, max_f);
                        let d = c.fi[k] - old;
                        if d.0 != 0 {
                            Self::apply(vel, a, b, c.ma, c.mb, c.t[k], &c.trow[k], d);
                        }
                    }
                    if c.twist_mass.0 > 0 {
                        let max_t = max_f * c.radius;
                        let wrel = (vel[b].w - vel[a].w).dot(c.normal);
                        let old = c.twi;
                        c.twi = (old - c.twist_mass * wrel).clamp(-max_t, max_t);
                        let d = c.twi - old;
                        if d.0 != 0 {
                            Self::apply_twist(vel, a, b, c, d);
                        }
                    }
                }
                let n = c.normal;
                for p in c.points[..c.count as usize].iter_mut() {
                    let vn = p.row.velocity(n, &vel[a], &vel[b]);
                    let old = p.ni;
                    p.ni = (old - p.row.mass * (vn + p.bias)).max(Fx::ZERO);
                    let d = p.ni - old;
                    if d.0 != 0 {
                        Self::apply(vel, a, b, c.ma, c.mb, n, &p.row, d);
                    }
                }
            }
        }
    }

    /// Split-impulse position correction on pseudo velocities.
    #[inline(never)]
    pub fn solve_positions(&mut self, iterations: u32) {
        let pvel = &mut self.pvel;
        for _ in 0..iterations {
            for c in self.constraints.iter_mut() {
                let (a, b) = (c.a as usize, c.b as usize);
                let n = c.normal;
                for p in c.points[..c.count as usize].iter_mut() {
                    if p.pbias.0 == 0 {
                        continue;
                    }
                    let vn = p.row.velocity(n, &pvel[a], &pvel[b]);
                    let old = p.pi;
                    p.pi = (old - p.row.mass * (vn + p.pbias)).max(Fx::ZERO);
                    let d = p.pi - old;
                    if d.0 != 0 {
                        Self::apply(pvel, a, b, c.ma, c.mb, n, &p.row, d);
                    }
                }
            }
        }
    }

    /// Copy accumulated impulses back for warm starting next step.
    #[inline(never)]
    pub fn store_impulses(&self, manifolds: &mut [Manifold]) {
        for c in &self.constraints {
            let m = &mut manifolds[c.manifold as usize];
            for (k, p) in c.points[..c.count as usize].iter().enumerate() {
                m.points[k].normal_impulse = p.ni;
            }
            m.friction_impulse = c.fi;
            m.twist_impulse = c.twi;
        }
    }
}
