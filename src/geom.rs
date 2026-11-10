//! Solid geometry: BSP-tree boolean operations (union / subtract / intersect), extrusion,
//! revolution, primitives, mesh finishing (welding, T-junction repair, triangulation),
//! picking helpers and STL / OBJ reading and writing.

use glam::{DMat3, DQuat, DVec2, DVec3};
use std::collections::HashMap;

pub const EPS: f64 = 1e-6;

// ---------------------------------------------------------------- polygons & BSP CSG

#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub n: DVec3,
    pub w: f64,
}

impl Plane {
    pub fn from_points(a: DVec3, b: DVec3, c: DVec3) -> Option<Plane> {
        let n = (b - a).cross(c - a);
        let l = n.length();
        if l < 1e-12 {
            return None;
        }
        let n = n / l;
        Some(Plane { n, w: n.dot(a) })
    }
    fn flip(&mut self) {
        self.n = -self.n;
        self.w = -self.w;
    }
}

#[derive(Clone, Debug)]
pub struct Poly {
    pub v: Vec<DVec3>,
    pub plane: Plane,
}

impl Poly {
    pub fn new(v: Vec<DVec3>) -> Option<Poly> {
        if v.len() < 3 {
            return None;
        }
        // Newell normal: robust for any planar polygon
        let mut n = DVec3::ZERO;
        for i in 0..v.len() {
            let a = v[i];
            let b = v[(i + 1) % v.len()];
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
        let l = n.length();
        if l < 1e-12 {
            return None;
        }
        let n = n / l;
        let c = v.iter().fold(DVec3::ZERO, |s, p| s + *p) / v.len() as f64;
        Some(Poly { plane: Plane { n, w: n.dot(c) }, v })
    }
    fn flip(&mut self) {
        self.v.reverse();
        self.plane.flip();
    }
}

#[derive(Clone, Debug, Default)]
pub struct Solid {
    pub polys: Vec<Poly>,
}

const COPLANAR: u8 = 0;
const FRONT: u8 = 1;
const BACK: u8 = 2;
const SPANNING: u8 = 3;
const SPLIT_EPS: f64 = 1e-5;

fn split_polygon(
    plane: &Plane,
    poly: &Poly,
    cf: &mut Vec<Poly>,
    cb: &mut Vec<Poly>,
    f: &mut Vec<Poly>,
    b: &mut Vec<Poly>,
) {
    let mut ptype = 0u8;
    let mut types = Vec::with_capacity(poly.v.len());
    for p in &poly.v {
        let t = plane.n.dot(*p) - plane.w;
        let ty = if t < -SPLIT_EPS { BACK } else if t > SPLIT_EPS { FRONT } else { COPLANAR };
        ptype |= ty;
        types.push(ty);
    }
    match ptype {
        COPLANAR => {
            if plane.n.dot(poly.plane.n) > 0.0 {
                cf.push(poly.clone())
            } else {
                cb.push(poly.clone())
            }
        }
        FRONT => f.push(poly.clone()),
        BACK => b.push(poly.clone()),
        _ => {
            let mut fv = Vec::new();
            let mut bv = Vec::new();
            let n = poly.v.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let (ti, tj) = (types[i], types[j]);
                let (vi, vj) = (poly.v[i], poly.v[j]);
                if ti != BACK {
                    fv.push(vi);
                }
                if ti != FRONT {
                    bv.push(vi);
                }
                if (ti | tj) == SPANNING {
                    let t = (plane.w - plane.n.dot(vi)) / plane.n.dot(vj - vi);
                    let v = vi.lerp(vj, t);
                    fv.push(v);
                    bv.push(v);
                }
            }
            if fv.len() >= 3 {
                f.push(Poly { v: fv, plane: poly.plane });
            }
            if bv.len() >= 3 {
                b.push(Poly { v: bv, plane: poly.plane });
            }
        }
    }
}

#[derive(Default)]
struct Node {
    plane: Option<Plane>,
    front: Option<Box<Node>>,
    back: Option<Box<Node>>,
    polys: Vec<Poly>,
}

impl Node {
    fn new(polys: Vec<Poly>) -> Node {
        let mut n = Node::default();
        n.build(polys);
        n
    }
    fn invert(&mut self) {
        for p in &mut self.polys {
            p.flip();
        }
        if let Some(pl) = &mut self.plane {
            pl.flip();
        }
        if let Some(f) = &mut self.front {
            f.invert();
        }
        if let Some(b) = &mut self.back {
            b.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }
    fn clip_polygons(&self, polys: Vec<Poly>) -> Vec<Poly> {
        let plane = match &self.plane {
            Some(p) => p,
            None => return polys,
        };
        let mut f = Vec::new();
        let mut b = Vec::new();
        for p in &polys {
            let mut cf = Vec::new();
            let mut cb = Vec::new();
            split_polygon(plane, p, &mut cf, &mut cb, &mut f, &mut b);
            f.extend(cf);
            b.extend(cb);
        }
        let mut f = match &self.front {
            Some(n) => n.clip_polygons(f),
            None => f,
        };
        let b = match &self.back {
            Some(n) => n.clip_polygons(b),
            None => Vec::new(),
        };
        f.extend(b);
        f
    }
    fn clip_to(&mut self, bsp: &Node) {
        self.polys = bsp.clip_polygons(std::mem::take(&mut self.polys));
        if let Some(f) = &mut self.front {
            f.clip_to(bsp);
        }
        if let Some(b) = &mut self.back {
            b.clip_to(bsp);
        }
    }
    fn all_polygons(&self) -> Vec<Poly> {
        let mut out = self.polys.clone();
        if let Some(f) = &self.front {
            out.extend(f.all_polygons());
        }
        if let Some(b) = &self.back {
            out.extend(b.all_polygons());
        }
        out
    }
    fn build(&mut self, polys: Vec<Poly>) {
        if polys.is_empty() {
            return;
        }
        if self.plane.is_none() {
            // pick a splitter near the middle of the list (better balance than the first)
            self.plane = Some(polys[polys.len() / 2].plane);
        }
        let plane = self.plane.unwrap();
        let mut f = Vec::new();
        let mut b = Vec::new();
        let mut cf = Vec::new();
        let mut cb = Vec::new();
        for p in &polys {
            split_polygon(&plane, p, &mut cf, &mut cb, &mut f, &mut b);
        }
        self.polys.extend(cf);
        self.polys.extend(cb);
        if !f.is_empty() {
            self.front.get_or_insert_with(Default::default).build(f);
        }
        if !b.is_empty() {
            self.back.get_or_insert_with(Default::default).build(b);
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: DVec3,
    pub max: DVec3,
}

impl Default for Aabb {
    fn default() -> Self {
        Aabb::empty()
    }
}

impl Aabb {
    pub fn empty() -> Aabb {
        Aabb { min: DVec3::splat(f64::MAX), max: DVec3::splat(f64::MIN) }
    }
    pub fn add(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }
    pub fn valid(&self) -> bool {
        self.min.x <= self.max.x
    }
    pub fn overlaps(&self, o: &Aabb) -> bool {
        let e = 1e-6;
        self.min.x <= o.max.x + e
            && self.max.x >= o.min.x - e
            && self.min.y <= o.max.y + e
            && self.max.y >= o.min.y - e
            && self.min.z <= o.max.z + e
            && self.max.z >= o.min.z - e
    }
    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }
    pub fn size(&self) -> DVec3 {
        if self.valid() { self.max - self.min } else { DVec3::ZERO }
    }
    pub fn merge(&mut self, o: &Aabb) {
        if o.valid() {
            self.add(o.min);
            self.add(o.max);
        }
    }
}

impl Solid {
    pub fn from_polys(polys: Vec<Poly>) -> Solid {
        Solid { polys }
    }
    pub fn is_empty(&self) -> bool {
        self.polys.is_empty()
    }
    pub fn bbox(&self) -> Aabb {
        let mut b = Aabb::empty();
        for p in &self.polys {
            for v in &p.v {
                b.add(*v);
            }
        }
        b
    }
    pub fn union(&self, other: &Solid) -> Solid {
        if self.is_empty() {
            return other.clone();
        }
        if other.is_empty() {
            return self.clone();
        }
        if !self.bbox().overlaps(&other.bbox()) {
            let mut p = self.polys.clone();
            p.extend(other.polys.iter().cloned());
            return Solid { polys: p };
        }
        let mut a = Node::new(self.polys.clone());
        let mut b = Node::new(other.polys.clone());
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(b.all_polygons());
        Solid { polys: a.all_polygons() }
    }
    pub fn subtract(&self, other: &Solid) -> Solid {
        if self.is_empty() || other.is_empty() || !self.bbox().overlaps(&other.bbox()) {
            return self.clone();
        }
        let mut a = Node::new(self.polys.clone());
        let mut b = Node::new(other.polys.clone());
        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(b.all_polygons());
        a.invert();
        Solid { polys: a.all_polygons() }
    }
    #[allow(dead_code)]
    pub fn intersect(&self, other: &Solid) -> Solid {
        if self.is_empty() || other.is_empty() || !self.bbox().overlaps(&other.bbox()) {
            return Solid::default();
        }
        let mut a = Node::new(self.polys.clone());
        let mut b = Node::new(other.polys.clone());
        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(b.all_polygons());
        a.invert();
        Solid { polys: a.all_polygons() }
    }
    pub fn transformed(&self, f: impl Fn(DVec3) -> DVec3, mirror: bool) -> Solid {
        let mut out = Vec::with_capacity(self.polys.len());
        for p in &self.polys {
            let mut v: Vec<DVec3> = p.v.iter().map(|x| f(*x)).collect();
            if mirror {
                v.reverse();
            }
            if let Some(np) = Poly::new(v) {
                out.push(np);
            }
        }
        Solid { polys: out }
    }
    pub fn translated(&self, d: DVec3) -> Solid {
        self.transformed(|p| p + d, false)
    }
    pub fn rotated(&self, origin: DVec3, axis: DVec3, angle: f64) -> Solid {
        let q = DQuat::from_axis_angle(axis.normalize(), angle);
        self.transformed(|p| origin + q * (p - origin), false)
    }
    /// Signed volume (positive for a correctly oriented closed solid), in mm³.
    pub fn volume(&self) -> f64 {
        let mut v = 0.0;
        for p in &self.polys {
            for i in 1..p.v.len() - 1 {
                v += p.v[0].dot(p.v[i].cross(p.v[i + 1]));
            }
        }
        v / 6.0
    }
    pub fn fix_orientation(mut self) -> Solid {
        if self.volume() < 0.0 {
            for p in &mut self.polys {
                p.flip();
            }
        }
        self
    }
}

// ---------------------------------------------------------------- 2D helpers

pub fn signed_area(pts: &[DVec2]) -> f64 {
    let mut a = 0.0;
    for i in 0..pts.len() {
        let p = pts[i];
        let q = pts[(i + 1) % pts.len()];
        a += p.x * q.y - q.x * p.y;
    }
    a * 0.5
}

pub fn point_in_polygon(p: DVec2, poly: &[DVec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

pub fn dist_point_segment(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    if l2 < 1e-18 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    (a + ab * t - p).length()
}

/// Removes consecutive duplicates and collinear points; returns CCW polygon.
pub fn clean_polygon(pts: &[DVec2]) -> Vec<DVec2> {
    let mut v: Vec<DVec2> = Vec::new();
    for p in pts {
        if v.last().map_or(true, |l: &DVec2| (*l - *p).length() > 1e-7) {
            v.push(*p);
        }
    }
    while v.len() > 1 && (v[0] - *v.last().unwrap()).length() <= 1e-7 {
        v.pop();
    }
    let mut changed = true;
    while changed && v.len() >= 3 {
        changed = false;
        let n = v.len();
        for i in 0..n {
            let a = v[(i + n - 1) % n];
            let b = v[i];
            let c = v[(i + 1) % n];
            let cr = (b - a).perp_dot(c - b);
            if cr.abs() < 1e-10 * (1.0 + (b - a).length() * (c - b).length()) {
                v.remove(i);
                changed = true;
                break;
            }
        }
    }
    if v.len() >= 3 && signed_area(&v) < 0.0 {
        v.reverse();
    }
    v
}

/// Ear-clipping triangulation of a simple polygon given in 2D. Returns index triples.
/// Works with CW or CCW input; skips zero-area (collinear) ears.
pub fn triangulate_2d(pts: &[DVec2]) -> Vec<[usize; 3]> {
    let n = pts.len();
    let mut out = Vec::new();
    if n < 3 {
        return out;
    }
    let ccw = signed_area(pts) >= 0.0;
    let mut idx: Vec<usize> = if ccw { (0..n).collect() } else { (0..n).rev().collect() };
    let mut guard = 0;
    while idx.len() > 3 && guard < n * n + 10 {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let ia = idx[(i + m - 1) % m];
            let ib = idx[i];
            let ic = idx[(i + 1) % m];
            let (a, b, c) = (pts[ia], pts[ib], pts[ic]);
            let cr = (b - a).perp_dot(c - b);
            let scale = (b - a).length() * (c - b).length();
            if cr <= 1e-12 * scale.max(1e-12) {
                continue; // reflex or collinear
            }
            // no other vertex inside triangle
            let mut ok = true;
            for &k in &idx {
                if k == ia || k == ib || k == ic {
                    continue;
                }
                let p = pts[k];
                if (p - a).length() < 1e-9 || (p - b).length() < 1e-9 || (p - c).length() < 1e-9 {
                    continue;
                }
                let d1 = (b - a).perp_dot(p - a);
                let d2 = (c - b).perp_dot(p - b);
                let d3 = (a - c).perp_dot(p - c);
                let e = 1e-12 * scale.max(1e-12);
                if d1 >= -e && d2 >= -e && d3 >= -e {
                    ok = false;
                    break;
                }
            }
            if ok {
                out.push(if ccw { [ia, ib, ic] } else { [ia, ic, ib] });
                idx.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            // remove a collinear vertex if any, otherwise give up gracefully (fan the rest)
            let m = idx.len();
            let mut removed = false;
            for i in 0..m {
                let a = pts[idx[(i + m - 1) % m]];
                let b = pts[idx[i]];
                let c = pts[idx[(i + 1) % m]];
                if (b - a).perp_dot(c - b).abs() < 1e-12 {
                    idx.remove(i);
                    removed = true;
                    break;
                }
            }
            if !removed {
                for i in 1..idx.len() - 1 {
                    let t = [idx[0], idx[i], idx[i + 1]];
                    out.push(if ccw { t } else { [t[0], t[2], t[1]] });
                }
                return out;
            }
        }
    }
    if idx.len() == 3 {
        let (a, b, c) = (pts[idx[0]], pts[idx[1]], pts[idx[2]]);
        if (b - a).perp_dot(c - b).abs() > 1e-14 {
            out.push(if ccw { [idx[0], idx[1], idx[2]] } else { [idx[0], idx[2], idx[1]] });
        }
    }
    out
}

pub fn segments_self_intersect(pts: &[DVec2]) -> bool {
    let n = pts.len();
    if n < 4 {
        return false;
    }
    fn cross(o: DVec2, a: DVec2, b: DVec2) -> f64 {
        (a - o).perp_dot(b - o)
    }
    for i in 0..n {
        let a1 = pts[i];
        let a2 = pts[(i + 1) % n];
        for j in (i + 2)..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            let b1 = pts[j];
            let b2 = pts[(j + 1) % n];
            let d1 = cross(b1, b2, a1);
            let d2 = cross(b1, b2, a2);
            let d3 = cross(a1, a2, b1);
            let d4 = cross(a1, a2, b2);
            if ((d1 > 1e-12 && d2 < -1e-12) || (d1 < -1e-12 && d2 > 1e-12))
                && ((d3 > 1e-12 && d4 < -1e-12) || (d3 < -1e-12 && d4 > 1e-12))
            {
                return true;
            }
        }
    }
    false
}

// ---------------------------------------------------------------- frames & builders

/// A placement in space: origin plus orthonormal u, v (in-plane) and n = u × v.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct Frame {
    pub o: DVec3,
    pub u: DVec3,
    pub v: DVec3,
}

impl Frame {
    pub fn n(&self) -> DVec3 {
        self.u.cross(self.v)
    }
    pub fn to3(&self, p: DVec2, z: f64) -> DVec3 {
        self.o + self.u * p.x + self.v * p.y + self.n() * z
    }
    pub fn to2(&self, p: DVec3) -> DVec2 {
        let d = p - self.o;
        DVec2::new(d.dot(self.u), d.dot(self.v))
    }
    pub fn xy() -> Frame {
        Frame { o: DVec3::ZERO, u: DVec3::X, v: DVec3::Y }
    }
    pub fn yz() -> Frame {
        Frame { o: DVec3::ZERO, u: DVec3::Y, v: DVec3::Z }
    }
    pub fn zx() -> Frame {
        Frame { o: DVec3::ZERO, u: DVec3::Z, v: DVec3::X }
    }
    /// A frame on the plane with normal n through point p; in-plane axes chosen sensibly and
    /// the origin is the world origin projected onto the plane.
    pub fn from_normal(p: DVec3, n: DVec3) -> Frame {
        let n = n.normalize();
        let pick = if n.z.abs() > 0.9 { DVec3::X } else if n.x.abs() > 0.9 { DVec3::Y } else { DVec3::Z };
        let mut u = (pick - n * pick.dot(n)).normalize();
        if n.x.abs() > 0.9 && n.x < 0.0 || n.y.abs() > 0.9 && n.y > 0.0 {
            // keep axes readable when looking from the front
        }
        let mut v = n.cross(u);
        if n.y.abs() > 0.9 {
            // plane facing front/back: u = X, v = Z like a front view
            u = DVec3::X * if n.y < 0.0 { 1.0 } else { -1.0 };
            v = n.cross(u);
        }
        let o = n * p.dot(n);
        // re-orthonormalise
        u = u.normalize();
        v = v.normalize();
        Frame { o, u, v }
    }
}

pub fn extrude(profile: &[DVec2], f: &Frame, z0: f64, z1: f64) -> Solid {
    let (z0, z1) = if z0 <= z1 { (z0, z1) } else { (z1, z0) };
    let pts = clean_polygon(profile);
    if pts.len() < 3 || z1 - z0 < 1e-9 {
        return Solid::default();
    }
    let mut polys = Vec::new();
    for t in triangulate_2d(&pts) {
        // bottom (normal -n): reversed order
        if let Some(p) = Poly::new(vec![f.to3(pts[t[0]], z0), f.to3(pts[t[2]], z0), f.to3(pts[t[1]], z0)]) {
            polys.push(p);
        }
        if let Some(p) = Poly::new(vec![f.to3(pts[t[0]], z1), f.to3(pts[t[1]], z1), f.to3(pts[t[2]], z1)]) {
            polys.push(p);
        }
    }
    let n = pts.len();
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        if let Some(p) = Poly::new(vec![f.to3(a, z0), f.to3(b, z0), f.to3(b, z1), f.to3(a, z1)]) {
            polys.push(p);
        }
    }
    Solid::from_polys(polys).fix_orientation()
}

/// Revolve a 2D profile around an in-plane axis (point `ap`, direction `ad`, both in sketch
/// coordinates) by `angle` radians.
pub fn revolve(profile: &[DVec2], f: &Frame, ap: DVec2, ad: DVec2, angle: f64) -> Result<Solid, String> {
    let pts = clean_polygon(profile);
    if pts.len() < 3 {
        return Err("profile too small".into());
    }
    let ad = ad.normalize();
    // all points must be on one side of the axis
    let mut side = 0.0f64;
    for p in &pts {
        let s = ad.perp_dot(*p - ap);
        if s.abs() > 1e-7 {
            if side != 0.0 && s.signum() != side {
                return Err("the profile crosses the rotation axis".into());
            }
            side = s.signum();
        }
    }
    if side == 0.0 {
        return Err("the profile lies on the axis".into());
    }
    let full = angle >= std::f64::consts::TAU - 1e-6;
    let angle = angle.min(std::f64::consts::TAU);
    let o3 = f.to3(ap, 0.0);
    let d3 = (f.u * ad.x + f.v * ad.y).normalize();
    let segs = ((64.0 * angle / std::f64::consts::TAU).ceil() as usize).max(4);
    let pts3: Vec<DVec3> = pts.iter().map(|p| f.to3(*p, 0.0)).collect();
    let rot = |p: DVec3, a: f64| -> DVec3 { o3 + DQuat::from_axis_angle(d3, a) * (p - o3) };
    let mut polys = Vec::new();
    let n = pts3.len();
    for s in 0..segs {
        let a0 = angle * s as f64 / segs as f64;
        let a1 = if full && s == segs - 1 { 0.0 } else { angle * (s + 1) as f64 / segs as f64 };
        for i in 0..n {
            let a = pts3[i];
            let b = pts3[(i + 1) % n];
            let mut quad = vec![rot(a, a0), rot(b, a0), rot(b, a1), rot(a, a1)];
            // drop duplicates (points on axis)
            let mut q2: Vec<DVec3> = Vec::new();
            for p in quad.drain(..) {
                if q2.last().map_or(true, |l: &DVec3| (*l - p).length() > 1e-9) && (q2.is_empty() || (q2[0] - p).length() > 1e-9) {
                    q2.push(p);
                }
            }
            if let Some(p) = Poly::new(q2) {
                polys.push(p);
            }
        }
    }
    if !full {
        for t in triangulate_2d(&pts) {
            let tri: Vec<DVec3> = t.iter().map(|&k| pts3[k]).collect();
            if let Some(p) = Poly::new(vec![tri[0], tri[2], tri[1]]) {
                polys.push(p);
            }
            if let Some(p) = Poly::new(tri.iter().map(|p| rot(*p, angle)).collect()) {
                polys.push(p);
            }
        }
    }
    // sides and caps follow one convention; flip everything if it came out inside-out
    Ok(Solid::from_polys(polys).fix_orientation())
}

pub fn box_solid(min: DVec3, max: DVec3) -> Solid {
    let f = Frame::xy();
    let prof = vec![
        DVec2::new(min.x, min.y),
        DVec2::new(max.x, min.y),
        DVec2::new(max.x, max.y),
        DVec2::new(min.x, max.y),
    ];
    extrude(&prof, &f, min.z, max.z)
}

pub fn circle_pts(c: DVec2, r: f64) -> Vec<DVec2> {
    let segs = circle_segments(r);
    (0..segs)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / segs as f64;
            c + DVec2::new(a.cos(), a.sin()) * r
        })
        .collect()
}

pub fn circle_segments(r: f64) -> usize {
    ((r * 4.0).ceil() as usize).clamp(32, 128)
}

pub fn cylinder_solid(base: DVec3, axis: DVec3, r: f64, h: f64) -> Solid {
    let f = Frame::from_normal(base, axis);
    let f = Frame { o: base, ..f };
    extrude(&circle_pts(DVec2::ZERO, r), &f, 0.0, h)
}

pub fn sphere_solid(c: DVec3, r: f64) -> Solid {
    let seg = circle_segments(r);
    let rings = seg / 2;
    let p = |i: usize, j: usize| -> DVec3 {
        let th = std::f64::consts::PI * i as f64 / rings as f64;
        let ph = std::f64::consts::TAU * (j % seg) as f64 / seg as f64;
        c + DVec3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()) * r
    };
    let mut polys = Vec::new();
    for i in 0..rings {
        for j in 0..seg {
            let a = p(i, j);
            let b = p(i + 1, j);
            let cc = p(i + 1, j + 1);
            let d = p(i, j + 1);
            if i == 0 {
                polys.extend(Poly::new(vec![a, b, cc]));
            } else if i == rings - 1 {
                polys.extend(Poly::new(vec![a, b, d]));
            } else {
                polys.extend(Poly::new(vec![a, b, cc]));
                polys.extend(Poly::new(vec![a, cc, d]));
            }
        }
    }
    Solid::from_polys(polys).fix_orientation()
}

// ---------------------------------------------------------------- finished meshes

/// Welded, T-junction-free triangle mesh used for display, picking and export.
#[derive(Clone, Default)]
pub struct Mesh {
    pub pos: Vec<DVec3>,
    pub tris: Vec<[u32; 3]>,
    pub tri_normal: Vec<DVec3>,
    /// planar face id per triangle (connected coplanar triangles)
    pub face: Vec<u32>,
    /// smooth face id per triangle (connected through gentle angles, e.g. cylinders)
    pub smooth: Vec<u32>,
    /// feature edges to draw
    pub edges: Vec<(DVec3, DVec3)>,
    pub bbox: Aabb,
}

fn qkey(p: DVec3) -> (i64, i64, i64) {
    let s = 1e4;
    ((p.x * s).round() as i64, (p.y * s).round() as i64, (p.z * s).round() as i64)
}

impl Mesh {
    pub fn from_solid(s: &Solid) -> Mesh {
        // 1. weld
        let mut map: HashMap<(i64, i64, i64), u32> = HashMap::new();
        let mut pos: Vec<DVec3> = Vec::new();
        let mut loops: Vec<(Vec<u32>, DVec3)> = Vec::new();
        for p in &s.polys {
            let mut l: Vec<u32> = Vec::new();
            for v in &p.v {
                let k = qkey(*v);
                let id = *map.entry(k).or_insert_with(|| {
                    pos.push(*v);
                    (pos.len() - 1) as u32
                });
                if l.last() != Some(&id) {
                    l.push(id);
                }
            }
            while l.len() > 1 && l[0] == *l.last().unwrap() {
                l.pop();
            }
            if l.len() >= 3 {
                loops.push((l, p.plane.n));
            }
        }
        // 2. T-junction repair: spatial grid of vertices
        let mut bbox = Aabb::empty();
        for p in &pos {
            bbox.add(*p);
        }
        let cell = (bbox.size().max_element() / 64.0).max(1e-3);
        let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
        let gk = |p: DVec3| -> (i64, i64, i64) {
            ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64)
        };
        for (i, p) in pos.iter().enumerate() {
            grid.entry(gk(*p)).or_default().push(i as u32);
        }
        let mut tris = Vec::new();
        let mut tri_normal = Vec::new();
        for (l, n) in &loops {
            let mut full: Vec<u32> = Vec::new();
            for i in 0..l.len() {
                let a = l[i];
                let b = l[(i + 1) % l.len()];
                full.push(a);
                let pa = pos[a as usize];
                let pb = pos[b as usize];
                let ab = pb - pa;
                let len2 = ab.length_squared();
                if len2 < 1e-16 {
                    continue;
                }
                let lo = gk(pa.min(pb) - DVec3::splat(2e-4));
                let hi = gk(pa.max(pb) + DVec3::splat(2e-4));
                let cells = ((hi.0 - lo.0 + 1) * (hi.1 - lo.1 + 1) * (hi.2 - lo.2 + 1)) as usize;
                let mut found: Vec<(f64, u32)> = Vec::new();
                let mut test = |k: u32| {
                    if k == a || k == b {
                        return;
                    }
                    let p = pos[k as usize];
                    let t = (p - pa).dot(ab) / len2;
                    if t <= 1e-9 || t >= 1.0 - 1e-9 {
                        return;
                    }
                    let d = (pa + ab * t - p).length();
                    if d < 2e-4 {
                        found.push((t, k));
                    }
                };
                if cells < 4096 {
                    for x in lo.0..=hi.0 {
                        for y in lo.1..=hi.1 {
                            for z in lo.2..=hi.2 {
                                if let Some(list) = grid.get(&(x, y, z)) {
                                    for &k in list {
                                        test(k);
                                    }
                                }
                            }
                        }
                    }
                } else {
                    for k in 0..pos.len() as u32 {
                        test(k);
                    }
                }
                found.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
                found.dedup_by_key(|x| x.1);
                for (_, k) in found {
                    if full.last() != Some(&k) {
                        full.push(k);
                    }
                }
            }
            // 3. triangulate in the polygon's plane
            let n = *n;
            let u = if n.x.abs() < 0.9 { DVec3::X.cross(n).normalize() } else { DVec3::Y.cross(n).normalize() };
            let v = n.cross(u);
            let p2: Vec<DVec2> = full.iter().map(|&k| { let p = pos[k as usize]; DVec2::new(p.dot(u), p.dot(v)) }).collect();
            for t in triangulate_2d(&p2) {
                let tri = [full[t[0]], full[t[1]], full[t[2]]];
                let pa = pos[tri[0] as usize];
                let pb = pos[tri[1] as usize];
                let pc = pos[tri[2] as usize];
                let tn = (pb - pa).cross(pc - pa);
                if tn.length() < 1e-12 {
                    continue;
                }
                tris.push(tri);
                tri_normal.push(n);
            }
        }
        let mut m = Mesh { pos, tris, tri_normal, bbox, ..Default::default() };
        m.analyse();
        m
    }

    pub fn from_triangles(tri: &[[DVec3; 3]]) -> Mesh {
        let polys: Vec<Poly> = tri.iter().filter_map(|t| Poly::new(t.to_vec())).collect();
        Mesh::from_solid(&Solid::from_polys(polys))
    }

    fn analyse(&mut self) {
        let nt = self.tris.len();
        let mut edge_map: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
        for (ti, t) in self.tris.iter().enumerate() {
            for k in 0..3 {
                let a = t[k];
                let b = t[(k + 1) % 3];
                edge_map.entry((a.min(b), a.max(b))).or_default().push(ti as u32);
            }
        }
        // adjacency
        let mut adj: Vec<Vec<(u32, f64)>> = vec![Vec::new(); nt];
        for ts in edge_map.values() {
            if ts.len() == 2 {
                let (t0, t1) = (ts[0] as usize, ts[1] as usize);
                let d = self.tri_normal[t0].dot(self.tri_normal[t1]).clamp(-1.0, 1.0);
                adj[t0].push((t1 as u32, d));
                adj[t1].push((t0 as u32, d));
            }
        }
        let flood = |pred: &dyn Fn(f64, usize, usize) -> bool| -> Vec<u32> {
            let mut id = vec![u32::MAX; nt];
            let mut next = 0u32;
            for s in 0..nt {
                if id[s] != u32::MAX {
                    continue;
                }
                id[s] = next;
                let mut stack = vec![s];
                while let Some(t) = stack.pop() {
                    for &(o, d) in &adj[t] {
                        let o = o as usize;
                        if id[o] == u32::MAX && pred(d, t, o) {
                            id[o] = next;
                            stack.push(o);
                        }
                    }
                }
                next += 1;
            }
            id
        };
        let tn = self.tri_normal.clone();
        let pos = self.pos.clone();
        let tris = self.tris.clone();
        self.face = flood(&|d, t, o| {
            if d < 0.99999 {
                return false;
            }
            let n = tn[t];
            let pt = pos[tris[t][0] as usize];
            let po = pos[tris[o][0] as usize];
            (po - pt).dot(n).abs() < 1e-4
        });
        self.smooth = flood(&|d, _, _| d > (22.0f64).to_radians().cos());
        self.edges = self.feature_edges(&edge_map);
    }

    /// The lines to draw: sharp edges, plus the boundary where a flat face meets a rounded
    /// surface (start and end of a fillet). Lines between the patches of a rounded surface and
    /// tiny mesh gaps are not drawn.
    fn feature_edges(&self, edge_map: &HashMap<(u32, u32), Vec<u32>>) -> Vec<(DVec3, DVec3)> {
        // neighbouring triangles within the same planar face
        let mut nbr: Vec<Vec<usize>> = vec![Vec::new(); self.tris.len()];
        for ts in edge_map.values() {
            if ts.len() == 2 {
                let (a, b) = (ts[0] as usize, ts[1] as usize);
                if self.face[a] == self.face[b] {
                    nbr[a].push(b);
                    nbr[b].push(a);
                }
            }
        }
        // how far a face reaches away from an edge, looking only at the part of the face right
        // beside the edge (so a ring-shaped face around a rim counts as narrow)
        let width = |t0: usize, a: DVec3, e: DVec3, len: f64| -> f64 {
            let d = self.tri_normal[t0].cross(e).normalize_or_zero();
            let tol = len * 0.01 + 1e-6;
            let mut seen = std::collections::HashSet::new();
            let mut stack = vec![t0];
            seen.insert(t0);
            let mut reach = 0.0f64;
            let mut steps = 0;
            while let Some(t) = stack.pop() {
                steps += 1;
                if steps > 4000 {
                    break;
                }
                for &k in &self.tris[t] {
                    reach = reach.max((self.pos[k as usize] - a).dot(d).abs());
                }
                for &u in &nbr[t] {
                    if seen.contains(&u) {
                        continue;
                    }
                    let along: Vec<f64> = self.tris[u].iter().map(|k| (self.pos[*k as usize] - a).dot(e)).collect();
                    let (lo, hi) = (along.iter().cloned().fold(f64::MAX, f64::min), along.iter().cloned().fold(f64::MIN, f64::max));
                    if hi >= -tol && lo <= len + tol {
                        seen.insert(u);
                        stack.push(u);
                    }
                }
            }
            reach.max(1e-9)
        };
        let sharp = (25.0f64).to_radians().cos();
        let flat = (0.5f64).to_radians().cos();
        let mut out = Vec::new();
        let mut keys: Vec<&(u32, u32)> = edge_map.keys().collect();
        keys.sort_unstable();
        for k in keys {
            let ts = &edge_map[k];
            if ts.len() != 2 {
                continue; // a gap in the mesh, not a real edge
            }
            let (t0, t1) = (ts[0] as usize, ts[1] as usize);
            let d = self.tri_normal[t0].dot(self.tri_normal[t1]);
            let (pa, pb) = (self.pos[k.0 as usize], self.pos[k.1 as usize]);
            let draw = if d < sharp {
                true
            } else if d < flat && self.face[t0] != self.face[t1] {
                // a gentle bend is the start / end of a rounding only where a wide face meets
                // a narrow patch; between similar patches it is inside the rounding
                let len = (pb - pa).length();
                let e = (pb - pa) / len.max(1e-12);
                let (w0, w1) = (width(t0, pa, e, len), width(t1, pa, e, len));
                w0.max(w1) > 4.0 * w0.min(w1)
            } else {
                false
            };
            if draw {
                out.push((pa, pb));
            }
        }
        out
    }

    pub fn tri_pts(&self, t: usize) -> [DVec3; 3] {
        let tr = self.tris[t];
        [self.pos[tr[0] as usize], self.pos[tr[1] as usize], self.pos[tr[2] as usize]]
    }

    /// Closest hit along a ray: (distance, triangle index).
    pub fn raycast(&self, o: DVec3, d: DVec3) -> Option<(f64, usize)> {
        if !self.bbox.valid() {
            return None;
        }
        if !ray_box(o, d, &self.bbox) {
            return None;
        }
        let mut best: Option<(f64, usize)> = None;
        for t in 0..self.tris.len() {
            let tp = self.tri_pts(t);
            if let Some(dist) = ray_tri(o, d, &tp) {
                if dist > 0.0 && best.map_or(true, |b| dist < b.0) {
                    best = Some((dist, t));
                }
            }
        }
        best
    }

    pub fn volume(&self) -> f64 {
        let mut v = 0.0;
        for t in 0..self.tris.len() {
            let p = self.tri_pts(t);
            v += p[0].dot(p[1].cross(p[2]));
        }
        v / 6.0
    }

    /// Plane of the planar face containing triangle t: (point, normal).
    pub fn face_plane(&self, t: usize) -> (DVec3, DVec3) {
        let f = self.face[t];
        let mut c = DVec3::ZERO;
        let mut area = 0.0;
        for (i, ff) in self.face.iter().enumerate() {
            if *ff == f {
                let p = self.tri_pts(i);
                let a = (p[1] - p[0]).cross(p[2] - p[0]).length() * 0.5;
                c += (p[0] + p[1] + p[2]) / 3.0 * a;
                area += a;
            }
        }
        let c = if area > 0.0 { c / area } else { self.tri_pts(t)[0] };
        (c, self.tri_normal[t])
    }

    /// Axis of the curved (cylindrical/conical) face containing triangle t, if any:
    /// (point on axis, direction, radius estimate).
    pub fn face_axis(&self, t: usize) -> Option<(DVec3, DVec3, f64)> {
        let g = self.smooth[t];
        let members: Vec<usize> = (0..self.tris.len()).filter(|i| self.smooth[*i] == g).collect();
        let mut m = DMat3::ZERO;
        let mut distinct = false;
        let n0 = self.tri_normal[members[0]];
        let mut data = Vec::new();
        for &i in &members {
            let n = self.tri_normal[i];
            if n.dot(n0) < 0.999 {
                distinct = true;
            }
            let p = self.tri_pts(i);
            let a = (p[1] - p[0]).cross(p[2] - p[0]).length() * 0.5;
            let c = (p[0] + p[1] + p[2]) / 3.0;
            m += outer(n, n) * a;
            data.push((c, n, a));
        }
        if !distinct {
            return None;
        }
        let d = smallest_eigenvector(m);
        // point on axis: least squares intersection of the normal lines, in plane ⟂ d
        let mut a_m = outer(d, d);
        let mut b = DVec3::ZERO;
        for (c, n, a) in &data {
            let pm = (DMat3::IDENTITY - outer(*n, *n)) * *a;
            a_m += pm;
            b += pm * *c;
        }
        if a_m.determinant().abs() < 1e-12 {
            return None;
        }
        let mut p = a_m.inverse() * b;
        // centre p along the axis at the middle of the face
        let ts: Vec<f64> = data.iter().map(|(c, _, _)| (*c - p).dot(d)).collect();
        let (tmin, tmax) = ts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), t| (lo.min(*t), hi.max(*t)));
        p += d * ((tmin + tmax) * 0.5);
        let r = data.iter().map(|(c, _, a)| { let w = *c - p; (w - d * w.dot(d)).length() * a }).sum::<f64>()
            / data.iter().map(|x| x.2).sum::<f64>();
        Some((p, d, r))
    }
}

fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

/// Eigenvector with the smallest eigenvalue of a symmetric 3x3 matrix (Jacobi iteration).
fn smallest_eigenvector(m: DMat3) -> DVec3 {
    let mut a = [
        [m.x_axis.x, m.y_axis.x, m.z_axis.x],
        [m.x_axis.y, m.y_axis.y, m.z_axis.y],
        [m.x_axis.z, m.y_axis.z, m.z_axis.z],
    ];
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut max) = (0, 1, 0.0f64);
        for i in 0..3 {
            for j in (i + 1)..3 {
                if a[i][j].abs() > max {
                    max = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max < 1e-14 {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let t = if theta == 0.0 { 1.0 } else { t };
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        for k in 0..3 {
            let akp = a[k][p];
            let akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for k in 0..3 {
            let apk = a[p][k];
            let aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
        for k in 0..3 {
            let vkp = v[k][p];
            let vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
        }
    }
    let mut best = 0;
    for i in 1..3 {
        if a[i][i] < a[best][best] {
            best = i;
        }
    }
    DVec3::new(v[0][best], v[1][best], v[2][best]).normalize()
}

pub fn ray_tri(o: DVec3, d: DVec3, t: &[DVec3; 3]) -> Option<f64> {
    let e1 = t[1] - t[0];
    let e2 = t[2] - t[0];
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - t[0];
    let u = s.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let dist = e2.dot(q) * inv;
    if dist > 0.0 { Some(dist) } else { None }
}

fn ray_box(o: DVec3, d: DVec3, b: &Aabb) -> bool {
    let mut tmin = f64::MIN;
    let mut tmax = f64::MAX;
    for i in 0..3 {
        let (oi, di, lo, hi) = (o[i], d[i], b.min[i] - 1e-3, b.max[i] + 1e-3);
        if di.abs() < 1e-15 {
            if oi < lo || oi > hi {
                return false;
            }
        } else {
            let t1 = (lo - oi) / di;
            let t2 = (hi - oi) / di;
            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }
    }
    tmax >= tmin && tmax > 0.0
}

// ---------------------------------------------------------------- file formats

pub fn write_stl(path: &str, tris: &[[DVec3; 3]]) -> std::io::Result<()> {
    let mut buf: Vec<u8> = Vec::with_capacity(84 + tris.len() * 50);
    let mut header = [0u8; 80];
    let h = b"BetterCAD binary STL, units: millimetres";
    header[..h.len()].copy_from_slice(h);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in tris {
        let n = (t[1] - t[0]).cross(t[2] - t[0]).normalize_or_zero();
        for f in [n.x, n.y, n.z] {
            buf.extend_from_slice(&(f as f32).to_le_bytes());
        }
        for p in t {
            for f in [p.x, p.y, p.z] {
                buf.extend_from_slice(&(f as f32).to_le_bytes());
            }
        }
        buf.extend_from_slice(&[0, 0]);
    }
    std::fs::write(path, buf)
}

pub fn write_obj(path: &str, tris: &[[DVec3; 3]]) -> std::io::Result<()> {
    let mut map: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut vs = String::new();
    let mut fs = String::new();
    let mut count = 0usize;
    for t in tris {
        let mut ids = [0usize; 3];
        for (k, p) in t.iter().enumerate() {
            let key = qkey(*p);
            let id = *map.entry(key).or_insert_with(|| {
                count += 1;
                vs.push_str(&format!("v {:.6} {:.6} {:.6}\n", p.x, p.y, p.z));
                count
            });
            ids[k] = id;
        }
        fs.push_str(&format!("f {} {} {}\n", ids[0], ids[1], ids[2]));
    }
    std::fs::write(path, format!("# BetterCAD OBJ export (millimetres)\no model\n{}{}", vs, fs))
}

pub fn read_stl(bytes: &[u8]) -> Result<Vec<[DVec3; 3]>, String> {
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if 84 + n * 50 == bytes.len() {
            let mut out = Vec::with_capacity(n);
            for i in 0..n {
                let base = 84 + i * 50 + 12;
                let f = |k: usize| -> f64 {
                    let o = base + k * 4;
                    f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]) as f64
                };
                out.push([
                    DVec3::new(f(0), f(1), f(2)),
                    DVec3::new(f(3), f(4), f(5)),
                    DVec3::new(f(6), f(7), f(8)),
                ]);
            }
            return Ok(out);
        }
    }
    let text = String::from_utf8_lossy(bytes);
    if !text.trim_start().starts_with("solid") {
        return Err("not a valid STL file".into());
    }
    let mut out = Vec::new();
    let mut cur: Vec<DVec3> = Vec::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        if it.next() == Some("vertex") {
            let v: Vec<f64> = it.filter_map(|s| s.parse().ok()).collect();
            if v.len() == 3 {
                cur.push(DVec3::new(v[0], v[1], v[2]));
                if cur.len() == 3 {
                    out.push([cur[0], cur[1], cur[2]]);
                    cur.clear();
                }
            }
        }
    }
    if out.is_empty() {
        return Err("the STL file has no triangles".into());
    }
    Ok(out)
}

pub fn read_obj(text: &str) -> Result<Vec<[DVec3; 3]>, String> {
    let mut v: Vec<DVec3> = Vec::new();
    let mut out = Vec::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let c: Vec<f64> = it.take(3).filter_map(|s| s.parse().ok()).collect();
                if c.len() == 3 {
                    v.push(DVec3::new(c[0], c[1], c[2]));
                }
            }
            Some("f") => {
                let idx: Vec<usize> = it
                    .filter_map(|s| s.split('/').next().and_then(|x| x.parse::<i64>().ok()))
                    .map(|i| if i < 0 { (v.len() as i64 + i) as usize } else { (i - 1) as usize })
                    .collect();
                if idx.iter().any(|i| *i >= v.len()) {
                    continue;
                }
                for k in 1..idx.len().saturating_sub(1) {
                    out.push([v[idx[0]], v[idx[k]], v[idx[k + 1]]]);
                }
            }
            _ => {}
        }
    }
    if out.is_empty() {
        return Err("the OBJ file has no faces".into());
    }
    Ok(out)
}

pub fn mesh_triangles(m: &Mesh, xf: impl Fn(DVec3) -> DVec3) -> Vec<[DVec3; 3]> {
    (0..m.tris.len()).map(|t| { let p = m.tri_pts(t); [xf(p[0]), xf(p[1]), xf(p[2])] }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn box_minus_box() {
        let a = box_solid(DVec3::ZERO, DVec3::splat(10.0));
        let b = box_solid(DVec3::new(2.0, 2.0, -1.0), DVec3::new(8.0, 8.0, 11.0));
        let c = a.subtract(&b);
        assert!((c.volume() - (1000.0 - 360.0)).abs() < 1e-6);
        let m = Mesh::from_solid(&c);
        assert!((m.volume() - 640.0).abs() < 1e-3);
    }
    #[test]
    fn revolve_cyl() {
        let prof = vec![DVec2::new(0.0, 0.0), DVec2::new(5.0, 0.0), DVec2::new(5.0, 10.0), DVec2::new(0.0, 10.0)];
        let s = revolve(&prof, &Frame::xy(), DVec2::ZERO, DVec2::Y, std::f64::consts::TAU).unwrap();
        let v = s.volume();
        assert!(v > 700.0 && v < 790.0, "{}", v);
        let s = revolve(&prof, &Frame::xy(), DVec2::ZERO, DVec2::Y, std::f64::consts::PI).unwrap();
        assert!(s.volume() > 350.0, "{}", s.volume());
    }
}
