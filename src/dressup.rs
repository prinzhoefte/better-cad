//! Dress-up and advanced feature geometry: edge fillets and chamfers, face-based shell,
//! thickness and draft, multi-sections solids (lofts), ribs / slots (sweeps) and hole types.

use crate::geom::*;
use glam::{DMat3, DQuat, DVec2, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap};

/// A picked edge, stored by geometry so it is found again when the part is rebuilt.
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub struct EdgeRef {
    pub p: DVec3,
    pub d: DVec3,
    pub n1: DVec3,
    pub n2: DVec3,
}

/// A picked face, stored by a point on it and its normal.
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub struct FaceRef {
    pub p: DVec3,
    pub n: DVec3,
}

/// A sharp edge segment of a mesh, with the normals of the two faces meeting there.
#[derive(Clone, Copy, Debug)]
pub struct SharpEdge {
    pub a: DVec3,
    pub b: DVec3,
    pub n1: DVec3,
    pub n2: DVec3,
    pub convex: bool,
    /// mesh vertex indices of a and b
    pub ia: u32,
    pub ib: u32,
}

pub fn sharp_edges(m: &Mesh) -> Vec<SharpEdge> {
    let mut map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, t) in m.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            map.entry((a.min(b), a.max(b))).or_default().push(ti);
        }
    }
    let mut out = Vec::new();
    let cos = 25f64.to_radians().cos();
    let mut keys: Vec<_> = map.keys().cloned().collect();
    keys.sort_unstable();
    for k in keys {
        let ts = &map[&k];
        if ts.len() != 2 {
            continue;
        }
        let (n1, n2) = (m.tri_normal[ts[0]], m.tri_normal[ts[1]]);
        if n1.dot(n2) > cos {
            continue;
        }
        let a = m.pos[k.0 as usize];
        let b = m.pos[k.1 as usize];
        let c2 = m.tri_pts(ts[1]).iter().fold(DVec3::ZERO, |s, p| s + *p) / 3.0;
        let convex = (c2 - a).dot(n1) < 0.0;
        out.push(SharpEdge { a, b, n1, n2, convex, ia: k.0, ib: k.1 });
    }
    out
}

fn same_normals(e: &SharpEdge, n1: DVec3, n2: DVec3) -> bool {
    (e.n1.dot(n1) > 0.999 && e.n2.dot(n2) > 0.999) || (e.n1.dot(n2) > 0.999 && e.n2.dot(n1) > 0.999)
}

fn dist_point_seg3(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let ab = b - a;
    let l = ab.length_squared();
    if l < 1e-18 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / l).clamp(0.0, 1.0);
    (a + ab * t - p).length()
}

/// The sharp edge segment a stored reference points at.
pub fn find_edge(edges: &[SharpEdge], r: &EdgeRef) -> Option<usize> {
    let mut best: Option<(f64, usize)> = None;
    for (i, e) in edges.iter().enumerate() {
        if !same_normals(e, r.n1, r.n2) {
            continue;
        }
        let d = dist_point_seg3(r.p, e.a, e.b);
        if d < 1e-3 && best.map_or(true, |b| d < b.0) {
            best = Some((d, i));
        }
    }
    best.map(|b| b.1)
}

/// The edge plus every edge that continues it smoothly (tangency propagation).
pub fn edge_chain(edges: &[SharpEdge], start: usize) -> Vec<usize> {
    let key = |p: DVec3| ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64, (p.z * 1e4).round() as i64);
    let mut at: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        at.entry(key(e.a)).or_default().push(i);
        at.entry(key(e.b)).or_default().push(i);
    }
    let cos = 30f64.to_radians().cos();
    let mut seen = BTreeSet::new();
    seen.insert(start);
    let mut stack = vec![start];
    while let Some(i) = stack.pop() {
        let e = edges[i];
        let di = (e.b - e.a).normalize_or_zero();
        for p in [e.a, e.b] {
            let cands: Vec<usize> = at.get(&key(p)).cloned().unwrap_or_default().into_iter().filter(|j| *j != i).collect();
            for j in cands {
                if seen.contains(&j) {
                    continue;
                }
                let f = edges[j];
                let dj = (f.b - f.a).normalize_or_zero();
                if di.dot(dj).abs() > cos && f.convex == e.convex {
                    seen.insert(j);
                    stack.push(j);
                }
            }
        }
    }
    seen.into_iter().collect()
}

pub fn edge_ref(e: &SharpEdge) -> EdgeRef {
    EdgeRef { p: (e.a + e.b) * 0.5, d: (e.b - e.a).normalize_or_zero(), n1: e.n1, n2: e.n2 }
}

/// Every edge segment the references select (with tangency propagation).
pub fn selected_segments(edges: &[SharpEdge], refs: &[EdgeRef]) -> Result<Vec<usize>, String> {
    let mut set = BTreeSet::new();
    for r in refs {
        let i = find_edge(edges, r).ok_or("an edge picked earlier no longer exists (pick it again)")?;
        set.extend(edge_chain(edges, i));
    }
    Ok(set.into_iter().collect())
}

fn vkey(p: DVec3) -> (i64, i64, i64) {
    ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64, (p.z * 1e4).round() as i64)
}

// ---------------------------------------------------------------- cross-sections on the real faces
//
// A fillet is the surface swept by a ball rolling along the edge while touching both faces
// (CATIA's "rolling ball"). At each point of the edge the faces are traced in the plane across
// the edge, and the ball is rolled along one trace until it touches the other. Because the
// traces follow the actual faces (not just their tangent planes at the edge), the rounding
// stays tangent to faces that curve across the edge and nothing of the old edge is left over.

/// Which triangles share each mesh edge, and which touch each vertex.
struct Topo {
    edge_tris: HashMap<(u32, u32), Vec<usize>>,
    vert_tris: Vec<Vec<usize>>,
}

impl Topo {
    fn new(m: &Mesh) -> Topo {
        let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        let mut vert_tris = vec![Vec::new(); m.pos.len()];
        for (t, tri) in m.tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                edge_tris.entry((a.min(b), a.max(b))).or_default().push(t);
                vert_tris[tri[k] as usize].push(t);
            }
        }
        Topo { edge_tris, vert_tris }
    }
    /// The triangle on the other side of edge e from triangle t (where exactly two meet).
    fn across(&self, e: (u32, u32), t: usize) -> Option<usize> {
        match self.edge_tris.get(&e)?.as_slice() {
            [a, b] => Some(if *a == t { *b } else { *a }),
            _ => None,
        }
    }
}

/// Where a face crosses a section plane, walked from the edge outwards: points, each with the
/// outward normal of the stretch of face leading to it.
type Trace = Vec<(DVec3, DVec3)>;

fn ekey(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

/// Follows the plane `d(vertex) = 0` across the mesh, starting at `start` on edge `from` of
/// triangle `tri`, until `max_len` is covered. Vertices on the plane count as in front of it,
/// so the walk never gets stuck on them.
fn walk(m: &Mesh, topo: &Topo, d: &dyn Fn(u32) -> f64, mut tri: usize, mut from: (u32, u32), start: DVec3, max_len: f64) -> Trace {
    let mut out: Trace = vec![(start, m.tri_normal[tri])];
    let mut len = 0.0;
    for _ in 0..5000 {
        let t = m.tris[tri];
        let exit = (0..3).map(|k| ekey(t[k], t[(k + 1) % 3])).find(|e| *e != from && (d(e.0) < 0.0) != (d(e.1) < 0.0));
        let Some(e) = exit else { break };
        let (da, db) = (d(e.0), d(e.1));
        let (pa, pb) = (m.pos[e.0 as usize], m.pos[e.1 as usize]);
        let q = pa + (pb - pa) * (da / (da - db));
        let step = (q - out.last().unwrap().0).length();
        if step > 1e-9 {
            out.push((q, m.tri_normal[tri]));
            len += step;
        }
        if len >= max_len {
            break;
        }
        match topo.across(e, tri) {
            Some(n) => {
                tri = n;
                from = e;
            }
            None => break,
        }
    }
    if out.len() > 1 {
        out[0].1 = out[1].1;
    }
    out
}

/// The two face traces through mesh vertex v, in the plane across the edge (normal t).
fn traces_at_vertex(m: &Mesh, topo: &Topo, v: u32, t: DVec3, max_len: f64) -> Option<[Trace; 2]> {
    let p = m.pos[v as usize];
    let d = |k: u32| if k == v { 0.0 } else { (m.pos[k as usize] - p).dot(t) };
    let mut out = Vec::new();
    for &tri in &topo.vert_tris[v as usize] {
        let others: Vec<u32> = m.tris[tri].iter().cloned().filter(|k| *k != v).collect();
        let neg: Vec<u32> = others.iter().cloned().filter(|k| d(*k) < 0.0).collect();
        // the plane leaves the vertex through this triangle
        if others.len() == 2 && neg.len() == 1 {
            out.push(walk(m, topo, &d, tri, ekey(v, neg[0]), p, max_len));
        }
    }
    match <[Trace; 2]>::try_from(out) {
        Ok(tr) if tr.iter().all(|x| x.len() > 1) => Some(tr),
        _ => None,
    }
}

/// The two face traces across the middle of mesh edge (ia, ib): (middle point, traces).
fn traces_at_segment(m: &Mesh, topo: &Topo, ia: u32, ib: u32, max_len: f64) -> Option<(DVec3, [Trace; 2])> {
    let (a, b) = (m.pos[ia as usize], m.pos[ib as usize]);
    let mid = (a + b) * 0.5;
    let t = (b - a).normalize_or_zero();
    let d = |k: u32| (m.pos[k as usize] - mid).dot(t);
    let e = ekey(ia, ib);
    let tris = topo.edge_tris.get(&e)?;
    if tris.len() != 2 {
        return None;
    }
    let tr = [walk(m, topo, &d, tris[0], e, mid, max_len), walk(m, topo, &d, tris[1], e, mid, max_len)];
    if tr.iter().any(|x| x.len() < 2) {
        return None;
    }
    Some((mid, tr))
}

/// Straight traces along flat faces with normals n1 and n2 (when the mesh cannot be walked).
fn flat_traces(p: DVec3, n1: DVec3, n2: DVec3, t: DVec3, convex: bool, len: f64) -> [Trace; 2] {
    let sgn = if convex { 1.0 } else { -1.0 };
    let side = |n: DVec3, other: DVec3| -> DVec3 {
        let w = t.cross(n).normalize_or_zero();
        if sgn * w.dot(other) < 0.0 { w } else { -w }
    };
    [vec![(p, n1), (p + side(n1, n2) * len, n1)], vec![(p, n2), (p + side(n2, n1) * len, n2)]]
}

/// Orders two traces so the first runs along the face with normal n1.
fn order_traces(tr: [Trace; 2], n1: DVec3) -> [Trace; 2] {
    let [a, b] = tr;
    if b[0].1.dot(n1) > a[0].1.dot(n1) { [b, a] } else { [a, b] }
}

/// A face trace flattened into its section plane: points from the edge outwards, the outward
/// normal at the start and end of each stretch (blended across gentle bends of a faceted
/// round face, kept apart at real creases) and the arc length so far.
struct Curve {
    p: Vec<DVec2>,
    n: Vec<(DVec2, DVec2)>,
    s: Vec<f64>,
}

impl Curve {
    fn new(tr: &Trace, o: DVec3, e1: DVec3, e2: DVec3, reach: f64) -> Option<Curve> {
        let to2 = |q: DVec3| DVec2::new((q - o).dot(e1), (q - o).dot(e2));
        let mut p: Vec<DVec2> = tr.iter().map(|x| to2(x.0)).collect();
        let mut sn: Vec<DVec2> = tr.iter().skip(1).map(|x| DVec2::new(x.1.dot(e1), x.1.dot(e2)).normalize_or_zero()).collect();
        if p.len() < 2 || sn.iter().any(|n| n.length() < 0.5) {
            return None;
        }
        // carry straight on past the end of the walk, so the curve is always long enough
        let dir = (p[p.len() - 1] - p[p.len() - 2]).normalize_or_zero();
        p.push(*p.last().unwrap() + dir * reach);
        sn.push(*sn.last().unwrap());
        let smooth = 25f64.to_radians().cos();
        let joint = |a: DVec2, b: DVec2| if a.dot(b) > smooth { (a + b).normalize_or_zero() } else { a };
        let n: Vec<(DVec2, DVec2)> = (0..sn.len())
            .map(|i| {
                let start = if i > 0 { joint(sn[i], sn[i - 1]) } else { sn[i] };
                let end = if i + 1 < sn.len() { joint(sn[i], sn[i + 1]) } else { sn[i] };
                (start, end)
            })
            .collect();
        let mut s = vec![0.0];
        for i in 1..p.len() {
            s.push(s[i - 1] + (p[i] - p[i - 1]).length());
        }
        Some(Curve { p, n, s })
    }
    fn length(&self) -> f64 {
        *self.s.last().unwrap()
    }
    /// Point and outward normal at arc length s.
    fn at(&self, s: f64) -> (DVec2, DVec2) {
        let s = s.clamp(0.0, self.length());
        let i = self.s.partition_point(|x| *x < s).clamp(1, self.p.len() - 1);
        let l = self.s[i] - self.s[i - 1];
        let f = if l > 0.0 { (s - self.s[i - 1]) / l } else { 0.0 };
        let (na, nb) = self.n[i - 1];
        (self.p[i - 1].lerp(self.p[i], f), na.lerp(nb, f).normalize_or_zero())
    }
    /// Distance from q to the curve, and the arc length of the nearest point (the one closest
    /// to the edge when several are equally near: the ball touches that one first).
    fn nearest(&self, q: DVec2) -> (f64, f64) {
        let mut best = (f64::MAX, 0.0);
        for i in 1..self.p.len() {
            let (a, b) = (self.p[i - 1], self.p[i]);
            let ab = b - a;
            let l2 = ab.length_squared();
            let f = if l2 > 0.0 { ((q - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
            let d = (a + ab * f - q).length();
            if d < best.0 - 1e-7 * (1.0 + d) {
                best = (d, self.s[i - 1] + f * (self.s[i] - self.s[i - 1]));
            }
        }
        best
    }
}

/// Points along each face in a tool section, and segments of a fillet's arc.
const FACE_SAMPLES: usize = 6;
const ARC_SEGS: usize = 16;

/// Cross-section of the fillet / chamfer tool at point o of the edge (section plane normal t),
/// from the traces of the two faces: offsets from o. Chamfer lengths are measured along the
/// faces. The tool reaches `eps` beyond the faces so it cleanly takes the old edge away.
fn section(o: DVec3, t: DVec3, tr: &[Trace; 2], convex: bool, size: f64, chamfer: bool, eps: f64) -> Option<Vec<DVec3>> {
    let e1 = t.any_orthonormal_vector();
    let e2 = t.cross(e1);
    let reach = size * 4.0 + 1.0;
    let ca = Curve::new(&tr[0], o, e1, e2, reach)?;
    let cb = Curve::new(&tr[1], o, e1, e2, reach)?;
    let sgn = if convex { 1.0 } else { -1.0 };
    let (sa, sb, centre) = if chamfer {
        (size, size, None)
    } else {
        // roll the ball (centre `size` inside face a) along a until it touches face b
        let centre_at = |s: f64| {
            let (p, n) = ca.at(s);
            p - n * (sgn * size)
        };
        // the ball touches face b once it is (to rounding) `size` away from it
        let g = |s: f64| cb.nearest(centre_at(s)).0 - size * (1.0 - 1e-9);
        if g(0.0) >= 0.0 {
            return None;
        }
        let h = size / 24.0;
        let top = ca.length() - h;
        let mut s = 0.0;
        let mut hit = None;
        while s < top {
            let s1 = (s + h).min(top);
            if g(s1) >= 0.0 {
                let (mut lo, mut hi) = (s, s1);
                for _ in 0..40 {
                    let mid = 0.5 * (lo + hi);
                    if g(mid) < 0.0 { lo = mid } else { hi = mid }
                }
                hit = Some(hi);
                break;
            }
            s = s1;
        }
        let sa = hit?;
        let c = centre_at(sa);
        (sa, cb.nearest(c).1, Some(c))
    };
    let to3 = |q: DVec2| e1 * q.x + e2 * q.y;
    let off = |c: &Curve, s: f64| {
        let (p, n) = c.at(s);
        to3(p + n * (sgn * eps))
    };
    let mut v = vec![to3((ca.at(0.0).1 + cb.at(0.0).1).normalize_or_zero() * (sgn * eps))];
    for j in 1..=FACE_SAMPLES {
        v.push(off(&ca, sa * j as f64 / FACE_SAMPLES as f64));
    }
    let (pa, pb) = (ca.at(sa).0, cb.at(sb).0);
    v.push(to3(pa));
    if let Some(c) = centre {
        let (u, w) = (pa - c, pb - c);
        let ang = u.perp_dot(w).atan2(u.dot(w));
        let (ru, rw) = (u.length(), w.length());
        let u = u.normalize_or_zero();
        for k in 1..ARC_SEGS {
            let f = k as f64 / ARC_SEGS as f64;
            v.push(to3(c + DVec2::from_angle(ang * f).rotate(u) * (ru + (rw - ru) * f)));
        }
    }
    v.push(to3(pb));
    for j in (1..=FACE_SAMPLES).rev() {
        v.push(off(&cb, sb * j as f64 / FACE_SAMPLES as f64));
    }
    Some(v)
}

/// Splits the selected segments into smooth runs (no sharp turns), each in order:
/// the run's segment indices with a flag telling whether each is walked from b to a.
fn edge_runs(edges: &[SharpEdge], segs: &[usize]) -> Vec<(Vec<(usize, bool)>, bool)> {
    let mut at: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for &i in segs {
        at.entry(vkey(edges[i].a)).or_default().push(i);
        at.entry(vkey(edges[i].b)).or_default().push(i);
    }
    let cos = 30f64.to_radians().cos();
    let dir = |i: usize, rev: bool| -> DVec3 {
        let d = (edges[i].b - edges[i].a).normalize_or_zero();
        if rev { -d } else { d }
    };
    // the smooth continuation of segment i (walked in direction d) at point p
    let next_at = |p: DVec3, from: usize, d: DVec3, used: &BTreeSet<usize>| -> Option<(usize, bool)> {
        at.get(&vkey(p))?.iter().filter(|j| **j != from && !used.contains(*j)).find_map(|&j| {
            let rev = vkey(edges[j].b) == vkey(p);
            let dj = dir(j, rev);
            (dj.dot(d) > cos && edges[j].convex == edges[from].convex).then_some((j, rev))
        })
    };
    let mut used = BTreeSet::new();
    let mut runs = Vec::new();
    for &s in segs {
        if used.contains(&s) {
            continue;
        }
        used.insert(s);
        // walk backwards to the start of the run first
        let mut run = std::collections::VecDeque::new();
        run.push_back((s, false));
        loop {
            let (first, rev) = *run.front().unwrap();
            let p = if rev { edges[first].b } else { edges[first].a };
            let d = -dir(first, rev);
            match next_at(p, first, d, &used) {
                Some((j, r)) => {
                    used.insert(j);
                    run.push_front((j, !r));
                }
                None => break,
            }
        }
        loop {
            let (last, rev) = *run.back().unwrap();
            let p = if rev { edges[last].a } else { edges[last].b };
            match next_at(p, last, dir(last, rev), &used) {
                Some((j, r)) => {
                    used.insert(j);
                    run.push_back((j, r));
                }
                None => break,
            }
        }
        let run: Vec<(usize, bool)> = run.into_iter().collect();
        let (f, fr) = run[0];
        let (l, lr) = run[run.len() - 1];
        let start = if fr { edges[f].b } else { edges[f].a };
        let end = if lr { edges[l].a } else { edges[l].b };
        let closed = run.len() > 2 && vkey(start) == vkey(end);
        runs.push((run, closed));
    }
    runs
}

/// A smooth run of selected edge segments, ready to be swept.
struct Run {
    pts: Vec<DVec3>,
    /// mesh vertex of each point
    vids: Vec<u32>,
    /// face normals of each segment, consistently ordered along the run
    normals: Vec<(DVec3, DVec3)>,
    convex: bool,
    closed: bool,
}

impl Run {
    fn new(edges: &[SharpEdge], run: &[(usize, bool)], closed: bool) -> Run {
        let (mut pts, mut vids, mut normals) = (Vec::new(), Vec::new(), Vec::<(DVec3, DVec3)>::new());
        for (k, &(i, rev)) in run.iter().enumerate() {
            let e = edges[i];
            let (a, b, ia, ib) = if rev { (e.b, e.a, e.ib, e.ia) } else { (e.a, e.b, e.ia, e.ib) };
            if k == 0 {
                pts.push(a);
                vids.push(ia);
            }
            pts.push(b);
            vids.push(ib);
            let (mut n1, mut n2) = (e.n1, e.n2);
            if let Some(&(p1, _)) = normals.last() {
                if n2.dot(p1) > n1.dot(p1) {
                    std::mem::swap(&mut n1, &mut n2);
                }
            }
            normals.push((n1, n2));
        }
        if closed {
            pts.pop();
            vids.pop();
        }
        Run { pts, vids, normals, convex: edges[run[0].0].convex, closed }
    }
    fn seg_dir(&self, k: usize) -> DVec3 {
        (self.pts[(k + 1) % self.pts.len()] - self.pts[k]).normalize_or_zero()
    }
    /// Direction pointing out of the run past its start / end.
    fn beyond(&self, start: bool) -> DVec3 {
        if start { -self.seg_dir(0) } else { self.seg_dir(self.pts.len() - 2) }
    }
    fn length(&self) -> f64 {
        self.pts.windows(2).map(|w| (w[1] - w[0]).length()).sum()
    }
    /// Face normals of the segment at the start / end.
    fn end_normals(&self, start: bool) -> (DVec3, DVec3) {
        if start { self.normals[0] } else { *self.normals.last().unwrap() }
    }
}

/// How a run's tool finishes at one of its ends.
#[derive(Clone, Copy, Debug)]
enum End {
    /// carry on this far past the end
    Extend(f64),
    /// meet another run on the mitre plane through the end with this normal (picture frame)
    Mitre(DVec3),
    /// stop this far short of the end, where a ball-shaped corner takes over
    Setback(f64),
}

fn slerp(a: DVec3, b: DVec3, f: f64) -> DVec3 {
    let th = a.dot(b).clamp(-1.0, 1.0).acos();
    if th < 1e-9 {
        return a;
    }
    (a * ((1.0 - f) * th).sin() + b * (f * th).sin()) / th.sin()
}

/// The material to take off where three filleted convex edges meet at a corner of three flat
/// faces: everything in the corner outside a ball that touches all three faces (CATIA's
/// rounded corner). `dirs` point from the corner along the three edges. Returns the tool and
/// how far each edge's own tool stops short of the corner (where the ball takes over).
///
/// The tool is built directly as a closed solid: the ball's corner patch, split into one
/// four-sided piece per face, joined along rays from the ball's centre to the faces (moved
/// out by eps, so the old corner is cleanly taken away).
fn ball_corner(v: DVec3, faces: [DVec3; 3], dirs: [DVec3; 3], size: f64, eps: f64) -> Option<(Solid, [f64; 3])> {
    let a = DMat3::from_cols(faces[0], faces[1], faces[2]).transpose();
    if a.determinant().abs() < 1e-6 {
        return None;
    }
    let inv = a.inverse();
    let solve = |off: f64| inv * DVec3::new(faces[0].dot(v) + off, faces[1].dot(v) + off, faces[2].dot(v) + off);
    let c = solve(-size);
    let vo = solve(eps);
    let back = [0, 1, 2].map(|i| dirs[i].dot(c - v));
    if back.iter().any(|b| *b <= eps) {
        return None;
    }
    // each edge lies on two faces, each face holds two edges
    let on = |f: usize, e: usize| faces[f].dot(dirs[e]).abs() < 1e-3;
    let face_edges: Vec<Vec<usize>> = (0..3).map(|f| (0..3).filter(|e| on(f, *e)).collect()).collect();
    if face_edges.iter().any(|x| x.len() != 2) {
        return None;
    }
    // directions from the centre: where the ball touches each face, towards each (moved out)
    // edge across its stop plane, and towards the (moved out) corner
    let mid = [0, 1, 2].map(|i| (vo + dirs[i] * dirs[i].dot(c - vo) - c).normalize());
    let corner = (vo - c).normalize();
    const G: usize = 8;
    let mut polys = Vec::new();
    let mut push = |pts: Vec<DVec3>, want: DVec3| {
        if let Some(p) = Poly::new(pts.clone()) {
            polys.push(if p.plane.n.dot(want) < 0.0 { Poly::new(pts.into_iter().rev().collect()).unwrap() } else { p });
        }
    };
    for f in 0..3 {
        let (e1, e2) = (face_edges[f][0], face_edges[f][1]);
        let (p, m1, m2) = (faces[f], mid[e1], mid[e2]);
        // a Coons patch of directions; its sides are exact arcs, so neighbours match
        let dir = |i: usize, j: usize| -> DVec3 {
            let (u, w) = (i as f64 / G as f64, j as f64 / G as f64);
            let bottom = |u: f64| slerp(p, m1, u);
            let top = |u: f64| slerp(m2, corner, u);
            let left = |w: f64| slerp(p, m2, w);
            let right = |w: f64| slerp(m1, corner, w);
            match (i, j) {
                (_, 0) => bottom(u),
                (_, G) => top(u),
                (0, _) => left(w),
                (G, _) => right(w),
                _ => ((1.0 - w) * bottom(u) + w * top(u) + (1.0 - u) * left(w) + u * right(w)
                    - ((1.0 - u) * (1.0 - w) * p + u * (1.0 - w) * m1 + (1.0 - u) * w * m2 + u * w * corner))
                    .normalize(),
            }
        };
        let n = faces[f];
        let inner = |i, j| c + dir(i, j) * size;
        let outer = |i, j| {
            let d = dir(i, j);
            c + d * ((size + eps) / n.dot(d))
        };
        for i in 0..G {
            for j in 0..G {
                let (q0, q1, q2, q3) = ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1));
                let centre_side = c - inner(i, j);
                push(vec![inner(q0.0, q0.1), inner(q1.0, q1.1), inner(q2.0, q2.1)], centre_side);
                push(vec![inner(q0.0, q0.1), inner(q2.0, q2.1), inner(q3.0, q3.1)], centre_side);
                push(vec![outer(q0.0, q0.1), outer(q1.0, q1.1), outer(q2.0, q2.1), outer(q3.0, q3.1)], n);
            }
            // sides on the stop planes of the two edges (the patch's bottom and left sides)
            push(vec![inner(i, 0), inner(i + 1, 0), outer(i + 1, 0), outer(i, 0)], dirs[e1]);
            push(vec![inner(0, i), inner(0, i + 1), outer(0, i + 1), outer(0, i)], dirs[e2]);
        }
    }
    let s = Solid::from_polys(polys);
    if s.volume() <= 0.0 {
        return None;
    }
    Some((s, back))
}

/// Builds the material to remove (convex edges) and add (concave edges) for a fillet
/// (radius) or chamfer (length) along the given edge segments of mesh `m`. Each smooth run of
/// edges is swept as one piece, so curved edges (hole rims, round pads) stay fast.
///
/// Like CATIA V5: the rounding rolls on the real faces (also faces curving across the edge),
/// two filleted edges meeting at a corner join like a picture frame, three convex ones get a
/// ball-shaped corner, and a fillet ending at a convex corner runs on until the next face
/// trims it.
pub fn edge_tools(m: &Mesh, edges: &[SharpEdge], segs: &[usize], size: f64, chamfer: bool) -> (Vec<Solid>, Vec<Solid>) {
    let topo = Topo::new(m);
    let eps = 0.02f64.min(size * 0.05).max(1e-3);
    let walk_len = size * 3.0 + 1.0;
    let runs: Vec<Run> = edge_runs(edges, segs).iter().map(|(r, closed)| Run::new(edges, r, *closed)).collect();
    // sharp edges at each corner: how many, and whether all are convex
    let mut corner: HashMap<(i64, i64, i64), (usize, bool)> = HashMap::new();
    for e in edges {
        for p in [e.a, e.b] {
            let c = corner.entry(vkey(p)).or_insert((0, true));
            c.0 += 1;
            c.1 &= e.convex;
        }
    }
    // the run ends meeting at each point
    let mut ends_at: HashMap<(i64, i64, i64), Vec<(usize, bool)>> = HashMap::new();
    for (ri, r) in runs.iter().enumerate() {
        if !r.closed {
            ends_at.entry(vkey(r.pts[0])).or_default().push((ri, true));
            ends_at.entry(vkey(*r.pts.last().unwrap())).or_default().push((ri, false));
        }
    }
    let mut cuts = Vec::new();
    let mut adds = Vec::new();
    let mut end_kind: HashMap<(usize, bool), End> = HashMap::new();
    let mut keys: Vec<_> = ends_at.keys().cloned().collect();
    keys.sort_unstable();
    for key in keys {
        let list = &ends_at[&key];
        let overlap = End::Extend(size * 0.3);
        let mut kinds: Vec<End> = vec![overlap; list.len()];
        match list.len() {
            1 => {
                let r = &runs[list[0].0];
                let (n, all_convex) = corner.get(&key).copied().unwrap_or((1, false));
                // a convex fillet runs on past a convex corner until the next face trims it
                // (it only cuts air there); an added (concave) fillet ends flush with the end
                // face so it does not stick out
                kinds[0] = End::Extend(if !r.convex { 0.0 } else if all_convex && n >= 3 { size * 1.5 } else { eps });
            }
            2 => {
                let ((r1, s1), (r2, s2)) = (list[0], list[1]);
                let (a, b) = (&runs[r1], &runs[r2]);
                // arriving along the first run, leaving along the second
                let (arrive, leave) = (a.beyond(s1), -b.beyond(s2));
                let mitre = (arrive + leave).normalize_or_zero();
                // where the edges turn towards the first run's material (an outside corner) both
                // tools simply run on past the corner, into the air; where they turn away (an
                // inside corner) running on would cut into the other leg, so they meet on the
                // mitre plane like a picture frame
                let (n1, n2) = a.end_normals(s1);
                let (m1, m2) = b.end_normals(s2);
                let shared = [n1, n2].into_iter().find(|n| m1.dot(*n) > 0.999 || m2.dot(*n) > 0.999);
                let inside = shared.map(|ns| {
                    let other = if ns == n1 { n2 } else { n1 };
                    let sgn = if a.convex { 1.0 } else { -1.0 };
                    let w = arrive.cross(ns).normalize_or_zero();
                    let into_face = if sgn * w.dot(other) < 0.0 { w } else { -w };
                    leave.dot(into_face) < 0.0
                });
                if a.convex == b.convex && inside == Some(true) && arrive.dot(mitre).abs() > 0.3 {
                    kinds = vec![End::Mitre(mitre); 2];
                }
            }
            3 if !chamfer && list.iter().all(|(r, _)| runs[*r].convex) => {
                let v = if list[0].1 { runs[list[0].0].pts[0] } else { *runs[list[0].0].pts.last().unwrap() };
                let mut faces: Vec<DVec3> = Vec::new();
                for &(r, s) in list {
                    let (n1, n2) = runs[r].end_normals(s);
                    for n in [n1, n2] {
                        if !faces.iter().any(|f| f.dot(n) > 0.999) {
                            faces.push(n);
                        }
                    }
                }
                let dirs = [0, 1, 2].map(|k| -runs[list[k].0].beyond(list[k].1));
                if let Ok(faces) = <[DVec3; 3]>::try_from(faces) {
                    if let Some((tool, back)) = ball_corner(v, faces, dirs, size, eps) {
                        if (0..3).all(|k| back[k] < runs[list[k].0].length()) {
                            cuts.push(tool);
                            kinds = back.iter().map(|b| End::Setback(*b)).collect();
                        }
                    }
                }
            }
            _ => {}
        }
        for (k, &(r, s)) in list.iter().enumerate() {
            end_kind.insert((r, s), kinds[k]);
        }
    }
    for (ri, r) in runs.iter().enumerate() {
        let np = r.pts.len();
        let nseg = r.normals.len();
        let mut rings = Vec::with_capacity(np);
        for i in 0..np {
            // segments before and after this point
            let before = if i > 0 { Some(i - 1) } else if r.closed { Some(nseg - 1) } else { None };
            let after = if i < nseg { Some(i) } else { None };
            let (mut n1, mut n2, mut t) = (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
            for k in [before, after].into_iter().flatten() {
                n1 += r.normals[k].0;
                n2 += r.normals[k].1;
                t += r.seg_dir(k % np);
            }
            let (n1, n2, t) = (n1.normalize_or_zero(), n2.normalize_or_zero(), t.normalize_or_zero());
            let is_end = !r.closed && (i == 0 || i == np - 1);
            // trace the faces across the edge: at run ends across the middle of the end
            // segment (the end point itself is a corner of other faces)
            let (o, traced) = if is_end {
                let k = if i == 0 { 0 } else { nseg - 1 };
                match traces_at_segment(m, &topo, r.vids[k], r.vids[k + 1], walk_len) {
                    Some((mid, tr)) => (mid, Some(tr)),
                    None => (r.pts[i], None),
                }
            } else {
                (r.pts[i], traces_at_vertex(m, &topo, r.vids[i], t, walk_len))
            };
            let flat = || order_traces(flat_traces(o, n1, n2, t, r.convex, walk_len), n1);
            let sec = traced
                .map(|tr| order_traces(tr, n1))
                .and_then(|tr| section(o, t, &tr, r.convex, size, chamfer, eps))
                .or_else(|| section(o, t, &flat(), r.convex, size, chamfer, eps));
            let Some(sec) = sec else { break };
            let mut p = r.pts[i];
            let ring: Vec<DVec3> = if is_end {
                let u = r.beyond(i == 0);
                match end_kind.get(&(ri, i == 0)).copied().unwrap_or(End::Extend(eps)) {
                    End::Extend(ext) => {
                        p += u * ext;
                        sec.iter().map(|o| p + *o).collect()
                    }
                    End::Setback(b) => {
                        p -= u * b;
                        sec.iter().map(|o| p + *o).collect()
                    }
                    End::Mitre(mn) => sec
                        .iter()
                        .map(|o| {
                            // exactly onto the plane: the other tool ends there too, so
                            // together they cover the corner without overlapping faces
                            p + *o + u * (-(*o).dot(mn) / u.dot(mn))
                        })
                        .collect(),
                }
            } else {
                sec.iter().map(|o| p + *o).collect()
            };
            rings.push(ring);
        }
        if rings.len() != np || rings.len() < 2 {
            continue;
        }
        if let Ok(piece) = skin(&rings, r.closed) {
            if r.convex {
                cuts.push(piece);
            } else {
                adds.push(piece);
            }
        }
    }
    (layers(cuts), layers(adds))
}

/// Groups tool pieces into layers of pieces that do not touch, each layer simply put
/// together (they cannot intersect). Overlapping pieces go into separate layers to be applied
/// one after the other: joining two tools first would have to sort out their coinciding faces
/// (e.g. both lying on the same face of the part), which the boolean operations handle poorly.
pub fn layers(pieces: Vec<Solid>) -> Vec<Solid> {
    let mut out: Vec<(Solid, Vec<Aabb>)> = Vec::new();
    for p in pieces {
        let b = p.bbox();
        match out.iter_mut().find(|(_, boxes)| !boxes.iter().any(|x| x.overlaps(&b))) {
            Some((s, boxes)) => {
                s.polys.extend(p.polys);
                boxes.push(b);
            }
            None => out.push((p, vec![b])),
        }
    }
    out.into_iter().map(|x| x.0).collect()
}

/// Applies edge tools to a body: removes the cut layers, then adds the added ones.
pub fn apply_edge_tools(body: &Solid, cuts: &[Solid], adds: &[Solid]) -> Solid {
    let mut b = body.clone();
    for c in cuts {
        b = b.subtract(c);
    }
    for a in adds {
        b = b.union(a);
    }
    b
}

// ---------------------------------------------------------------- face-based deformation

/// Which planar face group of the mesh a reference points at.
pub fn find_face(m: &Mesh, r: &FaceRef) -> Option<u32> {
    let mut best: Option<(f64, u32)> = None;
    for t in 0..m.tris.len() {
        let n = m.tri_normal[t];
        if n.dot(r.n) < 0.9999 {
            continue;
        }
        let p = m.tri_pts(t);
        if (r.p - p[0]).dot(n).abs() > 1e-3 {
            continue;
        }
        // distance from the point to the triangle (in its plane)
        let q = r.p - n * (r.p - p[0]).dot(n);
        let inside = (0..3).all(|k| (p[(k + 1) % 3] - p[k]).cross(q - p[k]).dot(n) >= -1e-6);
        let d = if inside { 0.0 } else { (0..3).map(|k| dist_point_seg3(q, p[k], p[(k + 1) % 3])).fold(f64::MAX, f64::min) };
        if d < 1e-3 && best.map_or(true, |b| d < b.0) {
            best = Some((d, m.face[t]));
        }
    }
    best.map(|b| b.1)
}

/// Rebuilds the mesh with some face planes moved: every vertex goes to the point that best
/// lies on the new planes of the faces around it. `plane_of(face id, normal, offset)` returns
/// the new (normal, offset) of the plane n·x = c.
pub fn replane(m: &Mesh, plane_of: &dyn Fn(u32, DVec3, f64) -> (DVec3, f64)) -> Solid {
    let mut faces_of: Vec<Vec<u32>> = vec![Vec::new(); m.pos.len()];
    let mut face_plane: HashMap<u32, (DVec3, f64)> = HashMap::new();
    for (t, tri) in m.tris.iter().enumerate() {
        let f = m.face[t];
        face_plane.entry(f).or_insert_with(|| {
            let n = m.tri_normal[t];
            plane_of(f, n, n.dot(m.pos[tri[0] as usize]))
        });
        for &v in tri {
            if !faces_of[v as usize].contains(&f) {
                faces_of[v as usize].push(f);
            }
        }
    }
    let lambda = 1e-10;
    let newpos: Vec<DVec3> = (0..m.pos.len())
        .map(|v| {
            let p = m.pos[v];
            let mut a = DMat3::IDENTITY * lambda;
            let mut b = p * lambda;
            for f in &faces_of[v] {
                let (n, c) = face_plane[f];
                a += DMat3::from_cols(n * n.x, n * n.y, n * n.z);
                b += n * c;
            }
            if a.determinant().abs() < 1e-18 { p } else { a.inverse() * b }
        })
        .collect();
    let polys = m.tris.iter().filter_map(|t| Poly::new(vec![newpos[t[0] as usize], newpos[t[1] as usize], newpos[t[2] as usize]])).collect();
    Solid::from_polys(polys)
}

// ---------------------------------------------------------------- multi-sections & sweeps

/// Resamples a closed loop to `n` points evenly spaced along its length.
pub fn resample(loop_: &[DVec3], n: usize) -> Vec<DVec3> {
    let m = loop_.len();
    let lens: Vec<f64> = (0..m).map(|i| (loop_[(i + 1) % m] - loop_[i]).length()).collect();
    let total: f64 = lens.iter().sum();
    let mut out = Vec::with_capacity(n);
    let mut seg = 0;
    let mut acc = 0.0;
    for k in 0..n {
        let target = total * k as f64 / n as f64;
        while seg < m - 1 && acc + lens[seg] < target {
            acc += lens[seg];
            seg += 1;
        }
        let t = if lens[seg] > 0.0 { ((target - acc) / lens[seg]).clamp(0.0, 1.0) } else { 0.0 };
        out.push(loop_[seg].lerp(loop_[(seg + 1) % m], t));
    }
    out
}

fn newell(p: &[DVec3]) -> DVec3 {
    let mut n = DVec3::ZERO;
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    n
}

/// Joins a stack of rings (each the same point count) into a closed solid with end caps
/// (`closed` joins the last ring back to the first and needs no caps).
fn skin(rings: &[Vec<DVec3>], closed: bool) -> Result<Solid, String> {
    let n = rings[0].len();
    let mut polys = Vec::new();
    let r = rings.len();
    let segs = if closed { r } else { r - 1 };
    for k in 0..segs {
        let (a, b) = (&rings[k], &rings[(k + 1) % r]);
        for i in 0..n {
            let j = (i + 1) % n;
            polys.extend(Poly::new(vec![a[i], a[j], b[j]]));
            polys.extend(Poly::new(vec![a[i], b[j], b[i]]));
        }
    }
    if !closed {
        for (ring, flip) in [(&rings[0], true), (&rings[r - 1], false)] {
            let nrm = newell(ring).normalize_or_zero();
            let u = nrm.any_orthonormal_vector();
            let v = nrm.cross(u);
            let p2: Vec<DVec2> = ring.iter().map(|p| DVec2::new(p.dot(u), p.dot(v))).collect();
            for t in triangulate_2d(&p2) {
                let tri = if flip { vec![ring[t[0]], ring[t[2]], ring[t[1]]] } else { vec![ring[t[0]], ring[t[1]], ring[t[2]]] };
                polys.extend(Poly::new(tri));
            }
        }
    }
    let s = Solid::from_polys(polys).fix_orientation();
    if s.volume().abs() < 1e-9 {
        return Err("the result has no volume".into());
    }
    Ok(s)
}

/// A solid blended through several closed profiles (multi-sections solid).
pub fn loft(sections: &[Vec<DVec3>]) -> Result<Solid, String> {
    if sections.len() < 2 {
        return Err("pick at least two sketches as sections".into());
    }
    let n = 96;
    let mut rings: Vec<Vec<DVec3>> = Vec::new();
    let n0 = newell(&sections[0]);
    for s in sections {
        let mut r = resample(s, n);
        if newell(&r).dot(n0) < 0.0 {
            r.reverse();
        }
        if let Some(prev) = rings.last() {
            // start where it best matches the previous section (avoids twisting)
            let mut best = (f64::MAX, 0);
            for shift in 0..n {
                let e: f64 = (0..n).step_by(4).map(|i| (r[(i + shift) % n] - prev[i]).length_squared()).sum();
                if e < best.0 {
                    best = (e, shift);
                }
            }
            r.rotate_left(best.1);
        }
        rings.push(r);
    }
    skin(&rings, false)
}

/// Sweeps closed profiles along a path (rib / slot). The profile keeps its angle to the
/// path (it is carried along with the path's turns).
pub fn sweep(profile: &[DVec3], path: &[DVec3], closed_path: bool) -> Result<Solid, String> {
    let m = path.len();
    if m < 2 {
        return Err("the centre curve is too short".into());
    }
    let seg_dir = |i: usize| (path[(i + 1) % m] - path[i]).normalize_or_zero();
    let tangent = |i: usize| -> DVec3 {
        if closed_path {
            (seg_dir((i + m - 1) % m) + seg_dir(i)).normalize_or_zero()
        } else if i == 0 {
            seg_dir(0)
        } else if i == m - 1 {
            seg_dir(m - 2)
        } else {
            (seg_dir(i - 1) + seg_dir(i)).normalize_or_zero()
        }
    };
    let mut rings = Vec::with_capacity(m);
    let mut rot = DQuat::IDENTITY;
    let mut prev_t = tangent(0);
    for i in 0..m {
        let t = tangent(i);
        if i > 0 {
            let q = if prev_t.dot(t) < -0.9999 { DQuat::from_axis_angle(t.any_orthonormal_vector(), std::f64::consts::PI) } else { DQuat::from_rotation_arc(prev_t, t) };
            rot = (q * rot).normalize();
            prev_t = t;
        }
        let ring: Vec<DVec3> = profile.iter().map(|q| path[i] + rot * (*q - path[0])).collect();
        rings.push(ring);
    }
    skin(&rings, closed_path)
}

/// A hole's material, revolved from its profile: simple, counterbored or countersunk.
/// `kind`: 0 simple, 1 counterbored, 2 countersunk.
pub fn hole_solid(pos: DVec3, dir: DVec3, d: f64, depth: f64, kind: u8, d2: f64, depth2: f64, angle: f64) -> Solid {
    let dir = dir.normalize();
    let u = dir.any_orthonormal_vector();
    let f = Frame { o: pos, u, v: dir };
    let r = d / 2.0;
    let e = 1e-3;
    let r2 = (d2 / 2.0).max(r * 1.01);
    let prof: Vec<DVec2> = match kind {
        1 => vec![DVec2::new(0.0, -e), DVec2::new(r2, -e), DVec2::new(r2, depth2.min(depth)), DVec2::new(r, depth2.min(depth)), DVec2::new(r, depth), DVec2::new(0.0, depth)],
        2 => {
            let half = (angle.clamp(10.0, 170.0) / 2.0).to_radians();
            let hc = ((r2 - r) / half.tan()).min(depth);
            vec![DVec2::new(0.0, -e), DVec2::new(r2 + e * half.tan(), -e), DVec2::new(r, hc), DVec2::new(r, depth), DVec2::new(0.0, depth)]
        }
        _ => vec![DVec2::new(0.0, -e), DVec2::new(r, -e), DVec2::new(r, depth), DVec2::new(0.0, depth)],
    };
    revolve(&prof, &f, DVec2::ZERO, DVec2::Y, std::f64::consts::TAU).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_mesh() -> (Solid, Mesh) {
        let s = box_solid(DVec3::ZERO, DVec3::new(20.0, 10.0, 10.0));
        let m = Mesh::from_solid(&s);
        (s, m)
    }

    #[test]
    fn fillet_and_chamfer_an_edge() {
        let (s, m) = box_mesh();
        let edges = sharp_edges(&m);
        // the top edge along X at y = 10, z = 10
        let i = edges.iter().position(|e| (e.a.y - 10.0).abs() < 1e-9 && (e.a.z - 10.0).abs() < 1e-9 && (e.b.y - 10.0).abs() < 1e-9 && (e.b.z - 10.0).abs() < 1e-9).unwrap();
        let r = edge_ref(&edges[i]);
        let segs = selected_segments(&edges, &[r]).unwrap();
        let (cut, add) = edge_tools(&m, &edges, &segs, 2.0, false);
        assert!(add.is_empty());
        let filleted = apply_edge_tools(&s, &cut, &add);
        let removed = 2000.0 - filleted.volume();
        let expect = (4.0 - std::f64::consts::PI) * 20.0; // (r² - πr²/4) · length
        assert!((removed - expect).abs() < 0.3, "{} vs {}", removed, expect);
        let (cut, _) = edge_tools(&m, &edges, &segs, 2.0, true);
        let chamfered = apply_edge_tools(&s, &cut, &[]);
        assert!(((2000.0 - chamfered.volume()) - 40.0).abs() < 0.3, "{}", 2000.0 - chamfered.volume());
    }

    #[test]
    fn shell_thickness_draft() {
        let (s, m) = box_mesh();
        // shell 1 mm, open at the top
        let top = FaceRef { p: DVec3::new(10.0, 5.0, 10.0), n: DVec3::Z };
        let top_id = find_face(&m, &top).unwrap();
        let inner = replane(&m, &|f, n, c| if f == top_id { (n, c + 1.0) } else { (n, c - 1.0) });
        let shell = s.subtract(&inner);
        let expect = 2000.0 - 18.0 * 8.0 * 9.0;
        assert!((shell.volume() - expect).abs() < 1e-3, "{} vs {}", shell.volume(), expect);
        // thicken the top by 5
        let thick = replane(&m, &|f, n, c| if f == top_id { (n, c + 5.0) } else { (n, c) });
        assert!((thick.volume() - 3000.0).abs() < 1e-3);
        // draft the +X side face by 10 degrees about the floor, pulling up
        let side = find_face(&m, &FaceRef { p: DVec3::new(20.0, 5.0, 5.0), n: DVec3::X }).unwrap();
        let a = 10f64.to_radians();
        let dr = replane(&m, &|f, n, c| if f == side { (n * a.cos() + DVec3::Z * a.sin(), c * a.cos()) } else { (n, c) });
        let lost = 2000.0 - dr.volume();
        let expect = 0.5 * 10.0 * 10.0 * a.tan() * 10.0;
        assert!((lost - expect).abs() < 1e-3, "{} vs {}", lost, expect);
    }

    #[test]
    fn loft_sweep_hole() {
        let sq = |z: f64, h: f64| vec![DVec3::new(-h, -h, z), DVec3::new(h, -h, z), DVec3::new(h, h, z), DVec3::new(-h, h, z)];
        let l = loft(&[sq(0.0, 10.0), sq(10.0, 10.0)]).unwrap();
        assert!((l.volume() - 4000.0).abs() < 1.0, "{}", l.volume());
        let ring: Vec<DVec3> = (0..32).map(|i| {
            let a = std::f64::consts::TAU * i as f64 / 32.0;
            DVec3::new(0.0, a.cos() * 2.0, a.sin() * 2.0)
        }).collect();
        let path = vec![DVec3::ZERO, DVec3::new(30.0, 0.0, 0.0)];
        let s = sweep(&ring, &path, false).unwrap();
        let area = 0.5 * 32.0 * 4.0 * (std::f64::consts::TAU / 32.0).sin();
        assert!((s.volume() - area * 30.0).abs() < 0.5, "{}", s.volume());
        let h = hole_solid(DVec3::ZERO, -DVec3::Z, 6.0, 10.0, 1, 10.0, 3.0, 90.0);
        let v = h.volume();
        let ideal = std::f64::consts::PI * (9.0 * 10.0 + (25.0 - 9.0) * 3.0);
        assert!((v - ideal).abs() / ideal < 0.03, "{} vs {}", v, ideal);
    }
}

#[cfg(test)]
mod perf {
    use super::*;
    use std::time::Instant;

    /// A plate with 36 through holes, all top hole rims filleted.
    #[test]
    #[ignore]
    fn fillet_timing() {
        let mut body = box_solid(DVec3::new(-50.0, -50.0, 0.0), DVec3::new(50.0, 50.0, 10.0));
        let t0 = Instant::now();
        for i in 0..3 {
            for j in 0..3 {
                let c = DVec3::new(-40.0 + 16.0 * i as f64, -40.0 + 16.0 * j as f64, -1.0);
                body = body.subtract(&cylinder_solid(c, DVec3::Z, 3.0, 12.0));
            }
        }
        println!("plate built: {:?}", t0.elapsed());
        let t = Instant::now();
        let m = Mesh::from_solid(&body);
        let edges = sharp_edges(&m);
        println!("mesh {} tris, {} sharp edges: {:?}", m.tris.len(), edges.len(), t.elapsed());
        let refs: Vec<EdgeRef> = (0..6)
            .flat_map(|i| (0..3).map(move |j| (i, j)))
            .filter_map(|(i, j)| {
                let p = DVec3::new(-40.0 + 16.0 * i as f64 + 3.0, -40.0 + 16.0 * j as f64, 10.0);
                edges.iter().find(|e| (e.a - p).length() < 0.6 && e.a.z > 9.99 && e.b.z > 9.99).map(edge_ref)
            })
            .collect();
        let t = Instant::now();
        let segs = selected_segments(&edges, &refs).unwrap();
        let (cut, _) = edge_tools(&m, &edges, &segs, 1.0, false);
        println!("{} refs, {} segments, tool {} polys: {:?}", refs.len(), segs.len(), cut.iter().map(|c| c.polys.len()).sum::<usize>(), t.elapsed());
        let t = Instant::now();
        let r = apply_edge_tools(&body, &cut, &[]);
        println!("subtract: {:?}", t.elapsed());
        let removed = body.volume() - r.volume();
        // fillet spandrel (1 - pi/4) r^2 swept around the rim at its centroid radius
        let pi = std::f64::consts::PI;
        let ideal = 9.0 * (1.0 - pi / 4.0) * std::f64::consts::TAU * (3.0 + (10.0 - 3.0 * pi) / (12.0 - 3.0 * pi));
        println!("removed {} mm3 (about {})", removed, ideal);
    }
}

#[cfg(test)]
mod edge_lines {
    use super::*;
    #[test]
    fn fillet_edge_lines() {
        let s = box_solid(DVec3::ZERO, DVec3::new(20.0, 10.0, 10.0));
        let m = Mesh::from_solid(&s);
        let edges = sharp_edges(&m);
        let i = edges.iter().position(|e| (e.a.y - 10.0).abs() < 1e-9 && (e.a.z - 10.0).abs() < 1e-9 && (e.b.y - 10.0).abs() < 1e-9 && (e.b.z - 10.0).abs() < 1e-9).unwrap();
        let segs = selected_segments(&edges, &[edge_ref(&edges[i])]).unwrap();
        let (cut, _) = edge_tools(&m, &edges, &segs, 2.0, false);
        let r = Mesh::from_solid(&apply_edge_tools(&s, &cut, &[]));
        let mut map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (t, tri) in r.tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                map.entry((a.min(b), a.max(b))).or_default().push(t);
            }
        }
        let open = map.values().filter(|v| v.len() != 2).count();
        let sharp = map.values().filter(|v| v.len() == 2 && r.tri_normal[v[0]].dot(r.tri_normal[v[1]]) < 25f64.to_radians().cos()).count();
        println!("drawn edges {}, open/non-manifold {}, sharp {}", r.edges.len(), open, sharp);
        // the two lines where the rounding starts and ends are drawn (along y = 8 and z = 8)
        let along = |y: f64, z: f64| r.edges.iter().filter(|(a, b)| (a.y - y).abs() < 1e-6 && (b.y - y).abs() < 1e-6 && (a.z - z).abs() < 1e-6 && (b.z - z).abs() < 1e-6).count();
        assert!(along(8.0, 10.0) > 0 && along(10.0, 8.0) > 0, "boundaries not drawn");
        let covered = |y: f64, z: f64| -> f64 {
            r.edges.iter().filter(|(a, b)| (a.y - y).abs() < 1e-6 && (b.y - y).abs() < 1e-6 && (a.z - z).abs() < 1e-6 && (b.z - z).abs() < 1e-6).map(|(a, b)| (a.x - b.x).abs()).sum()
        };
        println!("boundary coverage: top {:.2} of 20, side {:.2} of 20", covered(8.0, 10.0), covered(10.0, 8.0));
        // nothing is drawn along the inside of the rounding
        let inside = r.edges.iter().filter(|(a, b)| a.y > 8.01 && a.z > 8.01 && b.y > 8.01 && b.z > 8.01 && (a.x - b.x).abs() > 1e-6).count();
        assert_eq!(inside, 0, "lines drawn across the rounded surface");
    }
}

#[cfg(test)]
mod rim_lines {
    use super::*;
    /// Lines drawn strictly inside the rounded band of a filleted cylinder rim.
    fn inside_count(radius: f64, size: f64, segs_check: bool) -> (usize, usize) {
        let body = cylinder_solid(DVec3::ZERO, DVec3::Z, radius, 10.0);
        let m = Mesh::from_solid(&body);
        let edges = sharp_edges(&m);
        let i = edges.iter().position(|e| e.a.z > 9.99 && e.b.z > 9.99).unwrap();
        let segs = selected_segments(&edges, &[edge_ref(&edges[i])]).unwrap();
        let (cut, _) = edge_tools(&m, &edges, &segs, size, false);
        let r = Mesh::from_solid(&apply_edge_tools(&body, &cut, &[]));
        let rr = |p: DVec3| (p.x * p.x + p.y * p.y).sqrt();
        let inside = |p: DVec3| rr(p) > radius - size + 0.05 && rr(p) < radius - 0.05 && p.z > 10.0 - size + 0.05 && p.z < 9.95;
        let bad = r.edges.iter().filter(|(a, b)| inside(*a) || inside(*b) || inside((*a + *b) * 0.5)).count();
        // the start (on the top face) and end (on the wall) of the rounding are drawn
        let on_top = r.edges.iter().filter(|(a, b)| a.z > 9.999 && b.z > 9.999 && (rr(*a) - (radius - size)).abs() < 0.2).count();
        let on_wall = r.edges.iter().filter(|(a, b)| (rr(*a) - radius).abs() < 0.2 && (a.z - (10.0 - size)).abs() < 0.05 && (b.z - (10.0 - size)).abs() < 0.05).count();
        assert!(on_top > 10 && on_wall > 10, "boundary lines missing: top {} wall {}", on_top, on_wall);
        let _ = segs_check;
        (bad, r.edges.len())
    }
    #[test]
    fn rim_fillet_lines() {
        for (radius, size) in [(10.0, 2.0), (10.0, 5.0), (30.0, 1.0)] {
            let (bad, total) = inside_count(radius, size, true);
            println!("R{} fillet {}: {} of {} lines across the rounding", radius, size, bad, total);
            assert_eq!(bad, 0, "R{} fillet {}", radius, size);
        }
    }
}


#[cfg(test)]
mod curved_faces {
    use super::*;

    /// Fillets / chamfers the edge picked by `pick`, then counts the creases (sharp mesh edges)
    /// left inside `region` — a clean rounding blends into both faces without any.
    fn creases_after(body: &Solid, pick: &dyn Fn(&SharpEdge) -> bool, size: f64, chamfer: bool, region: &dyn Fn(DVec3) -> bool) -> (usize, Solid) {
        let m = Mesh::from_solid(body);
        let edges = sharp_edges(&m);
        let i = edges.iter().position(|e| pick(e)).expect("edge to fillet");
        let segs = selected_segments(&edges, &[edge_ref(&edges[i])]).unwrap();
        let (cut, add) = edge_tools(&m, &edges, &segs, size, chamfer);
        let out = apply_edge_tools(&body, &cut, &add);
        let r = Mesh::from_solid(&out);
        // creases sharper than the facets of the round faces themselves (9° here)
        let mut map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (t, tri) in r.tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                map.entry((a.min(b), a.max(b))).or_default().push(t);
            }
        }
        let cos = 12f64.to_radians().cos();
        let left = map
            .iter()
            .filter(|(k, ts)| ts.len() == 2 && r.tri_normal[ts[0]].dot(r.tri_normal[ts[1]]) < cos && region(r.pos[k.0 as usize]) && region(r.pos[k.1 as usize]))
            .count();
        (left, out)
    }

    /// A block with a round groove along X: the top edge of the groove has a face that
    /// curves away (concavely) across the edge.
    #[test]
    fn groove_edge_fillet_leaves_no_ridge() {
        let block = box_solid(DVec3::new(0.0, -20.0, 0.0), DVec3::new(40.0, 20.0, 20.0));
        let groove = cylinder_solid(DVec3::new(-1.0, 0.0, 25.0), DVec3::X, 10.0, 42.0);
        let body = block.subtract(&groove);
        let y0 = 75f64.sqrt();
        let pick = |e: &SharpEdge| (e.a.z - 20.0).abs() < 1e-6 && (e.b.z - 20.0).abs() < 1e-6 && (e.a.y - y0).abs() < 0.2 && (e.b.y - y0).abs() < 0.2 && (e.a.x - e.b.x).abs() > 1.0;
        let region = |p: DVec3| p.x > 1.0 && p.x < 39.0 && p.y > 3.0 && p.y < 14.0 && p.z > 13.0;
        for chamfer in [false, true] {
            let (left, out) = creases_after(&body, &pick, 3.0, chamfer, &region);
            if !chamfer {
                // the ball (radius 3) touches the top (z = 20) and the groove wall (radius 10
                // about y = 0, z = 25): no material may remain between it and the old edge
                let c = DVec2::new((13.0f64 * 13.0 - 8.0 * 8.0).sqrt(), 17.0);
                let (ct, cw) = (DVec2::new(c.x, 20.0), DVec2::new(0.0, 25.0) + (c - DVec2::new(0.0, 25.0)).normalize() * 10.0);
                let edge = DVec2::new(y0, 20.0);
                let inside = |q: DVec2| [(edge, ct), (ct, cw), (cw, edge)].iter().all(|(a, b)| (*b - *a).perp_dot(q - *a) >= -1e-6) || [(edge, ct), (ct, cw), (cw, edge)].iter().all(|(a, b)| (*b - *a).perp_dot(q - *a) <= 1e-6);
                let r = Mesh::from_solid(&out);
                let left_over = r.pos.iter().filter(|p| p.x > 1.0 && p.x < 39.0 && inside(DVec2::new(p.y, p.z)) && (DVec2::new(p.y, p.z) - c).length() < 2.9).count();
                assert_eq!(left_over, 0, "material of the old edge is left inside the rounding");
            }
            if chamfer {
                // the chamfer takes the whole old edge away
                let r = Mesh::from_solid(&out);
                let near = r.pos.iter().filter(|p| p.x > 1.0 && p.x < 39.0 && DVec2::new(p.y - y0, p.z - 20.0).length() < 0.5).count();
                assert_eq!(near, 0, "the old edge still stands next to the chamfer");
            } else {
                assert_eq!(left, 0, "creases left along the groove fillet");
            }
        }
    }

    /// A cylinder with a flat cut along its length: the round face bends away (convexly)
    /// across the edge.
    #[test]
    fn flat_on_cylinder_fillet_is_tangent() {
        let cyl = cylinder_solid(DVec3::ZERO, DVec3::Z, 10.0, 20.0);
        let cut = box_solid(DVec3::new(5.0, -20.0, -1.0), DVec3::new(20.0, 20.0, 21.0));
        let body = cyl.subtract(&cut);
        let pick = |e: &SharpEdge| (e.a.x - 5.0).abs() < 0.05 && (e.b.x - 5.0).abs() < 0.05 && e.a.y > 0.0 && (e.a.z - e.b.z).abs() > 1.0;
        let region = |p: DVec3| p.z > 1.0 && p.z < 19.0 && p.y > 3.0 && p.x > 0.0;
        let (left, _) = creases_after(&body, &pick, 3.0, false, &region);
        assert_eq!(left, 0, "creases left where the fillet meets the round face");
    }
}


#[cfg(test)]
mod corners {
    use super::*;

    fn pick_edges(m: &Mesh, want: &dyn Fn(&SharpEdge) -> bool) -> (Vec<SharpEdge>, Vec<EdgeRef>) {
        let edges = sharp_edges(m);
        let refs = edges.iter().filter(|e| want(e)).map(edge_ref).collect();
        (edges, refs)
    }

    /// Mesh edges not shared by exactly two triangles: how many, and the longest.
    fn defects(r: &Mesh) -> (usize, f64) {
        let mut map: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &r.tris {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                *map.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let bad: Vec<_> = map.iter().filter(|(_, c)| **c != 2).collect();
        (bad.len(), bad.iter().map(|(k, _)| (r.pos[k.0 as usize] - r.pos[k.1 as usize]).length()).fold(0.0, f64::max))
    }
    fn open_edges(r: &Mesh) -> usize {
        defects(r).0
    }
    /// Where faces meet tangentially right beside another face (inside corners, added
    /// coves) the BSP booleans can leave a few zero-thickness slivers (lying within a face, so
    /// they cannot be seen); allow those, nothing more.
    fn nearly_clean(r: &Mesh) {
        let (n, _) = defects(r);
        assert!(n <= 6, "{} mesh defects", n);
    }

    /// Three filleted edges meeting at a box corner: a ball-shaped corner, as in CATIA.
    #[test]
    fn three_edges_make_a_ball_corner() {
        let s = box_solid(DVec3::ZERO, DVec3::splat(20.0));
        let m = Mesh::from_solid(&s);
        let corner = DVec3::splat(20.0);
        let (edges, refs) = pick_edges(&m, &|e| (e.a - corner).length() < 1e-6 || (e.b - corner).length() < 1e-6);
        assert_eq!(refs.len(), 3);
        let segs = selected_segments(&edges, &refs).unwrap();
        let (cut, _) = edge_tools(&m, &edges, &segs, 4.0, false);
        let out = apply_edge_tools(&s, &cut, &[]);
        let r = Mesh::from_solid(&out);
        assert_eq!(open_edges(&r), 0);
        // removed: three quarter-round spandrels along 16 mm and the corner cube minus a ball octant
        let pi = std::f64::consts::PI;
        let ideal = 3.0 * 16.0 * 16.0 * (1.0 - pi / 4.0) + 64.0 - pi * 64.0 / 6.0;
        let removed = 8000.0 - out.volume();
        assert!((removed - ideal).abs() < 0.03 * ideal, "removed {} vs {}", removed, ideal);
        // the rounded corner is smooth: no point of it sticks out of the ball
        let c = DVec3::splat(16.0);
        let out_of_ball = r.pos.iter().filter(|p| p.x > 16.05 && p.y > 16.05 && p.z > 16.05 && (**p - c).length() > 4.05).count();
        assert_eq!(out_of_ball, 0, "corner not rounded");
        let lines = r.edges.iter().filter(|(a, b)| [*a, *b].iter().all(|p| p.x > 16.3 && p.y > 16.3 && p.z > 16.3)).count();
        assert_eq!(lines, 0, "lines drawn across the ball corner");
    }

    /// Two filleted top edges meeting at an outside corner: both fillets run on past the
    /// corner and cross each other there.
    #[test]
    fn two_edges_at_outside_corner() {
        let s = box_solid(DVec3::ZERO, DVec3::new(30.0, 20.0, 10.0));
        let m = Mesh::from_solid(&s);
        let (edges, refs) = pick_edges(&m, &|e| e.a.z > 9.99 && e.b.z > 9.99 && ((e.a.x > 29.99 && e.b.x > 29.99) || (e.a.y > 19.99 && e.b.y > 19.99)));
        assert_eq!(refs.len(), 2);
        let segs = selected_segments(&edges, &refs).unwrap();
        let (cut, _) = edge_tools(&m, &edges, &segs, 3.0, false);
        let out = apply_edge_tools(&s, &cut, &[]);
        assert_eq!(open_edges(&Mesh::from_solid(&out)), 0);
        let pi = std::f64::consts::PI;
        let spandrel = 9.0 * (1.0 - pi / 4.0);
        // both spandrels along their full edges, less the bit at the corner both take away
        // (r³ (5/3 - π/2))
        let ideal = spandrel * (20.0 + 30.0) - 27.0 * (5.0 / 3.0 - pi / 2.0);
        let removed = 6000.0 - out.volume();
        assert!((removed - ideal).abs() < 0.02 * ideal, "removed {} vs {}", removed, ideal);
    }

    /// An L-shaped plate: two filleted top edges meet at the inside corner of the L. Neither
    /// tool may cut into the other leg.
    #[test]
    fn inside_corner_is_not_gouged() {
        let a = box_solid(DVec3::ZERO, DVec3::new(40.0, 10.0, 10.0));
        let b = box_solid(DVec3::ZERO, DVec3::new(10.0, 40.0, 10.0));
        let s = a.union(&b);
        let m = Mesh::from_solid(&s);
        // top edges at y = 10 (x from 10 to 40) and x = 10 (y from 10 to 40)
        let (edges, refs) = pick_edges(&m, &|e| e.a.z > 9.99 && e.b.z > 9.99 && (((e.a.y - 10.0).abs() < 1e-6 && (e.b.y - 10.0).abs() < 1e-6 && e.a.x > 9.99 && e.b.x > 9.99) || ((e.a.x - 10.0).abs() < 1e-6 && (e.b.x - 10.0).abs() < 1e-6 && e.a.y > 9.99 && e.b.y > 9.99)));
        assert!(refs.len() >= 2);
        let segs = selected_segments(&edges, &refs).unwrap();
        let (cut, _) = edge_tools(&m, &edges, &segs, 3.0, false);
        let out = apply_edge_tools(&s, &cut, &[]);
        let r = Mesh::from_solid(&out);
        nearly_clean(&r);
        // the other leg keeps its material right beside the corner; the filleted edges lose theirs
        let inside = |p: DVec3| {
            let d = DVec3::new(0.31, 0.17, 1.0).normalize();
            (0..r.tris.len()).filter(|t| ray_tri(p, d, &r.tri_pts(*t)).is_some()).count() % 2 == 1
        };
        for p in [DVec3::new(6.0, 8.5, 9.5), DVec3::new(3.0, 9.0, 9.8), DVec3::new(8.5, 6.0, 9.5), DVec3::new(9.0, 3.0, 9.8)] {
            assert!(inside(p), "material gone at {:?}: a fillet cut into the other leg", p);
        }
        // ... and the inside corner itself is rounded where the two fillets meet (mitre x = y)
        for p in [DVec3::new(25.0, 9.8, 9.8), DVec3::new(9.8, 25.0, 9.8), DVec3::new(12.0, 9.8, 9.8), DVec3::new(9.5, 9.5, 9.9)] {
            assert!(!inside(p), "edge not rounded at {:?}", p);
        }
    }

    /// The inside (concave) edge of an L-section adds material: a cove of radius r.
    #[test]
    fn concave_fillet_adds_a_cove() {
        let a = box_solid(DVec3::ZERO, DVec3::new(30.0, 30.0, 5.0));
        let b = box_solid(DVec3::ZERO, DVec3::new(30.0, 5.0, 30.0));
        let s = a.union(&b);
        let m = Mesh::from_solid(&s);
        let (edges, refs) = pick_edges(&m, &|e| !e.convex && (e.a.y - 5.0).abs() < 1e-6 && (e.a.z - 5.0).abs() < 1e-6 && (e.b.y - 5.0).abs() < 1e-6);
        assert!(!refs.is_empty());
        let segs = selected_segments(&edges, &refs).unwrap();
        let (cut, add) = edge_tools(&m, &edges, &segs, 4.0, false);
        assert!(cut.is_empty());
        let out = apply_edge_tools(&s, &cut, &add);
        nearly_clean(&Mesh::from_solid(&out));
        let ideal = 16.0 * (1.0 - std::f64::consts::PI / 4.0) * 30.0;
        let added = out.volume() - s.volume();
        assert!((added - ideal).abs() < 0.03 * ideal, "added {} vs {}", added, ideal);
    }
}
