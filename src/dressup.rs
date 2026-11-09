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
        out.push(SharpEdge { a, b, n1, n2, convex });
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

fn slerp_unit(a: DVec3, b: DVec3, f: f64) -> DVec3 {
    let d = a.dot(b).clamp(-1.0, 1.0);
    let th = d.acos();
    if th < 1e-9 {
        return a;
    }
    (a * ((1.0 - f) * th).sin() + b * (f * th).sin()) / th.sin()
}

/// Cross-section of a fillet / chamfer at an edge point, as offsets from that point.
fn edge_section(n1: DVec3, n2: DVec3, t: DVec3, convex: bool, size: f64, chamfer: bool, eps: f64) -> Vec<DVec3> {
    let sgn = if convex { 1.0 } else { -1.0 };
    let c = n1.dot(n2);
    let side = |n: DVec3, other: DVec3| -> DVec3 {
        let w = t.cross(n).normalize_or_zero();
        if sgn * w.dot(other) < 0.0 { w } else { -w }
    };
    let out = (n1 + n2).normalize_or_zero() * sgn * eps;
    if chamfer {
        let (w1, w2) = (side(n1, n2), side(n2, n1));
        return vec![out, w1 * size + n1 * sgn * eps, w1 * size, w2 * size, w2 * size + n2 * sgn * eps];
    }
    let x = -(n1 + n2) * (sgn * size / (1.0 + c));
    let t1 = x + n1 * sgn * size;
    let t2 = x + n2 * sgn * size;
    let mut v = vec![out, t1 + n1 * sgn * eps, t1];
    const K: usize = 16;
    for k in 1..K {
        v.push(x + slerp_unit(n1 * sgn, n2 * sgn, k as f64 / K as f64) * size);
    }
    v.push(t2);
    v.push(t2 + n2 * sgn * eps);
    v
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

/// Builds the material to remove (convex edges) and add (concave edges) for a fillet
/// (radius) or chamfer (length) along the given edge segments. Each smooth run of edges is
/// swept as one piece, so curved edges (hole rims, round pads) stay fast.
pub fn edge_tools(edges: &[SharpEdge], segs: &[usize], size: f64, chamfer: bool) -> (Solid, Solid) {
    let mut count: HashMap<(i64, i64, i64), usize> = HashMap::new();
    for &i in segs {
        *count.entry(vkey(edges[i].a)).or_default() += 1;
        *count.entry(vkey(edges[i].b)).or_default() += 1;
    }
    let eps = 0.02f64.min(size * 0.05).max(1e-3);
    let mut cuts = Vec::new();
    let mut adds = Vec::new();
    for (run, closed) in edge_runs(edges, segs) {
        // points along the run and the (consistently ordered) face normals of each segment
        let mut pts = Vec::new();
        let mut normals: Vec<(DVec3, DVec3)> = Vec::new();
        for (k, &(i, rev)) in run.iter().enumerate() {
            let e = edges[i];
            let (a, b) = if rev { (e.b, e.a) } else { (e.a, e.b) };
            if k == 0 {
                pts.push(a);
            }
            pts.push(b);
            let (mut n1, mut n2) = (e.n1, e.n2);
            if let Some(&(p1, _)) = normals.last() {
                if n2.dot(p1) > n1.dot(p1) {
                    std::mem::swap(&mut n1, &mut n2);
                }
            }
            normals.push((n1, n2));
        }
        let convex = edges[run[0].0].convex;
        let m = run.len();
        if closed {
            pts.pop();
        }
        let np = pts.len();
        let seg_dir = |k: usize| (pts[(k + 1) % np] - pts[k]).normalize_or_zero();
        let mut rings = Vec::with_capacity(np);
        for i in 0..np {
            // segments before and after this point
            let before = if i > 0 { Some(i - 1) } else if closed { Some(m - 1) } else { None };
            let after = if i < m { Some(i) } else { None };
            let (mut n1, mut n2, mut t) = (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
            for k in [before, after].into_iter().flatten() {
                n1 += normals[k].0;
                n2 += normals[k].1;
                t += seg_dir(k % np);
            }
            let (n1, n2, t) = (n1.normalize_or_zero(), n2.normalize_or_zero(), t.normalize_or_zero());
            let mut p = pts[i];
            if !closed && (i == 0 || i == np - 1) {
                // run ends: reach a little past free ends, overlap where other runs meet
                let ext = if count.get(&vkey(p)).copied().unwrap_or(0) > 1 { size * 0.3 } else { eps };
                p += if i == 0 { -t * ext } else { t * ext };
            }
            rings.push(edge_section(n1, n2, t, convex, size, chamfer, eps).into_iter().map(|o| p + o).collect::<Vec<_>>());
        }
        if rings.len() < 2 {
            continue;
        }
        if let Ok(piece) = skin(&rings, closed) {
            if convex {
                cuts.push(piece);
            } else {
                adds.push(piece);
            }
        }
    }
    (join_disjoint(cuts), join_disjoint(adds))
}

/// Unions solids, merging only pieces whose boxes overlap; separate groups are simply put
/// together (they cannot intersect), which keeps many small pieces fast.
pub fn join_disjoint(pieces: Vec<Solid>) -> Solid {
    let n = pieces.len();
    let boxes: Vec<Aabb> = pieces.iter().map(|p| p.bbox()).collect();
    let mut group: Vec<usize> = (0..n).collect();
    fn root(g: &mut Vec<usize>, i: usize) -> usize {
        let mut r = i;
        while g[r] != r {
            r = g[r];
        }
        g[i] = r;
        r
    }
    for i in 0..n {
        for j in (i + 1)..n {
            if boxes[i].overlaps(&boxes[j]) {
                let (a, b) = (root(&mut group, i), root(&mut group, j));
                group[a] = b;
            }
        }
    }
    let mut merged: HashMap<usize, Solid> = HashMap::new();
    for (i, p) in pieces.into_iter().enumerate() {
        let r = root(&mut group, i);
        let e = merged.entry(r).or_default();
        *e = if e.is_empty() { p } else { e.union(&p) };
    }
    let mut out = Solid::default();
    let mut keys: Vec<usize> = merged.keys().cloned().collect();
    keys.sort_unstable();
    for k in keys {
        out.polys.extend(merged.remove(&k).unwrap().polys);
    }
    out
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
        let (cut, add) = edge_tools(&edges, &segs, 2.0, false);
        assert!(add.is_empty());
        let filleted = s.subtract(&cut);
        let removed = 2000.0 - filleted.volume();
        let expect = (4.0 - std::f64::consts::PI) * 20.0; // (r² - πr²/4) · length
        assert!((removed - expect).abs() < 0.3, "{} vs {}", removed, expect);
        let (cut, _) = edge_tools(&edges, &segs, 2.0, true);
        let chamfered = s.subtract(&cut);
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
        let (cut, _) = edge_tools(&edges, &segs, 1.0, false);
        println!("{} refs, {} segments, tool {} polys: {:?}", refs.len(), segs.len(), cut.polys.len(), t.elapsed());
        let t = Instant::now();
        let r = body.subtract(&cut);
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
        let (cut, _) = edge_tools(&edges, &segs, 2.0, false);
        let r = Mesh::from_solid(&s.subtract(&cut));
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
        let (cut, _) = edge_tools(&edges, &segs, size, false);
        let r = Mesh::from_solid(&body.subtract(&cut));
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

