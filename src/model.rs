//! Documents: parts (a history of features rebuilt into a solid) and assemblies
//! (products made of part components placed by constraints).

use crate::geom::*;
pub use crate::dressup::*;
use glam::{DMat4, DQuat, DVec2, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};
use std::sync::Arc;

pub use crate::sketch::*;

// ---------------------------------------------------------------- features

#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Debug)]
pub enum Axis3 {
    X,
    Y,
    Z,
}

impl Axis3 {
    pub fn v(self) -> DVec3 {
        match self {
            Axis3::X => DVec3::X,
            Axis3::Y => DVec3::Y,
            Axis3::Z => DVec3::Z,
        }
    }
    pub fn next(self) -> Axis3 {
        match self {
            Axis3::X => Axis3::Y,
            Axis3::Y => Axis3::Z,
            Axis3::Z => Axis3::X,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Axis3::X => "X",
            Axis3::Y => "Y",
            Axis3::Z => "Z",
        }
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Debug)]
pub enum PrimKind {
    Box,
    Cylinder,
    Sphere,
}

#[derive(Clone, Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum FeatureKind {
    Sketch(Sketch),
    Pad { sketch: u32, length: f64, symmetric: bool, reverse: bool },
    Pocket { sketch: u32, depth: f64, through: bool, reverse: bool },
    Shaft { sketch: u32, angle: f64, axis_h: bool },
    Groove { sketch: u32, angle: f64, axis_h: bool },
    Hole {
        pos: DVec3,
        dir: DVec3,
        diameter: f64,
        depth: f64,
        through: bool,
        /// 0 simple, 1 counterbored, 2 countersunk
        #[serde(default)]
        htype: u8,
        /// counterbore / countersink diameter, counterbore depth, countersink angle
        #[serde(default)]
        d2: f64,
        #[serde(default)]
        depth2: f64,
        #[serde(default)]
        angle2: f64,
    },
    Primitive { prim: PrimKind, pos: DVec3, a: f64, b: f64, c: f64, axis: Axis3, subtract: bool },
    Mirror { plane: Axis3 },
    RectPattern { feature: u32, dir: Axis3, count: f64, step: f64, dir2: Axis3, count2: f64, step2: f64 },
    CircPattern { feature: u32, axis: Axis3, count: f64, total: f64 },
    Import { file: String, tris: Arc<Vec<[f32; 9]>>, scale: f64 },
    // dress-up features
    Fillet { edges: Vec<EdgeRef>, radius: f64 },
    Chamfer { edges: Vec<EdgeRef>, length: f64 },
    Draft { faces: Vec<FaceRef>, angle: f64, neutral: Axis3, reverse: bool },
    Shell { faces: Vec<FaceRef>, thickness: f64 },
    Thickness { faces: Vec<FaceRef>, thickness: f64 },
    // transformations of the whole body
    Translate { v: DVec3 },
    Rotate { axis: Axis3, angle: f64 },
    Symmetry { plane: Axis3 },
    Scale { factor: f64 },
    /// reference plane: base 0 xy, 1 yz, 2 zx, 3 a face; moved along its normal by offset
    RefPlane { base: u8, face: Option<FaceRef>, offset: f64 },
    /// multi-sections solid (or removed multi-sections solid)
    Loft { sections: Vec<u32>, remove: bool },
    /// rib (or slot): a profile swept along a centre curve
    Sweep { profile: u32, path: u32, remove: bool },
    /// cut the body with a plane (base 0 xy, 1 yz, 2 zx, 3 a reference plane), keeping one side
    Split { base: u8, plane: u32, offset: f64, flip: bool },
    /// stiffener: the open lines of a sketch thickened into ribs reaching into the part
    Stiffener { sketch: u32, thickness: f64, depth: f64, reverse: bool },
}

impl FeatureKind {
    pub fn type_name(&self) -> &'static str {
        match self {
            FeatureKind::Sketch(_) => "Sketch",
            FeatureKind::Pad { .. } => "Pad",
            FeatureKind::Pocket { .. } => "Pocket",
            FeatureKind::Shaft { .. } => "Shaft",
            FeatureKind::Groove { .. } => "Groove",
            FeatureKind::Hole { .. } => "Hole",
            FeatureKind::Primitive { prim: PrimKind::Box, .. } => "Box",
            FeatureKind::Primitive { prim: PrimKind::Cylinder, .. } => "Cylinder",
            FeatureKind::Primitive { prim: PrimKind::Sphere, .. } => "Sphere",
            FeatureKind::Mirror { .. } => "Mirror",
            FeatureKind::RectPattern { .. } => "RectPattern",
            FeatureKind::CircPattern { .. } => "CircPattern",
            FeatureKind::Import { .. } => "ImportedSolid",
            FeatureKind::Fillet { .. } => "EdgeFillet",
            FeatureKind::Chamfer { .. } => "Chamfer",
            FeatureKind::Draft { .. } => "Draft",
            FeatureKind::Shell { .. } => "Shell",
            FeatureKind::Thickness { .. } => "Thickness",
            FeatureKind::Translate { .. } => "Translation",
            FeatureKind::Rotate { .. } => "Rotation",
            FeatureKind::Symmetry { .. } => "Symmetry",
            FeatureKind::Scale { .. } => "Scaling",
            FeatureKind::RefPlane { .. } => "Plane",
            FeatureKind::Loft { remove: false, .. } => "MultiSectionsSolid",
            FeatureKind::Loft { remove: true, .. } => "RemovedMultiSections",
            FeatureKind::Sweep { remove: false, .. } => "Rib",
            FeatureKind::Sweep { remove: true, .. } => "Slot",
            FeatureKind::Split { .. } => "Split",
            FeatureKind::Stiffener { .. } => "Stiffener",
        }
    }
    /// Every sketch or feature this feature is built from.
    pub fn refs(&self) -> Vec<u32> {
        let mut v: Vec<u32> = self.sketch_ref().into_iter().chain(self.feature_ref()).collect();
        match self {
            FeatureKind::Loft { sections, .. } => v.extend(sections.iter().cloned()),
            FeatureKind::Sweep { profile, path, .. } => {
                v.push(*profile);
                v.push(*path);
            }
            FeatureKind::Stiffener { sketch, .. } => v.push(*sketch),
            FeatureKind::Split { base: 3, plane, .. } => v.push(*plane),
            _ => {}
        }
        v
    }
    pub fn sketch_ref(&self) -> Option<u32> {
        match self {
            FeatureKind::Pad { sketch, .. } | FeatureKind::Pocket { sketch, .. } | FeatureKind::Shaft { sketch, .. } | FeatureKind::Groove { sketch, .. } => Some(*sketch),
            _ => None,
        }
    }
    pub fn feature_ref(&self) -> Option<u32> {
        match self {
            FeatureKind::RectPattern { feature, .. } | FeatureKind::CircPattern { feature, .. } => Some(*feature),
            _ => None,
        }
    }
    /// Features whose tool body can be patterned.
    pub fn patternable(&self) -> bool {
        matches!(
            self,
            FeatureKind::Pad { .. } | FeatureKind::Pocket { .. } | FeatureKind::Shaft { .. } | FeatureKind::Groove { .. } | FeatureKind::Hole { .. } | FeatureKind::Primitive { .. }
        )
    }
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Feature {
    pub id: u32,
    pub name: String,
    pub kind: FeatureKind,
    #[serde(default)]
    pub hidden: bool,
    #[serde(default)]
    pub suppressed: bool,
}

/// The surface finish a part is shown with in realistic rendering.
#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Debug, Default)]
pub enum Finish {
    /// neutral, smooth plastic
    #[default]
    Plastic,
    /// FDM 3D print: matte with fine layer lines
    Print3D,
    /// machined metal (the colour sets which: grey steel, gold brass, ...)
    Metal,
    /// wood grain (the default grey shows natural oak; other colours stain it)
    Wood,
}

impl Finish {
    pub const ALL: [Finish; 4] = [Finish::Plastic, Finish::Print3D, Finish::Metal, Finish::Wood];
    pub fn name(self) -> &'static str {
        match self {
            Finish::Plastic => "Plastic",
            Finish::Print3D => "3D Print",
            Finish::Metal => "Metal",
            Finish::Wood => "Wood",
        }
    }
    pub fn index(self) -> usize {
        Finish::ALL.iter().position(|f| *f == self).unwrap_or(0)
    }
}

#[derive(Clone, Default)]
pub struct PartCache {
    /// body after each feature
    pub bodies: Vec<Arc<Solid>>,
    /// tool solid and "adds material" per feature id
    pub tools: HashMap<u32, (Arc<Solid>, bool)>,
    pub errors: HashMap<u32, String>,
    pub valid_upto: usize,
    pub mesh: Option<Arc<Mesh>>,
    pub dirty: bool,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Part {
    pub name: String,
    pub features: Vec<Feature>,
    pub next_id: u32,
    pub counters: HashMap<String, u32>,
    pub color: [f32; 3],
    #[serde(default)]
    pub finish: Finish,
    #[serde(skip)]
    pub cache: PartCache,
    /// bumped on every change, so a background rebuild of an older state is thrown away
    #[serde(skip)]
    pub version: u64,
    #[serde(skip)]
    pub job: JobSlot,
}

/// A rebuild running in the background.
pub struct BuildJob {
    handle: std::thread::JoinHandle<PartCache>,
    version: u64,
}

/// Holds the running rebuild, if any. Copies of a part (undo snapshots) never share it.
#[derive(Default)]
pub struct JobSlot(Option<BuildJob>);

impl Clone for JobSlot {
    fn clone(&self) -> Self {
        JobSlot(None)
    }
}

pub const DEFAULT_PART_COLOR: [f32; 3] = [0.72, 0.74, 0.78];

impl Part {
    pub fn new(name: &str) -> Part {
        let mut p = Part { name: name.into(), features: vec![], next_id: 1, counters: HashMap::new(), color: DEFAULT_PART_COLOR, finish: Finish::Plastic, cache: PartCache::default(), version: 0, job: JobSlot::default() };
        p.cache.dirty = true;
        p
    }

    pub fn add_feature(&mut self, kind: FeatureKind) -> u32 {
        let t = kind.type_name().to_string();
        let c = self.counters.entry(t.clone()).or_insert(0);
        *c += 1;
        let id = self.next_id;
        self.next_id += 1;
        self.features.push(Feature { id, name: format!("{}.{}", t, c), kind, hidden: false, suppressed: false });
        self.touch(self.features.len() - 1);
        id
    }

    pub fn index_of(&self, id: u32) -> Option<usize> {
        self.features.iter().position(|f| f.id == id)
    }
    pub fn feature(&self, id: u32) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }
    pub fn feature_mut(&mut self, id: u32) -> Option<&mut Feature> {
        self.features.iter_mut().find(|f| f.id == id)
    }

    /// Marks feature `idx` and everything after it for rebuilding.
    pub fn touch(&mut self, idx: usize) {
        self.cache.valid_upto = self.cache.valid_upto.min(idx);
        self.cache.dirty = true;
        self.version += 1;
    }
    pub fn touch_id(&mut self, id: u32) {
        if let Some(i) = self.index_of(id) {
            self.touch(i);
        }
    }

    /// Deletes a feature and every feature that depends on it. Returns names deleted.
    pub fn delete_feature(&mut self, id: u32) -> Vec<String> {
        let mut doomed = vec![id];
        let mut changed = true;
        while changed {
            changed = false;
            for f in &self.features {
                if doomed.contains(&f.id) {
                    continue;
                }
                if f.kind.refs().iter().any(|r| doomed.contains(r)) {
                    doomed.push(f.id);
                    changed = true;
                }
            }
        }
        let first = self.features.iter().position(|f| doomed.contains(&f.id)).unwrap_or(0);
        let names = self.features.iter().filter(|f| doomed.contains(&f.id)).map(|f| f.name.clone()).collect();
        self.features.retain(|f| !doomed.contains(&f.id));
        self.touch(first);
        names
    }

    fn start_job(&self) -> BuildJob {
        let start = self.cache.valid_upto.min(self.features.len()).min(self.cache.bodies.len());
        let feats = self.features.clone();
        let mut cache = self.cache.clone();
        cache.bodies.truncate(start);
        // run the boolean work on a thread with a large stack: BSP trees recurse deeply
        let handle = std::thread::Builder::new()
            .stack_size(512 * 1024 * 1024)
            .spawn(move || {
                rebuild(&feats, start, &mut cache);
                cache.valid_upto = feats.len();
                cache.dirty = false;
                cache
            })
            .expect("thread");
        BuildJob { handle, version: self.version }
    }

    fn finish_job(&mut self, job: BuildJob) {
        let cache = job.handle.join().unwrap_or_default();
        if job.version == self.version {
            self.cache = cache;
        }
        // otherwise the part changed meanwhile: the result is out of date and dropped
    }

    /// Keeps the part rebuilding in the background without waiting. Returns true while busy.
    pub fn poll_build(&mut self) -> bool {
        if let Some(job) = &self.job.0 {
            if !job.handle.is_finished() {
                return true;
            }
            let job = self.job.0.take().unwrap();
            self.finish_job(job);
        }
        if self.cache.dirty {
            self.job.0 = Some(self.start_job());
            return true;
        }
        false
    }

    pub fn is_building(&self) -> bool {
        self.job.0.is_some() || self.cache.dirty
    }

    /// Waits until the part is fully rebuilt.
    pub fn ensure_built(&mut self) {
        if let Some(job) = self.job.0.take() {
            self.finish_job(job);
        }
        if !self.cache.dirty {
            return;
        }
        let job = self.start_job();
        self.finish_job(job);
    }

    /// The latest finished mesh, without waiting for a rebuild in progress.
    pub fn mesh_live(&self) -> Arc<Mesh> {
        self.cache.mesh.clone().unwrap_or_default()
    }

    pub fn mesh(&mut self) -> Arc<Mesh> {
        self.ensure_built();
        self.cache.mesh.clone().unwrap_or_default()
    }

    pub fn body(&self) -> Option<Arc<Solid>> {
        self.cache.bodies.last().cloned()
    }

    pub fn last_sketch_unused(&self) -> Option<u32> {
        let used: Vec<u32> = self.features.iter().flat_map(|f| f.kind.refs()).collect();
        self.features.iter().rev().find(|f| matches!(f.kind, FeatureKind::Sketch(_)) && !used.contains(&f.id)).map(|f| f.id)
    }
    /// Sketches no feature uses yet, in tree order.
    pub fn unused_sketches(&self) -> Vec<u32> {
        let used: Vec<u32> = self.features.iter().flat_map(|f| f.kind.refs()).collect();
        self.features.iter().filter(|f| matches!(f.kind, FeatureKind::Sketch(_)) && !used.contains(&f.id)).map(|f| f.id).collect()
    }
    pub fn sketches(&self) -> Vec<u32> {
        self.features.iter().filter(|f| matches!(f.kind, FeatureKind::Sketch(_))).map(|f| f.id).collect()
    }
    /// The placement of a reference plane feature.
    pub fn ref_plane_frame(&self, id: u32) -> Option<Frame> {
        ref_plane_frame_of(&self.feature(id)?.kind)
    }
    /// Reference plane features, by id and name.
    pub fn ref_planes(&self) -> Vec<(u32, String)> {
        self.features.iter().filter(|f| matches!(f.kind, FeatureKind::RefPlane { .. })).map(|f| (f.id, f.name.clone())).collect()
    }
    pub fn last_sketch(&self) -> Option<u32> {
        self.features.iter().rev().find(|f| matches!(f.kind, FeatureKind::Sketch(_))).map(|f| f.id)
    }
}

fn through_length(body: &Solid) -> f64 {
    let b = body.bbox();
    if b.valid() {
        (b.size().length() + b.min.abs().max(b.max.abs()).length()) * 2.0 + 10.0
    } else {
        1000.0
    }
}

fn sketch_solid(sk: &Sketch, build: &dyn Fn(&[DVec2]) -> Result<Solid, String>) -> Result<Solid, String> {
    let (solids, holes) = sk.regions()?;
    let mut add = Solid::default();
    for s in &solids {
        add = add.union(&build(s)?);
    }
    for h in &holes {
        add = add.subtract(&build(h)?);
    }
    if add.is_empty() {
        return Err("nothing to build".into());
    }
    Ok(add)
}

fn axis_dirs(axis: Axis3) -> (DVec3, DVec3) {
    let a = axis.v();
    let b = axis.next().v();
    (a, b)
}

/// Placement of a reference plane feature.
pub fn ref_plane_frame_of(kind: &FeatureKind) -> Option<Frame> {
    match kind {
        FeatureKind::RefPlane { base, face, offset } => {
            let f = match (base, face) {
                (0, _) => Frame::xy(),
                (1, _) => Frame::yz(),
                (2, _) => Frame::zx(),
                (_, Some(fr)) => Frame::from_normal(fr.p, fr.n),
                _ => Frame::xy(),
            };
            Some(Frame { o: f.o + f.n() * *offset, ..f })
        }
        _ => None,
    }
}

/// The centre curve of a sketch for a rib / slot: its open elements joined end to end
/// (or one closed outline). Returns the points and whether the curve is closed.
fn sketch_path(s: &Sketch) -> Result<(Vec<DVec2>, bool), String> {
    let mut opens: Vec<Vec<DVec2>> = Vec::new();
    for (i, sh) in s.shapes.iter().enumerate() {
        if s.is_construction(i) {
            continue;
        }
        if sh.is_open() {
            opens.push(sh.curve());
        }
    }
    if opens.is_empty() {
        let first = s.shapes.iter().enumerate().find(|(i, sh)| !s.is_construction(*i) && !sh.is_open());
        return match first {
            Some((_, sh)) => Ok((sh.outline(), true)),
            None => Err("the centre curve sketch is empty".into()),
        };
    }
    // start at a free end
    let ends: Vec<DVec2> = opens.iter().flat_map(|o| [o[0], *o.last().unwrap()]).collect();
    let free = |p: DVec2| ends.iter().filter(|q| (**q - p).length() < JOIN_TOL).count() == 1;
    let mut start = 0;
    let mut flip = false;
    for (i, o) in opens.iter().enumerate() {
        if free(o[0]) {
            start = i;
            break;
        }
        if free(*o.last().unwrap()) {
            start = i;
            flip = true;
            break;
        }
    }
    let mut used = vec![false; opens.len()];
    used[start] = true;
    let mut chain = opens[start].clone();
    if flip {
        chain.reverse();
    }
    loop {
        let end = *chain.last().unwrap();
        let next = opens.iter().enumerate().find(|(j, o)| !used[*j] && ((o[0] - end).length() < JOIN_TOL || (*o.last().unwrap() - end).length() < JOIN_TOL));
        match next {
            Some((j, o)) => {
                let mut seg = o.clone();
                if (seg[0] - end).length() >= JOIN_TOL {
                    seg.reverse();
                }
                used[j] = true;
                chain.extend(seg.into_iter().skip(1));
            }
            None => break,
        }
    }
    if used.iter().any(|u| !u) {
        return Err("the centre curve must be one connected chain".into());
    }
    let closed = chain.len() > 2 && (chain[0] - *chain.last().unwrap()).length() < JOIN_TOL;
    if closed {
        chain.pop();
    }
    Ok((chain, closed))
}

/// Features that change the body directly (dress-ups, transformations, reference planes).
fn direct_body(kind: &FeatureKind, body: &Solid) -> Option<Result<Solid, String>> {
    let need_body = || -> Result<(), String> { if body.is_empty() { Err("there is no solid yet".into()) } else { Ok(()) } };
    let r = match kind {
        FeatureKind::Fillet { edges, radius: size } | FeatureKind::Chamfer { edges, length: size } => (|| {
            need_body()?;
            if edges.is_empty() {
                return Err("click edges in the 3D view to select them".into());
            }
            if *size <= 0.0 {
                return Err("the size must be positive".into());
            }
            let m = Mesh::from_solid(body);
            let sharp = sharp_edges(&m);
            let segs = selected_segments(&sharp, edges)?;
            let (cut, add) = edge_tools(&m, &sharp, &segs, *size, matches!(kind, FeatureKind::Chamfer { .. }));
            Ok(apply_edge_tools(body, &cut, &add))
        })(),
        FeatureKind::Draft { faces, angle, neutral, reverse } => (|| {
            need_body()?;
            if faces.is_empty() {
                return Err("click the faces to draft in the 3D view".into());
            }
            let m = Mesh::from_solid(body);
            let ids: Vec<u32> = faces.iter().map(|f| find_face(&m, f).ok_or("a face picked earlier no longer exists")).collect::<Result<_, _>>()?;
            let d = neutral.v() * if *reverse { -1.0 } else { 1.0 };
            let a = angle.to_radians();
            Ok(replane(&m, &|f, n, c| if ids.contains(&f) { (n * a.cos() + d * a.sin(), c * a.cos()) } else { (n, c) }).fix_orientation())
        })(),
        FeatureKind::Shell { faces, thickness } => (|| {
            need_body()?;
            if *thickness <= 0.0 {
                return Err("the wall thickness must be positive".into());
            }
            let m = Mesh::from_solid(body);
            let ids: Vec<u32> = faces.iter().map(|f| find_face(&m, f).ok_or("a face picked earlier no longer exists")).collect::<Result<_, _>>()?;
            let t = *thickness;
            let inner = replane(&m, &|f, n, c| if ids.contains(&f) { (n, c + t) } else { (n, c - t) }).fix_orientation();
            Ok(body.subtract(&inner))
        })(),
        FeatureKind::Thickness { faces, thickness } => (|| {
            need_body()?;
            if faces.is_empty() {
                return Err("click the faces to thicken in the 3D view".into());
            }
            let m = Mesh::from_solid(body);
            let ids: Vec<u32> = faces.iter().map(|f| find_face(&m, f).ok_or("a face picked earlier no longer exists")).collect::<Result<_, _>>()?;
            let t = *thickness;
            Ok(replane(&m, &|f, n, c| if ids.contains(&f) { (n, c + t) } else { (n, c) }).fix_orientation())
        })(),
        FeatureKind::Translate { v } => need_body().map(|_| body.translated(*v)),
        FeatureKind::Rotate { axis, angle } => need_body().map(|_| body.rotated(DVec3::ZERO, axis.v(), angle.to_radians())),
        FeatureKind::Symmetry { plane } => need_body().map(|_| {
            let n = plane.v();
            body.transformed(|p| p - n * (2.0 * p.dot(n)), true)
        }),
        FeatureKind::Scale { factor } => (|| {
            need_body()?;
            if *factor <= 0.0 {
                return Err("the ratio must be positive".into());
            }
            let k = *factor;
            Ok(body.transformed(|p| p * k, false))
        })(),
        FeatureKind::RefPlane { .. } => Ok(body.clone()),
        _ => return None,
    };
    Some(r)
}

/// Rebuilds features from index `start`, filling the cache.
fn rebuild(feats: &[Feature], start: usize, cache: &mut PartCache) {
    let mut body: Arc<Solid> = if start == 0 { Arc::new(Solid::default()) } else { cache.bodies[start - 1].clone() };
    for f in feats.iter().skip(start) {
        cache.errors.remove(&f.id);
        if f.suppressed {
            cache.bodies.push(body.clone());
            continue;
        }
        if let Some(r) = direct_body(&f.kind, &body) {
            match r {
                Ok(nb) => body = Arc::new(nb),
                Err(e) => {
                    cache.errors.insert(f.id, e);
                }
            }
            cache.bodies.push(body.clone());
            continue;
        }
        let sk = |id: u32| -> Result<Sketch, String> {
            match feats.iter().find(|x| x.id == id).map(|x| &x.kind) {
                Some(FeatureKind::Sketch(s)) => Ok(s.clone()),
                _ => Err("its sketch is missing".into()),
            }
        };
        let res: Result<(Solid, bool), String> = (|| match &f.kind {
            FeatureKind::Sketch(_) => Err(String::new()),
            FeatureKind::Pad { sketch, length, symmetric, reverse } => {
                let s = sk(*sketch)?;
                let l = if *reverse { -*length } else { *length };
                let (z0, z1) = if *symmetric { (-l.abs(), l.abs()) } else { (0.0, l) };
                if (z1 - z0).abs() < 1e-6 {
                    return Err("length must not be zero".into());
                }
                Ok((sketch_solid(&s, &|p| Ok(extrude(p, &s.frame, z0, z1)))?, true))
            }
            FeatureKind::Pocket { sketch, depth, through, reverse } => {
                let s = sk(*sketch)?;
                let d = if *through { through_length(&body) } else { *depth };
                if d.abs() < 1e-6 {
                    return Err("depth must not be zero".into());
                }
                // a pocket cuts against the sketch normal unless reversed; nudge the start
                // slightly outward so it cleanly opens the face it was drawn on
                let (z0, z1) = if *reverse { (-1e-3, d) } else { (1e-3, -d) };
                Ok((sketch_solid(&s, &|p| Ok(extrude(p, &s.frame, z0, z1)))?, false))
            }
            FeatureKind::Shaft { sketch, angle, axis_h } | FeatureKind::Groove { sketch, angle, axis_h } => {
                let s = sk(*sketch)?;
                // an Axis drawn in the sketch wins over the sketch's H / V axis
                let (ap, dir) = s.axis().unwrap_or((DVec2::ZERO, if *axis_h { DVec2::X } else { DVec2::Y }));
                let a = angle.to_radians();
                if a.abs() < 1e-6 {
                    return Err("angle must not be zero".into());
                }
                let solid = sketch_solid(&s, &|p| revolve(p, &s.frame, ap, dir, a.abs()))?;
                Ok((solid, matches!(f.kind, FeatureKind::Shaft { .. })))
            }
            FeatureKind::Hole { pos, dir, diameter, depth, through, htype, d2, depth2, angle2 } => {
                let d = if *through { through_length(&body) } else { *depth };
                if *diameter <= 0.0 || d <= 0.0 {
                    return Err("diameter and depth must be positive".into());
                }
                let dir = dir.normalize();
                let d2 = if *d2 > *diameter { *d2 } else { diameter * 1.8 };
                let depth2 = if *depth2 > 0.0 { *depth2 } else { diameter * 0.6 };
                let angle2 = if *angle2 > 0.0 { *angle2 } else { 90.0 };
                if *htype == 0 {
                    return Ok((cylinder_solid(*pos - dir * 1e-3, dir, diameter / 2.0, d + 1e-3), false));
                }
                Ok((hole_solid(*pos, dir, *diameter, d, *htype, d2, depth2, angle2), false))
            }
            FeatureKind::Loft { sections, remove } => {
                let mut loops = Vec::new();
                for id in sections {
                    let s = sk(*id)?;
                    let (solids, holes) = s.regions()?;
                    if solids.len() != 1 || !holes.is_empty() {
                        return Err("each section sketch must hold exactly one closed profile".into());
                    }
                    loops.push(solids[0].iter().map(|p| s.frame.to3(*p, 0.0)).collect::<Vec<_>>());
                }
                Ok((loft(&loops)?, !*remove))
            }
            FeatureKind::Split { base, plane, offset, flip } => {
                if body.is_empty() {
                    return Err("there is no solid to split yet".into());
                }
                let f = match base {
                    0 => Frame::xy(),
                    1 => Frame::yz(),
                    2 => Frame::zx(),
                    _ => feats.iter().find(|x| x.id == *plane).and_then(|x| ref_plane_frame_of(&x.kind)).ok_or("its splitting plane is missing")?,
                };
                let f = Frame { o: f.o + f.n() * *offset, ..f };
                // remove everything on one side: a big block beyond the plane
                let l = through_length(&body) + f.o.length() + 10.0;
                let sq = vec![DVec2::new(-l, -l), DVec2::new(l, -l), DVec2::new(l, l), DVec2::new(-l, l)];
                let (z0, z1) = if *flip { (-l, 0.0) } else { (0.0, l) };
                Ok((extrude(&sq, &f, z0, z1), false))
            }
            FeatureKind::Stiffener { sketch, thickness, depth, reverse } => {
                let s = sk(*sketch)?;
                if *thickness <= 0.0 || *depth <= 0.0 {
                    return Err("thickness and depth must be positive".into());
                }
                let mut tool = Solid::default();
                let (z0, z1) = if *reverse { (0.0, *depth) } else { (-*depth, 0.0) };
                let mut count = 0;
                for (i, sh) in s.shapes.iter().enumerate() {
                    if s.is_construction(i) || !sh.is_open() {
                        continue;
                    }
                    let c = sh.curve();
                    for w in c.windows(2) {
                        let d = w[1] - w[0];
                        if d.length() < 1e-9 {
                            continue;
                        }
                        let n = DVec2::new(-d.y, d.x).normalize() * (*thickness / 2.0);
                        let rect = vec![w[0] - n, w[1] - n, w[1] + n, w[0] + n];
                        tool = tool.union(&extrude(&rect, &s.frame, z0, z1));
                        count += 1;
                    }
                }
                if count == 0 {
                    return Err("the stiffener sketch needs open lines (draw the rib as a line)".into());
                }
                Ok((tool, true))
            }
            FeatureKind::Sweep { profile, path, remove } => {
                let ps = sk(*profile)?;
                let (solids, holes) = ps.regions()?;
                let (path2, closed) = sketch_path(&sk(*path)?)?;
                let path_s = sk(*path)?;
                let path3: Vec<DVec3> = path2.iter().map(|p| path_s.frame.to3(*p, 0.0)).collect();
                let mut tool = Solid::default();
                for l in &solids {
                    let loop3: Vec<DVec3> = l.iter().map(|p| ps.frame.to3(*p, 0.0)).collect();
                    tool = tool.union(&sweep(&loop3, &path3, closed)?);
                }
                for l in &holes {
                    let loop3: Vec<DVec3> = l.iter().map(|p| ps.frame.to3(*p, 0.0)).collect();
                    tool = tool.subtract(&sweep(&loop3, &path3, closed)?);
                }
                Ok((tool, !*remove))
            }
            FeatureKind::Primitive { prim, pos, a, b, c, axis, subtract } => {
                let s = match prim {
                    PrimKind::Box => {
                        let h = DVec3::new(a.abs(), b.abs(), c.abs()) / 2.0;
                        box_solid(*pos - h, *pos + h)
                    }
                    PrimKind::Cylinder => cylinder_solid(*pos, axis.v(), a.abs() / 2.0, b.abs()),
                    PrimKind::Sphere => sphere_solid(*pos, a.abs() / 2.0),
                };
                if s.is_empty() {
                    return Err("sizes must not be zero".into());
                }
                Ok((s, !*subtract))
            }
            FeatureKind::Mirror { plane } => {
                if body.is_empty() {
                    return Err("nothing to mirror yet".into());
                }
                let n = plane.v();
                let m = body.transformed(|p| p - n * (2.0 * p.dot(n)), true);
                Ok((m, true))
            }
            FeatureKind::RectPattern { feature, dir, count, step, dir2, count2, step2 } => {
                let (tool, add) = cache.tools.get(feature).cloned().ok_or("the patterned feature has no shape")?;
                let mut acc = Solid::default();
                let (n1, n2) = (count.round().max(1.0) as i32, count2.round().max(1.0) as i32);
                for i in 0..n1 {
                    for j in 0..n2 {
                        if i == 0 && j == 0 {
                            continue;
                        }
                        let d = dir.v() * (*step * i as f64) + dir2.v() * (*step2 * j as f64);
                        acc = acc.union(&tool.translated(d));
                    }
                }
                if acc.is_empty() {
                    return Err("the pattern needs at least 2 instances".into());
                }
                Ok((acc, add))
            }
            FeatureKind::CircPattern { feature, axis, count, total } => {
                let (tool, add) = cache.tools.get(feature).cloned().ok_or("the patterned feature has no shape")?;
                let n = count.round().max(1.0) as i32;
                let full = (total.abs() - 360.0).abs() < 1e-6;
                let step = if full { total.to_radians() / n as f64 } else { total.to_radians() / (n - 1).max(1) as f64 };
                let mut acc = Solid::default();
                for i in 1..n {
                    acc = acc.union(&tool.rotated(DVec3::ZERO, axis.v(), step * i as f64));
                }
                if acc.is_empty() {
                    return Err("the pattern needs at least 2 instances".into());
                }
                let _ = axis_dirs(*axis);
                Ok((acc, add))
            }
            FeatureKind::Import { tris, scale, .. } => {
                let polys: Vec<Poly> = tris
                    .iter()
                    .filter_map(|t| {
                        Poly::new(vec![
                            DVec3::new(t[0] as f64, t[1] as f64, t[2] as f64) * *scale,
                            DVec3::new(t[3] as f64, t[4] as f64, t[5] as f64) * *scale,
                            DVec3::new(t[6] as f64, t[7] as f64, t[8] as f64) * *scale,
                        ])
                    })
                    .collect();
                Ok((Solid::from_polys(polys).fix_orientation(), true))
            }
            // dress-ups, transformations and planes are handled by direct_body
            _ => Err(String::new()),
        })();
        match res {
            Ok((tool, add)) => {
                let tool = Arc::new(tool);
                cache.tools.insert(f.id, (tool.clone(), add));
                let nb = if add {
                    if body.is_empty() { (*tool).clone() } else { body.union(&tool) }
                } else if body.is_empty() {
                    cache.errors.insert(f.id, "there is no material to remove yet".into());
                    Solid::default()
                } else {
                    body.subtract(&tool)
                };
                body = Arc::new(nb);
            }
            Err(e) => {
                if !e.is_empty() {
                    cache.errors.insert(f.id, e);
                }
            }
        }
        cache.bodies.push(body.clone());
    }
    cache.mesh = Some(Arc::new(Mesh::from_solid(&body)));
}

// ---------------------------------------------------------------- assemblies

#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
#[serde(tag = "type")]
pub enum GeomRef {
    Plane { p: DVec3, n: DVec3 },
    Axis { p: DVec3, d: DVec3 },
}

impl GeomRef {
    pub fn transformed(&self, m: &DMat4) -> GeomRef {
        match self {
            GeomRef::Plane { p, n } => GeomRef::Plane { p: m.transform_point3(*p), n: m.transform_vector3(*n).normalize() },
            GeomRef::Axis { p, d } => GeomRef::Axis { p: m.transform_point3(*p), d: m.transform_vector3(*d).normalize() },
        }
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq, Eq)]
pub enum CKind {
    Contact,
    Offset,
    Coincidence,
    Angle,
}

impl CKind {
    pub fn name(self) -> &'static str {
        match self {
            CKind::Contact => "Contact",
            CKind::Offset => "Offset",
            CKind::Coincidence => "Coincidence",
            CKind::Angle => "Angle",
        }
    }
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Constraint {
    pub name: String,
    pub kind: CKind,
    pub a: u32,
    pub ra: GeomRef,
    pub b: u32,
    pub rb: GeomRef,
    pub value: f64,
    #[serde(default)]
    pub flip: bool,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Component {
    pub id: u32,
    pub name: String,
    pub part: Part,
    pub pos: DVec3,
    pub rot: DQuat,
    pub fixed: bool,
    #[serde(default)]
    pub hidden: bool,
}

impl Component {
    pub fn matrix(&self) -> DMat4 {
        DMat4::from_rotation_translation(self.rot, self.pos)
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Product {
    pub name: String,
    pub comps: Vec<Component>,
    pub constraints: Vec<Constraint>,
    pub next_id: u32,
    pub counters: HashMap<String, u32>,
}

/// Colours new assembly components get, in turn.
pub const PALETTE: [[f32; 3]; 8] = [
    [0.72, 0.74, 0.78],
    [0.86, 0.72, 0.50],
    [0.55, 0.70, 0.88],
    [0.60, 0.80, 0.58],
    [0.88, 0.60, 0.60],
    [0.78, 0.66, 0.88],
    [0.90, 0.85, 0.55],
    [0.55, 0.82, 0.82],
];

/// Ready-made colours offered in the appearance editor (the palette above, then strong
/// colours and common materials).
pub const SWATCHES: [[f32; 3]; 16] = [
    [0.72, 0.74, 0.78],
    [0.86, 0.72, 0.50],
    [0.55, 0.70, 0.88],
    [0.60, 0.80, 0.58],
    [0.88, 0.60, 0.60],
    [0.78, 0.66, 0.88],
    [0.90, 0.85, 0.55],
    [0.55, 0.82, 0.82],
    [0.95, 0.95, 0.93],
    [0.15, 0.15, 0.17],
    [0.80, 0.12, 0.10],
    [0.95, 0.45, 0.08],
    [0.98, 0.80, 0.10],
    [0.10, 0.55, 0.22],
    [0.10, 0.30, 0.75],
    [0.85, 0.62, 0.28],
];

impl Product {
    pub fn new(name: &str) -> Product {
        Product { name: name.into(), comps: vec![], constraints: vec![], next_id: 1, counters: HashMap::new() }
    }
    pub fn add_component(&mut self, mut part: Part, pos: DVec3) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let n = self.comps.iter().filter(|c| c.part.name == part.name).count() + 1;
        if part.color == DEFAULT_PART_COLOR && !self.comps.is_empty() {
            part.color = PALETTE[self.comps.len() % PALETTE.len()];
        }
        let name = format!("{} ({}.{})", part.name, part.name, n);
        self.comps.push(Component { id, name, part, pos, rot: DQuat::IDENTITY, fixed: false, hidden: false });
        id
    }
    pub fn comp(&self, id: u32) -> Option<&Component> {
        self.comps.iter().find(|c| c.id == id)
    }
    pub fn comp_mut(&mut self, id: u32) -> Option<&mut Component> {
        self.comps.iter_mut().find(|c| c.id == id)
    }
    pub fn next_name(&mut self, t: &str) -> String {
        let c = self.counters.entry(t.to_string()).or_insert(0);
        *c += 1;
        format!("{}.{}", t, c)
    }
    pub fn remove_component(&mut self, id: u32) {
        self.comps.retain(|c| c.id != id);
        self.constraints.retain(|c| c.a != id && c.b != id);
    }

    /// Moves components until the constraints hold (fixed components never move).
    pub fn solve(&mut self) -> Vec<String> {
        let mut problems = Vec::new();
        for _ in 0..30 {
            for ci in 0..self.constraints.len() {
                let c = self.constraints[ci].clone();
                let (fa, fb) = match (self.comp(c.a), self.comp(c.b)) {
                    (Some(a), Some(b)) => (a.fixed, b.fixed),
                    _ => continue,
                };
                let (mover, mref, other, oref) = if !fa { (c.a, c.ra, c.b, c.rb) } else if !fb { (c.b, c.rb, c.a, c.ra) } else { continue };
                let target = oref.transformed(&self.comp(other).unwrap().matrix());
                let comp = self.comp_mut(mover).unwrap();
                apply_constraint(comp, c.kind, mref, target, c.value, c.flip);
            }
        }
        for c in &self.constraints {
            if let (Some(a), Some(b)) = (self.comp(c.a), self.comp(c.b)) {
                if a.fixed && b.fixed {
                    continue;
                }
                let err = constraint_error(c, a, b);
                if err > 1e-3 {
                    problems.push(format!("{} cannot be fully satisfied", c.name));
                }
            }
        }
        problems
    }
}

fn shortest_arc(from: DVec3, to: DVec3, fallback_axis: DVec3) -> DQuat {
    let d = from.dot(to).clamp(-1.0, 1.0);
    if d < -0.999999 {
        let mut ax = fallback_axis - from * fallback_axis.dot(from);
        if ax.length() < 1e-6 {
            ax = from.any_orthonormal_vector();
        }
        return DQuat::from_axis_angle(ax.normalize(), PI);
    }
    DQuat::from_rotation_arc(from, to)
}

fn rotate_about(comp: &mut Component, q: DQuat, pivot: DVec3) {
    comp.pos = pivot + q * (comp.pos - pivot);
    comp.rot = (q * comp.rot).normalize();
}

fn apply_constraint(comp: &mut Component, kind: CKind, local: GeomRef, target: GeomRef, value: f64, flip: bool) {
    let world = local.transformed(&comp.matrix());
    match (kind, world, target) {
        (CKind::Contact | CKind::Offset, GeomRef::Plane { p, n }, GeomRef::Plane { p: tp, n: tn }) => {
            let want = if flip { tn } else { -tn };
            let q = shortest_arc(n, want, tn.any_orthonormal_vector());
            rotate_about(comp, q, p);
            let off = if kind == CKind::Offset { value } else { 0.0 };
            let dist = (p - tp).dot(tn);
            comp.pos += tn * (off - dist);
        }
        (CKind::Coincidence, GeomRef::Axis { p, d }, GeomRef::Axis { p: tp, d: td }) => {
            let want = if (d.dot(td) >= 0.0) != flip { td } else { -td };
            let q = shortest_arc(d, want, td.any_orthonormal_vector());
            rotate_about(comp, q, p);
            let w = p - tp;
            let perp = w - td * w.dot(td);
            comp.pos -= perp;
        }
        (CKind::Angle, GeomRef::Plane { p, n }, GeomRef::Plane { n: tn, .. }) => {
            // rotate n about the axis perpendicular to both so that angle(n, tn) = value
            let cur = n.dot(tn).clamp(-1.0, 1.0).acos();
            let mut ax = n.cross(tn);
            if ax.length() < 1e-9 {
                ax = tn.any_orthonormal_vector();
            }
            let delta = cur - value.to_radians();
            rotate_about(comp, DQuat::from_axis_angle(ax.normalize(), delta), p);
        }
        _ => {}
    }
}

pub fn constraint_error(c: &Constraint, a: &Component, b: &Component) -> f64 {
    let wa = c.ra.transformed(&a.matrix());
    let wb = c.rb.transformed(&b.matrix());
    match (c.kind, wa, wb) {
        (CKind::Contact | CKind::Offset, GeomRef::Plane { p, n }, GeomRef::Plane { p: tp, n: tn }) => {
            let off = if c.kind == CKind::Offset { c.value } else { 0.0 };
            let want = if c.flip { 1.0 } else { -1.0 };
            (n.dot(tn) - want).abs() + ((p - tp).dot(tn) - off).abs()
        }
        (CKind::Coincidence, GeomRef::Axis { p, d }, GeomRef::Axis { p: tp, d: td }) => {
            let w = p - tp;
            (1.0 - d.dot(td).abs()) + (w - td * w.dot(td)).length()
        }
        (CKind::Angle, GeomRef::Plane { n, .. }, GeomRef::Plane { n: tn, .. }) => (n.dot(tn).clamp(-1.0, 1.0).acos().to_degrees() - c.value).abs() * 0.01,
        _ => 0.0,
    }
}

// ---------------------------------------------------------------- documents

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "document")]
pub enum Doc {
    Part(Part),
    Product(Product),
}

pub fn save_doc(doc: &Doc, path: &str) -> Result<(), String> {
    let s = serde_json::to_string(doc).map_err(|e| e.to_string())?;
    std::fs::write(path, s).map_err(|e| e.to_string())
}

pub fn load_doc(path: &str) -> Result<Doc, String> {
    let s = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut d: Doc = serde_json::from_str(&s).map_err(|e| format!("not a BetterCAD file ({})", e))?;
    match &mut d {
        Doc::Part(p) => p.touch(0),
        Doc::Product(pr) => {
            for c in &mut pr.comps {
                c.part.touch(0);
            }
        }
    }
    Ok(d)
}

pub fn tris_to_f32(t: &[[DVec3; 3]]) -> Vec<[f32; 9]> {
    t.iter()
        .map(|t| [t[0].x as f32, t[0].y as f32, t[0].z as f32, t[1].x as f32, t[1].y as f32, t[1].z as f32, t[2].x as f32, t[2].y as f32, t[2].z as f32])
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pad_pocket() {
        let mut p = Part::new("P");
        let s = p.add_feature(FeatureKind::Sketch(Sketch {
            frame: Frame::xy(),
            support: "xy".into(),
            shapes: vec![Shape::Rect { cx: 0.0, cy: 0.0, w: 40.0, h: 30.0, r: 0.0 }],
            construction: vec![],
            dims: vec![],
        }));
        p.add_feature(FeatureKind::Pad { sketch: s, length: 20.0, symmetric: false, reverse: false });
        let m = p.mesh();
        assert!((m.volume() - 24000.0).abs() < 1e-2, "{}", m.volume());
        let s2 = p.add_feature(FeatureKind::Sketch(Sketch {
            frame: Frame { o: DVec3::new(0.0, 0.0, 20.0), u: DVec3::X, v: DVec3::Y },
            support: "face".into(),
            shapes: vec![Shape::Circle { cx: 0.0, cy: 0.0, r: 8.0 }],
            construction: vec![],
            dims: vec![],
        }));
        p.add_feature(FeatureKind::Pocket { sketch: s2, depth: 10.0, through: false, reverse: false });
        let m = p.mesh();
        let expect = 24000.0 - std::f64::consts::PI * 64.0 * 10.0;
        assert!((m.volume() - expect).abs() < 30.0, "{}", m.volume());
        p.add_feature(FeatureKind::RectPattern { feature: 4, dir: Axis3::X, count: 2.0, step: 12.0, dir2: Axis3::Y, count2: 1.0, step2: 10.0 });
        let m2 = p.mesh();
        assert!(m2.volume() < m.volume() - 1000.0);
    }

    #[test]
    fn contact_constraint() {
        let mut pr = Product::new("A");
        let mut part = Part::new("B");
        part.add_feature(FeatureKind::Primitive { prim: PrimKind::Box, pos: DVec3::ZERO, a: 10.0, b: 10.0, c: 10.0, axis: Axis3::Z, subtract: false });
        let a = pr.add_component(part.clone(), DVec3::ZERO);
        let b = pr.add_component(part, DVec3::new(30.0, 5.0, 3.0));
        pr.comp_mut(a).unwrap().fixed = true;
        pr.constraints.push(Constraint {
            name: "Contact.1".into(),
            kind: CKind::Contact,
            a: b,
            ra: GeomRef::Plane { p: DVec3::new(0.0, 0.0, -5.0), n: -DVec3::Z },
            b: a,
            rb: GeomRef::Plane { p: DVec3::new(0.0, 0.0, 5.0), n: DVec3::Z },
            value: 0.0,
            flip: false,
        });
        let probs = pr.solve();
        assert!(probs.is_empty());
        let c = pr.comp(b).unwrap();
        assert!((c.pos.z - 10.0).abs() < 1e-9, "{:?}", c.pos);
    }

    #[test]
    fn more_features_and_save() {
        let t = std::time::Instant::now();
        let mut p = Part::new("P");
        let s = p.add_feature(FeatureKind::Sketch(Sketch {
            frame: Frame::zx(),
            support: "zx".into(),
            shapes: vec![Shape::Rect { cx: 10.0, cy: 15.0, w: 10.0, h: 30.0, r: 0.0 }],
            construction: vec![],
            dims: vec![],
        }));
        let shaft = p.add_feature(FeatureKind::Shaft { sketch: s, angle: 360.0, axis_h: false });
        let m = p.mesh();
        assert!(p.cache.errors.is_empty(), "{:?}", p.cache.errors);
        // ring of inner radius 5, outer 15, height 30 along the zx sketch V axis (= X)
        let ideal = std::f64::consts::PI * (225.0 - 25.0) * 30.0;
        assert!((m.volume() - ideal).abs() / ideal < 0.02, "shaft {} vs {}", m.volume(), ideal);
        let _ = shaft;
        let h = p.add_feature(FeatureKind::Hole { pos: DVec3::new(15.0, 0.0, 15.0), dir: -DVec3::Z, diameter: 4.0, depth: 10.0, through: false, htype: 0, d2: 0.0, depth2: 0.0, angle2: 0.0 });
        p.add_feature(FeatureKind::CircPattern { feature: h, axis: Axis3::X, count: 8.0, total: 360.0 });
        p.add_feature(FeatureKind::Mirror { plane: Axis3::X });
        let m2 = p.mesh();
        assert!(p.cache.errors.is_empty(), "{:?}", p.cache.errors);
        assert!(m2.volume() > m.volume(), "mirror should add a copy");
        // box with a 6 x 6 grid of holes
        let mut q = Part::new("Q");
        q.add_feature(FeatureKind::Primitive { prim: PrimKind::Box, pos: DVec3::new(0.0, 0.0, 5.0), a: 100.0, b: 100.0, c: 10.0, axis: Axis3::Z, subtract: false });
        let hh = q.add_feature(FeatureKind::Hole { pos: DVec3::new(-40.0, -40.0, 10.0), dir: -DVec3::Z, diameter: 6.0, depth: 0.0, through: true, htype: 0, d2: 0.0, depth2: 0.0, angle2: 0.0 });
        q.add_feature(FeatureKind::RectPattern { feature: hh, dir: Axis3::X, count: 6.0, step: 16.0, dir2: Axis3::Y, count2: 6.0, step2: 16.0 });
        let mq = q.mesh();
        assert!(q.cache.errors.is_empty(), "{:?}", q.cache.errors);
        assert!(mq.volume() < 100000.0 - 30.0 * 9.0 * std::f64::consts::PI * 10.0);
        // save and reload
        let doc = Doc::Part(q.clone());
        let path = std::env::temp_dir().join("bc_test.bcpart");
        save_doc(&doc, path.to_str().unwrap()).unwrap();
        match load_doc(path.to_str().unwrap()).unwrap() {
            Doc::Part(mut r) => assert!((r.mesh().volume() - mq.volume()).abs() < 1e-3),
            _ => panic!(),
        }
        // watertight: every edge shared by exactly two triangles
        let mut edges = std::collections::HashMap::new();
        for t in &mq.tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        let bad = edges.values().filter(|c| **c != 2).count();
        println!("holes plate: {} tris, {} non-manifold edges, {:?}", mq.tris.len(), bad, t.elapsed());
        assert_eq!(bad, 0);
    }

    #[test]
    fn appearance_saved_and_old_files_load() {
        let mut p = Part::new("A");
        p.color = [0.1, 0.2, 0.3];
        p.finish = Finish::Wood;
        let json = serde_json::to_string(&Doc::Part(p)).unwrap();
        match serde_json::from_str::<Doc>(&json).unwrap() {
            Doc::Part(q) => assert!(q.finish == Finish::Wood && q.color == [0.1, 0.2, 0.3]),
            _ => panic!(),
        }
        // files saved before finishes existed open as plastic
        let old = json.replace(",\"finish\":\"Wood\"", "");
        assert!(!old.contains("finish"), "{}", old);
        match serde_json::from_str::<Doc>(&old).unwrap() {
            Doc::Part(q) => assert_eq!(q.finish, Finish::Plastic),
            _ => panic!(),
        }
    }

    #[test]
    fn split_and_stiffener() {
        let mut p = Part::new("S");
        p.add_feature(FeatureKind::Primitive { prim: PrimKind::Box, pos: DVec3::new(0.0, 0.0, 10.0), a: 20.0, b: 20.0, c: 20.0, axis: Axis3::Z, subtract: false });
        let split = p.add_feature(FeatureKind::Split { base: 0, plane: 0, offset: 5.0, flip: false });
        let v = p.mesh().volume();
        assert!((v - 2000.0).abs() < 1e-6, "{}", v);
        p.feature_mut(split).unwrap().kind = FeatureKind::Split { base: 0, plane: 0, offset: 5.0, flip: true };
        p.touch_id(split);
        assert!((p.mesh().volume() - 6000.0).abs() < 1e-6);
        // a 2 mm stiffener line on top, reaching 3 mm up from z = 20 (reversed: along +Z)
        let s = p.add_feature(FeatureKind::Sketch(Sketch { frame: Frame { o: DVec3::new(0.0, 0.0, 20.0), u: DVec3::X, v: DVec3::Y }, support: "face".into(), shapes: vec![Shape::Line { x1: -10.0, y1: 0.0, x2: 10.0, y2: 0.0 }], construction: vec![], dims: vec![] }));
        p.add_feature(FeatureKind::Stiffener { sketch: s, thickness: 2.0, depth: 3.0, reverse: true });
        assert!((p.mesh().volume() - 6120.0).abs() < 1e-6, "{}", p.mesh().volume());
        assert!(p.cache.errors.is_empty());
    }
}
