//! Sketch dimensional constraints (classic CAD "Constraint" tool): lengths, widths, heights,
//! radii, diameters, distances and angles that stay attached to the geometry and drive it.
//! Editing a value moves the geometry; a small iterative solver keeps all dimensions true.

use crate::sketch::*;
use glam::DVec2;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// What a dimension is attached to.
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub enum GRef {
    /// a whole element (line, circle, arc, rectangle...)
    Elem(usize),
    /// one of an element's key points (ends, middle, centre, corners), by index
    Key(usize, usize),
    AxisH,
    AxisV,
    Origin,
}

impl GRef {
    pub fn elem(&self) -> Option<usize> {
        match self {
            GRef::Elem(i) | GRef::Key(i, _) => Some(*i),
            _ => None,
        }
    }
    pub fn fixed(&self) -> bool {
        matches!(self, GRef::AxisH | GRef::AxisV | GRef::Origin)
    }
    fn shifted_after_remove(&self, removed: usize) -> Option<GRef> {
        match *self {
            GRef::Elem(i) if i == removed => None,
            GRef::Key(i, _) if i == removed => None,
            GRef::Elem(i) => Some(GRef::Elem(if i > removed { i - 1 } else { i })),
            GRef::Key(i, k) => Some(GRef::Key(if i > removed { i - 1 } else { i }, k)),
            other => Some(other),
        }
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq, Eq)]
pub enum DimKind {
    Length,
    Width,
    Height,
    Radius,
    Diameter,
    Distance,
    Angle,
    /// geometric constraints: a line kept horizontal / vertical
    Horizontal,
    Vertical,
}

impl DimKind {
    pub fn name(self) -> &'static str {
        match self {
            DimKind::Length => "Length",
            DimKind::Width => "Width",
            DimKind::Height => "Height",
            DimKind::Radius => "Radius",
            DimKind::Diameter => "Diameter",
            DimKind::Distance => "Distance",
            DimKind::Angle => "Angle",
            DimKind::Horizontal => "Horizontal",
            DimKind::Vertical => "Vertical",
        }
    }
    /// Horizontal / vertical are shown as a small tag and have no value to edit.
    pub fn is_geometric(self) -> bool {
        matches!(self, DimKind::Horizontal | DimKind::Vertical)
    }
}

#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
pub struct Dim {
    pub kind: DimKind,
    pub a: GRef,
    pub b: Option<GRef>,
    pub value: f64,
    /// where the value text sits, relative to the dimension's anchor
    pub offset: DVec2,
    /// a reference dimension only shows the value and never drives the geometry
    #[serde(default)]
    pub reference: bool,
}

impl Dim {
    pub fn label(&self) -> String {
        let v = fmt((self.value * 1000.0).round() / 1000.0);
        let t = match self.kind {
            DimKind::Radius => format!("R{}", v),
            DimKind::Diameter => format!("D{}", v),
            DimKind::Angle => format!("{} deg", v),
            DimKind::Horizontal => "H".into(),
            DimKind::Vertical => "V".into(),
            _ => v,
        };
        if self.reference { format!("({})", t) } else { t }
    }
}

/// A geometric reading of a reference: a point, or an infinite line.
#[derive(Clone, Copy, Debug)]
pub enum Form {
    Pt(DVec2),
    Ln(DVec2, DVec2),
}

/// How a dimension is drawn.
pub enum DimGeom {
    /// extension points p, q; dimension line offset `o` along unit normal `n`; text position
    Linear { p: DVec2, q: DVec2, n: DVec2, o: f64, text: DVec2 },
    /// centre, radius, direction to the text, diameter or radius style
    Radial { c: DVec2, r: f64, dir: DVec2, diameter: bool, text: DVec2 },
    /// vertex, the two directions, arc radius
    Angular { v: DVec2, d1: DVec2, d2: DVec2, r: f64, text: DVec2 },
    /// a small tag (H / V) next to a line
    Tag { at: DVec2 },
}

fn perp(d: DVec2) -> DVec2 {
    DVec2::new(-d.y, d.x)
}

fn rotate(p: DVec2, c: DVec2, ang: f64) -> DVec2 {
    let (s, co) = ang.sin_cos();
    let d = p - c;
    c + DVec2::new(d.x * co - d.y * s, d.x * s + d.y * co)
}

impl Sketch {
    /// Keeps dimension references valid after shape `i` was removed.
    pub fn dims_after_remove(&mut self, i: usize) {
        let dims = std::mem::take(&mut self.dims);
        for mut d in dims {
            let a = match d.a.shifted_after_remove(i) {
                Some(a) => a,
                None => continue,
            };
            let b = match d.b.map(|b| b.shifted_after_remove(i)) {
                Some(None) => continue,
                Some(Some(b)) => Some(b),
                None => None,
            };
            d.a = a;
            d.b = b;
            self.dims.push(d);
        }
    }

    pub fn form(&self, r: &GRef) -> Option<Form> {
        match *r {
            GRef::AxisH => Some(Form::Ln(DVec2::ZERO, DVec2::X)),
            GRef::AxisV => Some(Form::Ln(DVec2::ZERO, DVec2::Y)),
            GRef::Origin => Some(Form::Pt(DVec2::ZERO)),
            GRef::Key(i, k) => self.shapes.get(i).and_then(|s| s.key_points().get(k).copied()).map(Form::Pt),
            GRef::Elem(i) => match self.shapes.get(i)? {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                    let d = DVec2::new(x2 - x1, y2 - y1);
                    if d.length() < 1e-12 {
                        Some(Form::Pt(DVec2::new(*x1, *y1)))
                    } else {
                        Some(Form::Ln(DVec2::new(*x1, *y1), d.normalize()))
                    }
                }
                Shape::Circle { cx, cy, .. } | Shape::Arc { cx, cy, .. } | Shape::Ellipse { cx, cy, .. } | Shape::Polygon { cx, cy, .. } | Shape::Rect { cx, cy, .. } => Some(Form::Pt(DVec2::new(*cx, *cy))),
                Shape::Slot { x1, y1, x2, y2, .. } => Some(Form::Pt(DVec2::new(x1 + x2, y1 + y2) * 0.5)),
                _ => None,
            },
        }
    }

    /// The segment of a straight reference (for anchoring and angle pivots).
    fn segment(&self, r: &GRef) -> Option<(DVec2, DVec2)> {
        match *r {
            GRef::Elem(i) => match self.shapes.get(i)? {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => Some((DVec2::new(*x1, *y1), DVec2::new(*x2, *y2))),
                _ => None,
            },
            _ => None,
        }
    }

    /// Pivot and directions (pointing away from the pivot) of two straight references.
    fn angle_frame(&self, a: &GRef, b: &GRef) -> Option<(DVec2, DVec2, DVec2)> {
        let (Some(Form::Ln(oa, da)), Some(Form::Ln(ob, db))) = (self.form(a), self.form(b)) else { return None };
        let den = da.perp_dot(db);
        if den.abs() < 1e-9 {
            return None;
        }
        // shared end point, else the crossing of the two lines
        let mut pivot = None;
        if let (Some((a0, a1)), Some((b0, b1))) = (self.segment(a), self.segment(b)) {
            for p in [a0, a1] {
                for q in [b0, b1] {
                    if (p - q).length() < JOIN_TOL {
                        pivot = Some(p);
                    }
                }
            }
        }
        let pivot = pivot.unwrap_or_else(|| oa + da * ((ob - oa).perp_dot(db) / den));
        let away = |r: &GRef, d: DVec2| -> DVec2 {
            match self.segment(r) {
                Some((p0, p1)) => {
                    let far = if (p0 - pivot).length() > (p1 - pivot).length() { p0 } else { p1 };
                    let v = far - pivot;
                    if v.length() < 1e-12 { d } else { v.normalize() }
                }
                None => d,
            }
        };
        Some((pivot, away(a, da), away(b, db)))
    }

    /// The current value of a dimension (None when it no longer makes sense).
    pub fn dim_measure(&self, d: &Dim) -> Option<f64> {
        let shape = d.a.elem().and_then(|i| self.shapes.get(i));
        match d.kind {
            DimKind::Length => match shape? {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => Some(DVec2::new(x2 - x1, y2 - y1).length()),
                Shape::Slot { x1, y1, x2, y2, .. } => Some(DVec2::new(x2 - x1, y2 - y1).length()),
                _ => None,
            },
            DimKind::Width => match shape? {
                Shape::Rect { w, .. } => Some(w.abs()),
                _ => None,
            },
            DimKind::Height => match shape? {
                Shape::Rect { h, .. } => Some(h.abs()),
                _ => None,
            },
            DimKind::Radius => match shape? {
                Shape::Circle { r, .. } | Shape::Arc { r, .. } | Shape::Polygon { r, .. } | Shape::Slot { r, .. } => Some(r.abs()),
                _ => None,
            },
            DimKind::Diameter => match shape? {
                Shape::Circle { r, .. } | Shape::Arc { r, .. } => Some(r.abs() * 2.0),
                _ => None,
            },
            DimKind::Distance => {
                let (fa, fb) = (self.form(&d.a)?, self.form(d.b.as_ref()?)?);
                match (fa, fb) {
                    (Form::Pt(p), Form::Pt(q)) => Some((q - p).length()),
                    (Form::Pt(p), Form::Ln(o, dir)) | (Form::Ln(o, dir), Form::Pt(p)) => Some(dir.perp_dot(p - o).abs()),
                    (Form::Ln(oa, da), Form::Ln(ob, db)) => {
                        if da.perp_dot(db).abs() > 1e-6 {
                            None
                        } else {
                            Some(da.perp_dot(ob - oa).abs())
                        }
                    }
                }
            }
            DimKind::Angle => {
                let (_, d1, d2) = self.angle_frame(&d.a, d.b.as_ref()?)?;
                Some(d1.dot(d2).clamp(-1.0, 1.0).acos().to_degrees())
            }
            DimKind::Horizontal | DimKind::Vertical => match shape? {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => Some(if d.kind == DimKind::Horizontal { (y2 - y1).abs() } else { (x2 - x1).abs() }),
                _ => None,
            },
        }
    }

    pub fn dim_ok(&self, d: &Dim) -> bool {
        d.reference || self.dim_measure(d).map_or(false, |v| (v - d.value).abs() < 1e-6)
    }

    // ------------------------------------------------------------ moving geometry

    /// Moves the end point `p` of connected elements by `delta`. Lines standing square to the
    /// movement slide along with it (so rectangles stay rectangles); others stretch.
    fn push_point(&mut self, p: DVec2, delta: DVec2, visited: &mut HashSet<usize>) {
        if delta.length() < 1e-15 {
            return;
        }
        let dn = delta.normalize();
        for f in 0..self.shapes.len() {
            if visited.contains(&f) || self.is_ext(f) {
                continue;
            }
            let ends = match self.shapes[f].endpoints() {
                Some(e) => e,
                None => continue,
            };
            let at_start = (ends.0 - p).length() < JOIN_TOL;
            let at_end = (ends.1 - p).length() < JOIN_TOL;
            if !at_start && !at_end {
                continue;
            }
            let other = if at_start { ends.1 } else { ends.0 };
            match self.shapes[f].clone() {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                    let dir = DVec2::new(x2 - x1, y2 - y1).normalize_or_zero();
                    if dir.dot(dn).abs() < 1e-6 {
                        visited.insert(f);
                        self.shapes[f].translate(delta);
                        self.push_point(other, delta, visited);
                    } else {
                        self.shapes[f].set_handle(if at_start { 0 } else { 1 }, p + delta);
                    }
                }
                Shape::Arc { .. } => {
                    visited.insert(f);
                    self.shapes[f].translate(delta);
                    self.push_point(other, delta, visited);
                }
                Shape::Spline { pts, .. } => {
                    let k = if at_start { 0 } else { pts.len() - 1 };
                    self.shapes[f].set_handle(k, p + delta);
                }
                _ => {}
            }
        }
    }

    /// Elements whose position the dimensions fully decide (shown green, like a fully
    /// constrained element), and the points they pin (plus the origin). Dimension `skip` is
    /// left out of the count.
    pub fn fixed_state(&self, skip: Option<usize>) -> (HashSet<usize>, Vec<DVec2>) {
        let n = self.shapes.len();
        // reference elements from other sketches are locked
        let mut fixed: HashSet<usize> = (0..n).filter(|i| self.is_ext(*i)).collect();
        let mut orient: HashSet<usize> = HashSet::new();
        loop {
            let mut pts: Vec<DVec2> = vec![DVec2::ZERO];
            for &i in &fixed {
                pts.extend(self.shapes[i].key_points());
                if let Some((a, b)) = self.shapes[i].endpoints() {
                    pts.push(a);
                    pts.push(b);
                }
            }
            let fixed_pt = |p: DVec2| pts.iter().any(|q| (*q - p).length() < JOIN_TOL);
            let ref_fixed = |r: &GRef| -> bool {
                match *r {
                    GRef::AxisH | GRef::AxisV | GRef::Origin => true,
                    GRef::Elem(i) => fixed.contains(&i),
                    GRef::Key(i, k) => fixed.contains(&i) || self.shapes.get(i).and_then(|s| s.key_points().get(k).copied()).map_or(false, fixed_pt),
                }
            };
            let ref_orient = |r: &GRef| -> bool {
                match *r {
                    GRef::AxisH | GRef::AxisV => true,
                    GRef::Elem(i) => orient.contains(&i) || fixed.contains(&i),
                    _ => false,
                }
            };
            let live = |j: usize, d: &Dim| Some(j) != skip && !d.reference;
            let mut changed = false;
            // orientation known: H / V, or an angle to something whose orientation is known
            let mut new_orient = Vec::new();
            for (j, d) in self.dims.iter().enumerate() {
                if !live(j, d) {
                    continue;
                }
                match d.kind {
                    DimKind::Horizontal | DimKind::Vertical => new_orient.extend(d.a.elem()),
                    DimKind::Angle => {
                        if let Some(b) = d.b {
                            if ref_orient(&b) {
                                new_orient.extend(d.a.elem());
                            }
                            if ref_orient(&d.a) {
                                new_orient.extend(b.elem());
                            }
                        }
                    }
                    _ => {}
                }
            }
            let mut new_fixed = Vec::new();
            for i in 0..n {
                if fixed.contains(&i) {
                    continue;
                }
                let sh = &self.shapes[i];
                let need = match sh {
                    Shape::Line { .. } | Shape::Axis { .. } | Shape::Rect { .. } => 4,
                    Shape::Circle { .. } | Shape::Polygon { .. } => 3,
                    Shape::Arc { .. } | Shape::Slot { .. } => 5,
                    _ => continue,
                };
                let mut got = 0;
                if matches!(sh, Shape::Line { .. } | Shape::Axis { .. }) && (orient.contains(&i) || new_orient.contains(&i)) {
                    got += 1;
                }
                for (j, d) in self.dims.iter().enumerate() {
                    if !live(j, d) {
                        continue;
                    }
                    match d.kind {
                        DimKind::Length | DimKind::Width | DimKind::Height | DimKind::Radius | DimKind::Diameter => {
                            if d.a == GRef::Elem(i) {
                                got += 1;
                            }
                        }
                        DimKind::Distance => {
                            if let Some(b) = d.b {
                                let (ea, eb) = (d.a.elem(), b.elem());
                                if ea == Some(i) && eb != Some(i) && ref_fixed(&b) || eb == Some(i) && ea != Some(i) && ref_fixed(&d.a) {
                                    got += 1;
                                }
                            }
                        }
                        _ => {}
                    }
                }
                // end points / centre sitting on pinned points
                if let Some((a, b)) = sh.endpoints() {
                    got += 2 * (fixed_pt(a) as i32 + fixed_pt(b) as i32);
                }
                if let Shape::Circle { cx, cy, .. } | Shape::Arc { cx, cy, .. } | Shape::Rect { cx, cy, .. } | Shape::Polygon { cx, cy, .. } = sh {
                    if fixed_pt(DVec2::new(*cx, *cy)) {
                        got += 2;
                    }
                }
                if got >= need {
                    new_fixed.push(i);
                }
            }
            for o in new_orient {
                changed |= orient.insert(o);
            }
            for f in new_fixed {
                changed |= fixed.insert(f);
            }
            if !changed {
                return (fixed, pts);
            }
        }
    }

    /// Elements a dimension (being applied) must not move: fully fixed ones and ones tied to
    /// fixed things by other dimensions.
    fn held_for(&self, di: usize) -> HashSet<usize> {
        let mut h = self.fixed_state(Some(di)).0;
        h.extend(self.anchored_elems(di));
        h
    }

    fn point_pinned(&self, di: usize, p: DVec2) -> bool {
        let (_, pts) = self.fixed_state(Some(di));
        pts.iter().any(|q| (*q - p).length() < JOIN_TOL)
    }

    /// Moves a whole element, taking connected elements along.
    fn shift_elem(&mut self, i: usize, delta: DVec2, visited: &mut HashSet<usize>) {
        let ends = self.shapes[i].endpoints();
        visited.insert(i);
        self.shapes[i].translate(delta);
        if let Some((a, b)) = ends {
            self.push_point(a, delta, visited);
            self.push_point(b, delta, visited);
        }
    }

    /// Moves what a reference points at: a line's end point stretches the line, anything else
    /// moves its whole element.
    fn shift_ref(&mut self, r: &GRef, delta: DVec2, held: &HashSet<usize>) {
        // held elements are never pushed along
        let mut visited: HashSet<usize> = held.iter().cloned().filter(|h| Some(*h) != r.elem()).collect();
        match *r {
            GRef::Key(i, k) if k < 2 && matches!(self.shapes.get(i), Some(Shape::Line { .. }) | Some(Shape::Axis { .. })) => {
                let old = self.shapes[i].handle_points()[k];
                self.shapes[i].set_handle(k, old + delta);
                visited.insert(i);
                self.push_point(old, delta, &mut visited);
            }
            GRef::Key(i, _) | GRef::Elem(i) => {
                if i < self.shapes.len() {
                    self.shift_elem(i, delta, &mut visited);
                }
            }
            _ => {}
        }
    }

    /// Changes the geometry so that dimension `di` holds its value.
    pub fn dim_apply(&mut self, di: usize) {
        let d = match self.dims.get(di) {
            Some(d) if !d.reference => d.clone(),
            _ => return,
        };
        let cur = match self.dim_measure(&d) {
            Some(c) => c,
            None => return,
        };
        let v = d.value;
        if (cur - v).abs() < 1e-9 {
            return;
        }
        let i = d.a.elem().unwrap_or(usize::MAX);
        // a single-element dimension on a locked reference element cannot drive it
        if d.b.is_none() && self.is_ext(i) {
            return;
        }
        match d.kind {
            DimKind::Length => match self.shapes[i].clone() {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                    let a = DVec2::new(x1, y1);
                    let b = DVec2::new(x2, y2);
                    let dir = (b - a).normalize_or_zero();
                    let mut visited = self.held_for(di);
                    visited.insert(i);
                    // stretch at the end that is free (the second one unless it is pinned)
                    let (k, p, delta) = if self.point_pinned(di, b) && !self.point_pinned(di, a) { (0, a, -dir * (v - cur)) } else { (1, b, dir * (v - cur)) };
                    self.shapes[i].set_handle(k, p + delta);
                    self.push_point(p, delta, &mut visited);
                }
                Shape::Slot { x1, y1, x2, y2, .. } => {
                    let dir = DVec2::new(x2 - x1, y2 - y1).normalize_or_zero();
                    if let Shape::Slot { x2, y2, .. } = &mut self.shapes[i] {
                        *x2 += dir.x * (v - cur);
                        *y2 += dir.y * (v - cur);
                    }
                }
                _ => {}
            },
            DimKind::Width | DimKind::Height => {
                if let Shape::Rect { w, h, .. } = &mut self.shapes[i] {
                    if d.kind == DimKind::Width {
                        *w = v.max(1e-6);
                    } else {
                        *h = v.max(1e-6);
                    }
                }
            }
            DimKind::Radius | DimKind::Diameter => {
                let r = if d.kind == DimKind::Diameter { v / 2.0 } else { v };
                let old_ends = self.shapes[i].endpoints();
                match &mut self.shapes[i] {
                    Shape::Circle { r: rr, .. } | Shape::Arc { r: rr, .. } | Shape::Polygon { r: rr, .. } | Shape::Slot { r: rr, .. } => *rr = r.max(1e-6),
                    _ => {}
                }
                // ends of an arc moved: connected lines follow
                if let (Some((a0, b0)), Some((a1, b1))) = (old_ends, self.shapes[i].endpoints()) {
                    let mut visited = self.held_for(di);
                    visited.insert(i);
                    self.push_point(a0, a1 - a0, &mut visited);
                    self.push_point(b0, b1 - b0, &mut visited);
                }
            }
            DimKind::Distance => {
                let b = match d.b {
                    Some(b) => b,
                    None => return,
                };
                // the second item moves, unless it is fixed (an axis or the origin)
                let (mover, other) = match self.pick_mover(di, d.a, b) {
                    Some(x) => x,
                    None => return,
                };
                let (fm, fo) = match (self.form(&mover), self.form(&other)) {
                    (Some(x), Some(y)) => (x, y),
                    _ => return,
                };
                let delta = match (fm, fo) {
                    (Form::Pt(pm), Form::Pt(po)) => {
                        let dir = if (pm - po).length() > 1e-12 { (pm - po).normalize() } else { DVec2::X };
                        dir * (v - cur)
                    }
                    (Form::Pt(pm), Form::Ln(o, dir)) => {
                        let n = perp(dir);
                        let side = if n.dot(pm - o) >= 0.0 { 1.0 } else { -1.0 };
                        n * side * (v - cur)
                    }
                    (Form::Ln(o, dir), Form::Pt(po)) => {
                        let n = perp(dir);
                        let side = if n.dot(po - o) >= 0.0 { 1.0 } else { -1.0 };
                        -n * side * (v - cur)
                    }
                    (Form::Ln(om, _), Form::Ln(oo, dir)) => {
                        let n = perp(dir);
                        let side = if n.dot(om - oo) >= 0.0 { 1.0 } else { -1.0 };
                        n * side * (v - cur)
                    }
                };
                self.shift_ref(&mover, delta, &self.held_for(di));
            }
            DimKind::Horizontal | DimKind::Vertical => {
                let hs = self.shapes[i].handle_points();
                if hs.len() != 2 {
                    return;
                }
                let (a, b) = (hs[0], hs[1]);
                // move the end that is not pinned so the line lies flat / upright
                let (k, p, target) = if self.point_pinned(di, b) && !self.point_pinned(di, a) { (0, a, b) } else { (1, b, a) };
                let np = if d.kind == DimKind::Horizontal { DVec2::new(p.x, target.y) } else { DVec2::new(target.x, p.y) };
                let mut visited = self.held_for(di);
                visited.insert(i);
                self.shapes[i].set_handle(k, np);
                self.push_point(p, np - p, &mut visited);
            }
            DimKind::Angle => {
                let b = match d.b {
                    Some(b) => b,
                    None => return,
                };
                let (mover, other) = match self.pick_mover(di, d.a, b) {
                    Some(x) => x,
                    None => return,
                };
                let (pivot, d_other, d_mover) = match self.angle_frame(&other, &mover) {
                    Some(f) => f,
                    None => return,
                };
                let s = if d_other.perp_dot(d_mover) >= 0.0 { 1.0 } else { -1.0 };
                let phi = (v - cur).to_radians() * s;
                let mi = match mover.elem() {
                    Some(m) => m,
                    None => return,
                };
                let hs = self.shapes[mi].handle_points();
                if hs.len() != 2 {
                    return;
                }
                let mut visited = self.held_for(di);
                visited.insert(mi);
                for (k, h) in hs.iter().enumerate() {
                    let np = rotate(*h, pivot, phi);
                    if (np - *h).length() > 1e-12 {
                        self.shapes[mi].set_handle(k, np);
                        self.push_point(*h, np - *h, &mut visited);
                    }
                }
            }
        }
    }

    /// Elements already held in place by other dimensions: tied (directly or through a chain of
    /// distances / angles) to an axis or the origin. Dimension `skip` is left out.
    pub fn anchored_elems(&self, skip: usize) -> HashSet<usize> {
        let mut set = HashSet::new();
        loop {
            let mut changed = false;
            for (j, d) in self.dims.iter().enumerate() {
                if j == skip || d.reference || !matches!(d.kind, DimKind::Distance | DimKind::Angle) {
                    continue;
                }
                let b = match d.b {
                    Some(b) => b,
                    None => continue,
                };
                let held = |r: &GRef| self.locked(r) || r.elem().map_or(false, |e| set.contains(&e));
                let (fa, fb) = (held(&d.a), held(&b));
                if fa && !fb {
                    if let Some(e) = b.elem() {
                        changed |= set.insert(e);
                    }
                } else if fb && !fa {
                    if let Some(e) = d.a.elem() {
                        changed |= set.insert(e);
                    }
                }
            }
            if !changed {
                break;
            }
        }
        set
    }

    /// Which side of a two-item dimension moves: the one not held by other dimensions
    /// (the second item when both are free). Axes and the origin never move.
    /// Axes, the origin and reference elements from other sketches never move.
    pub fn locked(&self, r: &GRef) -> bool {
        r.fixed() || r.elem().map_or(false, |e| self.is_ext(e))
    }

    fn pick_mover(&self, di: usize, a: GRef, b: GRef) -> Option<(GRef, GRef)> {
        let anchored = self.held_for(di);
        let held = |r: &GRef| self.locked(r) || r.elem().map_or(false, |e| anchored.contains(&e));
        let same_elem = a.elem().is_some() && a.elem() == b.elem();
        if !held(&b) || same_elem && !self.locked(&b) {
            Some((b, a))
        } else if !held(&a) {
            Some((a, b))
        } else if !self.locked(&b) {
            // both are held: try the second anyway (the solver reports it if it cannot hold)
            Some((b, a))
        } else if !self.locked(&a) {
            Some((a, b))
        } else {
            None
        }
    }

    /// Makes every driving dimension true (the one being edited first).
    pub fn solve_dims(&mut self, first: Option<usize>) {
        for _ in 0..12 {
            if let Some(f) = first {
                self.dim_apply(f);
            }
            for i in 0..self.dims.len() {
                self.dim_apply(i);
            }
            if self.dims.iter().all(|d| self.dim_ok(d)) {
                break;
            }
        }
    }

    // ------------------------------------------------------------ drawing geometry

    /// Anchor point of a dimension (its text sits at anchor + offset).
    pub fn dim_anchor(&self, d: &Dim) -> Option<DVec2> {
        match self.dim_geom_raw(d, DVec2::ZERO)? {
            DimGeom::Linear { p, q, .. } => Some((p + q) * 0.5),
            DimGeom::Radial { c, .. } => Some(c),
            DimGeom::Angular { v, .. } => Some(v),
            DimGeom::Tag { at } => Some(at),
        }
    }

    pub fn dim_geom(&self, d: &Dim) -> Option<DimGeom> {
        self.dim_geom_raw(d, d.offset)
    }

    fn dim_geom_raw(&self, d: &Dim, offset: DVec2) -> Option<DimGeom> {
        let shape = d.a.elem().and_then(|i| self.shapes.get(i));
        let linear = |p: DVec2, q: DVec2| -> DimGeom {
            let t = q - p;
            let n = if t.length() > 1e-12 { perp(t.normalize()) } else { DVec2::Y };
            let mid = (p + q) * 0.5;
            DimGeom::Linear { p, q, n, o: offset.dot(n), text: mid + offset }
        };
        match d.kind {
            DimKind::Length => match shape? {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } | Shape::Slot { x1, y1, x2, y2, .. } => Some(linear(DVec2::new(*x1, *y1), DVec2::new(*x2, *y2))),
                _ => None,
            },
            DimKind::Width => match shape? {
                Shape::Rect { cx, cy, w, h, .. } => Some(linear(DVec2::new(cx - w.abs() / 2.0, cy - h.abs() / 2.0), DVec2::new(cx + w.abs() / 2.0, cy - h.abs() / 2.0))),
                _ => None,
            },
            DimKind::Height => match shape? {
                Shape::Rect { cx, cy, w, h, .. } => Some(linear(DVec2::new(cx + w.abs() / 2.0, cy - h.abs() / 2.0), DVec2::new(cx + w.abs() / 2.0, cy + h.abs() / 2.0))),
                _ => None,
            },
            DimKind::Radius | DimKind::Diameter => {
                let (c, r) = match shape? {
                    Shape::Circle { cx, cy, r } | Shape::Arc { cx, cy, r, .. } | Shape::Polygon { cx, cy, r, .. } => (DVec2::new(*cx, *cy), r.abs()),
                    Shape::Slot { x2, y2, r, .. } => (DVec2::new(*x2, *y2), r.abs()),
                    _ => return None,
                };
                let dir = if offset.length() > 1e-12 { offset.normalize() } else { DVec2::new(0.7071, 0.7071) };
                Some(DimGeom::Radial { c, r, dir, diameter: d.kind == DimKind::Diameter, text: c + offset })
            }
            DimKind::Distance => {
                let (fa, fb) = (self.form(&d.a)?, self.form(d.b.as_ref()?)?);
                let (p, q) = match (fa, fb) {
                    (Form::Pt(p), Form::Pt(q)) => (p, q),
                    (Form::Pt(p), Form::Ln(o, dir)) => (p, o + dir * (p - o).dot(dir)),
                    (Form::Ln(o, dir), Form::Pt(q)) => (o + dir * (q - o).dot(dir), q),
                    (Form::Ln(_, _), Form::Ln(ob, db)) => {
                        let p = match self.segment(&d.a) {
                            Some((s0, s1)) => (s0 + s1) * 0.5,
                            None => match self.segment(d.b.as_ref()?) {
                                Some((s0, s1)) => {
                                    // measuring from an axis: anchor across from the line's middle
                                    let m = (s0 + s1) * 0.5;
                                    if let Form::Ln(oa, da) = fa {
                                        oa + da * (m - oa).dot(da)
                                    } else {
                                        m
                                    }
                                }
                                None => DVec2::ZERO,
                            },
                        };
                        (p, ob + db * (p - ob).dot(db))
                    }
                };
                Some(linear(p, q))
            }
            DimKind::Angle => {
                let (v, d1, d2) = self.angle_frame(&d.a, d.b.as_ref()?)?;
                Some(DimGeom::Angular { v, d1, d2, r: offset.length().max(1e-6), text: v + offset })
            }
            DimKind::Horizontal | DimKind::Vertical => match shape? {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => Some(DimGeom::Tag { at: DVec2::new(x1 + x2, y1 + y2) * 0.5 + offset }),
                _ => None,
            },
        }
    }

    /// The dimension a single element gets with the Constraint tool, given where the text is
    /// placed (a rectangle gets its width or its height depending on the side).
    pub fn single_dim_for(&self, i: usize, place: DVec2) -> Option<Dim> {
        let s = self.shapes.get(i)?;
        let kind = match s {
            Shape::Line { .. } | Shape::Axis { .. } | Shape::Slot { .. } => DimKind::Length,
            Shape::Circle { .. } => DimKind::Diameter,
            Shape::Arc { .. } | Shape::Polygon { .. } => DimKind::Radius,
            Shape::Rect { cx, cy, w, h, .. } => {
                let rel = place - DVec2::new(*cx, *cy);
                if rel.y.abs() / h.abs().max(1e-9) > rel.x.abs() / w.abs().max(1e-9) { DimKind::Width } else { DimKind::Height }
            }
            _ => return None,
        };
        let mut d = Dim { kind, a: GRef::Elem(i), b: None, value: 0.0, offset: DVec2::ZERO, reference: false };
        d.value = self.dim_measure(&d)?;
        let anchor = self.dim_anchor(&d)?;
        d.offset = place - anchor;
        Some(d)
    }

    /// The dimension between two references: an angle for two lines that are not parallel,
    /// otherwise a distance.
    pub fn pair_dim_for(&self, a: GRef, b: GRef, place: DVec2) -> Option<Dim> {
        let both_lines = matches!((self.form(&a), self.form(&b)), (Some(Form::Ln(_, da)), Some(Form::Ln(_, db))) if da.perp_dot(db).abs() > 1e-6);
        let kind = if both_lines { DimKind::Angle } else { DimKind::Distance };
        let mut d = Dim { kind, a, b: Some(b), value: 0.0, offset: DVec2::ZERO, reference: false };
        d.value = self.dim_measure(&d)?;
        let anchor = self.dim_anchor(&d)?;
        d.offset = place - anchor;
        if kind == DimKind::Angle && d.offset.length() < 1e-6 {
            d.offset = DVec2::new(5.0, 5.0);
        }
        Some(d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{signed_area, Frame};

    fn line(a: (f64, f64), b: (f64, f64)) -> Shape {
        Shape::Line { x1: a.0, y1: a.1, x2: b.0, y2: b.1 }
    }

    #[test]
    fn length_keeps_rectangle_square() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(line((0.0, 0.0), (20.0, 0.0)), false);
        sk.push(line((20.0, 0.0), (20.0, 10.0)), false);
        sk.push(line((20.0, 10.0), (0.0, 10.0)), false);
        sk.push(line((0.0, 10.0), (0.0, 0.0)), false);
        let mut d = sk.single_dim_for(0, DVec2::new(10.0, -5.0)).unwrap();
        assert_eq!(d.kind, DimKind::Length);
        assert!((d.value - 20.0).abs() < 1e-9);
        d.value = 35.0;
        sk.dims.push(d);
        sk.solve_dims(Some(0));
        let (s, _) = sk.regions().unwrap();
        assert!((signed_area(&s[0]) - 350.0).abs() < 1e-6, "{}", signed_area(&s[0]));
        // the right side stayed vertical
        if let Shape::Line { x1, x2, .. } = sk.shapes[1] {
            assert!((x1 - 35.0).abs() < 1e-9 && (x2 - 35.0).abs() < 1e-9);
        }
    }

    #[test]
    fn distance_angle_and_radius() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(Shape::Circle { cx: 10.0, cy: 0.0, r: 3.0 }, false);
        let mut d = sk.pair_dim_for(GRef::Origin, GRef::Elem(0), DVec2::new(5.0, 5.0)).unwrap();
        d.value = 25.0;
        sk.dims.push(d);
        let mut r = sk.single_dim_for(0, DVec2::new(15.0, 5.0)).unwrap();
        assert_eq!(r.kind, DimKind::Diameter);
        r.value = 8.0;
        sk.dims.push(r);
        sk.solve_dims(None);
        if let Shape::Circle { cx, r, .. } = sk.shapes[0] {
            assert!((cx - 25.0).abs() < 1e-9 && (r - 4.0).abs() < 1e-9);
        }
        // angle between two lines meeting at a corner
        sk.push(line((0.0, 0.0), (10.0, 0.0)), false);
        sk.push(line((0.0, 0.0), (10.0, 10.0)), false);
        let mut a = sk.pair_dim_for(GRef::Elem(1), GRef::Elem(2), DVec2::new(4.0, 2.0)).unwrap();
        assert_eq!(a.kind, DimKind::Angle);
        assert!((a.value - 45.0).abs() < 1e-9);
        a.value = 60.0;
        sk.dims.push(a);
        sk.solve_dims(Some(2));
        assert!(sk.dims.iter().all(|d| sk.dim_ok(d)));
        if let Shape::Line { x2, y2, .. } = sk.shapes[2] {
            let ang = y2.atan2(x2).to_degrees();
            assert!((ang - 60.0).abs() < 1e-6, "{}", ang);
        }
        // removing the circle drops its dimensions
        sk.remove(0);
        let real: Vec<&Dim> = sk.dims.iter().filter(|d| !d.kind.is_geometric()).collect();
        assert_eq!(real.len(), 1);
        assert_eq!(real[0].a, GRef::Elem(0));
    }

    #[test]
    fn moves_the_free_line() {
        for order in [false, true] {
            let mut sk = Sketch::new(Frame::xy(), "xy".into());
            sk.push(line((0.0, 10.0), (20.0, 10.0)), false); // line 0
            sk.push(line((0.0, 18.0), (20.0, 18.0)), false); // line 1
            // line 0 held 10 mm from the H axis
            sk.dims.push(Dim { kind: DimKind::Distance, a: GRef::AxisH, b: Some(GRef::Elem(0)), value: 10.0, offset: DVec2::ZERO, reference: false });
            // distance between the lines, clicked in either order
            let (a, b) = if order { (GRef::Elem(1), GRef::Elem(0)) } else { (GRef::Elem(0), GRef::Elem(1)) };
            sk.dims.push(Dim { kind: DimKind::Distance, a, b: Some(b), value: 8.0, offset: DVec2::ZERO, reference: false });
            let last = sk.dims.len() - 1;
            sk.dims[last].value = 25.0;
            sk.solve_dims(Some(last));
            assert!(sk.dims.iter().all(|d| sk.dim_ok(d)), "order {}", order);
            if let (Shape::Line { y1: a, .. }, Shape::Line { y1: b, .. }) = (&sk.shapes[0], &sk.shapes[1]) {
                assert!((a - 10.0).abs() < 1e-9 && (b - 35.0).abs() < 1e-9, "order {}: {} {}", order, a, b);
            }
        }
    }

    #[test]
    fn fixed_line_stays_put() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(line((0.0, 10.0), (30.0, 10.0)), false); // 0 bottom (gets H)
        sk.push(line((30.0, 10.0), (30.0, 20.0)), false); // 1 right (gets V)
        sk.push(line((30.0, 20.0), (0.0, 20.0)), false); // 2 top (gets H)
        sk.push(line((0.0, 20.0), (0.0, 10.0)), false); // 3 left (gets V)
        assert_eq!(sk.dims.iter().filter(|d| d.kind.is_geometric()).count(), 4);
        // fix the bottom line: length, height above H axis, its left end from the V axis
        let d = |kind, a, b: Option<GRef>, value| Dim { kind, a, b, value, offset: DVec2::ZERO, reference: false };
        sk.dims.push(d(DimKind::Length, GRef::Elem(0), None, 30.0));
        sk.dims.push(d(DimKind::Distance, GRef::AxisH, Some(GRef::Elem(0)), 10.0));
        sk.dims.push(d(DimKind::Distance, GRef::AxisV, Some(GRef::Key(0, 0)), 0.0));
        let (fixed, _) = sk.fixed_state(None);
        assert!(fixed.contains(&0), "bottom line should count as fixed");
        assert!(!fixed.contains(&2), "top line is still free");
        // distance between the lines, picked bottom first (so the bottom would be the default mover)
        sk.dims.push(d(DimKind::Distance, GRef::Elem(2), Some(GRef::Elem(0)), 10.0));
        let last = sk.dims.len() - 1;
        sk.dims[last].value = 25.0;
        sk.solve_dims(Some(last));
        assert!(sk.dims.iter().all(|x| sk.dim_ok(x)));
        let ys = |i: usize| match sk.shapes[i] {
            Shape::Line { y1, y2, .. } => (y1, y2),
            _ => (0.0, 0.0),
        };
        assert_eq!(ys(0), (10.0, 10.0), "the fixed line must not move");
        assert_eq!(ys(2), (35.0, 35.0), "the free line moves");
        let (s2, _) = sk.regions().unwrap();
        assert!((crate::geom::signed_area(&s2[0]) - 750.0).abs() < 1e-6);
    }
}
