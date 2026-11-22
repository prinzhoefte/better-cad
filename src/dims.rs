//! Sketch constraints, classic CAD (CATIA V5) style: dimensional constraints (lengths,
//! distances, radii, angles, semi-axes) and geometric ones (horizontal, vertical, coincidence,
//! concentricity, tangency, parallelism, perpendicularity, midpoint, equidistant point,
//! symmetry, fix). They stay attached to the geometry and drive it.
//!
//! All constraints are solved together: every element's numbers are unknowns, every constraint
//! adds equations, and a Levenberg-Marquardt solver finds the smallest change that makes all
//! of them true. A rank analysis of the same equations tells which elements are still free
//! (under-constrained), fully decided (iso-constrained, green) or over-constrained (purple),
//! and which constraints cannot hold (red).

use crate::sketch::*;
use glam::DVec2;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::f64::consts::{PI, TAU};

/// What a constraint is attached to.
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub enum GRef {
    /// a whole element (line, circle, arc, point...)
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
    Horizontal,
    Vertical,
    /// semimajor / semiminor axis of an ellipse
    MajorAxis,
    MinorAxis,
    Coincidence,
    Concentric,
    Tangent,
    Parallel,
    Perpendicular,
    /// a point at the middle of a line
    Midpoint,
    /// a point (a) as far from point b as from point c
    Equidistant,
    /// a and b mirror images across line c
    Symmetry,
    /// the element never moves
    Fix,
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
            DimKind::MajorAxis => "Semimajor axis",
            DimKind::MinorAxis => "Semiminor axis",
            DimKind::Coincidence => "Coincidence",
            DimKind::Concentric => "Concentricity",
            DimKind::Tangent => "Tangency",
            DimKind::Parallel => "Parallelism",
            DimKind::Perpendicular => "Perpendicularity",
            DimKind::Midpoint => "Midpoint",
            DimKind::Equidistant => "Equidistant point",
            DimKind::Symmetry => "Symmetry",
            DimKind::Fix => "Fix",
        }
    }
    /// Geometric constraints are shown as a small symbol and have no value to edit.
    pub fn is_geometric(self) -> bool {
        !matches!(self, DimKind::Length | DimKind::Width | DimKind::Height | DimKind::Radius | DimKind::Diameter | DimKind::Distance | DimKind::Angle | DimKind::MajorAxis | DimKind::MinorAxis)
    }
}

/// How a distance between two points is measured (classic CAD's right-click choice).
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq, Eq, Default)]
pub enum MeasDir {
    /// the true (shortest) distance
    #[default]
    Normal,
    /// along H only
    Horizontal,
    /// along V only
    Vertical,
}

#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
pub struct Dim {
    pub kind: DimKind,
    pub a: GRef,
    pub b: Option<GRef>,
    /// third item (symmetry line, second point of an equidistant point)
    #[serde(default)]
    pub c: Option<GRef>,
    pub value: f64,
    /// where the value text sits, relative to the dimension's anchor
    pub offset: DVec2,
    /// a reference dimension only shows the value and never drives the geometry
    #[serde(default)]
    pub reference: bool,
    #[serde(default)]
    pub dir: MeasDir,
}

impl Dim {
    pub fn new(kind: DimKind, a: GRef, b: Option<GRef>, value: f64) -> Dim {
        Dim { kind, a, b, c: None, value, offset: DVec2::ZERO, reference: false, dir: MeasDir::Normal }
    }
    pub fn label(&self) -> String {
        let v = fmt((self.value * 1000.0).round() / 1000.0);
        let t = match self.kind {
            DimKind::Radius => format!("R{}", v),
            DimKind::Diameter => format!("D{}", v),
            DimKind::Angle => format!("{} deg", v),
            DimKind::Horizontal => "H".into(),
            DimKind::Vertical => "V".into(),
            k if k.is_geometric() => String::new(),
            _ => v,
        };
        if self.reference { format!("({})", t) } else { t }
    }
    fn refs(&self) -> impl Iterator<Item = GRef> + '_ {
        std::iter::once(self.a).chain(self.b).chain(self.c)
    }
    /// The elements this constraint ties together.
    pub fn elems(&self) -> Vec<usize> {
        let mut v: Vec<usize> = self.refs().filter_map(|r| r.elem()).collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// A geometric reading of a reference: a point, or an infinite line (unit direction).
#[derive(Clone, Copy, Debug)]
pub enum Form {
    Pt(DVec2),
    Ln(DVec2, DVec2),
}

/// How a dimension is drawn.
pub enum DimGeom {
    /// extension lines from p and q out to the dimension line a..b (normal n); text position
    Linear { p: DVec2, q: DVec2, a: DVec2, b: DVec2, n: DVec2, text: DVec2 },
    /// centre, radius, direction to the text, diameter or radius style
    Radial { c: DVec2, r: f64, dir: DVec2, diameter: bool, text: DVec2 },
    /// vertex, the two directions, arc radius
    Angular { v: DVec2, d1: DVec2, d2: DVec2, r: f64, text: DVec2 },
    /// a small symbol next to the constrained geometry (and a second one on the other element)
    Tag { at: DVec2, also: Option<DVec2> },
}

/// How far a constraint is from holding, per element / constraint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CState {
    /// can still move (white)
    Under,
    /// fully decided (green); for constraints: holds
    Iso,
    /// more constraints than needed (purple)
    Over,
    /// cannot hold (red)
    Bad,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SkStatus {
    pub elems: Vec<CState>,
    pub dims: Vec<CState>,
}

/// Which numbers of which elements should rather stay where they are while solving (a
/// dragged element, the side of a dimension that was picked first), and which never move.
#[derive(Default, Clone)]
pub struct SolveOpts {
    /// (element, parameter index) pairs that are expensive to move
    pub heavy: Vec<(usize, usize)>,
    /// elements that must not move at all
    pub lock: Vec<usize>,
}

fn perp(d: DVec2) -> DVec2 {
    DVec2::new(-d.y, d.x)
}

fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(TAU) - PI
}

fn sgn(x: f64) -> f64 {
    if x < 0.0 { -1.0 } else { 1.0 }
}

// ---------------------------------------------------------------- reading references

fn key(sh: &[Shape], i: usize, k: usize) -> Option<DVec2> {
    sh.get(i)?.key_points().get(k).copied()
}

/// A point reference: a key point, the origin or a Point element.
fn point(sh: &[Shape], r: &GRef) -> Option<DVec2> {
    match *r {
        GRef::Key(i, k) => key(sh, i, k),
        GRef::Origin => Some(DVec2::ZERO),
        GRef::Elem(i) => match sh.get(i)? {
            Shape::Point { x, y } => Some(DVec2::new(*x, *y)),
            _ => None,
        },
        _ => None,
    }
}

/// A straight reference: two points on it.
fn line(sh: &[Shape], r: &GRef) -> Option<(DVec2, DVec2)> {
    match *r {
        GRef::AxisH => Some((DVec2::ZERO, DVec2::X)),
        GRef::AxisV => Some((DVec2::ZERO, DVec2::Y)),
        GRef::Elem(i) => match sh.get(i)? {
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => Some((DVec2::new(*x1, *y1), DVec2::new(*x2, *y2))),
            _ => None,
        },
        _ => None,
    }
}

/// A line as origin and unit direction.
fn line_n(sh: &[Shape], r: &GRef) -> Option<(DVec2, DVec2)> {
    let (a, b) = line(sh, r)?;
    let d = b - a;
    if d.length() < 1e-12 {
        return None;
    }
    Some((a, d.normalize()))
}

/// A round reference: centre and radius.
fn circ(sh: &[Shape], r: &GRef) -> Option<(DVec2, f64)> {
    match *r {
        GRef::Elem(i) => match sh.get(i)? {
            Shape::Circle { cx, cy, r } | Shape::Arc { cx, cy, r, .. } => Some((DVec2::new(*cx, *cy), r.abs())),
            _ => None,
        },
        _ => None,
    }
}

/// The centre of a round element, or a point reference.
fn centre(sh: &[Shape], r: &GRef) -> Option<DVec2> {
    match *r {
        GRef::Elem(i) => match sh.get(i)? {
            Shape::Circle { cx, cy, .. } | Shape::Arc { cx, cy, .. } | Shape::Ellipse { cx, cy, .. } | Shape::Polygon { cx, cy, .. } | Shape::Rect { cx, cy, .. } => Some(DVec2::new(*cx, *cy)),
            _ => point(sh, r),
        },
        _ => point(sh, r),
    }
}

fn ellipse(sh: &[Shape], r: &GRef) -> Option<(DVec2, f64, f64, f64)> {
    match *r {
        GRef::Elem(i) => match sh.get(i)? {
            Shape::Ellipse { cx, cy, a, b, rot } => Some((DVec2::new(*cx, *cy), a.abs(), b.abs(), rot.to_radians())),
            _ => None,
        },
        _ => None,
    }
}

fn arc_ends(sh: &[Shape], r: &GRef) -> Option<(DVec2, DVec2)> {
    match *r {
        GRef::Elem(i) => match sh.get(i)? {
            s @ Shape::Arc { .. } => s.endpoints(),
            _ => None,
        },
        _ => None,
    }
}

fn form(sh: &[Shape], r: &GRef) -> Option<Form> {
    match *r {
        GRef::AxisH => Some(Form::Ln(DVec2::ZERO, DVec2::X)),
        GRef::AxisV => Some(Form::Ln(DVec2::ZERO, DVec2::Y)),
        GRef::Origin | GRef::Key(..) => point(sh, r).map(Form::Pt),
        GRef::Elem(i) => match sh.get(i)? {
            Shape::Line { x1, y1, .. } | Shape::Axis { x1, y1, .. } => match line_n(sh, r) {
                Some((o, d)) => Some(Form::Ln(o, d)),
                None => Some(Form::Pt(DVec2::new(*x1, *y1))),
            },
            Shape::Slot { x1, y1, x2, y2, .. } => Some(Form::Pt(DVec2::new(x1 + x2, y1 + y2) * 0.5)),
            _ => centre(sh, r).map(Form::Pt),
        },
    }
}

/// The middle of an element (where its symbols are drawn).
fn elem_mid(sh: &[Shape], r: &GRef) -> Option<DVec2> {
    if let Some(p) = point(sh, r) {
        return Some(p);
    }
    if let Some((a, b)) = line(sh, r) {
        return Some(if r.fixed() { a } else { (a + b) * 0.5 });
    }
    let c = sh.get(r.elem()?)?.curve();
    c.get(c.len() / 2).copied()
}

/// A line and a circle among two references, in that order.
fn line_and_circle(sh: &[Shape], a: &GRef, b: &GRef) -> Option<((DVec2, DVec2), (DVec2, f64))> {
    if let (Some(l), Some(c)) = (line_n(sh, a), circ(sh, b)) {
        return Some((l, c));
    }
    if let (Some(l), Some(c)) = (line_n(sh, b), circ(sh, a)) {
        return Some((l, c));
    }
    None
}

/// Pivot and directions (pointing away from the pivot) of two straight references.
fn angle_frame(sh: &[Shape], a: &GRef, b: &GRef) -> Option<(DVec2, DVec2, DVec2)> {
    let ((oa, da), (ob, db)) = (line_n(sh, a)?, line_n(sh, b)?);
    let den = da.perp_dot(db);
    if den.abs() < 1e-9 {
        return None;
    }
    let seg = |r: &GRef| if r.fixed() { None } else { line(sh, r) };
    // shared end point, else the crossing of the two lines
    let mut pivot = None;
    if let (Some((a0, a1)), Some((b0, b1))) = (seg(a), seg(b)) {
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
        match seg(r) {
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

fn reflect(p: DVec2, o: DVec2, d: DVec2) -> DVec2 {
    let w = p - o;
    o + d * (2.0 * w.dot(d)) - w
}

// ---------------------------------------------------------------- equations

/// Signs picked from the geometry when solving starts: which side of a line a point stays
/// on, which way an angle opens, outside or inside tangency, how symmetric ends pair up.
fn signs(sh: &[Shape], d: &Dim) -> [f64; 3] {
    let mut s = [1.0; 3];
    let Some(b) = d.b else { return s };
    match d.kind {
        DimKind::Distance => {
            if d.dir != MeasDir::Normal {
                if let (Some(p), Some(q)) = (point(sh, &d.a), point(sh, &b)) {
                    s[0] = sgn(if d.dir == MeasDir::Horizontal { q.x - p.x } else { q.y - p.y });
                }
                return s;
            }
            match (form(sh, &d.a), form(sh, &b)) {
                (Some(Form::Pt(p)), Some(Form::Ln(o, dir))) | (Some(Form::Ln(o, dir)), Some(Form::Pt(p))) => s[0] = sgn(dir.perp_dot(p - o)),
                (Some(Form::Ln(..)), Some(Form::Ln(..))) => {
                    if let Some((o, dir, m)) = line_pair(sh, &d.a, &b) {
                        s[0] = sgn(dir.perp_dot(m - o));
                    }
                }
                _ => {}
            }
        }
        DimKind::Angle => {
            if let (Some((_, da)), Some((_, db)), Some((_, ua, ub))) = (line_n(sh, &d.a), line_n(sh, &b), angle_frame(sh, &d.a, &b)) {
                s[0] = sgn(ua.dot(da));
                s[1] = sgn(ub.dot(db));
                let (u1, u2) = (da * s[0], db * s[1]);
                s[2] = sgn(u1.perp_dot(u2).atan2(u1.dot(u2)));
            }
        }
        DimKind::Tangent => {
            // s[2]: which end the two are joined at (0: not joined; see Sketch::signs_of)
            s[2] = 0.0;
            if let Some(((o, dir), (c, _))) = line_and_circle(sh, &d.a, &b) {
                s[0] = sgn(dir.perp_dot(c - o));
            } else if let (Some((c1, r1)), Some((c2, r2))) = (circ(sh, &d.a), circ(sh, &b)) {
                let dist = (c1 - c2).length();
                s[0] = if (dist - (r1 + r2)).abs() <= (dist - (r1 - r2).abs()).abs() { 1.0 } else { -1.0 };
                s[1] = sgn(r1 - r2);
            }
        }
        DimKind::Symmetry => {
            let Some(c) = d.c else { return s };
            let Some((o, dir)) = line_n(sh, &c) else { return s };
            let ends = |r: &GRef| line(sh, r).filter(|_| !r.fixed()).or_else(|| arc_ends(sh, r));
            if let (Some((a1, _)), Some((b1, b2))) = (ends(&d.a), ends(&b)) {
                let ra = reflect(a1, o, dir);
                s[0] = if (ra - b1).length() <= (ra - b2).length() { 1.0 } else { -1.0 };
            }
        }
        _ => {}
    }
    s
}

/// For a distance between two straight references: the line measured from (origin, unit
/// direction) and the middle of the other one. An axis is always measured from.
fn line_pair(sh: &[Shape], a: &GRef, b: &GRef) -> Option<(DVec2, DVec2, DVec2)> {
    let (from, to) = if b.fixed() && !a.fixed() { (b, a) } else { (a, b) };
    let (o, dir) = line_n(sh, from)?;
    let (e1, e2) = line(sh, to)?;
    let m = if to.fixed() { e1 } else { (e1 + e2) * 0.5 };
    Some((o, dir, m))
}

/// The equations of a constraint (all zero when it holds), or None when it does not apply to
/// what it is attached to.
fn residuals(sh: &[Shape], d: &Dim, sg: &[f64; 3]) -> Option<Vec<f64>> {
    let v = d.value;
    let shape = d.a.elem().and_then(|i| sh.get(i));
    Some(match d.kind {
        DimKind::Length => match shape? {
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } | Shape::Slot { x1, y1, x2, y2, .. } => vec![DVec2::new(x2 - x1, y2 - y1).length() - v],
            _ => return None,
        },
        DimKind::Width => match shape? {
            Shape::Rect { w, .. } => vec![w.abs() - v],
            _ => return None,
        },
        DimKind::Height => match shape? {
            Shape::Rect { h, .. } => vec![h.abs() - v],
            _ => return None,
        },
        DimKind::Radius => match shape? {
            Shape::Circle { r, .. } | Shape::Arc { r, .. } | Shape::Polygon { r, .. } | Shape::Slot { r, .. } => vec![r.abs() - v],
            _ => return None,
        },
        DimKind::Diameter => match shape? {
            Shape::Circle { r, .. } | Shape::Arc { r, .. } => vec![r.abs() * 2.0 - v],
            _ => return None,
        },
        DimKind::MajorAxis | DimKind::MinorAxis => match shape? {
            Shape::Ellipse { a, b, .. } => vec![if d.kind == DimKind::MajorAxis { a.abs() } else { b.abs() } - v],
            _ => return None,
        },
        DimKind::Distance => {
            let b = d.b?;
            if d.dir != MeasDir::Normal {
                let (p, q) = (point(sh, &d.a).or_else(|| centre(sh, &d.a))?, point(sh, &b).or_else(|| centre(sh, &b))?);
                let k = if d.dir == MeasDir::Horizontal { 0 } else { 1 };
                return Some(vec![sg[0] * (q[k] - p[k]) - v]);
            }
            match (form(sh, &d.a)?, form(sh, &b)?) {
                (Form::Pt(p), Form::Pt(q)) => vec![(q - p).length() - v],
                (Form::Pt(p), Form::Ln(o, dir)) | (Form::Ln(o, dir), Form::Pt(p)) => vec![sg[0] * dir.perp_dot(p - o) - v],
                (Form::Ln(..), Form::Ln(..)) => {
                    let (o, dir, m) = line_pair(sh, &d.a, &b)?;
                    vec![sg[0] * dir.perp_dot(m - o) - v]
                }
            }
        }
        DimKind::Angle => {
            let ((_, da), (_, db)) = (line_n(sh, &d.a)?, line_n(sh, &d.b?)?);
            let (u1, u2) = (da * sg[0], db * sg[1]);
            vec![wrap(u1.perp_dot(u2).atan2(u1.dot(u2)) - sg[2] * v.to_radians())]
        }
        DimKind::Horizontal | DimKind::Vertical => {
            let (a, b) = line(sh, &d.a)?;
            vec![if d.kind == DimKind::Horizontal { b.y - a.y } else { b.x - a.x }]
        }
        DimKind::Coincidence => {
            let b = d.b?;
            if let (Some(p), Some(q)) = (point(sh, &d.a), point(sh, &b)) {
                vec![p.x - q.x, p.y - q.y]
            } else if let Some(r) = on_curve(sh, &d.a, &b).or_else(|| on_curve(sh, &b, &d.a)) {
                r
            } else if let (Some((o, dir)), Some((e1, e2))) = (line_n(sh, &d.a), line(sh, &b)) {
                vec![dir.perp_dot(e1 - o), dir.perp_dot(e2 - o)]
            } else if let (Some((c1, r1)), Some((c2, r2))) = (circ(sh, &d.a), circ(sh, &b)) {
                vec![c1.x - c2.x, c1.y - c2.y, r1 - r2]
            } else {
                return None;
            }
        }
        DimKind::Concentric => {
            let (p, q) = (centre(sh, &d.a)?, centre(sh, &d.b?)?);
            vec![p.x - q.x, p.y - q.y]
        }
        DimKind::Tangent => {
            let b = d.b?;
            if let Some(((o, dir), (c, r))) = line_and_circle(sh, &d.a, &b) {
                if sg[2] > 0.0 {
                    // joined at a line end: the line stands square to the radius there
                    let (l, _) = if line_n(sh, &d.a).is_some() { (d.a, b) } else { (b, d.a) };
                    let (e0, e1) = line(sh, &l)?;
                    let e = if sg[2] < 1.5 { e0 } else { e1 };
                    vec![dir.dot(e - c)]
                } else {
                    vec![sg[0] * dir.perp_dot(c - o) - r]
                }
            } else {
                let ((c1, r1), (c2, r2)) = (circ(sh, &d.a)?, circ(sh, &b)?);
                if sg[2] > 0.0 {
                    // joined at an arc end: both centres in line with that point
                    let (e0, e1) = arc_ends(sh, &d.a)?;
                    let e = if sg[2] < 1.5 { e0 } else { e1 };
                    vec![(c2 - c1).normalize_or_zero().perp_dot(e - c1)]
                } else {
                    let dist = (c1 - c2).length();
                    vec![if sg[0] > 0.0 { dist - (r1 + r2) } else { dist - sg[1] * (r1 - r2) }]
                }
            }
        }
        DimKind::Parallel | DimKind::Perpendicular => {
            let ((_, da), (_, db)) = (line_n(sh, &d.a)?, line_n(sh, &d.b?)?);
            vec![if d.kind == DimKind::Parallel { da.perp_dot(db) } else { da.dot(db) }]
        }
        DimKind::Midpoint => {
            let b = d.b?;
            let (p, (e1, e2)) = match (point(sh, &d.a), line(sh, &b)) {
                (Some(p), Some(l)) => (p, l),
                _ => (point(sh, &b)?, line(sh, &d.a)?),
            };
            let m = (e1 + e2) * 0.5;
            vec![p.x - m.x, p.y - m.y]
        }
        DimKind::Equidistant => {
            let (p, q1, q2) = (point(sh, &d.a)?, point(sh, &d.b?)?, point(sh, &d.c?)?);
            vec![(p - q1).length() - (p - q2).length()]
        }
        DimKind::Symmetry => {
            let (b, c) = (d.b?, d.c?);
            let (o, dir) = line_n(sh, &c)?;
            if let (Some(p), Some(q)) = (point(sh, &d.a), point(sh, &b)) {
                let m = (p + q) * 0.5;
                vec![dir.perp_dot(m - o), dir.dot(q - p)]
            } else if let (Some((a1, a2)), Some((b1, b2))) = (line(sh, &d.a).filter(|_| !d.a.fixed()), line(sh, &b).filter(|_| !b.fixed())) {
                let (b1, b2) = if sg[0] > 0.0 { (b1, b2) } else { (b2, b1) };
                let (r1, r2) = (reflect(a1, o, dir), reflect(a2, o, dir));
                vec![r1.x - b1.x, r1.y - b1.y, r2.x - b2.x, r2.y - b2.y]
            } else if let (Some((a1, a2)), Some((b1, b2)), Some((_, ra)), Some((_, rb))) = (arc_ends(sh, &d.a), arc_ends(sh, &b), circ(sh, &d.a), circ(sh, &b)) {
                let (b1, b2) = if sg[0] > 0.0 { (b1, b2) } else { (b2, b1) };
                let (r1, r2) = (reflect(a1, o, dir), reflect(a2, o, dir));
                vec![r1.x - b1.x, r1.y - b1.y, r2.x - b2.x, r2.y - b2.y, ra - rb]
            } else {
                let ((c1, r1), (c2, r2)) = (circ(sh, &d.a)?, circ(sh, &b)?);
                let m = reflect(c1, o, dir);
                vec![m.x - c2.x, m.y - c2.y, r1 - r2]
            }
        }
        DimKind::Fix => vec![],
    })
}

/// A point lying on a line, circle, arc or ellipse.
fn on_curve(sh: &[Shape], p: &GRef, c: &GRef) -> Option<Vec<f64>> {
    let p = point(sh, p)?;
    if let Some((o, dir)) = line_n(sh, c) {
        return Some(vec![dir.perp_dot(p - o)]);
    }
    if let Some((cc, r)) = circ(sh, c) {
        return Some(vec![(p - cc).length() - r]);
    }
    if let Some((cc, a, b, rot)) = ellipse(sh, c) {
        let (s, co) = rot.sin_cos();
        let w = p - cc;
        let (x, y) = (w.x * co + w.y * s, -w.x * s + w.y * co);
        return Some(vec![((x / a).powi(2) + (y / b).powi(2) - 1.0) * a.min(b) * 0.5]);
    }
    None
}

// ---------------------------------------------------------------- unknowns

/// Angles are scaled to lengths (radians times radius) so the solver weighs turning an arc
/// like moving its ends.
fn shape_scale(s: &Shape) -> f64 {
    match s {
        Shape::Arc { r, .. } | Shape::Polygon { r, .. } => r.abs().max(1.0),
        Shape::Ellipse { a, .. } => a.abs().max(1.0),
        _ => 1.0,
    }
}

/// The numbers that decide an element (its unknowns).
pub fn params(s: &Shape, sc: f64) -> Vec<f64> {
    match s {
        Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => vec![*x1, *y1, *x2, *y2],
        Shape::Point { x, y } => vec![*x, *y],
        Shape::Circle { cx, cy, r } => vec![*cx, *cy, *r],
        Shape::Arc { cx, cy, r, a0, a1 } => vec![*cx, *cy, *r, a0.to_radians() * sc, a1.to_radians() * sc],
        Shape::Ellipse { cx, cy, a, b, rot } => vec![*cx, *cy, *a, *b, rot.to_radians() * sc],
        Shape::Rect { cx, cy, w, h, .. } => vec![*cx, *cy, *w, *h],
        Shape::Polygon { cx, cy, r, rot, .. } => vec![*cx, *cy, *r, rot.to_radians() * sc],
        Shape::Slot { x1, y1, x2, y2, r } => vec![*x1, *y1, *x2, *y2, *r],
        Shape::Profile { pts } | Shape::Spline { pts, .. } => pts.iter().flat_map(|p| [p.x, p.y]).collect(),
    }
}

pub fn set_params(s: &mut Shape, p: &[f64], sc: f64) {
    match s {
        Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
            (*x1, *y1, *x2, *y2) = (p[0], p[1], p[2], p[3]);
        }
        Shape::Point { x, y } => (*x, *y) = (p[0], p[1]),
        Shape::Circle { cx, cy, r } => (*cx, *cy, *r) = (p[0], p[1], p[2]),
        Shape::Arc { cx, cy, r, a0, a1 } => (*cx, *cy, *r, *a0, *a1) = (p[0], p[1], p[2], (p[3] / sc).to_degrees(), (p[4] / sc).to_degrees()),
        Shape::Ellipse { cx, cy, a, b, rot } => (*cx, *cy, *a, *b, *rot) = (p[0], p[1], p[2], p[3], (p[4] / sc).to_degrees()),
        Shape::Rect { cx, cy, w, h, .. } => (*cx, *cy, *w, *h) = (p[0], p[1], p[2], p[3]),
        Shape::Polygon { cx, cy, r, rot, .. } => (*cx, *cy, *r, *rot) = (p[0], p[1], p[2], (p[3] / sc).to_degrees()),
        Shape::Slot { x1, y1, x2, y2, r } => (*x1, *y1, *x2, *y2, *r) = (p[0], p[1], p[2], p[3], p[4]),
        Shape::Profile { pts } | Shape::Spline { pts, .. } => {
            for (k, q) in pts.iter_mut().enumerate() {
                *q = DVec2::new(p[2 * k], p[2 * k + 1]);
            }
        }
    }
}

/// The unknowns of a dragged handle point (see `Shape::handle_points`).
pub fn handle_params(s: &Shape, k: usize) -> Vec<usize> {
    match s {
        Shape::Line { .. } | Shape::Axis { .. } | Shape::Profile { .. } | Shape::Spline { .. } => vec![2 * k, 2 * k + 1],
        Shape::Point { .. } => vec![0, 1],
        _ => vec![],
    }
}

/// The equations and unknowns of a sketch.
struct System {
    /// unknowns: (element, parameter index)
    vars: Vec<(usize, usize)>,
    /// unknown indices of each element (empty when locked)
    of_shape: Vec<Vec<usize>>,
    scale: Vec<f64>,
    /// constraints taking part, their signs, their unknowns and how many equations each has
    eqs: Vec<(usize, [f64; 3], Vec<usize>, usize)>,
    /// constraints that do not apply to what they are attached to
    broken: Vec<usize>,
}

impl System {
    fn gather(&self, sh: &[Shape]) -> Vec<f64> {
        self.vars.iter().map(|&(i, k)| params(&sh[i], self.scale[i])[k]).collect()
    }
    fn write_shape(&self, sh: &mut [Shape], x: &[f64], i: usize) {
        if self.of_shape[i].is_empty() {
            return;
        }
        // pinned numbers keep their value
        let mut p = params(&sh[i], self.scale[i]);
        for &v in &self.of_shape[i] {
            p[self.vars[v].1] = x[v];
        }
        set_params(&mut sh[i], &p, self.scale[i]);
    }
    fn write(&self, sh: &mut [Shape], x: &[f64]) {
        for i in 0..sh.len() {
            self.write_shape(sh, x, i);
        }
    }
    fn eval(&self, sh: &[Shape], dims: &[Dim]) -> Vec<f64> {
        let mut out = Vec::new();
        for (j, sg, _, n) in &self.eqs {
            match residuals(sh, &dims[*j], sg) {
                Some(r) if r.len() == *n && r.iter().all(|x| x.is_finite()) => out.extend(r),
                _ => out.extend(std::iter::repeat(1e6).take(*n)),
            }
        }
        out
    }
    /// Sparse Jacobian rows: (unknown, derivative) per equation, by central differences.
    fn jacobian(&self, sh: &mut [Shape], dims: &[Dim], x: &[f64]) -> Vec<Vec<(usize, f64)>> {
        let mut rows = Vec::new();
        let mut xx = x.to_vec();
        for (j, sg, deps, n) in &self.eqs {
            let mut block: Vec<Vec<(usize, f64)>> = vec![vec![]; *n];
            for &v in deps {
                let i = self.vars[v].0;
                let h = 1e-6 * x[v].abs().max(1.0);
                xx[v] = x[v] + h;
                self.write_shape(sh, &xx, i);
                let rp = residuals(sh, &dims[*j], sg);
                xx[v] = x[v] - h;
                self.write_shape(sh, &xx, i);
                let rm = residuals(sh, &dims[*j], sg);
                xx[v] = x[v];
                self.write_shape(sh, &xx, i);
                if let (Some(rp), Some(rm)) = (rp, rm) {
                    for k in 0..(*n).min(rp.len()).min(rm.len()) {
                        let g = (rp[k] - rm[k]) / (2.0 * h);
                        if g != 0.0 && g.is_finite() {
                            block[k].push((v, g));
                        }
                    }
                }
            }
            rows.extend(block);
        }
        rows
    }
}

/// Solves the square system `a` (n x n, row major) for `b` in place (partial pivoting).
fn gauss(a: &mut [f64], b: &mut [f64], n: usize) -> bool {
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i * n + c].abs().total_cmp(&a[j * n + c].abs())).unwrap();
        if a[p * n + c].abs() < 1e-300 {
            return false;
        }
        if p != c {
            for k in 0..n {
                a.swap(p * n + k, c * n + k);
            }
            b.swap(p, c);
        }
        let piv = a[c * n + c];
        for i in (c + 1)..n {
            let f = a[i * n + c] / piv;
            if f != 0.0 {
                for k in c..n {
                    a[i * n + k] -= f * a[c * n + k];
                }
                b[i] -= f * b[c];
            }
        }
    }
    for c in (0..n).rev() {
        let mut s = b[c];
        for k in (c + 1)..n {
            s -= a[c * n + k] * b[k];
        }
        b[c] = s / a[c * n + c];
    }
    b.iter().all(|x| x.is_finite())
}

fn max_abs(r: &[f64]) -> f64 {
    r.iter().fold(0.0, |m, x| m.max(x.abs()))
}

/// A constraint holds when its equations are this close to zero (mm).
const HOLD_TOL: f64 = 1e-7;
const HEAVY: f64 = 1e4;

impl Sketch {
    /// Keeps constraint references valid after element `i` was removed (constraints on it go).
    pub fn dims_after_remove(&mut self, i: usize) {
        let dims = std::mem::take(&mut self.dims);
        for mut d in dims {
            let shift = |r: Option<GRef>| -> Option<Option<GRef>> {
                match r {
                    None => Some(None),
                    Some(r) => r.shifted_after_remove(i).map(Some),
                }
            };
            let (Some(a), Some(b), Some(c)) = (d.a.shifted_after_remove(i), shift(d.b), shift(d.c)) else { continue };
            d.a = a;
            d.b = b;
            d.c = c;
            self.dims.push(d);
        }
    }

    pub fn form(&self, r: &GRef) -> Option<Form> {
        form(&self.shapes, r)
    }

    /// The signs of a constraint; a tangency also learns whether the two elements are joined
    /// at an end (then it is solved as "square to the radius at that end", which stays well
    /// behaved where a distance to the centre would not).
    fn signs_of(&self, d: &Dim) -> [f64; 3] {
        let mut sg = signs(&self.shapes, d);
        if d.kind != DimKind::Tangent {
            return sg;
        }
        let (Some(a), Some(b)) = (d.a.elem(), d.b.and_then(|b| b.elem())) else { return sg };
        let joined = |i: usize, k: usize, j: usize| {
            self.dims.iter().any(|c| {
                c.kind == DimKind::Coincidence && !c.reference && {
                    let other = |x: &GRef| matches!(*x, GRef::Key(jj, kk) if jj == j && kk < 2) || *x == GRef::Elem(j);
                    (c.a == GRef::Key(i, k) && c.b.as_ref().map_or(false, other)) || (c.b == Some(GRef::Key(i, k)) && other(&c.a))
                }
            })
        };
        let is_line = |i: usize| matches!(self.shapes.get(i), Some(Shape::Line { .. }) | Some(Shape::Axis { .. }));
        let is_arc = |i: usize| matches!(self.shapes.get(i), Some(Shape::Arc { .. }));
        if is_line(a) || is_line(b) {
            let (l, c) = if is_line(a) { (a, b) } else { (b, a) };
            for k in 0..2 {
                if joined(l, k, c) {
                    sg[2] = k as f64 + 1.0;
                }
            }
        } else if is_arc(a) && is_arc(b) {
            for k in 0..2 {
                if joined(a, k, b) {
                    sg[2] = k as f64 + 1.0;
                }
            }
        }
        sg
    }

    /// Axes, the origin and reference elements from other sketches never move.
    pub fn locked(&self, r: &GRef) -> bool {
        r.fixed() || r.elem().map_or(false, |e| self.is_ext(e))
    }

    /// An element with a Fix constraint.
    pub fn is_fixed(&self, i: usize) -> bool {
        self.dims.iter().any(|d| d.kind == DimKind::Fix && !d.reference && d.a == GRef::Elem(i))
    }

    /// The current value of a dimension (None when it no longer makes sense or has no value).
    pub fn dim_measure(&self, d: &Dim) -> Option<f64> {
        if d.kind.is_geometric() {
            return None;
        }
        let mut probe = d.clone();
        probe.value = 0.0;
        let sg = self.signs_of(d);
        let r = residuals(&self.shapes, &probe, &sg)?;
        // every dimensional equation reads "measure - value" (angles in radians)
        let m = *r.first()?;
        Some(if d.kind == DimKind::Angle { m.to_degrees().abs() } else { m })
    }

    /// Does the constraint hold?
    pub fn dim_ok(&self, d: &Dim) -> bool {
        if d.reference {
            return true;
        }
        let sg = self.signs_of(d);
        residuals(&self.shapes, d, &sg).map_or(false, |r| max_abs(&r) < 1e-6)
    }

    /// Reference dimensions show the current measure.
    pub fn refresh_refs(&mut self) {
        for k in 0..self.dims.len() {
            if self.dims[k].reference {
                if let Some(v) = self.dim_measure(&self.dims[k]) {
                    self.dims[k].value = v.abs();
                }
            }
        }
    }

    fn system(&self, lock: &[usize], pin: &[(usize, usize)]) -> System {
        let n = self.shapes.len();
        let mut locked: HashSet<usize> = (0..n).filter(|i| self.is_ext(*i) || self.is_fixed(*i)).collect();
        locked.extend(lock.iter().copied());
        let mut vars = Vec::new();
        let mut of_shape = vec![vec![]; n];
        let scale: Vec<f64> = self.shapes.iter().map(shape_scale).collect();
        for i in 0..n {
            if locked.contains(&i) {
                continue;
            }
            for k in 0..params(&self.shapes[i], scale[i]).len() {
                if pin.contains(&(i, k)) {
                    continue;
                }
                of_shape[i].push(vars.len());
                vars.push((i, k));
            }
        }
        let mut eqs = Vec::new();
        let mut broken = Vec::new();
        for (j, d) in self.dims.iter().enumerate() {
            if d.reference || d.kind == DimKind::Fix {
                continue;
            }
            let sg = self.signs_of(d);
            match residuals(&self.shapes, d, &sg) {
                Some(r) if !r.is_empty() => {
                    let deps: Vec<usize> = d.elems().iter().flat_map(|e| of_shape.get(*e).cloned().unwrap_or_default()).collect();
                    eqs.push((j, sg, deps, r.len()));
                }
                Some(_) => {}
                None => broken.push(j),
            }
        }
        System { vars, of_shape, scale, eqs, broken }
    }

    /// Makes every constraint hold, changing the geometry as little as possible.
    /// On failure the sketch is left as it was and false is returned.
    pub fn solve(&mut self) -> bool {
        self.solve_with(&SolveOpts::default())
    }

    /// Solves after dimension `di` got a new value: the item picked first (for a length, the
    /// start point; for a radius, the centre) rather stays, the other side moves.
    pub fn solve_edit(&mut self, di: usize) -> bool {
        let mut o = SolveOpts::default();
        if let Some(d) = self.dims.get(di) {
            if let Some(i) = d.a.elem() {
                let np = self.shapes.get(i).map_or(0, |s| params(s, 1.0).len());
                let ks: Vec<usize> = match d.kind {
                    DimKind::Length | DimKind::Radius | DimKind::Diameter | DimKind::MajorAxis | DimKind::MinorAxis => vec![0, 1],
                    _ if d.b.is_some() => (0..np).collect(),
                    _ => vec![],
                };
                o.heavy = ks.into_iter().filter(|k| *k < np).map(|k| (i, k)).collect();
            }
        }
        self.solve_with(&o)
    }

    /// Solves keeping `opts.heavy` exactly where they are when the constraints allow it, else
    /// moving them as little as possible.
    pub fn solve_with(&mut self, opts: &SolveOpts) -> bool {
        if !opts.heavy.is_empty() && self.solve_pinned(opts, true) {
            return true;
        }
        self.solve_pinned(opts, false)
    }

    fn solve_pinned(&mut self, opts: &SolveOpts, pin: bool) -> bool {
        let sys = self.system(&opts.lock, if pin { &opts.heavy } else { &[] });
        if !sys.broken.is_empty() {
            return false;
        }
        let mut sh = self.shapes.clone();
        let mut x = sys.gather(&sh);
        let w: Vec<f64> = sys.vars.iter().map(|v| if opts.heavy.contains(v) { HEAVY } else { 1.0 }).collect();
        let mut r = sys.eval(&sh, &self.dims);
        let m = r.len();
        let mut lambda = 1e-9;
        for _ in 0..100 {
            if max_abs(&r) < 1e-11 || m == 0 {
                break;
            }
            let jac = sys.jacobian(&mut sh, &self.dims, &x);
            // A = J W^-1 J^T
            let n = x.len();
            let mut dense = vec![0.0; n];
            let mut a = vec![0.0; m * m];
            for p in 0..m {
                for &(k, g) in &jac[p] {
                    dense[k] = g / w[k];
                }
                for q in p..m {
                    let s: f64 = jac[q].iter().map(|&(k, g)| dense[k] * g).sum();
                    a[p * m + q] = s;
                    a[q * m + p] = s;
                }
                for &(k, _) in &jac[p] {
                    dense[k] = 0.0;
                }
            }
            let f0: f64 = r.iter().map(|v| v * v).sum();
            let mut improved = false;
            while lambda < 1e9 {
                let mut aa = a.clone();
                for i in 0..m {
                    aa[i * m + i] += lambda * a[i * m + i].max(1e-9);
                }
                let mut y = r.clone();
                if !gauss(&mut aa, &mut y, m) {
                    lambda *= 10.0;
                    continue;
                }
                let mut xn = x.clone();
                for p in 0..m {
                    for &(k, g) in &jac[p] {
                        xn[k] -= g * y[p] / w[k];
                    }
                }
                let mut shn = sh.clone();
                sys.write(&mut shn, &xn);
                let rn = sys.eval(&shn, &self.dims);
                let f1: f64 = rn.iter().map(|v| v * v).sum();
                if f1 < f0 {
                    x = xn;
                    sh = shn;
                    r = rn;
                    lambda = (lambda / 10.0).max(1e-15);
                    improved = true;
                    break;
                }
                lambda *= 10.0;
            }
            if !improved {
                break;
            }
        }
        if max_abs(&r) < HOLD_TOL {
            self.shapes = sh;
            self.refresh_refs();
            true
        } else {
            false
        }
    }

    /// Which elements are free, fully constrained or over-constrained, and which constraints
    /// are redundant or cannot hold.
    pub fn analyze(&self) -> SkStatus {
        let n = self.shapes.len();
        let mut elems = vec![CState::Under; n];
        let mut dims = vec![CState::Iso; self.dims.len()];
        let sys = self.system(&[], &[]);
        let mut sh = self.shapes.clone();
        let x = sys.gather(&sh);
        let r = sys.eval(&sh, &self.dims);
        let jac = sys.jacobian(&mut sh, &self.dims, &x);
        let nv = x.len();
        for &j in &sys.broken {
            dims[j] = CState::Bad;
        }
        // the equation rows of each constraint
        let mut row = 0;
        let mut owner = Vec::new();
        for (j, _, _, c) in &sys.eqs {
            if max_abs(&r[row..row + c]) > 1e-6 {
                dims[*j] = CState::Bad;
            }
            owner.extend(std::iter::repeat(*j).take(*c));
            row += c;
        }
        // independent equations, in order: a row that the earlier ones already decide is
        // redundant (the constraint over-constrains)
        let mut basis: Vec<(Vec<f64>, usize)> = Vec::new();
        for (p, jr) in jac.iter().enumerate() {
            let mut v = vec![0.0; nv];
            for &(k, g) in jr {
                v[k] = g;
            }
            let s = max_abs(&v);
            if s < 1e-12 {
                // an equation on locked things only: it either holds or is inconsistent
                if dims[owner[p]] == CState::Iso {
                    dims[owner[p]] = CState::Over;
                }
                continue;
            }
            for x in v.iter_mut() {
                *x /= s;
            }
            for (b, piv) in &basis {
                let f = v[*piv];
                if f != 0.0 {
                    for k in 0..nv {
                        v[k] -= f * b[k];
                    }
                }
            }
            let (piv, m) = v.iter().enumerate().fold((0, 0.0), |acc, (k, x)| if x.abs() > acc.1 { (k, x.abs()) } else { acc });
            if m < 1e-7 {
                if dims[owner[p]] == CState::Iso {
                    dims[owner[p]] = CState::Over;
                }
                continue;
            }
            let f = v[piv];
            for x in v.iter_mut() {
                *x /= f;
            }
            basis.push((v, piv));
        }
        // reduced row echelon form: an unknown is decided when no free unknown moves it
        for i in 0..basis.len() {
            let (bi, pi) = basis[i].clone();
            for (j, (bj, _)) in basis.iter_mut().enumerate() {
                if j != i && bj[pi] != 0.0 {
                    let f = bj[pi];
                    for k in 0..nv {
                        bj[k] -= f * bi[k];
                    }
                }
            }
        }
        let pivots: HashSet<usize> = basis.iter().map(|b| b.1).collect();
        let free: Vec<usize> = (0..nv).filter(|k| !pivots.contains(k)).collect();
        let mut decided = vec![false; nv];
        for (b, piv) in &basis {
            decided[*piv] = free.iter().all(|f| b[*f].abs() < 1e-7);
        }
        for i in 0..n {
            let vs = &sys.of_shape[i];
            elems[i] = if vs.iter().all(|v| decided[*v]) { CState::Iso } else { CState::Under };
        }
        for (j, d) in self.dims.iter().enumerate() {
            if matches!(dims[j], CState::Over | CState::Bad) {
                for e in d.elems() {
                    if e < n && !self.is_ext(e) && elems[e] != CState::Bad {
                        elems[e] = dims[j];
                    }
                }
            }
        }
        SkStatus { elems, dims }
    }

    // ------------------------------------------------------------ drawing geometry

    /// Anchor point of a dimension (its text sits at anchor + offset).
    pub fn dim_anchor(&self, d: &Dim) -> Option<DVec2> {
        match self.dim_geom_raw(d, DVec2::ZERO)? {
            DimGeom::Linear { p, q, .. } => Some((p + q) * 0.5),
            DimGeom::Radial { c, .. } => Some(c),
            DimGeom::Angular { v, .. } => Some(v),
            DimGeom::Tag { at, .. } => Some(at),
        }
    }

    pub fn dim_geom(&self, d: &Dim) -> Option<DimGeom> {
        self.dim_geom_raw(d, d.offset)
    }

    fn dim_geom_raw(&self, d: &Dim, offset: DVec2) -> Option<DimGeom> {
        let sh = &self.shapes[..];
        let shape = d.a.elem().and_then(|i| sh.get(i));
        let linear = |p: DVec2, q: DVec2| -> DimGeom {
            let t = q - p;
            let n = if t.length() > 1e-12 { perp(t.normalize()) } else { DVec2::Y };
            let o = n * offset.dot(n);
            DimGeom::Linear { p, q, a: p + o, b: q + o, n, text: (p + q) * 0.5 + offset }
        };
        let tag = |at: DVec2, also: Option<DVec2>| Some(DimGeom::Tag { at: at + offset, also });
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
            DimKind::MajorAxis | DimKind::MinorAxis => {
                let (c, a, b, rot) = ellipse(sh, &d.a)?;
                let dir = if d.kind == DimKind::MajorAxis { DVec2::new(rot.cos(), rot.sin()) * a } else { DVec2::new(-rot.sin(), rot.cos()) * b };
                Some(linear(c, c + dir))
            }
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
                let b = d.b?;
                if d.dir != MeasDir::Normal {
                    let (p, q) = (point(sh, &d.a).or_else(|| centre(sh, &d.a))?, point(sh, &b).or_else(|| centre(sh, &b))?);
                    let mid = (p + q) * 0.5;
                    let (a, bb, n) = if d.dir == MeasDir::Horizontal {
                        let y = mid.y + offset.y;
                        (DVec2::new(p.x, y), DVec2::new(q.x, y), DVec2::Y)
                    } else {
                        let x = mid.x + offset.x;
                        (DVec2::new(x, p.y), DVec2::new(x, q.y), DVec2::X)
                    };
                    return Some(DimGeom::Linear { p, q, a, b: bb, n, text: mid + offset });
                }
                let (fa, fb) = (form(sh, &d.a)?, form(sh, &b)?);
                let (p, q) = match (fa, fb) {
                    (Form::Pt(p), Form::Pt(q)) => (p, q),
                    (Form::Pt(p), Form::Ln(o, dir)) => (p, o + dir * (p - o).dot(dir)),
                    (Form::Ln(o, dir), Form::Pt(q)) => (o + dir * (q - o).dot(dir), q),
                    (Form::Ln(..), Form::Ln(..)) => {
                        let (o, dir, m) = line_pair(sh, &d.a, &b)?;
                        (o + dir * (m - o).dot(dir), m)
                    }
                };
                Some(linear(p, q))
            }
            DimKind::Angle => {
                let (v, d1, d2) = angle_frame(sh, &d.a, d.b.as_ref()?)?;
                Some(DimGeom::Angular { v, d1, d2, r: offset.length().max(1e-6), text: v + offset })
            }
            DimKind::Horizontal | DimKind::Vertical | DimKind::Fix => tag(elem_mid(sh, &d.a)?, None),
            DimKind::Coincidence | DimKind::Concentric | DimKind::Midpoint => {
                let b = d.b?;
                let at = point(sh, &d.a).or_else(|| point(sh, &b)).or_else(|| centre(sh, &b)).or_else(|| elem_mid(sh, &b))?;
                tag(at, None)
            }
            DimKind::Tangent => {
                let b = d.b?;
                let at = if let Some(((o, dir), (c, _))) = line_and_circle(sh, &d.a, &b) {
                    o + dir * (c - o).dot(dir)
                } else {
                    let ((c1, r1), (c2, _)) = (circ(sh, &d.a)?, circ(sh, &b)?);
                    c1 + (c2 - c1).normalize_or_zero() * r1
                };
                tag(at, None)
            }
            DimKind::Parallel | DimKind::Perpendicular | DimKind::Symmetry => {
                let b = d.b?;
                tag(elem_mid(sh, &b)?, elem_mid(sh, &d.a).filter(|_| !d.a.fixed()))
            }
            DimKind::Equidistant => tag(point(sh, &d.a)?, None),
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
            Shape::Ellipse { cx, cy, rot, .. } => {
                let rel = place - DVec2::new(*cx, *cy);
                let ax = DVec2::new(rot.to_radians().cos(), rot.to_radians().sin());
                if rel.dot(ax).abs() >= rel.perp_dot(ax).abs() { DimKind::MajorAxis } else { DimKind::MinorAxis }
            }
            Shape::Rect { cx, cy, w, h, .. } => {
                let rel = place - DVec2::new(*cx, *cy);
                if rel.y.abs() / h.abs().max(1e-9) > rel.x.abs() / w.abs().max(1e-9) { DimKind::Width } else { DimKind::Height }
            }
            _ => return None,
        };
        let mut d = Dim::new(kind, GRef::Elem(i), None, 0.0);
        d.value = self.dim_measure(&d)?.abs();
        let anchor = self.dim_anchor(&d)?;
        d.offset = place - anchor;
        Some(d)
    }

    /// The dimension between two references: an angle for two lines that are not parallel,
    /// otherwise a distance (measured as `dir` says).
    pub fn pair_dim_for(&self, a: GRef, b: GRef, place: DVec2) -> Option<Dim> {
        let both_lines = matches!((self.form(&a), self.form(&b)), (Some(Form::Ln(_, da)), Some(Form::Ln(_, db))) if da.perp_dot(db).abs() > 1e-6);
        let kind = if both_lines { DimKind::Angle } else { DimKind::Distance };
        let mut d = Dim::new(kind, a, Some(b), 0.0);
        d.value = self.dim_measure(&d)?.abs();
        let anchor = self.dim_anchor(&d)?;
        d.offset = place - anchor;
        if kind == DimKind::Angle && d.offset.length() < 1e-6 {
            d.offset = DVec2::new(5.0, 5.0);
        }
        Some(d)
    }

    /// The constraint of `kind` on the picked items (in the order picked), with the current
    /// value for a dimension, or None when it does not apply to them.
    pub fn make_constraint(&self, kind: DimKind, picks: &[GRef]) -> Option<Dim> {
        let mut d = match (kind, picks.len()) {
            (DimKind::Symmetry, 3) => {
                // the symmetry line is whichever pick is straight (preferably the last)
                let k = (0..3).rev().find(|k| line(&self.shapes, &picks[*k]).is_some())?;
                let rest: Vec<GRef> = (0..3).filter(|i| *i != k).map(|i| picks[i]).collect();
                let mut d = Dim::new(kind, rest[0], Some(rest[1]), 0.0);
                d.c = Some(picks[k]);
                d
            }
            (DimKind::Equidistant, 3) => {
                let mut d = Dim::new(kind, picks[0], Some(picks[1]), 0.0);
                d.c = Some(picks[2]);
                d
            }
            (_, 1) => Dim::new(kind, picks[0], None, 0.0),
            (_, 2) => Dim::new(kind, picks[0], Some(picks[1]), 0.0),
            _ => return None,
        };
        if kind == DimKind::Fix {
            return matches!(d.a, GRef::Elem(i) if i < self.shapes.len() && !self.is_ext(i)).then_some(d);
        }
        if kind == DimKind::Distance {
            // distances need two different things that can be measured
            if d.a == d.b? || matches!((self.form(&d.a)?, self.form(&d.b?)?), (Form::Ln(_, x), Form::Ln(_, y)) if x.perp_dot(y).abs() > 1e-6) {
                return None;
            }
        }
        if kind == DimKind::Angle && line_n(&self.shapes, &d.a).is_none() {
            return None;
        }
        if !kind.is_geometric() {
            d.value = self.dim_measure(&d)?.abs();
        }
        let sg = signs(&self.shapes, &d);
        residuals(&self.shapes, &d, &sg)?;
        if kind == DimKind::Angle {
            angle_frame(&self.shapes, &d.a, d.b.as_ref()?)?;
            d.offset = DVec2::new(5.0, 5.0);
        }
        Some(d)
    }

    /// What Contact Constraint makes of two items: concentricity for two round ones, tangency
    /// for a line and a round one, otherwise coincidence.
    pub fn contact_kind(&self, a: &GRef, b: &GRef) -> DimKind {
        let round = |r: &GRef| circ(&self.shapes, r).is_some() || ellipse(&self.shapes, r).is_some();
        if round(a) && round(b) {
            DimKind::Concentric
        } else if line_and_circle(&self.shapes, a, b).is_some() {
            DimKind::Tangent
        } else {
            DimKind::Coincidence
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{signed_area, Frame};

    fn line(a: (f64, f64), b: (f64, f64)) -> Shape {
        Shape::Line { x1: a.0, y1: a.1, x2: b.0, y2: b.1 }
    }

    fn rect(sk: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) {
        sk.push(line((x0, y0), (x1, y0)), false);
        sk.push(line((x1, y0), (x1, y1)), false);
        sk.push(line((x1, y1), (x0, y1)), false);
        sk.push(line((x0, y1), (x0, y0)), false);
    }

    #[test]
    fn rectangle_gets_smartpick_constraints() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        rect(&mut sk, 5.0, 5.0, 25.0, 15.0);
        let count = |k: DimKind| sk.dims.iter().filter(|d| d.kind == k).count();
        assert_eq!(count(DimKind::Horizontal), 2);
        assert_eq!(count(DimKind::Vertical), 2);
        assert_eq!(count(DimKind::Coincidence), 4);
        // four lines, four corners joined, H / V: position and size are still free
        let st = sk.analyze();
        assert!(st.elems.iter().all(|e| *e == CState::Under));
        assert!(st.dims.iter().all(|d| *d == CState::Iso));
        // width, height and position from the axes: fully constrained
        sk.dims.push(Dim::new(DimKind::Length, GRef::Elem(0), None, 20.0));
        sk.dims.push(Dim::new(DimKind::Length, GRef::Elem(1), None, 10.0));
        sk.dims.push(Dim::new(DimKind::Distance, GRef::AxisV, Some(GRef::Elem(3)), 5.0));
        sk.dims.push(Dim::new(DimKind::Distance, GRef::AxisH, Some(GRef::Elem(0)), 5.0));
        let st = sk.analyze();
        assert!(st.elems.iter().all(|e| *e == CState::Iso), "{:?}", st.elems);
        // one more (the top's length) is redundant: purple
        sk.dims.push(Dim::new(DimKind::Length, GRef::Elem(2), None, 20.0));
        let st = sk.analyze();
        assert_eq!(*st.dims.last().unwrap(), CState::Over);
        assert_eq!(st.elems[2], CState::Over);
        // and an impossible one is refused
        sk.dims.last_mut().unwrap().value = 30.0;
        let before = sk.shapes.clone();
        assert!(!sk.solve());
        assert_eq!(sk.shapes, before, "a failed solve changes nothing");
    }

    #[test]
    fn length_keeps_rectangle_square() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        rect(&mut sk, 0.0, 0.0, 20.0, 10.0);
        let mut d = sk.single_dim_for(0, DVec2::new(10.0, -5.0)).unwrap();
        assert_eq!(d.kind, DimKind::Length);
        assert!((d.value - 20.0).abs() < 1e-9);
        d.value = 35.0;
        sk.dims.push(d);
        assert!(sk.solve_edit(sk.dims.len() - 1));
        let (s, _) = sk.regions().unwrap();
        assert!((signed_area(&s[0]) - 350.0).abs() < 1e-6, "{}", signed_area(&s[0]));
        // the start stayed (at the origin), the right side moved and stayed vertical
        if let Shape::Line { x1, x2, .. } = sk.shapes[1] {
            assert!((x1 - 35.0).abs() < 1e-7 && (x2 - 35.0).abs() < 1e-7, "{} {}", x1, x2);
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
        assert!(sk.solve());
        if let Shape::Circle { cx, cy, r } = sk.shapes[0] {
            assert!((cx - 25.0).abs() < 1e-7 && cy.abs() < 1e-7 && (r - 4.0).abs() < 1e-7, "{} {} {}", cx, cy, r);
        }
        // angle between two lines meeting at the origin
        sk.push(line((0.0, 0.0), (10.0, 0.0)), false);
        sk.push(line((0.0, 0.0), (10.0, 10.0)), false);
        let mut a = sk.pair_dim_for(GRef::Elem(1), GRef::Elem(2), DVec2::new(4.0, 2.0)).unwrap();
        assert_eq!(a.kind, DimKind::Angle);
        assert!((a.value - 45.0).abs() < 1e-9);
        a.value = 60.0;
        sk.dims.push(a);
        assert!(sk.solve_edit(sk.dims.len() - 1));
        assert!(sk.dims.iter().all(|d| sk.dim_ok(d)));
        if let (Shape::Line { x1, y1, x2, y2 }, Shape::Line { x2: bx, y2: by, .. }) = (&sk.shapes[2], &sk.shapes[1]) {
            assert!(x1.abs() < 1e-7 && y1.abs() < 1e-7, "the corner stays at the origin");
            assert!(by.abs() < 1e-7 && *bx > 0.0, "the first line stays");
            let ang = y2.atan2(*x2).to_degrees();
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
            sk.dims.push(Dim::new(DimKind::Distance, GRef::AxisH, Some(GRef::Elem(0)), 10.0));
            // distance between the lines, clicked in either order
            let (a, b) = if order { (GRef::Elem(1), GRef::Elem(0)) } else { (GRef::Elem(0), GRef::Elem(1)) };
            sk.dims.push(Dim::new(DimKind::Distance, a, Some(b), 25.0));
            assert!(sk.solve_edit(sk.dims.len() - 1), "order {}", order);
            assert!(sk.dims.iter().all(|d| sk.dim_ok(d)), "order {}", order);
            if let (Shape::Line { y1: a, .. }, Shape::Line { y1: b, .. }) = (&sk.shapes[0], &sk.shapes[1]) {
                assert!((a - 10.0).abs() < 1e-7 && (b - 35.0).abs() < 1e-7, "order {}: {} {}", order, a, b);
            }
        }
    }

    #[test]
    fn fixed_line_stays_put() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        rect(&mut sk, 0.0, 10.0, 30.0, 20.0);
        assert_eq!(sk.dims.iter().filter(|d| matches!(d.kind, DimKind::Horizontal | DimKind::Vertical)).count(), 4);
        // fix the bottom line: length, height above H axis, its left end from the V axis
        sk.dims.push(Dim::new(DimKind::Length, GRef::Elem(0), None, 30.0));
        sk.dims.push(Dim::new(DimKind::Distance, GRef::AxisH, Some(GRef::Elem(0)), 10.0));
        sk.dims.push(Dim::new(DimKind::Distance, GRef::AxisV, Some(GRef::Key(0, 0)), 0.0));
        let st = sk.analyze();
        assert_eq!(st.elems[0], CState::Iso, "bottom line should count as fixed");
        assert_eq!(st.elems[2], CState::Under, "top line is still free");
        // distance between the lines, picked top first (so the top is the one that rather stays)
        sk.dims.push(Dim::new(DimKind::Distance, GRef::Elem(2), Some(GRef::Elem(0)), 25.0));
        assert!(sk.solve_edit(sk.dims.len() - 1));
        let ys = |i: usize| match sk.shapes[i] {
            Shape::Line { y1, y2, .. } => (y1, y2),
            _ => (0.0, 0.0),
        };
        assert!((ys(0).0 - 10.0).abs() < 1e-7 && (ys(0).1 - 10.0).abs() < 1e-7, "the fixed line must not move");
        assert!((ys(2).0 - 35.0).abs() < 1e-7 && (ys(2).1 - 35.0).abs() < 1e-7, "the free line moves");
        let (s2, _) = sk.regions().unwrap();
        assert!((crate::geom::signed_area(&s2[0]) - 750.0).abs() < 1e-5);
        assert!(sk.analyze().elems.iter().all(|e| *e == CState::Iso));
    }

    #[test]
    fn geometric_constraints() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        let l0 = sk.push(line((0.0, 0.0), (20.0, 1.0)), false);
        let l1 = sk.push(line((0.0, 10.0), (20.0, 14.0)), false);
        let c = sk.push(Shape::Circle { cx: 30.0, cy: 5.0, r: 3.0 }, false);
        let c2 = sk.push(Shape::Circle { cx: 31.0, cy: 6.0, r: 1.0 }, false);
        for d in [
            sk.make_constraint(DimKind::Parallel, &[GRef::Elem(l0), GRef::Elem(l1)]).unwrap(),
            sk.make_constraint(DimKind::Tangent, &[GRef::Elem(l1), GRef::Elem(c)]).unwrap(),
            sk.make_constraint(DimKind::Concentric, &[GRef::Elem(c), GRef::Elem(c2)]).unwrap(),
            sk.make_constraint(DimKind::Fix, &[GRef::Elem(l0)]).unwrap(),
        ] {
            sk.dims.push(d);
        }
        assert!(sk.solve());
        let (Shape::Line { x1, y1, x2, y2 }, Shape::Line { x1: a1, y1: b1, x2: a2, y2: b2 }) = (sk.shapes[l0].clone(), sk.shapes[l1].clone()) else { panic!() };
        assert_eq!((x1, y1, x2, y2), (0.0, 0.0, 20.0, 1.0), "fixed");
        let (d0, d1) = (DVec2::new(x2 - x1, y2 - y1).normalize(), DVec2::new(a2 - a1, b2 - b1).normalize());
        assert!(d0.perp_dot(d1).abs() < 1e-7, "parallel");
        if let (Shape::Circle { cx, cy, r }, Shape::Circle { cx: qx, cy: qy, .. }) = (&sk.shapes[c], &sk.shapes[c2]) {
            let dist = d1.perp_dot(DVec2::new(*cx, *cy) - DVec2::new(a1, b1)).abs();
            assert!((dist - r).abs() < 1e-7, "tangent");
            assert!((cx - qx).abs() < 1e-7 && (cy - qy).abs() < 1e-7, "concentric");
        }
        // perpendicular, midpoint and symmetry
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        let a = sk.push(line((0.0, 0.0), (10.0, 2.0)), false);
        let b = sk.push(line((12.0, -5.0), (13.0, 5.0)), false);
        let p = sk.push(Shape::Point { x: 3.0, y: 4.0 }, false);
        let q = sk.push(Shape::Point { x: -2.0, y: 7.0 }, false);
        sk.dims.push(sk.make_constraint(DimKind::Perpendicular, &[GRef::Elem(a), GRef::Elem(b)]).unwrap());
        sk.dims.push(sk.make_constraint(DimKind::Midpoint, &[GRef::Elem(p), GRef::Elem(b)]).unwrap());
        sk.dims.push(sk.make_constraint(DimKind::Symmetry, &[GRef::Elem(p), GRef::Elem(q), GRef::AxisV]).unwrap());
        assert!(sk.solve());
        assert!(sk.dims.iter().all(|d| sk.dim_ok(d)));
        let pt = |i: usize| match sk.shapes[i] {
            Shape::Point { x, y } => DVec2::new(x, y),
            _ => panic!(),
        };
        assert!((pt(p).x + pt(q).x).abs() < 1e-7 && (pt(p).y - pt(q).y).abs() < 1e-7, "mirror images across V");
    }

    #[test]
    fn dragging_respects_constraints() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        rect(&mut sk, 0.0, 0.0, 20.0, 10.0);
        // drag the top right corner (end of line 1) to (30, 25): the rectangle stretches
        sk.shapes[1].set_handle(1, DVec2::new(30.0, 25.0));
        let opts = SolveOpts { heavy: handle_params(&sk.shapes[1], 1).into_iter().map(|k| (1, k)).collect(), lock: vec![] };
        assert!(sk.solve_with(&opts));
        let (s, _) = sk.regions().unwrap();
        let area = signed_area(&s[0]).abs();
        assert!((area - 750.0).abs() < 1e-3, "{}", area);
    }

    /// Number of free unknowns (degrees of freedom) left in a sketch.
    fn dof(sk: &Sketch) -> usize {
        let sys = sk.system(&[], &[]);
        let mut sh = sk.shapes.clone();
        let x = sys.gather(&sh);
        let jac = sys.jacobian(&mut sh, &sk.dims, &x);
        // rank by elimination
        let n = x.len();
        let mut basis: Vec<(Vec<f64>, usize)> = vec![];
        for jr in jac {
            let mut v = vec![0.0; n];
            for (k, g) in jr {
                v[k] = g;
            }
            for (b, p) in &basis {
                let f = v[*p];
                for k in 0..n {
                    v[k] -= f * b[k];
                }
            }
            let (p, m) = v.iter().enumerate().fold((0, 0.0), |a, (k, x)| if x.abs() > a.1 { (k, x.abs()) } else { a });
            if m > 1e-7 {
                let f = v[p];
                basis.push((v.iter().map(|x| x / f).collect(), p));
            }
        }
        n - basis.len()
    }

    #[test]
    fn composites_like_classic_cad() {
        let check = |sk: &Sketch, free: usize, what: &str| {
            let st = sk.analyze();
            assert!(st.dims.iter().all(|d| *d == CState::Iso), "{}: {:?}", what, st.dims);
            assert_eq!(dof(sk), free, "{}", what);
            assert!(sk.regions().is_ok(), "{}", what);
        };
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.add_rectangle(DVec2::new(5.0, 5.0), DVec2::new(25.0, 15.0), false, false, true);
        check(&sk, 4, "rectangle: position and size");
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.add_rectangle(DVec2::new(-10.0, -5.0), DVec2::new(10.0, 5.0), true, false, true);
        check(&sk, 2, "centred rectangle on the origin: size only");
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.add_polygon(DVec2::new(3.0, 4.0), 10.0, 6, 0.0, false, true);
        // a level side gets H: centre and size are free
        check(&sk, 3, "hexagon");
        // the hexagon stays regular when its circle grows
        let outer = sk.shapes.iter().position(|s| matches!(s, Shape::Circle { .. })).unwrap();
        sk.dims.push(Dim::new(DimKind::Radius, GRef::Elem(outer), None, 15.0));
        assert!(sk.solve_edit(sk.dims.len() - 1));
        let lens: Vec<f64> = (0..6).map(|i| sk.dim_measure(&Dim::new(DimKind::Length, GRef::Elem(i), None, 0.0)).unwrap()).collect();
        assert!(lens.iter().all(|l| (l - 15.0).abs() < 1e-6), "{:?}", lens);
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.add_slot(DVec2::new(0.0, 10.0), DVec2::new(20.0, 15.0), 3.0, false, true);
        check(&sk, 5, "elongated hole: two centres and the width");
        // corner and mirror keep the profile consistent
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.add_rectangle(DVec2::new(5.0, 5.0), DVec2::new(25.0, 15.0), false, false, true);
        crate::sketch::corner(&mut sk, DVec2::new(25.0, 15.0), 2.0, 0.5).unwrap();
        check(&sk, 5, "rounded corner: radius is free");
        let ax = sk.push(Shape::Axis { x1: 0.0, y1: -10.0, x2: 0.0, y2: 30.0 }, false);
        sk.dims.push(Dim::new(DimKind::Vertical, GRef::Elem(ax), None, 0.0));
        let n = sk.shapes.len();
        crate::sketch::mirror(&mut sk, &(0..n - 1).collect::<Vec<_>>(), DVec2::ZERO, DVec2::Y, Some(GRef::Elem(ax)));
        let st = sk.analyze();
        assert!(st.dims.iter().all(|d| *d != CState::Bad), "{:?}", st.dims);
        assert!(sk.solve());
    }

    #[test]
    fn older_sketches_get_coincidences() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        for (a, b) in [((0.0, 0.0), (10.0, 0.0)), ((10.0, 0.0), (10.0, 10.0)), ((10.0, 10.0), (0.0, 0.0))] {
            sk.push_raw(line(a, b), false);
        }
        sk.version = 0;
        sk.upgrade();
        assert_eq!(sk.dims.iter().filter(|d| d.kind == DimKind::Coincidence).count(), 3);
        // a length now drags the connected lines along
        sk.dims.push(Dim::new(DimKind::Length, GRef::Elem(0), None, 20.0));
        assert!(sk.solve_edit(sk.dims.len() - 1));
        assert!(sk.regions().is_ok());
    }
}
