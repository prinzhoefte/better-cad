//! Sketch geometry: individual elements (lines, arcs, circles, splines, ...) that connect at
//! their end points into closed profiles, plus the sketch operations (quick trim, corner,
//! chamfer, mirror, explode).

use crate::geom::*;
use glam::DVec2;
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// How close two end points must be to count as connected (mm).
pub const JOIN_TOL: f64 = 1e-3;

#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
#[serde(tag = "type")]
pub enum Shape {
    Rect { cx: f64, cy: f64, w: f64, h: f64, r: f64 },
    Circle { cx: f64, cy: f64, r: f64 },
    Polygon { cx: f64, cy: f64, r: f64, n: f64, rot: f64 },
    Slot { x1: f64, y1: f64, x2: f64, y2: f64, r: f64 },
    Profile { pts: Vec<DVec2> },
    Line { x1: f64, y1: f64, x2: f64, y2: f64 },
    /// counter-clockwise from a0 to a1 (degrees)
    Arc { cx: f64, cy: f64, r: f64, a0: f64, a1: f64 },
    Ellipse { cx: f64, cy: f64, a: f64, b: f64, rot: f64 },
    Spline { pts: Vec<DVec2>, closed: bool },
    /// rotation axis for Shaft / Groove (never part of a profile)
    Axis { x1: f64, y1: f64, x2: f64, y2: f64 },
}

fn arc_pts(c: DVec2, r: f64, a0: f64, a1: f64, out: &mut Vec<DVec2>) {
    let segs = ((circle_segments(r) as f64) * ((a1 - a0).abs() / TAU)).ceil().max(2.0) as usize;
    for i in 0..=segs {
        let a = a0 + (a1 - a0) * i as f64 / segs as f64;
        out.push(c + DVec2::new(a.cos(), a.sin()) * r);
    }
}

pub fn sweep_deg(a0: f64, a1: f64) -> f64 {
    let s = (a1 - a0).rem_euclid(360.0);
    if s < 1e-9 { 360.0 } else { s }
}

fn catmull(p0: DVec2, p1: DVec2, p2: DVec2, p3: DVec2, t: f64) -> DVec2 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * ((2.0 * p1) + (p2 - p0) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 + (3.0 * p1 - p0 - 3.0 * p2 + p3) * t3)
}

pub fn spline_pts(pts: &[DVec2], closed: bool) -> Vec<DVec2> {
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let get = |i: isize| -> DVec2 {
        if closed {
            pts[i.rem_euclid(n as isize) as usize]
        } else {
            pts[i.clamp(0, n as isize - 1) as usize]
        }
    };
    let segs = if closed { n } else { n - 1 };
    let mut out = Vec::new();
    for s in 0..segs {
        let i = s as isize;
        for k in 0..16 {
            out.push(catmull(get(i - 1), get(i), get(i + 1), get(i + 2), k as f64 / 16.0));
        }
    }
    if !closed {
        out.push(pts[n - 1]);
    }
    out
}

impl Shape {
    pub fn kind_name(&self) -> &'static str {
        match self {
            Shape::Rect { .. } => "Rectangle",
            Shape::Circle { .. } => "Circle",
            Shape::Polygon { .. } => "Polygon",
            Shape::Slot { .. } => "Elongated Hole",
            Shape::Profile { .. } => "Profile",
            Shape::Line { .. } => "Line",
            Shape::Arc { .. } => "Arc",
            Shape::Ellipse { .. } => "Ellipse",
            Shape::Spline { .. } => "Spline",
            Shape::Axis { .. } => "Axis",
        }
    }

    /// Open elements are joined end to end into profiles.
    pub fn is_open(&self) -> bool {
        matches!(self, Shape::Line { .. } | Shape::Arc { .. } | Shape::Axis { .. } | Shape::Spline { closed: false, .. })
    }

    /// The element as a polyline: from start to end for open ones, a loop for closed ones.
    pub fn curve(&self) -> Vec<DVec2> {
        match self {
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => vec![DVec2::new(*x1, *y1), DVec2::new(*x2, *y2)],
            Shape::Arc { cx, cy, r, a0, a1 } => {
                let mut v = Vec::new();
                let s = sweep_deg(*a0, *a1);
                arc_pts(DVec2::new(*cx, *cy), r.abs(), a0.to_radians(), (a0 + s).to_radians(), &mut v);
                v
            }
            Shape::Spline { pts, closed } => spline_pts(pts, *closed),
            _ => self.outline(),
        }
    }

    pub fn endpoints(&self) -> Option<(DVec2, DVec2)> {
        if !self.is_open() {
            return None;
        }
        match self {
            Shape::Arc { cx, cy, r, a0, a1 } => {
                let c = DVec2::new(*cx, *cy);
                let a = a0.to_radians();
                let b = (a0 + sweep_deg(*a0, *a1)).to_radians();
                Some((c + DVec2::new(a.cos(), a.sin()) * *r, c + DVec2::new(b.cos(), b.sin()) * *r))
            }
            Shape::Spline { pts, .. } => Some((pts[0], *pts.last().unwrap())),
            _ => {
                let c = self.curve();
                Some((c[0], *c.last().unwrap()))
            }
        }
    }

    /// Closed outline, counter-clockwise (closed shapes only).
    pub fn outline(&self) -> Vec<DVec2> {
        let mut out = Vec::new();
        match self {
            Shape::Rect { cx, cy, w, h, r } => {
                let (hw, hh) = (w.abs() / 2.0, h.abs() / 2.0);
                let r = r.max(0.0).min(hw.min(hh) * 0.999);
                let c = DVec2::new(*cx, *cy);
                if r < 1e-6 {
                    out = vec![c + DVec2::new(-hw, -hh), c + DVec2::new(hw, -hh), c + DVec2::new(hw, hh), c + DVec2::new(-hw, hh)];
                } else {
                    arc_pts(c + DVec2::new(hw - r, -hh + r), r, -PI / 2.0, 0.0, &mut out);
                    arc_pts(c + DVec2::new(hw - r, hh - r), r, 0.0, PI / 2.0, &mut out);
                    arc_pts(c + DVec2::new(-hw + r, hh - r), r, PI / 2.0, PI, &mut out);
                    arc_pts(c + DVec2::new(-hw + r, -hh + r), r, PI, 1.5 * PI, &mut out);
                }
            }
            Shape::Circle { cx, cy, r } => out = circle_pts(DVec2::new(*cx, *cy), r.abs()),
            Shape::Polygon { .. } => out = self.polygon_corners(),
            Shape::Slot { x1, y1, x2, y2, r } => {
                let a = DVec2::new(*x1, *y1);
                let b = DVec2::new(*x2, *y2);
                let d = b - a;
                if d.length() < 1e-9 {
                    return circle_pts(a, r.abs());
                }
                let ang = d.y.atan2(d.x);
                arc_pts(b, r.abs(), ang - PI / 2.0, ang + PI / 2.0, &mut out);
                arc_pts(a, r.abs(), ang + PI / 2.0, ang + 1.5 * PI, &mut out);
            }
            Shape::Ellipse { cx, cy, a, b, rot } => {
                let n = circle_segments(a.abs().max(b.abs()));
                let (s, c) = rot.to_radians().sin_cos();
                for i in 0..n {
                    let t = TAU * i as f64 / n as f64;
                    let p = DVec2::new(t.cos() * a.abs(), t.sin() * b.abs());
                    out.push(DVec2::new(cx + p.x * c - p.y * s, cy + p.x * s + p.y * c));
                }
            }
            Shape::Profile { pts } => out = pts.clone(),
            Shape::Spline { pts, closed: true } => out = spline_pts(pts, true),
            _ => return self.curve(),
        }
        clean_polygon(&out)
    }

    fn polygon_corners(&self) -> Vec<DVec2> {
        if let Shape::Polygon { cx, cy, r, n, rot } = self {
            let n = (n.round() as usize).clamp(3, 64);
            (0..n)
                .map(|i| {
                    let a = rot.to_radians() + TAU * i as f64 / n as f64 + PI / 2.0;
                    DVec2::new(*cx, *cy) + DVec2::new(a.cos(), a.sin()) * r.abs()
                })
                .collect()
        } else {
            vec![]
        }
    }

    pub fn translate(&mut self, d: DVec2) {
        self.map_points(&|p| p + d, false);
    }

    /// Applies a point mapping (a move or a reflection). Reflections reverse arc direction.
    pub fn map_points(&mut self, f: &dyn Fn(DVec2) -> DVec2, reflect: bool) {
        let mp = |x: &mut f64, y: &mut f64| {
            let p = f(DVec2::new(*x, *y));
            *x = p.x;
            *y = p.y;
        };
        // direction change of a reference vector tells how angles map
        let o = f(DVec2::ZERO);
        let ang_of = |deg: f64| -> f64 {
            let v = f(DVec2::new(deg.to_radians().cos(), deg.to_radians().sin())) - o;
            v.y.atan2(v.x).to_degrees()
        };
        match self {
            Shape::Rect { cx, cy, .. } | Shape::Circle { cx, cy, .. } => mp(cx, cy),
            Shape::Polygon { cx, cy, rot, .. } => {
                let a = ang_of(*rot + 90.0) - 90.0;
                mp(cx, cy);
                *rot = a;
            }
            Shape::Slot { x1, y1, x2, y2, .. } | Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                mp(x1, y1);
                mp(x2, y2);
            }
            Shape::Profile { pts } | Shape::Spline { pts, .. } => {
                for p in pts.iter_mut() {
                    *p = f(*p);
                }
            }
            Shape::Arc { cx, cy, a0, a1, .. } => {
                let (n0, n1) = (ang_of(*a0), ang_of(*a1));
                mp(cx, cy);
                if reflect {
                    *a0 = n1;
                    *a1 = n0;
                } else {
                    *a0 = n0;
                    *a1 = n1;
                }
            }
            Shape::Ellipse { cx, cy, rot, .. } => {
                let a = ang_of(*rot);
                mp(cx, cy);
                *rot = a;
            }
        }
    }

    /// Points other elements snap to: ends, middles and centres.
    pub fn key_points(&self) -> Vec<DVec2> {
        match self {
            Shape::Rect { .. } | Shape::Polygon { .. } => {
                let mut v = self.explode().iter().flat_map(|e| e.key_points()).collect::<Vec<_>>();
                if let Shape::Rect { cx, cy, .. } | Shape::Polygon { cx, cy, .. } = self {
                    v.push(DVec2::new(*cx, *cy));
                }
                v
            }
            Shape::Circle { cx, cy, r } => {
                let c = DVec2::new(*cx, *cy);
                vec![c, c + DVec2::X * *r, c - DVec2::X * *r, c + DVec2::Y * *r, c - DVec2::Y * *r]
            }
            Shape::Ellipse { cx, cy, .. } => vec![DVec2::new(*cx, *cy)],
            Shape::Slot { x1, y1, x2, y2, .. } => vec![DVec2::new(*x1, *y1), DVec2::new(*x2, *y2)],
            Shape::Profile { pts } | Shape::Spline { pts, .. } => pts.clone(),
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                let a = DVec2::new(*x1, *y1);
                let b = DVec2::new(*x2, *y2);
                vec![a, b, (a + b) * 0.5]
            }
            Shape::Arc { cx, cy, .. } => {
                let (a, b) = self.endpoints().unwrap();
                vec![a, b, DVec2::new(*cx, *cy)]
            }
        }
    }

    /// Editable points (dragged one at a time in the sketcher).
    pub fn handle_points(&self) -> Vec<DVec2> {
        match self {
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => vec![DVec2::new(*x1, *y1), DVec2::new(*x2, *y2)],
            Shape::Profile { pts } | Shape::Spline { pts, .. } => pts.clone(),
            _ => vec![],
        }
    }
    pub fn set_handle(&mut self, i: usize, p: DVec2) {
        match self {
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                if i == 0 {
                    *x1 = p.x;
                    *y1 = p.y;
                } else {
                    *x2 = p.x;
                    *y2 = p.y;
                }
            }
            Shape::Profile { pts } | Shape::Spline { pts, .. } => {
                if i < pts.len() {
                    pts[i] = p;
                }
            }
            _ => {}
        }
    }

    pub fn label(&self) -> String {
        match self {
            Shape::Rect { w, h, r, .. } => {
                if *r > 0.0 { format!("{} x {}  R{}", fmt(w.abs()), fmt(h.abs()), fmt(*r)) } else { format!("{} x {}", fmt(w.abs()), fmt(h.abs())) }
            }
            Shape::Circle { r, .. } => format!("D{}", fmt(r.abs() * 2.0)),
            Shape::Polygon { r, n, .. } => format!("{}-gon R{}", n.round(), fmt(r.abs())),
            Shape::Slot { x1, y1, x2, y2, r } => format!("L{}  W{}", fmt(DVec2::new(x2 - x1, y2 - y1).length()), fmt(r.abs() * 2.0)),
            Shape::Profile { pts } => format!("{} points", pts.len()),
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => format!("{}", fmt(DVec2::new(x2 - x1, y2 - y1).length())),
            Shape::Arc { r, a0, a1, .. } => format!("R{}  {}deg", fmt(r.abs()), fmt(sweep_deg(*a0, *a1))),
            Shape::Ellipse { a, b, .. } => format!("{} x {}", fmt(a.abs() * 2.0), fmt(b.abs() * 2.0)),
            Shape::Spline { pts, .. } => format!("{} points", pts.len()),
        }
    }

    /// Editable numbers with their labels.
    pub fn fields(&self) -> Vec<(String, f64)> {
        let v = |l: &str, x: f64| (l.to_string(), x);
        match self {
            Shape::Rect { cx, cy, w, h, r } => vec![v("Centre X", *cx), v("Centre Y", *cy), v("Width", *w), v("Height", *h), v("Corner radius", *r)],
            Shape::Circle { cx, cy, r } => vec![v("Centre X", *cx), v("Centre Y", *cy), v("Radius", *r)],
            Shape::Polygon { cx, cy, r, n, rot } => vec![v("Centre X", *cx), v("Centre Y", *cy), v("Radius", *r), v("Sides", *n), v("Rotation (deg)", *rot)],
            Shape::Slot { x1, y1, x2, y2, r } => vec![v("Centre 1 X", *x1), v("Centre 1 Y", *y1), v("Centre 2 X", *x2), v("Centre 2 Y", *y2), v("Radius", *r)],
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                let d = DVec2::new(x2 - x1, y2 - y1);
                vec![
                    v("Start H", *x1),
                    v("Start V", *y1),
                    v("End H", *x2),
                    v("End V", *y2),
                    v("Length", (d.length() * 1e6).round() / 1e6),
                    v("Angle (deg)", (d.y.atan2(d.x).to_degrees() * 1e4).round() / 1e4),
                ]
            }
            Shape::Arc { cx, cy, r, a0, a1 } => vec![v("Centre H", *cx), v("Centre V", *cy), v("Radius", *r), v("Start angle", *a0), v("End angle", *a1)],
            Shape::Ellipse { cx, cy, a, b, rot } => vec![v("Centre H", *cx), v("Centre V", *cy), v("Major radius", *a), v("Minor radius", *b), v("Rotation (deg)", *rot)],
            Shape::Profile { pts } | Shape::Spline { pts, .. } => {
                let mut out = Vec::new();
                for (i, p) in pts.iter().enumerate() {
                    out.push((format!("Point {} H", i + 1), p.x));
                    out.push((format!("Point {} V", i + 1), p.y));
                }
                out
            }
        }
    }

    pub fn set_field(&mut self, i: usize, val: f64) {
        match self {
            Shape::Rect { cx, cy, w, h, r } => *[cx, cy, w, h, r][i.min(4)] = val,
            Shape::Circle { cx, cy, r } => *[cx, cy, r][i.min(2)] = val,
            Shape::Polygon { cx, cy, r, n, rot } => {
                *[cx, cy, r, n, rot][i.min(4)] = val;
                *n = n.round().clamp(3.0, 64.0);
            }
            Shape::Slot { x1, y1, x2, y2, r } => *[x1, y1, x2, y2, r][i.min(4)] = val,
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                let d = DVec2::new(*x2 - *x1, *y2 - *y1);
                match i {
                    0..=3 => *[x1, y1, x2, y2][i] = val,
                    4 => {
                        let dir = if d.length() > 1e-12 { d.normalize() } else { DVec2::X };
                        *x2 = *x1 + dir.x * val;
                        *y2 = *y1 + dir.y * val;
                    }
                    _ => {
                        let l = d.length();
                        let a = val.to_radians();
                        *x2 = *x1 + a.cos() * l;
                        *y2 = *y1 + a.sin() * l;
                    }
                }
            }
            Shape::Arc { cx, cy, r, a0, a1 } => *[cx, cy, r, a0, a1][i.min(4)] = val,
            Shape::Ellipse { cx, cy, a, b, rot } => *[cx, cy, a, b, rot][i.min(4)] = val,
            Shape::Profile { pts } | Shape::Spline { pts, .. } => {
                if let Some(p) = pts.get_mut(i / 2) {
                    if i % 2 == 0 {
                        p.x = val
                    } else {
                        p.y = val
                    }
                }
            }
        }
    }

    /// Splits a composite shape into lines and arcs (other shapes stay as they are).
    pub fn explode(&self) -> Vec<Shape> {
        let lines = |pts: &[DVec2]| -> Vec<Shape> {
            (0..pts.len())
                .map(|i| {
                    let a = pts[i];
                    let b = pts[(i + 1) % pts.len()];
                    Shape::Line { x1: a.x, y1: a.y, x2: b.x, y2: b.y }
                })
                .collect()
        };
        match self {
            Shape::Rect { cx, cy, w, h, r } => {
                let (hw, hh) = (w.abs() / 2.0, h.abs() / 2.0);
                let r = r.max(0.0).min(hw.min(hh) * 0.999);
                let c = DVec2::new(*cx, *cy);
                if r < 1e-6 {
                    return lines(&[c + DVec2::new(-hw, -hh), c + DVec2::new(hw, -hh), c + DVec2::new(hw, hh), c + DVec2::new(-hw, hh)]);
                }
                let l = |a: DVec2, b: DVec2| Shape::Line { x1: a.x, y1: a.y, x2: b.x, y2: b.y };
                let arc = |p: DVec2, a0: f64| Shape::Arc { cx: p.x, cy: p.y, r, a0, a1: a0 + 90.0 };
                vec![
                    l(c + DVec2::new(-hw + r, -hh), c + DVec2::new(hw - r, -hh)),
                    arc(c + DVec2::new(hw - r, -hh + r), -90.0),
                    l(c + DVec2::new(hw, -hh + r), c + DVec2::new(hw, hh - r)),
                    arc(c + DVec2::new(hw - r, hh - r), 0.0),
                    l(c + DVec2::new(hw - r, hh), c + DVec2::new(-hw + r, hh)),
                    arc(c + DVec2::new(-hw + r, hh - r), 90.0),
                    l(c + DVec2::new(-hw, hh - r), c + DVec2::new(-hw, -hh + r)),
                    arc(c + DVec2::new(-hw + r, -hh + r), 180.0),
                ]
            }
            Shape::Polygon { .. } => lines(&self.polygon_corners()),
            Shape::Profile { pts } => lines(pts),
            Shape::Slot { x1, y1, x2, y2, r } => {
                let a = DVec2::new(*x1, *y1);
                let b = DVec2::new(*x2, *y2);
                let d = b - a;
                if d.length() < 1e-9 {
                    return vec![Shape::Circle { cx: a.x, cy: a.y, r: *r }];
                }
                let ang = d.y.atan2(d.x).to_degrees();
                let n = DVec2::new(-d.y, d.x).normalize() * r.abs();
                vec![
                    Shape::Line { x1: a.x - n.x, y1: a.y - n.y, x2: b.x - n.x, y2: b.y - n.y },
                    Shape::Arc { cx: b.x, cy: b.y, r: r.abs(), a0: ang - 90.0, a1: ang + 90.0 },
                    Shape::Line { x1: b.x + n.x, y1: b.y + n.y, x2: a.x + n.x, y2: a.y + n.y },
                    Shape::Arc { cx: a.x, cy: a.y, r: r.abs(), a0: ang + 90.0, a1: ang + 270.0 },
                ]
            }
            other => vec![other.clone()],
        }
    }

    pub fn is_composite(&self) -> bool {
        matches!(self, Shape::Rect { .. } | Shape::Polygon { .. } | Shape::Profile { .. } | Shape::Slot { .. })
    }

    /// Distance from a point to the drawn element.
    pub fn distance(&self, p: DVec2) -> f64 {
        let c = self.curve();
        let closed = !self.is_open();
        let n = c.len();
        let mut d = f64::MAX;
        let segs = if closed { n } else { n.saturating_sub(1) };
        for k in 0..segs {
            d = d.min(dist_point_segment(p, c[k], c[(k + 1) % n]));
        }
        if n == 1 {
            d = (p - c[0]).length();
        }
        d
    }
}

pub fn fmt(v: f64) -> String {
    let s = format!("{:.3}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" { "0".into() } else { s }
}

#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
pub struct Sketch {
    pub frame: Frame,
    pub support: String,
    pub shapes: Vec<Shape>,
    /// construction (helper) elements are drawn dashed and never become material
    #[serde(default)]
    pub construction: Vec<bool>,
    /// dimensional constraints (lengths, distances, radii, angles) that drive the geometry
    #[serde(default)]
    pub dims: Vec<crate::dims::Dim>,
    /// reference elements projected from earlier sketches (classic CAD "use-edges"): where
    /// each came from. They are locked, never material, and follow their source.
    #[serde(default)]
    pub ext: Vec<Option<ExtRef>>,
}

/// An element of another sketch: (sketch feature id, element, piece of the exploded element).
pub type ExtRef = (u32, usize, usize);

impl Sketch {
    pub fn new(frame: Frame, support: String) -> Sketch {
        Sketch { frame, support, shapes: vec![], construction: vec![], dims: vec![], ext: vec![] }
    }
    pub fn is_construction(&self, i: usize) -> bool {
        self.construction.get(i).copied().unwrap_or(false) || matches!(self.shapes.get(i), Some(Shape::Axis { .. })) || self.is_ext(i)
    }
    pub fn is_ext(&self, i: usize) -> bool {
        matches!(self.ext.get(i), Some(Some(_)))
    }
    /// The elements of sketch `src` (feature `src_id`) seen on this sketch's plane. On a
    /// parallel plane everything projects; on a tilted one only lines do.
    pub fn projected(&self, src_id: u32, src: &Sketch) -> Vec<(ExtRef, Shape)> {
        let par = src.frame.n().dot(self.frame.n());
        let f = |p: DVec2| self.frame.to2(src.frame.to3(p, 0.0));
        let mut out = vec![];
        for (i, s) in src.shapes.iter().enumerate() {
            if src.is_ext(i) {
                continue;
            }
            for (k, mut piece) in s.explode().into_iter().enumerate() {
                if (par.abs() - 1.0).abs() < 1e-9 {
                    piece.map_points(&f, par < 0.0);
                } else if let Shape::Line { .. } | Shape::Axis { .. } = piece {
                    // ponytail: circles on a tilted plane would project to ellipses; skipped
                    piece.map_points(&f, false);
                    if piece.endpoints().map_or(true, |(a, b)| (a - b).length() < 1e-9) {
                        continue;
                    }
                } else {
                    continue;
                }
                out.push(((src_id, i, k), piece));
            }
        }
        out
    }
    /// Adds a reference element (or finds the one already there); returns its index.
    pub fn push_ext(&mut self, r: ExtRef, s: Shape) -> usize {
        if let Some(i) = self.ext.iter().position(|e| *e == Some(r)) {
            return i;
        }
        self.construction.resize(self.shapes.len(), false);
        self.ext.resize(self.shapes.len(), None);
        self.shapes.push(s);
        self.construction.push(true);
        self.ext.push(Some(r));
        self.shapes.len() - 1
    }
    pub fn set_construction(&mut self, i: usize, c: bool) {
        if self.construction.len() < self.shapes.len() {
            self.construction.resize(self.shapes.len(), false);
        }
        if i < self.construction.len() {
            self.construction[i] = c;
        }
    }
    pub fn push(&mut self, s: Shape, construction: bool) -> usize {
        self.construction.resize(self.shapes.len(), false);
        // a line drawn exactly level or upright gets a horizontal / vertical constraint
        let hv = match &s {
            Shape::Line { x1, y1, x2, y2 } => {
                let d = DVec2::new(x2 - x1, y2 - y1);
                if d.length() < 1e-9 {
                    None
                } else if d.y.abs() < 1e-9 {
                    Some(crate::dims::DimKind::Horizontal)
                } else if d.x.abs() < 1e-9 {
                    Some(crate::dims::DimKind::Vertical)
                } else {
                    None
                }
            }
            _ => None,
        };
        self.ext.resize(self.shapes.len(), None);
        self.shapes.push(s);
        self.construction.push(construction);
        self.ext.push(None);
        let i = self.shapes.len() - 1;
        if let Some(kind) = hv {
            self.dims.push(crate::dims::Dim { kind, a: crate::dims::GRef::Elem(i), b: None, value: 0.0, offset: DVec2::ZERO, reference: false });
        }
        i
    }
    pub fn remove(&mut self, i: usize) {
        self.construction.resize(self.shapes.len(), false);
        self.ext.resize(self.shapes.len(), None);
        if i < self.shapes.len() {
            self.shapes.remove(i);
            self.construction.remove(i);
            self.ext.remove(i);
            self.dims_after_remove(i);
        }
    }
    /// Replaces shape i by several (keeping its construction flag); returns their indices.
    pub fn replace(&mut self, i: usize, with: Vec<Shape>) -> Vec<usize> {
        let c = self.is_construction(i);
        self.remove(i);
        with.into_iter().map(|s| self.push(s, c)).collect()
    }

    pub fn axis(&self) -> Option<(DVec2, DVec2)> {
        self.shapes.iter().find_map(|s| match s {
            Shape::Axis { x1, y1, x2, y2 } => {
                let a = DVec2::new(*x1, *y1);
                let d = DVec2::new(x2 - x1, y2 - y1);
                if d.length() > 1e-9 { Some((a, d.normalize())) } else { None }
            }
            _ => None,
        })
    }

    /// End points of open elements that are not connected to anything (shown in red).
    pub fn open_ends(&self) -> Vec<DVec2> {
        let mut ends = Vec::new();
        for (i, s) in self.shapes.iter().enumerate() {
            if self.is_construction(i) {
                continue;
            }
            if let Some((a, b)) = s.endpoints() {
                ends.push(a);
                ends.push(b);
            }
        }
        ends.iter().filter(|p| ends.iter().filter(|q| (**q - **p).length() < JOIN_TOL).count() == 1).cloned().collect()
    }

    /// Closed profiles of the sketch: material outlines and holes (by nesting depth).
    pub fn regions(&self) -> Result<(Vec<Vec<DVec2>>, Vec<Vec<DVec2>>), String> {
        let mut loops: Vec<Vec<DVec2>> = Vec::new();
        let mut opens: Vec<Vec<DVec2>> = Vec::new();
        for (i, s) in self.shapes.iter().enumerate() {
            if self.is_construction(i) {
                continue;
            }
            if s.is_open() {
                let c = s.curve();
                if c.len() >= 2 && (c[0] - *c.last().unwrap()).length() > 1e-9 || c.len() > 2 {
                    opens.push(c);
                }
            } else {
                let o = s.outline();
                if o.len() >= 3 {
                    loops.push(o);
                }
            }
        }
        // join open elements end to end
        let mut used = vec![false; opens.len()];
        for i in 0..opens.len() {
            if used[i] {
                continue;
            }
            used[i] = true;
            let mut chain = opens[i].clone();
            let start = chain[0];
            loop {
                let end = *chain.last().unwrap();
                if chain.len() > 2 && (end - start).length() < JOIN_TOL {
                    chain.pop();
                    break;
                }
                let mut cands = Vec::new();
                for (j, o) in opens.iter().enumerate() {
                    if used[j] {
                        continue;
                    }
                    if (o[0] - end).length() < JOIN_TOL {
                        cands.push((j, false));
                    } else if (*o.last().unwrap() - end).length() < JOIN_TOL {
                        cands.push((j, true));
                    }
                }
                if cands.is_empty() {
                    return Err(format!("the profile is open near H {}  V {} (connect the ends, or make the element construction)", fmt(end.x), fmt(end.y)));
                }
                if cands.len() > 1 {
                    return Err(format!("more than two elements meet at H {}  V {}", fmt(end.x), fmt(end.y)));
                }
                let (j, rev) = cands[0];
                used[j] = true;
                let mut seg = opens[j].clone();
                if rev {
                    seg.reverse();
                }
                chain.extend(seg.into_iter().skip(1));
            }
            loops.push(clean_polygon(&chain));
        }
        let loops: Vec<Vec<DVec2>> = loops.into_iter().filter(|l| l.len() >= 3).collect();
        if loops.is_empty() {
            return Err("the sketch has no closed profile".into());
        }
        for o in &loops {
            if segments_self_intersect(o) {
                return Err("a profile crosses itself".into());
            }
        }
        let mut solids = Vec::new();
        let mut holes = Vec::new();
        for (i, o) in loops.iter().enumerate() {
            let probe = o[0];
            let depth = loops.iter().enumerate().filter(|(j, q)| *j != i && point_in_polygon(probe, q)).count();
            if depth % 2 == 0 { solids.push(o.clone()) } else { holes.push(o.clone()) }
        }
        Ok((solids, holes))
    }

    /// Moves every open-element end point (and profile/spline point) at `old` to `new`, so
    /// connected elements stay connected. `skip` elements are left alone.
    pub fn move_point(&mut self, old: DVec2, new: DVec2, skip: &[usize]) {
        let ext = &self.ext;
        for (i, s) in self.shapes.iter_mut().enumerate() {
            // reference elements are locked
            if skip.contains(&i) || matches!(ext.get(i), Some(Some(_))) {
                continue;
            }
            let hs = s.handle_points();
            for (k, h) in hs.iter().enumerate() {
                if (*h - old).length() < JOIN_TOL {
                    s.set_handle(k, new);
                }
            }
        }
    }
}

// ---------------------------------------------------------------- operations

fn seg_intersections(a: DVec2, b: DVec2, c: DVec2, d: DVec2) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.perp_dot(s);
    if den.abs() < 1e-14 {
        return None;
    }
    let t = (c - a).perp_dot(s) / den;
    let u = (c - a).perp_dot(r) / den;
    if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) {
        Some((t, u))
    } else {
        None
    }
}

fn other_polylines(sk: &Sketch, skip: usize) -> Vec<(Vec<DVec2>, bool)> {
    sk.shapes
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != skip)
        .map(|(_, s)| (s.curve(), !s.is_open()))
        .collect()
}

fn circle_hits(c: DVec2, r: f64, others: &[(Vec<DVec2>, bool)]) -> Vec<f64> {
    let mut angs = Vec::new();
    for (pl, closed) in others {
        let n = pl.len();
        let segs = if *closed { n } else { n.saturating_sub(1) };
        for k in 0..segs {
            let (p, q) = (pl[k], pl[(k + 1) % n]);
            let d = q - p;
            let f = p - c;
            let a = d.dot(d);
            if a < 1e-18 {
                continue;
            }
            let b = 2.0 * f.dot(d);
            let cc = f.dot(f) - r * r;
            let disc = b * b - 4.0 * a * cc;
            if disc < 0.0 {
                continue;
            }
            for t in [(-b - disc.sqrt()) / (2.0 * a), (-b + disc.sqrt()) / (2.0 * a)] {
                if (-1e-9..=1.0 + 1e-9).contains(&t) {
                    let h = p + d * t - c;
                    angs.push(h.y.atan2(h.x).to_degrees().rem_euclid(360.0));
                }
            }
        }
    }
    angs
}

/// Quick trim: removes the piece of element `idx` around point `p` that lies between its
/// nearest intersections with other elements. Returns a message for the status bar.
pub fn quick_trim(sk: &mut Sketch, idx: usize, p: DVec2) -> Result<String, String> {
    let mut idx = idx;
    if sk.shapes[idx].is_composite() {
        let parts = sk.shapes[idx].explode();
        let new = sk.replace(idx, parts);
        idx = *new.iter().min_by(|a, b| sk.shapes[**a].distance(p).partial_cmp(&sk.shapes[**b].distance(p)).unwrap()).unwrap();
    }
    let others = other_polylines(sk, idx);
    let shape = sk.shapes[idx].clone();
    match shape {
        Shape::Line { x1, y1, x2, y2 } => {
            let a = DVec2::new(x1, y1);
            let b = DVec2::new(x2, y2);
            let mut ts = Vec::new();
            for (pl, closed) in &others {
                let n = pl.len();
                let segs = if *closed { n } else { n.saturating_sub(1) };
                for k in 0..segs {
                    if let Some((t, _)) = seg_intersections(a, b, pl[k], pl[(k + 1) % n]) {
                        if t > 1e-6 && t < 1.0 - 1e-6 {
                            ts.push(t);
                        }
                    }
                }
            }
            let ab = b - a;
            let tc = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
            let lo = ts.iter().cloned().filter(|t| *t < tc).fold(0.0, f64::max);
            let hi = ts.iter().cloned().filter(|t| *t > tc).fold(1.0, f64::min);
            let mut keep = Vec::new();
            if lo > 1e-6 {
                let q = a + ab * lo;
                keep.push(Shape::Line { x1: a.x, y1: a.y, x2: q.x, y2: q.y });
            }
            if hi < 1.0 - 1e-6 {
                let q = a + ab * hi;
                keep.push(Shape::Line { x1: q.x, y1: q.y, x2: b.x, y2: b.y });
            }
            sk.replace(idx, keep);
            Ok("Trimmed the line.".into())
        }
        Shape::Circle { cx, cy, r } => {
            let c = DVec2::new(cx, cy);
            let mut angs = circle_hits(c, r, &others);
            angs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            angs.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
            if angs.len() < 2 {
                sk.remove(idx);
                return Ok("The circle crossed nothing, so it was deleted.".into());
            }
            let ac = (p - c).y.atan2((p - c).x).to_degrees().rem_euclid(360.0);
            let hi = angs.iter().cloned().find(|a| *a > ac).unwrap_or(angs[0]);
            let lo = angs.iter().rev().cloned().find(|a| *a < ac).unwrap_or(*angs.last().unwrap());
            sk.replace(idx, vec![Shape::Arc { cx, cy, r, a0: hi, a1: lo }]);
            Ok("Trimmed the circle into an arc.".into())
        }
        Shape::Arc { cx, cy, r, a0, a1 } => {
            let c = DVec2::new(cx, cy);
            let sw = sweep_deg(a0, a1);
            let rel = |a: f64| (a - a0).rem_euclid(360.0);
            let ts: Vec<f64> = circle_hits(c, r, &others).into_iter().map(rel).filter(|t| *t > 1e-6 && *t < sw - 1e-6).collect();
            let tc = rel((p - c).y.atan2((p - c).x).to_degrees()).min(sw);
            let lo = ts.iter().cloned().filter(|t| *t < tc).fold(0.0, f64::max);
            let hi = ts.iter().cloned().filter(|t| *t > tc).fold(sw, f64::min);
            let mut keep = Vec::new();
            if lo > 1e-6 {
                keep.push(Shape::Arc { cx, cy, r, a0, a1: a0 + lo });
            }
            if hi < sw - 1e-6 {
                keep.push(Shape::Arc { cx, cy, r, a0: a0 + hi, a1: a0 + sw });
            }
            sk.replace(idx, keep);
            Ok("Trimmed the arc.".into())
        }
        _ => {
            sk.remove(idx);
            Ok("That element type cannot be cut, so it was deleted.".into())
        }
    }
}

/// Finds the two lines meeting at the corner nearest `p` (within `tol`), exploding a
/// rectangle / polygon first if needed. Returns (vertex, line a, line b).
fn corner_lines(sk: &mut Sketch, p: DVec2, tol: f64) -> Result<(DVec2, usize, usize), String> {
    // explode composites that have a corner here
    for i in 0..sk.shapes.len() {
        if sk.shapes[i].is_composite() && sk.shapes[i].explode().iter().any(|e| e.endpoints().map_or(false, |(a, b)| (a - p).length() < tol || (b - p).length() < tol)) {
            let parts = sk.shapes[i].explode();
            sk.replace(i, parts);
            break;
        }
    }
    let mut best: Option<(f64, DVec2)> = None;
    for s in &sk.shapes {
        if let Shape::Line { .. } = s {
            for e in s.handle_points() {
                let d = (e - p).length();
                if d < tol && best.map_or(true, |b| d < b.0) {
                    best = Some((d, e));
                }
            }
        }
    }
    let v = best.ok_or("Click on a corner where two lines meet.")?.1;
    let lines: Vec<usize> = sk
        .shapes
        .iter()
        .enumerate()
        .filter(|(_, s)| matches!(s, Shape::Line { .. }) && s.handle_points().iter().any(|h| (*h - v).length() < JOIN_TOL))
        .map(|(i, _)| i)
        .collect();
    if lines.len() != 2 {
        return Err("A corner needs exactly two lines meeting at that point.".into());
    }
    Ok((v, lines[0], lines[1]))
}

fn other_end(s: &Shape, v: DVec2) -> (usize, DVec2) {
    let h = s.handle_points();
    if (h[0] - v).length() < JOIN_TOL { (0, h[1]) } else { (1, h[0]) }
}

/// Rounds the corner nearest `p` with an arc of radius `r` (sketch "Corner").
pub fn corner(sk: &mut Sketch, p: DVec2, r: f64, tol: f64) -> Result<(), String> {
    let (v, ia, ib) = corner_lines(sk, p, tol)?;
    let (ka, ea) = other_end(&sk.shapes[ia], v);
    let (kb, eb) = other_end(&sk.shapes[ib], v);
    let d1 = (ea - v).normalize();
    let d2 = (eb - v).normalize();
    let cos = d1.dot(d2).clamp(-1.0, 1.0);
    let theta = cos.acos();
    if theta < 1e-3 || theta > PI - 1e-3 {
        return Err("The two lines are in line with each other.".into());
    }
    let t = r / (theta / 2.0).tan();
    if t >= (ea - v).length() || t >= (eb - v).length() {
        return Err("That radius is too big for these lines.".into());
    }
    let p1 = v + d1 * t;
    let p2 = v + d2 * t;
    let c = v + (d1 + d2).normalize() * (r / (theta / 2.0).sin());
    let mut a1 = (p1 - c).y.atan2((p1 - c).x).to_degrees();
    let mut a2 = (p2 - c).y.atan2((p2 - c).x).to_degrees();
    if sweep_deg(a1, a2) > 180.0 {
        std::mem::swap(&mut a1, &mut a2);
    }
    sk.shapes[ia].set_handle(ka, p1);
    sk.shapes[ib].set_handle(kb, p2);
    let constr = sk.is_construction(ia);
    sk.push(Shape::Arc { cx: c.x, cy: c.y, r, a0: a1, a1: a2 }, constr);
    Ok(())
}

/// Cuts the corner nearest `p` with a straight line `dist` from the corner on both lines.
pub fn chamfer(sk: &mut Sketch, p: DVec2, dist: f64, tol: f64) -> Result<(), String> {
    let (v, ia, ib) = corner_lines(sk, p, tol)?;
    let (ka, ea) = other_end(&sk.shapes[ia], v);
    let (kb, eb) = other_end(&sk.shapes[ib], v);
    if dist >= (ea - v).length() || dist >= (eb - v).length() {
        return Err("That chamfer is too big for these lines.".into());
    }
    let p1 = v + (ea - v).normalize() * dist;
    let p2 = v + (eb - v).normalize() * dist;
    sk.shapes[ia].set_handle(ka, p1);
    sk.shapes[ib].set_handle(kb, p2);
    let constr = sk.is_construction(ia);
    sk.push(Shape::Line { x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y }, constr);
    Ok(())
}

/// Mirror copies of `sel` across the line through `a` with direction `d`.
pub fn mirror(sk: &mut Sketch, sel: &[usize], a: DVec2, d: DVec2) -> Vec<usize> {
    let d = d.normalize();
    let refl = move |q: DVec2| -> DVec2 {
        let w = q - a;
        a + d * (2.0 * w.dot(d)) - w
    };
    let mut new = Vec::new();
    for &i in sel {
        if i >= sk.shapes.len() {
            continue;
        }
        let c = sk.is_construction(i);
        let parts = if matches!(sk.shapes[i], Shape::Rect { .. } | Shape::Polygon { .. }) { sk.shapes[i].explode() } else { vec![sk.shapes[i].clone()] };
        for mut s in parts {
            s.map_points(&refl, true);
            new.push(sk.push(s, c));
        }
    }
    new
}

/// Circle through three points.
pub fn circle3(a: DVec2, b: DVec2, c: DVec2) -> Option<(DVec2, f64)> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (a.length_squared(), b.length_squared(), c.length_squared());
    let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    let o = DVec2::new(ux, uy);
    Some((o, (a - o).length()))
}

/// Arc from `a` through `m` to `b`.
pub fn arc3(a: DVec2, m: DVec2, b: DVec2) -> Option<Shape> {
    let (c, r) = circle3(a, m, b)?;
    let ang = |p: DVec2| (p - c).y.atan2((p - c).x).to_degrees();
    let (aa, am, ab) = (ang(a), ang(m), ang(b));
    // counter-clockwise from a to b passes through m?
    let s_ab = sweep_deg(aa, ab);
    let s_am = sweep_deg(aa, am);
    if s_am < s_ab { Some(Shape::Arc { cx: c.x, cy: c.y, r, a0: aa, a1: ab }) } else { Some(Shape::Arc { cx: c.x, cy: c.y, r, a0: ab, a1: aa }) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: (f64, f64), b: (f64, f64)) -> Shape {
        Shape::Line { x1: a.0, y1: a.1, x2: b.0, y2: b.1 }
    }

    #[test]
    fn lines_make_a_profile() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(line((0.0, 0.0), (10.0, 0.0)), false);
        sk.push(line((10.0, 10.0), (10.0, 0.0)), false); // reversed on purpose
        sk.push(line((10.0, 10.0), (0.0, 10.0)), false);
        assert!(sk.regions().is_err(), "still open");
        assert_eq!(sk.open_ends().len(), 2);
        sk.push(line((0.0, 10.0), (0.0, 0.0)), false);
        let (s, h) = sk.regions().unwrap();
        assert_eq!(s.len(), 1);
        assert!(h.is_empty());
        assert!((signed_area(&s[0]) - 100.0).abs() < 1e-9);
        // round one corner and chamfer another
        corner(&mut sk, DVec2::new(10.0, 10.0), 2.0, 0.5).unwrap();
        chamfer(&mut sk, DVec2::new(0.0, 0.0), 1.0, 0.5).unwrap();
        let (s, _) = sk.regions().unwrap();
        let area = signed_area(&s[0]);
        let expect = 100.0 - (4.0 - PI) - 0.5;
        assert!((area - expect).abs() < 0.05, "{} vs {}", area, expect);
    }

    #[test]
    fn trim_circle_and_line() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(Shape::Circle { cx: 0.0, cy: 0.0, r: 10.0 }, false);
        sk.push(line((-20.0, 0.0), (20.0, 0.0)), false);
        // cut the outer stubs of the line and the lower half of the circle -> a D shape
        quick_trim(&mut sk, 1, DVec2::new(-15.0, 0.0)).unwrap();
        let li = sk.shapes.iter().position(|s| matches!(s, Shape::Line { .. })).unwrap();
        quick_trim(&mut sk, li, DVec2::new(15.0, 0.0)).unwrap();
        quick_trim(&mut sk, 0, DVec2::new(0.0, -10.0)).unwrap();
        let (s, _) = sk.regions().unwrap();
        let area = signed_area(&s[0]).abs();
        assert!((area - PI * 50.0).abs() < 1.0, "{}", area);
    }

    #[test]
    fn arc_through_points_and_mirror() {
        let a = arc3(DVec2::new(10.0, 0.0), DVec2::new(0.0, 10.0), DVec2::new(-10.0, 0.0)).unwrap();
        if let Shape::Arc { r, a0, a1, .. } = a {
            assert!((r - 10.0).abs() < 1e-9);
            assert!((sweep_deg(a0, a1) - 180.0).abs() < 1e-6);
        }
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(a, false);
        sk.push(line((-10.0, 0.0), (10.0, 0.0)), false);
        let (s, _) = sk.regions().unwrap();
        assert!((signed_area(&s[0]) - PI * 50.0).abs() < 1.0);
        let new = mirror(&mut sk, &[0], DVec2::ZERO, DVec2::X);
        assert_eq!(new.len(), 1);
        sk.remove(1);
        let (s, _) = sk.regions().unwrap();
        assert!((signed_area(&s[0]) - PI * 100.0).abs() < 2.0, "{}", signed_area(&s[0]));
    }
}

// ---------------------------------------------------------------- exact snapping geometry

/// Exact building blocks of an element: straight segments and circles / arcs.
#[derive(Clone, Debug)]
enum Prim {
    Seg(DVec2, DVec2),
    /// centre, radius, optional (start angle, sweep) in degrees for arcs
    Circ(DVec2, f64, Option<(f64, f64)>),
}

impl Prim {
    fn nearest(&self, p: DVec2) -> DVec2 {
        match self {
            Prim::Seg(a, b) => {
                let ab = *b - *a;
                let l = ab.length_squared();
                if l < 1e-18 {
                    return *a;
                }
                *a + ab * ((p - *a).dot(ab) / l).clamp(0.0, 1.0)
            }
            Prim::Circ(c, r, arc) => {
                let d = p - *c;
                let dir = if d.length() < 1e-12 { DVec2::X } else { d.normalize() };
                let on = *c + dir * *r;
                match arc {
                    None => on,
                    Some((a0, sw)) => {
                        if on_arc(*c, on, *a0, *sw) {
                            on
                        } else {
                            let e0 = *c + DVec2::new(a0.to_radians().cos(), a0.to_radians().sin()) * *r;
                            let a1 = (a0 + sw).to_radians();
                            let e1 = *c + DVec2::new(a1.cos(), a1.sin()) * *r;
                            if (e0 - p).length() < (e1 - p).length() { e0 } else { e1 }
                        }
                    }
                }
            }
        }
    }
    fn contains(&self, q: DVec2) -> bool {
        match self {
            Prim::Seg(a, b) => dist_point_segment(q, *a, *b) < 1e-6,
            Prim::Circ(c, r, arc) => ((q - *c).length() - r).abs() < 1e-6 && arc.map_or(true, |(a0, sw)| on_arc(*c, q, a0, sw)),
        }
    }
}

fn on_arc(c: DVec2, q: DVec2, a0: f64, sw: f64) -> bool {
    let a = (q - c).y.atan2((q - c).x).to_degrees();
    (a - a0).rem_euclid(360.0) <= sw + 1e-7
}

/// Points where the infinite line o + t·d meets a circle.
fn line_circle(o: DVec2, d: DVec2, c: DVec2, r: f64) -> Vec<(f64, DVec2)> {
    let a = d.dot(d);
    if a < 1e-18 {
        return vec![];
    }
    let f = o - c;
    let b = 2.0 * f.dot(d);
    let cc = f.dot(f) - r * r;
    let disc = b * b - 4.0 * a * cc;
    if disc < -1e-12 {
        return vec![];
    }
    let s = disc.max(0.0).sqrt();
    let mut v = vec![(-b - s) / (2.0 * a)];
    if s > 1e-12 {
        v.push((-b + s) / (2.0 * a));
    }
    v.into_iter().map(|t| (t, o + d * t)).collect()
}

fn prim_hits(a: &Prim, b: &Prim) -> Vec<DVec2> {
    let raw: Vec<DVec2> = match (a, b) {
        (Prim::Seg(p0, p1), Prim::Seg(q0, q1)) => {
            let r = *p1 - *p0;
            let s = *q1 - *q0;
            let den = r.perp_dot(s);
            if den.abs() < 1e-14 {
                vec![]
            } else {
                let t = (*q0 - *p0).perp_dot(s) / den;
                let u = (*q0 - *p0).perp_dot(r) / den;
                if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) { vec![*p0 + r * t] } else { vec![] }
            }
        }
        (Prim::Seg(p0, p1), Prim::Circ(c, r, _)) | (Prim::Circ(c, r, _), Prim::Seg(p0, p1)) => {
            line_circle(*p0, *p1 - *p0, *c, *r).into_iter().filter(|(t, _)| (-1e-9..=1.0 + 1e-9).contains(t)).map(|x| x.1).collect()
        }
        (Prim::Circ(c1, r1, _), Prim::Circ(c2, r2, _)) => {
            let d = (*c2 - *c1).length();
            if d < 1e-12 || d > r1 + r2 + 1e-9 || d < (r1 - r2).abs() - 1e-9 {
                vec![]
            } else {
                let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
                let h = (r1 * r1 - a * a).max(0.0).sqrt();
                let u = (*c2 - *c1) / d;
                let m = *c1 + u * a;
                let n = DVec2::new(-u.y, u.x);
                if h < 1e-9 { vec![m] } else { vec![m + n * h, m - n * h] }
            }
        }
    };
    raw.into_iter().filter(|q| a.contains(*q) && b.contains(*q)).collect()
}

impl Shape {
    fn prims(&self) -> Vec<Prim> {
        match self {
            Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => vec![Prim::Seg(DVec2::new(*x1, *y1), DVec2::new(*x2, *y2))],
            Shape::Circle { cx, cy, r } => vec![Prim::Circ(DVec2::new(*cx, *cy), r.abs(), None)],
            Shape::Arc { cx, cy, r, a0, a1 } => vec![Prim::Circ(DVec2::new(*cx, *cy), r.abs(), Some((*a0, sweep_deg(*a0, *a1))))],
            Shape::Rect { .. } | Shape::Polygon { .. } | Shape::Slot { .. } | Shape::Profile { .. } => self.explode().iter().flat_map(|e| e.prims()).collect(),
            _ => {
                // ellipses and splines: their drawn polyline
                let c = self.curve();
                let n = c.len();
                let m = if self.is_open() { n.saturating_sub(1) } else { n };
                (0..m).map(|k| Prim::Seg(c[k], c[(k + 1) % n])).collect()
            }
        }
    }

    /// The point on this element closest to `p` (exact for lines, circles and arcs).
    pub fn nearest_point(&self, p: DVec2) -> DVec2 {
        let mut best = (f64::MAX, p);
        for pr in self.prims() {
            let q = pr.nearest(p);
            let d = (q - p).length();
            if d < best.0 {
                best = (d, q);
            }
        }
        best.1
    }

    /// Where the infinite line through `o` with direction `d` crosses this element.
    pub fn line_hits(&self, o: DVec2, d: DVec2) -> Vec<DVec2> {
        let mut out = Vec::new();
        for pr in self.prims() {
            match &pr {
                Prim::Seg(a, b) => {
                    let s = *b - *a;
                    let den = d.perp_dot(s);
                    if den.abs() > 1e-14 {
                        let u = (*a - o).perp_dot(d) / den;
                        if (-1e-9..=1.0 + 1e-9).contains(&u) {
                            out.push(*a + s * u);
                        }
                    }
                }
                Prim::Circ(c, r, _) => {
                    for (_, q) in line_circle(o, d, *c, *r) {
                        if pr.contains(q) {
                            out.push(q);
                        }
                    }
                }
            }
        }
        out
    }
}

/// Points where two elements cross.
pub fn intersections(a: &Shape, b: &Shape) -> Vec<DVec2> {
    let pa = a.prims();
    let pb = b.prims();
    let mut out = Vec::new();
    for x in &pa {
        for y in &pb {
            out.extend(prim_hits(x, y));
        }
    }
    out
}

#[cfg(test)]
mod snap_tests {
    use super::*;
    #[test]
    fn snapping_onto_curves() {
        let c = Shape::Circle { cx: 0.0, cy: 0.0, r: 10.0 };
        let p = c.nearest_point(DVec2::new(7.0, 7.5));
        assert!((p.length() - 10.0).abs() < 1e-12);
        let a = Shape::Arc { cx: 0.0, cy: 0.0, r: 5.0, a0: 0.0, a1: 90.0 };
        let q = a.nearest_point(DVec2::new(-3.0, -3.0));
        assert!((q - DVec2::new(5.0, 0.0)).length() < 1e-9 || (q - DVec2::new(0.0, 5.0)).length() < 1e-9);
        let l = Shape::Line { x1: -20.0, y1: 3.0, x2: 20.0, y2: 3.0 };
        let hits = intersections(&l, &c);
        assert_eq!(hits.len(), 2);
        for h in &hits {
            assert!((h.length() - 10.0).abs() < 1e-9 && (h.y - 3.0).abs() < 1e-12);
        }
        let h = c.line_hits(DVec2::new(30.0, 6.0), DVec2::X);
        assert_eq!(h.len(), 2);
        assert!(h.iter().all(|p| (p.x.abs() - 8.0).abs() < 1e-9));
    }
}
