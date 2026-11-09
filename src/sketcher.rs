//! The Sketcher workbench: drawing tools, snapping, selection and editing of sketch elements.

use crate::app::*;
use crate::geom::*;
use crate::measure::*;
use crate::dims::*;
use crate::model::*;
use crate::ui::*;
use crate::view::quat_from_back_up;
use glam::{DVec2, Quat};
use macroquad::prelude::*;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SkTool {
    Select,
    Profile,
    Line,
    Axis,
    Rect,
    CRect,
    Circle,
    Circle3,
    Arc,
    Arc3,
    Ellipse,
    Spline,
    Polygon,
    Slot,
    Corner,
    Chamfer,
    Trim,
    Mirror,
    MeasureBetween,
    MeasureItem,
    Constraint,
}

impl SkTool {
    pub fn hint(self) -> &'static str {
        match self {
            SkTool::Select => "Select: click an element to edit it (Ctrl adds), drag elements or their end points, drag on empty space to box-select. Del deletes.",
            SkTool::Profile => "Profile: click points to draw connected lines. Click the first point to close, double-click or Enter to stop.",
            SkTool::Line => "Line: click the start point, then the end point. Lines snap to other ends to connect.",
            SkTool::Axis => "Axis: click two points. Shaft and Groove turn the sketch around this axis.",
            SkTool::Rect => "Rectangle: click the first corner, then the opposite corner.",
            SkTool::CRect => "Centered Rectangle: click the centre, then a corner.",
            SkTool::Circle => "Circle: click the centre, then a point on the circle.",
            SkTool::Circle3 => "Three Point Circle: click three points on the circle.",
            SkTool::Arc => "Arc: click the centre, the start point, then the end point (counter-clockwise).",
            SkTool::Arc3 => "Three Point Arc: click the start point, a point on the arc, then the end point.",
            SkTool::Ellipse => "Ellipse: click the centre, the end of the long axis, then a point for the width.",
            SkTool::Spline => "Spline: click points for a smooth curve. Click the first point to close it, double-click or Enter to finish.",
            SkTool::Polygon => "Hexagon / Polygon: click the centre, then a corner. Change the number of sides afterwards.",
            SkTool::Slot => "Elongated Hole: click both centres, then a point for the width.",
            SkTool::Corner => "Corner: click where two lines meet to round it (radius on the right).",
            SkTool::Chamfer => "Chamfer: click where two lines meet to cut the corner (length on the right).",
            SkTool::Trim => "Quick Trim: click the piece of a line, arc or circle to cut away (up to where it crosses others).",
            SkTool::Mirror => "Mirror: select elements first, then click a line, the axis or the H / V axis to mirror across.",
            SkTool::MeasureBetween => "Measure Between: click a first item (point, line, circle, arc or the H / V axis), then a second one.",
            SkTool::MeasureItem => "Measure Item: click a line, arc, circle, closed shape or point to see its sizes.",
            SkTool::Constraint => "Constraint: click an element and then empty space to place its dimension, or click two elements for a distance / angle. Double-click a dimension to change it.",
        }
    }
    fn needed(self) -> usize {
        match self {
            SkTool::Line | SkTool::Axis | SkTool::Rect | SkTool::CRect | SkTool::Circle | SkTool::Polygon => 1,
            SkTool::Circle3 | SkTool::Arc | SkTool::Arc3 | SkTool::Ellipse | SkTool::Slot => 2,
            _ => usize::MAX,
        }
    }
}

#[derive(Clone)]
pub enum SkDrag {
    Shapes { start: DVec2, orig: Vec<Shape>, moved: bool },
    Point { p0: DVec2, orig: Vec<Shape>, moved: bool },
    Box { start: DVec2 },
    Dim { idx: usize, start: DVec2, orig: DVec2, moved: bool },
}

pub struct SketchEdit {
    pub feat: u32,
    pub tool: SkTool,
    pub pts: Vec<DVec2>,
    pub sel: Vec<usize>,
    pub drag: Option<SkDrag>,
    pub cursor: DVec2,
    pub raw: DVec2,
    pub cursor_ok: bool,
    pub snapped_to_point: bool,
    /// what the cursor snapped to: "point", "intersection", "on curve", "origin"
    pub snap_kind: Option<&'static str>,
    pub hv: Option<&'static str>,
    pub prev_view: (Quat, Vec3, f32),
    pub snap: bool,
    pub grid: f64,
    pub construction_mode: bool,
    pub corner_r: f64,
    pub chamfer_d: f64,
    pub measures: Vec<SkMeasure>,
    pub meas_first: Option<MeasItem>,
    pub keep_measures: bool,
    /// first item picked with the Constraint tool
    pub cons_first: Option<GRef>,
    pub sel_dim: Option<usize>,
    /// where each dimension's text was drawn (screen), for clicking and dragging it
    pub dim_rects: Vec<(usize, Rect)>,
}

pub fn preview(tool: SkTool, pts: &[DVec2], c: DVec2) -> Vec<Shape> {
    let line = |a: DVec2, b: DVec2| Shape::Line { x1: a.x, y1: a.y, x2: b.x, y2: b.y };
    let n = pts.len();
    let out: Option<Shape> = match (tool, n) {
        (SkTool::Line, 1) | (SkTool::Profile, 1..) => Some(line(pts[n - 1], c)),
        (SkTool::Axis, 1) => Some(Shape::Axis { x1: pts[0].x, y1: pts[0].y, x2: c.x, y2: c.y }),
        (SkTool::Rect, 1) => {
            let m = (pts[0] + c) * 0.5;
            let d = (c - pts[0]).abs();
            (d.x > 1e-9 && d.y > 1e-9).then(|| Shape::Rect { cx: m.x, cy: m.y, w: d.x, h: d.y, r: 0.0 })
        }
        (SkTool::CRect, 1) => {
            let d = (c - pts[0]).abs() * 2.0;
            (d.x > 1e-9 && d.y > 1e-9).then(|| Shape::Rect { cx: pts[0].x, cy: pts[0].y, w: d.x, h: d.y, r: 0.0 })
        }
        (SkTool::Circle, 1) => {
            let r = (c - pts[0]).length();
            (r > 1e-9).then(|| Shape::Circle { cx: pts[0].x, cy: pts[0].y, r })
        }
        (SkTool::Circle3, 1) => Some(line(pts[0], c)),
        (SkTool::Circle3, 2) => circle3(pts[0], pts[1], c).map(|(o, r)| Shape::Circle { cx: o.x, cy: o.y, r }),
        (SkTool::Arc, 1) => {
            let r = (c - pts[0]).length();
            (r > 1e-9).then(|| Shape::Circle { cx: pts[0].x, cy: pts[0].y, r })
        }
        (SkTool::Arc, 2) => {
            let o = pts[0];
            let r = (pts[1] - o).length();
            let a0 = (pts[1] - o).y.atan2((pts[1] - o).x).to_degrees();
            let a1 = (c - o).y.atan2((c - o).x).to_degrees();
            (r > 1e-9).then(|| Shape::Arc { cx: o.x, cy: o.y, r, a0, a1 })
        }
        (SkTool::Arc3, 1) => Some(line(pts[0], c)),
        (SkTool::Arc3, 2) => arc3(pts[0], pts[1], c),
        (SkTool::Ellipse, 1) => Some(line(pts[0], c)),
        (SkTool::Ellipse, 2) => {
            let o = pts[0];
            let ax = pts[1] - o;
            let a = ax.length();
            let b = dist_point_line(c, o, ax);
            (a > 1e-9 && b > 1e-9).then(|| Shape::Ellipse { cx: o.x, cy: o.y, a, b, rot: ax.y.atan2(ax.x).to_degrees() })
        }
        (SkTool::Polygon, 1) => {
            let d = c - pts[0];
            let r = d.length();
            (r > 1e-9).then(|| Shape::Polygon { cx: pts[0].x, cy: pts[0].y, r, n: 6.0, rot: ((d.y.atan2(d.x).to_degrees() - 90.0) * 1000.0).round() / 1000.0 })
        }
        (SkTool::Slot, 1) => Some(Shape::Slot { x1: pts[0].x, y1: pts[0].y, x2: c.x, y2: c.y, r: ((c - pts[0]).length() * 0.25).max(1e-3) }),
        (SkTool::Slot, 2) => {
            let r = dist_point_segment(c, pts[0], pts[1]);
            (r > 1e-9).then(|| Shape::Slot { x1: pts[0].x, y1: pts[0].y, x2: pts[1].x, y2: pts[1].y, r })
        }
        (SkTool::Spline, 1..) => {
            let mut p = pts.to_vec();
            p.push(c);
            Some(Shape::Spline { pts: p, closed: false })
        }
        _ => None,
    };
    out.into_iter().collect()
}

fn dist_point_line(p: DVec2, o: DVec2, d: DVec2) -> f64 {
    let l = d.length();
    if l < 1e-12 { (p - o).length() } else { (d.perp_dot(p - o) / l).abs() }
}

impl App {
    pub fn set_sketch_tool(&mut self, t: SkTool) {
        if let Some(sk) = self.sketch.as_mut() {
            sk.tool = t;
            sk.pts.clear();
            sk.drag = None;
            sk.meas_first = None;
            sk.cons_first = None;
            if !sk.keep_measures && !matches!(t, SkTool::MeasureBetween | SkTool::MeasureItem) {
                sk.measures.clear();
            }
            if t != SkTool::Select && t != SkTool::Mirror {
                sk.sel.clear();
            }
        }
        self.say(t.hint());
    }

    pub fn update_grid(&mut self) {
        let f = match self.sketch_frame_world() {
            Some(f) => f,
            None => return,
        };
        let px = self.view.px_size(f.o.as_vec3()) as f64;
        let target = px * 12.0;
        let mut step = 0.1;
        'outer: for e in -1..6 {
            for m in [1.0, 2.0, 5.0] {
                step = m * 10f64.powi(e);
                if step >= target {
                    break 'outer;
                }
            }
        }
        if let Some(sk) = self.sketch.as_mut() {
            sk.grid = step;
        }
    }

    fn sketch_px(&self) -> f64 {
        self.sketch_frame_world().map(|f| self.view.px_size(f.o.as_vec3()) as f64).unwrap_or(1.0)
    }

    /// Element under the cursor (nearest outline, else the smallest closed shape around it).
    pub fn sketch_hover_shape(&self) -> Option<usize> {
        let sk = self.sketch.as_ref()?;
        let data = self.sketch_data()?;
        let px = self.sketch_px();
        let c = sk.raw;
        let mut best: Option<(f64, usize)> = None;
        for (i, s) in data.shapes.iter().enumerate() {
            let d = s.distance(c);
            let score = if d < px * 8.0 {
                d
            } else if !s.is_open() && point_in_polygon(c, &s.outline()) {
                px * 8.0 + signed_area(&s.outline()).abs().sqrt() * 0.01
            } else {
                continue;
            };
            if best.map_or(true, |b| score < b.0) {
                best = Some((score, i));
            }
        }
        best.map(|b| b.1)
    }

    fn handle_near(&self, p: DVec2) -> Option<DVec2> {
        let data = self.sketch_data()?;
        let px = self.sketch_px();
        let mut best: Option<(f64, DVec2)> = None;
        for s in &data.shapes {
            for h in s.handle_points() {
                let d = (h - p).length();
                if d < px * 7.0 && best.map_or(true, |b| d < b.0) {
                    best = Some((d, h));
                }
            }
        }
        best.map(|b| b.1)
    }

    pub fn sketch_input(&mut self, inp: &Input) {
        self.update_grid();
        let f = match self.sketch_frame_world() {
            Some(f) => f,
            None => return,
        };
        let (o, d) = self.view.ray(inp.mouse);
        let n = f.n();
        let den = d.dot(n);
        let px = self.sketch_px();
        let data = self.sketch_data().cloned();
        let Some(data) = data else { return };
        let sk = self.sketch.as_mut().unwrap();
        if den.abs() < 1e-9 {
            sk.cursor_ok = false;
            return;
        }
        let raw = f.to2(o + d * ((f.o - o).dot(n) / den));
        sk.raw = raw;
        sk.cursor_ok = true;

        // --- snapping: end / centre / middle points and intersections first, then any point
        //     on a line, circle or arc, then horizontal / vertical, then the grid
        let mut cur = raw;
        sk.snapped_to_point = false;
        sk.snap_kind = None;
        sk.hv = None;
        let dragging_point = match &sk.drag {
            Some(SkDrag::Point { p0, .. }) => Some(*p0),
            _ => None,
        };
        // elements the cursor may snap to (not the ones being dragged)
        let usable: Vec<usize> = (0..data.shapes.len())
            .filter(|i| {
                let moving_shape = matches!(sk.drag, Some(SkDrag::Shapes { .. })) && sk.sel.contains(i);
                let moving_point = dragging_point.map_or(false, |q| data.shapes[*i].handle_points().iter().any(|h| (*h - q).length() < JOIN_TOL));
                !moving_shape && !moving_point
            })
            .collect();
        let mut kp: Vec<(DVec2, &'static str)> = vec![(DVec2::ZERO, "origin")];
        for &i in &usable {
            kp.extend(data.shapes[i].key_points().into_iter().map(|p| (p, "point")));
        }
        kp.extend(sk.pts.iter().map(|p| (*p, "point")));
        // intersections of the elements right under the cursor
        let near: Vec<usize> = usable.iter().cloned().filter(|i| data.shapes[*i].distance(raw) < px * 10.0).collect();
        for x in 0..near.len() {
            for y in (x + 1)..near.len() {
                kp.extend(intersections(&data.shapes[near[x]], &data.shapes[near[y]]).into_iter().map(|p| (p, "intersection")));
            }
        }
        let mut best = px * 10.0;
        for (p, kind) in kp {
            if dragging_point.map_or(false, |q| (q - p).length() < JOIN_TOL) {
                continue;
            }
            let dd = (p - raw).length();
            if dd < best {
                best = dd;
                cur = p;
                sk.snapped_to_point = true;
                sk.snap_kind = Some(kind);
            }
        }
        let last = if matches!(sk.tool, SkTool::Line | SkTool::Profile | SkTool::Axis) { sk.pts.last().copied() } else { None };
        let hv_dir = last.and_then(|l| {
            let dv = raw - l;
            let tol = 3f64.to_radians().tan();
            if dv.y.abs() < dv.x.abs() * tol {
                Some(("H", DVec2::X))
            } else if dv.x.abs() < dv.y.abs() * tol {
                Some(("V", DVec2::Y))
            } else {
                None
            }
        });
        if !sk.snapped_to_point {
            // any point on the nearest line, circle, arc, ellipse or spline
            let mut on: Option<(f64, usize)> = None;
            for &i in &near {
                let d = data.shapes[i].distance(raw);
                if d < px * 8.0 && on.map_or(true, |o| d < o.0) {
                    on = Some((d, i));
                }
            }
            if let Some((_, i)) = on {
                let shape = &data.shapes[i];
                cur = shape.nearest_point(raw);
                sk.snapped_to_point = true;
                sk.snap_kind = Some("on curve");
                // a horizontal / vertical line can end exactly where it meets the curve
                if let (Some(l), Some((hv, dir))) = (last, hv_dir) {
                    let hit = shape.line_hits(l, dir).into_iter().filter(|h| (*h - raw).length() < px * 12.0).min_by(|a, b| (*a - raw).length().partial_cmp(&(*b - raw).length()).unwrap());
                    if let Some(h) = hit {
                        cur = h;
                        sk.hv = Some(hv);
                    }
                }
            }
        }
        if !sk.snapped_to_point {
            if sk.snap {
                cur = (raw / sk.grid).round() * sk.grid;
            }
            if let (Some(l), Some((hv, _))) = (last, hv_dir) {
                if hv == "H" {
                    cur.y = l.y;
                } else {
                    cur.x = l.x;
                }
                sk.hv = Some(hv);
            }
        }
        sk.cursor = cur;
        let tool = sk.tool;
        let (corner_r, chamfer_d) = (sk.corner_r, sk.chamfer_d);

        if matches!(tool, SkTool::Select | SkTool::Constraint) && self.dim_text_input(inp, raw) {
            return;
        }
        match tool {
            SkTool::Select => self.select_input(inp, cur, raw),
            SkTool::Constraint => {
                if inp.pressed[LMB] {
                    self.constraint_click(raw, px);
                }
            }
            SkTool::Corner | SkTool::Chamfer => {
                if inp.pressed[LMB] {
                    let (r, c) = (corner_r, chamfer_d);
                    let mut s = data.clone();
                    let res = if tool == SkTool::Corner { corner(&mut s, raw, r, px * 14.0) } else { chamfer(&mut s, raw, c, px * 14.0) };
                    match res {
                        Ok(()) => {
                            self.commit_sketch(s);
                            self.say(if tool == SkTool::Corner { "Corner rounded. Click another corner, or Esc." } else { "Chamfer added. Click another corner, or Esc." });
                        }
                        Err(e) => self.err(&e),
                    }
                }
            }
            SkTool::MeasureBetween | SkTool::MeasureItem => {
                if inp.pressed[LMB] {
                    let item = pick_item(&data, raw, px * 8.0);
                    let sk = self.sketch.as_mut().unwrap();
                    let Some(item) = item else {
                        self.err("Nothing to measure here: click a point, an element or the H / V axis.");
                        return;
                    };
                    let result = if tool == SkTool::MeasureItem {
                        measure_item(&data, &item)
                    } else if let Some(first) = sk.meas_first.take() {
                        measure_between(&data, &first, &item)
                    } else {
                        let name = item_name(&data, &item);
                        sk.meas_first = Some(item);
                        self.say(&format!("Measure Between: first item {}. Now click the second item.", name));
                        return;
                    };
                    let sk = self.sketch.as_mut().unwrap();
                    match result {
                        Some(m) => {
                            let summary = m.rows.iter().skip(if tool == SkTool::MeasureItem { 1 } else { 2 }).take(2).map(|r| format!("{}: {}", r.0, r.1)).collect::<Vec<_>>().join("   ");
                            if !sk.keep_measures {
                                sk.measures.clear();
                            }
                            sk.measures.push(m);
                            self.say(&summary);
                        }
                        None => self.err("That item has nothing to measure on its own (try Measure Between)."),
                    }
                }
            }
            SkTool::Trim => {
                if inp.pressed[LMB] {
                    match self.sketch_hover_shape() {
                        Some(i) if data.shapes[i].distance(raw) < px * 8.0 => {
                            let mut s = data.clone();
                            match quick_trim(&mut s, i, raw) {
                                Ok(m) => {
                                    self.commit_sketch(s);
                                    self.say(&m);
                                }
                                Err(e) => self.err(&e),
                            }
                        }
                        _ => self.err("Click right on the piece of a line, arc or circle to cut away."),
                    }
                }
            }
            SkTool::Mirror => {
                if inp.pressed[LMB] {
                    let sel = self.sketch.as_ref().unwrap().sel.clone();
                    if sel.is_empty() {
                        self.err("Select the elements to mirror first (Select tool), then pick Mirror.");
                        return;
                    }
                    // the mirror line: a line / axis element, or the sketch H / V axis
                    let mut line: Option<(DVec2, DVec2)> = None;
                    for (i, s) in data.shapes.iter().enumerate() {
                        if sel.contains(&i) {
                            continue;
                        }
                        if let Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } = s {
                            if s.distance(raw) < px * 8.0 {
                                line = Some((DVec2::new(*x1, *y1), DVec2::new(x2 - x1, y2 - y1)));
                            }
                        }
                    }
                    if line.is_none() {
                        if raw.y.abs() < px * 8.0 {
                            line = Some((DVec2::ZERO, DVec2::X));
                        } else if raw.x.abs() < px * 8.0 {
                            line = Some((DVec2::ZERO, DVec2::Y));
                        }
                    }
                    match line {
                        Some((a, dir)) if dir.length() > 1e-9 => {
                            let mut s = data.clone();
                            let new = mirror(&mut s, &sel, a, dir);
                            self.commit_sketch(s);
                            if let Some(sk) = self.sketch.as_mut() {
                                sk.sel = new;
                                sk.tool = SkTool::Select;
                            }
                            self.say("Mirrored. The copies are selected.");
                        }
                        _ => self.err("Click a line, the axis, or the red H / green V axis to mirror across."),
                    }
                }
            }
            _ => self.draw_tool_input(inp, tool, cur),
        }
    }

    fn select_input(&mut self, inp: &Input, cur: DVec2, raw: DVec2) {
        let sk = self.sketch.as_ref().unwrap();
        if let Some(drag) = sk.drag.clone() {
            if inp.down[LMB] {
                match drag {
                    SkDrag::Shapes { start, orig, moved } => {
                        let delta = cur - start;
                        if delta.length() > 1e-12 || moved {
                            if !moved {
                                self.checkpoint();
                                self.sketch.as_mut().unwrap().drag = Some(SkDrag::Shapes { start, orig: orig.clone(), moved: true });
                            }
                            let sel = self.sketch.as_ref().unwrap().sel.clone();
                            if let Some(sd) = self.sketch_data_mut() {
                                sd.shapes = orig.clone();
                                for &i in &sel {
                                    if i < sd.shapes.len() {
                                        let old = sd.shapes[i].handle_points();
                                        sd.shapes[i].translate(delta);
                                        // connected neighbours follow the moved end points
                                        for p in old {
                                            sd.move_point(p, p + delta, &sel);
                                        }
                                    }
                                }
                                // dimensions stay true while dragging
                                sd.solve_dims(None);
                            }
                            self.touch_sketch();
                        }
                    }
                    SkDrag::Point { p0, orig, moved } => {
                        if (cur - p0).length() > 1e-12 || moved {
                            if !moved {
                                self.checkpoint();
                                self.sketch.as_mut().unwrap().drag = Some(SkDrag::Point { p0, orig: orig.clone(), moved: true });
                            }
                            if let Some(sd) = self.sketch_data_mut() {
                                sd.shapes = orig.clone();
                                sd.move_point(p0, cur, &[]);
                                sd.solve_dims(None);
                            }
                            self.touch_sketch();
                        }
                    }
                    SkDrag::Box { .. } | SkDrag::Dim { .. } => {}
                }
            } else {
                if let SkDrag::Box { start } = drag {
                    let (lo, hi) = (start.min(raw), start.max(raw));
                    if (hi - lo).length() > self.sketch_px() * 3.0 {
                        let inside: Vec<usize> = self
                            .sketch_data()
                            .map(|d| {
                                d.shapes
                                    .iter()
                                    .enumerate()
                                    .filter(|(_, s)| s.curve().iter().all(|p| p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y))
                                    .map(|(i, _)| i)
                                    .collect()
                            })
                            .unwrap_or_default();
                        let sk = self.sketch.as_mut().unwrap();
                        for i in inside {
                            if !sk.sel.contains(&i) {
                                sk.sel.push(i);
                            }
                        }
                    }
                }
                self.sketch.as_mut().unwrap().drag = None;
            }
            return;
        }
        if inp.pressed[LMB] {
            let orig = self.sketch_data().map(|d| d.shapes.clone()).unwrap_or_default();
            if let Some(h) = self.handle_near(raw) {
                self.sketch.as_mut().unwrap().drag = Some(SkDrag::Point { p0: h, orig, moved: false });
                return;
            }
            let hover = self.sketch_hover_shape();
            let sk = self.sketch.as_mut().unwrap();
            sk.sel_dim = None;
            match hover {
                Some(i) => {
                    if inp.ctrl {
                        if let Some(k) = sk.sel.iter().position(|x| *x == i) {
                            sk.sel.remove(k);
                        } else {
                            sk.sel.push(i);
                        }
                    } else if !sk.sel.contains(&i) {
                        sk.sel = vec![i];
                    }
                    sk.drag = Some(SkDrag::Shapes { start: cur, orig, moved: false });
                }
                None => {
                    if !inp.ctrl {
                        sk.sel.clear();
                    }
                    sk.drag = Some(SkDrag::Box { start: raw });
                }
            }
        }
    }

    fn draw_tool_input(&mut self, inp: &Input, tool: SkTool, cur: DVec2) {
        let px = self.sketch_px();
        let sk = self.sketch.as_mut().unwrap();
        let finish_key = inp.key(KeyCode::Enter) || inp.key(KeyCode::KpEnter);
        // Profile: connected lines
        if tool == SkTool::Profile {
            if (finish_key || inp.double_click) && !sk.pts.is_empty() {
                sk.pts.clear();
                self.say("Profile finished. Start another, or pick a different tool.");
                return;
            }
            if inp.pressed[LMB] {
                if sk.pts.is_empty() {
                    sk.pts.push(cur);
                    return;
                }
                let last = *sk.pts.last().unwrap();
                if (cur - last).length() < 1e-9 {
                    return;
                }
                let first = sk.pts[0];
                let closing = sk.pts.len() >= 2 && (cur - first).length() < px * 2.0;
                let end = if closing { first } else { cur };
                let added = self.add_shapes(vec![Shape::Line { x1: last.x, y1: last.y, x2: end.x, y2: end.y }], false);
                let sk = self.sketch.as_mut().unwrap();
                if added && closing {
                    sk.pts.clear();
                    self.say("Profile closed.");
                } else if added {
                    sk.pts.push(end);
                }
            }
            return;
        }
        if tool == SkTool::Spline {
            let n = sk.pts.len();
            let close = inp.pressed[LMB] && n >= 3 && (cur - sk.pts[0]).length() < px * 2.0;
            if close || ((finish_key || inp.double_click) && n >= 2) {
                let mut pts = std::mem::take(&mut sk.pts);
                if !close && inp.double_click && pts.len() > 2 && (pts[pts.len() - 1] - pts[pts.len() - 2]).length() < 1e-9 {
                    pts.pop();
                }
                self.add_shapes(vec![Shape::Spline { pts, closed: close }], true);
                return;
            }
            if inp.pressed[LMB] && sk.pts.last().map_or(true, |l| (*l - cur).length() > 1e-9) {
                sk.pts.push(cur);
            }
            return;
        }
        if inp.pressed[LMB] {
            if sk.pts.len() >= tool.needed() {
                let shapes = preview(tool, &sk.pts, cur);
                sk.pts.clear();
                if tool == SkTool::Axis {
                    // only one axis per sketch: replace the old one
                    if let Some(sd) = self.sketch_data_mut() {
                        if let Some(i) = sd.shapes.iter().position(|s| matches!(s, Shape::Axis { .. })) {
                            sd.remove(i);
                        }
                    }
                }
                if !shapes.is_empty() {
                    self.add_shapes(shapes, true);
                }
            } else if sk.pts.last().map_or(true, |l| (*l - cur).length() > 1e-9) {
                sk.pts.push(cur);
            }
        }
    }

    /// The dimension reference under a sketch point: a key point, an element, an axis.
    fn pick_gref(&self, data: &Sketch, raw: DVec2, px: f64) -> Option<GRef> {
        match pick_item(data, raw, px * 8.0)? {
            MeasItem::Point(p) => {
                if p.length() < JOIN_TOL {
                    return Some(GRef::Origin);
                }
                // prefer line end points, then any element's key point
                let mut found = None;
                for (i, s) in data.shapes.iter().enumerate() {
                    for (k, q) in s.key_points().iter().enumerate() {
                        if (*q - p).length() < JOIN_TOL {
                            let is_line_end = matches!(s, Shape::Line { .. } | Shape::Axis { .. }) && k < 2;
                            if found.is_none() || is_line_end {
                                found = Some(GRef::Key(i, k));
                            }
                        }
                    }
                }
                found
            }
            MeasItem::Elem(i) => Some(GRef::Elem(i)),
            MeasItem::AxisH => Some(GRef::AxisH),
            MeasItem::AxisV => Some(GRef::AxisV),
        }
    }

    fn constraint_click(&mut self, raw: DVec2, px: f64) {
        let data = match self.sketch_data() {
            Some(d) => d.clone(),
            None => return,
        };
        let picked = self.pick_gref(&data, raw, px);
        let first = self.sketch.as_ref().unwrap().cons_first;
        let new_dim = match (first, picked) {
            (None, Some(g)) => {
                self.sketch.as_mut().unwrap().cons_first = Some(g);
                self.say("Constraint: click empty space to place the dimension, or click a second element for a distance or angle.");
                return;
            }
            (None, None) => {
                self.err("Click an element, a point or the H / V axis to dimension.");
                return;
            }
            (Some(a), Some(b)) if a != b => {
                // place the text a little to the side of the pair
                let mut d = match data.pair_dim_for(a, b, raw) {
                    Some(d) => d,
                    None => {
                        self.err("Those two cannot be dimensioned together (pick points, lines, circles or an axis).");
                        self.sketch.as_mut().unwrap().cons_first = None;
                        return;
                    }
                };
                if d.kind != DimKind::Angle {
                    if let Some(DimGeom::Linear { n, .. }) = data.dim_geom(&Dim { offset: DVec2::ZERO, ..d.clone() }) {
                        d.offset = n * px * 30.0;
                    }
                } else {
                    d.offset = d.offset.normalize_or_zero() * px * 45.0;
                }
                d
            }
            (Some(a), _) => {
                let i = match a {
                    GRef::Elem(i) => i,
                    _ => {
                        self.err("A point needs a second item: click another point, element or an axis.");
                        return;
                    }
                };
                match data.single_dim_for(i, raw) {
                    Some(d) => d,
                    None => {
                        self.err("That element has no single dimension; click a second item for a distance.");
                        self.sketch.as_mut().unwrap().cons_first = None;
                        return;
                    }
                }
            }
        };
        self.checkpoint();
        let name = new_dim.kind.name();
        let label = new_dim.label();
        if let Some(sd) = self.sketch_data_mut() {
            sd.dims.push(new_dim);
        }
        let n = self.sketch_data().map(|d| d.dims.len()).unwrap_or(1);
        let sk = self.sketch.as_mut().unwrap();
        sk.cons_first = None;
        sk.sel_dim = Some(n - 1);
        self.touch_sketch();
        self.say(&format!("{} constraint {} added. Double-click it to change the value; drag it to move it.", name, label));
    }

    /// Clicking, dragging and double-clicking dimension texts. Returns true when it used the input.
    fn dim_text_input(&mut self, inp: &Input, raw: DVec2) -> bool {
        let sk = self.sketch.as_ref().unwrap();
        if let Some(SkDrag::Dim { idx, start, orig, moved }) = sk.drag.clone() {
            if inp.down[LMB] {
                let delta = raw - start;
                if delta.length() > 1e-12 || moved {
                    if !moved {
                        self.checkpoint();
                        self.sketch.as_mut().unwrap().drag = Some(SkDrag::Dim { idx, start, orig, moved: true });
                    }
                    if let Some(sd) = self.sketch_data_mut() {
                        if let Some(d) = sd.dims.get_mut(idx) {
                            d.offset = orig + delta;
                        }
                    }
                }
            } else {
                self.sketch.as_mut().unwrap().drag = None;
            }
            return true;
        }
        if !inp.pressed[LMB] {
            return false;
        }
        let m = inp.mouse;
        let hit = sk.dim_rects.iter().rev().find(|(_, r)| r.contains(m)).map(|(i, _)| *i);
        let Some(idx) = hit else { return false };
        let orig = match self.sketch_data().and_then(|d| d.dims.get(idx)) {
            Some(d) => d.offset,
            None => return false,
        };
        let sk = self.sketch.as_mut().unwrap();
        sk.sel_dim = Some(idx);
        sk.sel.clear();
        sk.cons_first = None;
        let geometric = self.sketch_data().and_then(|d| d.dims.get(idx)).map_or(false, |d| d.kind.is_geometric());
        let sk = self.sketch.as_mut().unwrap();
        if inp.double_click && geometric {
            sk.drag = None;
            self.say("Horizontal / vertical constraint: press Del to remove it and free the line.");
        } else if inp.double_click {
            sk.drag = None;
            self.open_dim_dialog(idx);
        } else {
            sk.drag = Some(SkDrag::Dim { idx, start: raw, orig, moved: false });
        }
        true
    }

    pub fn open_dim_dialog(&mut self, idx: usize) {
        self.checkpoint();
        self.dialog = Dialog::Dim(idx);
        self.ui.focus = Some("dim_value".into());
        self.ui.edit_buf = self.sketch_data().and_then(|d| d.dims.get(idx)).map(|d| fmt(d.value)).unwrap_or_default();
        self.ui.select_all = true;
    }

    /// Sets a dimension's value and moves the geometry to match.
    pub fn set_dim_value(&mut self, idx: usize, value: f64, reference: bool) -> bool {
        let fid = match self.sketch.as_ref() {
            Some(s) => s.feat,
            None => return false,
        };
        let ok = if let Some(sd) = self.sketch_data_mut() {
            if let Some(d) = sd.dims.get_mut(idx) {
                d.value = value;
                d.reference = reference;
            }
            if reference {
                if let Some(v) = sd.dims.get(idx).and_then(|d| sd.dim_measure(d)) {
                    sd.dims[idx].value = v;
                }
            }
            sd.solve_dims(Some(idx));
            sd.dims.iter().all(|d| sd.dim_ok(d))
        } else {
            false
        };
        if let Some(p) = self.part_mut() {
            p.touch_id(fid);
        }
        ok
    }

    /// Adds elements to the sketch being edited. Returns false if they were invalid.
    pub fn add_shapes(&mut self, shapes: Vec<Shape>, select: bool) -> bool {
        for s in &shapes {
            if s.is_open() {
                let c = s.curve();
                let len: f64 = c.windows(2).map(|w| (w[1] - w[0]).length()).sum();
                if len < 1e-6 {
                    self.err("That element is too small.");
                    return false;
                }
            } else {
                let o = s.outline();
                if o.len() < 3 || signed_area(&o).abs() < 1e-6 {
                    self.err("That shape is too small.");
                    return false;
                }
                if segments_self_intersect(&o) {
                    self.err("A closed shape must not cross itself.");
                    return false;
                }
            }
        }
        self.checkpoint();
        let constr = self.sketch.as_ref().map_or(false, |s| s.construction_mode);
        let name = shapes.first().map(|s| s.kind_name()).unwrap_or("");
        let mut idx = Vec::new();
        if let Some(sd) = self.sketch_data_mut() {
            for s in shapes {
                idx.push(sd.push(s, constr));
            }
        }
        if select {
            if let Some(sk) = self.sketch.as_mut() {
                sk.sel = idx;
            }
        }
        self.touch_sketch();
        if select {
            self.say(&format!("{} added. Type exact sizes on the right, keep drawing, or Exit Sketch.", name));
        }
        true
    }

    /// Replaces the edited sketch's contents (one undo step).
    pub fn commit_sketch(&mut self, s: Sketch) {
        self.checkpoint();
        if let Some(sd) = self.sketch_data_mut() {
            *sd = s;
        }
        if let Some(sk) = self.sketch.as_mut() {
            sk.sel.clear();
        }
        self.touch_sketch();
    }

    pub fn touch_sketch(&mut self) {
        if let Some(id) = self.sketch.as_ref().map(|s| s.feat) {
            if let Some(p) = self.part_mut() {
                p.touch_id(id);
            }
        }
    }

    pub fn delete_sketch_selection(&mut self) {
        if let Some(di) = self.sketch.as_ref().and_then(|s| s.sel_dim) {
            self.checkpoint();
            if let Some(sd) = self.sketch_data_mut() {
                if di < sd.dims.len() {
                    sd.dims.remove(di);
                }
            }
            self.sketch.as_mut().unwrap().sel_dim = None;
            self.touch_sketch();
            self.say("Constraint deleted.");
            return;
        }
        let mut sel = match self.sketch.as_ref() {
            Some(s) if !s.sel.is_empty() => s.sel.clone(),
            _ => return,
        };
        self.checkpoint();
        sel.sort_unstable();
        sel.dedup();
        if let Some(sd) = self.sketch_data_mut() {
            for i in sel.into_iter().rev() {
                sd.remove(i);
            }
        }
        self.sketch.as_mut().unwrap().sel.clear();
        self.touch_sketch();
    }

    pub fn toggle_construction(&mut self) {
        let sel = self.sketch.as_ref().map(|s| s.sel.clone()).unwrap_or_default();
        if sel.is_empty() {
            if let Some(sk) = self.sketch.as_mut() {
                sk.construction_mode = !sk.construction_mode;
                let on = sk.construction_mode;
                self.say(if on { "New elements are construction (helper) elements." } else { "New elements are normal (they make material)." });
            }
            return;
        }
        self.checkpoint();
        if let Some(sd) = self.sketch_data_mut() {
            let make = !sel.iter().all(|i| sd.is_construction(*i));
            for i in sel {
                sd.set_construction(i, make);
            }
        }
        self.touch_sketch();
        self.say("Changed between construction and normal elements.");
    }

    pub fn enter_sketch(&mut self, id: u32) {
        let prev = (self.view.rot, self.view.target, self.view.dist);
        self.dialog = Dialog::None;
        self.sketch = Some(SketchEdit {
            feat: id,
            tool: SkTool::Select,
            pts: vec![],
            sel: vec![],
            drag: None,
            cursor: DVec2::ZERO,
            raw: DVec2::ZERO,
            cursor_ok: false,
            snapped_to_point: false,
            snap_kind: None,
            hv: None,
            prev_view: prev,
            snap: true,
            grid: 1.0,
            construction_mode: false,
            corner_r: 3.0,
            chamfer_d: 2.0,
            measures: vec![],
            meas_first: None,
            keep_measures: false,
            cons_first: None,
            sel_dim: None,
            dim_rects: vec![],
        });
        self.sel = Sel::Feature(id);
        self.hover = None;
        if let Some(f) = self.sketch_frame_world() {
            let rot = quat_from_back_up(f.n().as_vec3(), f.v.as_vec3());
            let shapes = self.sketch_data().map(|s| s.shapes.clone()).unwrap_or_default();
            let mut b = Aabb::empty();
            for s in &shapes {
                for p in s.curve() {
                    b.add(f.to3(p, 0.0));
                }
            }
            let (t, d) = if b.valid() {
                (b.center().as_vec3(), (b.size().length() as f32 * 1.6).max(60.0) / (2.0 * (self.view.fov / 2.0).tan()))
            } else {
                (f.o.as_vec3(), self.view.dist.min(400.0))
            };
            self.view.animate_to(rot, t, d);
        }
        self.say("Sketcher: pick a tool on the right (Profile, Line, Rectangle, Circle, Arc...). Ends snap together.");
    }

    pub fn exit_sketch(&mut self) {
        if let Some(sk) = self.sketch.take() {
            let (r, t, d) = sk.prev_view;
            self.view.animate_to(r, t, d);
            if let Some(p) = self.part_mut() {
                p.touch_id(sk.feat);
            }
            let data = self.part().and_then(|p| p.feature(sk.feat)).and_then(|f| match &f.kind {
                FeatureKind::Sketch(s) => Some(s.clone()),
                _ => None,
            });
            self.sel = Sel::Feature(sk.feat);
            match data {
                Some(s) if s.shapes.is_empty() => self.say("Sketch closed (it has no elements yet)."),
                Some(s) => match s.regions() {
                    Ok(_) => self.say("Sketch done. Now use Pad, Pocket, Shaft or Groove."),
                    Err(e) => self.err(&format!("Sketch check: {}.", e)),
                },
                None => {}
            }
        }
    }

    // ------------------------------------------------------------ drawing

    pub fn draw_sketch_2d(&mut self) {
        let f = match self.sketch_frame_world() {
            Some(f) => f,
            None => return,
        };
        let data = match self.sketch_data() {
            Some(d) => d.clone(),
            None => return,
        };
        let sk = self.sketch.as_ref().unwrap();
        let to_s = |p: DVec2| self.view.to_screen(f.to3(p, 0.0));
        let px = self.view.px_size(f.o.as_vec3()) as f64;
        // grid
        let grid = sk.grid;
        let ext = (self.view.view_height() as f64 * 1.2).max(grid * 10.0);
        let cu = {
            let (o, d) = self.view.ray(vec2(sw() / 2.0, sh() / 2.0));
            let n = f.n();
            let den = d.dot(n);
            let c = if den.abs() > 1e-9 { o + d * ((f.o - o).dot(n) / den) } else { f.o };
            f.to2(c)
        };
        let lines_n = (ext / grid) as i64;
        if lines_n < 400 && grid / px > 4.0 {
            let x0 = ((cu.x - ext) / grid).floor() as i64;
            let y0 = ((cu.y - ext) / grid).floor() as i64;
            for i in 0..=(2 * lines_n) {
                let gx = (x0 + i) as f64 * grid;
                let gy = (y0 + i) as f64 * grid;
                let major = |v: f64| ((v / grid).round() as i64) % 10 == 0;
                let cx = if major(gx) { Color::new(0.6, 0.65, 0.75, 0.30) } else { Color::new(0.6, 0.65, 0.75, 0.11) };
                let cy = if major(gy) { Color::new(0.6, 0.65, 0.75, 0.30) } else { Color::new(0.6, 0.65, 0.75, 0.11) };
                if let (Some(a), Some(b)) = (to_s(DVec2::new(gx, cu.y - ext)), to_s(DVec2::new(gx, cu.y + ext))) {
                    draw_line(a.x, a.y, b.x, b.y, 1.0, cx);
                }
                if let (Some(a), Some(b)) = (to_s(DVec2::new(cu.x - ext, gy)), to_s(DVec2::new(cu.x + ext, gy))) {
                    draw_line(a.x, a.y, b.x, b.y, 1.0, cy);
                }
            }
        }
        let big = ext * 3.0 + cu.length();
        let hcol = Color::new(0.95, 0.35, 0.3, 0.9);
        let vcol = Color::new(0.35, 0.85, 0.4, 0.9);
        if let (Some(a), Some(b)) = (to_s(DVec2::new(-big, 0.0)), to_s(DVec2::new(big, 0.0))) {
            draw_line(a.x, a.y, b.x, b.y, 1.5, hcol);
        }
        if let (Some(a), Some(b)) = (to_s(DVec2::new(0.0, -big)), to_s(DVec2::new(0.0, big))) {
            draw_line(a.x, a.y, b.x, b.y, 1.5, vcol);
        }
        if let Some(o) = to_s(DVec2::ZERO) {
            text("H", o.x + 60.0, o.y - 4.0, 15, hcol);
            text("V", o.x + 4.0, o.y - 60.0, 15, vcol);
        }
        // elements
        let fixed_set = data.fixed_state(None).0;
        let hover = if sk.drag.is_none() && matches!(sk.tool, SkTool::Select | SkTool::Trim | SkTool::Mirror | SkTool::MeasureBetween | SkTool::MeasureItem | SkTool::Constraint) { self.sketch_hover_shape() } else { None };
        for (i, s) in data.shapes.iter().enumerate() {
            let sel = sk.sel.contains(&i);
            let constr = data.is_construction(i);
            let is_axis = matches!(s, Shape::Axis { .. });
            let col = if sel {
                C_SEL
            } else if hover == Some(i) {
                Color::new(1.0, 0.8, 0.45, 1.0)
            } else if fixed_set.contains(&i) && !constr {
                // fully fixed by its constraints (like a fully constrained element)
                Color::new(0.30, 0.92, 0.40, 1.0)
            } else if is_axis {
                Color::new(0.9, 0.85, 0.4, 1.0)
            } else if constr {
                Color::new(0.65, 0.68, 0.75, 1.0)
            } else {
                WHITE
            };
            let c = s.curve();
            let sp: Vec<Vec2> = c.iter().filter_map(|p| to_s(*p)).collect();
            if sp.len() != c.len() || sp.is_empty() {
                continue;
            }
            let closed = !s.is_open();
            let segs = if closed { sp.len() } else { sp.len() - 1 };
            let w = if sel { 3.0 } else { 2.0 };
            let mut dash_pos = 0.0f32;
            for k in 0..segs {
                let (a, b) = (sp[k], sp[(k + 1) % sp.len()]);
                if constr || is_axis {
                    dashed(a, b, w - 0.5, col, &mut dash_pos, if is_axis { 14.0 } else { 6.0 });
                } else {
                    draw_line(a.x, a.y, b.x, b.y, w, col);
                }
            }
            for h in s.handle_points().into_iter().chain(s.endpoints().map(|(a, b)| vec![a, b]).unwrap_or_default()) {
                if let Some(p) = to_s(h) {
                    draw_rectangle(p.x - 2.5, p.y - 2.5, 5.0, 5.0, Color::new(0.85, 0.88, 0.95, 0.9));
                }
            }
            if let Shape::Circle { cx, cy, .. } | Shape::Arc { cx, cy, .. } | Shape::Ellipse { cx, cy, .. } = s {
                if let Some(p) = to_s(DVec2::new(*cx, *cy)) {
                    draw_line(p.x - 4.0, p.y, p.x + 4.0, p.y, 1.0, col);
                    draw_line(p.x, p.y - 4.0, p.x, p.y + 4.0, 1.0, col);
                }
            }
            // dimension label: closed shapes always, open elements when selected or hovered
            if !s.is_open() || sel || hover == Some(i) {
                let at = if s.is_open() {
                    let m = c[c.len() / 2];
                    to_s(m)
                } else {
                    sp.iter().fold(None::<Vec2>, |acc, p| match acc {
                        None => Some(*p),
                        Some(a) => Some(if p.y < a.y - 0.5 || (p.y - a.y).abs() <= 0.5 && p.x > a.x { *p } else { a }),
                    })
                };
                if let Some(top) = at {
                    let label = s.label();
                    let w = text_w(&label, 14) + 8.0;
                    draw_rectangle(top.x + 4.0, top.y - 22.0, w, 17.0, Color::new(0.08, 0.2, 0.38, 0.85));
                    text(&label, top.x + 8.0, top.y - 9.0, 14, Color::new(0.75, 0.95, 1.0, 1.0));
                }
            }
        }
        // unconnected ends in red (the sketch is not closed there)
        for e in data.open_ends() {
            if let Some(p) = to_s(e) {
                draw_circle(p.x, p.y, 4.5, Color::new(1.0, 0.25, 0.2, 1.0));
            }
        }
        // shape being drawn
        if sk.cursor_ok {
            for prev in preview(sk.tool, &sk.pts, sk.cursor) {
                let c = prev.curve();
                let sp: Vec<Vec2> = c.iter().filter_map(|p| to_s(*p)).collect();
                if sp.len() == c.len() && !sp.is_empty() {
                    let closed = !prev.is_open();
                    let segs = if closed { sp.len() } else { sp.len() - 1 };
                    for k in 0..segs {
                        let (a, b) = (sp[k], sp[(k + 1) % sp.len()]);
                        draw_line(a.x, a.y, b.x, b.y, 2.0, Color::new(0.45, 0.85, 1.0, 1.0));
                    }
                    if let Some(p) = sp.last() {
                        text(&prev.label(), p.x + 12.0, p.y + 34.0, 15, Color::new(0.45, 0.85, 1.0, 1.0));
                    }
                }
            }
            for p in &sk.pts {
                if let Some(s) = to_s(*p) {
                    draw_circle_lines(s.x, s.y, 4.0, 1.5, Color::new(0.45, 0.85, 1.0, 1.0));
                }
            }
        }
        // measurements: highlighted items and dimension lines
        let measuring = matches!(sk.tool, SkTool::MeasureBetween | SkTool::MeasureItem);
        let mcol = Color::new(1.0, 0.85, 0.25, 1.0);
        let draw_item = |it: &MeasItem, col: Color| match it {
            MeasItem::Point(p) => {
                if let Some(s) = to_s(*p) {
                    draw_circle_lines(s.x, s.y, 7.0, 2.0, col);
                }
            }
            MeasItem::Elem(i) => {
                if let Some(sh) = data.shapes.get(*i) {
                    let c = sh.curve();
                    let sp: Vec<Vec2> = c.iter().filter_map(|p| to_s(*p)).collect();
                    let n = sp.len();
                    let m = if sh.is_open() { n.saturating_sub(1) } else { n };
                    for k in 0..m {
                        draw_line(sp[k].x, sp[k].y, sp[(k + 1) % n].x, sp[(k + 1) % n].y, 4.0, Color::new(col.r, col.g, col.b, 0.75));
                    }
                }
            }
            MeasItem::AxisH | MeasItem::AxisV => {
                let (a, b) = if *it == MeasItem::AxisH { (DVec2::new(-big, 0.0), DVec2::new(big, 0.0)) } else { (DVec2::new(0.0, -big), DVec2::new(0.0, big)) };
                if let (Some(a), Some(b)) = (to_s(a), to_s(b)) {
                    draw_line(a.x, a.y, b.x, b.y, 3.0, Color::new(col.r, col.g, col.b, 0.75));
                }
            }
        };
        if measuring && sk.cursor_ok && sk.drag.is_none() && !self.ui.over_ui() {
            if let Some(it) = pick_item(&data, sk.raw, px * 8.0) {
                draw_item(&it, Color::new(1.0, 0.8, 0.45, 1.0));
            }
        }
        if let Some(first) = &sk.meas_first {
            draw_item(first, mcol);
        }
        if let Some(first) = &sk.cons_first {
            let it = match *first {
                GRef::Elem(i) => Some(MeasItem::Elem(i)),
                GRef::Key(i, k) => data.shapes.get(i).and_then(|s| s.key_points().get(k).copied()).map(MeasItem::Point),
                GRef::AxisH => Some(MeasItem::AxisH),
                GRef::AxisV => Some(MeasItem::AxisV),
                GRef::Origin => Some(MeasItem::Point(DVec2::ZERO)),
            };
            if let Some(it) = it {
                draw_item(&it, Color::new(0.45, 0.85, 1.0, 1.0));
            }
        }
        if sk.tool == SkTool::Constraint && sk.cursor_ok && sk.drag.is_none() && !self.ui.over_ui() {
            if let Some(it) = pick_item(&data, sk.raw, px * 8.0) {
                draw_item(&it, Color::new(1.0, 0.8, 0.45, 1.0));
            }
        }
        for m in &sk.measures {
            if let Some((a, b)) = m.line {
                if let (Some(sa), Some(sb)) = (to_s(a), to_s(b)) {
                    draw_line(sa.x, sa.y, sb.x, sb.y, 1.5, mcol);
                    let d = sb - sa;
                    if d.length() > 12.0 {
                        let u = d.normalize();
                        let n = vec2(-u.y, u.x);
                        for (tip, dir) in [(sa, u), (sb, -u)] {
                            draw_triangle(tip, tip + dir * 9.0 + n * 3.5, tip + dir * 9.0 - n * 3.5, mcol);
                        }
                        // little extension ticks
                        for t in [sa, sb] {
                            draw_line(t.x - n.x * 6.0, t.y - n.y * 6.0, t.x + n.x * 6.0, t.y + n.y * 6.0, 1.0, mcol);
                        }
                    } else {
                        draw_circle(sa.x, sa.y, 3.0, mcol);
                    }
                }
            }
            if let Some(at) = to_s(m.label_at) {
                let w = text_w(&m.label, 15) + 10.0;
                draw_rectangle(at.x + 6.0, at.y - 24.0, w, 19.0, Color::new(0.12, 0.10, 0.02, 0.92));
                draw_rectangle_lines(at.x + 6.0, at.y - 24.0, w, 19.0, 1.0, mcol);
                text(&m.label, at.x + 11.0, at.y - 10.0, 15, mcol);
            }
        }
        if let (Some(SkDrag::Box { start }), true) = (&sk.drag, sk.cursor_ok) {
            if let (Some(a), Some(b)) = (to_s(*start), to_s(sk.raw)) {
                let (lo, hi) = (a.min(b), a.max(b));
                draw_rectangle(lo.x, lo.y, hi.x - lo.x, hi.y - lo.y, Color::new(0.4, 0.6, 1.0, 0.12));
                draw_rectangle_lines(lo.x, lo.y, hi.x - lo.x, hi.y - lo.y, 1.0, Color::new(0.5, 0.7, 1.0, 0.9));
            }
        }
        // cursor
        if sk.cursor_ok && !self.ui.over_ui() {
            if let Some(c) = to_s(sk.cursor) {
                let col = if sk.snapped_to_point { Color::new(0.2, 1.0, 0.35, 1.0) } else { Color::new(1.0, 1.0, 1.0, 0.9) };
                draw_line(c.x - 9.0, c.y, c.x + 9.0, c.y, 1.0, col);
                draw_line(c.x, c.y - 9.0, c.x, c.y + 9.0, 1.0, col);
                match sk.snap_kind {
                    Some("on curve") => {
                        draw_rectangle_lines(c.x - 5.0, c.y - 5.0, 10.0, 10.0, 1.5, col);
                    }
                    Some("intersection") => {
                        draw_line(c.x - 6.0, c.y - 6.0, c.x + 6.0, c.y + 6.0, 2.0, col);
                        draw_line(c.x - 6.0, c.y + 6.0, c.x + 6.0, c.y - 6.0, 2.0, col);
                    }
                    Some(_) => draw_circle_lines(c.x, c.y, 6.0, 1.5, col),
                    None => {}
                }
                let mut t = format!("H {}  V {}", fmt(sk.cursor.x), fmt(sk.cursor.y));
                if let Some(k) = sk.snap_kind {
                    if k != "point" {
                        t = format!("{}   [{}]", t, k);
                    }
                }
                if let Some(hv) = sk.hv {
                    t = format!("{}   [{}]", t, if hv == "H" { "horizontal" } else { "vertical" });
                }
                text(&t, c.x + 12.0, c.y + 22.0, 14, Color::new(0.85, 0.92, 1.0, 1.0));
            }
        }
    }
}

fn arrow(tip: Vec2, along: Vec2, col: Color) {
    let d = along.normalize_or_zero();
    let n = vec2(-d.y, d.x);
    draw_triangle(tip, tip - d * 9.0 + n * 3.2, tip - d * 9.0 - n * 3.2, col);
}

impl App {
    /// Draws the sketch's dimensions like classic CAD constraints: extension lines, a
    /// dimension line with arrows and the value, which can be dragged and double-clicked.
    pub fn draw_dims(&mut self) {
        let f = match self.sketch_frame_world() {
            Some(f) => f,
            None => return,
        };
        let data = match self.sketch_data() {
            Some(d) => d.clone(),
            None => return,
        };
        let px = self.view.px_size(f.o.as_vec3()) as f64;
        let sk = self.sketch.as_ref().unwrap();
        let to_s = |p: DVec2| self.view.to_screen(f.to3(p, 0.0));
        let mut list: Vec<(Option<usize>, Dim)> = data.dims.iter().cloned().enumerate().map(|(i, d)| (Some(i), d)).collect();
        // preview of the dimension being placed
        if sk.tool == SkTool::Constraint && sk.cursor_ok && !self.ui.over_ui() {
            if let Some(first) = sk.cons_first {
                let under = self.pick_gref(&data, sk.raw, px);
                let prev = match under {
                    Some(b) if b != first => data.pair_dim_for(first, b, sk.raw).map(|mut d| {
                        if d.kind != DimKind::Angle {
                            if let Some(DimGeom::Linear { n, .. }) = data.dim_geom(&Dim { offset: DVec2::ZERO, ..d.clone() }) {
                                d.offset = n * px * 30.0;
                            }
                        } else {
                            d.offset = d.offset.normalize_or_zero() * px * 45.0;
                        }
                        d
                    }),
                    _ => match first {
                        GRef::Elem(i) => data.single_dim_for(i, sk.raw),
                        _ => None,
                    },
                };
                if let Some(p) = prev {
                    list.push((None, p));
                }
            }
        }
        let mut rects = Vec::new();
        for (idx, d) in &list {
            let g = match data.dim_geom(d) {
                Some(g) => g,
                None => continue,
            };
            let ok = data.dim_ok(d);
            let selected = idx.is_some() && sk.sel_dim == *idx;
            let col = if idx.is_none() {
                Color::new(0.45, 0.85, 1.0, 1.0)
            } else if selected {
                C_SEL
            } else if !ok {
                Color::new(0.95, 0.35, 0.85, 1.0)
            } else if d.reference {
                Color::new(0.7, 0.74, 0.8, 1.0)
            } else {
                Color::new(0.35, 0.95, 0.5, 1.0)
            };
            let text_at = match &g {
                DimGeom::Linear { p, q, n, o, text } => {
                    let (p2, q2) = (*p + *n * *o, *q + *n * *o);
                    let (Some(sp), Some(sq), Some(sp2), Some(sq2), Some(st)) = (to_s(*p), to_s(*q), to_s(p2), to_s(q2), to_s(*text)) else { continue };
                    // extension lines, a little past the dimension line
                    for (a, b) in [(sp, sp2), (sq, sq2)] {
                        let d = (b - a).normalize_or_zero();
                        if (b - a).length() > 1.0 {
                            draw_line(a.x + d.x * 3.0, a.y + d.y * 3.0, b.x + d.x * 5.0, b.y + d.y * 5.0, 1.0, col);
                        }
                    }
                    draw_line(sp2.x, sp2.y, sq2.x, sq2.y, 1.2, col);
                    let along = sq2 - sp2;
                    if along.length() > 24.0 {
                        arrow(sp2, -along, col);
                        arrow(sq2, along, col);
                    } else {
                        // short dimension: arrows from outside
                        arrow(sp2, along, col);
                        arrow(sq2, -along, col);
                    }
                    // reach out to the text when it was dragged beyond the ends
                    let l = along.length_squared();
                    if l > 1e-6 {
                        let t = (st - sp2).dot(along) / l;
                        let foot = sp2 + along * t;
                        if t < 0.0 {
                            draw_line(sp2.x, sp2.y, foot.x, foot.y, 1.0, col);
                        } else if t > 1.0 {
                            draw_line(sq2.x, sq2.y, foot.x, foot.y, 1.0, col);
                        }
                    }
                    st
                }
                DimGeom::Radial { c, r, dir, diameter, text } => {
                    let edge = *c + *dir * *r;
                    let (Some(sc), Some(se), Some(st)) = (to_s(*c), to_s(edge), to_s(*text)) else { continue };
                    if *diameter {
                        let Some(so) = to_s(*c - *dir * *r) else { continue };
                        draw_line(so.x, so.y, se.x, se.y, 1.2, col);
                        arrow(se, se - so, col);
                        arrow(so, so - se, col);
                    } else {
                        draw_line(sc.x, sc.y, se.x, se.y, 1.2, col);
                        arrow(se, se - sc, col);
                    }
                    if (st - sc).length() > (se - sc).length() {
                        draw_line(se.x, se.y, st.x, st.y, 1.0, col);
                    }
                    st
                }
                DimGeom::Angular { v, d1, d2, r, text } => {
                    let a1 = d1.y.atan2(d1.x);
                    let mut delta = d2.y.atan2(d2.x) - a1;
                    while delta > std::f64::consts::PI {
                        delta -= std::f64::consts::TAU;
                    }
                    while delta < -std::f64::consts::PI {
                        delta += std::f64::consts::TAU;
                    }
                    let pts: Vec<Vec2> = (0..=24)
                        .filter_map(|k| {
                            let a = a1 + delta * k as f64 / 24.0;
                            to_s(*v + DVec2::new(a.cos(), a.sin()) * *r)
                        })
                        .collect();
                    if pts.len() < 2 {
                        continue;
                    }
                    for w in pts.windows(2) {
                        draw_line(w[0].x, w[0].y, w[1].x, w[1].y, 1.2, col);
                    }
                    arrow(pts[0], pts[0] - pts[1], col);
                    arrow(pts[pts.len() - 1], pts[pts.len() - 1] - pts[pts.len() - 2], col);
                    // extension lines along both legs out to the arc
                    if let (Some(sv), Some(e1), Some(e2)) = (to_s(*v), to_s(*v + *d1 * *r), to_s(*v + *d2 * *r)) {
                        for e in [e1, e2] {
                            draw_line(sv.x, sv.y, e.x, e.y, 0.8, Color::new(col.r, col.g, col.b, 0.5));
                        }
                    }
                    let Some(st) = to_s(*text) else { continue };
                    st
                }
                DimGeom::Tag { at } => {
                    let Some(sa) = to_s(*at) else { continue };
                    // just beside the line's middle
                    sa + vec2(10.0, -12.0)
                }
            };
            let label = d.label();
            let w = text_w(&label, 15) + 10.0;
            let rr = if d.kind.is_geometric() { Rect::new(text_at.x - 7.0, text_at.y - 8.0, 14.0, 16.0) } else { Rect::new(text_at.x - w / 2.0, text_at.y - 10.0, w, 19.0) };
            draw_rectangle(rr.x, rr.y, rr.w, rr.h, Color::new(0.06, 0.07, 0.09, if selected { 0.95 } else { 0.78 }));
            if selected {
                draw_rectangle_lines(rr.x, rr.y, rr.w, rr.h, 1.0, col);
            }
            if d.kind.is_geometric() {
                text(&label, rr.x + 3.0, rr.y + 12.0, 13, col);
            } else {
                text(&label, rr.x + 5.0, rr.y + 14.0, 15, col);
            }
            if let Some(i) = idx {
                rects.push((*i, rr));
            }
        }
        for (i, r) in &rects {
            self.ui.reg(&format!("dim_{}", i), *r);
            if sk.sel_dim == Some(*i) {
                self.ui.reg("dim_sel", *r);
            }
        }
        if let Some(sk) = self.sketch.as_mut() {
            sk.dim_rects = rects;
        }
    }
}

fn dashed(a: Vec2, b: Vec2, w: f32, col: Color, pos: &mut f32, on: f32) {
    let len = (b - a).length();
    if len < 1e-3 {
        return;
    }
    let dir = (b - a) / len;
    let period = on + 5.0;
    let mut t = 0.0;
    while t < len {
        let phase = (*pos + t) % period;
        if phase < on {
            let e = (t + (on - phase)).min(len);
            let p = a + dir * t;
            let q = a + dir * e;
            draw_line(p.x, p.y, q.x, q.y, w, col);
            t = e;
        } else {
            t += period - phase;
        }
    }
    *pos += len;
}
