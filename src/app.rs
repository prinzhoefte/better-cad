//! Application state, the three workbenches (Part Design, Sketcher, Assembly Design),
//! 3D rendering of the documents and viewport interaction.

use crate::files::*;
use crate::geom::*;
use crate::geom::Mesh;
use crate::model::*;
use crate::ui::*;
use crate::view::*;
pub use crate::sketcher::*;
use glam::{DMat4, DQuat, DVec2, DVec3};
use macroquad::prelude::*;
use std::path::PathBuf;
use std::sync::Arc;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Sel {
    None,
    Root,
    Plane(u8),
    Feature(u32),
    Comp(u32),
    Constraint(usize),
    /// a face of the part being edited (local point and normal, triangle index)
    Face { tri: u32, p: DVec3, n: DVec3 },
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Nav {
    None,
    Pan,
    Rotate,
    Zoom,
    Compass,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Tool {
    None,
    PickSketchSupport,
    PickHolePoint,
    Measure(Option<DVec3>),
    Move,
    Constrain(CKind, Option<(u32, GeomRef, DVec3)>),
    Fix,
}

pub enum Dialog {
    None,
    Feature { id: u32, new: bool, backup: Option<Feature> },
    Comp(u32),
    Constraint(usize),
    File(FileDialog),
    Help,
    Options,
    /// "Constraint Definition" of a sketch dimension
    Dim(usize),
    /// colour and finish of a component (Some) or of the part being edited (None)
    Appearance(Option<u32>),
}

#[derive(Clone)]
pub struct Hit {
    pub comp: Option<u32>,
    pub tri: usize,
    pub point: DVec3,
    pub mesh: Arc<Mesh>,
    pub model: DMat4,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Handle {
    Axis(usize),
    Ring(usize),
    Free,
}

#[derive(Clone, Copy, Debug)]
pub struct GizmoDrag {
    pub comp: u32,
    pub handle: Handle,
    pub start_pos: DVec3,
    pub start_rot: DQuat,
    pub start_param: f64,
    pub start_vec: DVec3,
    pub center: DVec3,
    pub start_mouse: Vec2,
}

pub struct App {
    pub doc: Doc,
    pub path: Option<String>,
    pub dirty: bool,
    pub active_comp: Option<u32>,
    pub sel: Sel,
    pub tool: Tool,
    pub sketch: Option<SketchEdit>,
    pub dialog: Dialog,
    pub view: View,
    pub ui: Ui,
    pub rend: Renderer,
    pub status: String,
    pub status_err: bool,
    pub undo: Vec<Doc>,
    pub redo: Vec<Doc>,
    pub menu_open: Option<usize>,
    pub ctx_menu: Option<(Vec2, Sel)>,
    pub show_edges: bool,
    /// realistic rendering with the parts' surface finishes (instead of technical shading)
    pub realistic: bool,
    pub gizmo_drag: Option<GizmoDrag>,
    pub gizmo_hover: Option<Handle>,
    pub hover: Option<Hit>,
    pub hover_plane: Option<u8>,
    pub hover_refplane: Option<u32>,
    pub toolbar_scroll: f32,
    /// a part is being rebuilt in the background
    pub building: bool,
    pub hover_edge: Option<usize>,
    /// the body before the dress-up feature being edited: (feature, body pointer, mesh, sharp edges)
    pub pre_mesh: Option<(u32, usize, Arc<Mesh>, Arc<Vec<SharpEdge>>)>,
    pub last_dir: PathBuf,
    pub measure_result: Option<(DVec3, DVec3)>,
    pub orbit_drag: bool,
    pub nav: Nav,
    pub nav_moved: f32,
    pub rmb_moved: f32,
    pub last_mouse: Vec2,
    pub quit: bool,
    pub tree_rows: usize,
    pub frame_no: u64,
    pub part_counter: u32,
}

impl App {
    pub fn new() -> App {
        let docs = std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Documents")).filter(|p| p.is_dir()).or_else(|| std::env::var_os("HOME").map(PathBuf::from)).unwrap_or_else(|| PathBuf::from("."));
        let mut a = App {
            doc: Doc::Part(Part::new("Part1")),
            path: None,
            dirty: false,
            active_comp: None,
            sel: Sel::None,
            tool: Tool::None,
            sketch: None,
            dialog: Dialog::None,
            view: View::new(),
            ui: Ui::new(),
            rend: Renderer::new(),
            status: "Select a plane and click Sketch to start drawing, or add a Box / Cylinder / Sphere.".into(),
            status_err: false,
            undo: vec![],
            redo: vec![],
            menu_open: None,
            ctx_menu: None,
            show_edges: true,
            realistic: false,
            gizmo_drag: None,
            gizmo_hover: None,
            hover: None,
            hover_plane: None,
            hover_refplane: None,
            toolbar_scroll: 0.0,
            building: false,
            hover_edge: None,
            pre_mesh: None,
            last_dir: docs,
            measure_result: None,
            orbit_drag: false,
            nav: Nav::None,
            nav_moved: 0.0,
            rmb_moved: 0.0,
            last_mouse: Vec2::ZERO,
            quit: false,
            tree_rows: 0,
            frame_no: 0,
            part_counter: 1,
        };
        a.view.fit(&Aabb { min: DVec3::splat(-50.0), max: DVec3::splat(50.0) }, None);
        a.view.anim = None;
        a.view.update(1.0);
        a.view.dist = 260.0;
        a
    }

    // ------------------------------------------------------------ document helpers

    pub fn part(&self) -> Option<&Part> {
        match &self.doc {
            Doc::Part(p) => Some(p),
            Doc::Product(pr) => self.active_comp.and_then(|id| pr.comp(id)).map(|c| &c.part),
        }
    }
    pub fn part_mut(&mut self) -> Option<&mut Part> {
        match &mut self.doc {
            Doc::Part(p) => Some(p),
            Doc::Product(pr) => match self.active_comp {
                Some(id) => pr.comp_mut(id).map(|c| &mut c.part),
                None => None,
            },
        }
    }
    pub fn product(&self) -> Option<&Product> {
        match &self.doc {
            Doc::Product(p) => Some(p),
            _ => None,
        }
    }
    pub fn product_mut(&mut self) -> Option<&mut Product> {
        match &mut self.doc {
            Doc::Product(p) => Some(p),
            _ => None,
        }
    }
    /// Placement of the part being edited, in world space.
    pub fn part_model(&self) -> DMat4 {
        match (&self.doc, self.active_comp) {
            (Doc::Product(pr), Some(id)) => pr.comp(id).map(|c| c.matrix()).unwrap_or(DMat4::IDENTITY),
            _ => DMat4::IDENTITY,
        }
    }
    pub fn workbench(&self) -> &'static str {
        if self.sketch.is_some() {
            "Sketcher"
        } else if self.part().is_some() {
            "Part Design"
        } else {
            "Assembly Design"
        }
    }

    pub fn checkpoint(&mut self) {
        self.undo.push(self.doc.clone());
        if self.undo.len() > 60 {
            self.undo.remove(0);
        }
        self.redo.clear();
        self.dirty = true;
    }
    pub fn do_undo(&mut self) {
        if let Some(d) = self.undo.pop() {
            self.redo.push(std::mem::replace(&mut self.doc, d));
            self.after_history();
            self.say("Undone.");
        }
    }
    pub fn do_redo(&mut self) {
        if let Some(d) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.doc, d));
            self.after_history();
            self.say("Redone.");
        }
    }
    fn after_history(&mut self) {
        self.dialog = Dialog::None;
        self.gizmo_drag = None;
        if let Some(sk) = &self.sketch {
            let id = sk.feat;
            let exists = self.part().map_or(false, |p| p.feature(id).is_some());
            if !exists {
                self.sketch = None;
            } else if let Some(s) = self.sketch.as_mut() {
                s.sel.clear();
                s.drag = None;
            }
        }
        if let (Some(id), Some(pr)) = (self.active_comp, self.product()) {
            if pr.comp(id).is_none() {
                self.active_comp = None;
            }
        }
        if matches!(self.doc, Doc::Part(_)) {
            self.active_comp = None;
        }
    }

    pub fn say(&mut self, s: &str) {
        self.status = s.to_string();
        self.status_err = false;
    }
    pub fn err(&mut self, s: &str) {
        self.status = s.to_string();
        self.status_err = true;
    }

    pub fn scene_bbox(&mut self) -> Aabb {
        let mut b = Aabb::empty();
        match &mut self.doc {
            Doc::Part(p) => {
                let m = p.mesh_live();
                b.merge(&m.bbox);
            }
            Doc::Product(pr) => {
                for c in &mut pr.comps {
                    if c.hidden {
                        continue;
                    }
                    let m = c.part.mesh_live();
                    if m.bbox.valid() {
                        let mx = c.matrix();
                        for i in 0..8 {
                            let p = DVec3::new(
                                if i & 1 == 0 { m.bbox.min.x } else { m.bbox.max.x },
                                if i & 2 == 0 { m.bbox.min.y } else { m.bbox.max.y },
                                if i & 4 == 0 { m.bbox.min.z } else { m.bbox.max.z },
                            );
                            b.add(mx.transform_point3(p));
                        }
                    }
                }
            }
        }
        b
    }

    pub fn fit_all(&mut self) {
        let b = self.scene_bbox();
        let b = if b.valid() { b } else { Aabb { min: DVec3::splat(-50.0), max: DVec3::splat(50.0) } };
        self.view.fit(&b, None);
    }

    pub fn set_view(&mut self, back: Vec3, up: Vec3) {
        let b = self.scene_bbox();
        let rot = quat_from_back_up(back, up);
        if b.valid() {
            self.view.fit(&b, Some(rot));
        } else {
            let (t, d) = (self.view.target, self.view.dist);
            self.view.animate_to(rot, t, d);
        }
    }

    // ------------------------------------------------------------ per frame

    pub fn frame(&mut self, inp: Input, dt: f32) {
        self.frame_no += 1;
        self.ui.begin(inp);
        self.view.update(dt);
        self.rend.begin_frame();
        // rebuild geometry that changed
        // parts rebuild in the background; the window keeps showing the last result meanwhile
        self.building = match &mut self.doc {
            Doc::Part(p) => p.poll_build(),
            Doc::Product(pr) => {
                let mut busy = false;
                for c in &mut pr.comps {
                    busy |= c.part.poll_build();
                }
                busy
            }
        };
        let b = self.scene_bbox();
        if b.valid() {
            self.view.scene_radius = (b.size().length() as f32 * 0.5).max(self.view.scene_radius * 0.0 + 20.0).max(b.center().length() as f32);
        }

        clear_background(WHITE);
        set_ui_camera();
        draw_background(self.realistic);
        set_camera(&self.view.camera());
        self.rend.set_lights(&self.view);
        self.rend.set_view_scale(&self.view);
        self.draw_scene();
        set_ui_camera();
        self.draw_overlays_2d();

        // interface
        self.draw_ui();

        // viewport interaction
        let modal = matches!(self.dialog, Dialog::File(_) | Dialog::Help | Dialog::Options) || self.menu_open.is_some() || self.ctx_menu.is_some();
        let over = self.ui.over_ui();
        if !modal {
            self.viewport_input(over);
        } else {
            self.hover = None;
        }
        self.shortcuts();
        self.ui.draw_tooltip();
        self.last_mouse = self.ui.inp.mouse;
    }

    // ------------------------------------------------------------ drawing the scene

    fn draw_scene(&mut self) {
        let model_part = self.part_model();
        let sel_tint = vec4(1.0, 0.55, 0.1, 0.45);
        // realistic rendering shows the finishes and no edge lines (like shading with material)
        let real = self.realistic;
        let show_edges = self.show_edges && !real;
        let view_ptr: *const View = &self.view;
        // Part document or part being edited
        match &mut self.doc {
            Doc::Part(p) => {
                let m = p.mesh_live();
                let col = p.color;
                if real {
                    self.rend.draw_real(&m, col, p.finish, &DMat4::IDENTITY, vec4(0.0, 0.0, 0.0, 0.0));
                } else {
                    self.rend.draw(&m, col, &DMat4::IDENTITY, vec4(0.0, 0.0, 0.0, 0.0));
                }
                if show_edges {
                    self.rend.draw_edges(&m, col, &DMat4::IDENTITY, unsafe { &*view_ptr }, Color::new(0.08, 0.08, 0.12, 1.0));
                }
            }
            Doc::Product(pr) => {
                let active = self.active_comp;
                for c in &mut pr.comps {
                    if c.hidden {
                        continue;
                    }
                    let m = c.part.mesh_live();
                    let mx = c.matrix();
                    let selected = self.sel == Sel::Comp(c.id);
                    let tint = if active.is_some() && active != Some(c.id) {
                        vec4(0.85, 0.87, 0.9, 0.65)
                    } else if selected {
                        sel_tint
                    } else {
                        vec4(0.0, 0.0, 0.0, 0.0)
                    };
                    if real {
                        self.rend.draw_real(&m, c.part.color, c.part.finish, &mx, tint);
                    } else {
                        self.rend.draw(&m, c.part.color, &mx, tint);
                    }
                    if show_edges {
                        let ec = if active.is_some() && active != Some(c.id) { Color::new(0.5, 0.52, 0.56, 1.0) } else { Color::new(0.08, 0.08, 0.12, 1.0) };
                        self.rend.draw_edges(&m, c.part.color, &mx, unsafe { &*view_ptr }, ec);
                    }
                }
            }
        }
        // highlighted face under the mouse
        if let Some(h) = self.hover.clone() {
            let tris = self.hover_tris(&h);
            self.rend.draw_overlay(&*h.mesh, &tris, &h.model, [1.0, 0.62, 0.15]);
        }
        // selected face
        if let Sel::Face { tri, p, n } = self.sel {
            let model = self.part_model();
            let mesh = self.part().and_then(|pt| pt.cache.mesh.clone());
            match mesh {
                Some(m) if (tri as usize) < m.tris.len() && m.tri_normal[tri as usize].dot(n) > 0.999 && (m.tri_pts(tri as usize)[0] - p).dot(n).abs() < 1e-3 => {
                    let g = m.face[tri as usize];
                    let tris: Vec<usize> = (0..m.tris.len()).filter(|t| m.face[*t] == g).collect();
                    self.rend.draw_overlay(&m, &tris, &model, [1.0, 0.5, 0.05]);
                }
                _ => self.sel = Sel::None,
            }
        }
        // constraint first pick
        if let Tool::Constrain(_, Some((_, r, _))) = self.tool {
            self.draw_ref(r, Color::new(1.0, 0.3, 0.1, 1.0));
        }
        // planes and visible sketches of the active part
        if self.part().is_some() {
            self.draw_ref_planes(&model_part);
            self.draw_planes(&model_part);
            self.draw_sketches_3d(&model_part);
        }
        // constraint symbols
        if let Some(pr) = self.product() {
            let mut lines = vec![];
            for (i, c) in pr.constraints.iter().enumerate() {
                if let (Some(a), Some(b)) = (pr.comp(c.a), pr.comp(c.b)) {
                    let pa = ref_point(&c.ra.transformed(&a.matrix()));
                    let pb = ref_point(&c.rb.transformed(&b.matrix()));
                    let sel = self.sel == Sel::Constraint(i);
                    lines.push((pa, pb, sel));
                }
            }
            for (a, b, sel) in lines {
                let col = if sel { C_SEL } else { Color::new(0.1, 0.55, 0.15, 1.0) };
                draw_line_3d(a.as_vec3(), b.as_vec3(), col);
            }
        }
        if let Some((a, b)) = self.measure_result {
            draw_line_3d(a.as_vec3(), b.as_vec3(), Color::new(0.9, 0.1, 0.1, 1.0));
        }
        if let Tool::Measure(Some(a)) = self.tool {
            if let Some(h) = &self.hover {
                draw_line_3d(a.as_vec3(), h.point.as_vec3(), Color::new(0.9, 0.3, 0.1, 1.0));
            }
        }
    }

    fn draw_ref(&self, r: GeomRef, col: Color) {
        match r {
            GeomRef::Plane { p, n } => {
                let k = self.view.px_size(p.as_vec3()) as f64 * 40.0;
                draw_line_3d(p.as_vec3(), (p + n * k).as_vec3(), col);
            }
            GeomRef::Axis { p, d } => {
                let k = self.view.px_size(p.as_vec3()) as f64 * 80.0;
                draw_line_3d((p - d * k).as_vec3(), (p + d * k).as_vec3(), col);
            }
        }
    }

    pub fn plane_size(&self) -> f64 {
        let r = self.part().and_then(|p| p.cache.mesh.as_ref().map(|m| m.bbox)).filter(|b| b.valid()).map(|b| b.size().max_element() * 0.6).unwrap_or(40.0);
        r.max(40.0)
    }

    pub fn plane_frame(i: u8) -> Frame {
        match i {
            0 => Frame::xy(),
            1 => Frame::yz(),
            _ => Frame::zx(),
        }
    }

    fn draw_planes(&self, model: &DMat4) {
        let s = self.plane_size();
        let hidden_planes = self.sketch.is_some();
        for i in 0..3u8 {
            let f = Self::plane_frame(i);
            let hot = self.hover_plane == Some(i) || self.sel == Sel::Plane(i);
            if hidden_planes && !hot {
                continue;
            }
            let col = if hot { C_SEL } else { Color::new(0.85, 0.65, 0.20, 0.9) };
            let c = [DVec2::new(-s, -s), DVec2::new(s, -s), DVec2::new(s, s), DVec2::new(-s, s)];
            for k in 0..4 {
                let a = model.transform_point3(f.to3(c[k], 0.0)).as_vec3();
                let b = model.transform_point3(f.to3(c[(k + 1) % 4], 0.0)).as_vec3();
                draw_line_3d(a, b, col);
            }
            if hot {
                for t in 1..8 {
                    let x = -s + 2.0 * s * t as f64 / 8.0;
                    let a = model.transform_point3(f.to3(DVec2::new(x, -s), 0.0)).as_vec3();
                    let b = model.transform_point3(f.to3(DVec2::new(x, s), 0.0)).as_vec3();
                    draw_line_3d(a, b, Color::new(col.r, col.g, col.b, 0.35));
                }
            }
        }
    }

    fn draw_sketches_3d(&self, model: &DMat4) {
        let part = match self.part() {
            Some(p) => p,
            None => return,
        };
        let editing = self.sketch.as_ref().map(|s| s.feat);
        for f in &part.features {
            if let FeatureKind::Sketch(sk) = &f.kind {
                if Some(f.id) == editing {
                    continue; // drawn in 2D on top
                }
                let selected = self.sel == Sel::Feature(f.id);
                if f.hidden && !selected {
                    continue;
                }
                let col = if selected { C_SEL } else { Color::new(0.1, 0.1, 0.1, 1.0) };
                for s in &sk.shapes {
                    let o = s.outline();
                    for i in 0..o.len() {
                        let a = model.transform_point3(sk.frame.to3(o[i], 0.0)).as_vec3();
                        let b = model.transform_point3(sk.frame.to3(o[(i + 1) % o.len()], 0.0)).as_vec3();
                        draw_line_3d(a, b, col);
                    }
                }
            }
        }
    }

    fn draw_overlays_2d(&mut self) {
        // sketch being edited
        if self.sketch.is_some() {
            self.draw_sketch_2d();
            self.draw_dims();
        }
        self.draw_dressup_picks();
        // move gizmo
        if self.tool == Tool::Move {
            if let Some((c, axes)) = self.gizmo_geometry() {
                let hot = self.gizmo_drag.map(|g| g.handle).or(self.gizmo_hover);
                let cols = [Color::new(0.85, 0.15, 0.15, 1.0), Color::new(0.15, 0.65, 0.15, 1.0), Color::new(0.15, 0.3, 0.9, 1.0)];
                if let Some(cs) = self.view.to_screen(c) {
                    for k in 0..3 {
                        // rings
                        let ring = self.ring_points(c, axes[k]);
                        let rc = if hot == Some(Handle::Ring(k)) { C_SEL } else { Color::new(cols[k].r, cols[k].g, cols[k].b, 0.7) };
                        for i in 0..ring.len() {
                            let a = ring[i];
                            let b = ring[(i + 1) % ring.len()];
                            draw_line(a.x, a.y, b.x, b.y, if hot == Some(Handle::Ring(k)) { 3.0 } else { 2.0 }, rc);
                        }
                    }
                    for k in 0..3 {
                        let tip = c + axes[k] * self.gizmo_len(c);
                        if let Some(ts) = self.view.to_screen(tip) {
                            let col = if hot == Some(Handle::Axis(k)) { C_SEL } else { cols[k] };
                            draw_line(cs.x, cs.y, ts.x, ts.y, 4.0, col);
                            let d = (ts - cs).normalize_or_zero();
                            let p = vec2(-d.y, d.x);
                            draw_triangle(ts + d * 12.0, ts + p * 6.0, ts - p * 6.0, col);
                            text(["X", "Y", "Z"][k], ts.x + d.x * 18.0 - 4.0, ts.y + d.y * 18.0 + 5.0, 16, col);
                        }
                    }
                    draw_circle(cs.x, cs.y, 5.0, if hot == Some(Handle::Free) { C_SEL } else { WHITE });
                    draw_circle_lines(cs.x, cs.y, 5.0, 1.5, BLACK);
                }
            }
        }
        // constraint labels
        if let Some(pr) = self.product() {
            let mut labels = vec![];
            for (i, c) in pr.constraints.iter().enumerate() {
                if let (Some(a), Some(b)) = (pr.comp(c.a), pr.comp(c.b)) {
                    let pa = ref_point(&c.ra.transformed(&a.matrix()));
                    let pb = ref_point(&c.rb.transformed(&b.matrix()));
                    let err = constraint_error(c, a, b) > 1e-3 && !(a.fixed && b.fixed);
                    labels.push(((pa + pb) * 0.5, c.name.clone(), i, err));
                }
            }
            for (p, name, i, err) in labels {
                if let Some(s) = self.view.to_screen(p) {
                    let sel = self.sel == Sel::Constraint(i);
                    let w = text_w(&name, 14) + 8.0;
                    draw_rectangle(s.x - w / 2.0, s.y - 9.0, w, 17.0, if sel { C_ACTIVE } else if err { Color::new(0.35, 0.1, 0.1, 0.9) } else { Color::new(0.08, 0.22, 0.1, 0.88) });
                    draw_rectangle_lines(s.x - w / 2.0, s.y - 9.0, w, 17.0, 1.0, if err { C_ERR } else { Color::new(0.35, 0.8, 0.4, 1.0) });
                    text(&name, s.x - w / 2.0 + 4.0, s.y + 4.0, 14, C_TEXT);
                }
            }
        }
        // fixed anchors
        if let Some(pr) = self.product() {
            let mut anchors = vec![];
            for c in &pr.comps {
                if c.fixed && !c.hidden {
                    if let Some(m) = c.part.cache.mesh.as_ref() {
                        if m.bbox.valid() {
                            anchors.push(c.matrix().transform_point3(DVec3::new(m.bbox.min.x, m.bbox.min.y, m.bbox.min.z)));
                        }
                    }
                }
            }
            for a in anchors {
                if let Some(s) = self.view.to_screen(a) {
                    draw_anchor(s);
                }
            }
        }
        // plane names
        if self.part().is_some() && self.sketch.is_none() {
            let s = self.plane_size();
            let m = self.part_model();
            let names = ["xy plane", "yz plane", "zx plane"];
            for i in 0..3u8 {
                let f = Self::plane_frame(i);
                if let Some(p) = self.view.to_screen(m.transform_point3(f.to3(DVec2::new(-s, s), 0.0))) {
                    text(names[i as usize], p.x + 3.0, p.y + 14.0, 14, Color::new(0.95, 0.75, 0.35, 1.0));
                }
            }
        }
        // measurement
        if let Some((a, b)) = self.measure_result {
            if let (Some(sa), Some(sb)) = (self.view.to_screen(a), self.view.to_screen(b)) {
                let d = b - a;
                let t = format!("{} mm  (dx {}  dy {}  dz {})", fmt(d.length()), fmt(d.x), fmt(d.y), fmt(d.z));
                let m = (sa + sb) * 0.5;
                let w = text_w(&t, 16) + 10.0;
                draw_rectangle(m.x - w / 2.0, m.y - 26.0, w, 20.0, Color::new(0.1, 0.1, 0.12, 0.92));
                draw_rectangle_lines(m.x - w / 2.0, m.y - 26.0, w, 20.0, 1.0, C_ERR);
                text(&t, m.x - w / 2.0 + 5.0, m.y - 11.0, 16, C_TEXT);
                draw_circle(sa.x, sa.y, 3.5, C_ERR);
                draw_circle(sb.x, sb.y, 3.5, C_ERR);
            }
        }
        if let Tool::Measure(Some(a)) = self.tool {
            if let Some(s) = self.view.to_screen(a) {
                draw_circle(s.x, s.y, 4.0, C_ERR);
            }
        }
        self.draw_triad();
    }

    fn draw_triad(&self) {
        // axis triad, bottom right of the viewport
        let o = vec2(sw() - crate::panels::TOOLBAR_W - 60.0, sh() - crate::panels::STATUS_H - 80.0);
        let axes = [(Vec3::X, "x", Color::new(1.0, 0.35, 0.3, 1.0)), (Vec3::Y, "y", Color::new(0.35, 0.9, 0.4, 1.0)), (Vec3::Z, "z", Color::new(0.4, 0.6, 1.0, 1.0))];
        let inv = self.view.rot.inverse();
        let mut order: Vec<_> = axes.iter().map(|(a, n, c)| (inv * *a, *n, *c)).collect();
        order.sort_by(|a, b| a.0.z.partial_cmp(&b.0.z).unwrap());
        for (v, n, c) in order {
            let e = o + vec2(v.x, -v.y) * 30.0;
            draw_line(o.x, o.y, e.x, e.y, 2.0, c);
            text(n, e.x + v.x * 8.0 - 4.0, e.y - v.y * 8.0 + 5.0, 15, c);
        }
        self.draw_compass();
    }

    /// Orientation compass in the top right of the viewport (drag it to rotate the view).
    fn draw_compass(&self) {
        let r = self.compass_rect();
        let o = vec2(r.x + r.w * 0.45, r.y + r.h * 0.62);
        let inv = self.view.rot.inverse();
        let proj = |v: Vec3| -> Vec2 {
            let c = inv * v;
            o + vec2(c.x, -c.y) * 38.0
        };
        let hot = self.ui.hover(r) || self.nav == Nav::Compass;
        let base = if hot { Color::new(1.0, 0.75, 0.35, 1.0) } else { Color::new(0.75, 0.78, 0.85, 0.9) };
        // the xy square
        let sq = [vec3(0.0, 0.0, 0.0), vec3(0.45, 0.0, 0.0), vec3(0.45, 0.45, 0.0), vec3(0.0, 0.45, 0.0)];
        let sp: Vec<Vec2> = sq.iter().map(|v| proj(*v)).collect();
        draw_triangle(sp[0], sp[1], sp[2], Color::new(base.r, base.g, base.b, 0.18));
        draw_triangle(sp[0], sp[2], sp[3], Color::new(base.r, base.g, base.b, 0.18));
        for k in 0..4 {
            draw_line(sp[k].x, sp[k].y, sp[(k + 1) % 4].x, sp[(k + 1) % 4].y, 1.0, base);
        }
        // quarter arcs in each coordinate plane
        for (a, b) in [(Vec3::X, Vec3::Y), (Vec3::Y, Vec3::Z), (Vec3::Z, Vec3::X)] {
            let mut prev = proj(a * 0.8);
            for i in 1..=12 {
                let t = i as f32 / 12.0 * std::f32::consts::FRAC_PI_2;
                let p = proj((a * t.cos() + b * t.sin()) * 0.8);
                draw_line(prev.x, prev.y, p.x, p.y, 1.0, Color::new(base.r, base.g, base.b, 0.6));
                prev = p;
            }
        }
        for (v, n) in [(Vec3::X, "x"), (Vec3::Y, "y"), (Vec3::Z, "z")] {
            let e = proj(v);
            draw_line(o.x, o.y, e.x, e.y, 2.0, base);
            let d = (e - o).normalize_or_zero();
            let p = vec2(-d.y, d.x);
            draw_triangle(e + d * 7.0, e + p * 3.5, e - p * 3.5, base);
            text(n, e.x + d.x * 12.0 - 3.0, e.y + d.y * 12.0 + 5.0, 15, base);
        }
        draw_circle(o.x, o.y, 2.5, base);
    }

    // ------------------------------------------------------------ sketch drawing

    pub fn sketch_frame_world(&self) -> Option<Frame> {
        let sk = self.sketch.as_ref()?;
        let part = self.part()?;
        if let Some(FeatureKind::Sketch(s)) = part.feature(sk.feat).map(|f| &f.kind) {
            let m = self.part_model();
            let o = m.transform_point3(s.frame.o);
            let u = m.transform_vector3(s.frame.u).normalize();
            let v = m.transform_vector3(s.frame.v).normalize();
            return Some(Frame { o, u, v });
        }
        None
    }

    pub fn sketch_data(&self) -> Option<&Sketch> {
        let id = self.sketch.as_ref()?.feat;
        match self.part()?.feature(id).map(|f| &f.kind) {
            Some(FeatureKind::Sketch(s)) => Some(s),
            _ => None,
        }
    }

    pub fn sketch_data_mut(&mut self) -> Option<&mut Sketch> {
        let id = self.sketch.as_ref()?.feat;
        match self.part_mut()?.feature_mut(id).map(|f| &mut f.kind) {
            Some(FeatureKind::Sketch(s)) => Some(s),
            _ => None,
        }
    }




    // ------------------------------------------------------------ hit testing

    fn hover_tris(&self, h: &Hit) -> Vec<usize> {
        let m = &h.mesh;
        let use_smooth = matches!(self.tool, Tool::Constrain(CKind::Coincidence, _));
        if use_smooth {
            let g = m.smooth[h.tri];
            (0..m.tris.len()).filter(|t| m.smooth[*t] == g).collect()
        } else if matches!(self.tool, Tool::Measure(_)) {
            vec![]
        } else {
            let g = m.face[h.tri];
            (0..m.tris.len()).filter(|t| m.face[*t] == g).collect()
        }
    }

    /// Ray-cast the visible geometry. In Part Design only the active part is hit.
    fn pick(&mut self, mouse: Vec2) -> Option<Hit> {
        let (o, d) = self.view.ray(mouse);
        let mut best: Option<(f64, Hit)> = None;
        let active = self.active_comp;
        match &mut self.doc {
            Doc::Part(p) => {
                let m = p.mesh_live();
                if let Some((dist, t)) = m.raycast(o, d) {
                    best = Some((dist, Hit { comp: None, tri: t, point: o + d * dist, mesh: m, model: DMat4::IDENTITY }));
                }
            }
            Doc::Product(pr) => {
                for c in &mut pr.comps {
                    if c.hidden || (active.is_some() && active != Some(c.id)) {
                        continue;
                    }
                    let mx = c.matrix();
                    let inv = mx.inverse();
                    let lo = inv.transform_point3(o);
                    let ld = inv.transform_vector3(d);
                    let m = c.part.mesh_live();
                    if let Some((dist, t)) = m.raycast(lo, ld) {
                        let wp = mx.transform_point3(lo + ld * dist);
                        let wd = (wp - o).length();
                        if best.as_ref().map_or(true, |b| wd < b.0) {
                            best = Some((wd, Hit { comp: Some(c.id), tri: t, point: wp, mesh: m, model: mx }));
                        }
                    }
                }
            }
        }
        best.map(|b| b.1)
    }

    fn pick_plane(&self, mouse: Vec2) -> Option<(u8, f64)> {
        let (o, d) = self.view.ray(mouse);
        let m = self.part_model();
        let inv = m.inverse();
        let (o, d) = (inv.transform_point3(o), inv.transform_vector3(d));
        let s = self.plane_size();
        let mut best: Option<(u8, f64)> = None;
        for i in 0..3u8 {
            let f = Self::plane_frame(i);
            let n = f.n();
            let den = d.dot(n);
            if den.abs() < 1e-9 {
                continue;
            }
            let t = (f.o - o).dot(n) / den;
            if t <= 0.0 {
                continue;
            }
            let p = f.to2(o + d * t);
            // only the border region of a plane is pickable, so the inside stays free
            let inside = p.x.abs() <= s && p.y.abs() <= s;
            let px = self.view.px_size(self.view.target) as f64;
            let near_border = (s - p.x.abs()).min(s - p.y.abs()) < px * 10.0;
            if inside && near_border && best.map_or(true, |b| t < b.1) {
                best = Some((i, t));
            }
        }
        best
    }

    /// Measuring in 3D: snap to a corner of the part when the click is close to one.
    fn snap_to_corner(&self, h: &Hit, mouse: Vec2) -> DVec3 {
        let mut best: Option<(f32, DVec3)> = None;
        for p in h.mesh.tri_pts(h.tri) {
            let w = h.model.transform_point3(p);
            if let Some(s) = self.view.to_screen(w) {
                let d = (s - mouse).length();
                if d < 12.0 && best.map_or(true, |b| d < b.0) {
                    best = Some((d, w));
                }
            }
        }
        best.map(|b| b.1).unwrap_or(h.point)
    }

    /// Re-centres the view on the point under the mouse (middle click).
    fn centre_on(&mut self, mouse: Vec2) {
        let p = match self.pick_any(mouse) {
            Some(p) => p,
            None => {
                let (o, d) = self.view.ray(mouse);
                let n = self.view.back().as_dvec3();
                let t = (self.view.target.as_dvec3() - o).dot(n) / d.dot(n);
                o + d * t
            }
        };
        let (r, dist) = (self.view.rot, self.view.dist);
        self.view.animate_to(r, p.as_vec3(), dist);
    }

    /// The first visible surface point under the mouse, whatever is being edited.
    fn pick_any(&mut self, mouse: Vec2) -> Option<DVec3> {
        let saved = self.active_comp;
        if self.sketch.is_none() {
            self.active_comp = None;
        }
        let h = self.pick(mouse).map(|h| h.point);
        self.active_comp = saved;
        h
    }

    pub fn compass_rect(&self) -> Rect {
        Rect::new(sw() - crate::panels::TOOLBAR_W - 130.0, crate::panels::MENU_H + 10.0, 110.0, 110.0)
    }

    fn viewport_context_menu(&mut self, mouse: Vec2) {
        if self.part().is_some() {
            if let Some(h) = self.pick(mouse) {
                let p = h.model.inverse().transform_point3(h.point);
                self.sel = Sel::Face { tri: h.tri as u32, p, n: h.mesh.tri_normal[h.tri] };
                self.ctx_menu = Some((mouse, self.sel));
            }
        } else if let Some(h) = self.pick(mouse) {
            if let Some(id) = h.comp {
                self.sel = Sel::Comp(id);
                self.ctx_menu = Some((mouse, self.sel));
            }
        }
    }

    pub fn sketch_on_face(&mut self, p: DVec3, n: DVec3) {
        let f = Frame::from_normal(p, n);
        self.start_sketch_on(f, "face".into());
    }

    pub fn hole_on_face(&mut self, p: DVec3, n: DVec3) {
        self.tool = Tool::None;
        self.checkpoint();
        if let Some(part) = self.part_mut() {
            let id = part.add_feature(FeatureKind::Hole { pos: p, dir: -n, diameter: 6.0, depth: 10.0, through: false, htype: 0, d2: 0.0, depth2: 0.0, angle2: 0.0 });
            self.open_feature_dialog(id, true, None);
        }
    }

    // ------------------------------------------------------------ reference planes

    /// Reference plane features of the part being edited: (id, placement, name, hidden).
    pub fn ref_planes(&self) -> Vec<(u32, Frame, String, bool)> {
        let p = match self.part() {
            Some(p) => p,
            None => return vec![],
        };
        p.features.iter().filter_map(|f| p.ref_plane_frame(f.id).map(|fr| (f.id, fr, f.name.clone(), f.hidden))).collect()
    }

    fn pick_ref_plane(&self, mouse: Vec2) -> Option<(u32, f64)> {
        let (o, d) = self.view.ray(mouse);
        let inv = self.part_model().inverse();
        let (o, d) = (inv.transform_point3(o), inv.transform_vector3(d));
        let s = self.plane_size() * 0.8;
        let px = self.view.px_size(self.view.target) as f64;
        let mut best: Option<(u32, f64)> = None;
        for (id, f, _, hidden) in self.ref_planes() {
            if hidden {
                continue;
            }
            let n = f.n();
            let den = d.dot(n);
            if den.abs() < 1e-9 {
                continue;
            }
            let t = (f.o - o).dot(n) / den;
            if t <= 0.0 {
                continue;
            }
            let p = f.to2(o + d * t);
            let inside = p.x.abs() <= s && p.y.abs() <= s;
            let near_border = (s - p.x.abs()).min(s - p.y.abs()) < px * 10.0;
            if inside && near_border && best.map_or(true, |b| t < b.1) {
                best = Some((id, t));
            }
        }
        best
    }

    pub fn sketch_on_ref_plane(&mut self, id: u32) {
        let (f, name) = match self.part().and_then(|p| p.ref_plane_frame(id).map(|f| (f, p.feature(id).map(|x| x.name.clone()).unwrap_or_default()))) {
            Some(x) => x,
            None => return,
        };
        self.start_sketch_on(f, name);
    }

    fn draw_ref_planes(&self, model: &DMat4) {
        let s = self.plane_size() * 0.8;
        for (id, f, _, hidden) in self.ref_planes() {
            let hot = self.hover_refplane == Some(id) || self.sel == Sel::Feature(id);
            if hidden && !hot {
                continue;
            }
            let col = if hot { C_SEL } else { Color::new(0.95, 0.55, 0.35, 0.9) };
            let c = [DVec2::new(-s, -s), DVec2::new(s, -s), DVec2::new(s, s), DVec2::new(-s, s)];
            for k in 0..4 {
                let a = model.transform_point3(f.to3(c[k], 0.0)).as_vec3();
                let b = model.transform_point3(f.to3(c[(k + 1) % 4], 0.0)).as_vec3();
                draw_line_3d(a, b, col);
            }
        }
    }

    // ------------------------------------------------------------ dress-up picking

    /// The dress-up feature whose definition is open: (id, picks edges rather than faces).
    pub fn dressup_target(&self) -> Option<(u32, bool)> {
        let id = match self.dialog {
            Dialog::Feature { id, .. } => id,
            _ => return None,
        };
        match &self.part()?.feature(id)?.kind {
            FeatureKind::Fillet { .. } | FeatureKind::Chamfer { .. } => Some((id, true)),
            FeatureKind::Draft { .. } | FeatureKind::Shell { .. } | FeatureKind::Thickness { .. } => Some((id, false)),
            _ => None,
        }
    }

    /// Mesh and sharp edges of the body just before feature `id`.
    pub fn pre_mesh_for(&mut self, id: u32) -> Option<(Arc<Mesh>, Arc<Vec<SharpEdge>>)> {
        let body = {
            let p = self.part()?;
            let idx = p.index_of(id)?;
            if idx == 0 {
                return None;
            }
            p.cache.bodies.get(idx - 1)?.clone()
        };
        let ptr = Arc::as_ptr(&body) as usize;
        if let Some((fid, bp, m, e)) = &self.pre_mesh {
            if *fid == id && *bp == ptr {
                return Some((m.clone(), e.clone()));
            }
        }
        let m = Arc::new(Mesh::from_solid(&body));
        let e = Arc::new(sharp_edges(&m));
        self.pre_mesh = Some((id, ptr, m.clone(), e.clone()));
        Some((m, e))
    }

    fn dressup_input(&mut self, fid: u32, edges_mode: bool, inp: &Input) {
        let Some((mesh, sharp)) = self.pre_mesh_for(fid) else { return };
        let model = self.part_model();
        self.hover = None;
        self.hover_edge = None;
        if edges_mode {
            let mut best: (f32, Option<usize>) = (10.0, None);
            for (i, e) in sharp.iter().enumerate() {
                if let (Some(a), Some(b)) = (self.view.to_screen(model.transform_point3(e.a)), self.view.to_screen(model.transform_point3(e.b))) {
                    let d = dist_seg_2d(inp.mouse, a, b);
                    if d < best.0 {
                        best = (d, Some(i));
                    }
                }
            }
            self.hover_edge = best.1;
            if inp.pressed[LMB] {
                if let Some(i) = best.1 {
                    if let Some(p) = self.part_mut() {
                        if let Some(f) = p.feature_mut(fid) {
                            if let FeatureKind::Fillet { edges, .. } | FeatureKind::Chamfer { edges, .. } = &mut f.kind {
                                let before = edges.len();
                                edges.retain(|r| find_edge(&sharp, r).map_or(true, |k| !edge_chain(&sharp, k).contains(&i)));
                                if edges.len() == before {
                                    edges.push(edge_ref(&sharp[i]));
                                }
                            }
                        }
                        p.touch_id(fid);
                    }
                }
            }
        } else {
            let (o, d) = self.view.ray(inp.mouse);
            let inv = model.inverse();
            if let Some((dist, t)) = mesh.raycast(inv.transform_point3(o), inv.transform_vector3(d)) {
                let lp = inv.transform_point3(o) + inv.transform_vector3(d) * dist;
                self.hover = Some(Hit { comp: None, tri: t, point: model.transform_point3(lp), mesh: mesh.clone(), model });
                if inp.pressed[LMB] {
                    let fr = FaceRef { p: lp, n: mesh.tri_normal[t] };
                    let group = mesh.face[t];
                    if let Some(p) = self.part_mut() {
                        if let Some(f) = p.feature_mut(fid) {
                            if let FeatureKind::Draft { faces, .. } | FeatureKind::Shell { faces, .. } | FeatureKind::Thickness { faces, .. } = &mut f.kind {
                                let before = faces.len();
                                faces.retain(|r| find_face(&mesh, r) != Some(group));
                                if faces.len() == before {
                                    faces.push(fr);
                                }
                            }
                        }
                        p.touch_id(fid);
                    }
                }
            }
        }
    }

    /// Picked edges / faces of the open dress-up feature, drawn over the part.
    fn draw_dressup_picks(&mut self) {
        let Some((fid, edges_mode)) = self.dressup_target() else { return };
        let Some((mesh, sharp)) = self.pre_mesh_for(fid) else { return };
        let model = self.part_model();
        let kind = self.part().and_then(|p| p.feature(fid)).map(|f| f.kind.clone());
        if edges_mode {
            let refs = match &kind {
                Some(FeatureKind::Fillet { edges, .. }) | Some(FeatureKind::Chamfer { edges, .. }) => edges.clone(),
                _ => vec![],
            };
            let mut segs: Vec<(usize, Color)> = Vec::new();
            let picked: Vec<usize> = refs.iter().filter_map(|r| find_edge(&sharp, r)).collect();
            segs.extend(edge_chains(&sharp, &picked).into_iter().map(|k| (k, C_SEL)));
            if let Some(h) = self.hover_edge {
                segs.extend(edge_chain(&sharp, h).into_iter().map(|k| (k, Color::new(0.45, 0.85, 1.0, 1.0))));
            }
            for (k, col) in segs {
                let e = sharp[k];
                if let (Some(a), Some(b)) = (self.view.to_screen(model.transform_point3(e.a)), self.view.to_screen(model.transform_point3(e.b))) {
                    draw_line(a.x, a.y, b.x, b.y, 3.5, col);
                }
            }
        } else {
            let refs = match &kind {
                Some(FeatureKind::Draft { faces, .. }) | Some(FeatureKind::Shell { faces, .. }) | Some(FeatureKind::Thickness { faces, .. }) => faces.clone(),
                _ => vec![],
            };
            // outline the picked faces (their boundary edges) in orange
            for r in &refs {
                if let Some(g) = find_face(&mesh, r) {
                    let mut count: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
                    for (t, tri) in mesh.tris.iter().enumerate() {
                        if mesh.face[t] != g {
                            continue;
                        }
                        for k in 0..3 {
                            let (x, y) = (tri[k], tri[(k + 1) % 3]);
                            *count.entry((x.min(y), x.max(y))).or_default() += 1;
                        }
                    }
                    for ((x, y), c) in count {
                        if c == 1 {
                            if let (Some(a), Some(b)) = (self.view.to_screen(model.transform_point3(mesh.pos[x as usize])), self.view.to_screen(model.transform_point3(mesh.pos[y as usize]))) {
                                draw_line(a.x, a.y, b.x, b.y, 3.0, C_SEL);
                            }
                        }
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------ gizmo

    fn gizmo_geometry(&self) -> Option<(DVec3, [DVec3; 3])> {
        let id = match self.sel {
            Sel::Comp(id) => id,
            _ => return None,
        };
        let pr = self.product()?;
        let c = pr.comp(id)?;
        if c.fixed {
            return None;
        }
        let m = c.part.cache.mesh.as_ref()?;
        let center = if m.bbox.valid() { c.matrix().transform_point3(m.bbox.center()) } else { c.pos };
        Some((center, [DVec3::X, DVec3::Y, DVec3::Z]))
    }
    fn gizmo_len(&self, c: DVec3) -> f64 {
        self.view.px_size(c.as_vec3()) as f64 * 80.0
    }
    fn ring_points(&self, c: DVec3, axis: DVec3) -> Vec<Vec2> {
        let r = self.gizmo_len(c) * 0.62;
        let u = axis.any_orthonormal_vector();
        let v = axis.cross(u);
        (0..48)
            .filter_map(|i| {
                let a = std::f64::consts::TAU * i as f64 / 48.0;
                self.view.to_screen(c + (u * a.cos() + v * a.sin()) * r)
            })
            .collect()
    }
    fn gizmo_hit(&self, m: Vec2) -> Option<Handle> {
        let (c, axes) = self.gizmo_geometry()?;
        let cs = self.view.to_screen(c)?;
        if (m - cs).length() < 9.0 {
            return Some(Handle::Free);
        }
        let mut best: Option<(f32, Handle)> = None;
        for k in 0..3 {
            if let Some(ts) = self.view.to_screen(c + axes[k] * self.gizmo_len(c)) {
                let d = dist_seg_2d(m, cs, ts + (ts - cs).normalize_or_zero() * 12.0);
                if d < 8.0 && best.map_or(true, |b| d < b.0) {
                    best = Some((d, Handle::Axis(k)));
                }
            }
            let ring = self.ring_points(c, axes[k]);
            for i in 0..ring.len() {
                let d = dist_seg_2d(m, ring[i], ring[(i + 1) % ring.len()]);
                if d < 6.0 && best.map_or(true, |b| d < b.0) {
                    best = Some((d, Handle::Ring(k)));
                }
            }
        }
        best.map(|b| b.1)
    }

    fn axis_param(&self, mouse: Vec2, c: DVec3, axis: DVec3) -> f64 {
        // closest point between the mouse ray and the axis line
        let (o, d) = self.view.ray(mouse);
        let w0 = c - o;
        let a = axis.dot(axis);
        let b = axis.dot(d);
        let cc = d.dot(d);
        let dd = axis.dot(w0);
        let e = d.dot(w0);
        let den = a * cc - b * b;
        if den.abs() < 1e-12 {
            return 0.0;
        }
        (b * e - cc * dd) / den
    }

    fn ring_vec(&self, mouse: Vec2, c: DVec3, axis: DVec3) -> Option<DVec3> {
        let (o, d) = self.view.ray(mouse);
        let den = d.dot(axis);
        if den.abs() < 1e-6 {
            return None;
        }
        let t = (c - o).dot(axis) / den;
        let p = o + d * t;
        let v = p - c;
        let v = v - axis * v.dot(axis);
        if v.length() < 1e-9 { None } else { Some(v.normalize()) }
    }

    // ------------------------------------------------------------ viewport input

    fn viewport_input(&mut self, over_ui: bool) {
        let inp = self.ui.inp.clone();
        let delta = inp.mouse - self.last_mouse;
        // Mouse navigation as in classic CAD (CATIA V5 style):
        //  middle drag = pan; hold middle + hold left (or right) = rotate;
        //  hold middle, click left (or right) once, keep middle down and move = zoom (up = closer);
        //  middle click = centre the view on that point; left drag on the compass = rotate.
        if inp.pressed[MMB] && !over_ui {
            self.nav = Nav::Pan;
            self.nav_moved = 0.0;
        }
        if inp.pressed[LMB] && self.nav == Nav::None && self.compass_rect().contains(inp.mouse) {
            self.nav = Nav::Compass;
        }
        match self.nav {
            Nav::Compass => {
                if inp.down[LMB] {
                    self.view.orbit(delta.x, delta.y);
                } else {
                    self.nav = Nav::None;
                }
            }
            Nav::None => {}
            _ => {
                if !inp.down[MMB] {
                    if self.nav == Nav::Pan && self.nav_moved < 4.0 {
                        self.centre_on(inp.mouse);
                    }
                    self.nav = Nav::None;
                } else {
                    if inp.down[LMB] || inp.down[RMB] {
                        self.nav = Nav::Rotate;
                    } else if self.nav == Nav::Rotate {
                        self.nav = Nav::Zoom;
                    }
                    self.nav_moved += delta.length();
                    if delta.length() > 0.0 {
                        match self.nav {
                            Nav::Pan => self.view.pan(delta.x, delta.y),
                            Nav::Rotate => self.view.orbit(delta.x, delta.y),
                            Nav::Zoom => self.view.zoom_center(delta.y),
                            _ => {}
                        }
                    }
                }
            }
        }
        self.orbit_drag = self.nav != Nav::None;
        if !over_ui && inp.wheel != 0.0 {
            self.view.zoom_at(inp.wheel, inp.mouse);
        }
        if self.nav != Nav::None {
            self.hover = None;
            return;
        }
        // right click (without dragging) opens a context menu on what is under the mouse
        if inp.pressed[RMB] {
            self.rmb_moved = 0.0;
        }
        if inp.down[RMB] {
            self.rmb_moved += delta.length();
        }
        if inp.released[RMB] && self.rmb_moved < 4.0 && !over_ui && self.sketch.is_none() {
            self.viewport_context_menu(inp.mouse);
            return;
        }
        if over_ui && self.gizmo_drag.is_none() && self.sketch.as_ref().map_or(true, |s| s.drag.is_none()) {
            self.hover = None;
            self.hover_plane = None;
            return;
        }
        if self.sketch.is_some() {
            self.sketch_input(&inp);
            return;
        }
        if let Some((fid, edges_mode)) = self.dressup_target() {
            self.dressup_input(fid, edges_mode, &inp);
            return;
        }
        // hover
        let pre_highlight = self.tool == Tool::None && self.part().is_some();
        let want_faces = pre_highlight || matches!(self.tool, Tool::PickSketchSupport | Tool::PickHolePoint | Tool::Measure(_) | Tool::Constrain(..));
        self.hover = None;
        self.hover_plane = None;
        self.hover_refplane = None;
        if self.tool == Tool::PickSketchSupport {
            if let Some((id, t)) = self.pick_ref_plane(inp.mouse) {
                let h = self.pick(inp.mouse);
                let face_d = h.as_ref().map(|h| (h.point - self.view.ray(inp.mouse).0).length()).unwrap_or(f64::MAX);
                if t < face_d + 1e-6 {
                    self.hover_refplane = Some(id);
                }
            }
        }
        if self.tool == Tool::PickSketchSupport && self.hover_refplane.is_none() {
            if let Some((pl, t)) = self.pick_plane(inp.mouse) {
                let h = self.pick(inp.mouse);
                let face_d = h.as_ref().map(|h| (h.point - self.view.ray(inp.mouse).0).length()).unwrap_or(f64::MAX);
                if t < face_d + 1e-6 {
                    self.hover_plane = Some(pl);
                }
            }
        }
        if want_faces && self.hover_plane.is_none() && self.gizmo_drag.is_none() {
            self.hover = self.pick(inp.mouse);
        }
        // gizmo
        if self.tool == Tool::Move {
            if self.gizmo_drag.is_none() {
                self.gizmo_hover = self.gizmo_hit(inp.mouse);
            }
            if inp.pressed[LMB] {
                if let Some(h) = self.gizmo_hover {
                    self.start_gizmo(h, inp.mouse);
                    return;
                }
            }
            if let Some(g) = self.gizmo_drag {
                if inp.down[LMB] {
                    self.drag_gizmo(g, inp.mouse, inp.ctrl);
                } else {
                    self.gizmo_drag = None;
                    if let Some(pr) = self.product_mut() {
                        let probs = pr.solve();
                        if !probs.is_empty() {
                            let m = probs.join("; ");
                            self.err(&m);
                        }
                    }
                }
                return;
            }
        }
        if inp.pressed[LMB] {
            self.viewport_click(&inp);
        }
        if inp.double_click && self.tool == Tool::None {
            if let Doc::Product(_) = self.doc {
                if self.active_comp.is_none() {
                    if let Some(h) = self.pick(inp.mouse) {
                        if let Some(id) = h.comp {
                            self.edit_component(id);
                        }
                    }
                }
            }
        }
    }

    fn start_gizmo(&mut self, h: Handle, mouse: Vec2) {
        let (c, axes) = match self.gizmo_geometry() {
            Some(g) => g,
            None => return,
        };
        let id = match self.sel {
            Sel::Comp(id) => id,
            _ => return,
        };
        let (pos, rot) = match self.product().and_then(|p| p.comp(id)) {
            Some(cc) => (cc.pos, cc.rot),
            None => return,
        };
        self.checkpoint();
        let mut g = GizmoDrag { comp: id, handle: h, start_pos: pos, start_rot: rot, start_param: 0.0, start_vec: DVec3::ZERO, center: c, start_mouse: mouse };
        match h {
            Handle::Axis(k) => g.start_param = self.axis_param(mouse, c, axes[k]),
            Handle::Ring(k) => g.start_vec = self.ring_vec(mouse, c, axes[k]).unwrap_or(DVec3::X),
            Handle::Free => {}
        }
        self.gizmo_drag = Some(g);
    }

    fn drag_gizmo(&mut self, g: GizmoDrag, mouse: Vec2, snap: bool) {
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        let (mut new_pos, mut new_rot) = (g.start_pos, g.start_rot);
        match g.handle {
            Handle::Axis(k) => {
                let mut d = self.axis_param(mouse, g.center, axes[k]) - g.start_param;
                if snap {
                    d = d.round();
                }
                new_pos = g.start_pos + axes[k] * d;
            }
            Handle::Ring(k) => {
                if let Some(v) = self.ring_vec(mouse, g.center, axes[k]) {
                    let mut ang = g.start_vec.cross(v).dot(axes[k]).atan2(g.start_vec.dot(v));
                    if snap {
                        let s = 15f64.to_radians();
                        ang = (ang / s).round() * s;
                    }
                    let q = DQuat::from_axis_angle(axes[k], ang);
                    new_rot = (q * g.start_rot).normalize();
                    new_pos = g.center + q * (g.start_pos - g.center);
                }
            }
            Handle::Free => {
                let k = self.view.px_size(g.center.as_vec3()) as f64;
                let dm = mouse - g.start_mouse;
                let mut d = self.view.right().as_dvec3() * dm.x as f64 * k - self.view.up().as_dvec3() * dm.y as f64 * k;
                if snap {
                    d = d.round();
                }
                new_pos = g.start_pos + d;
            }
        }
        if let Some(c) = self.product_mut().and_then(|p| p.comp_mut(g.comp)) {
            c.pos = new_pos;
            c.rot = new_rot;
        }
    }

    fn viewport_click(&mut self, inp: &Input) {
        match self.tool {
            Tool::PickSketchSupport => {
                if let Some(id) = self.hover_refplane {
                    self.sketch_on_ref_plane(id);
                } else if let Some(pl) = self.hover_plane {
                    self.start_sketch_on(Self::plane_frame(pl), ["xy plane", "yz plane", "zx plane"][pl as usize].to_string());
                } else if let Some(h) = self.hover.clone() {
                    let m = &h.mesh;
                    let (_, n) = m.face_plane(h.tri);
                    let is_planar = m.face.iter().filter(|f| **f == m.face[h.tri]).count() >= 1;
                    let lp = h.model.inverse().transform_point3(h.point);
                    if is_planar {
                        let f = Frame::from_normal(lp, n);
                        self.start_sketch_on(f, "face".into());
                    }
                } else {
                    self.say("Click one of the three planes (on its border) or a flat face of the part.");
                }
            }
            Tool::PickHolePoint => {
                if let Some(h) = self.hover.clone() {
                    let lp = h.model.inverse().transform_point3(h.point);
                    let n = h.mesh.tri_normal[h.tri];
                    self.tool = Tool::None;
                    self.checkpoint();
                    let id = self.part_mut().unwrap().add_feature(FeatureKind::Hole { pos: lp, dir: -n, diameter: 6.0, depth: 10.0, through: false, htype: 0, d2: 0.0, depth2: 0.0, angle2: 0.0 });
                    self.open_feature_dialog(id, true, None);
                }
            }
            Tool::Measure(first) => {
                if let Some(mut h) = self.hover.clone() {
                    h.point = self.snap_to_corner(&h, inp.mouse);
                    match first {
                        None => {
                            self.tool = Tool::Measure(Some(h.point));
                            self.measure_result = None;
                            self.say("Measure: click the second point.");
                        }
                        Some(a) => {
                            self.measure_result = Some((a, h.point));
                            self.tool = Tool::Measure(None);
                            let d = h.point - a;
                            self.say(&format!("Distance {} mm. Click two more points to measure again, Esc to stop.", fmt(d.length())));
                        }
                    }
                }
            }
            Tool::Constrain(kind, first) => {
                if let Some(h) = self.hover.clone() {
                    let comp = match h.comp {
                        Some(c) => c,
                        None => return,
                    };
                    let r = if kind == CKind::Coincidence {
                        match h.mesh.face_axis(h.tri) {
                            Some((p, d, _)) => GeomRef::Axis { p, d },
                            None => {
                                self.err("Coincidence needs a round face (a hole or a pin). Click a curved surface.");
                                return;
                            }
                        }
                    } else {
                        let (p, n) = h.mesh.face_plane(h.tri);
                        GeomRef::Plane { p, n }
                    };
                    match first {
                        None => {
                            self.tool = Tool::Constrain(kind, Some((comp, r, h.point)));
                            self.say(&format!("{}: now click a face of another component.", kind.name()));
                        }
                        Some((ca, ra, _)) => {
                            if ca == comp {
                                self.err("Pick the second face on a different component.");
                                return;
                            }
                            self.add_constraint(kind, ca, ra, comp, r);
                        }
                    }
                }
            }
            Tool::Fix => {
                if let Some(h) = self.pick(inp.mouse) {
                    if let Some(id) = h.comp {
                        self.toggle_fix(id);
                    }
                }
                self.tool = Tool::None;
            }
            Tool::None if self.part().is_some() => match self.hover.clone() {
                Some(h) => {
                    let inv = h.model.inverse();
                    let p = inv.transform_point3(h.point);
                    let n = h.mesh.tri_normal[h.tri];
                    self.sel = Sel::Face { tri: h.tri as u32, p, n };
                    self.say("Face selected. Click Sketch to draw on it, or Hole to drill here. (Right-click for more.)");
                }
                None => {
                    if matches!(self.sel, Sel::Face { .. }) {
                        self.sel = Sel::None;
                    }
                }
            },
            Tool::Move | Tool::None => {
                if let Doc::Product(_) = self.doc {
                    if self.active_comp.is_none() {
                        match self.pick(inp.mouse) {
                            Some(h) => {
                                if let Some(id) = h.comp {
                                    self.sel = Sel::Comp(id);
                                    if self.tool == Tool::Move {
                                        self.start_gizmo(Handle::Free, inp.mouse);
                                    } else if !matches!(self.dialog, Dialog::Feature { .. }) {
                                        self.dialog = Dialog::Comp(id);
                                    }
                                }
                            }
                            None => {
                                if self.tool == Tool::None {
                                    self.sel = Sel::None;
                                    if matches!(self.dialog, Dialog::Comp(_) | Dialog::Constraint(_) | Dialog::Appearance(Some(_))) {
                                        self.dialog = Dialog::None;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn add_constraint(&mut self, kind: CKind, a: u32, ra: GeomRef, b: u32, rb: GeomRef) {
        self.checkpoint();
        let pr = self.product_mut().unwrap();
        let name = pr.next_name(kind.name());
        // offset and angle start from the current value
        let (ca, cb) = (pr.comp(a).unwrap(), pr.comp(b).unwrap());
        let wa = ra.transformed(&ca.matrix());
        let wb = rb.transformed(&cb.matrix());
        let value = match (kind, wa, wb) {
            (CKind::Offset, GeomRef::Plane { p, .. }, GeomRef::Plane { p: tp, n: tn }) => {
                let v = (p - tp).dot(tn);
                if v.abs() < 1e-6 { 10.0 } else { (v * 100.0).round() / 100.0 }
            }
            (CKind::Angle, GeomRef::Plane { n, .. }, GeomRef::Plane { n: tn, .. }) => (n.dot(tn).clamp(-1.0, 1.0).acos().to_degrees()).round(),
            _ => 0.0,
        };
        // if nothing is fixed yet, fix the second (target) component like an anchor
        if !pr.comps.iter().any(|c| c.fixed) {
            pr.comp_mut(b).unwrap().fixed = true;
        }
        pr.constraints.push(Constraint { name: name.clone(), kind, a, ra, b, rb, value, flip: false });
        let idx = pr.constraints.len() - 1;
        let probs = pr.solve();
        self.tool = Tool::Constrain(kind, None);
        self.sel = Sel::Constraint(idx);
        self.dialog = Dialog::Constraint(idx);
        if probs.is_empty() {
            self.say(&format!("{} created and applied. Pick two more faces for another, or Esc.", name));
        } else {
            let m = probs.join("; ");
            self.err(&m);
        }
    }

    pub fn toggle_fix(&mut self, id: u32) {
        self.checkpoint();
        if let Some(c) = self.product_mut().and_then(|p| p.comp_mut(id)) {
            c.fixed = !c.fixed;
            let s = if c.fixed { format!("{} is fixed in place.", c.name) } else { format!("{} can move again.", c.name) };
            self.say(&s);
        }
    }

    // ------------------------------------------------------------ sketcher

    pub fn start_sketch_on(&mut self, frame: Frame, support: String) {
        self.checkpoint();
        let id = match self.part_mut() {
            Some(p) => p.add_feature(FeatureKind::Sketch(Sketch::new(frame, support))),
            None => return,
        };
        self.tool = Tool::None;
        self.enter_sketch(id);
    }







    // ------------------------------------------------------------ commands

    pub fn new_part_doc(&mut self) {
        self.reset_doc(Doc::Part(Part::new("Part1")));
        self.part_counter = 1;
        self.path = None;
        self.say("New part. Select a plane and click Sketch to start.");
    }

    pub fn new_product_doc(&mut self) {
        self.reset_doc(Doc::Product(Product::new("Product1")));
        self.path = None;
        self.say("New assembly. Use New Part, Existing Part or Insert STL/OBJ to add components.");
    }

    pub fn reset_doc(&mut self, d: Doc) {
        self.doc = d;
        self.undo.clear();
        self.redo.clear();
        self.sel = Sel::None;
        self.tool = Tool::None;
        self.sketch = None;
        self.dialog = Dialog::None;
        self.active_comp = None;
        self.measure_result = None;
        self.dirty = false;
        self.fit_all();
    }

    pub fn edit_component(&mut self, id: u32) {
        self.active_comp = Some(id);
        self.sel = Sel::None;
        self.tool = Tool::None;
        self.dialog = Dialog::None;
        let name = self.product().and_then(|p| p.comp(id)).map(|c| c.part.name.clone()).unwrap_or_default();
        self.say(&format!("Editing {} in Part Design. Click 'Back to Assembly' when done.", name));
    }

    pub fn back_to_assembly(&mut self) {
        if self.sketch.is_some() {
            self.exit_sketch();
        }
        if let Some(id) = self.active_comp.take() {
            self.sel = Sel::Comp(id);
        }
        self.tool = Tool::None;
        self.dialog = Dialog::None;
        if let Some(pr) = self.product_mut() {
            pr.solve();
        }
        self.say("Back in Assembly Design.");
    }

    pub fn new_component_part(&mut self) {
        self.checkpoint();
        self.part_counter += 1;
        let existing: Vec<String> = self.product().map(|p| p.comps.iter().map(|c| c.part.name.clone()).collect()).unwrap_or_default();
        let mut n = 1;
        while existing.contains(&format!("Part{}", n)) {
            n += 1;
        }
        let pos = DVec3::ZERO;
        let id = self.product_mut().unwrap().add_component(Part::new(&format!("Part{}", n)), pos);
        self.edit_component(id);
    }

    /// A free spot beside what is already in the assembly.
    pub fn free_spot(&mut self, size: DVec3) -> DVec3 {
        let b = self.scene_bbox();
        if b.valid() { DVec3::new(b.max.x + size.x * 0.6 + 10.0, 0.0, 0.0) } else { DVec3::ZERO }
    }

    pub fn insert_part_component(&mut self, mut part: Part) -> u32 {
        self.checkpoint();
        part.ensure_built();
        let m = part.mesh();
        let pos = self.free_spot(m.bbox.size());
        let pos = if m.bbox.valid() { pos - DVec3::new(m.bbox.min.x.min(0.0), 0.0, 0.0) } else { pos };
        let id = self.product_mut().unwrap().add_component(part, pos);
        self.sel = Sel::Comp(id);
        id
    }

    pub fn duplicate_component(&mut self, id: u32) {
        let c = match self.product().and_then(|p| p.comp(id)) {
            Some(c) => c.clone(),
            None => return,
        };
        self.checkpoint();
        let size = c.part.cache.mesh.as_ref().map(|m| m.bbox.size()).unwrap_or(DVec3::splat(20.0));
        let pr = self.product_mut().unwrap();
        let nid = pr.add_component(c.part.clone(), c.pos + DVec3::new(size.x + 10.0, 0.0, 0.0));
        let nc = pr.comp_mut(nid).unwrap();
        nc.rot = c.rot;
        nc.part.color = c.part.color;
        self.sel = Sel::Comp(nid);
        self.say("Component duplicated.");
    }

    pub fn delete_selection(&mut self) {
        if self.sketch.is_some() {
            self.delete_sketch_selection();
            return;
        }
        match self.sel {
            Sel::Feature(id) => {
                self.checkpoint();
                if let Some(p) = self.part_mut() {
                    let names = p.delete_feature(id);
                    self.dialog = Dialog::None;
                    self.sel = Sel::None;
                    self.say(&format!("Deleted {}.", names.join(", ")));
                }
            }
            Sel::Comp(id) if self.active_comp.is_none() => {
                self.checkpoint();
                self.product_mut().unwrap().remove_component(id);
                self.sel = Sel::None;
                self.dialog = Dialog::None;
                self.say("Component deleted.");
            }
            Sel::Constraint(i) => {
                self.checkpoint();
                if let Some(pr) = self.product_mut() {
                    if i < pr.constraints.len() {
                        pr.constraints.remove(i);
                    }
                }
                self.sel = Sel::None;
                self.dialog = Dialog::None;
                self.say("Constraint deleted.");
            }
            _ => {}
        }
    }

    pub fn open_feature_dialog(&mut self, id: u32, new: bool, backup: Option<Feature>) {
        self.sel = Sel::Feature(id);
        self.dialog = Dialog::Feature { id, new, backup };
    }

    /// Pad / Pocket / Shaft / Groove: uses the selected sketch, or the last unused one.
    pub fn sketch_based(&mut self, which: &str) {
        let part = match self.part() {
            Some(p) => p,
            None => return,
        };
        let sk = match self.sel {
            Sel::Feature(id) if matches!(part.feature(id).map(|f| &f.kind), Some(FeatureKind::Sketch(_))) => Some(id),
            _ => part.last_sketch_unused().or(part.last_sketch()),
        };
        let sk = match sk {
            Some(s) => s,
            None => {
                self.err("Draw a sketch first: select a plane, click Sketch, draw a closed shape, then Exit Sketch.");
                return;
            }
        };
        if let Some(FeatureKind::Sketch(s)) = part.feature(sk).map(|f| &f.kind) {
            if s.shapes.is_empty() {
                self.err("That sketch is empty. Double-click it in the tree to draw in it.");
                return;
            }
        }
        let has_body = part.cache.bodies.last().map_or(false, |b| !b.is_empty());
        if (which == "Pocket" || which == "Groove") && !has_body {
            self.err(&format!("{} removes material, but there is no solid yet. Use Pad or Shaft first.", which));
            return;
        }
        self.checkpoint();
        let kind = match which {
            "Pad" => FeatureKind::Pad { sketch: sk, length: 20.0, symmetric: false, reverse: false },
            "Pocket" => FeatureKind::Pocket { sketch: sk, depth: 10.0, through: false, reverse: false },
            "Shaft" => FeatureKind::Shaft { sketch: sk, angle: 360.0, axis_h: false },
            _ => FeatureKind::Groove { sketch: sk, angle: 360.0, axis_h: false },
        };
        let p = self.part_mut().unwrap();
        if let Some(f) = p.feature_mut(sk) {
            f.hidden = true;
        }
        let id = p.add_feature(kind);
        self.open_feature_dialog(id, true, None);
    }

    pub fn add_primitive(&mut self, prim: PrimKind) {
        if self.part().is_none() {
            return;
        }
        self.checkpoint();
        let (pos, a, b, c) = match prim {
            PrimKind::Box => (DVec3::new(0.0, 0.0, 10.0), 20.0, 20.0, 20.0),
            PrimKind::Cylinder => (DVec3::ZERO, 20.0, 30.0, 0.0),
            PrimKind::Sphere => (DVec3::new(0.0, 0.0, 15.0), 30.0, 0.0, 0.0),
        };
        let id = self.part_mut().unwrap().add_feature(FeatureKind::Primitive { prim, pos, a, b, c, axis: Axis3::Z, subtract: false });
        self.open_feature_dialog(id, true, None);
    }

    fn has_body(&self) -> bool {
        self.part().and_then(|p| p.body()).map_or(false, |b| !b.is_empty())
    }

    /// Dress-up features (fillet, chamfer, draft, shell, thickness): the definition opens and
    /// edges / faces are then clicked in the 3D view.
    pub fn add_dressup(&mut self, mut kind: FeatureKind, edges: bool) {
        if !self.has_body() {
            self.err("Dress-up features work on a solid: build one first (Pad, Shaft or a shape).");
            return;
        }
        // a face selected beforehand is used straight away
        if let Sel::Face { p, n, .. } = self.sel {
            if let FeatureKind::Draft { faces, .. } | FeatureKind::Shell { faces, .. } | FeatureKind::Thickness { faces, .. } = &mut kind {
                faces.push(FaceRef { p, n });
            }
        }
        self.checkpoint();
        let id = self.part_mut().unwrap().add_feature(kind);
        self.open_feature_dialog(id, true, None);
        self.say(if edges { "Click the edges to treat in the 3D view (click again to remove), then OK." } else { "Click the faces in the 3D view (click again to remove), then OK." });
    }

    pub fn add_simple(&mut self, kind: FeatureKind) {
        if !self.has_body() {
            self.err("There is no solid to transform yet.");
            return;
        }
        self.checkpoint();
        let id = self.part_mut().unwrap().add_feature(kind);
        self.open_feature_dialog(id, true, None);
    }

    pub fn add_ref_plane(&mut self) {
        let kind = match self.sel {
            Sel::Plane(p) => FeatureKind::RefPlane { base: p, face: None, offset: 20.0 },
            Sel::Face { p, n, .. } => FeatureKind::RefPlane { base: 3, face: Some(FaceRef { p, n }), offset: 10.0 },
            _ => FeatureKind::RefPlane { base: 0, face: None, offset: 20.0 },
        };
        self.checkpoint();
        let id = self.part_mut().unwrap().add_feature(kind);
        self.open_feature_dialog(id, true, None);
        self.say("Plane: choose the reference and offset. Select it and click Sketch to draw on it.");
    }

    /// Split: cut the body with the selected plane (or xy) and keep one side.
    pub fn add_split(&mut self) {
        if !self.has_body() {
            self.err("Split needs a solid to cut.");
            return;
        }
        let kind = match self.sel {
            Sel::Plane(p) => FeatureKind::Split { base: p, plane: 0, offset: 0.0, flip: false },
            Sel::Feature(id) if self.part().and_then(|p| p.ref_plane_frame(id)).is_some() => FeatureKind::Split { base: 3, plane: id, offset: 0.0, flip: false },
            _ => {
                let mid = self.part().and_then(|p| p.cache.mesh.as_ref().map(|m| m.bbox)).filter(|b| b.valid()).map(|b| b.center().z).unwrap_or(0.0);
                FeatureKind::Split { base: 0, plane: 0, offset: (mid * 10.0).round() / 10.0, flip: false }
            }
        };
        self.checkpoint();
        let id = self.part_mut().unwrap().add_feature(kind);
        self.open_feature_dialog(id, true, None);
        self.say("Split: choose the plane and offset; tick 'Keep the other side' to swap.");
    }

    /// Stiffener: the open lines of the selected (or last unused) sketch become ribs.
    pub fn add_stiffener(&mut self) {
        let part = match self.part() {
            Some(p) => p,
            None => return,
        };
        let has_lines = |id: u32| match part.feature(id).map(|f| &f.kind) {
            Some(FeatureKind::Sketch(s)) => s.shapes.iter().enumerate().any(|(i, sh)| sh.is_open() && !s.is_construction(i)),
            _ => false,
        };
        let sk = match self.sel {
            Sel::Feature(id) if has_lines(id) => Some(id),
            _ => part.unused_sketches().into_iter().rev().find(|s| has_lines(*s)),
        };
        let Some(sk) = sk else {
            self.err("Draw the stiffener as one or more lines in a sketch first (e.g. between two walls).");
            return;
        };
        self.checkpoint();
        let p = self.part_mut().unwrap();
        if let Some(f) = p.feature_mut(sk) {
            f.hidden = true;
        }
        let id = p.add_feature(FeatureKind::Stiffener { sketch: sk, thickness: 2.0, depth: 10.0, reverse: false });
        self.open_feature_dialog(id, true, None);
    }

    /// Rib / Slot: a profile sketch swept along a centre-curve sketch.
    pub fn add_sweep(&mut self, remove: bool) {
        let part = match self.part() {
            Some(p) => p,
            None => return,
        };
        if remove && !self.has_body() {
            self.err("Slot removes material, but there is no solid yet.");
            return;
        }
        let sketches = part.unused_sketches();
        let all = part.sketches();
        if all.len() < 2 {
            self.err("Rib and Slot need two sketches: a closed profile and a centre curve (open lines / arcs / spline).");
            return;
        }
        let is_path = |id: u32| match part.feature(id).map(|f| &f.kind) {
            Some(FeatureKind::Sketch(s)) => s.shapes.iter().enumerate().any(|(i, sh)| sh.is_open() && !s.is_construction(i)),
            _ => false,
        };
        let pool: Vec<u32> = if sketches.len() >= 2 { sketches } else { all.clone() };
        let path = pool.iter().rev().cloned().find(|s| is_path(*s)).unwrap_or(pool[pool.len() - 1]);
        let profile = pool.iter().rev().cloned().find(|s| *s != path).unwrap_or(all[0]);
        self.checkpoint();
        let p = self.part_mut().unwrap();
        for s in [profile, path] {
            if let Some(f) = p.feature_mut(s) {
                f.hidden = true;
            }
        }
        let id = p.add_feature(FeatureKind::Sweep { profile, path, remove });
        self.open_feature_dialog(id, true, None);
        self.say("Rib / Slot: click the Profile and Center curve buttons to choose other sketches.");
    }

    /// Multi-sections solid through the sketches that each hold one closed profile.
    pub fn add_loft(&mut self, remove: bool) {
        let part = match self.part() {
            Some(p) => p,
            None => return,
        };
        if remove && !self.has_body() {
            self.err("Removed Multi-sections removes material, but there is no solid yet.");
            return;
        }
        let one_profile = |id: u32| match part.feature(id).map(|f| &f.kind) {
            Some(FeatureKind::Sketch(s)) => matches!(s.regions(), Ok((a, b)) if a.len() == 1 && b.is_empty()),
            _ => false,
        };
        let sections: Vec<u32> = part.unused_sketches().into_iter().filter(|s| one_profile(*s)).collect();
        if sections.len() < 2 {
            self.err("Multi-sections needs at least two sketches, each with one closed profile (e.g. on offset planes).");
            return;
        }
        self.checkpoint();
        let p = self.part_mut().unwrap();
        for s in &sections {
            if let Some(f) = p.feature_mut(*s) {
                f.hidden = true;
            }
        }
        let id = p.add_feature(FeatureKind::Loft { sections, remove });
        self.open_feature_dialog(id, true, None);
    }

    pub fn add_mirror(&mut self) {
        let has_body = self.part().and_then(|p| p.body()).map_or(false, |b| !b.is_empty());
        if !has_body {
            self.err("Mirror copies the solid: build something first.");
            return;
        }
        self.checkpoint();
        let id = self.part_mut().unwrap().add_feature(FeatureKind::Mirror { plane: Axis3::X });
        self.open_feature_dialog(id, true, None);
    }

    pub fn add_pattern(&mut self, circular: bool) {
        let part = match self.part() {
            Some(p) => p,
            None => return,
        };
        let target = match self.sel {
            Sel::Feature(id) if part.feature(id).map_or(false, |f| f.kind.patternable()) => Some(id),
            _ => part.features.iter().rev().find(|f| f.kind.patternable()).map(|f| f.id),
        };
        let target = match target {
            Some(t) => t,
            None => {
                self.err("Patterns repeat a Pad, Pocket, Hole or shape: create one first (or select it in the tree).");
                return;
            }
        };
        self.checkpoint();
        let kind = if circular {
            FeatureKind::CircPattern { feature: target, axis: Axis3::Z, count: 6.0, total: 360.0 }
        } else {
            FeatureKind::RectPattern { feature: target, dir: Axis3::X, count: 3.0, step: 25.0, dir2: Axis3::Y, count2: 1.0, step2: 25.0 }
        };
        let id = self.part_mut().unwrap().add_feature(kind);
        self.open_feature_dialog(id, true, None);
    }

    pub fn start_tool(&mut self, t: Tool, msg: &str) {
        self.tool = t;
        self.measure_result = None;
        self.say(msg);
    }

    pub fn cancel(&mut self) {
        if let Some(sk) = self.sketch.as_mut() {
            if !sk.pts.is_empty() {
                sk.pts.clear();
                return;
            }
            if sk.cons_first.is_some() || sk.meas_first.is_some() || sk.sel_dim.is_some() {
                sk.cons_first = None;
                sk.meas_first = None;
                sk.sel_dim = None;
                return;
            }
            if sk.tool != SkTool::Select {
                sk.tool = SkTool::Select;
                return;
            }
            if !sk.sel.is_empty() {
                sk.sel.clear();
                return;
            }
            return;
        }
        if self.tool != Tool::None {
            self.tool = Tool::None;
            self.hover = None;
            self.say("Ready.");
            return;
        }
        self.measure_result = None;
        self.sel = Sel::None;
    }

    fn shortcuts(&mut self) {
        if self.ui.typing() {
            return;
        }
        let inp = self.ui.inp.clone();
        if inp.ctrl {
            if inp.key(KeyCode::Z) {
                self.do_undo();
            }
            if inp.key(KeyCode::Y) {
                self.do_redo();
            }
            if inp.key(KeyCode::S) {
                self.save(false);
            }
            if inp.key(KeyCode::O) {
                self.open_file_dialog(Purpose::OpenDoc);
            }
            if inp.key(KeyCode::N) {
                self.new_part_doc();
            }
            if inp.key(KeyCode::Equal) || inp.key(KeyCode::KpAdd) {
                self.change_scale(Some(true));
            }
            if inp.key(KeyCode::Minus) || inp.key(KeyCode::KpSubtract) {
                self.change_scale(Some(false));
            }
            if inp.key(KeyCode::Key0) || inp.key(KeyCode::Kp0) {
                self.change_scale(None);
            }
            return;
        }
        if matches!(self.dialog, Dialog::File(_)) {
            return;
        }
        if inp.key(KeyCode::Escape) {
            if self.menu_open.is_some() || self.ctx_menu.is_some() {
                self.menu_open = None;
                self.ctx_menu = None;
            } else if matches!(self.dialog, Dialog::Help | Dialog::Options | Dialog::Appearance(_)) {
                self.dialog = Dialog::None;
            } else {
                self.cancel();
            }
        }
        if inp.key(KeyCode::Delete) {
            self.delete_selection();
        }
        if inp.key(KeyCode::Backspace) {
            if let Some(sk) = self.sketch.as_mut() {
                if !sk.pts.is_empty() {
                    sk.pts.pop();
                } else {
                    self.delete_selection();
                }
            }
        }
        if inp.key(KeyCode::F) && self.sketch.is_none() {
            self.fit_all();
        }
        if inp.key(KeyCode::G) && self.sketch.is_some() {
            self.toolbar_command("sk_snap");
        }
        if inp.key(KeyCode::R) && self.sketch.is_none() && !inp.ctrl {
            self.toggle_realistic();
        }
        if inp.key(KeyCode::F1) {
            self.dialog = Dialog::Help;
        }
    }

    // ------------------------------------------------------------ files

    pub fn open_file_dialog(&mut self, purpose: Purpose) {
        let exts: Vec<&'static str> = match purpose {
            Purpose::OpenDoc => vec!["bcpart", "bcasm"],
            Purpose::SaveDoc => match self.doc {
                Doc::Part(_) => vec!["bcpart"],
                Doc::Product(_) => vec!["bcasm"],
            },
            Purpose::ExportStl | Purpose::ExportStlSelected => vec!["stl"],
            Purpose::ExportObj => vec!["obj"],
            Purpose::ImportMesh | Purpose::InsertMesh => vec!["stl", "obj"],
            Purpose::InsertPart => vec!["bcpart"],
            Purpose::SavePartOfComp => vec!["bcpart"],
        };
        let base = self.doc_name();
        let name = match purpose {
            Purpose::SaveDoc => format!("{}.{}", base, exts[0]),
            Purpose::ExportStl | Purpose::ExportObj => format!("{}.{}", base, exts[0]),
            Purpose::ExportStlSelected | Purpose::SavePartOfComp => {
                let n = match self.sel {
                    Sel::Comp(id) => self.product().and_then(|p| p.comp(id)).map(|c| c.part.name.clone()).unwrap_or(base),
                    _ => base,
                };
                format!("{}.{}", n, exts[0])
            }
            _ => String::new(),
        };
        self.menu_open = None;
        self.dialog = Dialog::File(FileDialog::new(purpose, self.last_dir.clone(), exts, name));
    }

    pub fn doc_name(&self) -> String {
        match &self.doc {
            Doc::Part(p) => p.name.clone(),
            Doc::Product(p) => p.name.clone(),
        }
    }

    pub fn save(&mut self, save_as: bool) {
        match (&self.path, save_as) {
            (Some(p), false) => {
                let p = p.clone();
                self.file_chosen(Purpose::SaveDoc, p);
            }
            _ => self.open_file_dialog(Purpose::SaveDoc),
        }
    }

    /// Triangles of the whole document (or one component) in world coordinates.
    pub fn export_triangles(&mut self, only: Option<u32>) -> Vec<[DVec3; 3]> {
        match &mut self.doc {
            Doc::Part(p) => {
                let m = p.mesh();
                mesh_triangles(&m, |x| x)
            }
            Doc::Product(pr) => {
                let mut out = vec![];
                for c in &mut pr.comps {
                    if c.hidden || only.map_or(false, |o| o != c.id) {
                        continue;
                    }
                    let mx = c.matrix();
                    let m = c.part.mesh();
                    out.extend(mesh_triangles(&m, |x| mx.transform_point3(x)));
                }
                out
            }
        }
    }

    pub fn file_chosen(&mut self, purpose: Purpose, path: String) {
        if let Some(parent) = PathBuf::from(&path).parent() {
            self.last_dir = parent.to_path_buf();
        }
        let file_stem = PathBuf::from(&path).file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "Part".into());
        match purpose {
            Purpose::OpenDoc => match load_doc(&path) {
                Ok(d) => {
                    self.reset_doc(d);
                    self.path = Some(path.clone());
                    self.say(&format!("Opened {}.", path));
                }
                Err(e) => self.err(&format!("Could not open: {}", e)),
            },
            Purpose::SaveDoc => {
                // keep the document's name in step with the file name
                match &mut self.doc {
                    Doc::Part(p) => p.name = file_stem.clone(),
                    Doc::Product(p) => p.name = file_stem.clone(),
                }
                match save_doc(&self.doc, &path) {
                    Ok(()) => {
                        self.path = Some(path.clone());
                        self.dirty = false;
                        self.say(&format!("Saved {}.", path));
                    }
                    Err(e) => self.err(&format!("Could not save: {}", e)),
                }
            }
            Purpose::SavePartOfComp => {
                if let Sel::Comp(id) = self.sel {
                    if let Some(c) = self.product().and_then(|p| p.comp(id)) {
                        let mut part = c.part.clone();
                        part.name = file_stem;
                        match save_doc(&Doc::Part(part), &path) {
                            Ok(()) => self.say(&format!("Saved part {}.", path)),
                            Err(e) => self.err(&format!("Could not save: {}", e)),
                        }
                    }
                }
            }
            Purpose::ExportStl | Purpose::ExportStlSelected | Purpose::ExportObj => {
                let only = if purpose == Purpose::ExportStlSelected { if let Sel::Comp(id) = self.sel { Some(id) } else { None } } else { None };
                let tris = self.export_triangles(only);
                if tris.is_empty() {
                    self.err("There is no solid to export yet.");
                    return;
                }
                let r = if purpose == Purpose::ExportObj { write_obj(&path, &tris) } else { write_stl(&path, &tris) };
                match r {
                    Ok(()) => self.say(&format!("Exported {} triangles to {} (millimetres).", tris.len(), path)),
                    Err(e) => self.err(&format!("Could not export: {}", e)),
                }
            }
            Purpose::ImportMesh | Purpose::InsertMesh => {
                let tris = match read_mesh_file(&path) {
                    Ok(t) => t,
                    Err(e) => {
                        self.err(&format!("Could not import: {}", e));
                        return;
                    }
                };
                let n = tris.len();
                let kind = FeatureKind::Import { file: path.clone(), tris: Arc::new(tris_to_f32(&tris)), scale: 1.0 };
                if purpose == Purpose::ImportMesh {
                    self.checkpoint();
                    if let Some(p) = self.part_mut() {
                        let id = p.add_feature(kind);
                        self.open_feature_dialog(id, true, None);
                    }
                } else {
                    let mut part = Part::new(&file_stem);
                    part.add_feature(kind);
                    self.insert_part_component(part);
                }
                self.say(&format!("Imported {} triangles from {}.", n, path));
                self.fit_all();
            }
            Purpose::InsertPart => match load_doc(&path) {
                Ok(Doc::Part(p)) => {
                    self.insert_part_component(p);
                    self.say(&format!("Inserted {}.", path));
                    self.fit_all();
                }
                Ok(Doc::Product(_)) => self.err("That file is an assembly; pick a part (.bcpart) file."),
                Err(e) => self.err(&format!("Could not open: {}", e)),
            },
        }
    }
}

pub fn read_mesh_file(path: &str) -> Result<Vec<[DVec3; 3]>, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let lower = path.to_lowercase();
    if lower.ends_with(".obj") {
        read_obj(&String::from_utf8_lossy(&bytes))
    } else {
        read_stl(&bytes)
    }
}


pub fn ref_point(r: &GeomRef) -> DVec3 {
    match r {
        GeomRef::Plane { p, .. } => *p,
        GeomRef::Axis { p, .. } => *p,
    }
}

pub fn dist_seg_2d(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let l = ab.length_squared();
    if l < 1e-9 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / l).clamp(0.0, 1.0);
    (a + ab * t - p).length()
}

fn draw_anchor(s: Vec2) {
    let c = Color::new(0.15, 0.25, 0.6, 1.0);
    draw_circle_lines(s.x, s.y - 8.0, 3.0, 1.5, c);
    draw_line(s.x, s.y - 5.0, s.x, s.y + 6.0, 2.0, c);
    draw_line(s.x - 6.0, s.y + 2.0, s.x, s.y + 7.0, 2.0, c);
    draw_line(s.x + 6.0, s.y + 2.0, s.x, s.y + 7.0, 2.0, c);
}
