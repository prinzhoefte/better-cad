//! Interface panels: menu bar, workbench toolbars, specification tree, view toolbar,
//! status bar, definition dialogs and context menus.

use crate::app::*;
use crate::files::*;
use crate::model::*;
use crate::measure::item_name;
use crate::dims::DimKind;
use glam::DVec3;
use crate::ui::*;
use crate::view::quat_from_back_up;
use glam::{DQuat, EulerRot};
use macroquad::prelude::*;

pub const MENU_H: f32 = 26.0;
pub const STATUS_H: f32 = 24.0;
pub const TOOLBAR_W: f32 = 152.0;
pub const DIALOG_W: f32 = 320.0;

struct TreeRow {
    depth: usize,
    label: String,
    icon: &'static str,
    sel: Sel,
    error: bool,
    dim: bool,
    bold: bool,
    last: bool,
}

impl App {
    pub fn draw_ui(&mut self) {
        self.ui.capture_mouse = self.gizmo_drag.is_some() || self.orbit_drag || self.sketch.as_ref().map_or(false, |s| s.drag.is_some());
        self.draw_tree();
        self.draw_toolbar();
        self.draw_viewbar();
        self.draw_status();
        self.draw_dialog();
        self.draw_menubar();
        self.draw_ctx_menu();
        if let Dialog::Help = self.dialog {
            self.draw_help();
        }
        if let Dialog::Options = self.dialog {
            self.draw_options();
        }
        if let Dialog::File(_) = self.dialog {
            let mut fd = match std::mem::replace(&mut self.dialog, Dialog::None) {
                Dialog::File(fd) => fd,
                _ => unreachable!(),
            };
            match fd.show(&mut self.ui) {
                FileResult::None => self.dialog = Dialog::File(fd),
                FileResult::Cancel => {}
                FileResult::Chosen(p) => {
                    self.last_dir = fd.dir.clone();
                    self.file_chosen(fd.purpose, p);
                }
            }
        }
    }

    // ------------------------------------------------------------ menu bar

    fn draw_menubar(&mut self) {
        let w = sw();
        let bar = Rect::new(0.0, 0.0, w, MENU_H);
        self.ui.block(bar);
        draw_rectangle(0.0, 0.0, w, MENU_H, C_PANEL_DARK);
        draw_line(0.0, MENU_H, w, MENU_H, 1.0, C_BORDER);
        let menus = ["Start", "File", "Edit", "View", "Insert", "Tools", "Help"];
        let mut x = 6.0;
        let mut xs = vec![];
        for (i, m) in menus.iter().enumerate() {
            let r = Rect::new(x, 2.0, text_w(m, 17) + 18.0, MENU_H - 4.0);
            xs.push(x);
            self.ui.reg(&format!("menu_{}", m.to_lowercase()), r);
            let open = self.menu_open == Some(i);
            if open || self.ui.hover(r) {
                draw_rectangle(r.x, r.y, r.w, r.h, if open { C_ACTIVE } else { C_HOT });
            }
            text(m, r.x + 9.0, r.y + 16.0, 17, C_TEXT);
            if self.ui.clicked(r) {
                self.menu_open = if open { None } else { Some(i) };
                self.ctx_menu = None;
            } else if self.menu_open.is_some() && self.ui.hover(r) {
                self.menu_open = Some(i);
            }
            x += r.w + 2.0;
        }
        let title = format!("BetterCAD - {}{}", self.doc_name(), if self.dirty { " *" } else { "" });
        text(&title, x + 24.0, 18.0, 17, C_DIM);
        let wb = self.workbench();
        let tw = text_w(wb, 17);
        draw_rectangle(w - tw - 22.0, 3.0, tw + 16.0, MENU_H - 6.0, C_TITLE);
        text(wb, w - tw - 14.0, 18.0, 17, WHITE);

        if let Some(mi) = self.menu_open {
            let in_asm = self.product().is_some();
            let mut items: Vec<(String, String, bool)> = vec![];
            let mut it = |a: &str, b: &str, c: bool| items.push((a.to_string(), b.to_string(), c));
            match mi {
                0 => {
                    it("Part Design (new part)", "", true);
                    it("Assembly Design (new assembly)", "", true);
                }
                1 => {
                    it("New Part", "Ctrl+N", true);
                    it("New Assembly", "", true);
                    it("Open...", "Ctrl+O", true);
                    it("Save", "Ctrl+S", true);
                    it("Save As...", "", true);
                    it("-", "", true);
                    it("Export STL...", "3D printing", true);
                    it("Export Selected as STL...", "", in_asm && matches!(self.sel, Sel::Comp(_)));
                    it("Export OBJ...", "games", true);
                    it("Import STL/OBJ...", "", true);
                    it("Save Selected Part As...", "", in_asm && matches!(self.sel, Sel::Comp(_)));
                    it("-", "", true);
                    it("Quit", "", true);
                }
                2 => {
                    let sk_sel = self.sketch.as_ref().map_or(false, |s| !s.sel.is_empty());
                    it("Undo", "Ctrl+Z", !self.undo.is_empty());
                    it("Redo", "Ctrl+Y", !self.redo.is_empty());
                    it("Delete", "Del", (self.sel != Sel::None && !matches!(self.sel, Sel::Face { .. })) || sk_sel);
                    it("Properties", "", matches!(self.sel, Sel::Feature(_) | Sel::Comp(_) | Sel::Constraint(_)));
                }
                3 => {
                    it("Fit All", "F", true);
                    it("Isometric", "", true);
                    it(if self.view.persp { "Parallel Projection" } else { "Perspective Projection" }, "", true);
                    it(if self.show_edges { "Hide Edges" } else { "Show Edges" }, "", true);
                    it("-", "", true);
                    it("Bigger Interface", "Ctrl +", ui_scale() < 3.99);
                    it("Smaller Interface", "Ctrl -", ui_scale() > 0.76);
                    it("Interface Size 100 %", "Ctrl 0", true);
                }
                4 => {
                    for (g, tools) in self.toolbar_groups() {
                        it(&format!("- {}", g), "", false);
                        for (id, name, _) in tools {
                            it(&format!("   {}", name), id, true);
                        }
                    }
                }
                5 => {
                    it("Options...", "", true);
                    it("Measure", "", self.sketch.is_none());
                    it(if self.show_edges { "Hide Edges" } else { "Show Edges" }, "", true);
                }
                _ => it("Controls and Tips", "F1", true),
            }
            let mx = xs[mi];
            let mw = 300.0;
            let mh = items.iter().map(|i| if i.0 == "-" { 8.0 } else { 22.0 }).sum::<f32>() + 6.0;
            let my = MENU_H;
            let mh = mh.min(sh() - my - 4.0);
            let mr = Rect::new(mx, my, mw, mh);
            self.ui.panel(mr, None);
            let mut y = my + 3.0;
            let mut chosen: Option<(String, String)> = None;
            for (label, short, enabled) in &items {
                if label == "-" {
                    draw_line(mx + 6.0, y + 4.0, mx + mw - 6.0, y + 4.0, 1.0, C_BORDER);
                    y += 8.0;
                    continue;
                }
                if y + 22.0 > my + mh {
                    break;
                }
                let r = Rect::new(mx + 2.0, y, mw - 4.0, 22.0);
                self.ui.reg(&format!("mi_{}", label.trim()), r);
                if *enabled && self.ui.hover(r) {
                    draw_rectangle(r.x, r.y, r.w, r.h, C_HOT);
                }
                let heading = label.starts_with("- ");
                let shown = if heading { &label[2..] } else { label.as_str() };
                text(shown, r.x + 10.0, r.y + 16.0, 16, if heading { C_ACCENT } else if *enabled { C_TEXT } else { C_DIM });
                if mi != 4 {
                    text(short, r.x + r.w - text_w(short, 14) - 10.0, r.y + 16.0, 14, C_DIM);
                }
                if *enabled && self.ui.clicked(r) {
                    chosen = Some((label.trim().to_string(), short.clone()));
                }
                y += 22.0;
            }
            if let Some((c, extra)) = chosen {
                self.menu_open = None;
                if mi == 4 {
                    self.toolbar_command(&extra);
                } else {
                    self.menu_command(&c);
                }
            } else if self.ui.inp.pressed[LMB] && !mr.contains(self.ui.inp.mouse) && !bar.contains(self.ui.inp.mouse) {
                self.menu_open = None;
            }
        }
    }

    fn menu_command(&mut self, c: &str) {
        match c {
            "New Part" | "Part Design (new part)" => self.new_part_doc(),
            "New Assembly" | "Assembly Design (new assembly)" => self.new_product_doc(),
            "Open..." => self.open_file_dialog(Purpose::OpenDoc),
            "Save" => self.save(false),
            "Save As..." => self.save(true),
            "Export STL..." => self.open_file_dialog(Purpose::ExportStl),
            "Export Selected as STL..." => self.open_file_dialog(Purpose::ExportStlSelected),
            "Export OBJ..." => self.open_file_dialog(Purpose::ExportObj),
            "Import STL/OBJ..." => {
                if self.part().is_some() {
                    self.open_file_dialog(Purpose::ImportMesh)
                } else {
                    self.open_file_dialog(Purpose::InsertMesh)
                }
            }
            "Save Selected Part As..." => self.open_file_dialog(Purpose::SavePartOfComp),
            "Quit" => self.quit = true,
            "Undo" => self.do_undo(),
            "Redo" => self.do_redo(),
            "Delete" => self.delete_selection(),
            "Properties" => self.open_properties(self.sel),
            "Fit All" => self.fit_all(),
            "Isometric" => self.set_view(vec3(1.0, 1.0, 0.8), Vec3::Z),
            "Parallel Projection" | "Perspective Projection" => self.view.persp = !self.view.persp,
            "Hide Edges" | "Show Edges" => self.show_edges = !self.show_edges,
            "Measure" => self.start_tool(Tool::Measure(None), "Measure: click the first point on a surface."),
            "Controls and Tips" => self.dialog = Dialog::Help,
            "Options..." => self.dialog = Dialog::Options,
            "Bigger Interface" => self.change_scale(Some(true)),
            "Smaller Interface" => self.change_scale(Some(false)),
            "Interface Size 100 %" => self.change_scale(None),
            _ => {}
        }
    }

    pub fn open_properties(&mut self, s: Sel) {
        match s {
            Sel::Feature(id) => {
                let f = self.part().and_then(|p| p.feature(id)).cloned();
                if let Some(f) = f {
                    if matches!(f.kind, FeatureKind::Sketch(_)) {
                        if self.sketch.is_none() {
                            self.enter_sketch(id);
                        }
                    } else {
                        self.checkpoint();
                        self.open_feature_dialog(id, false, Some(f));
                    }
                }
            }
            Sel::Comp(id) => self.dialog = Dialog::Comp(id),
            Sel::Constraint(i) => self.dialog = Dialog::Constraint(i),
            _ => {}
        }
    }

    // ------------------------------------------------------------ toolbars

    /// The workbench's toolbars: (toolbar name, [(id, name, description)]).
    pub fn toolbar_groups(&self) -> Vec<(&'static str, Vec<(&'static str, &'static str, &'static str)>)> {
        if self.sketch.is_some() {
            return vec![
                ("Select", vec![("sk_select", "Select", "Select, drag and edit elements")]),
                (
                    "Profile",
                    vec![
                        ("sk_profile", "Profile", "Connected lines: click point after point"),
                        ("sk_line", "Line", "A single line between two points"),
                        ("sk_axis", "Axis", "Rotation axis for Shaft and Groove"),
                        ("sk_rect", "Rectangle", "Rectangle from two opposite corners"),
                        ("sk_crect", "Centered Rectangle", "Rectangle from its centre and a corner"),
                        ("sk_polygon", "Hexagon", "Regular polygon (6 sides, editable)"),
                        ("sk_slot", "Elongated Hole", "Slot: two centres and a width"),
                    ],
                ),
                (
                    "Curves",
                    vec![
                        ("sk_circle", "Circle", "Circle from its centre and a point"),
                        ("sk_circle3", "Three Point Circle", "Circle through three points"),
                        ("sk_arc", "Arc", "Arc from centre, start and end"),
                        ("sk_arc3", "Three Point Arc", "Arc from start, a point on it, and end"),
                        ("sk_ellipse", "Ellipse", "Ellipse from centre and both axes"),
                        ("sk_spline", "Spline", "Smooth curve through points"),
                    ],
                ),
                (
                    "Operation",
                    vec![
                        ("sk_corner", "Corner", "Round the corner where two lines meet"),
                        ("sk_chamfer", "Chamfer", "Cut the corner where two lines meet"),
                        ("sk_trim", "Quick Trim", "Cut away a piece up to where it crosses others"),
                        ("sk_mirror", "Mirror", "Mirror the selected elements across a line"),
                    ],
                ),
                ("Constraint", vec![("sk_constraint", "Constraint", "Dimension an element (length, radius, width...) or two elements\n(distance, angle). Drag dimensions to move them,\ndouble-click to change the value: the shape follows.")]),
                (
                    "Measure",
                    vec![
                        ("sk_measure", "Measure Between", "Distance and angle between two points, lines, circles or axes"),
                        ("sk_measitem", "Measure Item", "Length, radius, angle, area and position of one element or point"),
                    ],
                ),
                (
                    "Sketch Tools",
                    vec![
                        ("sk_constr", "Construction / Standard Element", "Make elements helpers (dashed, no material) or normal"),
                        ("sk_snap", "Snap to Point", "Snap the cursor to the grid on/off"),
                    ],
                ),
                ("Workbench", vec![("sk_exit", "Exit Workbench", "Leave the sketcher and go back to 3D")]),
            ];
        }
        if self.part().is_some() {
            let mut v = vec![
                ("Sketcher", vec![("tb_sketch", "Sketch", "Draw on a plane or a flat face (select it first, or click it after)")]),
                (
                    "Sketch-Based Features",
                    vec![
                        ("tb_pad", "Pad", "Extrude a sketch into a solid"),
                        ("tb_pocket", "Pocket", "Cut a sketch's shape out of the solid"),
                        ("tb_shaft", "Shaft", "Revolve a sketch around its axis (adds material)"),
                        ("tb_groove", "Groove", "Revolve a sketch around its axis (removes material)"),
                        ("tb_hole", "Hole", "Simple, counterbored or countersunk hole in a flat face"),
                        ("tb_rib", "Rib", "Sweep a profile sketch along a centre-curve sketch"),
                        ("tb_slot", "Slot", "Sweep a profile along a centre curve, removing material"),
                        ("tb_loft", "Multi-sections Solid", "Blend a solid through several section sketches"),
                        ("tb_rloft", "Removed Multi-sections Solid", "Blend through several sections, removing material"),
                        ("tb_stiffener", "Stiffener", "Thicken the lines of a sketch into reinforcing ribs"),
                    ],
                ),
                ("Surface-Based Features", vec![("tb_split", "Split", "Cut the part with a plane and keep one side")]),
                (
                    "Dress-Up Features",
                    vec![
                        ("tb_fillet", "Edge Fillet", "Round edges: click the edges, set the radius"),
                        ("tb_chamfer", "Chamfer", "Bevel edges: click the edges, set the length"),
                        ("tb_draft", "Draft Angle", "Tilt side faces for mould release"),
                        ("tb_shell", "Shell", "Hollow the part, opening the faces you click"),
                        ("tb_thick", "Thickness", "Add (or remove) thickness on the faces you click"),
                    ],
                ),
                (
                    "Transformation Features",
                    vec![
                        ("tb_translate", "Translation", "Move the whole body"),
                        ("tb_rotate", "Rotation", "Turn the whole body about an axis"),
                        ("tb_symmetry", "Symmetry", "Mirror the body to the other side (without keeping the original)"),
                        ("tb_mirror", "Mirror", "Mirror the whole solid across a plane, keeping the original"),
                        ("tb_rpat", "Rectangular Pattern", "Repeat a feature in rows and columns"),
                        ("tb_cpat", "Circular Pattern", "Repeat a feature around an axis"),
                        ("tb_scale", "Scaling", "Scale the whole body about the origin"),
                    ],
                ),
                ("Reference Elements", vec![("tb_plane", "Plane", "A plane offset from a plane or a selected face, to sketch on")]),
                (
                    "Shapes",
                    vec![
                        ("tb_box", "Box", "Add a box (or cut one out)"),
                        ("tb_cyl", "Cylinder", "Add a cylinder (or cut one out)"),
                        ("tb_sphere", "Sphere", "Add a sphere (or cut one out)"),
                    ],
                ),
                (
                    "Other",
                    vec![
                        ("tb_import", "Import Mesh", "Bring an STL or OBJ file into this part"),
                        ("tb_measure", "Measure", "Distance between two points"),
                    ],
                ),
            ];
            if self.product().is_some() {
                v.push(("Workbench", vec![("tb_back", "Back to Assembly", "Finish editing this part and return to the assembly")]));
            }
            return v;
        }
        vec![
            (
                "Product Structure",
                vec![
                    ("as_newpart", "New Part", "Create a new part in the assembly and edit it"),
                    ("as_insert", "Existing Component", "Insert a saved part file (.bcpart)"),
                    ("as_mesh", "Insert Mesh", "Insert an STL or OBJ file as a component"),
                    ("as_edit", "Edit Part", "Edit the selected component's part (double-click works too)"),
                    ("as_dup", "Duplicate", "Copy the selected component"),
                ],
            ),
            ("Move", vec![("as_move", "Manipulation", "Drag components: arrows move along X/Y/Z, rings rotate,\nthe centre dot moves freely. Ctrl snaps (1 mm / 15 deg)")]),
            (
                "Constraints",
                vec![
                    ("as_contact", "Contact Constraint", "Put two flat faces against each other"),
                    ("as_offset", "Offset Constraint", "Keep two flat faces facing each other at a distance"),
                    ("as_coinc", "Coincidence Constraint", "Line up two round faces (pin in hole) on one axis"),
                    ("as_angle", "Angle Constraint", "Set the angle between two flat faces"),
                    ("as_fix", "Fix Component", "Anchor a component so constraints never move it"),
                    ("as_update", "Update All", "Re-apply all constraints"),
                ],
            ),
            ("Measure", vec![("as_measure", "Measure", "Distance between two points")]),
        ]
    }

    fn draw_toolbar(&mut self) {
        let w = sw();
        let x0 = w - TOOLBAR_W;
        let y0 = MENU_H;
        let h = sh() - MENU_H - STATUS_H;
        let r = Rect::new(x0, y0, TOOLBAR_W, h);
        self.ui.block(r);
        draw_rectangle(r.x, r.y, r.w, r.h, C_PANEL_DARK);
        draw_line(r.x, r.y, r.x, r.y + r.h, 1.0, C_BORDER);
        let bs = 44.0;
        let cols = 3usize;
        // scroll with the wheel when the toolbars are taller than the window
        let groups = self.toolbar_groups();
        let content: f32 = groups.iter().map(|(_, t)| 8.0 + ((t.len() + cols - 1) / cols) as f32 * (bs + 4.0) + 6.0).sum::<f32>() + 6.0;
        let max_scroll = (content - h).max(0.0);
        if self.ui.hover(r) && self.ui.inp.wheel != 0.0 {
            self.toolbar_scroll = (self.toolbar_scroll - self.ui.inp.wheel * 60.0).clamp(0.0, max_scroll);
        }
        self.toolbar_scroll = self.toolbar_scroll.min(max_scroll);
        let mut y = y0 + 6.0 - self.toolbar_scroll;
        let mut clicked = None;
        for (gname, tools) in groups {
            if y + 8.0 > y0 {
                draw_rectangle(x0 + 6.0, y, TOOLBAR_W - 12.0, 4.0, Color::new(0.30, 0.32, 0.37, 1.0));
                let gr = Rect::new(x0 + 6.0, y - 2.0, TOOLBAR_W - 12.0, 8.0);
                self.ui.tip(gr, gname);
            }
            y += 8.0;
            for (i, (id, name, tip)) in tools.iter().enumerate() {
                let col = (i % cols) as f32;
                let row = (i / cols) as f32;
                let br = Rect::new(x0 + 6.0 + col * (bs + 4.0), y + row * (bs + 4.0), bs, bs);
                if br.y < y0 || br.y + br.h > y0 + h {
                    continue;
                }
                self.ui.reg(id, br);
                let active = self.tool_active(id);
                let hot = self.ui.hover(br);
                draw_rectangle(br.x, br.y, br.w, br.h, if active { C_ACTIVE } else if hot { C_HOT } else { C_BUTTON });
                draw_rectangle_lines(br.x, br.y, br.w, br.h, 1.0, if active { C_SEL } else { C_BORDER });
                draw_icon(id, br.x + br.w / 2.0, br.y + br.h / 2.0);
                self.ui.tip(br, &format!("{}\n{}", name, tip));
                if self.ui.clicked(br) {
                    clicked = Some(*id);
                }
            }
            y += ((tools.len() + cols - 1) / cols) as f32 * (bs + 4.0) + 6.0;
        }
        if max_scroll > 0.0 {
            // a thin scroll indicator
            let frac = h / content;
            let bar_h = (h * frac).max(30.0);
            let by = y0 + (h - bar_h) * (self.toolbar_scroll / max_scroll);
            draw_rectangle(x0 + TOOLBAR_W - 4.0, by, 3.0, bar_h, C_BORDER);
        }
        if let Some(id) = clicked {
            self.toolbar_command(id);
        }
    }

    fn tool_active(&self, id: &str) -> bool {
        if let Some(sk) = &self.sketch {
            let t = match id {
                "sk_select" => SkTool::Select,
                "sk_profile" => SkTool::Profile,
                "sk_line" => SkTool::Line,
                "sk_axis" => SkTool::Axis,
                "sk_rect" => SkTool::Rect,
                "sk_crect" => SkTool::CRect,
                "sk_circle" => SkTool::Circle,
                "sk_circle3" => SkTool::Circle3,
                "sk_arc" => SkTool::Arc,
                "sk_arc3" => SkTool::Arc3,
                "sk_ellipse" => SkTool::Ellipse,
                "sk_spline" => SkTool::Spline,
                "sk_polygon" => SkTool::Polygon,
                "sk_slot" => SkTool::Slot,
                "sk_corner" => SkTool::Corner,
                "sk_chamfer" => SkTool::Chamfer,
                "sk_trim" => SkTool::Trim,
                "sk_mirror" => SkTool::Mirror,
                "sk_measure" => SkTool::MeasureBetween,
                "sk_measitem" => SkTool::MeasureItem,
                "sk_constraint" => SkTool::Constraint,
                "sk_snap" => return sk.snap,
                "sk_constr" => return sk.construction_mode,
                _ => return false,
            };
            return sk.tool == t;
        }
        match (id, self.tool) {
            ("tb_sketch", Tool::PickSketchSupport) => true,
            ("tb_hole", Tool::PickHolePoint) => true,
            ("tb_measure" | "as_measure", Tool::Measure(_)) => true,
            ("as_move", Tool::Move) => true,
            ("as_fix", Tool::Fix) => true,
            ("as_contact", Tool::Constrain(CKind::Contact, _)) => true,
            ("as_offset", Tool::Constrain(CKind::Offset, _)) => true,
            ("as_coinc", Tool::Constrain(CKind::Coincidence, _)) => true,
            ("as_angle", Tool::Constrain(CKind::Angle, _)) => true,
            _ => false,
        }
    }

    pub fn toolbar_command(&mut self, id: &str) {
        // finish an open definition dialog first
        if let Dialog::Feature { .. } = self.dialog {
            if !id.starts_with("sk_") {
                self.dialog = Dialog::None;
            }
        }
        if self.sketch.is_some() {
            let t = match id {
                "sk_select" => Some(SkTool::Select),
                "sk_profile" => Some(SkTool::Profile),
                "sk_line" => Some(SkTool::Line),
                "sk_axis" => Some(SkTool::Axis),
                "sk_rect" => Some(SkTool::Rect),
                "sk_crect" => Some(SkTool::CRect),
                "sk_circle" => Some(SkTool::Circle),
                "sk_circle3" => Some(SkTool::Circle3),
                "sk_arc" => Some(SkTool::Arc),
                "sk_arc3" => Some(SkTool::Arc3),
                "sk_ellipse" => Some(SkTool::Ellipse),
                "sk_spline" => Some(SkTool::Spline),
                "sk_polygon" => Some(SkTool::Polygon),
                "sk_slot" => Some(SkTool::Slot),
                "sk_corner" => Some(SkTool::Corner),
                "sk_chamfer" => Some(SkTool::Chamfer),
                "sk_trim" => Some(SkTool::Trim),
                "sk_mirror" => Some(SkTool::Mirror),
                "sk_measure" => Some(SkTool::MeasureBetween),
                "sk_measitem" => Some(SkTool::MeasureItem),
                "sk_constraint" => Some(SkTool::Constraint),
                _ => None,
            };
            if let Some(t) = t {
                self.set_sketch_tool(t);
                return;
            }
            match id {
                "sk_snap" => {
                    if let Some(sk) = self.sketch.as_mut() {
                        sk.snap = !sk.snap;
                    }
                }
                "sk_constr" => self.toggle_construction(),
                "sk_exit" => self.exit_sketch(),
                _ => {}
            }
            return;
        }
        match id {
            "tb_sketch" => match self.sel {
                Sel::Plane(p) => self.start_sketch_on(App::plane_frame(p), ["xy plane", "yz plane", "zx plane"][p as usize].into()),
                Sel::Face { p, n, .. } => self.sketch_on_face(p, n),
                Sel::Feature(fid) if self.part().and_then(|pt| pt.ref_plane_frame(fid)).is_some() => self.sketch_on_ref_plane(fid),
                _ => self.start_tool(Tool::PickSketchSupport, "Sketch: click a flat face of the part, or a plane (on its border, or in the tree)."),
            },
            "tb_pad" => self.sketch_based("Pad"),
            "tb_pocket" => self.sketch_based("Pocket"),
            "tb_shaft" => self.sketch_based("Shaft"),
            "tb_groove" => self.sketch_based("Groove"),
            "tb_hole" => {
                let has = self.part().and_then(|p| p.body()).map_or(false, |b| !b.is_empty());
                if let Sel::Face { p, n, .. } = self.sel {
                    self.hole_on_face(p, n);
                } else if has {
                    self.start_tool(Tool::PickHolePoint, "Hole: click a point on a flat face.");
                } else {
                    self.err("A hole needs a solid to drill into. Build something first.");
                }
            }
            "tb_rib" | "tb_slot" => self.add_sweep(id == "tb_slot"),
            "tb_loft" | "tb_rloft" => self.add_loft(id == "tb_rloft"),
            "tb_fillet" => self.add_dressup(FeatureKind::Fillet { edges: vec![], radius: 2.0 }, true),
            "tb_chamfer" => self.add_dressup(FeatureKind::Chamfer { edges: vec![], length: 1.0 }, true),
            "tb_draft" => self.add_dressup(FeatureKind::Draft { faces: vec![], angle: 5.0, neutral: Axis3::Z, reverse: false }, false),
            "tb_shell" => self.add_dressup(FeatureKind::Shell { faces: vec![], thickness: 1.0 }, false),
            "tb_thick" => self.add_dressup(FeatureKind::Thickness { faces: vec![], thickness: 1.0 }, false),
            "tb_translate" => self.add_simple(FeatureKind::Translate { v: DVec3::new(10.0, 0.0, 0.0) }),
            "tb_rotate" => self.add_simple(FeatureKind::Rotate { axis: Axis3::Z, angle: 90.0 }),
            "tb_symmetry" => self.add_simple(FeatureKind::Symmetry { plane: Axis3::X }),
            "tb_scale" => self.add_simple(FeatureKind::Scale { factor: 1.5 }),
            "tb_plane" => self.add_ref_plane(),
            "tb_split" => self.add_split(),
            "tb_stiffener" => self.add_stiffener(),
            "tb_box" => self.add_primitive(PrimKind::Box),
            "tb_cyl" => self.add_primitive(PrimKind::Cylinder),
            "tb_sphere" => self.add_primitive(PrimKind::Sphere),
            "tb_mirror" => self.add_mirror(),
            "tb_rpat" => self.add_pattern(false),
            "tb_cpat" => self.add_pattern(true),
            "tb_import" => self.open_file_dialog(Purpose::ImportMesh),
            "tb_measure" | "as_measure" => self.start_tool(Tool::Measure(None), "Measure: click the first point on a surface."),
            "tb_back" => self.back_to_assembly(),
            "as_newpart" => self.new_component_part(),
            "as_insert" => self.open_file_dialog(Purpose::InsertPart),
            "as_mesh" => self.open_file_dialog(Purpose::InsertMesh),
            "as_edit" => {
                if let Sel::Comp(id) = self.sel {
                    self.edit_component(id);
                } else {
                    self.err("Select a component first (click it, or pick it in the tree).");
                }
            }
            "as_dup" => {
                if let Sel::Comp(id) = self.sel {
                    self.duplicate_component(id);
                } else {
                    self.err("Select a component to duplicate.");
                }
            }
            "as_move" => {
                if self.tool == Tool::Move {
                    self.tool = Tool::None;
                    self.say("Ready.");
                } else {
                    self.start_tool(Tool::Move, "Manipulation: click a component, then drag the arrows (move) or rings (rotate). Ctrl snaps.");
                }
            }
            "as_contact" => self.start_tool(Tool::Constrain(CKind::Contact, None), "Contact: click a flat face of the component to move, then a face of another one."),
            "as_offset" => self.start_tool(Tool::Constrain(CKind::Offset, None), "Offset: click a flat face of the component to move, then a face of another one."),
            "as_coinc" => self.start_tool(Tool::Constrain(CKind::Coincidence, None), "Coincidence: click a round face (pin), then a round face (hole) of another component."),
            "as_angle" => self.start_tool(Tool::Constrain(CKind::Angle, None), "Angle: click a flat face of the component to turn, then a face of another one."),
            "as_fix" => {
                if let Sel::Comp(id) = self.sel {
                    self.toggle_fix(id);
                } else {
                    self.start_tool(Tool::Fix, "Fix: click the component to anchor.");
                }
            }
            "as_update" => {
                if let Some(pr) = self.product_mut() {
                    let p = pr.solve();
                    if p.is_empty() {
                        self.say("All constraints are satisfied.");
                    } else {
                        let m = p.join("; ");
                        self.err(&m);
                    }
                }
            }
            _ => {}
        }
    }

    // ------------------------------------------------------------ specification tree

    fn tree_rows_list(&self) -> Vec<TreeRow> {
        let mut rows = Vec::new();
        let part_rows = |p: &Part, depth: usize, rows: &mut Vec<TreeRow>, editing: Option<u32>| {
            rows.push(TreeRow { depth, label: "xy plane".into(), icon: "plane", sel: Sel::Plane(0), error: false, dim: false, bold: false, last: false });
            rows.push(TreeRow { depth, label: "yz plane".into(), icon: "plane", sel: Sel::Plane(1), error: false, dim: false, bold: false, last: false });
            rows.push(TreeRow { depth, label: "zx plane".into(), icon: "plane", sel: Sel::Plane(2), error: false, dim: false, bold: false, last: false });
            rows.push(TreeRow { depth, label: "PartBody".into(), icon: "body", sel: Sel::None, error: false, dim: false, bold: true, last: true });
            let n = p.features.len();
            for (i, f) in p.features.iter().enumerate() {
                let icon = match &f.kind {
                    FeatureKind::Sketch(_) => "sketch",
                    FeatureKind::Pad { .. } => "pad",
                    FeatureKind::Pocket { .. } => "pocket",
                    FeatureKind::Shaft { .. } => "shaft",
                    FeatureKind::Groove { .. } => "groove",
                    FeatureKind::Hole { .. } => "hole",
                    FeatureKind::Primitive { prim: PrimKind::Box, .. } => "box",
                    FeatureKind::Primitive { prim: PrimKind::Cylinder, .. } => "cyl",
                    FeatureKind::Primitive { prim: PrimKind::Sphere, .. } => "sphere",
                    FeatureKind::Mirror { .. } => "mirror",
                    FeatureKind::RectPattern { .. } => "rpat",
                    FeatureKind::CircPattern { .. } => "cpat",
                    FeatureKind::Import { .. } => "import",
                    FeatureKind::Fillet { .. } => "fillet",
                    FeatureKind::Chamfer { .. } => "chamfer",
                    FeatureKind::Draft { .. } => "draft",
                    FeatureKind::Shell { .. } => "shell",
                    FeatureKind::Thickness { .. } => "thick",
                    FeatureKind::Translate { .. } | FeatureKind::Rotate { .. } | FeatureKind::Symmetry { .. } | FeatureKind::Scale { .. } => "mirror",
                    FeatureKind::RefPlane { .. } => "plane",
                    FeatureKind::Loft { remove, .. } | FeatureKind::Sweep { remove, .. } => if *remove { "pocket" } else { "pad" },
                    FeatureKind::Split { .. } => "pocket",
                    FeatureKind::Stiffener { .. } => "pad",
                };
                let mut label = f.name.clone();
                if f.suppressed {
                    label.push_str("  (off)");
                }
                rows.push(TreeRow {
                    depth: depth + 1,
                    label,
                    icon,
                    sel: Sel::Feature(f.id),
                    error: p.cache.errors.contains_key(&f.id),
                    dim: f.suppressed || (f.hidden && matches!(f.kind, FeatureKind::Sketch(_))),
                    bold: editing == Some(f.id),
                    last: i + 1 == n,
                });
            }
        };
        let editing = self.sketch.as_ref().map(|s| s.feat);
        match &self.doc {
            Doc::Part(p) => {
                rows.push(TreeRow { depth: 0, label: p.name.clone(), icon: "part", sel: Sel::Root, error: false, dim: false, bold: true, last: true });
                part_rows(p, 1, &mut rows, editing);
            }
            Doc::Product(pr) => {
                rows.push(TreeRow { depth: 0, label: pr.name.clone(), icon: "product", sel: Sel::Root, error: false, dim: false, bold: true, last: true });
                for c in &pr.comps {
                    let active = self.active_comp == Some(c.id);
                    let mut label = c.name.clone();
                    if c.fixed {
                        label.push_str("  [fixed]");
                    }
                    rows.push(TreeRow { depth: 1, label, icon: "part", sel: Sel::Comp(c.id), error: false, dim: c.hidden, bold: active, last: false });
                    if active {
                        part_rows(&c.part, 2, &mut rows, editing);
                    }
                }
                rows.push(TreeRow { depth: 1, label: "Constraints".into(), icon: "constraints", sel: Sel::None, error: false, dim: false, bold: false, last: true });
                for (i, c) in pr.constraints.iter().enumerate() {
                    let (an, bn) = (pr.comp(c.a).map(|x| x.part.name.clone()).unwrap_or_default(), pr.comp(c.b).map(|x| x.part.name.clone()).unwrap_or_default());
                    let err = match (pr.comp(c.a), pr.comp(c.b)) {
                        (Some(a), Some(b)) => constraint_error(c, a, b) > 1e-3 && !(a.fixed && b.fixed),
                        _ => true,
                    };
                    let val = match c.kind {
                        CKind::Offset => format!(" = {} mm", fmt(c.value)),
                        CKind::Angle => format!(" = {} deg", fmt(c.value)),
                        _ => String::new(),
                    };
                    rows.push(TreeRow { depth: 2, label: format!("{} ({}, {}){}", c.name, an, bn, val), icon: "constraint", sel: Sel::Constraint(i), error: err, dim: false, bold: false, last: i + 1 == pr.constraints.len() });
                }
            }
        }
        rows
    }

    fn draw_tree(&mut self) {
        let rows = self.tree_rows_list();
        self.tree_rows = rows.len();
        let x0 = 12.0;
        let y0 = MENU_H + 12.0;
        let rh = 21.0;
        let max_rows = ((sh() - y0 - STATUS_H - 160.0) / rh).max(5.0) as usize;
        let mut clicked: Option<(Sel, bool)> = None;
        let mut right: Option<(Sel, Vec2)> = None;
        for (i, r) in rows.iter().enumerate().take(max_rows) {
            let y = y0 + i as f32 * rh;
            let x = x0 + r.depth as f32 * 18.0;
            // connector lines
            if r.depth > 0 {
                let px = x - 11.0;
                draw_line(px, y + rh / 2.0, x - 2.0, y + rh / 2.0, 1.0, Color::new(0.6, 0.65, 0.75, 0.5));
                draw_line(px, y - 2.0, px, y + rh / 2.0, 1.0, Color::new(0.6, 0.65, 0.75, 0.5));
            }
            let tw = text_w(&r.label, 15);
            let rr = Rect::new(x, y + 1.0, tw + 26.0, rh - 2.0);
            let selected = r.sel != Sel::None && r.sel == self.sel && r.sel != Sel::Root;
            let hot = self.ui.hover(rr);
            if selected {
                draw_rectangle(rr.x, rr.y, rr.w, rr.h, C_ACTIVE);
            } else if hot {
                draw_rectangle(rr.x, rr.y, rr.w, rr.h, Color::new(0.32, 0.38, 0.5, 0.85));
            }
            draw_tree_icon(r.icon, x + 9.0, y + rh / 2.0);
            let col = if r.error { C_ERR } else if r.dim { C_DIM } else { C_TEXT };
            text(&r.label, x + 21.0, y + 15.0, 15, col);
            if r.bold {
                draw_line(x + 21.0, y + 17.0, x + 21.0 + tw, y + 17.0, 1.0, col);
            }
            if r.error {
                text("!", x + 24.0 + tw, y + 15.0, 17, C_ERR);
            }
            let id = match r.sel {
                Sel::Plane(p) => format!("tree_plane_{}", p),
                Sel::Feature(f) => format!("tree_feat_{}", f),
                Sel::Comp(c) => format!("tree_comp_{}", c),
                Sel::Constraint(c) => format!("tree_con_{}", c),
                _ => format!("tree_row_{}", i),
            };
            self.ui.reg(&id, rr);
            self.ui.block(rr);
            if let Some(e) = self.error_for(r.sel) {
                self.ui.tip(rr, &e);
            }
            if self.ui.clicked(rr) {
                clicked = Some((r.sel, self.ui.inp.double_click));
            }
            if self.ui.inp.pressed[RMB] && self.ui.hover(rr) {
                right = Some((r.sel, self.ui.inp.mouse));
            }
        }
        if rows.len() > max_rows {
            text(&format!("... {} more", rows.len() - max_rows), x0 + 20.0, y0 + max_rows as f32 * rh + 14.0, 14, C_DIM);
        }
        if let Some((s, dbl)) = clicked {
            self.tree_click(s, dbl);
        }
        if let Some((s, p)) = right {
            if s != Sel::None {
                self.sel = s;
                self.ctx_menu = Some((p, s));
                self.menu_open = None;
            }
        }
    }

    fn error_for(&self, s: Sel) -> Option<String> {
        if let Sel::Feature(id) = s {
            return self.part().and_then(|p| p.cache.errors.get(&id).cloned());
        }
        None
    }

    fn tree_click(&mut self, s: Sel, dbl: bool) {
        if s == Sel::None {
            return;
        }
        if self.tool == Tool::PickSketchSupport {
            if let Sel::Feature(fid) = s {
                if self.part().and_then(|pt| pt.ref_plane_frame(fid)).is_some() {
                    self.sketch_on_ref_plane(fid);
                    return;
                }
            }
            if let Sel::Plane(p) = s {
                self.start_sketch_on(App::plane_frame(p), ["xy plane", "yz plane", "zx plane"][p as usize].into());
                return;
            }
        }
        if let Sel::Comp(id) = s {
            if self.active_comp.is_some() && self.active_comp != Some(id) {
                // switch from editing a part back to the assembly
                self.back_to_assembly();
            }
        }
        if self.sketch.is_some() {
            if let Sel::Feature(id) = s {
                if self.sketch.as_ref().map(|k| k.feat) == Some(id) {
                    return;
                }
            }
            self.exit_sketch();
        }
        self.sel = s;
        match s {
            Sel::Feature(_) => {
                if dbl {
                    self.open_properties(s);
                } else if let Dialog::Feature { .. } = self.dialog {
                    self.dialog = Dialog::None;
                }
            }
            Sel::Comp(id) => {
                if dbl && self.active_comp.is_none() {
                    self.edit_component(id);
                } else if self.active_comp.is_none() {
                    self.dialog = Dialog::Comp(id);
                }
            }
            Sel::Constraint(i) => self.dialog = Dialog::Constraint(i),
            Sel::Root => {
                if self.active_comp.is_some() {
                    self.back_to_assembly();
                }
                self.sel = Sel::None;
            }
            Sel::Plane(_) => {
                self.say("Plane selected. Click Sketch to draw on it.");
            }
            _ => {}
        }
    }

    fn draw_ctx_menu(&mut self) {
        let (p, s) = match self.ctx_menu {
            Some(c) => c,
            None => return,
        };
        let mut items: Vec<&str> = vec![];
        match s {
            Sel::Feature(id) => {
                let is_sketch = self.part().and_then(|p| p.feature(id)).map_or(false, |f| matches!(f.kind, FeatureKind::Sketch(_)));
                items.push(if is_sketch { "Edit Sketch" } else { "Edit Definition" });
                if is_sketch {
                    items.push("Hide / Show");
                } else {
                    items.push("Deactivate / Activate");
                }
                items.push("Delete");
            }
            Sel::Comp(_) => {
                items.extend(["Edit Part", "Properties", "Duplicate", "Fix / Unfix", "Hide / Show", "Delete"]);
            }
            Sel::Constraint(_) => items.extend(["Edit", "Delete"]),
            Sel::Plane(_) => items.push("Sketch on this plane"),
            Sel::Face { .. } => items.extend(["Sketch on this face", "Hole here", "Measure from here"]),
            _ => {}
        }
        if items.is_empty() {
            self.ctx_menu = None;
            return;
        }
        let w = 200.0;
        let r = Rect::new(p.x, p.y, w, items.len() as f32 * 24.0 + 6.0);
        self.ui.panel(r, None);
        let mut chosen = None;
        for (i, it) in items.iter().enumerate() {
            let rr = Rect::new(r.x + 2.0, r.y + 3.0 + i as f32 * 24.0, w - 4.0, 24.0);
            self.ui.reg(&format!("ctx_{}", it), rr);
            if self.ui.hover(rr) {
                draw_rectangle(rr.x, rr.y, rr.w, rr.h, C_HOT);
            }
            text(it, rr.x + 10.0, rr.y + 17.0, 16, C_TEXT);
            if self.ui.clicked(rr) {
                chosen = Some(*it);
            }
        }
        if let Some(c) = chosen {
            self.ctx_menu = None;
            match (c, s) {
                ("Edit Sketch" | "Edit Definition" | "Properties" | "Edit", _) => self.open_properties(s),
                ("Hide / Show", Sel::Feature(id)) => {
                    if let Some(f) = self.part_mut().and_then(|p| p.feature_mut(id)) {
                        f.hidden = !f.hidden;
                    }
                }
                ("Deactivate / Activate", Sel::Feature(id)) => {
                    self.checkpoint();
                    if let Some(p) = self.part_mut() {
                        if let Some(f) = p.feature_mut(id) {
                            f.suppressed = !f.suppressed;
                        }
                        p.touch_id(id);
                    }
                }
                ("Delete", _) => self.delete_selection(),
                ("Edit Part", Sel::Comp(id)) => self.edit_component(id),
                ("Duplicate", Sel::Comp(id)) => self.duplicate_component(id),
                ("Fix / Unfix", Sel::Comp(id)) => self.toggle_fix(id),
                ("Hide / Show", Sel::Comp(id)) => {
                    if let Some(c) = self.product_mut().and_then(|p| p.comp_mut(id)) {
                        c.hidden = !c.hidden;
                    }
                }
                ("Sketch on this face", Sel::Face { p, n, .. }) => self.sketch_on_face(p, n),
                ("Hole here", Sel::Face { p, n, .. }) => self.hole_on_face(p, n),
                ("Measure from here", Sel::Face { p, .. }) => {
                    let w = self.part_model().transform_point3(p);
                    self.start_tool(Tool::Measure(Some(w)), "Measure: click the second point.");
                }
                ("Sketch on this plane", Sel::Plane(pl)) => self.start_sketch_on(App::plane_frame(pl), ["xy plane", "yz plane", "zx plane"][pl as usize].into()),
                _ => {}
            }
        } else if (self.ui.inp.pressed[LMB] || self.ui.inp.pressed[RMB]) && !r.contains(self.ui.inp.mouse) && self.frame_no > 0 {
            if !self.ui.inp.pressed[RMB] || !self.ui.hover(r) {
                self.ctx_menu = None;
            }
        }
    }

    // ------------------------------------------------------------ view toolbar & status

    fn draw_viewbar(&mut self) {
        let items = [
            ("v_fit", "Fit All"),
            ("v_iso", "Iso"),
            ("v_front", "Front"),
            ("v_back", "Back"),
            ("v_left", "Left"),
            ("v_right", "Right"),
            ("v_top", "Top"),
            ("v_bottom", "Bottom"),
            ("v_normal", "Normal To"),
            ("v_proj", if self.view.persp { "Perspective" } else { "Parallel" }),
            ("v_edges", "Edges"),
        ];
        let bw = 36.0;
        let total = items.len() as f32 * (bw + 3.0);
        let avail = sw() - TOOLBAR_W;
        let x0 = (avail - total) / 2.0;
        let y = sh() - STATUS_H - 42.0;
        let bar = Rect::new(x0 - 6.0, y - 4.0, total + 9.0, bw + 8.0);
        self.ui.block(bar);
        draw_rectangle(bar.x, bar.y, bar.w, bar.h, Color::new(0.12, 0.13, 0.15, 0.85));
        draw_rectangle_lines(bar.x, bar.y, bar.w, bar.h, 1.0, C_BORDER);
        let mut clicked = None;
        for (i, (id, label)) in items.iter().enumerate() {
            let r = Rect::new(x0 + i as f32 * (bw + 3.0), y, bw, bw);
            let active = (*id == "v_edges" && self.show_edges) || (*id == "v_proj" && !self.view.persp);
            self.ui.reg(id, r);
            self.ui.block(r);
            let hot = self.ui.hover(r);
            draw_rectangle(r.x, r.y, r.w, r.h, if active { C_ACTIVE } else if hot { C_HOT } else { C_BUTTON });
            draw_rectangle_lines(r.x, r.y, r.w, r.h, 1.0, C_BORDER);
            draw_view_icon(id, r.x + r.w / 2.0, r.y + r.h / 2.0);
            let tip = match *id {
                "v_fit" => "Fit All In",
                "v_iso" => "Isometric View",
                "v_front" => "Front View",
                "v_back" => "Back View",
                "v_left" => "Left View",
                "v_right" => "Right View",
                "v_top" => "Top View",
                "v_bottom" => "Bottom View",
                "v_normal" => "Normal View (look straight at the selected plane, face or sketch)",
                "v_proj" => "Perspective / Parallel projection",
                _ => "Shading with Edges on/off",
            };
            let _ = label;
            self.ui.tip(r, tip);
            if self.ui.clicked(r) {
                clicked = Some(*id);
            }
        }
        let normal_to = |s: &App| -> Option<(Vec3, Vec3)> {
            if let Some(sk) = s.sketch_data() {
                let m = s.part_model();
                return Some((m.transform_vector3(sk.frame.n()).as_vec3(), m.transform_vector3(sk.frame.v).as_vec3()));
            }
            if let Sel::Feature(id) = s.sel {
                if let Some(FeatureKind::Sketch(sk)) = s.part().and_then(|p| p.feature(id)).map(|f| &f.kind) {
                    let m = s.part_model();
                    return Some((m.transform_vector3(sk.frame.n()).as_vec3(), m.transform_vector3(sk.frame.v).as_vec3()));
                }
            }
            if let Sel::Plane(p) = s.sel {
                let f = App::plane_frame(p);
                return Some((f.n().as_vec3(), f.v.as_vec3()));
            }
            if let Sel::Face { p, n, .. } = s.sel {
                let f = crate::geom::Frame::from_normal(p, n);
                let m = s.part_model();
                return Some((m.transform_vector3(f.n()).as_vec3(), m.transform_vector3(f.v).as_vec3()));
            }
            None
        };
        match clicked {
            Some("v_fit") => {
                if self.sketch.is_some() {
                    let (r, t) = (self.view.rot, self.view.target);
                    let b = self.scene_bbox();
                    let d = if b.valid() { (b.size().length() as f32 * 1.4).max(60.0) / (2.0 * (self.view.fov / 2.0).tan()) } else { self.view.dist };
                    self.view.animate_to(r, t, d);
                } else {
                    self.fit_all()
                }
            }
            Some("v_iso") => self.set_view(vec3(1.0, 1.0, 0.8), Vec3::Z),
            Some("v_front") => self.set_view(Vec3::X, Vec3::Z),
            Some("v_back") => self.set_view(-Vec3::X, Vec3::Z),
            Some("v_left") => self.set_view(-Vec3::Y, Vec3::Z),
            Some("v_right") => self.set_view(Vec3::Y, Vec3::Z),
            Some("v_top") => self.set_view(Vec3::Z, -Vec3::X),
            Some("v_bottom") => self.set_view(-Vec3::Z, Vec3::X),
            Some("v_normal") => match normal_to(self) {
                Some((n, v)) => {
                    let rot = quat_from_back_up(n, v);
                    let (t, d) = (self.view.target, self.view.dist);
                    self.view.animate_to(rot, t, d);
                }
                None => self.err("Normal To: select a plane or a sketch first."),
            },
            Some("v_proj") => self.view.persp = !self.view.persp,
            Some("v_edges") => self.show_edges = !self.show_edges,
            _ => {}
        }
    }

    fn draw_status(&mut self) {
        let w = sw();
        let y = sh() - STATUS_H;
        let r = Rect::new(0.0, y, w, STATUS_H);
        self.ui.block(r);
        self.ui.reg("statusbar", r);
        draw_rectangle(r.x, r.y, r.w, r.h, C_PANEL_DARK);
        draw_line(0.0, y, w, y, 1.0, C_BORDER);
        // information on the right: size and volume of the active thing
        let mut info = self.info_text();
        if self.building {
            let dots = ".".repeat(((get_time() * 3.0) as usize) % 4);
            info = format!("Updating{:<3}   {}", dots, info);
        }
        let iw = text_w(&info, 15);
        text(&info, w - iw - 10.0, y + 17.0, 15, C_TEXT);
        let msg = fit_tail(&self.status, w - iw - 30.0, 15);
        text(&msg, 8.0, y + 17.0, 15, if self.status_err { C_ERR } else { C_TEXT });
    }

    pub fn info_text(&self) -> String {
        let mesh_info = |m: &crate::geom::Mesh| -> String {
            if !m.bbox.valid() {
                return "empty".into();
            }
            let s = m.bbox.size();
            format!("{} x {} x {} mm   volume {} cm3", fmt((s.x * 10.0).round() / 10.0), fmt((s.y * 10.0).round() / 10.0), fmt((s.z * 10.0).round() / 10.0), fmt((m.volume() / 100.0).round() / 10.0))
        };
        if let Some(p) = self.part() {
            if let Some(m) = &p.cache.mesh {
                return format!("{}: {}", p.name, mesh_info(m));
            }
        }
        if let (Sel::Comp(id), Some(pr)) = (self.sel, self.product()) {
            if let Some(c) = pr.comp(id) {
                if let Some(m) = &c.part.cache.mesh {
                    return format!("{}: {}", c.part.name, mesh_info(m));
                }
            }
        }
        if let Some(pr) = self.product() {
            return format!("{} components, {} constraints   units: mm", pr.comps.len(), pr.constraints.len());
        }
        "units: mm".into()
    }

    // ------------------------------------------------------------ dialogs

    fn dialog_rect(&self, h: f32) -> Rect {
        Rect::new(sw() - TOOLBAR_W - DIALOG_W - 10.0, MENU_H + 128.0, DIALOG_W, h)
    }

    fn draw_dialog(&mut self) {
        match &self.dialog {
            Dialog::Feature { id, .. } => {
                let id = *id;
                self.feature_dialog(id);
            }
            Dialog::Comp(id) => {
                let id = *id;
                self.comp_dialog(id);
            }
            Dialog::Constraint(i) => {
                let i = *i;
                self.constraint_dialog(i);
            }
            Dialog::Dim(i) => {
                let i = *i;
                if self.sketch.is_some() {
                    self.dim_dialog(i);
                } else {
                    self.dialog = Dialog::None;
                }
            }
            _ => {
                if self.sketch.is_some() {
                    self.shape_dialog();
                }
            }
        }
    }

    fn feature_dialog(&mut self, id: u32) {
        let f = match self.part().and_then(|p| p.feature(id)) {
            Some(f) => f.clone(),
            None => {
                self.dialog = Dialog::None;
                return;
            }
        };
        let mut kind = f.kind.clone();
        let mut name = f.name.clone();
        let err = self.part().and_then(|p| p.cache.errors.get(&id).cloned());
        let lines = match &kind {
            FeatureKind::Pad { .. } => 5,
            FeatureKind::Pocket { .. } => 5,
            FeatureKind::Shaft { .. } | FeatureKind::Groove { .. } => 4,
            FeatureKind::Primitive { prim: PrimKind::Box, .. } => 8,
            FeatureKind::Primitive { prim: PrimKind::Cylinder, .. } => 8,
            FeatureKind::Primitive { .. } => 6,
            FeatureKind::Mirror { .. } => 2,
            FeatureKind::RectPattern { .. } => 8,
            FeatureKind::CircPattern { .. } => 5,
            FeatureKind::Import { .. } => 3,
            FeatureKind::Sketch(_) => 1,
            FeatureKind::Hole { htype: 0, .. } => 8,
            FeatureKind::Hole { .. } => 10,
            FeatureKind::Fillet { .. } | FeatureKind::Chamfer { .. } => 4,
            FeatureKind::Draft { .. } => 6,
            FeatureKind::Shell { .. } | FeatureKind::Thickness { .. } => 4,
            FeatureKind::Translate { .. } => 3,
            FeatureKind::Rotate { .. } => 2,
            FeatureKind::Symmetry { .. } | FeatureKind::Scale { .. } => 1,
            FeatureKind::RefPlane { .. } => 2,
            FeatureKind::Loft { .. } => self.part().map(|p| p.sketches().len()).unwrap_or(0).min(12) + 2,
            FeatureKind::Sweep { .. } => 3,
            FeatureKind::Split { .. } => 4,
            FeatureKind::Stiffener { .. } => 5,
        };
        let h = 24.0 + 40.0 + lines as f32 * 30.0 + 50.0 + if err.is_some() { 22.0 } else { 0.0 };
        let r = self.dialog_rect(h);
        let title = format!("{} Definition", kind.type_name());
        self.ui.panel(r, Some(&title));
        let lx = r.x + 10.0;
        let fx = r.x + 150.0;
        let fw = r.w - 160.0;
        let mut y = r.y + 34.0;
        text("Name", lx, y + 17.0, 16, C_TEXT);
        let mut changed = false;
        if self.ui.text_field("dlg_name", Rect::new(fx, y, fw, 24.0), &mut name) {
            changed = true;
        }
        y += 34.0;
        let sketch_name = |s: u32| self.part().and_then(|p| p.feature(s)).map(|f| f.name.clone()).unwrap_or_else(|| "?".into());
        macro_rules! num {
            ($label:expr, $id:expr, $v:expr, $step:expr) => {{
                text($label, lx, y + 17.0, 16, C_TEXT);
                if self.ui.num_field($id, Rect::new(fx, y, fw, 24.0), $v, $step) {
                    changed = true;
                }
                y += 30.0;
            }};
        }
        macro_rules! check {
            ($label:expr, $id:expr, $v:expr) => {{
                if self.ui.checkbox($id, fx, y + 2.0, $label, $v) {
                    changed = true;
                }
                y += 30.0;
            }};
        }
        macro_rules! info {
            ($label:expr, $val:expr) => {{
                text($label, lx, y + 17.0, 16, C_TEXT);
                text(&$val, fx, y + 17.0, 16, C_TITLE);
                y += 30.0;
            }};
        }
        macro_rules! axis {
            ($label:expr, $id:expr, $v:expr, $opts:expr) => {{
                text($label, lx, y + 17.0, 16, C_TEXT);
                let mut s = match *$v {
                    Axis3::X => 0,
                    Axis3::Y => 1,
                    Axis3::Z => 2,
                };
                if self.ui.choice($id, Rect::new(fx, y, fw, 24.0), &$opts, &mut s) {
                    *$v = [Axis3::X, Axis3::Y, Axis3::Z][s];
                    changed = true;
                }
                y += 30.0;
            }};
        }
        match &mut kind {
            FeatureKind::Pad { sketch, length, symmetric, reverse } => {
                info!("Profile", sketch_name(*sketch));
                num!("Length (mm)", "dlg_len", length, 1.0);
                check!("Mirrored extent", "dlg_sym", symmetric);
                check!("Reverse direction", "dlg_rev", reverse);
                y += 0.0;
            }
            FeatureKind::Pocket { sketch, depth, through, reverse } => {
                info!("Profile", sketch_name(*sketch));
                num!("Depth (mm)", "dlg_depth", depth, 1.0);
                check!("Up to last (through all)", "dlg_thru", through);
                check!("Reverse direction", "dlg_rev", reverse);
            }
            FeatureKind::Shaft { sketch, angle, axis_h } | FeatureKind::Groove { sketch, angle, axis_h } => {
                info!("Profile", sketch_name(*sketch));
                num!("Angle (deg)", "dlg_angle", angle, 15.0);
                text("Axis", lx, y + 17.0, 16, C_TEXT);
                let mut s = if *axis_h { 1 } else { 0 };
                if self.ui.choice("dlg_axis", Rect::new(fx, y, fw, 24.0), &["V axis", "H axis"], &mut s) {
                    *axis_h = s == 1;
                    changed = true;
                }
                y += 30.0;
            }
            FeatureKind::Hole { pos, diameter, depth, through, htype, d2, depth2, angle2, .. } => {
                text("Type", lx, y + 17.0, 16, C_TEXT);
                let mut t = *htype as usize;
                if self.ui.choice("dlg_htype", Rect::new(fx, y, fw, 24.0), &["Simple", "C'bored", "C'sunk"], &mut t) {
                    *htype = t as u8;
                    changed = true;
                }
                y += 30.0;
                num!("Diameter (mm)", "dlg_dia", diameter, 0.5);
                num!("Depth (mm)", "dlg_depth", depth, 1.0);
                if *htype > 0 {
                    if *d2 <= *diameter {
                        *d2 = (*diameter * 1.8 * 10.0).round() / 10.0;
                    }
                    if *depth2 <= 0.0 {
                        *depth2 = (*diameter * 0.6 * 10.0).round() / 10.0;
                    }
                    if *angle2 <= 0.0 {
                        *angle2 = 90.0;
                    }
                    let lab = if *htype == 1 { "Head diameter" } else { "Sink diameter" };
                    num!(lab, "dlg_d2", d2, 0.5);
                    if *htype == 1 {
                        num!("Head depth (mm)", "dlg_depth2", depth2, 0.5);
                    } else {
                        num!("Sink angle (deg)", "dlg_angle2", angle2, 5.0);
                    }
                }
                check!("Up to last (through all)", "dlg_thru", through);
                num!("Position X", "dlg_px", &mut pos.x, 1.0);
                num!("Position Y", "dlg_py", &mut pos.y, 1.0);
                num!("Position Z", "dlg_pz", &mut pos.z, 1.0);
            }
            FeatureKind::Primitive { prim, pos, a, b, c, axis, subtract } => {
                match prim {
                    PrimKind::Box => {
                        num!("Size X (mm)", "dlg_a", a, 1.0);
                        num!("Size Y (mm)", "dlg_b", b, 1.0);
                        num!("Size Z (mm)", "dlg_c", c, 1.0);
                        num!("Centre X", "dlg_px", &mut pos.x, 1.0);
                        num!("Centre Y", "dlg_py", &mut pos.y, 1.0);
                        num!("Centre Z", "dlg_pz", &mut pos.z, 1.0);
                    }
                    PrimKind::Cylinder => {
                        num!("Diameter (mm)", "dlg_a", a, 1.0);
                        num!("Height (mm)", "dlg_b", b, 1.0);
                        axis!("Axis", "dlg_axis", axis, ["X", "Y", "Z"]);
                        num!("Base centre X", "dlg_px", &mut pos.x, 1.0);
                        num!("Base centre Y", "dlg_py", &mut pos.y, 1.0);
                        num!("Base centre Z", "dlg_pz", &mut pos.z, 1.0);
                    }
                    PrimKind::Sphere => {
                        num!("Diameter (mm)", "dlg_a", a, 1.0);
                        num!("Centre X", "dlg_px", &mut pos.x, 1.0);
                        num!("Centre Y", "dlg_py", &mut pos.y, 1.0);
                        num!("Centre Z", "dlg_pz", &mut pos.z, 1.0);
                    }
                }
                check!("Cut out instead of add", "dlg_sub", subtract);
            }
            FeatureKind::Mirror { plane } => {
                axis!("Mirror plane", "dlg_plane", plane, ["yz", "zx", "xy"]);
            }
            FeatureKind::RectPattern { feature, dir, count, step, dir2, count2, step2 } => {
                info!("Object", sketch_name(*feature));
                axis!("Direction 1", "dlg_d1", dir, ["X", "Y", "Z"]);
                num!("Instances", "dlg_n1", count, 1.0);
                num!("Spacing (mm)", "dlg_s1", step, 1.0);
                axis!("Direction 2", "dlg_d2", dir2, ["X", "Y", "Z"]);
                num!("Instances", "dlg_n2", count2, 1.0);
                num!("Spacing (mm)", "dlg_s2", step2, 1.0);
                *count = count.round().clamp(1.0, 200.0);
                *count2 = count2.round().clamp(1.0, 200.0);
            }
            FeatureKind::CircPattern { feature, axis, count, total } => {
                info!("Object", sketch_name(*feature));
                axis!("Axis", "dlg_axis", axis, ["X", "Y", "Z"]);
                num!("Instances", "dlg_n1", count, 1.0);
                num!("Total angle", "dlg_tot", total, 15.0);
                *count = count.round().clamp(1.0, 360.0);
            }
            FeatureKind::Import { file, scale, .. } => {
                let short = std::path::Path::new(file.as_str()).file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
                info!("File", fit_tail(&short, fw, 16));
                num!("Scale", "dlg_scale", scale, 0.1);
                if *scale <= 0.0 {
                    *scale = 1.0;
                }
            }
            FeatureKind::Sketch(_) => {}
            FeatureKind::Fillet { edges, radius: size } | FeatureKind::Chamfer { edges, length: size } => {
                let is_fillet = matches!(f.kind, FeatureKind::Fillet { .. });
                text("Object(s)", lx, y + 17.0, 16, C_TEXT);
                text(&format!("{} edge(s)", edges.len()), fx, y + 17.0, 16, C_ACCENT);
                if self.ui.button("dlg_clear", Rect::new(fx + fw - 70.0, y, 70.0, 24.0), "Clear") {
                    edges.clear();
                    changed = true;
                }
                y += 30.0;
                if is_fillet {
                    num!("Radius (mm)", "dlg_size", size, 0.5);
                } else {
                    num!("Length (mm)", "dlg_size", size, 0.5);
                    text("Angle", lx, y + 17.0, 16, C_DIM);
                    text("45 deg (equal lengths)", fx, y + 17.0, 16, C_DIM);
                    y += 30.0;
                }
                text("Click edges in the 3D view to add / remove them.", lx, y + 14.0, 14, C_DIM);
                text("Smoothly connected edges come along.", lx, y + 30.0, 14, C_DIM);
                y += 30.0;
            }
            FeatureKind::Draft { faces, angle, neutral, reverse } => {
                text("Faces to draft", lx, y + 17.0, 16, C_TEXT);
                text(&format!("{} face(s)", faces.len()), fx, y + 17.0, 16, C_ACCENT);
                if self.ui.button("dlg_clear", Rect::new(fx + fw - 70.0, y, 70.0, 24.0), "Clear") {
                    faces.clear();
                    changed = true;
                }
                y += 30.0;
                num!("Angle (deg)", "dlg_angle", angle, 1.0);
                axis!("Pulling direction", "dlg_pull", neutral, ["X", "Y", "Z"]);
                check!("Reverse direction", "dlg_rev", reverse);
                text("Neutral element: the plane through the origin", lx, y + 14.0, 14, C_DIM);
                text("square to the pulling direction. Click side faces.", lx, y + 30.0, 14, C_DIM);
                y += 30.0;
            }
            FeatureKind::Shell { faces, thickness } | FeatureKind::Thickness { faces, thickness } => {
                let shell = matches!(f.kind, FeatureKind::Shell { .. });
                text(if shell { "Faces to remove" } else { "Faces" }, lx, y + 17.0, 16, C_TEXT);
                text(&format!("{} face(s)", faces.len()), fx, y + 17.0, 16, C_ACCENT);
                if self.ui.button("dlg_clear", Rect::new(fx + fw - 70.0, y, 70.0, 24.0), "Clear") {
                    faces.clear();
                    changed = true;
                }
                y += 30.0;
                num!(if shell { "Inside thickness" } else { "Thickness (mm)" }, "dlg_thick", thickness, 0.5);
                text(if shell { "Click the faces to open (none = closed hollow)." } else { "Click faces; a negative value removes material." }, lx, y + 14.0, 14, C_DIM);
                y += 30.0;
            }
            FeatureKind::Translate { v } => {
                num!("Distance X (mm)", "dlg_tx", &mut v.x, 1.0);
                num!("Distance Y (mm)", "dlg_ty", &mut v.y, 1.0);
                num!("Distance Z (mm)", "dlg_tz", &mut v.z, 1.0);
            }
            FeatureKind::Rotate { axis, angle } => {
                axis!("Axis", "dlg_axis", axis, ["X", "Y", "Z"]);
                num!("Angle (deg)", "dlg_angle", angle, 15.0);
            }
            FeatureKind::Symmetry { plane } => {
                axis!("Reference plane", "dlg_plane", plane, ["yz", "zx", "xy"]);
            }
            FeatureKind::Scale { factor } => {
                num!("Ratio", "dlg_ratio", factor, 0.1);
            }
            FeatureKind::RefPlane { base, face, offset } => {
                text("Reference", lx, y + 17.0, 16, C_TEXT);
                let opts: Vec<&str> = if face.is_some() { vec!["xy", "yz", "zx", "face"] } else { vec!["xy", "yz", "zx"] };
                let mut b = (*base as usize).min(opts.len() - 1);
                if self.ui.choice("dlg_base", Rect::new(fx, y, fw, 24.0), &opts, &mut b) {
                    *base = b as u8;
                    changed = true;
                }
                y += 30.0;
                num!("Offset (mm)", "dlg_offset", offset, 1.0);
            }
            FeatureKind::Loft { sections, remove } => {
                text("Sections (in order):", lx, y + 17.0, 16, C_TEXT);
                y += 30.0;
                let all: Vec<(u32, String)> = self.part().map(|p| p.sketches().iter().filter_map(|s| p.feature(*s).map(|f| (*s, f.name.clone()))).collect()).unwrap_or_default();
                for (k, (sid, sname)) in all.iter().enumerate().take(12) {
                    let on = sections.contains(sid);
                    let label = match sections.iter().position(|x| x == sid) {
                        Some(i) => format!("{}  (section {})", sname, i + 1),
                        None => sname.clone(),
                    };
                    if self.ui.button_ex(&format!("dlg_sec{}", k), Rect::new(lx, y, r.w - 20.0, 24.0), &label, on, true) {
                        if on {
                            sections.retain(|x| x != sid);
                        } else {
                            sections.push(*sid);
                            // keep the tree order
                            let order: Vec<u32> = all.iter().map(|a| a.0).collect();
                            sections.sort_by_key(|x| order.iter().position(|o| o == x));
                        }
                        changed = true;
                    }
                    y += 30.0;
                }
                check!("Remove material", "dlg_rem", remove);
            }
            FeatureKind::Split { base, plane, offset, flip } => {
                let planes: Vec<(u32, String)> = self.part().map(|p| p.ref_planes()).unwrap_or_default();
                text("Splitting plane", lx, y + 17.0, 16, C_TEXT);
                let opts: Vec<&str> = if planes.is_empty() { vec!["xy", "yz", "zx"] } else { vec!["xy", "yz", "zx", "Plane"] };
                let mut b = (*base as usize).min(opts.len() - 1);
                if self.ui.choice("dlg_base", Rect::new(fx, y, fw, 24.0), &opts, &mut b) {
                    *base = b as u8;
                    if b == 3 && !planes.iter().any(|x| x.0 == *plane) {
                        *plane = planes.last().map(|x| x.0).unwrap_or(0);
                    }
                    changed = true;
                }
                y += 30.0;
                if *base == 3 {
                    let name = planes.iter().find(|x| x.0 == *plane).map(|x| x.1.clone()).unwrap_or_else(|| "?".into());
                    text("Plane", lx, y + 17.0, 16, C_TEXT);
                    if self.ui.button("dlg_plane_pick", Rect::new(fx, y, fw, 24.0), &format!("{}   >", name)) {
                        let i = planes.iter().position(|x| x.0 == *plane).map(|i| (i + 1) % planes.len().max(1)).unwrap_or(0);
                        if let Some(n) = planes.get(i) {
                            *plane = n.0;
                            changed = true;
                        }
                    }
                }
                y += 30.0;
                num!("Offset (mm)", "dlg_offset", offset, 1.0);
                check!("Keep the other side", "dlg_flip", flip);
            }
            FeatureKind::Stiffener { sketch, thickness, depth, reverse } => {
                info!("Profile", sketch_name(*sketch));
                num!("Thickness (mm)", "dlg_thick", thickness, 0.5);
                num!("Depth (mm)", "dlg_depth", depth, 1.0);
                check!("Reverse direction", "dlg_rev", reverse);
                text("Draw the stiffener as lines in a sketch.", lx, y + 14.0, 14, C_DIM);
                y += 30.0;
            }
            FeatureKind::Sweep { profile, path, remove } => {
                let all: Vec<(u32, String)> = self.part().map(|p| p.sketches().iter().filter_map(|s| p.feature(*s).map(|f| (*s, f.name.clone()))).collect()).unwrap_or_default();
                let name_of = |id: u32| all.iter().find(|a| a.0 == id).map(|a| a.1.clone()).unwrap_or_else(|| "?".into());
                let next_of = |id: u32| -> u32 {
                    let i = all.iter().position(|a| a.0 == id).map(|i| (i + 1) % all.len().max(1)).unwrap_or(0);
                    all.get(i).map(|a| a.0).unwrap_or(id)
                };
                text("Profile", lx, y + 17.0, 16, C_TEXT);
                if self.ui.button("dlg_profile", Rect::new(fx, y, fw, 24.0), &format!("{}   >", name_of(*profile))) {
                    *profile = next_of(*profile);
                    changed = true;
                }
                y += 30.0;
                text("Center curve", lx, y + 17.0, 16, C_TEXT);
                if self.ui.button("dlg_path", Rect::new(fx, y, fw, 24.0), &format!("{}   >", name_of(*path))) {
                    *path = next_of(*path);
                    changed = true;
                }
                y += 30.0;
                check!("Remove material (Slot)", "dlg_rem", remove);
            }
        }
        if let Some(e) = &err {
            text(&format!("Problem: {}", e), lx, y + 14.0, 15, C_ERR);
            y += 22.0;
        }
        let _ = y;
        let ok = self.ui.button("dlg_ok", Rect::new(r.x + r.w - 190.0, r.y + r.h - 36.0, 86.0, 28.0), "OK");
        let cancel = self.ui.button("dlg_cancel", Rect::new(r.x + r.w - 96.0, r.y + r.h - 36.0, 86.0, 28.0), "Cancel");
        if changed {
            if let Some(p) = self.part_mut() {
                if let Some(ff) = p.feature_mut(id) {
                    ff.kind = kind;
                    ff.name = name;
                }
                p.touch_id(id);
            }
        }
        if ok {
            self.dialog = Dialog::None;
            self.ui.focus = None;
            let e = self.part_mut().map(|p| {
                p.ensure_built();
                p.cache.errors.get(&id).cloned()
            });
            match e.flatten() {
                Some(e) => self.err(&format!("{}: {}", f.name, e)),
                None => self.say(&format!("{} done.", f.name)),
            }
        } else if cancel {
            let d = std::mem::replace(&mut self.dialog, Dialog::None);
            self.ui.focus = None;
            if let Dialog::Feature { new, backup, .. } = d {
                if new {
                    // throw away the half-made feature, and show its sketch again
                    self.do_undo();
                    self.redo.clear();
                } else if let Some(b) = backup {
                    if let Some(p) = self.part_mut() {
                        if let Some(ff) = p.feature_mut(id) {
                            *ff = b;
                        }
                        p.touch_id(id);
                    }
                    self.undo.pop();
                }
            }
            self.say("Cancelled.");
        }
    }

    fn shape_dialog(&mut self) {
        let (tool, sel) = match self.sketch.as_ref() {
            Some(s) => (s.tool, s.sel.clone()),
            None => return,
        };
        // tool options for Corner / Chamfer
        if matches!(tool, SkTool::Corner | SkTool::Chamfer) {
            let r = self.dialog_rect(100.0);
            let title = if tool == SkTool::Corner { "Corner" } else { "Chamfer" };
            self.ui.panel(r, Some(title));
            let label = if tool == SkTool::Corner { "Radius (mm)" } else { "Length (mm)" };
            text(label, r.x + 10.0, r.y + 51.0, 16, C_TEXT);
            let sk = self.sketch.as_mut().unwrap();
            let v = if tool == SkTool::Corner { &mut sk.corner_r } else { &mut sk.chamfer_d };
            let mut val = *v;
            if self.ui.num_field("sk_opt", Rect::new(r.x + 150.0, r.y + 34.0, r.w - 160.0, 24.0), &mut val, 0.5) {
                let sk = self.sketch.as_mut().unwrap();
                let v = if tool == SkTool::Corner { &mut sk.corner_r } else { &mut sk.chamfer_d };
                *v = val.max(0.01);
            }
            text("Then click a corner where two lines meet.", r.x + 10.0, r.y + 84.0, 15, C_DIM);
            return;
        }
        if matches!(tool, SkTool::MeasureBetween | SkTool::MeasureItem) {
            let (last, keep, first) = {
                let sk = self.sketch.as_ref().unwrap();
                (sk.measures.last().cloned(), sk.keep_measures, sk.meas_first.clone())
            };
            let rows = last.as_ref().map(|m| m.rows.len()).unwrap_or(0);
            let h = 24.0 + 66.0 + rows as f32 * 22.0 + 46.0;
            let r = self.dialog_rect(h);
            let title = if tool == SkTool::MeasureBetween { "Measure Between" } else { "Measure Item" };
            self.ui.panel(r, Some(title));
            let step = match (tool, &first) {
                (SkTool::MeasureItem, _) => "Click an element or a point.".to_string(),
                (_, None) => "Click the first item (point, element or axis).".to_string(),
                (_, Some(f)) => format!("First: {}.  Click the second item.", self.sketch_data().map(|d| item_name(d, f)).unwrap_or_default()),
            };
            // wrap the hint onto two lines when it is too wide
            let (l1, l2) = match step.split_once(".  ") {
                Some((a, b)) if text_w(&step, 15) > r.w - 20.0 => (format!("{}.", a), b.to_string()),
                _ => (step.clone(), String::new()),
            };
            text(&fit_tail(&l1, r.w - 20.0, 15), r.x + 10.0, r.y + 44.0, 15, C_ACCENT);
            text(&l2, r.x + 10.0, r.y + 60.0, 15, C_ACCENT);
            let mut y = r.y + 68.0;
            match &last {
                Some(m) => {
                    for (l, v) in &m.rows {
                        text(l, r.x + 10.0, y + 14.0, 15, C_DIM);
                        text(v, r.x + 150.0, y + 14.0, 15, C_TEXT);
                        y += 22.0;
                    }
                }
                None => text("No measure yet.", r.x + 10.0, y + 14.0, 15, C_DIM),
            }
            let mut k = keep;
            if self.ui.checkbox("meas_keep", r.x + 10.0, r.y + r.h - 32.0, "Keep measures", &mut k) {
                self.sketch.as_mut().unwrap().keep_measures = k;
            }
            if self.ui.button("meas_clear", Rect::new(r.x + r.w - 96.0, r.y + r.h - 36.0, 86.0, 28.0), "Clear") {
                let sk = self.sketch.as_mut().unwrap();
                sk.measures.clear();
                sk.meas_first = None;
            }
            return;
        }
        if sel.is_empty() {
            let r = self.dialog_rect(170.0);
            self.ui.panel(r, Some("Sketcher"));
            let lines = [
                "1. Pick a tool on the right: Profile and Line",
                "   draw connected lines, plus rectangles,",
                "   circles, arcs, ellipses and splines.",
                "2. Click in the view. Ends snap together;",
                "   red dots are ends not yet connected.",
                "3. Select an element to type exact sizes.",
                "Closed outlines inside others become holes.",
            ];
            for (i, l) in lines.iter().enumerate() {
                text(l, r.x + 10.0, r.y + 44.0 + i as f32 * 18.0, 15, C_TEXT);
            }
            return;
        }
        if sel.len() > 1 {
            let r = self.dialog_rect(120.0);
            self.ui.panel(r, Some("Selection"));
            text(&format!("{} elements selected", sel.len()), r.x + 10.0, r.y + 48.0, 16, C_TEXT);
            text("Drag to move them, Mirror to copy them across a line.", r.x + 10.0, r.y + 68.0, 14, C_DIM);
            if self.ui.button("shape_delete", Rect::new(r.x + 10.0, r.y + r.h - 36.0, 90.0, 28.0), "Delete") {
                self.delete_sketch_selection();
            }
            if self.ui.button("shape_constr", Rect::new(r.x + 106.0, r.y + r.h - 36.0, 110.0, 28.0), "Construction") {
                self.toggle_construction();
            }
            return;
        }
        let idx = sel[0];
        let (shape, constr) = match self.sketch_data() {
            Some(d) if idx < d.shapes.len() => (d.shapes[idx].clone(), d.is_construction(idx)),
            _ => return,
        };
        let title = format!("{}{}", shape.kind_name(), if constr { " (construction)" } else { "" });
        let fields = shape.fields();
        let nfields = fields.len().min(24);
        let h = 24.0 + 16.0 + nfields as f32 * 30.0 + 46.0;
        let r = self.dialog_rect(h);
        self.ui.panel(r, Some(&title));
        let mut y = r.y + 34.0;
        let mut change: Option<(usize, f64)> = None;
        for (i, (label, v)) in fields.into_iter().enumerate().take(24) {
            text(&label, r.x + 10.0, y + 17.0, 16, C_TEXT);
            let mut val = v;
            if self.ui.num_field(&format!("shape_f{}", i), Rect::new(r.x + 150.0, y, r.w - 160.0, 24.0), &mut val, 1.0) {
                change = Some((i, val));
            }
            y += 30.0;
        }
        if self.ui.button("shape_delete", Rect::new(r.x + 10.0, r.y + r.h - 36.0, 80.0, 28.0), "Delete") {
            self.delete_sketch_selection();
            return;
        }
        if self.ui.button("shape_constr", Rect::new(r.x + 96.0, r.y + r.h - 36.0, 110.0, 28.0), if constr { "Standard" } else { "Construction" }) {
            self.toggle_construction();
            return;
        }
        if self.ui.button("shape_close", Rect::new(r.x + r.w - 86.0, r.y + r.h - 36.0, 76.0, 28.0), "Close") {
            if let Some(s) = self.sketch.as_mut() {
                s.sel.clear();
            }
        }
        if let Some((fi, val)) = change {
            let mut ns = shape.clone();
            let old_handles = ns.handle_points();
            ns.set_field(fi, val);
            if !ns.is_open() {
                let o = ns.outline();
                if o.len() < 3 || crate::geom::segments_self_intersect(&o) {
                    self.err("Those values make an invalid shape.");
                    return;
                }
            }
            let new_handles = ns.handle_points();
            self.checkpoint();
            if let Some(sd) = self.sketch_data_mut() {
                if idx < sd.shapes.len() {
                    sd.shapes[idx] = ns;
                    // keep connected elements attached to moved end points
                    for (o, n) in old_handles.iter().zip(new_handles.iter()) {
                        if (*o - *n).length() > 1e-12 {
                            sd.move_point(*o, *n, &[idx]);
                        }
                    }
                    sd.solve_dims(None);
                }
            }
            self.touch_sketch();
        }
    }

    /// "Constraint Definition": change a sketch dimension's value (the geometry follows).
    fn dim_dialog(&mut self, i: usize) {
        let d = match self.sketch_data().and_then(|s| s.dims.get(i)) {
            Some(d) => d.clone(),
            None => {
                self.dialog = Dialog::None;
                return;
            }
        };
        let ok_now = self.sketch_data().map_or(true, |s| s.dim_ok(&d));
        let r = self.dialog_rect(200.0);
        self.ui.panel(r, Some("Constraint Definition"));
        let lx = r.x + 10.0;
        let fx = r.x + 120.0;
        text("Type", lx, r.y + 50.0, 16, C_DIM);
        text(d.kind.name(), fx, r.y + 50.0, 16, C_TEXT);
        text("Value", lx, r.y + 84.0, 16, C_TEXT);
        let mut v = d.value;
        let changed = self.ui.num_field("dim_value", Rect::new(fx, r.y + 67.0, r.w - 180.0, 26.0), &mut v, if d.kind == DimKind::Angle { 5.0 } else { 1.0 });
        text(if d.kind == DimKind::Angle { "deg" } else { "mm" }, r.x + r.w - 50.0, r.y + 85.0, 16, C_DIM);
        let mut reference = d.reference;
        let ref_changed = self.ui.checkbox("dim_ref", fx, r.y + 104.0, "Reference (only shows)", &mut reference);
        if !ok_now {
            text("Cannot be reached with the other constraints.", lx, r.y + 146.0, 15, C_ERR);
        }
        if changed || ref_changed {
            let valid = match d.kind {
                DimKind::Angle => v > 0.0 && v < 180.0,
                DimKind::Distance => v >= 0.0,
                _ => v > 0.0,
            };
            if !valid {
                self.err("That value is not possible for this dimension.");
            } else if !self.set_dim_value(i, v, reference) {
                self.err("The sketch cannot reach that value with its other constraints.");
            } else {
                self.say(&format!("{} set to {}.", d.kind.name(), fmt(v)));
            }
        }
        let ok = self.ui.button("dim_ok", Rect::new(r.x + r.w - 190.0, r.y + r.h - 36.0, 86.0, 28.0), "OK");
        let cancel = self.ui.button("dim_cancel", Rect::new(r.x + r.w - 96.0, r.y + r.h - 36.0, 86.0, 28.0), "Cancel");
        if ok {
            self.dialog = Dialog::None;
            self.ui.focus = None;
        } else if cancel {
            self.dialog = Dialog::None;
            self.ui.focus = None;
            // back to how it was when the dialog opened
            self.do_undo();
            self.redo.clear();
            self.touch_sketch();
            self.say("Cancelled.");
        }
    }

    fn comp_dialog(&mut self, id: u32) {
        let c = match self.product().and_then(|p| p.comp(id)) {
            Some(c) => c.clone(),
            None => {
                self.dialog = Dialog::None;
                return;
            }
        };
        let r = self.dialog_rect(24.0 + 10.0 * 30.0 + 100.0);
        self.ui.panel(r, Some("Component Properties"));
        let lx = r.x + 10.0;
        let fx = r.x + 140.0;
        let fw = r.w - 150.0;
        let mut y = r.y + 34.0;
        let mut name = c.part.name.clone();
        let mut pos = c.pos;
        let (ex, ey, ez) = c.rot.to_euler(EulerRot::XYZ);
        let mut e = [ex.to_degrees(), ey.to_degrees(), ez.to_degrees()];
        for v in e.iter_mut() {
            *v = (*v * 1000.0).round() / 1000.0;
        }
        let mut fixed = c.fixed;
        let mut changed_pose = false;
        let mut changed_other = false;
        text("Part name", lx, y + 17.0, 16, C_TEXT);
        if self.ui.text_field("comp_name", Rect::new(fx, y, fw, 24.0), &mut name) {
            changed_other = true;
        }
        y += 34.0;
        for (i, (label, v)) in [("Position X", &mut pos.x), ("Position Y", &mut pos.y), ("Position Z", &mut pos.z)].into_iter().enumerate() {
            text(label, lx, y + 17.0, 16, C_TEXT);
            if self.ui.num_field(&format!("comp_p{}", i), Rect::new(fx, y, fw, 24.0), v, 1.0) {
                changed_pose = true;
            }
            y += 30.0;
        }
        for (i, label) in ["Rotation X (deg)", "Rotation Y (deg)", "Rotation Z (deg)"].iter().enumerate() {
            text(label, lx, y + 17.0, 16, C_TEXT);
            if self.ui.num_field(&format!("comp_r{}", i), Rect::new(fx, y, fw - 56.0, 24.0), &mut e[i], 15.0) {
                changed_pose = true;
            }
            if self.ui.button(&format!("comp_r90_{}", i), Rect::new(fx + fw - 52.0, y, 52.0, 24.0), "+90") {
                e[i] += 90.0;
                changed_pose = true;
            }
            y += 30.0;
        }
        if self.ui.checkbox("comp_fixed", fx, y + 2.0, "Fixed (anchored)", &mut fixed) {
            changed_other = true;
        }
        y += 30.0;
        text("Colour", lx, y + 17.0, 16, C_TEXT);
        let mut color = c.part.color;
        for (i, p) in PALETTE.iter().enumerate() {
            let sr = Rect::new(fx + i as f32 * 21.0, y + 2.0, 18.0, 20.0);
            self.ui.reg(&format!("comp_col{}", i), sr);
            draw_rectangle(sr.x, sr.y, sr.w, sr.h, Color::new(p[0], p[1], p[2], 1.0));
            let cur = (p[0] - color[0]).abs() < 0.01 && (p[1] - color[1]).abs() < 0.01 && (p[2] - color[2]).abs() < 0.01;
            draw_rectangle_lines(sr.x, sr.y, sr.w, sr.h, if cur { 3.0 } else { 1.0 }, if cur { C_SEL } else { C_BORDER });
            if self.ui.clicked(sr) {
                color = *p;
                changed_other = true;
            }
        }
        y += 34.0;
        if self.ui.button("comp_edit", Rect::new(lx, y, 140.0, 28.0), "Edit Part") {
            self.edit_component(id);
            return;
        }
        if self.ui.button("comp_dup", Rect::new(lx + 146.0, y, 140.0, 28.0), "Duplicate") {
            self.duplicate_component(id);
            return;
        }
        let _ = y;
        if self.ui.button("comp_close", Rect::new(r.x + r.w - 96.0, r.y + r.h - 36.0, 86.0, 28.0), "Close") {
            self.dialog = Dialog::None;
            return;
        }
        if changed_pose || changed_other {
            self.checkpoint();
            let probs = {
                let pr = self.product_mut().unwrap();
                let cm = pr.comp_mut(id).unwrap();
                cm.part.name = name.clone();
                cm.name = format!("{} ({}.{})", name, name, 1);
                cm.fixed = fixed;
                cm.part.color = color;
                if changed_pose {
                    cm.pos = pos;
                    cm.rot = DQuat::from_euler(EulerRot::XYZ, e[0].to_radians(), e[1].to_radians(), e[2].to_radians());
                }
                // numbering of instances with the same part name
                let mut seen: std::collections::HashMap<String, u32> = Default::default();
                for c in pr.comps.iter_mut() {
                    let n = seen.entry(c.part.name.clone()).or_insert(0);
                    *n += 1;
                    c.name = format!("{} ({}.{})", c.part.name, c.part.name, n);
                }
                if changed_pose && !fixed {
                    // explicit placement wins over constraints on this component for now
                    Vec::new()
                } else {
                    pr.solve()
                }
            };
            if !probs.is_empty() {
                let m = probs.join("; ");
                self.err(&m);
            }
        }
    }

    fn constraint_dialog(&mut self, i: usize) {
        let c = match self.product().and_then(|p| p.constraints.get(i)) {
            Some(c) => c.clone(),
            None => {
                self.dialog = Dialog::None;
                return;
            }
        };
        let r = self.dialog_rect(24.0 + 5.0 * 30.0 + 60.0);
        self.ui.panel(r, Some(&format!("{} Constraint", c.kind.name())));
        let lx = r.x + 10.0;
        let fx = r.x + 140.0;
        let fw = r.w - 150.0;
        let mut y = r.y + 34.0;
        let (an, bn) = {
            let pr = self.product().unwrap();
            (pr.comp(c.a).map(|x| x.name.clone()).unwrap_or_default(), pr.comp(c.b).map(|x| x.name.clone()).unwrap_or_default())
        };
        text("Moves", lx, y + 17.0, 16, C_TEXT);
        text(&fit_tail(&an, fw, 16), fx, y + 17.0, 16, C_TITLE);
        y += 30.0;
        text("Against", lx, y + 17.0, 16, C_TEXT);
        text(&fit_tail(&bn, fw, 16), fx, y + 17.0, 16, C_TITLE);
        y += 30.0;
        let mut value = c.value;
        let mut flip = c.flip;
        let mut changed = false;
        match c.kind {
            CKind::Offset => {
                text("Offset (mm)", lx, y + 17.0, 16, C_TEXT);
                changed |= self.ui.num_field("con_val", Rect::new(fx, y, fw, 24.0), &mut value, 1.0);
                y += 30.0;
            }
            CKind::Angle => {
                text("Angle (deg)", lx, y + 17.0, 16, C_TEXT);
                changed |= self.ui.num_field("con_val", Rect::new(fx, y, fw, 24.0), &mut value, 15.0);
                y += 30.0;
            }
            _ => {}
        }
        if c.kind != CKind::Angle {
            changed |= self.ui.checkbox("con_flip", fx, y + 2.0, "Flip orientation", &mut flip);
        }
        if self.ui.button("con_delete", Rect::new(lx, r.y + r.h - 36.0, 90.0, 28.0), "Delete") {
            self.sel = Sel::Constraint(i);
            self.delete_selection();
            return;
        }
        if self.ui.button("con_close", Rect::new(r.x + r.w - 96.0, r.y + r.h - 36.0, 86.0, 28.0), "Close") {
            self.dialog = Dialog::None;
            return;
        }
        if changed {
            self.checkpoint();
            let pr = self.product_mut().unwrap();
            pr.constraints[i].value = value;
            pr.constraints[i].flip = flip;
            let probs = pr.solve();
            if !probs.is_empty() {
                let m = probs.join("; ");
                self.err(&m);
            }
        }
    }

    /// Changes the interface size: bigger, smaller, or back to 100 % (None).
    pub fn change_scale(&mut self, up: Option<bool>) {
        let s = match up {
            Some(u) => step_ui_scale(u),
            None => {
                set_ui_scale(1.0);
                save_ui_scale();
                1.0
            }
        };
        self.ui.focus = None;
        self.say(&format!("Interface size {} % (saved for next time). Ctrl + / Ctrl - change it, Ctrl 0 resets.", (s * 100.0).round()));
    }

    fn draw_options(&mut self) {
        let w = 560.0f32.min(sw() - 20.0);
        let h = 250.0;
        let r = Rect::new((sw() - w) / 2.0, (sh() - h) / 2.0, w, h);
        self.ui.block(Rect::new(0.0, 0.0, sw(), sh()));
        draw_rectangle(0.0, 0.0, sw(), sh(), Color::new(0.0, 0.0, 0.0, 0.35));
        self.ui.panel(r, Some("Options"));
        text("Interface size", r.x + 14.0, r.y + 52.0, 17, C_ACCENT);
        text("Makes menus, toolbars, text and dialogs bigger, e.g. 200 % on a 4K monitor.", r.x + 14.0, r.y + 74.0, 15, C_DIM);
        let bw = ((w - 28.0) / SCALE_STEPS.len() as f32) - 4.0;
        let cur = ui_scale();
        let mut pick = None;
        for (i, s) in SCALE_STEPS.iter().enumerate() {
            let br = Rect::new(r.x + 14.0 + i as f32 * (bw + 4.0), r.y + 90.0, bw, 32.0);
            let label = format!("{}%", (s * 100.0).round());
            if self.ui.button_ex(&format!("opt_scale_{}", i), br, &label, (cur - s).abs() < 1e-3, true) {
                pick = Some(*s);
            }
        }
        let mut custom = (cur * 100.0) as f64;
        text("Custom (%)", r.x + 14.0, r.y + 155.0, 16, C_TEXT);
        if self.ui.num_field("opt_scale_custom", Rect::new(r.x + 120.0, r.y + 138.0, 100.0, 26.0), &mut custom, 5.0) {
            pick = Some((custom as f32 / 100.0).clamp(0.5, 4.0));
        }
        text("Shortcuts: Ctrl + bigger, Ctrl - smaller, Ctrl 0 back to 100 %.", r.x + 14.0, r.y + 190.0, 15, C_DIM);
        if self.ui.button("opt_close", Rect::new(r.x + w - 100.0, r.y + h - 38.0, 88.0, 28.0), "Close") || (self.ui.inp.key(KeyCode::Escape) && self.ui.focus.is_none()) {
            self.dialog = Dialog::None;
        }
        if let Some(s) = pick {
            set_ui_scale(s);
            save_ui_scale();
            self.ui.focus = None;
            self.say(&format!("Interface size {} % (saved for next time).", (s * 100.0).round()));
        }
    }

    fn draw_help(&mut self) {
        let lines = [
            "MOUSE (classic CAD style)",
            "  Middle drag: pan.   Hold middle + hold left (or right) and drag: rotate.",
            "  Hold middle, click left once, keep middle down and move up/down: zoom.",
            "  Middle click: centre the view there.  Wheel: zoom.  Drag the compass: rotate.",
            "  Left click a face to select it; right-click a face or tree item for options.",
            "",
            "MAKING A PART",
            "  Select a plane or a face > Sketch > draw closed outlines > Exit Workbench",
            "  Pad extrudes the sketch, Pocket cuts it, Shaft / Groove revolve it",
            "  around the sketch's Axis (or its green V axis). Outlines inside outlines are holes.",
            "  Sketcher: Profile / Line draw connected lines; ends snap together (red = open).",
            "  Corner, Chamfer, Quick Trim and Mirror edit lines; Construction makes helpers.",
            "  Measure Between / Measure Item: distances, angles, lengths, radii and areas.",
            "  Constraint: dimensions that drive the shape; drag them, double-click to edit.",
            "  Every feature stays editable: double-click it in the tree.",
            "  Dress-ups: Edge Fillet / Chamfer (click edges), Draft, Shell, Thickness (click faces).",
            "  Rib / Slot sweep a profile sketch along a centre curve; Multi-sections blends sketches.",
            "  Plane makes an offset plane to sketch on; Translation, Rotation, Symmetry, Scaling move the body.",
            "",
            "ASSEMBLY",
            "  File > New Assembly. Add parts with New Part / Existing Part / Insert Mesh.",
            "  Move drags parts; Contact, Offset, Coincidence and Angle snap faces",
            "  together (click a face on the part to move first). Fix anchors a part.",
            "",
            "FILES",
            "  Save keeps parts (.bcpart) and assemblies (.bcasm) fully editable.",
            "  Export STL for 3D printers (millimetres), OBJ for game engines.",
            "  Import STL/OBJ brings meshes in (e.g. parts exported from other CAD).",
            "",
            "KEYS  Ctrl+Z undo, Ctrl+Y redo, Ctrl+S save, Del delete, Esc cancel, F fit",
            "      Ctrl + / Ctrl - / Ctrl 0: interface size (also Tools > Options, e.g. for 4K).",
            "Number fields accept maths and units: 20*3, 1.5in, 2cm.",
        ];
        let w = 640.0;
        let h = lines.len() as f32 * 19.0 + 80.0;
        let r = Rect::new((sw() - w) / 2.0, (sh() - h) / 2.0, w, h);
        self.ui.block(Rect::new(0.0, 0.0, sw(), sh()));
        self.ui.panel(r, Some("Controls and Tips"));
        for (i, l) in lines.iter().enumerate() {
            let bold = !l.starts_with(' ') && !l.is_empty();
            text(l, r.x + 14.0, r.y + 46.0 + i as f32 * 19.0, 16, if bold { C_TITLE } else { C_TEXT });
        }
        if self.ui.button("help_close", Rect::new(r.x + w - 100.0, r.y + h - 36.0, 88.0, 28.0), "Close") {
            self.dialog = Dialog::None;
        }
    }
}

// ------------------------------------------------------------ icons

fn ln(x1: f32, y1: f32, x2: f32, y2: f32, t: f32, c: Color) {
    draw_line(x1, y1, x2, y2, t, c);
}

/// Simple drawn icons for the toolbars (centre cx, cy; about 30 px).
pub fn draw_icon(id: &str, cx: f32, cy: f32) {
    let dark = Color::new(0.85, 0.88, 0.93, 1.0);
    let solid = Color::new(0.55, 0.65, 0.82, 1.0);
    let cut = Color::new(0.85, 0.35, 0.25, 1.0);
    let sk = Color::new(0.2, 0.6, 0.25, 1.0);
    let gold = Color::new(0.9, 0.7, 0.2, 1.0);
    // a little isometric box helper
    let iso_box = |x: f32, y: f32, w: f32, h: f32, d: f32, c: Color| {
        let top = [vec2(x, y), vec2(x + w, y), vec2(x + w + d, y - d * 0.6), vec2(x + d, y - d * 0.6)];
        draw_rectangle(x, y, w, h, c);
        draw_triangle(top[0], top[1], top[2], Color::new(c.r + 0.15, c.g + 0.15, c.b + 0.12, 1.0));
        draw_triangle(top[0], top[2], top[3], Color::new(c.r + 0.15, c.g + 0.15, c.b + 0.12, 1.0));
        draw_triangle(vec2(x + w, y), vec2(x + w + d, y - d * 0.6), vec2(x + w + d, y + h - d * 0.6), Color::new(c.r - 0.12, c.g - 0.12, c.b - 0.1, 1.0));
        draw_triangle(vec2(x + w, y), vec2(x + w + d, y + h - d * 0.6), vec2(x + w, y + h), Color::new(c.r - 0.12, c.g - 0.12, c.b - 0.1, 1.0));
        draw_rectangle_lines(x, y, w, h, 1.0, dark);
    };
    match id {
        "tb_sketch" => {
            draw_rectangle_lines(cx - 13.0, cy - 9.0, 22.0, 18.0, 1.5, gold);
            ln(cx - 6.0, cy + 6.0, cx + 10.0, cy - 10.0, 3.0, sk);
            draw_triangle(vec2(cx - 8.0, cy + 8.0), vec2(cx - 6.0, cy + 3.0), vec2(cx - 3.0, cy + 6.0), dark);
        }
        "tb_pad" => {
            iso_box(cx - 12.0, cy - 2.0, 18.0, 10.0, 7.0, solid);
            ln(cx, cy - 4.0, cx, cy - 14.0, 2.5, sk);
            draw_triangle(vec2(cx - 4.0, cy - 11.0), vec2(cx + 4.0, cy - 11.0), vec2(cx, cy - 16.0), sk);
        }
        "tb_pocket" => {
            iso_box(cx - 12.0, cy - 4.0, 20.0, 12.0, 7.0, solid);
            draw_ellipse(cx + 1.5, cy - 7.0, 6.0, 2.5, 0.0, cut);
            ln(cx + 1.0, cy - 14.0, cx + 1.0, cy - 5.0, 2.0, cut);
        }
        "tb_shaft" | "tb_groove" => {
            let c = if id == "tb_shaft" { solid } else { cut };
            draw_rectangle(cx - 8.0, cy - 10.0, 16.0, 20.0, c);
            draw_ellipse(cx, cy - 10.0, 8.0, 3.0, 0.0, Color::new(c.r + 0.15, c.g + 0.15, c.b + 0.1, 1.0));
            draw_ellipse_lines(cx, cy + 10.0, 8.0, 3.0, 0.0, 1.0, dark);
            ln(cx, cy - 15.0, cx, cy + 15.0, 1.0, dark);
            draw_ellipse_lines(cx, cy, 13.0, 4.0, 0.0, 1.5, sk);
        }
        "tb_hole" => {
            iso_box(cx - 12.0, cy - 2.0, 20.0, 10.0, 7.0, solid);
            draw_ellipse(cx + 1.5, cy - 4.0, 5.0, 2.0, 0.0, dark);
            ln(cx + 1.5, cy - 16.0, cx + 1.5, cy - 7.0, 2.0, cut);
        }
        "tb_box" => iso_box(cx - 11.0, cy - 5.0, 16.0, 14.0, 8.0, solid),
        "tb_cyl" => {
            draw_rectangle(cx - 9.0, cy - 8.0, 18.0, 16.0, solid);
            draw_ellipse(cx, cy + 8.0, 9.0, 3.5, 0.0, solid);
            draw_ellipse(cx, cy - 8.0, 9.0, 3.5, 0.0, Color::new(0.7, 0.8, 0.95, 1.0));
            draw_ellipse_lines(cx, cy - 8.0, 9.0, 3.5, 0.0, 1.0, dark);
        }
        "tb_sphere" => {
            draw_circle(cx, cy, 11.0, solid);
            draw_circle(cx - 4.0, cy - 4.0, 4.0, Color::new(0.8, 0.86, 0.97, 1.0));
            draw_circle_lines(cx, cy, 11.0, 1.0, dark);
        }
        "tb_mirror" => {
            ln(cx, cy - 14.0, cx, cy + 14.0, 1.5, gold);
            draw_triangle(vec2(cx - 3.0, cy - 8.0), vec2(cx - 3.0, cy + 8.0), vec2(cx - 13.0, cy + 8.0), solid);
            draw_triangle(vec2(cx + 3.0, cy - 8.0), vec2(cx + 3.0, cy + 8.0), vec2(cx + 13.0, cy + 8.0), Color::new(0.75, 0.8, 0.9, 1.0));
        }
        "tb_rpat" => {
            for i in 0..3 {
                for j in 0..2 {
                    draw_rectangle(cx - 13.0 + i as f32 * 9.0, cy - 8.0 + j as f32 * 9.0, 7.0, 7.0, if i == 0 && j == 0 { solid } else { Color::new(0.75, 0.8, 0.9, 1.0) });
                }
            }
        }
        "tb_cpat" => {
            for i in 0..6 {
                let a = i as f32 * std::f32::consts::TAU / 6.0;
                draw_circle(cx + a.cos() * 10.0, cy + a.sin() * 10.0, 3.5, if i == 0 { solid } else { Color::new(0.75, 0.8, 0.9, 1.0) });
            }
            draw_circle(cx, cy, 2.0, dark);
        }
        "tb_import" | "as_mesh" => {
            for i in 0..3 {
                let x = cx - 10.0 + i as f32 * 7.0;
                draw_triangle(vec2(x, cy + 8.0), vec2(x + 7.0, cy + 8.0), vec2(x + 3.5, cy - 2.0 + (i % 2) as f32 * 4.0), solid);
                draw_triangle_lines(vec2(x, cy + 8.0), vec2(x + 7.0, cy + 8.0), vec2(x + 3.5, cy - 2.0 + (i % 2) as f32 * 4.0), 1.0, dark);
            }
            ln(cx + 10.0, cy - 14.0, cx + 10.0, cy - 4.0, 2.5, sk);
            draw_triangle(vec2(cx + 6.0, cy - 7.0), vec2(cx + 14.0, cy - 7.0), vec2(cx + 10.0, cy - 2.0), sk);
        }
        "tb_measure" | "as_measure" => {
            draw_rectangle(cx - 14.0, cy - 5.0, 28.0, 10.0, gold);
            for i in 0..7 {
                let x = cx - 12.0 + i as f32 * 4.0;
                ln(x, cy - 5.0, x, cy - if i % 2 == 0 { 0.0 } else { 2.0 }, 1.0, dark);
            }
            draw_rectangle_lines(cx - 14.0, cy - 5.0, 28.0, 10.0, 1.0, dark);
        }
        "tb_rib" | "tb_slot" => {
            let c = if id == "tb_rib" { solid } else { cut };
            let mut prev = vec2(cx - 13.0, cy + 8.0);
            for i in 1..=12 {
                let t = i as f32 / 12.0;
                let p = vec2(cx - 13.0 + 26.0 * t, cy + 8.0 - (t * 3.1).sin() * 14.0);
                ln(prev.x, prev.y, p.x, p.y, 5.0, c);
                prev = p;
            }
            draw_circle(cx - 13.0, cy + 8.0, 4.0, c);
            draw_circle_lines(cx - 13.0, cy + 8.0, 4.0, 1.0, dark);
        }
        "tb_loft" | "tb_rloft" => {
            let c = if id == "tb_loft" { solid } else { cut };
            draw_triangle(vec2(cx - 12.0, cy + 10.0), vec2(cx + 12.0, cy + 10.0), vec2(cx + 6.0, cy - 10.0), c);
            draw_triangle(vec2(cx - 12.0, cy + 10.0), vec2(cx + 6.0, cy - 10.0), vec2(cx - 6.0, cy - 10.0), c);
            draw_ellipse_lines(cx, cy + 10.0, 12.0, 3.0, 0.0, 1.5, sk);
            draw_rectangle_lines(cx - 6.0, cy - 12.0, 12.0, 4.0, 1.5, sk);
        }
        "tb_fillet" => {
            draw_rectangle(cx - 12.0, cy - 4.0, 18.0, 16.0, solid);
            draw_rectangle(cx - 12.0, cy - 12.0, 10.0, 8.0, solid);
            draw_circle(cx - 2.0, cy - 4.0, 8.0, solid);
            draw_arc(cx - 2.0, cy - 4.0, 16, 8.0, 270.0, 2.0, 90.0, gold);
            draw_rectangle(cx + 6.0, cy - 12.0, 8.0, 24.0, Color::new(0.24, 0.25, 0.28, 1.0));
        }
        "tb_chamfer" => {
            draw_triangle(vec2(cx - 12.0, cy - 12.0), vec2(cx + 4.0, cy - 12.0), vec2(cx - 12.0, cy + 12.0), solid);
            draw_triangle(vec2(cx + 4.0, cy - 12.0), vec2(cx + 12.0, cy - 4.0), vec2(cx - 12.0, cy + 12.0), solid);
            draw_triangle(vec2(cx + 12.0, cy - 4.0), vec2(cx + 12.0, cy + 12.0), vec2(cx - 12.0, cy + 12.0), solid);
            ln(cx + 4.0, cy - 12.0, cx + 12.0, cy - 4.0, 2.5, gold);
        }
        "tb_draft" => {
            draw_triangle(vec2(cx - 12.0, cy + 12.0), vec2(cx + 12.0, cy + 12.0), vec2(cx + 7.0, cy - 12.0), solid);
            draw_triangle(vec2(cx - 12.0, cy + 12.0), vec2(cx + 7.0, cy - 12.0), vec2(cx - 7.0, cy - 12.0), solid);
            ln(cx + 12.0, cy + 12.0, cx + 12.0, cy - 12.0, 1.0, dark);
            draw_arc(cx + 12.0, cy + 12.0, 12, 14.0, 255.0, 1.5, 15.0, gold);
        }
        "tb_shell" => {
            iso_box(cx - 12.0, cy - 4.0, 18.0, 14.0, 7.0, solid);
            draw_rectangle(cx - 9.0, cy - 2.0, 12.0, 9.0, Color::new(0.2, 0.22, 0.27, 1.0));
        }
        "tb_thick" => {
            iso_box(cx - 12.0, cy - 1.0, 18.0, 11.0, 7.0, solid);
            draw_rectangle(cx - 12.0, cy - 7.0, 18.0, 6.0, gold);
        }
        "tb_translate" => {
            iso_box(cx - 13.0, cy - 2.0, 10.0, 10.0, 5.0, Color::new(0.75, 0.8, 0.9, 1.0));
            ln(cx - 2.0, cy + 2.0, cx + 11.0, cy + 2.0, 2.5, sk);
            draw_triangle(vec2(cx + 14.0, cy + 2.0), vec2(cx + 8.0, cy - 3.0), vec2(cx + 8.0, cy + 7.0), sk);
        }
        "tb_rotate" => {
            iso_box(cx - 7.0, cy - 2.0, 10.0, 10.0, 5.0, solid);
            draw_arc(cx, cy, 24, 13.0, 200.0, 2.0, 140.0, sk);
            draw_triangle(vec2(cx + 12.0, cy - 6.0), vec2(cx + 7.0, cy - 9.0), vec2(cx + 13.0, cy + 1.0), sk);
        }
        "tb_symmetry" => {
            ln(cx, cy - 14.0, cx, cy + 14.0, 1.5, gold);
            draw_triangle(vec2(cx - 3.0, cy - 8.0), vec2(cx - 3.0, cy + 8.0), vec2(cx - 13.0, cy + 8.0), Color::new(0.5, 0.5, 0.55, 0.8));
            draw_triangle(vec2(cx + 3.0, cy - 8.0), vec2(cx + 3.0, cy + 8.0), vec2(cx + 13.0, cy + 8.0), solid);
        }
        "tb_scale" => {
            draw_rectangle_lines(cx - 12.0, cy - 2.0, 10.0, 10.0, 1.5, Color::new(0.75, 0.8, 0.9, 1.0));
            draw_rectangle(cx - 6.0, cy - 12.0, 18.0, 18.0, solid);
            ln(cx - 12.0, cy + 8.0, cx + 12.0, cy - 12.0, 1.5, sk);
        }
        "tb_plane" => {
            let q = [vec2(cx - 13.0, cy + 6.0), vec2(cx + 3.0, cy + 10.0), vec2(cx + 13.0, cy - 2.0), vec2(cx - 3.0, cy - 6.0)];
            draw_triangle(q[0], q[1], q[2], Color::new(0.95, 0.55, 0.35, 0.5));
            draw_triangle(q[0], q[2], q[3], Color::new(0.95, 0.55, 0.35, 0.5));
            for k in 0..4 {
                ln(q[k].x, q[k].y, q[(k + 1) % 4].x, q[(k + 1) % 4].y, 1.5, gold);
            }
        }
        "tb_split" => {
            iso_box(cx - 12.0, cy - 2.0, 18.0, 12.0, 7.0, solid);
            let q = [vec2(cx - 15.0, cy + 2.0), vec2(cx + 9.0, cy + 2.0), vec2(cx + 16.0, cy - 6.0), vec2(cx - 8.0, cy - 6.0)];
            draw_triangle(q[0], q[1], q[2], Color::new(0.95, 0.55, 0.35, 0.55));
            draw_triangle(q[0], q[2], q[3], Color::new(0.95, 0.55, 0.35, 0.55));
            for k in 0..4 {
                ln(q[k].x, q[k].y, q[(k + 1) % 4].x, q[(k + 1) % 4].y, 1.5, gold);
            }
        }
        "tb_stiffener" => {
            draw_rectangle(cx - 13.0, cy + 4.0, 26.0, 8.0, solid);
            draw_rectangle(cx - 13.0, cy - 12.0, 5.0, 16.0, solid);
            draw_rectangle(cx + 8.0, cy - 12.0, 5.0, 16.0, solid);
            draw_rectangle(cx - 8.0, cy - 3.0, 16.0, 4.0, gold);
        }
        "tb_back" => {
            ln(cx - 12.0, cy, cx + 12.0, cy, 3.0, sk);
            draw_triangle(vec2(cx - 14.0, cy), vec2(cx - 6.0, cy - 7.0), vec2(cx - 6.0, cy + 7.0), sk);
        }
        "sk_select" => {
            draw_triangle(vec2(cx - 6.0, cy - 12.0), vec2(cx - 6.0, cy + 8.0), vec2(cx + 7.0, cy + 2.0), WHITE);
            draw_triangle_lines(vec2(cx - 6.0, cy - 12.0), vec2(cx - 6.0, cy + 8.0), vec2(cx + 7.0, cy + 2.0), 1.5, dark);
        }
        "sk_rect" => draw_rectangle_lines(cx - 12.0, cy - 8.0, 24.0, 16.0, 2.5, sk),
        "sk_crect" => {
            draw_rectangle_lines(cx - 12.0, cy - 8.0, 24.0, 16.0, 2.5, sk);
            draw_circle(cx, cy, 2.0, dark);
        }
        "sk_line" => {
            ln(cx - 11.0, cy + 9.0, cx + 11.0, cy - 9.0, 2.5, sk);
            draw_rectangle(cx - 13.0, cy + 7.0, 4.0, 4.0, dark);
            draw_rectangle(cx + 9.0, cy - 11.0, 4.0, 4.0, dark);
        }
        "sk_axis" => {
            for i in 0..4 {
                let t0 = -13.0 + i as f32 * 7.0;
                ln(cx + t0 * 0.3, cy - t0, cx + (t0 + 4.5) * 0.3, cy - t0 - 4.5, 2.0, gold);
            }
            draw_ellipse_lines(cx, cy, 9.0, 3.0, 0.0, 1.0, dark);
        }
        "sk_circle3" => {
            draw_circle_lines(cx, cy, 10.0, 2.5, sk);
            for a in [0.3f32, 2.4, 4.4] {
                draw_circle(cx + a.cos() * 10.0, cy + a.sin() * 10.0, 2.5, dark);
            }
        }
        "sk_arc" => {
            draw_arc(cx - 4.0, cy + 6.0, 24, 13.0, 270.0, 2.5, 120.0, sk);
            draw_circle(cx - 4.0, cy + 6.0, 2.0, dark);
        }
        "sk_arc3" => {
            draw_arc(cx, cy + 6.0, 24, 12.0, 180.0, 2.5, 180.0, sk);
            for p in [vec2(cx - 12.0, cy + 6.0), vec2(cx, cy - 6.0), vec2(cx + 12.0, cy + 6.0)] {
                draw_circle(p.x, p.y, 2.5, dark);
            }
        }
        "sk_ellipse" => draw_ellipse_lines(cx, cy, 13.0, 7.0, -20.0, 2.5, sk),
        "sk_spline" => {
            let mut prev = vec2(cx - 13.0, cy + 6.0);
            for i in 1..=20 {
                let t = i as f32 / 20.0;
                let p = vec2(cx - 13.0 + 26.0 * t, cy + 6.0 - (t * 6.3).sin() * 9.0);
                ln(prev.x, prev.y, p.x, p.y, 2.5, sk);
                prev = p;
            }
        }
        "sk_corner" => {
            ln(cx - 11.0, cy - 11.0, cx - 11.0, cy + 2.0, 2.5, sk);
            ln(cx - 2.0, cy + 11.0, cx + 12.0, cy + 11.0, 2.5, sk);
            draw_arc(cx - 2.0, cy + 2.0, 16, 9.0, 90.0, 2.5, 90.0, gold);
        }
        "sk_chamfer" => {
            ln(cx - 11.0, cy - 11.0, cx - 11.0, cy + 3.0, 2.5, sk);
            ln(cx - 3.0, cy + 11.0, cx + 12.0, cy + 11.0, 2.5, sk);
            ln(cx - 11.0, cy + 3.0, cx - 3.0, cy + 11.0, 2.5, gold);
        }
        "sk_trim" => {
            ln(cx - 13.0, cy, cx - 3.0, cy, 2.5, sk);
            ln(cx + 3.0, cy, cx + 13.0, cy, 2.5, Color::new(0.5, 0.5, 0.5, 0.8));
            ln(cx + 5.0, cy - 11.0, cx + 5.0, cy + 11.0, 2.0, dark);
            draw_circle_lines(cx - 3.0, cy - 9.0, 3.0, 1.5, Color::new(0.9, 0.4, 0.3, 1.0));
            draw_circle_lines(cx - 3.0, cy + 9.0, 3.0, 1.5, Color::new(0.9, 0.4, 0.3, 1.0));
        }
        "sk_mirror" => {
            ln(cx, cy - 13.0, cx, cy + 13.0, 1.5, gold);
            ln(cx - 12.0, cy + 8.0, cx - 4.0, cy - 8.0, 2.5, sk);
            ln(cx + 12.0, cy + 8.0, cx + 4.0, cy - 8.0, 2.0, Color::new(0.5, 0.75, 0.5, 0.8));
        }
        "sk_constraint" => {
            let g = Color::new(0.35, 0.95, 0.5, 1.0);
            ln(cx - 12.0, cy - 12.0, cx - 12.0, cy + 6.0, 1.5, g);
            ln(cx + 12.0, cy - 12.0, cx + 12.0, cy + 6.0, 1.5, g);
            ln(cx - 12.0, cy - 6.0, cx + 12.0, cy - 6.0, 1.5, g);
            draw_triangle(vec2(cx - 12.0, cy - 6.0), vec2(cx - 7.0, cy - 9.0), vec2(cx - 7.0, cy - 3.0), g);
            draw_triangle(vec2(cx + 12.0, cy - 6.0), vec2(cx + 7.0, cy - 9.0), vec2(cx + 7.0, cy - 3.0), g);
            ln(cx - 14.0, cy + 9.0, cx + 14.0, cy + 9.0, 2.5, dark);
        }
        "sk_measure" => {
            draw_rectangle(cx - 14.0, cy + 2.0, 28.0, 8.0, gold);
            for i in 0..7 {
                let x = cx - 12.0 + i as f32 * 4.0;
                ln(x, cy + 2.0, x, cy + if i % 2 == 0 { 6.0 } else { 4.0 }, 1.0, Color::new(0.1, 0.1, 0.1, 1.0));
            }
            ln(cx - 12.0, cy - 8.0, cx + 12.0, cy - 8.0, 1.5, dark);
            draw_triangle(vec2(cx - 13.0, cy - 8.0), vec2(cx - 8.0, cy - 11.0), vec2(cx - 8.0, cy - 5.0), dark);
            draw_triangle(vec2(cx + 13.0, cy - 8.0), vec2(cx + 8.0, cy - 11.0), vec2(cx + 8.0, cy - 5.0), dark);
            draw_circle(cx - 13.0, cy - 13.0, 2.0, sk);
            draw_circle(cx + 13.0, cy - 13.0, 2.0, sk);
        }
        "sk_measitem" => {
            draw_rectangle(cx - 14.0, cy + 2.0, 28.0, 8.0, gold);
            for i in 0..7 {
                let x = cx - 12.0 + i as f32 * 4.0;
                ln(x, cy + 2.0, x, cy + if i % 2 == 0 { 6.0 } else { 4.0 }, 1.0, Color::new(0.1, 0.1, 0.1, 1.0));
            }
            draw_arc(cx, cy - 2.0, 16, 10.0, 200.0, 2.5, 140.0, sk);
        }
        "sk_constr" => {
            let mut x = cx - 13.0;
            while x < cx + 12.0 {
                ln(x, cy + (x - cx) * 0.4, x + 4.0, cy + (x + 4.0 - cx) * 0.4, 2.5, dark);
                x += 7.0;
            }
        }
        "sk_circle" => draw_circle_lines(cx, cy, 10.0, 2.5, sk),
        "sk_polygon" => {
            for i in 0..6 {
                let a0 = i as f32 * std::f32::consts::TAU / 6.0;
                let a1 = (i + 1) as f32 * std::f32::consts::TAU / 6.0;
                ln(cx + a0.cos() * 11.0, cy + a0.sin() * 11.0, cx + a1.cos() * 11.0, cy + a1.sin() * 11.0, 2.5, sk);
            }
        }
        "sk_slot" => {
            ln(cx - 7.0, cy - 6.0, cx + 7.0, cy - 6.0, 2.5, sk);
            ln(cx - 7.0, cy + 6.0, cx + 7.0, cy + 6.0, 2.5, sk);
            draw_arc(cx - 7.0, cy, 24, 6.0, 90.0, 2.5, 180.0, sk);
            draw_arc(cx + 7.0, cy, 24, 6.0, -90.0, 2.5, 180.0, sk);
        }
        "sk_profile" => {
            let p = [vec2(cx - 12.0, cy + 9.0), vec2(cx - 8.0, cy - 9.0), vec2(cx + 2.0, cy - 2.0), vec2(cx + 12.0, cy - 10.0), vec2(cx + 10.0, cy + 9.0)];
            for i in 0..p.len() {
                let a = p[i];
                let b = p[(i + 1) % p.len()];
                ln(a.x, a.y, b.x, b.y, 2.5, sk);
            }
        }
        "sk_snap" => {
            for i in 0..4 {
                for j in 0..4 {
                    draw_circle(cx - 10.0 + i as f32 * 6.7, cy - 10.0 + j as f32 * 6.7, 1.5, dark);
                }
            }
        }
        "sk_exit" => {
            draw_rectangle_lines(cx - 12.0, cy - 9.0, 18.0, 18.0, 2.0, gold);
            ln(cx - 4.0, cy, cx + 13.0, cy, 2.5, sk);
            draw_triangle(vec2(cx + 15.0, cy), vec2(cx + 8.0, cy - 6.0), vec2(cx + 8.0, cy + 6.0), sk);
        }
        "as_newpart" => {
            iso_box(cx - 12.0, cy - 3.0, 16.0, 12.0, 7.0, solid);
            ln(cx + 8.0, cy - 14.0, cx + 8.0, cy - 4.0, 2.5, sk);
            ln(cx + 3.0, cy - 9.0, cx + 13.0, cy - 9.0, 2.5, sk);
        }
        "as_insert" => {
            draw_rectangle(cx - 10.0, cy - 12.0, 16.0, 22.0, WHITE);
            draw_rectangle_lines(cx - 10.0, cy - 12.0, 16.0, 22.0, 1.0, dark);
            iso_box(cx - 6.0, cy - 2.0, 8.0, 7.0, 4.0, solid);
            ln(cx + 12.0, cy - 6.0, cx + 12.0, cy + 6.0, 2.5, sk);
        }
        "as_edit" => {
            iso_box(cx - 12.0, cy - 2.0, 14.0, 12.0, 6.0, solid);
            ln(cx - 2.0, cy + 6.0, cx + 12.0, cy - 10.0, 3.0, gold);
        }
        "as_dup" => {
            iso_box(cx - 13.0, cy - 6.0, 12.0, 10.0, 5.0, Color::new(0.75, 0.8, 0.9, 1.0));
            iso_box(cx - 4.0, cy + 1.0, 12.0, 10.0, 5.0, solid);
        }
        "as_move" => {
            let red = Color::new(0.85, 0.15, 0.15, 1.0);
            let grn = Color::new(0.15, 0.6, 0.15, 1.0);
            let blu = Color::new(0.15, 0.3, 0.9, 1.0);
            ln(cx, cy, cx + 13.0, cy + 6.0, 2.5, red);
            ln(cx, cy, cx - 13.0, cy + 6.0, 2.5, grn);
            ln(cx, cy, cx, cy - 14.0, 2.5, blu);
            draw_ellipse_lines(cx, cy, 9.0, 4.0, 0.0, 1.0, dark);
        }
        "as_contact" => {
            draw_rectangle(cx - 12.0, cy + 1.0, 24.0, 9.0, solid);
            draw_rectangle(cx - 8.0, cy - 10.0, 16.0, 10.0, Color::new(0.86, 0.72, 0.5, 1.0));
            ln(cx - 14.0, cy + 1.0, cx + 14.0, cy + 1.0, 2.0, sk);
        }
        "as_offset" => {
            draw_rectangle(cx - 12.0, cy + 4.0, 24.0, 8.0, solid);
            draw_rectangle(cx - 8.0, cy - 13.0, 16.0, 8.0, Color::new(0.86, 0.72, 0.5, 1.0));
            ln(cx + 10.0, cy - 5.0, cx + 10.0, cy + 4.0, 1.5, sk);
            ln(cx + 7.0, cy - 5.0, cx + 13.0, cy - 5.0, 1.5, sk);
            ln(cx + 7.0, cy + 4.0, cx + 13.0, cy + 4.0, 1.5, sk);
        }
        "as_coinc" => {
            draw_rectangle(cx - 12.0, cy - 4.0, 24.0, 12.0, solid);
            draw_rectangle(cx - 3.0, cy - 14.0, 6.0, 22.0, Color::new(0.86, 0.72, 0.5, 1.0));
            ln(cx, cy - 16.0, cx, cy + 14.0, 1.0, sk);
        }
        "as_angle" => {
            ln(cx - 12.0, cy + 9.0, cx + 13.0, cy + 9.0, 2.5, solid);
            ln(cx - 12.0, cy + 9.0, cx + 6.0, cy - 11.0, 2.5, Color::new(0.86, 0.6, 0.3, 1.0));
            draw_arc(cx - 12.0, cy + 9.0, 16, 12.0, -48.0, 1.5, 48.0, sk);
        }
        "as_fix" => {
            let c = Color::new(0.15, 0.25, 0.6, 1.0);
            draw_circle_lines(cx, cy - 10.0, 4.0, 2.0, c);
            ln(cx, cy - 6.0, cx, cy + 10.0, 2.5, c);
            ln(cx - 9.0, cy + 3.0, cx, cy + 11.0, 2.5, c);
            ln(cx + 9.0, cy + 3.0, cx, cy + 11.0, 2.5, c);
        }
        "as_update" => {
            draw_arc(cx, cy, 24, 10.0, 30.0, 3.0, 290.0, sk);
            draw_triangle(vec2(cx + 9.0, cy - 10.0), vec2(cx + 13.0, cy - 2.0), vec2(cx + 4.0, cy - 3.0), sk);
        }
        _ => draw_circle_lines(cx, cy, 8.0, 1.5, dark),
    }
}

/// Icons of the view toolbar: a small cube with the viewed face highlighted, and friends.
fn draw_view_icon(id: &str, cx: f32, cy: f32) {
    let edge = Color::new(0.85, 0.88, 0.93, 1.0);
    let face = Color::new(0.45, 0.55, 0.72, 1.0);
    let hi = Color::new(1.0, 0.7, 0.3, 1.0);
    // isometric cube corners
    let c = |x: f32, y: f32| vec2(cx + x, cy + y);
    let top = [c(0.0, -11.0), c(10.0, -6.0), c(0.0, -1.0), c(-10.0, -6.0)];
    let left = [c(-10.0, -6.0), c(0.0, -1.0), c(0.0, 11.0), c(-10.0, 6.0)];
    let right = [c(0.0, -1.0), c(10.0, -6.0), c(10.0, 6.0), c(0.0, 11.0)];
    let quad = |q: &[Vec2; 4], col: Color| {
        draw_triangle(q[0], q[1], q[2], col);
        draw_triangle(q[0], q[2], q[3], col);
        for k in 0..4 {
            draw_line(q[k].x, q[k].y, q[(k + 1) % 4].x, q[(k + 1) % 4].y, 1.0, edge);
        }
    };
    let cube = |hl: Option<usize>| {
        let cols = [face, Color::new(0.38, 0.46, 0.6, 1.0), Color::new(0.32, 0.4, 0.52, 1.0)];
        for (k, q) in [top, left, right].iter().enumerate() {
            quad(q, if hl == Some(k) { hi } else { cols[k] });
        }
    };
    match id {
        "v_fit" => {
            draw_rectangle_lines(cx - 8.0, cy - 8.0, 16.0, 16.0, 1.5, face);
            for (dx, dy) in [(-1.0f32, -1.0f32), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let p = vec2(cx + dx * 13.0, cy + dy * 13.0);
                ln(p.x, p.y, p.x - dx * 5.0, p.y, 2.0, edge);
                ln(p.x, p.y, p.x, p.y - dy * 5.0, 2.0, edge);
            }
        }
        "v_iso" => cube(None),
        "v_top" => cube(Some(0)),
        "v_front" => cube(Some(1)),
        "v_right" => cube(Some(2)),
        "v_back" | "v_left" | "v_bottom" => {
            cube(None);
            let p = match id {
                "v_back" => c(-5.0, 2.0),
                "v_left" => c(5.0, 2.0),
                _ => c(0.0, -6.0),
            };
            draw_circle_lines(p.x, p.y, 3.5, 1.5, hi);
        }
        "v_normal" => {
            quad(&[c(-11.0, 2.0), c(4.0, -3.0), c(11.0, 4.0), c(-4.0, 9.0)], face);
            ln(cx, cy - 13.0, cx, cy + 1.0, 2.0, hi);
            draw_triangle(c(-3.5, -2.0), c(3.5, -2.0), c(0.0, 3.0), hi);
        }
        "v_proj" => {
            quad(&[c(-6.0, -9.0), c(6.0, -9.0), c(11.0, 9.0), c(-11.0, 9.0)], face);
        }
        _ => {
            cube(None);
            for q in [top, left, right] {
                for k in 0..4 {
                    draw_line(q[k].x, q[k].y, q[(k + 1) % 4].x, q[(k + 1) % 4].y, 2.0, Color::new(0.05, 0.05, 0.08, 1.0));
                }
            }
        }
    }
}

fn draw_tree_icon(icon: &str, cx: f32, cy: f32) {
    let dark = Color::new(0.85, 0.88, 0.93, 1.0);
    match icon {
        "plane" => {
            draw_rectangle(cx - 6.0, cy - 5.0, 12.0, 10.0, Color::new(0.95, 0.8, 0.35, 1.0));
            draw_rectangle_lines(cx - 6.0, cy - 5.0, 12.0, 10.0, 1.0, dark);
        }
        "body" => {
            draw_rectangle(cx - 6.0, cy - 5.0, 12.0, 11.0, Color::new(0.55, 0.65, 0.82, 1.0));
            draw_rectangle_lines(cx - 6.0, cy - 5.0, 12.0, 11.0, 1.0, dark);
        }
        "part" => {
            draw_rectangle(cx - 6.0, cy - 6.0, 12.0, 12.0, Color::new(0.85, 0.75, 0.45, 1.0));
            draw_rectangle(cx - 3.0, cy - 3.0, 6.0, 6.0, Color::new(0.55, 0.65, 0.82, 1.0));
        }
        "product" => {
            draw_rectangle(cx - 7.0, cy - 6.0, 7.0, 6.0, Color::new(0.85, 0.75, 0.45, 1.0));
            draw_rectangle(cx, cy - 2.0, 7.0, 6.0, Color::new(0.55, 0.65, 0.82, 1.0));
            draw_rectangle(cx - 5.0, cy + 1.0, 6.0, 5.0, Color::new(0.6, 0.8, 0.58, 1.0));
        }
        "sketch" => {
            draw_rectangle_lines(cx - 6.0, cy - 5.0, 12.0, 10.0, 1.0, Color::new(0.85, 0.65, 0.2, 1.0));
            draw_line(cx - 4.0, cy + 3.0, cx + 5.0, cy - 4.0, 2.0, Color::new(0.2, 0.6, 0.25, 1.0));
        }
        "constraints" | "constraint" => {
            draw_circle_lines(cx - 2.0, cy, 4.0, 1.5, Color::new(0.1, 0.55, 0.15, 1.0));
            draw_circle_lines(cx + 3.0, cy, 4.0, 1.5, Color::new(0.1, 0.55, 0.15, 1.0));
        }
        other => {
            // reuse toolbar art, scaled down by drawing a simple marker
            let c = match other {
                "pocket" | "groove" | "hole" => Color::new(0.85, 0.35, 0.25, 1.0),
                "import" => Color::new(0.6, 0.5, 0.8, 1.0),
                _ => Color::new(0.55, 0.65, 0.82, 1.0),
            };
            match other {
                "cyl" | "shaft" | "groove" => {
                    draw_rectangle(cx - 5.0, cy - 5.0, 10.0, 10.0, c);
                    draw_ellipse(cx, cy - 5.0, 5.0, 2.0, 0.0, c);
                }
                "sphere" => draw_circle(cx, cy, 6.0, c),
                "hole" => {
                    draw_rectangle(cx - 6.0, cy - 4.0, 12.0, 9.0, Color::new(0.55, 0.65, 0.82, 1.0));
                    draw_circle(cx, cy, 3.0, dark);
                }
                "rpat" | "cpat" | "mirror" => {
                    draw_rectangle(cx - 6.0, cy - 5.0, 5.0, 5.0, c);
                    draw_rectangle(cx + 1.0, cy - 5.0, 5.0, 5.0, Color::new(0.8, 0.84, 0.92, 1.0));
                    draw_rectangle(cx - 6.0, cy + 1.0, 5.0, 5.0, Color::new(0.8, 0.84, 0.92, 1.0));
                    draw_rectangle(cx + 1.0, cy + 1.0, 5.0, 5.0, Color::new(0.8, 0.84, 0.92, 1.0));
                }
                _ => {
                    draw_rectangle(cx - 6.0, cy - 4.0, 12.0, 10.0, c);
                    draw_rectangle_lines(cx - 6.0, cy - 4.0, 12.0, 10.0, 1.0, dark);
                    if other == "pad" || other == "box" {
                        draw_line(cx, cy - 2.0, cx, cy - 8.0, 2.0, Color::new(0.2, 0.6, 0.25, 1.0));
                    }
                    if other == "pocket" {
                        draw_rectangle(cx - 2.0, cy - 4.0, 4.0, 5.0, WHITE);
                    }
                }
            }
        }
    }
}
