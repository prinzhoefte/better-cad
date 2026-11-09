mod app;
mod files;
mod geom;
mod model;
mod panels;
mod dims;
mod dressup;
mod measure;
mod sketch;
mod sketcher;
mod ui;
mod view;

use app::*;
use files::Purpose;
use glam::{DVec2, DVec3};
use macroquad::prelude::*;
use model::*;
use ui::{sh, sw, Input};

fn window_conf() -> macroquad::conf::Conf {
    macroquad::conf::Conf {
        miniquad_conf: miniquad::conf::Conf {
            window_title: "PartForge - part and assembly design".to_owned(),
            window_width: 1440,
            window_height: 900,
            high_dpi: false,
            sample_count: 4,
            window_resizable: true,
            ..Default::default()
        },
        draw_call_vertex_capacity: 60000,
        draw_call_index_capacity: 60000,
        ..Default::default()
    }
}

use macroquad::miniquad;

fn arg_value(name: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1).cloned())
}

// ---------------------------------------------------------------- scripted self test

static mut ROT_BEFORE: Option<Quat> = None;
static mut VOL_BEFORE: f64 = 0.0;

#[derive(Clone)]
enum Act {
    Wait(u32),
    ClickId(&'static str),
    DoubleClickId(&'static str),
    ClickSketch(f64, f64),
    ClickWorld(fn(&mut App) -> DVec3),
    /// drag from a screen point (computed from the app) by an offset in pixels
    Drag(fn(&mut App) -> Option<(Vec2, Vec2)>, usize),
    Call(fn(&mut App)),
    /// type text into the focused field and press Enter
    Type(&'static str),
    Check(&'static str, fn(&mut App) -> (bool, String)),
}

struct SelfTest {
    path: String,
    acts: Vec<Act>,
    i: usize,
    sub: u32,
    mouse: Vec2,
    results: Vec<(String, bool, String)>,
    done: bool,
    drag: Option<(Vec2, Vec2)>,
    dbl: bool,
}

impl SelfTest {
    fn new(path: String) -> SelfTest {
        use Act::*;
        let acts = vec![
            Wait(20),
            // --- part design: sketch a rectangle on the xy plane and pad it
            ClickId("tree_plane_0"),
            ClickId("tb_sketch"),
            Wait(40),
            ClickId("sk_rect"),
            ClickSketch(-20.0, -15.0),
            ClickSketch(20.0, 15.0),
            // type exact sizes into the shape's fields, like a user would
            ClickId("shape_f0"),
            Type("0"),
            ClickId("shape_f1"),
            Type("0"),
            ClickId("shape_f2"),
            Type("40"),
            ClickId("shape_f3"),
            Type("30"),
            Wait(2),
            ClickId("sk_exit"),
            Wait(40),
            ClickId("tb_pad"),
            Wait(3),
            ClickId("dlg_ok"),
            Wait(5),
            Check("act", |a| {
                let v = a.part_mut().map(|p| p.mesh().volume()).unwrap_or(0.0);
                let ok = (v - 24000.0).abs() < 1.0;
                (ok, format!("sketched a 40 x 30 mm rectangle and padded it 20 mm: volume {} mm3 (expected 24000)", fmt(v.round())))
            }),
            Call(|a| a.fit_all()),
            Wait(40),
            // --- sketch on the top face and pocket a circle
            // select the top face first, then Sketch (classic CAD order)
            ClickWorld(|_| DVec3::new(5.0, 5.0, 20.0)),
            Check("select face", |a| (matches!(a.sel, Sel::Face { n, .. } if n.z > 0.99), format!("clicked the top face: selection {:?}", a.sel))),
            ClickId("tb_sketch"),
            Wait(40),
            ClickId("sk_circle"),
            ClickSketch(0.0, 0.0),
            ClickSketch(8.0, 0.0),
            ClickId("sk_exit"),
            Wait(40),
            ClickId("tb_pocket"),
            Wait(3),
            ClickId("dlg_ok"),
            Wait(5),
            Check("pocket", |a| {
                let v = a.part_mut().map(|p| p.mesh().volume()).unwrap_or(0.0);
                let expect = 24000.0 - std::f64::consts::PI * 64.0 * 10.0;
                let face_sketch = a.part().map_or(false, |p| p.features.iter().any(|f| matches!(&f.kind, FeatureKind::Sketch(s) if s.support == "face")));
                ((v - expect).abs() < 25.0 && face_sketch, format!("sketched a D16 circle on the top face and pocketed it 10 mm: volume {} mm3 (expected about {})", fmt(v.round()), fmt(expect.round())))
            }),
            // a triangle drawn from single connected lines on the side face, padded outward
            Call(|a| a.fit_all()),
            Wait(40),
            ClickWorld(|_| DVec3::new(20.0, 0.0, 10.0)),
            ClickId("tb_sketch"),
            Wait(40),
            ClickId("sk_line"),
            ClickSketch(-10.0, 5.0),
            ClickSketch(10.0, 5.0),
            ClickId("sk_line"),
            ClickSketch(10.0, 5.0),
            ClickSketch(0.0, 15.0),
            ClickId("sk_profile"),
            ClickSketch(0.0, 15.0),
            ClickSketch(-10.0, 5.0),
            Act::Type(""),
            Check("lines", |a| match a.sketch_data().map(|s| s.regions()) {
                Some(Ok((solids, _))) => {
                    let area = solids.first().map(|p| geom::signed_area(p).abs()).unwrap_or(0.0);
                    ((area - 100.0).abs() < 1e-6, format!("drew a triangle from 3 separate lines that snap together into one closed profile (area {} mm2)", fmt(area)))
                }
                Some(Err(e)) => (false, format!("profile not closed: {}", e)),
                None => (false, "not in the sketcher".into()),
            }),
            // measure the triangle's base between its two corner points, like a user would
            ClickId("sk_measure"),
            ClickSketch(-10.0, 5.0),
            ClickSketch(10.0, 5.0),
            Check("measure", |a| {
                let m = a.sketch.as_ref().and_then(|s| s.measures.last().cloned());
                match m {
                    Some(m) => {
                        let d = m.rows.iter().find(|r| r.0 == "Minimum distance").map(|r| r.1.clone()).unwrap_or_default();
                        (d == "20 mm", format!("Measure Between two sketch points: {}", m.rows.iter().map(|r| format!("{} = {}", r.0, r.1)).collect::<Vec<_>>().join(", ")))
                    }
                    None => (false, "no measurement was made".into()),
                }
            }),
            ClickId("sk_measitem"),
            ClickSketch(7.5, 7.5),
            Check("measure item", |a| {
                let m = a.sketch.as_ref().and_then(|s| s.measures.last().cloned());
                match m {
                    Some(m) => {
                        let l = m.rows.iter().find(|r| r.0 == "Length").map(|r| r.1.clone()).unwrap_or_default();
                        (l == "14.142 mm", format!("Measure Item on a sloped line: {}", m.rows.iter().take(3).map(|r| format!("{} = {}", r.0, r.1)).collect::<Vec<_>>().join(", ")))
                    }
                    None => (false, "no measurement was made".into()),
                }
            }),
            // dimension the triangle's base like classic CAD: Constraint tool, click the line,
            // click empty space to place it; then double-click the dimension and type a value
            ClickId("sk_constraint"),
            ClickSketch(4.0, 5.0),
            ClickSketch(5.0, -3.0),
            Check("constraint", |a| match a.sketch_data().and_then(|s| s.dims.iter().find(|d| d.kind == dims::DimKind::Length).cloned()) {
                Some(d) => ((d.value - 20.0).abs() < 1e-9, format!("Constraint tool put a {} dimension of {} on the base line (which also got an automatic H constraint)", d.kind.name(), d.label())),
                None => (false, "no dimension was created".into()),
            }),
            Wait(2),
            DoubleClickId("dim_sel"),
            Wait(2),
            Type("30"),
            ClickId("dim_ok"),
            Check("drive dimension", |a| {
                let s = match a.sketch_data() {
                    Some(s) => s.clone(),
                    None => return (false, "not in the sketcher".into()),
                };
                let len = s.dims.iter().find(|d| d.kind == dims::DimKind::Length).and_then(|d| s.dim_measure(d)).unwrap_or(0.0);
                let area = s.regions().ok().and_then(|(r, _)| r.first().map(|p| geom::signed_area(p).abs())).unwrap_or(0.0);
                ((len - 30.0).abs() < 1e-6 && (area - 150.0).abs() < 1e-6, format!("double-clicked the dimension and typed 30: the base line is now {} mm and the triangle stays closed (area {} mm2)", fmt(len), fmt(area)))
            }),
            Drag(
                |a| {
                    let r = a.ui.prev_ids.get("dim_sel")?;
                    Some((r.center(), vec2(40.0, 35.0)))
                },
                10,
            ),
            Check("drag dimension", |a| {
                let s = match a.sketch_data() {
                    Some(s) => s.clone(),
                    None => return (false, "not in the sketcher".into()),
                };
                let d = match s.dims.iter().find(|d| d.kind == dims::DimKind::Length) {
                    Some(d) => d,
                    None => return (false, "the dimension is gone".into()),
                };
                let ok = s.dim_ok(d) && (d.value - 30.0).abs() < 1e-9 && d.offset.length() > 3.0;
                (ok, format!("dragged the dimension text to a new place (offset {} mm); its value stayed {}", fmt((d.offset.length() * 10.0).round() / 10.0), d.label()))
            }),
            ClickId("sk_select"),
            // a line that starts anywhere on a circle (here at 30 degrees) snaps exactly onto it
            ClickId("sk_circle"),
            ClickSketch(-30.0, 25.0),
            ClickSketch(-24.0, 25.0),
            ClickId("sk_line"),
            ClickSketch(-24.35, 28.2),
            ClickSketch(-15.0, 40.0),
            Check("snap on circle", |a| {
                let s = a.sketch_data().and_then(|d| d.shapes.last().cloned());
                match s {
                    Some(Shape::Line { x1, y1, .. }) => {
                        let r = DVec2::new(x1 + 30.0, y1 - 25.0).length();
                        let ang = (y1 - 25.0).atan2(x1 + 30.0).to_degrees();
                        ((r - 6.0).abs() < 1e-9, format!("line start snapped onto the R6 circle at {} deg (distance from centre {} mm)", fmt((ang * 10.0).round() / 10.0), fmt(r)))
                    }
                    _ => (false, "no line was drawn".into()),
                }
            }),
            Call(|a| {
                a.do_undo();
                a.do_undo();
            }),
            ClickId("sk_exit"),
            Wait(40),
            ClickId("tb_pad"),
            Wait(3),
            ClickId("dlg_ok"),
            Wait(5),
            Check("pad lines", |a| {
                let v = a.part_mut().map(|p| p.mesh().volume()).unwrap_or(0.0);
                let before = 24000.0 - std::f64::consts::PI * 64.0 * 10.0;
                ((v - before - 3000.0).abs() < 40.0, format!("padded the dimensioned line triangle 20 mm: volume {} mm3 (expected about {})", fmt(v.round()), fmt((before + 3000.0).round())))
            }),
            // round the top front edge (along X at y = 15, z = 20) with an Edge Fillet, R2
            Call(|a| {
                a.sel = Sel::None;
                unsafe { VOL_BEFORE = a.part_mut().map(|p| p.mesh().volume()).unwrap_or(0.0) };
            }),
            ClickId("tb_fillet"),
            Wait(3),
            ClickWorld(|a| a.part_model().transform_point3(DVec3::new(-5.0, 15.0, 20.0))),
            Wait(3),
            ClickId("dlg_ok"),
            Wait(5),
            Check("edge fillet", |a| {
                let v = a.part_mut().map(|p| p.mesh().volume()).unwrap_or(0.0);
                let removed = unsafe { VOL_BEFORE } - v;
                let expect = (4.0 - std::f64::consts::PI) * 40.0;
                let n = a.part().map(|p| p.features.iter().filter(|f| matches!(&f.kind, FeatureKind::Fillet { edges, .. } if !edges.is_empty())).count()).unwrap_or(0);
                // lines drawn where the rounding starts (top face, y = 13) and ends (side face, z = 18)
                let mesh = a.part_mut().map(|p| p.mesh()).unwrap_or_default();
                let cover = |y: f64, z: f64| -> f64 {
                    mesh.edges.iter().filter(|(p, q)| (p.y - y).abs() < 1e-4 && (q.y - y).abs() < 1e-4 && (p.z - z).abs() < 1e-4 && (q.z - z).abs() < 1e-4).map(|(p, q)| (p.x - q.x).abs()).sum()
                };
                let across = mesh.edges.iter().filter(|(p, q)| p.y > 13.05 && q.y > 13.05 && p.z > 18.05 && q.z > 18.05 && p.y < 14.95 && p.z < 19.95 && (p.x - q.x).abs() > 1e-6).count();
                let (top, side) = (cover(13.0, 20.0), cover(15.0, 18.0));
                (n == 1 && (removed - expect).abs() < 1.0 && across == 0 && top > 39.0 && side > 15.0, format!("Edge Fillet R2 on the 40 mm top edge (picked in 3D): removed {} mm3 (expected about {}); lines: start of rounding {} mm, end {} mm, across the rounding {}", fmt((removed * 10.0).round() / 10.0), fmt((expect * 10.0).round() / 10.0), fmt(top.round()), fmt(side.round()), across))
            }),
            // look closely at the rounded edge
            Call(|a| {
                let t = a.part_model().transform_point3(DVec3::new(10.0, 15.0, 20.0)).as_vec3();
                let rot = view::quat_from_back_up(vec3(0.6, 1.0, 0.7), Vec3::Z);
                a.view.animate_to(rot, t, 70.0);
            }),
            Wait(40),
            Wait(2),
            Call(|a| a.fit_all()),
            Wait(30),
            Check("export", |a| {
                let dir = std::path::Path::new("shots");
                let _ = std::fs::create_dir_all(dir);
                let p = dir.join("selftest.stl").to_string_lossy().to_string();
                a.file_chosen(Purpose::ExportStl, p.clone());
                match std::fs::read(&p).map_err(|e| e.to_string()).and_then(|b| geom::read_stl(&b)) {
                    Ok(t) => {
                        let m = geom::Mesh::from_triangles(&t);
                        let ok = !t.is_empty() && (m.volume() - a.part_mut().unwrap().mesh().volume()).abs() < 5.0;
                        (ok, format!("exported {} triangles to STL and read them back (volume {} mm3)", t.len(), fmt(m.volume().round())))
                    }
                    Err(e) => (false, format!("STL export failed: {}", e)),
                }
            }),
            // --- assembly: two new parts, move one with the gizmo, then a contact constraint
            Call(|a| a.new_product_doc()),
            Wait(5),
            ClickId("as_newpart"),
            ClickId("tb_box"),
            ClickId("dlg_ok"),
            ClickId("tb_back"),
            ClickId("as_newpart"),
            ClickId("tb_cyl"),
            ClickId("dlg_ok"),
            ClickId("tb_back"),
            Call(|a| a.fit_all()),
            Wait(40),
            ClickId("tree_comp_2"),
            ClickId("as_move"),
            Wait(3),
            Drag(
                |a| {
                    // grab the X arrow's tip and pull it along the arrow
                    let (pos, c) = {
                        let pr = a.product()?;
                        let c = pr.comp(2)?;
                        let m = c.part.cache.mesh.as_ref()?;
                        (c.pos, c.matrix().transform_point3(m.bbox.center()))
                    };
                    let _ = pos;
                    let len = a.view.px_size(c.as_vec3()) as f64 * 80.0;
                    let s0 = a.view.to_screen(c + DVec3::X * len * 0.8)?;
                    let s1 = a.view.to_screen(c + DVec3::X * len * 6.0)?;
                    Some((s0, s1 - s0))
                },
                20,
            ),
            Wait(3),
            Call(|a| a.measure_result = None),
            Drag(|a| {
                a.view.anim = None;
                unsafe { ROT_BEFORE = Some(a.view.rot) };
                Some((vec2(sw() * 0.45, sh() * 0.45), vec2(60.0, 25.0)))
            }, 10),
            Check("move", |a| {
                let x = a.product().and_then(|p| p.comp(2)).map(|c| c.pos.x).unwrap_or(0.0);
                let turned = unsafe { ROT_BEFORE }.map_or(0.0, |r| r.angle_between(a.view.rot).to_degrees());
                let ok = x > 15.0 && a.tool == Tool::Move && turned > 5.0;
                (ok, format!("dragged the cylinder's X arrow with the Manipulation tool: it moved {} mm along X; then rotated the view {} deg by holding middle + left and dragging", fmt((x * 10.0).round() / 10.0), fmt(turned.round() as f64)))
            }),
            ClickId("as_move"),
            Call(|a| a.set_view(vec3(1.0, 1.0, 0.8), Vec3::Z)),
            Wait(40),
            ClickId("as_contact"),
            ClickWorld(|a| {
                let c = a.product().unwrap().comp(2).unwrap();
                c.matrix().transform_point3(DVec3::new(3.0, 3.0, 30.0))
            }),
            ClickWorld(|a| {
                let c = a.product().unwrap().comp(1).unwrap();
                c.matrix().transform_point3(DVec3::new(-4.0, -4.0, 20.0))
            }),
            Wait(5),
            Check("assemble", |a| {
                let pr = match a.product() {
                    Some(p) => p,
                    None => return (false, "no assembly".into()),
                };
                if pr.constraints.is_empty() {
                    return (false, "no constraint was created".into());
                }
                let c = &pr.constraints[0];
                let e = constraint_error(c, pr.comp(c.a).unwrap(), pr.comp(c.b).unwrap());
                (e < 1e-3, format!("{} between {} and {} applied (error {:.6})", c.name, pr.comp(c.a).unwrap().part.name, pr.comp(c.b).unwrap().part.name, e))
            }),
            Call(|a| {
                a.tool = Tool::None;
                a.fit_all();
            }),
            Wait(30),
            Check("hud", |a| {
                let ids = &a.ui.prev_ids;
                let toolbar = ids.keys().filter(|k| k.starts_with("as_") || k.starts_with("tb_")).count();
                let tree = ids.keys().filter(|k| k.starts_with("tree_")).count();
                let menus = ids.contains_key("menu_file");
                let status = ids.contains_key("statusbar");
                let info = a.info_text();
                let ok = toolbar >= 10 && tree >= 3 && menus && status && !info.is_empty();
                (ok, format!("menu bar, {} toolbar buttons, specification tree ({} rows), view toolbar and status bar ('{}') are drawn", toolbar, tree, info))
            }),
        ];
        SelfTest { path, acts, i: 0, sub: 0, mouse: vec2(700.0, 450.0), results: vec![], done: false, drag: None, dbl: false }
    }

    fn click_input(&mut self, target: Option<Vec2>) -> Input {
        let mut inp = Input { mouse: self.mouse, ..Default::default() };
        let t = match target {
            Some(t) => t,
            None => {
                self.next();
                return inp;
            }
        };
        match self.sub {
            0 => {
                self.mouse = t;
                inp.mouse = t;
            }
            1 => {
                inp.pressed[0] = true;
                inp.down[0] = true;
                inp.double_click = self.dbl;
            }
            _ => {
                inp.released[0] = true;
                self.next();
                return inp;
            }
        }
        self.sub += 1;
        inp
    }

    fn next(&mut self) {
        self.dbl = false;
        self.i += 1;
        self.sub = 0;
        self.drag = None;
    }

    fn input(&mut self, app: &mut App) -> Input {
        if self.i >= self.acts.len() {
            self.done = true;
            return Input { mouse: self.mouse, ..Default::default() };
        }
        let act = self.acts[self.i].clone();
        if self.sub == 0 && std::env::var("PF_DEBUG").is_ok() {
            println!("[{}] status: {} | tool {:?} | sel {:?} | gizmo {:?}", self.i, app.status, app.tool, app.sel, app.gizmo_hover);
        }
        match act {
            Act::Wait(n) => {
                self.sub += 1;
                if self.sub >= n {
                    self.next();
                }
                Input { mouse: self.mouse, ..Default::default() }
            }
            Act::ClickId(id) => {
                let t = app.ui.prev_ids.get(id).or_else(|| app.ui.ids.get(id)).map(|r| r.center());
                if std::env::var("PF_DEBUG").is_ok() {
                    println!("[{}] click {} -> {:?}   status: {}", self.i, id, t, app.status);
                }
                if t.is_none() {
                    println!("self-test: control '{}' not on screen", id);
                    self.results.push((format!("click {}", id), false, format!("the control '{}' was not on screen", id)));
                }
                self.click_input(t)
            }
            Act::DoubleClickId(id) => {
                self.dbl = true;
                let t = app.ui.prev_ids.get(id).or_else(|| app.ui.ids.get(id)).map(|r| r.center());
                if t.is_none() {
                    println!("self-test: control '{}' not on screen", id);
                    self.results.push((format!("double-click {}", id), false, format!("'{}' was not on screen", id)));
                }
                self.click_input(t)
            }
            Act::ClickSketch(u, v) => {
                let t = app.sketch_data().map(|s| s.frame).and_then(|f| {
                    let w = app.part_model().transform_point3(f.to3(DVec2::new(u, v), 0.0));
                    app.view.to_screen(w)
                });
                self.click_input(t)
            }
            Act::ClickWorld(f) => {
                let p = f(app);
                let t = app.view.to_screen(p);
                self.click_input(t)
            }
            Act::Drag(f, frames) => {
                if self.sub == 0 {
                    self.drag = f(app);
                    if self.drag.is_none() {
                        self.results.push(("drag".into(), false, "drag target not visible".into()));
                        self.next();
                        return Input { mouse: self.mouse, ..Default::default() };
                    }
                }
                let (start, off) = self.drag.unwrap();
                // the second drag rotates the view the classic way: hold middle, then left
                let rotate = off == vec2(60.0, 25.0);
                let mut inp = Input { mouse: self.mouse, ..Default::default() };
                let n = frames as u32;
                let set = |inp: &mut Input, f: &dyn Fn(&mut Input, usize)| {
                    if rotate {
                        f(inp, 2);
                    }
                    f(inp, 0);
                };
                if self.sub == 0 {
                    self.mouse = start;
                } else if self.sub == 1 {
                    if rotate {
                        inp.pressed[2] = true;
                        inp.down[2] = true;
                    } else {
                        inp.pressed[0] = true;
                        inp.down[0] = true;
                    }
                } else if self.sub == 2 && rotate {
                    inp.down[2] = true;
                    inp.pressed[0] = true;
                    inp.down[0] = true;
                } else if self.sub <= n + 2 {
                    let k = (self.sub - 2) as f32 / n as f32;
                    self.mouse = start + off * k.max(0.0);
                    set(&mut inp, &|i, b| i.down[b] = true);
                } else {
                    set(&mut inp, &|i, b| i.released[b] = true);
                    self.sub += 1;
                    inp.mouse = self.mouse;
                    self.next();
                    return inp;
                }
                inp.mouse = self.mouse;
                self.sub += 1;
                inp
            }
            Act::Type(t) => {
                self.next();
                Input { mouse: self.mouse, chars: t.chars().collect(), keys: vec![KeyCode::Enter], ..Default::default() }
            }
            Act::Call(f) => {
                f(app);
                self.next();
                Input { mouse: self.mouse, ..Default::default() }
            }
            Act::Check(name, f) => {
                let (ok, detail) = f(app);
                println!("self-test {}: {} - {}", name, if ok { "PASS" } else { "FAIL" }, detail);
                self.results.push((name.to_string(), ok, detail));
                self.next();
                Input { mouse: self.mouse, ..Default::default() }
            }
        }
    }

    fn write(&self) {
        let steps: Vec<serde_json::Value> = self
            .results
            .iter()
            .map(|(s, p, d)| serde_json::json!({"step": s, "passed": p, "detail": d}))
            .collect();
        let v = serde_json::json!({"version": 1, "steps": steps});
        if let Some(parent) = std::path::Path::new(&self.path).parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(&self.path, serde_json::to_string_pretty(&v).unwrap());
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let shot = arg_value("--screenshot");
    let selftest = arg_value("--self-test");
    // interface size: the saved choice for normal use; 100 % for automatic checks unless --scale
    if let Some(s) = arg_value("--scale").and_then(|s| s.trim_end_matches('%').parse::<f32>().ok()) {
        ui::set_ui_scale(if s > 10.0 { s / 100.0 } else { s });
    } else if shot.is_none() && selftest.is_none() {
        ui::load_ui_scale();
    }
    let mut app = App::new();
    // open a document given on the command line (e.g. a double-clicked file)
    let args: Vec<String> = std::env::args().collect();
    if let Some(f) = args.iter().skip(1).find(|a| !a.starts_with("--") && (a.ends_with(".pfpart") || a.ends_with(".pfasm") || a.ends_with(".stl") || a.ends_with(".obj"))) {
        if f.ends_with(".stl") || f.ends_with(".obj") {
            app.file_chosen(Purpose::ImportMesh, f.clone());
        } else {
            app.file_chosen(Purpose::OpenDoc, f.clone());
        }
    }
    let mut driver = selftest.map(SelfTest::new);
    let mut last_click = 0.0;
    let mut last_pos = Vec2::ZERO;
    let mut frames = 0u32;
    loop {
        let (inp, dt) = match driver.as_mut() {
            Some(d) => (d.input(&mut app), 1.0 / 60.0),
            None => (Input::gather(&mut last_click, &mut last_pos), get_frame_time().min(0.1)),
        };
        app.frame(inp, dt);
        frames += 1;
        if let (Some(d), Ok(at)) = (driver.as_ref(), std::env::var("PF_SHOT_AT")) {
            for (k, a) in at.split(',').enumerate() {
                if a.parse::<usize>().ok() == Some(d.i) && d.sub == 1 {
                    get_screen_data().export_png(&format!("shots/at{}.png", k));
                }
            }
        }
        if let Some(d) = driver.as_mut() {
            if d.done {
                d.write();
                if let Some(p) = &shot {
                    get_screen_data().export_png(p);
                }
                break;
            }
        } else if let Some(p) = &shot {
            if frames >= 120 {
                if let Some(parent) = std::path::Path::new(p).parent() {
                    let _ = std::fs::create_dir_all(parent);
                }
                get_screen_data().export_png(p);
                break;
            }
        }
        if app.quit {
            break;
        }
        next_frame().await;
    }
}
