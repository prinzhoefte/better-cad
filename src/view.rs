//! 3D view: orbit camera, shaded rendering with a lit material, realistic rendering with
//! surface finishes (plastic, 3D print, metal, wood), edges, picking rays.

use crate::geom::{Aabb, Mesh};
use crate::model::{Finish, DEFAULT_PART_COLOR};
use glam::{DMat4, DVec3};
use macroquad::prelude::*;
use crate::ui::{sh, sw};
use std::collections::HashMap;
use std::sync::Arc;

pub struct View {
    pub target: Vec3,
    pub rot: Quat,
    pub dist: f32,
    pub persp: bool,
    pub fov: f32,
    pub scene_radius: f32,
    /// animation towards a new orientation
    pub anim: Option<(Quat, Quat, Vec3, Vec3, f32, f32, f32)>,
}

pub fn quat_from_back_up(back: Vec3, up: Vec3) -> Quat {
    let z = back.normalize();
    let mut x = up.cross(z);
    if x.length() < 1e-5 {
        x = z.any_orthonormal_vector();
    }
    let x = x.normalize();
    let y = z.cross(x);
    Quat::from_mat3(&Mat3::from_cols(x, y, z)).normalize()
}

impl View {
    pub fn new() -> View {
        View {
            target: Vec3::ZERO,
            rot: quat_from_back_up(vec3(1.0, 1.0, 0.8), Vec3::Z),
            dist: 220.0,
            persp: true,
            fov: 30f32.to_radians(),
            scene_radius: 100.0,
            anim: None,
        }
    }
    pub fn back(&self) -> Vec3 {
        self.rot * Vec3::Z
    }
    pub fn up(&self) -> Vec3 {
        self.rot * Vec3::Y
    }
    pub fn right(&self) -> Vec3 {
        self.rot * Vec3::X
    }
    pub fn eye(&self) -> Vec3 {
        let d = if self.persp { self.dist } else { self.dist + self.scene_radius * 4.0 + 100.0 };
        self.target + self.back() * d
    }
    /// height of the visible area at the target, in model units
    pub fn view_height(&self) -> f32 {
        2.0 * self.dist * (self.fov / 2.0).tan()
    }
    pub fn camera(&self) -> Camera3D {
        let eye = self.eye();
        let d = (eye - self.target).length();
        Camera3D {
            position: eye,
            target: self.target,
            up: self.up(),
            fovy: if self.persp { self.fov } else { self.view_height() },
            projection: if self.persp { Projection::Perspective } else { Projection::Orthographics },
            z_near: if self.persp { (self.dist * 0.08).max(0.01) } else { 0.5 },
            z_far: d + self.scene_radius * 8.0 + 1000.0,
            ..Default::default()
        }
    }
    pub fn vp(&self) -> Mat4 {
        self.camera().matrix()
    }
    pub fn ray(&self, m: Vec2) -> (DVec3, DVec3) {
        let inv = self.vp().inverse();
        let x = 2.0 * m.x / sw() - 1.0;
        let y = 1.0 - 2.0 * m.y / sh();
        let p0 = inv.project_point3(vec3(x, y, -1.0));
        let p1 = inv.project_point3(vec3(x, y, 1.0));
        let d = (p1 - p0).normalize();
        (p0.as_dvec3(), d.as_dvec3())
    }
    pub fn to_screen(&self, p: DVec3) -> Option<Vec2> {
        let c = self.vp() * p.as_vec3().extend(1.0);
        if c.w <= 1e-6 {
            return None;
        }
        let n = c.truncate() / c.w;
        Some(vec2((n.x + 1.0) * 0.5 * sw(), (1.0 - n.y) * 0.5 * sh()))
    }
    /// model units per screen pixel at a point
    pub fn px_size(&self, at: Vec3) -> f32 {
        let d = if self.persp { (at - self.eye()).dot(-self.back()).max(0.01) } else { self.dist };
        2.0 * d * (self.fov / 2.0).tan() / sh()
    }
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        let s = 0.006;
        self.rot = (self.rot * Quat::from_rotation_y(-dx * s) * Quat::from_rotation_x(-dy * s)).normalize();
        self.anim = None;
    }
    pub fn pan(&mut self, dx: f32, dy: f32) {
        let k = self.px_size(self.target);
        self.target += -self.right() * dx * k + self.up() * dy * k;
        self.anim = None;
    }
    pub fn zoom_at(&mut self, steps: f32, mouse: Vec2) {
        let f = 0.85f32.powf(steps);
        // keep the point under the cursor fixed (on the plane through the target)
        let (o, d) = self.ray(mouse);
        let n = self.back().as_dvec3();
        let t = (self.target.as_dvec3() - o).dot(n) / d.dot(n);
        let p = (o + d * t).as_vec3();
        self.dist = (self.dist * f).clamp(0.05, 1e6);
        self.target = p + (self.target - p) * f;
        self.anim = None;
    }
    /// Zoom towards the screen centre by a vertical mouse movement (up = closer).
    pub fn zoom_center(&mut self, dy: f32) {
        self.dist = (self.dist * (dy * 0.006).exp()).clamp(0.05, 1e6);
        self.anim = None;
    }
    pub fn animate_to(&mut self, rot: Quat, target: Vec3, dist: f32) {
        self.anim = Some((self.rot, rot, self.target, target, self.dist, dist, 0.0));
    }
    pub fn fit(&mut self, b: &Aabb, keep_rot: Option<Quat>) {
        let (c, r) = if b.valid() { (b.center().as_vec3(), (b.size().length() as f32 * 0.5).max(5.0)) } else { (Vec3::ZERO, 60.0) };
        self.scene_radius = r;
        let aspect = sw() / sh();
        let half = (self.fov / 2.0).tan() * aspect.min(1.0);
        let dist = r / half * 1.15;
        self.animate_to(keep_rot.unwrap_or(self.rot), c, dist);
    }
    pub fn update(&mut self, dt: f32) {
        if let Some((r0, r1, t0, t1, d0, d1, mut k)) = self.anim {
            k = (k + dt * 4.0).min(1.0);
            let s = k * k * (3.0 - 2.0 * k);
            self.rot = r0.slerp(r1, s).normalize();
            self.target = t0.lerp(t1, s);
            self.dist = d0 + (d1 - d0) * s;
            self.anim = if k >= 1.0 { None } else { Some((r0, r1, t0, t1, d0, d1, k)) };
        }
    }
}

// ---------------------------------------------------------------- render meshes

pub struct RenderMesh {
    pub chunks: Vec<macroquad::models::Mesh>,
    pub edges: Vec<(Vec3, Vec3)>,
}

pub struct Renderer {
    pub material: Material,
    /// realistic rendering with surface finishes
    pub real: Material,
    pub cache: HashMap<(usize, [u8; 3]), (RenderMesh, u64)>,
    pub frame: u64,
    /// model units per pixel at the view target and parallel projection (for `real`)
    px: f32,
    ortho: bool,
}

const VS: &str = r#"#version 100
attribute vec3 position;
attribute vec2 texcoord;
attribute vec4 color0;
attribute vec4 normal;
varying lowp vec4 color;
varying highp vec3 wn;
uniform mat4 Model;
uniform mat4 Projection;
void main() {
    gl_Position = Projection * Model * vec4(position, 1.0);
    color = color0 / 255.0;
    wn = (Model * vec4(normal.xyz, 0.0)).xyz;
}
"#;

const FS: &str = r#"#version 100
precision highp float;
varying lowp vec4 color;
varying highp vec3 wn;
uniform vec3 LightDir;
uniform vec3 FillDir;
uniform vec4 Tint;
void main() {
    vec3 n = normalize(wn);
    float d1 = max(dot(n, normalize(LightDir)), 0.0);
    float d2 = max(dot(n, normalize(FillDir)), 0.0);
    float l = 0.38 + 0.55 * d1 + 0.22 * d2;
    vec3 base = mix(color.rgb, Tint.rgb, Tint.a);
    vec3 c = base * l + vec3(0.10) * pow(d1, 24.0);
    gl_FragColor = vec4(c, 1.0);
}
"#;

// Realistic rendering: physically based lighting (GGX highlights, Fresnel reflections of a
// photo-studio surrounding, ACES tone mapping) and procedural surface finishes worked out
// from the part's own coordinates, so the texture moves with the part.
const VS_REAL: &str = r#"#version 100
attribute vec3 position;
attribute vec2 texcoord;
attribute vec4 color0;
attribute vec4 normal;
varying lowp vec4 color;
varying highp vec3 wn;
varying highp vec3 wp;
varying highp vec3 lp;
varying highp vec3 wz;
uniform mat4 Model;
uniform mat4 Projection;
void main() {
    vec4 w = Model * vec4(position, 1.0);
    gl_Position = Projection * w;
    color = color0 / 255.0;
    wn = (Model * vec4(normal.xyz, 0.0)).xyz;
    wp = w.xyz;
    lp = position;
    wz = (Model * vec4(0.0, 0.0, 1.0, 0.0)).xyz;
}
"#;

const FS_REAL: &str = r#"#version 100
precision highp float;
varying lowp vec4 color;
varying highp vec3 wn;
varying highp vec3 wp;
varying highp vec3 lp;
varying highp vec3 wz;
uniform vec3 LightDir;
uniform vec3 FillDir;
uniform vec3 Eye;
uniform vec3 ViewBack;
uniform vec4 Tint;
// x: finish (0 plastic, 1 3D print, 2 metal, 3 wood), y: mm per pixel, z: parallel projection
uniform vec4 Params;

float hash(vec3 p) {
    p = fract(p * 0.3183099 + vec3(0.71, 0.113, 0.419));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}
float noise(vec3 x) {
    vec3 i = floor(x);
    vec3 f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash(i), hash(i + vec3(1.0, 0.0, 0.0)), f.x),
                   mix(hash(i + vec3(0.0, 1.0, 0.0)), hash(i + vec3(1.0, 1.0, 0.0)), f.x), f.y),
               mix(mix(hash(i + vec3(0.0, 0.0, 1.0)), hash(i + vec3(1.0, 0.0, 1.0)), f.x),
                   mix(hash(i + vec3(0.0, 1.0, 1.0)), hash(i + vec3(1.0, 1.0, 1.0)), f.x), f.y), f.z);
}
float fbm(vec3 p) {
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 4; i++) {
        v += a * noise(p);
        p = p * 2.03 + vec3(1.7, 9.2, 3.1);
        a *= 0.5;
    }
    return v;
}
// a photo studio around the part (Z up): bright ceiling, grey horizon, dark floor and two
// soft boxes; rougher surfaces see it more blurred
vec3 env(vec3 d, float rough) {
    vec3 c = mix(vec3(0.58, 0.60, 0.64), vec3(0.90, 0.92, 0.96), smoothstep(0.0, 0.9, d.z));
    c = mix(vec3(0.15, 0.15, 0.17), c, smoothstep(-0.15 - rough * 0.3, 0.03 + rough * 0.3, d.z));
    float w = 0.04 + rough * 0.5;
    float k = 1.0 - rough * 0.75;
    c += vec3(2.6) * k * smoothstep(0.96 - w, 0.985, dot(d, normalize(vec3(0.45, -0.55, 0.70))));
    c += vec3(1.4) * k * smoothstep(0.97 - w, 0.99, dot(d, normalize(vec3(-0.70, 0.35, 0.60))));
    return c;
}
void main() {
    vec3 n = normalize(wn);
    vec3 v = Params.z > 0.5 ? normalize(ViewBack) : normalize(Eye - wp);
    vec3 base = mix(color.rgb, Tint.rgb, Tint.a);
    vec3 albedo = pow(base, vec3(2.2));
    float metal = 0.0;
    float rough = 0.32;
    float spec = 0.5;
    float fin = Params.x;
    if (fin > 2.5) {
        // wood: growth rings around a line along the part's Y axis, warped, with fibres
        float warp = fbm(lp * 0.06) * 2.5;
        float r = length(lp.xz + vec2(37.0, 11.0)) * 0.35 + warp;
        float band = pow(fract(r), 4.0);
        float fib = noise(vec3(lp.x * 3.0, lp.y * 0.12, lp.z * 3.0));
        albedo = mix(albedo * 1.15, albedo * 0.42, band * 0.8 + fib * 0.2);
        rough = 0.45 + 0.15 * band;
        spec = 0.7;
    } else if (fin > 1.5) {
        // metal, brushed along the part's X axis
        metal = 1.0;
        float brush = noise(vec3(lp.x * 0.05, lp.y * 9.0, lp.z * 9.0));
        rough = 0.2 + 0.12 * brush;
        albedo *= 0.92 + 0.12 * brush;
    } else if (fin > 0.5) {
        // FDM print: 0.2 mm layers stacked along the part's Z axis, shown once they are far
        // enough apart on screen not to flicker
        float h = 0.2;
        float ph = lp.z / h * 6.2831853;
        float vis = clamp((h / max(Params.y, 1e-4) - 1.5) / 2.5, 0.0, 1.0);
        vec3 up = normalize(wz);
        vec3 t = up - n * dot(up, n);
        n = normalize(n + t * (cos(ph) * 0.45 * vis));
        albedo *= 1.0 - 0.10 * vis * (0.5 - 0.5 * sin(ph));
        albedo *= 0.94 + 0.06 * noise(lp * 1.7);
        rough = 0.62;
        spec = 0.4;
    }
    vec3 f0 = mix(vec3(0.08 * spec), albedo, metal);
    vec3 diffc = albedo * (1.0 - metal);
    float a = rough * rough;
    float nv = max(dot(n, v), 1e-3);
    vec3 col = vec3(0.0);
    for (int i = 0; i < 2; i++) {
        vec3 l = normalize(i == 0 ? LightDir : FillDir);
        float rad = i == 0 ? 2.4 : 0.8;
        vec3 hv = normalize(l + v);
        float nl = max(dot(n, l), 0.0);
        float nh = max(dot(n, hv), 0.0);
        float vh = max(dot(v, hv), 0.0);
        float dd = nh * nh * (a * a - 1.0) + 1.0;
        float d = a * a / (3.14159 * dd * dd);
        float k = (rough + 1.0) * (rough + 1.0) / 8.0;
        float g = (nv / (nv * (1.0 - k) + k)) * (nl / (nl * (1.0 - k) + k));
        vec3 f = f0 + (1.0 - f0) * pow(1.0 - vh, 5.0);
        vec3 sp = f * (d * g / max(4.0 * nv * nl, 1e-3));
        col += ((vec3(1.0) - f) * diffc / 3.14159 + sp) * nl * rad;
    }
    vec3 fa = f0 + (max(vec3(1.0 - rough), f0) - f0) * pow(1.0 - nv, 5.0);
    col += diffc * (vec3(1.0) - fa) * env(n, 1.0) * 0.55;
    col += fa * env(reflect(-v, n), rough);
    col = (col * (2.51 * col + 0.03)) / (col * (2.43 * col + 0.59) + 0.14);
    gl_FragColor = vec4(pow(clamp(col, 0.0, 1.0), vec3(1.0 / 2.2)), 1.0);
}
"#;

/// The colour a part is drawn with for a finish: plain grey wood is shown as natural oak.
pub fn finish_color(color: [f32; 3], finish: Finish) -> [f32; 3] {
    if finish == Finish::Wood && color == DEFAULT_PART_COLOR {
        [0.74, 0.53, 0.32]
    } else {
        color
    }
}

impl Renderer {
    pub fn new() -> Renderer {
        let material = load_material(
            ShaderSource::Glsl { vertex: VS, fragment: FS },
            MaterialParams {
                pipeline_params: PipelineParams {
                    depth_write: true,
                    depth_test: Comparison::LessOrEqual,
                    ..Default::default()
                },
                uniforms: vec![
                    UniformDesc::new("LightDir", UniformType::Float3),
                    UniformDesc::new("FillDir", UniformType::Float3),
                    UniformDesc::new("Tint", UniformType::Float4),
                ],
                ..Default::default()
            },
        )
        .expect("shader");
        let real = load_material(
            ShaderSource::Glsl { vertex: VS_REAL, fragment: FS_REAL },
            MaterialParams {
                pipeline_params: PipelineParams {
                    depth_write: true,
                    depth_test: Comparison::LessOrEqual,
                    ..Default::default()
                },
                uniforms: vec![
                    UniformDesc::new("LightDir", UniformType::Float3),
                    UniformDesc::new("FillDir", UniformType::Float3),
                    UniformDesc::new("Eye", UniformType::Float3),
                    UniformDesc::new("ViewBack", UniformType::Float3),
                    UniformDesc::new("Tint", UniformType::Float4),
                    UniformDesc::new("Params", UniformType::Float4),
                ],
                ..Default::default()
            },
        )
        .expect("realistic shader");
        Renderer { material, real, cache: HashMap::new(), frame: 0, px: 0.1, ortho: false }
    }

    pub fn begin_frame(&mut self) {
        self.frame += 1;
        let f = self.frame;
        self.cache.retain(|_, (_, used)| f - *used < 3);
    }

    pub fn get(&mut self, m: &Arc<Mesh>, color: [f32; 3]) -> &RenderMesh {
        let key = (Arc::as_ptr(m) as usize, [(color[0] * 255.0) as u8, (color[1] * 255.0) as u8, (color[2] * 255.0) as u8]);
        let frame = self.frame;
        let e = self.cache.entry(key).or_insert_with(|| (build_render_mesh(m, color, None), frame));
        e.1 = frame;
        &e.0
    }

    pub fn set_lights(&self, view: &View) {
        let l = (view.back() * 0.8 + view.up() * 0.6 - view.right() * 0.3).normalize();
        let f = (view.back() * 0.5 - view.up() * 0.4 + view.right() * 0.7).normalize();
        self.material.set_uniform("LightDir", l);
        self.material.set_uniform("FillDir", f);
        self.real.set_uniform("LightDir", l);
        self.real.set_uniform("FillDir", f);
        self.real.set_uniform("Eye", view.eye());
        self.real.set_uniform("ViewBack", view.back());
    }

    /// Remembers the view's scale and projection for realistic rendering.
    pub fn set_view_scale(&mut self, view: &View) {
        self.px = view.px_size(view.target);
        self.ortho = !view.persp;
    }

    /// Draws a part realistically with its surface finish (tint as in `draw`).
    pub fn draw_real(&mut self, m: &Arc<Mesh>, color: [f32; 3], finish: Finish, model: &DMat4, tint: Vec4) {
        let color = finish_color(color, finish);
        self.real.set_uniform("Tint", tint);
        self.real.set_uniform("Params", vec4(finish.index() as f32, self.px, if self.ortho { 1.0 } else { 0.0 }, 0.0));
        gl_use_material(&self.real);
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.push_model_matrix(model.as_mat4());
        let rm = self.get(m, color);
        for c in &rm.chunks {
            draw_mesh(c);
        }
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.pop_model_matrix();
        gl_use_default_material();
    }

    /// Draws shaded geometry with a model matrix and tint (alpha = tint strength).
    pub fn draw(&mut self, m: &Arc<Mesh>, color: [f32; 3], model: &DMat4, tint: Vec4) {
        self.material.set_uniform("Tint", tint);
        gl_use_material(&self.material);
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.push_model_matrix(model.as_mat4());
        let rm = self.get(m, color);
        for c in &rm.chunks {
            draw_mesh(c);
        }
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.pop_model_matrix();
        gl_use_default_material();
    }

    pub fn draw_edges(&mut self, m: &Arc<Mesh>, color: [f32; 3], model: &DMat4, view: &View, col: Color) {
        let inv = model.inverse();
        let eye_local = inv.transform_point3(view.eye().as_dvec3()).as_vec3();
        let back_local = inv.transform_vector3(view.back().as_dvec3()).as_vec3().normalize();
        let persp = view.persp;
        let px = view.px_size(view.target);
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.push_model_matrix(model.as_mat4());
        let rm = self.get(m, color);
        for (a, b) in &rm.edges {
            let off = |p: Vec3| -> Vec3 {
                let dir = if persp { (eye_local - p).normalize() } else { back_local };
                p + dir * px * 4.0
            };
            draw_line_3d(off(*a), off(*b), col);
        }
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.pop_model_matrix();
    }

    /// Draws a set of triangles (e.g. a hovered face) as an overlay.
    pub fn draw_overlay(&mut self, m: &Mesh, tris: &[usize], model: &DMat4, col: [f32; 3]) {
        if tris.is_empty() {
            return;
        }
        let rm = build_render_mesh(m, col, Some(tris));
        self.material.set_uniform("Tint", vec4(0.0, 0.0, 0.0, 0.0));
        gl_use_material(&self.material);
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.push_model_matrix(model.as_mat4());
        for c in &rm.chunks {
            draw_mesh(c);
        }
        let gl = unsafe { get_internal_gl() };
        gl.quad_gl.pop_model_matrix();
        gl_use_default_material();
    }
}

pub const CHUNK_TRIS: usize = 15000;

fn build_render_mesh(m: &Mesh, color: [f32; 3], only: Option<&[usize]>) -> RenderMesh {
    let mut chunks = Vec::new();
    let col = [(color[0] * 255.0) as u8, (color[1] * 255.0) as u8, (color[2] * 255.0) as u8, 255];
    let list: Vec<usize> = match only {
        Some(l) => l.to_vec(),
        None => (0..m.tris.len()).collect(),
    };
    // smooth normals inside curved faces (cylinders, spheres), flat elsewhere
    let mut acc: HashMap<(u32, u32), glam::DVec3> = HashMap::new();
    if m.smooth.len() == m.tris.len() {
        for t in 0..m.tris.len() {
            let p = m.tri_pts(t);
            let area = (p[1] - p[0]).cross(p[2] - p[0]).length();
            for &vi in &m.tris[t] {
                *acc.entry((vi, m.smooth[t])).or_insert(glam::DVec3::ZERO) += m.tri_normal[t] * area;
            }
        }
    }
    for chunk in list.chunks(CHUNK_TRIS) {
        let mut vertices = Vec::with_capacity(chunk.len() * 3);
        let mut indices = Vec::with_capacity(chunk.len() * 3);
        for &t in chunk {
            let p = m.tri_pts(t);
            let fnorm = m.tri_normal[t];
            for (k, q) in p.iter().enumerate() {
                let vi = m.tris[t][k];
                let mut n = acc.get(&(vi, m.smooth.get(t).copied().unwrap_or(0))).map(|v| v.normalize_or_zero()).unwrap_or(fnorm);
                // keep genuinely flat faces flat when they border a curved one
                if n.dot(fnorm) < 0.93 {
                    n = fnorm;
                }
                indices.push(vertices.len() as u16);
                vertices.push(macroquad::models::Vertex { position: q.as_vec3(), uv: Vec2::ZERO, color: col, normal: n.as_vec3().extend(0.0) });
            }
        }
        chunks.push(macroquad::models::Mesh { vertices, indices, texture: None });
    }
    let edges = if only.is_none() { m.edges.iter().map(|(a, b)| (a.as_vec3(), b.as_vec3())).collect() } else { vec![] };
    RenderMesh { chunks, edges }
}

/// Background gradient like a classic engineering viewport, or a light photo studio
/// backdrop for realistic rendering.
pub fn draw_background(studio: bool) {
    let w = sw();
    let h = sh();
    let (top, bot) = if studio {
        // dark at the top, where the tree's light text sits, lighter towards the floor
        (Color::new(0.24, 0.25, 0.28, 1.0), Color::new(0.66, 0.67, 0.70, 1.0))
    } else {
        (Color::new(0.07, 0.08, 0.11, 1.0), Color::new(0.22, 0.26, 0.33, 1.0))
    };
    let steps = 48;
    for i in 0..steps {
        let t0 = i as f32 / steps as f32;
        let c = Color::new(
            top.r + (bot.r - top.r) * t0,
            top.g + (bot.g - top.g) * t0,
            top.b + (bot.b - top.b) * t0,
            1.0,
        );
        draw_rectangle(0.0, h * t0, w, h / steps as f32 + 1.0, c);
    }
}
