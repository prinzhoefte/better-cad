//! Input snapshot (real or scripted) and a small immediate-mode widget kit.

use macroquad::prelude::*;
use std::collections::HashMap;

#[derive(Clone, Default)]
pub struct Input {
    pub mouse: Vec2,
    pub down: [bool; 3],
    pub pressed: [bool; 3],
    pub released: [bool; 3],
    pub double_click: bool,
    pub wheel: f32,
    pub chars: Vec<char>,
    pub keys: Vec<KeyCode>,
    pub ctrl: bool,
    pub shift: bool,
}

pub const LMB: usize = 0;
pub const RMB: usize = 1;
pub const MMB: usize = 2;

impl Input {
    pub fn gather(last_click: &mut f64, last_pos: &mut Vec2) -> Input {
        let (mx, my) = mouse_position();
        let (mx, my) = (mx / ui_scale(), my / ui_scale());
        let btns = [MouseButton::Left, MouseButton::Right, MouseButton::Middle];
        let mut i = Input { mouse: vec2(mx, my), ..Default::default() };
        for k in 0..3 {
            i.down[k] = is_mouse_button_down(btns[k]);
            i.pressed[k] = is_mouse_button_pressed(btns[k]);
            i.released[k] = is_mouse_button_released(btns[k]);
        }
        if i.pressed[LMB] {
            let now = get_time();
            if now - *last_click < 0.35 && (i.mouse - *last_pos).length() < 5.0 {
                i.double_click = true;
                *last_click = 0.0;
            } else {
                *last_click = now;
            }
            *last_pos = i.mouse;
        }
        let w = mouse_wheel().1;
        i.wheel = if w > 0.0 { 1.0 } else if w < 0.0 { -1.0 } else { 0.0 };
        while let Some(c) = get_char_pressed() {
            if !c.is_control() {
                i.chars.push(c);
            }
        }
        i.keys = get_keys_pressed().into_iter().collect();
        i.ctrl = is_key_down(KeyCode::LeftControl) || is_key_down(KeyCode::RightControl);
        i.shift = is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift);
        i
    }
    pub fn key(&self, k: KeyCode) -> bool {
        self.keys.contains(&k)
    }
}

// colours of the interface
pub const C_PANEL: Color = Color::new(0.19, 0.20, 0.23, 0.97);
pub const C_PANEL_DARK: Color = Color::new(0.14, 0.15, 0.17, 1.0);
pub const C_BORDER: Color = Color::new(0.34, 0.36, 0.41, 1.0);
pub const C_TEXT: Color = Color::new(0.88, 0.89, 0.92, 1.0);
pub const C_DIM: Color = Color::new(0.55, 0.57, 0.62, 1.0);
pub const C_HOT: Color = Color::new(0.30, 0.36, 0.46, 1.0);
pub const C_ACTIVE: Color = Color::new(0.72, 0.45, 0.12, 1.0);
pub const C_SEL: Color = Color::new(0.98, 0.55, 0.12, 1.0);
pub const C_TITLE: Color = Color::new(0.17, 0.32, 0.52, 1.0);
pub const C_BUTTON: Color = Color::new(0.24, 0.25, 0.28, 1.0);
pub const C_FIELD: Color = Color::new(0.11, 0.12, 0.14, 1.0);
pub const C_ACCENT: Color = Color::new(0.55, 0.78, 1.0, 1.0);
pub const C_ERR: Color = Color::new(1.0, 0.42, 0.36, 1.0);

pub struct Ui {
    pub inp: Input,
    pub rects: Vec<Rect>,
    pub prev_rects: Vec<Rect>,
    pub ids: HashMap<String, Rect>,
    pub prev_ids: HashMap<String, Rect>,
    pub focus: Option<String>,
    pub edit_buf: String,
    pub tooltip: Option<(Vec2, String)>,
    pub capture_mouse: bool,
    pub font_size: u16,
    /// the whole text of a just-focused field is selected: typing replaces it
    pub select_all: bool,
    /// a focused field that lost focus this frame commits its value once
    pending_commit: Option<String>,
}

// The interface font: Noto Sans (SIL Open Font License, see assets/fonts/OFL.txt), built into
// the program so it looks the same everywhere. It is much easier to read than macroquad's
// built-in pixel font.
static UI_FONT_DATA: &[u8] = include_bytes!("../assets/fonts/NotoSans.ttf");

thread_local! {
    // Leaked on purpose: the font lives as long as the program, and dropping it at exit would
    // free its texture after the graphics context is gone, which crashes.
    static UI_FONT: Option<&'static Font> =
        load_ttf_font_from_bytes(UI_FONT_DATA).ok().map(|f| &*Box::leak(Box::new(f)));
}

fn with_font<R>(f: impl FnOnce(Option<&Font>) -> R) -> R {
    UI_FONT.with(|font| f(*font))
}

pub fn text_w(s: &str, size: u16) -> f32 {
    with_font(|font| measure_text(s, font, size, 1.0).width)
}

pub fn text(s: &str, x: f32, y: f32, size: u16, c: Color) {
    // render glyphs at the real pixel size so text stays sharp when the interface is scaled
    let (fs, sc, asp) = camera_font_scale(size as f32);
    let k = ui_scale();
    with_font(|font| {
        draw_text_ex(
            s,
            (x * k).round() / k,
            (y * k).round() / k,
            TextParams { font, font_size: fs, font_scale: sc, font_scale_aspect: asp, color: c, ..Default::default() },
        );
    });
}

// ---------------------------------------------------------------- interface scaling

static mut UI_SCALE: f32 = 1.0;

pub const SCALE_STEPS: [f32; 9] = [0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0];

/// Size of the interface: 1.0 = 100 %, 2.0 = 200 % (e.g. for 4K monitors).
pub fn ui_scale() -> f32 {
    unsafe { UI_SCALE }
}
pub fn set_ui_scale(s: f32) {
    unsafe { UI_SCALE = s.clamp(0.5, 4.0) }
}
/// Width of the window in interface units (pixels / scale).
pub fn sw() -> f32 {
    screen_width() / ui_scale()
}
/// Height of the window in interface units.
pub fn sh() -> f32 {
    screen_height() / ui_scale()
}
/// Draws everything 2D in interface units, scaled up to the real pixels.
pub fn set_ui_camera() {
    let (w, h) = (sw(), sh());
    // y grows downwards like screen pixels
    set_camera(&Camera2D { target: vec2(w / 2.0, h / 2.0), zoom: vec2(2.0 / w, 2.0 / h), ..Default::default() });
}

fn settings_path() -> Option<std::path::PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".config")))
        .or_else(|| std::env::var_os("APPDATA").map(std::path::PathBuf::from))?;
    Some(base.join("bettercad").join("settings.json"))
}

/// Loads the saved interface size (100 % when there is none).
pub fn load_ui_scale() {
    if let Some(p) = settings_path() {
        if let Ok(t) = std::fs::read_to_string(p) {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(&t) {
                if let Some(s) = v.get("ui_scale").and_then(|x| x.as_f64()) {
                    set_ui_scale(s as f32);
                }
            }
        }
    }
}

pub fn save_ui_scale() {
    if let Some(p) = settings_path() {
        if let Some(d) = p.parent() {
            let _ = std::fs::create_dir_all(d);
        }
        let _ = std::fs::write(p, serde_json::json!({ "ui_scale": ui_scale() }).to_string());
    }
}

/// One step bigger or smaller along SCALE_STEPS.
pub fn step_ui_scale(up: bool) -> f32 {
    let cur = ui_scale();
    let next = if up {
        SCALE_STEPS.iter().cloned().find(|s| *s > cur + 1e-3).unwrap_or(cur)
    } else {
        SCALE_STEPS.iter().rev().cloned().find(|s| *s < cur - 1e-3).unwrap_or(cur)
    };
    set_ui_scale(next);
    save_ui_scale();
    next
}

impl Ui {
    pub fn new() -> Ui {
        Ui {
            inp: Input::default(),
            rects: vec![],
            prev_rects: vec![],
            ids: HashMap::new(),
            prev_ids: HashMap::new(),
            focus: None,
            edit_buf: String::new(),
            tooltip: None,
            capture_mouse: false,
            font_size: 16,
            select_all: false,
            pending_commit: None,
        }
    }

    pub fn begin(&mut self, inp: Input) {
        self.inp = inp;
        self.prev_rects = std::mem::take(&mut self.rects);
        self.prev_ids = std::mem::take(&mut self.ids);
        self.tooltip = None;
        // clicking anywhere outside the focused field ends editing
        if self.inp.pressed[LMB] {
            if let Some(f) = &self.focus {
                if let Some(r) = self.prev_ids.get(f) {
                    if !r.contains(self.inp.mouse) {
                        self.pending_commit = self.focus.take();
                    }
                }
            }
        }
    }

    /// Whether the mouse is over interface (as laid out last frame).
    pub fn over_ui(&self) -> bool {
        self.capture_mouse || self.prev_rects.iter().chain(self.rects.iter()).any(|r| r.contains(self.inp.mouse))
    }
    pub fn typing(&self) -> bool {
        self.focus.is_some()
    }

    pub fn block(&mut self, r: Rect) {
        self.rects.push(r);
    }
    pub fn reg(&mut self, id: &str, r: Rect) {
        self.ids.insert(id.to_string(), r);
    }
    pub fn hover(&self, r: Rect) -> bool {
        r.contains(self.inp.mouse)
    }
    pub fn clicked(&self, r: Rect) -> bool {
        self.inp.pressed[LMB] && r.contains(self.inp.mouse)
    }

    pub fn panel(&mut self, r: Rect, title: Option<&str>) {
        self.block(r);
        draw_rectangle(r.x + 3.0, r.y + 3.0, r.w, r.h, Color::new(0.0, 0.0, 0.0, 0.35));
        draw_rectangle(r.x, r.y, r.w, r.h, C_PANEL);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, 1.0, C_BORDER);
        if let Some(t) = title {
            draw_rectangle(r.x, r.y, r.w, 24.0, C_TITLE);
            text(t, r.x + 8.0, r.y + 17.0, 17, WHITE);
        }
    }

    pub fn button(&mut self, id: &str, r: Rect, label: &str) -> bool {
        self.button_ex(id, r, label, false, true)
    }

    pub fn button_ex(&mut self, id: &str, r: Rect, label: &str, active: bool, enabled: bool) -> bool {
        self.reg(id, r);
        self.block(r);
        let hot = enabled && self.hover(r);
        let bg = if active { C_ACTIVE } else if hot { C_HOT } else { C_BUTTON };
        draw_rectangle(r.x, r.y, r.w, r.h, bg);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, 1.0, C_BORDER);
        let tw = text_w(label, self.font_size);
        text(label, r.x + (r.w - tw) / 2.0, r.y + r.h / 2.0 + 5.0, self.font_size, if enabled { C_TEXT } else { C_DIM });
        enabled && self.clicked(r)
    }

    pub fn checkbox(&mut self, id: &str, x: f32, y: f32, label: &str, v: &mut bool) -> bool {
        let r = Rect::new(x, y, 18.0 + text_w(label, self.font_size) + 8.0, 20.0);
        self.reg(id, r);
        self.block(r);
        draw_rectangle(x, y + 2.0, 16.0, 16.0, C_FIELD);
        draw_rectangle_lines(x, y + 2.0, 16.0, 16.0, 1.0, C_BORDER);
        if *v {
            draw_line(x + 3.0, y + 10.0, x + 7.0, y + 15.0, 2.5, C_ACCENT);
            draw_line(x + 7.0, y + 15.0, x + 14.0, y + 4.0, 2.5, C_ACCENT);
        }
        text(label, x + 24.0, y + 15.0, self.font_size, C_TEXT);
        if self.clicked(r) {
            *v = !*v;
            return true;
        }
        false
    }

    /// A row of mutually exclusive choices. Returns true when changed.
    pub fn choice(&mut self, id: &str, r: Rect, options: &[&str], sel: &mut usize) -> bool {
        let w = r.w / options.len() as f32;
        let mut changed = false;
        for (i, o) in options.iter().enumerate() {
            let rr = Rect::new(r.x + w * i as f32, r.y, w - 2.0, r.h);
            if self.button_ex(&format!("{}_{}", id, i), rr, o, *sel == i, true) && *sel != i {
                *sel = i;
                changed = true;
            }
        }
        changed
    }

    /// Numeric field: click to type, Enter / Tab / clicking elsewhere applies, Esc cancels.
    /// Mouse wheel over the field nudges the value. Returns true when the value changed.
    pub fn num_field(&mut self, id: &str, r: Rect, v: &mut f64, step: f64) -> bool {
        self.reg(id, r);
        self.block(r);
        let focused = self.focus.as_deref() == Some(id);
        let mut changed = false;
        if self.pending_commit.as_deref() == Some(id) {
            self.pending_commit = None;
            if let Some(x) = eval_number(&self.edit_buf) {
                if x != *v {
                    *v = x;
                    changed = true;
                }
            }
        }
        draw_rectangle(r.x, r.y, r.w, r.h, if focused { Color::new(0.07, 0.08, 0.10, 1.0) } else { C_FIELD });
        draw_rectangle_lines(r.x, r.y, r.w, r.h, if focused { 2.0 } else { 1.0 }, if focused { C_SEL } else { C_BORDER });
        if focused {
            self.type_into_buffer();
            let caret = if (get_time() * 2.0) as i64 % 2 == 0 && !self.select_all { "|" } else { "" };
            if self.select_all {
                draw_rectangle(r.x + 3.0, r.y + 3.0, text_w(&self.edit_buf, self.font_size) + 4.0, r.h - 6.0, Color::new(0.20, 0.38, 0.65, 1.0));
            }
            text(&format!("{}{}", self.edit_buf, caret), r.x + 5.0, r.y + r.h / 2.0 + 5.0, self.font_size, C_TEXT);
            if self.inp.key(KeyCode::Enter) || self.inp.key(KeyCode::KpEnter) || self.inp.key(KeyCode::Tab) {
                if let Some(x) = eval_number(&self.edit_buf) {
                    if x != *v {
                        *v = x;
                        changed = true;
                    }
                }
                self.focus = None;
            } else if self.inp.key(KeyCode::Escape) {
                self.focus = None;
            }
        } else {
            text(&crate::model::fmt(*v), r.x + 5.0, r.y + r.h / 2.0 + 5.0, self.font_size, C_TEXT);
            if self.clicked(r) {
                if let Some(old) = self.focus.take() {
                    self.pending_commit = Some(old);
                }
                self.focus = Some(id.to_string());
                self.edit_buf = crate::model::fmt(*v);
                self.select_all = true;
            }
            if self.hover(r) && self.inp.wheel != 0.0 && step > 0.0 {
                *v += step * self.inp.wheel as f64;
                changed = true;
            }
        }
        changed
    }

    pub fn text_field(&mut self, id: &str, r: Rect, s: &mut String) -> bool {
        self.reg(id, r);
        self.block(r);
        let focused = self.focus.as_deref() == Some(id);
        let mut changed = false;
        if self.pending_commit.as_deref() == Some(id) {
            self.pending_commit = None;
            if *s != self.edit_buf {
                *s = self.edit_buf.clone();
                changed = true;
            }
        }
        draw_rectangle(r.x, r.y, r.w, r.h, C_FIELD);
        draw_rectangle_lines(r.x, r.y, r.w, r.h, if focused { 2.0 } else { 1.0 }, if focused { C_SEL } else { C_BORDER });
        if focused {
            self.type_into_buffer();
            let caret = if (get_time() * 2.0) as i64 % 2 == 0 { "|" } else { "" };
            let shown = fit_tail(&format!("{}{}", self.edit_buf, caret), r.w - 10.0, self.font_size);
            text(&shown, r.x + 5.0, r.y + r.h / 2.0 + 5.0, self.font_size, C_TEXT);
            if self.inp.key(KeyCode::Enter) || self.inp.key(KeyCode::KpEnter) || self.inp.key(KeyCode::Tab) {
                if *s != self.edit_buf {
                    *s = self.edit_buf.clone();
                    changed = true;
                }
                self.focus = None;
            } else if self.inp.key(KeyCode::Escape) {
                self.focus = None;
            }
        } else {
            let shown = fit_tail(s, r.w - 10.0, self.font_size);
            text(&shown, r.x + 5.0, r.y + r.h / 2.0 + 5.0, self.font_size, C_TEXT);
            if self.clicked(r) {
                if let Some(old) = self.focus.take() {
                    self.pending_commit = Some(old);
                }
                self.focus = Some(id.to_string());
                self.edit_buf = s.clone();
                self.select_all = true;
            }
        }
        changed
    }

    fn type_into_buffer(&mut self) {
        let chars = self.inp.chars.clone();
        if !chars.is_empty() && self.select_all {
            self.edit_buf.clear();
        }
        for c in chars {
            self.edit_buf.push(c);
            self.select_all = false;
        }
        if self.inp.key(KeyCode::Backspace) {
            if self.select_all {
                self.edit_buf.clear();
            } else {
                self.edit_buf.pop();
            }
            self.select_all = false;
        }
        if self.inp.key(KeyCode::Left) || self.inp.key(KeyCode::Right) || self.inp.key(KeyCode::End) {
            self.select_all = false;
        }
    }

    /// The text currently typed into a focused field.
    pub fn live_text(&self, id: &str) -> Option<&str> {
        if self.focus.as_deref() == Some(id) { Some(&self.edit_buf) } else { None }
    }

    pub fn tip(&mut self, r: Rect, s: &str) {
        if self.hover(r) {
            self.tooltip = Some((self.inp.mouse + vec2(14.0, 18.0), s.to_string()));
        }
    }

    pub fn draw_tooltip(&self) {
        if let Some((p, s)) = &self.tooltip {
            let lines: Vec<&str> = s.lines().collect();
            let w = lines.iter().map(|l| text_w(l, 15)).fold(0.0, f32::max) + 12.0;
            let h = lines.len() as f32 * 17.0 + 8.0;
            let x = p.x.min(sw() - w - 4.0);
            let y = p.y.min(sh() - h - 4.0);
            draw_rectangle(x, y, w, h, Color::new(0.26, 0.26, 0.20, 0.97));
            draw_rectangle_lines(x, y, w, h, 1.0, C_BORDER);
            for (i, l) in lines.iter().enumerate() {
                text(l, x + 6.0, y + 16.0 + i as f32 * 17.0, 15, C_TEXT);
            }
        }
    }
}

pub fn fit_tail(s: &str, w: f32, size: u16) -> String {
    if text_w(s, size) <= w {
        return s.to_string();
    }
    let chars: Vec<char> = s.chars().collect();
    for start in 0..chars.len() {
        let t: String = "...".chars().chain(chars[start..].iter().cloned()).collect();
        if text_w(&t, size) <= w {
            return t;
        }
    }
    String::new()
}

/// Parses a number or a simple arithmetic expression such as "20*3/2+1".
pub fn eval_number(s: &str) -> Option<f64> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect::<String>().replace(',', ".");
    if s.is_empty() {
        return None;
    }
    let b = s.as_bytes();
    let mut pos = 0usize;
    fn expr(b: &[u8], p: &mut usize) -> Option<f64> {
        let mut v = term(b, p)?;
        while *p < b.len() && (b[*p] == b'+' || b[*p] == b'-') {
            let op = b[*p];
            *p += 1;
            let r = term(b, p)?;
            if op == b'+' { v += r } else { v -= r }
        }
        Some(v)
    }
    fn term(b: &[u8], p: &mut usize) -> Option<f64> {
        let mut v = factor(b, p)?;
        while *p < b.len() && (b[*p] == b'*' || b[*p] == b'/') {
            let op = b[*p];
            *p += 1;
            let r = factor(b, p)?;
            if op == b'*' { v *= r } else { v /= r }
        }
        Some(v)
    }
    fn factor(b: &[u8], p: &mut usize) -> Option<f64> {
        if *p < b.len() && b[*p] == b'-' {
            *p += 1;
            return factor(b, p).map(|v| -v);
        }
        if *p < b.len() && b[*p] == b'(' {
            *p += 1;
            let v = expr(b, p)?;
            if *p < b.len() && b[*p] == b')' {
                *p += 1;
            }
            return Some(v);
        }
        let start = *p;
        while *p < b.len() && (b[*p].is_ascii_digit() || b[*p] == b'.' || b[*p] == b'e') {
            *p += 1;
        }
        let tok = std::str::from_utf8(&b[start..*p]).ok()?;
        let mut v: f64 = tok.parse().ok()?;
        // optional unit suffixes
        let rest = std::str::from_utf8(&b[*p..]).ok()?;
        for (u, f) in [("mm", 1.0), ("cm", 10.0), ("in", 25.4), ("deg", 1.0), ("m", 1000.0)] {
            if rest.starts_with(u) {
                v *= f;
                *p += u.len();
                break;
            }
        }
        Some(v)
    }
    let v = expr(b, &mut pos)?;
    if pos != b.len() || !v.is_finite() {
        return None;
    }
    Some(v)
}
