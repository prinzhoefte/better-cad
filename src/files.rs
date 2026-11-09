//! A simple built-in file browser for opening and saving.

use crate::ui::*;
use macroquad::prelude::*;
use std::path::PathBuf;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Purpose {
    OpenDoc,
    SaveDoc,
    ExportStl,
    ExportStlSelected,
    ExportObj,
    ImportMesh,
    InsertPart,
    InsertMesh,
    SavePartOfComp,
}

impl Purpose {
    pub fn saving(self) -> bool {
        matches!(self, Purpose::SaveDoc | Purpose::ExportStl | Purpose::ExportStlSelected | Purpose::ExportObj | Purpose::SavePartOfComp)
    }
    pub fn title(self) -> &'static str {
        match self {
            Purpose::OpenDoc => "Open document",
            Purpose::SaveDoc => "Save document as",
            Purpose::ExportStl => "Export STL (for 3D printing)",
            Purpose::ExportStlSelected => "Export selected part as STL",
            Purpose::ExportObj => "Export OBJ (for games)",
            Purpose::ImportMesh => "Import STL / OBJ into the part",
            Purpose::InsertPart => "Insert an existing part",
            Purpose::InsertMesh => "Insert an STL / OBJ as a component",
            Purpose::SavePartOfComp => "Save selected component's part as",
        }
    }
}

pub struct FileDialog {
    pub purpose: Purpose,
    pub dir: PathBuf,
    pub exts: Vec<&'static str>,
    pub name: String,
    pub entries: Vec<(String, bool)>,
    pub scroll: usize,
    pub error: String,
}

pub enum FileResult {
    None,
    Cancel,
    Chosen(String),
}

impl FileDialog {
    pub fn new(purpose: Purpose, dir: PathBuf, exts: Vec<&'static str>, name: String) -> FileDialog {
        let mut d = FileDialog { purpose, dir, exts, name, entries: vec![], scroll: 0, error: String::new() };
        d.refresh();
        d
    }

    pub fn refresh(&mut self) {
        self.entries.clear();
        self.scroll = 0;
        if let Ok(rd) = std::fs::read_dir(&self.dir) {
            for e in rd.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if name.starts_with('.') {
                    continue;
                }
                let is_dir = e.path().is_dir();
                if !is_dir {
                    let lower = name.to_lowercase();
                    if !self.exts.iter().any(|x| lower.ends_with(&format!(".{}", x))) {
                        continue;
                    }
                }
                self.entries.push((name, is_dir));
            }
        }
        self.entries.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.to_lowercase().cmp(&b.0.to_lowercase())));
    }

    pub fn show(&mut self, ui: &mut Ui) -> FileResult {
        let w = 640.0f32.min(sw() - 40.0);
        let h = 470.0f32.min(sh() - 40.0);
        let r = Rect::new((sw() - w) / 2.0, (sh() - h) / 2.0, w, h);
        // dim everything behind
        ui.block(Rect::new(0.0, 0.0, sw(), sh()));
        draw_rectangle(0.0, 0.0, sw(), sh(), Color::new(0.0, 0.0, 0.0, 0.45));
        ui.panel(r, Some(self.purpose.title()));
        let mut y = r.y + 32.0;
        if ui.button("fd_up", Rect::new(r.x + 8.0, y, 60.0, 24.0), "Up") {
            if let Some(p) = self.dir.parent() {
                self.dir = p.to_path_buf();
                self.refresh();
            }
        }
        if ui.button("fd_home", Rect::new(r.x + 72.0, y, 60.0, 24.0), "Home") {
            if let Some(h) = std::env::var_os("HOME") {
                self.dir = PathBuf::from(h);
                self.refresh();
            }
        }
        let mut dir_s = self.dir.to_string_lossy().to_string();
        if ui.text_field("fd_dir", Rect::new(r.x + 138.0, y, w - 146.0, 24.0), &mut dir_s) {
            let p = PathBuf::from(&dir_s);
            if p.is_dir() {
                self.dir = p;
                self.refresh();
            }
        }
        y += 32.0;
        let list = Rect::new(r.x + 8.0, y, w - 16.0, h - 140.0);
        draw_rectangle(list.x, list.y, list.w, list.h, C_FIELD);
        draw_rectangle_lines(list.x, list.y, list.w, list.h, 1.0, C_BORDER);
        let row_h = 22.0;
        let rows = (list.h / row_h) as usize;
        if ui.hover(list) && ui.inp.wheel != 0.0 {
            if ui.inp.wheel < 0.0 {
                self.scroll = (self.scroll + 3).min(self.entries.len().saturating_sub(rows));
            } else {
                self.scroll = self.scroll.saturating_sub(3);
            }
        }
        let mut open_dir: Option<String> = None;
        let mut chosen: Option<String> = None;
        for (i, (name, is_dir)) in self.entries.iter().enumerate().skip(self.scroll).take(rows) {
            let rr = Rect::new(list.x + 2.0, list.y + 2.0 + (i - self.scroll) as f32 * row_h, list.w - 4.0, row_h);
            let hot = ui.hover(rr);
            let sel = !*is_dir && *name == self.name;
            if sel {
                draw_rectangle(rr.x, rr.y, rr.w, rr.h, C_ACTIVE);
            } else if hot {
                draw_rectangle(rr.x, rr.y, rr.w, rr.h, C_HOT);
            }
            if *is_dir {
                draw_rectangle(rr.x + 6.0, rr.y + 6.0, 14.0, 10.0, Color::new(0.95, 0.78, 0.35, 1.0));
                draw_rectangle(rr.x + 6.0, rr.y + 4.0, 6.0, 3.0, Color::new(0.95, 0.78, 0.35, 1.0));
            } else {
                draw_rectangle_lines(rr.x + 8.0, rr.y + 3.0, 11.0, 15.0, 1.0, C_DIM);
            }
            text(name, rr.x + 28.0, rr.y + 16.0, 16, C_TEXT);
            ui.reg(&format!("fd_row_{}", name), rr);
            if ui.clicked(rr) {
                if *is_dir {
                    open_dir = Some(name.clone());
                } else {
                    self.name = name.clone();
                    if ui.inp.double_click {
                        chosen = Some(name.clone());
                    }
                }
            }
        }
        if self.entries.is_empty() {
            text("(no matching files here)", list.x + 10.0, list.y + 22.0, 16, C_DIM);
        }
        if let Some(d) = open_dir {
            self.dir = self.dir.join(d);
            self.refresh();
        }
        let y2 = list.y + list.h + 10.0;
        text("File name:", r.x + 8.0, y2 + 17.0, 16, C_TEXT);
        ui.text_field("fd_name", Rect::new(r.x + 90.0, y2, w - 98.0, 24.0), &mut self.name);
        let typed = ui.live_text("fd_name").map(|s| s.to_string());
        text(&format!("Types: {}", self.exts.iter().map(|e| format!("*.{}", e)).collect::<Vec<_>>().join(", ")), r.x + 8.0, y2 + 46.0, 15, C_DIM);
        if !self.error.is_empty() {
            text(&self.error, r.x + 8.0, y2 + 64.0, 15, C_ERR);
        }
        let ok_label = if self.purpose.saving() { "Save" } else { "Open" };
        let ok = ui.button("fd_ok", Rect::new(r.x + w - 190.0, r.y + h - 34.0, 88.0, 26.0), ok_label) || ui.inp.key(KeyCode::Enter) && ui.focus.is_none();
        let enter_in_name = typed.is_some() && (ui.inp.key(KeyCode::Enter) || ui.inp.key(KeyCode::KpEnter));
        if ui.button("fd_cancel", Rect::new(r.x + w - 96.0, r.y + h - 34.0, 88.0, 26.0), "Cancel") || (ui.inp.key(KeyCode::Escape) && ui.focus.is_none()) {
            return FileResult::Cancel;
        }
        if let Some(c) = chosen {
            return FileResult::Chosen(self.dir.join(c).to_string_lossy().to_string());
        }
        if ok || enter_in_name {
            let mut name = typed.unwrap_or_else(|| self.name.clone());
            if name.trim().is_empty() {
                self.error = "Please type or pick a file name.".into();
                return FileResult::None;
            }
            if self.purpose.saving() {
                let lower = name.to_lowercase();
                if !self.exts.iter().any(|x| lower.ends_with(&format!(".{}", x))) {
                    name = format!("{}.{}", name, self.exts[0]);
                }
            }
            let p = if PathBuf::from(&name).is_absolute() { PathBuf::from(&name) } else { self.dir.join(&name) };
            if !self.purpose.saving() && !p.is_file() {
                self.error = "That file does not exist.".into();
                return FileResult::None;
            }
            return FileResult::Chosen(p.to_string_lossy().to_string());
        }
        FileResult::None
    }
}
