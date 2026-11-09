// On NixOS the window/OpenGL libraries are not in a standard place. miniquad loads them
// at runtime with dlopen, so we bake the folders that hold them into the program's run path.
use std::path::Path;

/// Only 64-bit x86-64 libraries can be loaded by this program (the store also holds 32-bit ones).
fn is_elf64_x86_64(p: &Path) -> bool {
    use std::io::Read;
    let mut b = [0u8; 20];
    match std::fs::File::open(p).and_then(|mut f| f.read_exact(&mut b)) {
        Ok(()) => &b[0..4] == b"\x7fELF" && b[4] == 2 && u16::from_le_bytes([b[18], b[19]]) == 62,
        Err(_) => false,
    }
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=RUSTC_LINKER");
    // a different compiler on PATH may use a different C library: look again
    println!("cargo:rerun-if-env-changed=PATH");
    if !cfg!(target_os = "linux") || !Path::new("/nix/store").exists() {
        return;
    }
    let wanted = [
        ("libGL.so.1", "libglvnd"),
        ("libX11.so.6", "libx11"),
        ("libXi.so.6", "libxi"),
        ("libXcursor.so.1", "libxcursor"),
        ("libxkbcommon.so.0", "libxkbcommon"),
        ("libXrandr.so.2", "libxrandr"),
        ("libXext.so.6", "libxext"),
        ("libXrender.so.1", "libxrender"),
    ];
    // the C library the compiler links against: loaded libraries must be built for the same one
    // the linker cargo resolved (tools/cc), else cc on PATH
    let linker = std::env::var("RUSTC_LINKER").unwrap_or_else(|_| "cc".into());
    let glibc = std::process::Command::new(&linker)
        .arg("-print-file-name=libc.so.6")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .and_then(|s| {
            let s = s.trim().to_string();
            s.strip_prefix("/nix/store/").map(|r| format!("/nix/store/{}", r.split('/').next().unwrap_or("")))
        });
    let matches_glibc = |f: &Path| -> bool {
        match (&glibc, std::fs::read(f)) {
            (Some(g), Ok(bytes)) => bytes.windows(g.len()).any(|w| w == g.as_bytes()),
            _ => false,
        }
    };
    let mut dirs: Vec<String> = Vec::new();
    if Path::new("/run/opengl-driver/lib").exists() {
        dirs.push("/run/opengl-driver/lib".into());
    }
    let entries: Vec<_> = match std::fs::read_dir("/nix/store") {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(_) => return,
    };
    for (lib, pkg) in wanted.iter() {
        let mut best: Option<(String, String)> = None;
        let mut fallback: Option<(String, String)> = None;
        for p in &entries {
            let name = match p.file_name().and_then(|n| n.to_str()) { Some(n) => n.to_string(), None => continue };
            if name.len() < 34 { continue; }
            let rest = name[33..].to_lowercase();
            if !rest.starts_with(&format!("{}-", pkg)) { continue; }
            if rest.contains("-dev") || rest.ends_with(".drv") { continue; }
            let f = p.join("lib").join(lib);
            if f.exists() && is_elf64_x86_64(&f) {
                let key = rest.clone();
                let entry = (key.clone(), p.join("lib").to_string_lossy().to_string());
                if matches_glibc(&f) {
                    if best.as_ref().map_or(true, |(k, _)| key > *k) {
                        best = Some(entry);
                    }
                } else if fallback.as_ref().map_or(true, |(k, _)| key > *k) {
                    fallback = Some(entry);
                }
            }
        }
        if let Some((_, d)) = best.or(fallback) {
            if !dirs.contains(&d) { dirs.push(d); }
        }
    }
    // If the chosen window library was built for a different (newer) C library than the
    // compiler's, run the program with that C library: newer C libraries run older programs.
    let x11 = dirs.iter().find(|d| d.contains("-libx11-") || d.contains("-libX11-")).cloned();
    if let Some(x) = x11 {
        if let Ok(bytes) = std::fs::read(Path::new(&x).join("libX11.so.6")) {
            let text = String::from_utf8_lossy(&bytes);
            if let Some(pos) = text.find("-glibc-") {
                let start = text[..pos].rfind("/nix/store/").unwrap_or(pos);
                let end = text[pos..].find(|c: char| c == '/' || c == ':' || c == '\0').map(|e| pos + e).unwrap_or(text.len());
                let g = text[start..end].to_string();
                let ld = format!("{}/lib/ld-linux-x86-64.so.2", g);
                if Some(&g) != glibc.as_ref() && Path::new(&ld).exists() {
                    println!("cargo:rustc-link-arg=-Wl,-rpath,{}/lib", g);
                    println!("cargo:rustc-link-arg=-Wl,--dynamic-linker={}", ld);
                }
            }
        }
    }
    for d in dirs {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", d);
    }
}
