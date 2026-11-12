//! CATIA V5 `.CATPart` import.
//!
//! The format is proprietary and undocumented. The feature history (sketches, pads, ...)
//! is not readable, but CATIA also stores the tessellated solid it shows on screen, and
//! that is what gets imported here: a watertight triangle mesh in millimetres.
//!
//! Layout of one tessellated body, worked out from a V5R21 file:
//!   count nv            vertices
//!   count nn            vertices that carry a smooth normal
//!   4 bytes             unknown
//!   2 bits per vertex   how much of the position is new (see `NEW`)
//!   count + f32 LE      the new coordinates, x-sorted, repeats left out
//!   count + i16 pairs   packed normals (skipped: recomputed from the triangles)
//!   nn nibbles          normal flags (skipped)
//!   f32 + 4 bytes       sag and unknown
//!   count               faces, each:
//!     u8                index size in bytes - 1
//!     u8                0x40 | 1 triangles | 2 strips | 4 fans | 8 planar
//!     count per kind present, count of all indices, [3 x f32 normal if planar],
//!     strip lengths, fan lengths, then the big-endian indices: triangles, strips, fans
//! A "count" is one byte, or 0xff followed by a u32 LE.

use glam::DVec3;

struct Cur<'a> {
    d: &'a [u8],
    p: usize,
}

impl<'a> Cur<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let s = self.d.get(self.p..self.p.checked_add(n)?)?;
        self.p += n;
        Some(s)
    }
    fn u8(&mut self) -> Option<usize> {
        Some(self.take(1)?[0] as usize)
    }
    fn count(&mut self) -> Option<usize> {
        match self.u8()? {
            255 => Some(u32::from_le_bytes(self.take(4)?.try_into().ok()?) as usize),
            b => Some(b),
        }
    }
    fn counts(&mut self, n: usize) -> Option<Vec<usize>> {
        (0..n).map(|_| self.count()).collect()
    }
}

/// New coordinates per 2-bit vertex code: 0 = x y z, 1 = same point again, 2 = z, 3 = y z.
const NEW: [usize; 4] = [3, 0, 1, 2];

/// Tries to read one tessellated body starting at `start`. Every count is cross-checked,
/// so a wrong offset is rejected rather than read as garbage.
fn parse_body(d: &[u8], start: usize) -> Option<(Vec<[DVec3; 3]>, usize)> {
    let mut c = Cur { d, p: start };
    let (nv, nn) = (c.count()?, c.count()?);
    if nv < 3 || nn > nv {
        return None;
    }
    c.take(4)?;
    let codes = c.take((nv + 3) / 4)?;
    let code = |i: usize| NEW[(codes[i / 4] >> (2 * (i % 4)) & 3) as usize];
    if c.count()? != (0..nv).map(code).sum::<usize>() {
        return None;
    }
    let mut verts = Vec::with_capacity(nv);
    let mut p = [0.0f64; 3];
    for i in 0..nv {
        for axis in 3 - code(i)..3 {
            p[axis] = f32::from_le_bytes(c.take(4)?.try_into().ok()?) as f64;
            if !p[axis].is_finite() {
                return None;
            }
        }
        verts.push(DVec3::from(p));
    }
    let normals = c.count()?;
    c.take(normals.checked_mul(2)?)?;
    c.take((nn + 1) / 2 + 8)?;

    let mut tris = Vec::new();
    for _ in 0..c.count()? {
        // ponytail: index size taken from the face's first byte (2 bytes in the only sample seen);
        // revisit with a part of more than 65535 or fewer than 256 vertices if those fail to import
        let (width, kind) = (c.u8()? + 1, c.u8()?);
        if width > 4 || kind & 0xf0 != 0x40 {
            return None;
        }
        let ntri = if kind & 1 != 0 { c.count()? } else { 0 };
        let nstrip = if kind & 2 != 0 { c.count()? } else { 0 };
        let nfan = if kind & 4 != 0 { c.count()? } else { 0 };
        let total = c.count()?;
        if kind & 8 != 0 {
            c.take(12)?;
        }
        let (strips, fans) = (c.counts(nstrip)?, c.counts(nfan)?);
        if ntri.checked_mul(3)?.checked_add(strips.iter().chain(&fans).sum())? != total {
            return None;
        }
        let pts: Vec<DVec3> = c
            .take(total.checked_mul(width)?)?
            .chunks(width)
            .map(|b| verts.get(b.iter().fold(0usize, |a, &x| a << 8 | x as usize)).copied())
            .collect::<Option<_>>()?;
        let (single, mut rest) = pts.split_at(ntri * 3);
        tris.extend(single.chunks(3).map(|t| [t[0], t[1], t[2]]));
        for (i, &n) in strips.iter().chain(&fans).enumerate() {
            let (s, tail) = rest.split_at(n);
            rest = tail;
            for k in 0..n.saturating_sub(2) {
                tris.push(if i >= strips.len() {
                    [s[0], s[k + 1], s[k + 2]]
                } else if k % 2 == 0 {
                    [s[k], s[k + 1], s[k + 2]]
                } else {
                    [s[k + 1], s[k], s[k + 2]]
                });
            }
        }
    }
    tris.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    if tris.is_empty() {
        return None;
    }
    Some((tris, c.p))
}

pub fn read_catpart(d: &[u8]) -> Result<Vec<[DVec3; 3]>, String> {
    if !d.starts_with(b"V5_CFV2") {
        return Err("not a CATIA V5 file".into());
    }
    // ponytail: the bodies are found by trying every offset instead of walking CATIA's container
    // tree (undocumented). Fine for parts; walk the tree if huge files ever feel slow.
    let mut out = Vec::new();
    let mut i = 0;
    while i < d.len() {
        match parse_body(d, i) {
            Some((tris, end)) => {
                out.extend(tris);
                i = end;
            }
            None => i += 1,
        }
    }
    if out.is_empty() {
        return Err("no displayable solid found in this CATPart (it may be empty, or saved by a CATIA release that stores it differently)".into());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn volume(t: &[[DVec3; 3]]) -> f64 {
        t.iter().map(|t| t[0].dot(t[1].cross(t[2])) / 6.0).sum()
    }

    #[test]
    fn reads_a_tetrahedron() {
        let mut d = b"V5_CFV2\0junk".to_vec();
        d.extend([4, 0, 0, 0, 0, 0]); // 4 vertices, none with normals, 4 unknown bytes
        d.push(0x38); // codes: xyz, z, yz, xyz
        d.push(9);
        for f in [0.0f32, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0] {
            d.extend(f.to_le_bytes());
        }
        d.push(0); // no normals
        d.extend([0; 8]); // sag + unknown
        d.push(1); // one face: 2 triangles and a strip of 4, one-byte indices
        d.extend([0, 0x43, 2, 1, 10, 4]);
        d.extend([0, 2, 3, 0, 3, 1, 0, 1, 2, 3]);
        d.extend(b"trailing bytes");
        let t = read_catpart(&d).unwrap();
        assert_eq!(t.len(), 4);
        assert!((volume(&t) - 1.0 / 6.0).abs() < 1e-9, "closed and facing outwards");
        assert!(read_catpart(b"V5_CFV2\0nothing here").is_err());
        assert!(read_catpart(b"solid x").is_err());
    }

    /// Set BETTERCAD_CATPART=/path/to/file.CATPart to check a real file: it must come out watertight.
    #[test]
    fn real_file_is_watertight() {
        let Ok(path) = std::env::var("BETTERCAD_CATPART") else { return };
        let t = read_catpart(&std::fs::read(path).unwrap()).unwrap();
        let mut edges = std::collections::HashMap::new();
        let key = |p: DVec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        for tri in &t {
            for k in 0..3 {
                *edges.entry((key(tri[k]), key(tri[(k + 1) % 3]))).or_insert(0) += 1;
            }
        }
        let open = edges.iter().filter(|((a, b), &n)| n != 1 || !edges.contains_key(&(*b, *a))).count();
        println!("{} triangles, volume {:.1} mm3, {} open edges", t.len(), volume(&t), open);
        assert_eq!(open, 0);
        assert!(volume(&t) > 0.0);
    }
}
