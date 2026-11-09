//! Sketcher measurements: "Measure Between" two items and "Measure Item" for one item.
//! Items are sketch points (ends, centres, corners), sketch elements, or the H / V axes.

use crate::geom::dist_point_segment;
use crate::sketch::*;
use glam::DVec2;
use std::f64::consts::PI;

#[derive(Clone, Debug, PartialEq)]
pub enum MeasItem {
    Point(DVec2),
    Elem(usize),
    AxisH,
    AxisV,
}

/// One finished measurement, kept on screen until cleared.
#[derive(Clone, Debug)]
pub struct SkMeasure {
    pub title: String,
    /// (label, value) rows shown in the result panel
    pub rows: Vec<(String, String)>,
    /// dimension line drawn in the sketch (from, to)
    pub line: Option<(DVec2, DVec2)>,
    /// short text drawn next to the dimension
    pub label: String,
    pub label_at: DVec2,
}

fn mm(v: f64) -> String {
    format!("{} mm", fmt((v * 1000.0).round() / 1000.0))
}
fn deg(v: f64) -> String {
    format!("{} deg", fmt((v * 1000.0).round() / 1000.0))
}
fn pt(p: DVec2) -> String {
    format!("H {}  V {}", fmt((p.x * 1000.0).round() / 1000.0), fmt((p.y * 1000.0).round() / 1000.0))
}

pub fn item_name(sk: &Sketch, it: &MeasItem) -> String {
    match it {
        MeasItem::Point(p) => format!("Point ({})", pt(*p)),
        MeasItem::Elem(i) => sk.shapes.get(*i).map(|s| format!("{}.{}", s.kind_name(), i + 1)).unwrap_or_default(),
        MeasItem::AxisH => "H axis".into(),
        MeasItem::AxisV => "V axis".into(),
    }
}

/// Polyline of an item and whether it is closed. Axes are long lines through the origin.
fn item_curve(sk: &Sketch, it: &MeasItem, extent: f64) -> (Vec<DVec2>, bool) {
    match it {
        MeasItem::Point(p) => (vec![*p], false),
        MeasItem::Elem(i) => match sk.shapes.get(*i) {
            Some(s) => (s.curve(), !s.is_open()),
            None => (vec![], false),
        },
        MeasItem::AxisH => (vec![DVec2::new(-extent, 0.0), DVec2::new(extent, 0.0)], false),
        MeasItem::AxisV => (vec![DVec2::new(0.0, -extent), DVec2::new(0.0, extent)], false),
    }
}

/// A straight item's start and direction (lines, axes).
fn item_line(sk: &Sketch, it: &MeasItem) -> Option<(DVec2, DVec2)> {
    match it {
        MeasItem::AxisH => Some((DVec2::ZERO, DVec2::X)),
        MeasItem::AxisV => Some((DVec2::ZERO, DVec2::Y)),
        MeasItem::Elem(i) => match sk.shapes.get(*i) {
            Some(Shape::Line { x1, y1, x2, y2 }) | Some(Shape::Axis { x1, y1, x2, y2 }) => {
                let d = DVec2::new(x2 - x1, y2 - y1);
                if d.length() > 1e-12 { Some((DVec2::new(*x1, *y1), d.normalize())) } else { None }
            }
            _ => None,
        },
        _ => None,
    }
}

fn item_centre(sk: &Sketch, it: &MeasItem) -> Option<(DVec2, f64)> {
    match it {
        MeasItem::Elem(i) => match sk.shapes.get(*i) {
            Some(Shape::Circle { cx, cy, r }) | Some(Shape::Arc { cx, cy, r, .. }) => Some((DVec2::new(*cx, *cy), r.abs())),
            _ => None,
        },
        _ => None,
    }
}

fn closest_on_segment(p: DVec2, a: DVec2, b: DVec2) -> DVec2 {
    let ab = b - a;
    let l = ab.length_squared();
    if l < 1e-18 {
        return a;
    }
    a + ab * ((p - a).dot(ab) / l).clamp(0.0, 1.0)
}

fn segs(c: &[DVec2], closed: bool) -> Vec<(DVec2, DVec2)> {
    let n = c.len();
    if n == 1 {
        return vec![(c[0], c[0])];
    }
    let m = if closed { n } else { n - 1 };
    (0..m).map(|k| (c[k], c[(k + 1) % n])).collect()
}

/// Closest points between two polylines.
fn closest(a: &[DVec2], ac: bool, b: &[DVec2], bc: bool) -> Option<(DVec2, DVec2)> {
    let mut best: Option<(f64, DVec2, DVec2)> = None;
    for (p0, p1) in segs(a, ac) {
        for (q0, q1) in segs(b, bc) {
            // crossing segments touch
            let r = p1 - p0;
            let s = q1 - q0;
            let den = r.perp_dot(s);
            if den.abs() > 1e-14 {
                let t = (q0 - p0).perp_dot(s) / den;
                let u = (q0 - p0).perp_dot(r) / den;
                if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                    let x = p0 + r * t;
                    return Some((x, x));
                }
            }
            for (p, q) in [
                (p0, closest_on_segment(p0, q0, q1)),
                (p1, closest_on_segment(p1, q0, q1)),
                (closest_on_segment(q0, p0, p1), q0),
                (closest_on_segment(q1, p0, p1), q1),
            ] {
                let d = (p - q).length();
                if best.map_or(true, |b| d < b.0) {
                    best = Some((d, p, q));
                }
            }
        }
    }
    best.map(|b| (b.1, b.2))
}

/// Measure Between: minimum distance, plus angle for two straight items and centre
/// distance for circles / arcs.
pub fn measure_between(sk: &Sketch, a: &MeasItem, b: &MeasItem) -> Option<SkMeasure> {
    let (ca, cla) = item_curve(sk, a, 1e5);
    let (cb, clb) = item_curve(sk, b, 1e5);
    if ca.is_empty() || cb.is_empty() {
        return None;
    }
    let mut rows = vec![("From".to_string(), item_name(sk, a)), ("To".to_string(), item_name(sk, b))];
    let (mut p, mut q) = closest(&ca, cla, &cb, clb)?;
    let mut label;
    match (item_line(sk, a), item_line(sk, b)) {
        (Some((pa, da)), Some((pb, db))) => {
            let ang = da.dot(db).clamp(-1.0, 1.0).acos().to_degrees();
            let acute = if ang > 90.0 { 180.0 - ang } else { ang };
            if acute < 1e-6 {
                // parallel: perpendicular distance between the two lines
                let dist = da.perp_dot(pb - pa).abs();
                rows.push(("Distance (parallel)".into(), mm(dist)));
                // dimension from the middle of the first item straight across
                let mid = if let MeasItem::Elem(_) = a { (ca[0] + ca[ca.len() - 1]) * 0.5 } else { closest_on_segment(cb[0], ca[0], ca[1]) };
                let n = DVec2::new(-da.y, da.x);
                let side = if n.dot(pb - mid) >= 0.0 { 1.0 } else { -1.0 };
                p = mid;
                q = mid + n * side * dist;
                label = format!("{}", fmt((dist * 1000.0).round() / 1000.0));
            } else {
                let d = (p - q).length();
                rows.push(("Minimum distance".into(), mm(d)));
                label = format!("{} deg", fmt((acute * 1000.0).round() / 1000.0));
            }
            rows.push(("Angle".into(), format!("{}  (other side {})", deg(acute), deg(180.0 - acute))));
            if acute >= 1e-6 {
                return Some(SkMeasure { title: "Measure Between".into(), rows, line: None, label, label_at: p });
            }
        }
        _ => {
            let d = (p - q).length();
            rows.push(("Minimum distance".into(), mm(d)));
            label = fmt((d * 1000.0).round() / 1000.0);
            if let (MeasItem::Point(x), MeasItem::Point(y)) = (a, b) {
                rows.push(("Delta H".into(), mm((y.x - x.x).abs())));
                rows.push(("Delta V".into(), mm((y.y - x.y).abs())));
                let ang = (y.y - x.y).atan2(y.x - x.x).to_degrees();
                rows.push(("Angle to H axis".into(), deg(ang.rem_euclid(180.0))));
            } else {
                rows.push(("Delta H".into(), mm((q.x - p.x).abs())));
                rows.push(("Delta V".into(), mm((q.y - p.y).abs())));
            }
            if let (Some((c1, _)), Some((c2, _))) = (item_centre(sk, a), item_centre(sk, b)) {
                let cd = (c2 - c1).length();
                rows.push(("Centre distance".into(), mm(cd)));
                if d < 1e-9 || (p - q).length() < 1e-9 {
                    p = c1;
                    q = c2;
                    label = format!("C {}", fmt((cd * 1000.0).round() / 1000.0));
                }
            } else {
                let pc = match (a, b) {
                    (MeasItem::Point(x), _) => item_centre(sk, b).map(|(c, _)| (*x - c).length()),
                    (_, MeasItem::Point(x)) => item_centre(sk, a).map(|(c, _)| (*x - c).length()),
                    _ => None,
                };
                if let Some(d) = pc {
                    rows.push(("Distance to centre".into(), mm(d)));
                }
            }
        }
    }
    Some(SkMeasure { title: "Measure Between".into(), rows, line: Some((p, q)), label, label_at: (p + q) * 0.5 })
}

fn polyline_len(c: &[DVec2], closed: bool) -> f64 {
    segs(c, closed).iter().map(|(a, b)| (*b - *a).length()).sum()
}

fn area(c: &[DVec2]) -> f64 {
    crate::geom::signed_area(c).abs()
}

/// Measure Item: the properties of one item.
pub fn measure_item(sk: &Sketch, it: &MeasItem) -> Option<SkMeasure> {
    let mut rows = vec![("Item".to_string(), item_name(sk, it))];
    let (label, at, line) = match it {
        MeasItem::Point(p) => {
            rows.push(("Position".into(), pt(*p)));
            rows.push(("Distance to origin".into(), mm(p.length())));
            (pt(*p), *p, None)
        }
        MeasItem::AxisH | MeasItem::AxisV => return None,
        MeasItem::Elem(i) => {
            let s = sk.shapes.get(*i)?;
            match s {
                Shape::Line { x1, y1, x2, y2 } | Shape::Axis { x1, y1, x2, y2 } => {
                    let a = DVec2::new(*x1, *y1);
                    let b = DVec2::new(*x2, *y2);
                    let d = b - a;
                    rows.push(("Length".into(), mm(d.length())));
                    rows.push(("Angle to H axis".into(), deg(d.y.atan2(d.x).to_degrees().rem_euclid(180.0))));
                    rows.push(("Delta H / Delta V".into(), format!("{} / {}", mm(d.x.abs()), mm(d.y.abs()))));
                    rows.push(("Start point".into(), pt(a)));
                    rows.push(("End point".into(), pt(b)));
                    rows.push(("Middle point".into(), pt((a + b) * 0.5)));
                    (format!("L {}", fmt((d.length() * 1000.0).round() / 1000.0)), (a + b) * 0.5, Some((a, b)))
                }
                Shape::Circle { cx, cy, r } => {
                    let c = DVec2::new(*cx, *cy);
                    rows.push(("Radius".into(), mm(*r)));
                    rows.push(("Diameter".into(), mm(r * 2.0)));
                    rows.push(("Circumference".into(), mm(2.0 * PI * r)));
                    rows.push(("Area".into(), format!("{} mm2", fmt((PI * r * r * 1000.0).round() / 1000.0))));
                    rows.push(("Centre".into(), pt(c)));
                    (format!("D {}", fmt((r * 2000.0).round() / 1000.0)), c + DVec2::new(r * 0.7, r * 0.7), Some((c - DVec2::X * *r, c + DVec2::X * *r)))
                }
                Shape::Arc { cx, cy, r, a0, a1 } => {
                    let c = DVec2::new(*cx, *cy);
                    let sw = sweep_deg(*a0, *a1);
                    let (pa, pb) = s.endpoints()?;
                    rows.push(("Radius".into(), mm(*r)));
                    rows.push(("Diameter".into(), mm(r * 2.0)));
                    rows.push(("Angle".into(), deg(sw)));
                    rows.push(("Arc length".into(), mm(r * sw.to_radians())));
                    rows.push(("Chord".into(), mm((pb - pa).length())));
                    rows.push(("Centre".into(), pt(c)));
                    let mid = (a0 + sw / 2.0).to_radians();
                    let pm = c + DVec2::new(mid.cos(), mid.sin()) * *r;
                    (format!("R {}", fmt((r * 1000.0).round() / 1000.0)), pm, Some((c, pm)))
                }
                Shape::Ellipse { cx, cy, a, b, .. } => {
                    let (a, b) = (a.abs(), b.abs());
                    let h = ((a - b) / (a + b)).powi(2);
                    let per = PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
                    rows.push(("Major / minor diameter".into(), format!("{} / {}", mm(a * 2.0), mm(b * 2.0))));
                    rows.push(("Perimeter".into(), mm(per)));
                    rows.push(("Area".into(), format!("{} mm2", fmt((PI * a * b * 1000.0).round() / 1000.0))));
                    rows.push(("Centre".into(), pt(DVec2::new(*cx, *cy))));
                    (format!("{} x {}", fmt(a * 2.0), fmt(b * 2.0)), DVec2::new(*cx, *cy), None)
                }
                Shape::Spline { closed: false, .. } => {
                    let c = s.curve();
                    rows.push(("Length".into(), mm(polyline_len(&c, false))));
                    let (pa, pb) = s.endpoints()?;
                    rows.push(("Start point".into(), pt(pa)));
                    rows.push(("End point".into(), pt(pb)));
                    (format!("L {}", fmt((polyline_len(&c, false) * 1000.0).round() / 1000.0)), c[c.len() / 2], None)
                }
                _ => {
                    // closed outlines: rectangle, polygon, slot, profile, closed spline
                    let o = s.outline();
                    let mut lo = DVec2::splat(f64::MAX);
                    let mut hi = DVec2::splat(f64::MIN);
                    for p in &o {
                        lo = lo.min(*p);
                        hi = hi.max(*p);
                    }
                    rows.push(("Perimeter".into(), mm(polyline_len(&o, true))));
                    rows.push(("Area".into(), format!("{} mm2", fmt((area(&o) * 1000.0).round() / 1000.0))));
                    rows.push(("Width x height".into(), format!("{} x {}", mm(hi.x - lo.x), mm(hi.y - lo.y))));
                    rows.push(("Centre of box".into(), pt((lo + hi) * 0.5)));
                    (s.label(), hi, None)
                }
            }
        }
    };
    Some(SkMeasure { title: "Measure Item".into(), rows, line, label, label_at: at })
}

/// The item under a sketch point: points first (within `tol`), then elements, then axes.
pub fn pick_item(sk: &Sketch, p: DVec2, tol: f64) -> Option<MeasItem> {
    let mut best: Option<(f64, DVec2)> = None;
    let mut cands = vec![DVec2::ZERO];
    for s in &sk.shapes {
        cands.extend(s.key_points());
        if let Some((a, b)) = s.endpoints() {
            cands.push(a);
            cands.push(b);
        }
    }
    for c in cands {
        let d = (c - p).length();
        if d < tol && best.map_or(true, |b| d < b.0) {
            best = Some((d, c));
        }
    }
    if let Some((_, c)) = best {
        return Some(MeasItem::Point(c));
    }
    let mut eb: Option<(f64, usize)> = None;
    for (i, s) in sk.shapes.iter().enumerate() {
        let d = s.distance(p);
        if d < tol && eb.map_or(true, |b| d < b.0) {
            eb = Some((d, i));
        }
    }
    if let Some((_, i)) = eb {
        return Some(MeasItem::Elem(i));
    }
    if p.y.abs() < tol {
        return Some(MeasItem::AxisH);
    }
    if p.x.abs() < tol {
        return Some(MeasItem::AxisV);
    }
    let _ = dist_point_segment;
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Frame;

    #[test]
    fn measures() {
        let mut sk = Sketch::new(Frame::xy(), "xy".into());
        sk.push(Shape::Line { x1: 0.0, y1: 0.0, x2: 30.0, y2: 0.0 }, false);
        sk.push(Shape::Line { x1: 0.0, y1: 10.0, x2: 30.0, y2: 10.0 }, false);
        sk.push(Shape::Circle { cx: 50.0, cy: 0.0, r: 5.0 }, false);
        sk.push(Shape::Line { x1: 0.0, y1: 0.0, x2: 10.0, y2: 10.0 }, false);
        let m = measure_between(&sk, &MeasItem::Elem(0), &MeasItem::Elem(1)).unwrap();
        assert!(m.rows.iter().any(|r| r.0.contains("parallel") && r.1 == "10 mm"), "{:?}", m.rows);
        let m = measure_between(&sk, &MeasItem::Point(DVec2::new(30.0, 0.0)), &MeasItem::Elem(2)).unwrap();
        assert!(m.rows.iter().any(|r| r.0 == "Minimum distance" && r.1 == "15 mm"), "{:?}", m.rows);
        assert!(m.rows.iter().any(|r| r.0 == "Distance to centre" && r.1 == "20 mm"), "{:?}", m.rows);
        let m = measure_between(&sk, &MeasItem::Elem(0), &MeasItem::Elem(3)).unwrap();
        assert!(m.rows.iter().any(|r| r.0 == "Angle" && r.1.starts_with("45 deg")), "{:?}", m.rows);
        let m = measure_item(&sk, &MeasItem::Elem(2)).unwrap();
        assert!(m.rows.iter().any(|r| r.0 == "Diameter" && r.1 == "10 mm"));
        let m = measure_item(&sk, &MeasItem::Elem(3)).unwrap();
        assert!(m.rows.iter().any(|r| r.0 == "Length" && r.1 == "14.142 mm"), "{:?}", m.rows);
        assert_eq!(pick_item(&sk, DVec2::new(30.2, 0.1), 0.5), Some(MeasItem::Point(DVec2::new(30.0, 0.0))));
        assert_eq!(pick_item(&sk, DVec2::new(20.0, 10.2), 0.5), Some(MeasItem::Elem(1)));
    }
}
