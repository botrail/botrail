//! G-code subset parser → [`crate::toolpath::ToolMove`] list.
//!
//! Scope (see `design/design-machining.md` §3.1): G0/G1 linear, G2/G3 arcs
//! (IJK center offsets or R, tessellated at chord tolerance, helical via
//! the off-plane word), G17/G18/G19 planes, G20/G21 units, G90/G91
//! absolute/incremental, F feed (G94 units-per-minute only). Harmless
//! spindle/coolant words (S, M3/M4/M5, M7-M9) are collected as warnings;
//! anything that would silently change the toolpath's meaning — cutter
//! compensation (G41/G42), tool change (T/M6), per-rev feed (G95), dwell
//! (G4), canned cycles — is a line-numbered error, never a silent skip.
//!
//! Coordinates are part-frame millimeters (or inches under G20) and come
//! out as meters; the tool axis is the part frame's `+Z` for every target
//! (3-axis semantics — 5-axis paths enter via APT, a later phase).

use nalgebra::{Point3, Unit, Vector3};
use thiserror::Error;

use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind};

#[derive(Debug, Error)]
pub enum GcodeError {
    #[error("line {line}: {message}")]
    Unsupported { line: usize, message: String },
    #[error("line {line}: malformed word `{word}`")]
    Malformed { line: usize, word: String },
    #[error("line {line}: cutting move before any F word set a feed")]
    FeedUndefined { line: usize },
    #[error("line {line}: arc has neither IJK center offsets nor R")]
    ArcCenterMissing { line: usize },
    #[error("line {line}: arc radius {radius:.3}mm is shorter than half the chord")]
    ArcRadiusTooShort { line: usize, radius: f64 },
    #[error("program contains no motion")]
    Empty,
}

#[derive(Debug, Clone)]
pub struct GcodeOptions {
    /// Maximum chord-to-arc deviation when tessellating arcs (m).
    pub chord_tol: f64,
}

impl Default for GcodeOptions {
    fn default() -> Self {
        GcodeOptions { chord_tol: 1e-4 }
    }
}

/// Parse result: the moves plus non-fatal notes (ignored spindle/coolant
/// words, work-offset selections...).
#[derive(Debug, Clone)]
pub struct ParsedGcode {
    pub moves: Vec<ToolMove>,
    pub warnings: Vec<String>,
}

#[derive(Clone, Copy, PartialEq)]
enum Motion {
    Rapid,
    Linear,
    ArcCw,
    ArcCcw,
}

#[derive(Clone, Copy)]
enum Plane {
    Xy,
    Zx,
    Yz,
}

impl Plane {
    /// In-plane axis indices (a, b) and the off-plane index, in the order
    /// the arc sweeps them (G17: X=a, Y=b, Z off).
    fn axes(self) -> (usize, usize, usize) {
        match self {
            Plane::Xy => (0, 1, 2),
            Plane::Zx => (2, 0, 1),
            Plane::Yz => (1, 2, 0),
        }
    }
}

struct State {
    motion: Option<Motion>,
    plane: Plane,
    /// Coordinate scale to meters (mm: 1e-3, inch: 0.0254).
    scale: f64,
    absolute: bool,
    /// Feed in m/s; G-code F is units/minute.
    feed: Option<f64>,
    position: Vector3<f64>,
}

fn strip_comments(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut depth = 0usize;
    for c in line.chars() {
        match c {
            '(' => depth += 1,
            ')' => depth = depth.saturating_sub(1),
            ';' if depth == 0 => break,
            _ if depth == 0 => out.push(c),
            _ => {}
        }
    }
    out
}

fn code_eq(value: f64, code: f64) -> bool {
    (value - code).abs() < 1e-6
}

/// Tessellates one arc into targets appended to `out`. `end` and `center`
/// are absolute part-frame positions in meters; the sweep runs from
/// `state.position` about the plane's off-axis.
#[allow(clippy::too_many_arguments)]
fn emit_arc(
    out: &mut Vec<PathTarget>,
    start: Vector3<f64>,
    end: Vector3<f64>,
    center_ab: (f64, f64),
    cw: bool,
    plane: Plane,
    chord_tol: f64,
    tool_axis: Unit<Vector3<f64>>,
) {
    let (ia, ib, ic) = plane.axes();
    let (ca, cb) = center_ab;
    let (sa, sb) = (start[ia] - ca, start[ib] - cb);
    let (ea, eb) = (end[ia] - ca, end[ib] - cb);
    let radius = (sa * sa + sb * sb).sqrt();
    let a0 = sb.atan2(sa);
    let a1 = eb.atan2(ea);
    // Sweep in the commanded direction; a zero sweep with distinct...
    // identical endpoints means a full circle (IJK form).
    let mut sweep = a1 - a0;
    if cw {
        while sweep >= -1e-9 {
            sweep -= std::f64::consts::TAU;
        }
    } else {
        while sweep <= 1e-9 {
            sweep += std::f64::consts::TAU;
        }
    }
    // Chord tolerance → angular step via the sagitta: e = r(1 - cos(α/2)).
    let alpha = if radius > chord_tol {
        2.0 * (1.0 - chord_tol / radius).acos()
    } else {
        std::f64::consts::FRAC_PI_2
    };
    let steps = (sweep.abs() / alpha.min(0.5)).ceil().max(1.0) as usize;
    for k in 1..=steps {
        let u = k as f64 / steps as f64;
        let ang = a0 + sweep * u;
        let mut p = Vector3::zeros();
        p[ia] = ca + radius * ang.cos();
        p[ib] = cb + radius * ang.sin();
        p[ic] = start[ic] + (end[ic] - start[ic]) * u;
        // Land exactly on the commanded endpoint (float drift on the trig
        // path would otherwise leak into the datum).
        if k == steps {
            p[ia] = end[ia];
            p[ib] = end[ib];
            p[ic] = end[ic];
        }
        out.push(PathTarget {
            position: Point3::from(p),
            tool_axis,
            spin: None,
        });
    }
}

/// Parses a G-code program into toolpath moves. Consecutive targets under
/// the same motion mode and feed merge into one [`ToolMove`].
pub fn parse_gcode(text: &str, options: &GcodeOptions) -> Result<ParsedGcode, GcodeError> {
    let tool_axis = Unit::new_normalize(Vector3::z());
    let mut state = State {
        motion: None,
        plane: Plane::Xy,
        scale: 1e-3,
        absolute: true,
        feed: None,
        position: Vector3::zeros(),
    };
    let mut moves: Vec<ToolMove> = Vec::new();
    let mut warnings: Vec<String> = Vec::new();
    let mut warned: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    let warn_once = |warned: &mut std::collections::BTreeSet<String>,
                         warnings: &mut Vec<String>,
                     key: &str,
                     line: usize| {
        if warned.insert(key.to_string()) {
            warnings.push(format!("line {line}: {key} ignored"));
        }
    };

    'lines: for (lineno, raw) in text.lines().enumerate() {
        let line = lineno + 1;
        let clean = strip_comments(raw);
        let mut words: Vec<(char, f64, String)> = Vec::new();
        let mut chars = clean.chars().peekable();
        while let Some(c) = chars.next() {
            if c.is_whitespace() || c == '%' {
                continue;
            }
            let letter = c.to_ascii_uppercase();
            let mut number = String::new();
            while let Some(&n) = chars.peek() {
                if n.is_ascii_digit() || n == '.' || n == '-' || n == '+' {
                    number.push(n);
                    chars.next();
                } else {
                    break;
                }
            }
            let word = format!("{letter}{number}");
            let value: f64 = number.parse().map_err(|_| GcodeError::Malformed {
                line,
                word: word.clone(),
            })?;
            words.push((letter, value, word));
        }
        if words.is_empty() {
            continue;
        }

        // Pass 1: modal words (G/M/F/S/T/N) — motion codes latch, the
        // coordinates of this line then execute under the latched state.
        let mut has_motion_words = false;
        let mut arc_center: [Option<f64>; 3] = [None; 3];
        let mut arc_radius: Option<f64> = None;
        let mut target = state.position;
        let mut target_seen = [false; 3];
        for (letter, value, word) in &words {
            match letter {
                'N' => {}
                'G' => match *value {
                    v if code_eq(v, 0.0) => state.motion = Some(Motion::Rapid),
                    v if code_eq(v, 1.0) => state.motion = Some(Motion::Linear),
                    v if code_eq(v, 2.0) => state.motion = Some(Motion::ArcCw),
                    v if code_eq(v, 3.0) => state.motion = Some(Motion::ArcCcw),
                    v if code_eq(v, 17.0) => state.plane = Plane::Xy,
                    v if code_eq(v, 18.0) => state.plane = Plane::Zx,
                    v if code_eq(v, 19.0) => state.plane = Plane::Yz,
                    v if code_eq(v, 20.0) => state.scale = 0.0254,
                    v if code_eq(v, 21.0) => state.scale = 1e-3,
                    v if code_eq(v, 90.0) => state.absolute = true,
                    v if code_eq(v, 91.0) => state.absolute = false,
                    v if code_eq(v, 94.0) => {}
                    v if code_eq(v, 40.0) => {}
                    v if code_eq(v, 49.0) => {}
                    v if code_eq(v, 80.0) => {}
                    v if (54..=59).any(|c| code_eq(v, c as f64)) => {
                        warn_once(&mut warned, &mut warnings, &format!("work offset G{v}"), line);
                    }
                    v if code_eq(v, 41.0) || code_eq(v, 42.0) => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: format!(
                                "G{v}: cutter radius compensation is not supported — \
                                 post the toolpath pre-compensated"
                            ),
                        });
                    }
                    v if code_eq(v, 95.0) => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: "G95: feed-per-revolution is not supported (G94 only)"
                                .to_string(),
                        });
                    }
                    v if code_eq(v, 4.0) => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: "G4: dwell is not supported (author it as a sequence step)"
                                .to_string(),
                        });
                    }
                    v if code_eq(v, 28.0) || code_eq(v, 53.0) => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: format!("G{v}: machine-coordinate moves are not supported"),
                        });
                    }
                    v if (81..=89).any(|c| code_eq(v, c as f64)) => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: format!("G{v}: canned cycles are not supported"),
                        });
                    }
                    v => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: format!("G{v} is not supported"),
                        });
                    }
                },
                'M' => match *value {
                    v if code_eq(v, 2.0) || code_eq(v, 30.0) => break 'lines,
                    v if code_eq(v, 3.0) || code_eq(v, 4.0) || code_eq(v, 5.0) => {
                        warn_once(
                            &mut warned,
                            &mut warnings,
                            &format!("spindle word M{v}"),
                            line,
                        );
                    }
                    v if (7..=9).any(|c| code_eq(v, c as f64)) => {
                        warn_once(
                            &mut warned,
                            &mut warnings,
                            &format!("coolant word M{v}"),
                            line,
                        );
                    }
                    v if code_eq(v, 6.0) => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: "M6: tool change is not supported".to_string(),
                        });
                    }
                    v => {
                        return Err(GcodeError::Unsupported {
                            line,
                            message: format!("M{v} is not supported"),
                        });
                    }
                },
                'F' => state.feed = Some(value * state.scale / 60.0),
                'S' => warn_once(&mut warned, &mut warnings, "spindle speed S", line),
                'T' => {
                    return Err(GcodeError::Unsupported {
                        line,
                        message: "T: tool selection is not supported".to_string(),
                    });
                }
                'X' | 'Y' | 'Z' => {
                    let idx = (*letter as u8 - b'X') as usize;
                    let v = value * state.scale;
                    target[idx] = if state.absolute {
                        v
                    } else {
                        state.position[idx] + v
                    };
                    target_seen[idx] = true;
                    has_motion_words = true;
                }
                'I' | 'J' | 'K' => {
                    let idx = (*letter as u8 - b'I') as usize;
                    arc_center[idx] = Some(value * state.scale);
                    has_motion_words = true;
                }
                'R' => {
                    arc_radius = Some(value * state.scale);
                    has_motion_words = true;
                }
                _ => {
                    return Err(GcodeError::Malformed {
                        line,
                        word: word.clone(),
                    });
                }
            }
        }
        if !has_motion_words {
            continue;
        }
        let Some(motion) = state.motion else {
            return Err(GcodeError::Unsupported {
                line,
                message: "coordinates before any motion mode (G0/G1/G2/G3)".to_string(),
            });
        };

        // Pass 2: execute the motion.
        let kind = match motion {
            Motion::Rapid => ToolMoveKind::Rapid,
            Motion::Linear | Motion::ArcCw | Motion::ArcCcw => ToolMoveKind::Feed(
                state
                    .feed
                    .ok_or(GcodeError::FeedUndefined { line })?,
            ),
        };
        let mut targets: Vec<PathTarget> = Vec::new();
        match motion {
            Motion::Rapid | Motion::Linear => {
                targets.push(PathTarget {
                    position: Point3::from(target),
                    tool_axis,
                    spin: None,
                });
            }
            Motion::ArcCw | Motion::ArcCcw => {
                let cw = motion == Motion::ArcCw;
                let (ia, ib, _) = state.plane.axes();
                let center_ab = if arc_center.iter().any(Option::is_some) {
                    // IJK: incremental offsets from the start point.
                    (
                        state.position[ia] + arc_center[ia].unwrap_or(0.0),
                        state.position[ib] + arc_center[ib].unwrap_or(0.0),
                    )
                } else if let Some(r) = arc_radius {
                    // R form: center on the perpendicular bisector; +R takes
                    // the minor arc, -R the major.
                    let (dx, dy) = (
                        target[ia] - state.position[ia],
                        target[ib] - state.position[ib],
                    );
                    let half = 0.5 * (dx * dx + dy * dy).sqrt();
                    if r.abs() + 1e-12 < half {
                        return Err(GcodeError::ArcRadiusTooShort {
                            line,
                            radius: r.abs() * 1e3,
                        });
                    }
                    let h = (r * r - half * half).max(0.0).sqrt();
                    let (mx, my) = (
                        0.5 * (state.position[ia] + target[ia]),
                        0.5 * (state.position[ib] + target[ib]),
                    );
                    // Left normal of the chord; a minor CCW arc has its
                    // center on the left, a minor CW arc on the right, and
                    // a negative R (major arc) flips the side.
                    let (px, py) = (-dy / (2.0 * half), dx / (2.0 * half));
                    let side = if cw == (r >= 0.0) { -1.0 } else { 1.0 };
                    (mx + side * h * px, my + side * h * py)
                } else {
                    return Err(GcodeError::ArcCenterMissing { line });
                };
                emit_arc(
                    &mut targets,
                    state.position,
                    target,
                    center_ab,
                    cw,
                    state.plane,
                    options.chord_tol,
                    tool_axis,
                );
            }
        }
        state.position = *targets
            .last()
            .map(|t| &t.position.coords)
            .unwrap_or(&state.position);

        // Merge into the previous move when the kind matches.
        match moves.last_mut() {
            Some(last) if last.kind == kind => last.targets.extend(targets),
            _ => moves.push(ToolMove { kind, targets }),
        }
    }

    if moves.iter().all(|m| m.targets.is_empty()) {
        return Err(GcodeError::Empty);
    }
    Ok(ParsedGcode { moves, warnings })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn positions(moves: &[ToolMove]) -> Vec<Vec<[f64; 3]>> {
        moves
            .iter()
            .map(|m| m.targets.iter().map(|t| t.position.coords.into()).collect())
            .collect()
    }

    #[test]
    fn linear_program_parses_with_modal_state() {
        let parsed = parse_gcode(
            "G21 G90 G17\nG0 X10 Y0 Z5\nG1 Z-1 F300\nG1 X20\nY10\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        assert_eq!(parsed.moves.len(), 2);
        assert!(matches!(parsed.moves[0].kind, ToolMoveKind::Rapid));
        let ToolMoveKind::Feed(feed) = parsed.moves[1].kind else {
            panic!("expected feed");
        };
        assert!((feed - 0.005).abs() < 1e-12, "300mm/min = 5mm/s, got {feed}");
        let p = positions(&parsed.moves);
        assert_eq!(p[0], vec![[0.010, 0.0, 0.005]]);
        // Modal G1 and modal XZ: three cutting targets.
        assert_eq!(p[1].len(), 3);
        assert_eq!(p[1][2], [0.020, 0.010, -0.001]);
    }

    #[test]
    fn incremental_mode_accumulates() {
        let parsed = parse_gcode(
            "G91\nG0 X10\nG0 X10 Y5\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        let p = positions(&parsed.moves);
        assert_eq!(p[0], vec![[0.010, 0.0, 0.0], [0.020, 0.005, 0.0]]);
    }

    #[test]
    fn arc_stays_on_radius_and_lands_on_endpoint() {
        // Quarter circle, r=10mm, from (10,0) to (0,10) about (0,0), CCW.
        let parsed = parse_gcode(
            "G90 G17\nG0 X10 Y0\nG3 X0 Y10 I-10 J0 F600\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        let arc = &parsed.moves[1].targets;
        assert!(arc.len() > 3, "tessellated into {} points", arc.len());
        for t in arc {
            let r = (t.position.x.powi(2) + t.position.y.powi(2)).sqrt();
            assert!((r - 0.010).abs() < 1.1e-4, "radius drifted to {r}");
        }
        let last = arc.last().unwrap();
        assert_eq!(
            [last.position.x, last.position.y, last.position.z],
            [0.0, 0.010, 0.0]
        );
    }

    #[test]
    fn r_form_arc_matches_ijk() {
        let ijk = parse_gcode(
            "G0 X10 Y0\nG3 X0 Y10 I-10 J0 F600\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        let r = parse_gcode("G0 X10 Y0\nG3 X0 Y10 R10 F600\n", &GcodeOptions::default()).unwrap();
        let (a, b) = (&ijk.moves[1].targets, &r.moves[1].targets);
        assert_eq!(a.len(), b.len());
        for (ta, tb) in a.iter().zip(b) {
            assert!((ta.position - tb.position).norm() < 1e-9);
        }
    }

    #[test]
    fn helical_arc_interpolates_z() {
        let parsed = parse_gcode(
            "G0 X10 Y0 Z0\nG2 X10 Y0 Z-2 I-10 J0 F600\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        let arc = &parsed.moves[1].targets;
        // Full circle (start == end in-plane) descending 2mm.
        let last = arc.last().unwrap();
        assert!((last.position.z + 0.002).abs() < 1e-12);
        let mid = &arc[arc.len() / 2];
        assert!(mid.position.z < -1e-4 && mid.position.z > -0.002);
    }

    #[test]
    fn dangerous_codes_are_line_numbered_errors() {
        for (src, needle) in [
            ("G1 X5 F100\nG41 D1\n", "G41"),
            ("T2 M6\n", "tool"),
            ("G95\n", "G95"),
            ("G4 P1\n", "dwell"),
            ("G81 X0 Y0 Z-5 R1 F100\n", "canned"),
        ] {
            let err = parse_gcode(src, &GcodeOptions::default()).unwrap_err();
            let msg = err.to_string();
            assert!(msg.contains("line"), "no line number in `{msg}`");
            assert!(
                msg.to_lowercase().contains(&needle.to_lowercase()),
                "`{msg}` does not mention {needle}"
            );
        }
    }

    #[test]
    fn cutting_before_feed_is_an_error() {
        assert!(matches!(
            parse_gcode("G1 X10\n", &GcodeOptions::default()),
            Err(GcodeError::FeedUndefined { line: 1 })
        ));
    }

    #[test]
    fn spindle_words_become_warnings_not_errors() {
        let parsed = parse_gcode(
            "S12000 M3\nG0 X1\nG1 X2 F100\nM5\nM30\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        assert_eq!(parsed.moves.len(), 2);
        assert!(parsed.warnings.iter().any(|w| w.contains('S')));
        assert!(parsed.warnings.iter().any(|w| w.contains("M3")));
    }

    #[test]
    fn inch_units_scale() {
        let parsed = parse_gcode("G20\nG0 X1\n", &GcodeOptions::default()).unwrap();
        let p = positions(&parsed.moves);
        assert!((p[0][0][0] - 0.0254).abs() < 1e-12);
    }

    #[test]
    fn comments_and_blank_lines_are_ignored() {
        let parsed = parse_gcode(
            "%\n( header )\nG0 X10 ; move over\n(G1 X99 F1)\n\nG1 X0 F100\n",
            &GcodeOptions::default(),
        )
        .unwrap();
        assert_eq!(parsed.moves.len(), 2);
        assert_eq!(parsed.moves[1].targets.len(), 1);
    }
}
