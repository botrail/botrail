//! APT / CL-data subset parser → [`crate::toolpath::ToolMove`] list — the
//! 5-axis entry format (see `design/design-machining.md` §3.1): CAM
//! cutter-location output carries the tool axis as a plain `i,j,k` vector
//! per point, machine-independent, where 5-axis G-code would need the
//! machine's kinematics to interpret its ABC angles.
//!
//! Scope: `GOTO/x,y,z[,i,j,k]`, `FROM/…` (the initial position),
//! `RAPID` (arms the *next* GOTO), `FEDRAT/[MMPM,]f`, `UNITS/MM|INCHES`,
//! `MULTAX/ON|OFF` (informational), `$`-continuation lines, `$$`
//! comments, `FINI`. Harmless process words (`SPINDL`, `COOLNT`,
//! `PPRINT`, `PARTNO`, `MACHIN`, first `LOADTL`) are collected as
//! warnings or ignored; anything that would change the path's meaning —
//! a second `LOADTL` (tool change mid-path), `CUTCOM` (compensation),
//! `CIRCLE` records (posts that emit them have not expanded arcs),
//! `GOHOME` — is a line-numbered error, never a silent skip.

use nalgebra::{Point3, Unit, Vector3};
use thiserror::Error;

use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind};

#[derive(Debug, Error)]
pub enum AptError {
    #[error("line {line}: {message}")]
    Unsupported { line: usize, message: String },
    #[error("line {line}: malformed record `{record}`")]
    Malformed { line: usize, record: String },
    #[error("line {line}: GOTO before any F word set a feed (add FEDRAT or RAPID)")]
    FeedUndefined { line: usize },
    #[error("line {line}: tool-axis vector is zero")]
    ZeroAxis { line: usize },
    #[error("program contains no motion")]
    Empty,
}

/// Parse result: the moves plus non-fatal notes.
#[derive(Debug, Clone)]
pub struct ParsedApt {
    pub moves: Vec<ToolMove>,
    pub warnings: Vec<String>,
}

pub fn parse_apt(text: &str) -> Result<ParsedApt, AptError> {
    let mut moves: Vec<ToolMove> = Vec::new();
    let mut warnings: Vec<String> = Vec::new();
    let mut scale = 1e-3; // millimeters unless UNITS says otherwise
    let mut feed: Option<f64> = None;
    let mut rapid_armed = false;
    let mut loadtl_seen = false;

    // Join `$` continuation lines, remembering each statement's first
    // line number for error messages.
    let mut statements: Vec<(usize, String)> = Vec::new();
    let mut pending: Option<(usize, String)> = None;
    for (lineno, raw) in text.lines().enumerate() {
        let line = lineno + 1;
        // `$$` starts a comment (to end of line); a bare trailing `$`
        // continues the statement on the next line.
        let without_comment = match raw.find("$$") {
            Some(i) => &raw[..i],
            None => raw,
        };
        let trimmed = without_comment.trim();
        if trimmed.is_empty() {
            continue;
        }
        let (fragment, continues) = match trimmed.strip_suffix('$') {
            Some(head) => (head.trim_end(), true),
            None => (trimmed, false),
        };
        match pending.take() {
            Some((first, mut acc)) => {
                acc.push_str(fragment);
                if continues {
                    pending = Some((first, acc));
                } else {
                    statements.push((first, acc));
                }
            }
            None => {
                if continues {
                    pending = Some((line, fragment.to_string()));
                } else {
                    statements.push((line, fragment.to_string()));
                }
            }
        }
    }
    if let Some((line, record)) = pending {
        return Err(AptError::Malformed { line, record });
    }

    let mut push_target = |kind: ToolMoveKind, target: PathTarget| {
        match moves.last_mut() {
            Some(last) if last.kind == kind => last.targets.push(target),
            _ => moves.push(ToolMove {
                kind,
                targets: vec![target],
            }),
        }
    };

    'statements: for (line, statement) in statements {
        let (word, args) = match statement.split_once('/') {
            Some((w, a)) => (w.trim().to_ascii_uppercase(), a.trim()),
            None => (statement.trim().to_ascii_uppercase(), ""),
        };
        match word.as_str() {
            "GOTO" | "FROM" => {
                let values: Vec<f64> = args
                    .split(',')
                    .map(|v| v.trim().parse::<f64>())
                    .collect::<Result<_, _>>()
                    .map_err(|_| AptError::Malformed {
                        line,
                        record: statement.clone(),
                    })?;
                let (position, axis) = match values.len() {
                    3 => (
                        Point3::new(values[0], values[1], values[2]) * scale,
                        Vector3::z(),
                    ),
                    6 => (
                        Point3::new(values[0], values[1], values[2]) * scale,
                        Vector3::new(values[3], values[4], values[5]),
                    ),
                    _ => {
                        return Err(AptError::Malformed {
                            line,
                            record: statement.clone(),
                        })
                    }
                };
                if axis.norm() < 1e-9 {
                    return Err(AptError::ZeroAxis { line });
                }
                let target = PathTarget {
                    position,
                    tool_axis: Unit::new_normalize(axis),
                    spin: None,
                };
                // FROM and a RAPID-armed GOTO are positioning moves; a
                // plain GOTO cuts at the current feed.
                if word == "FROM" || rapid_armed {
                    push_target(ToolMoveKind::Rapid, target);
                    rapid_armed = false;
                } else {
                    let f = feed.ok_or(AptError::FeedUndefined { line })?;
                    push_target(ToolMoveKind::Feed(f), target);
                }
            }
            "RAPID" => rapid_armed = true,
            "FEDRAT" => {
                // FEDRAT/300 or FEDRAT/MMPM,300 (per-minute); IPM under
                // UNITS/INCHES. Per-revolution has no spindle model here.
                let mut parts = args.split(',').map(str::trim);
                let first = parts.next().unwrap_or("");
                let value = match first.to_ascii_uppercase().as_str() {
                    "MMPM" | "IPM" => parts.next().unwrap_or(""),
                    "MMPR" | "IPR" => {
                        return Err(AptError::Unsupported {
                            line,
                            message: "FEDRAT per revolution is not supported (per-minute only)"
                                .to_string(),
                        })
                    }
                    _ => first,
                };
                let value: f64 = value.parse().map_err(|_| AptError::Malformed {
                    line,
                    record: statement.clone(),
                })?;
                feed = Some(value * scale / 60.0);
            }
            "UNITS" => {
                scale = match args.to_ascii_uppercase().as_str() {
                    "MM" => 1e-3,
                    "INCHES" | "INCH" => 0.0254,
                    other => {
                        return Err(AptError::Unsupported {
                            line,
                            message: format!("UNITS/{other} is not supported"),
                        })
                    }
                };
            }
            "LOADTL" => {
                if loadtl_seen {
                    return Err(AptError::Unsupported {
                        line,
                        message: "second LOADTL: tool changes mid-path are not supported"
                            .to_string(),
                    });
                }
                loadtl_seen = true;
                warnings.push(format!("line {line}: LOADTL/{args} noted (single tool)"));
            }
            "SPINDL" | "COOLNT" => {
                warnings.push(format!("line {line}: {word}/{args} ignored"));
            }
            "MULTAX" | "PARTNO" | "MACHIN" | "PPRINT" | "END" => {}
            "FINI" => break 'statements,
            "CUTCOM" => {
                return Err(AptError::Unsupported {
                    line,
                    message: "CUTCOM: cutter compensation is not supported — post the \
                              toolpath pre-compensated"
                        .to_string(),
                });
            }
            "CIRCLE" => {
                return Err(AptError::Unsupported {
                    line,
                    message: "CIRCLE records are not supported — have the post expand \
                              arcs to GOTO points"
                        .to_string(),
                });
            }
            "GOHOME" => {
                return Err(AptError::Unsupported {
                    line,
                    message: "GOHOME: machine-home moves are not supported".to_string(),
                });
            }
            other => {
                return Err(AptError::Unsupported {
                    line,
                    message: format!("`{other}` is not supported"),
                });
            }
        }
    }

    if moves.iter().all(|m| m.targets.is_empty()) {
        return Err(AptError::Empty);
    }
    Ok(ParsedApt { moves, warnings })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn five_axis_program_parses_with_ijk() {
        let parsed = parse_apt(
            "PARTNO / DEMO\nMULTAX/ON\nUNITS/MM\nLOADTL/1\nSPINDL/RPM,18000,CLW\n\
             FROM/0,0,50\nRAPID\nGOTO/10,0,5,0,0,1\nFEDRAT/MMPM,600\n\
             GOTO/20,0,0,0.1736,0,0.9848\nGOTO/30,0,0,0.3420,0,0.9397\nFINI\n",
        )
        .unwrap();
        assert_eq!(parsed.moves.len(), 2);
        assert!(matches!(parsed.moves[0].kind, ToolMoveKind::Rapid));
        // FROM + the armed GOTO merged into one rapid move.
        assert_eq!(parsed.moves[0].targets.len(), 2);
        let ToolMoveKind::Feed(feed) = parsed.moves[1].kind else {
            panic!("expected feed");
        };
        assert!((feed - 0.01).abs() < 1e-12, "600mm/min = 10mm/s, got {feed}");
        let t = &parsed.moves[1].targets[0];
        assert!((t.position.coords - Vector3::new(0.020, 0.0, 0.0)).norm() < 1e-12);
        let axis = t.tool_axis.into_inner();
        assert!((axis - Vector3::new(0.1736, 0.0, 0.9848).normalize()).norm() < 1e-6);
        assert!(parsed.warnings.iter().any(|w| w.contains("SPINDL")));
        assert!(parsed.warnings.iter().any(|w| w.contains("LOADTL")));
    }

    #[test]
    fn continuation_lines_join() {
        let parsed = parse_apt(
            "FEDRAT/300\nGOTO/10,0,0, $\n  0,0,1\nGOTO/20,0, $$ trailing comment\n",
        );
        // The second GOTO lost its z to the comment strip: malformed.
        assert!(parsed.is_err());
        let parsed = parse_apt("FEDRAT/300\nGOTO/10,0,0, $\n  0,0,1\n").unwrap();
        assert_eq!(parsed.moves[0].targets.len(), 1);
        assert_eq!(
            parsed.moves[0].targets[0].tool_axis.into_inner(),
            Vector3::z()
        );
    }

    #[test]
    fn rapid_arms_exactly_one_goto() {
        let parsed = parse_apt(
            "FEDRAT/600\nRAPID\nGOTO/0,0,10\nGOTO/10,0,0\n",
        )
        .unwrap();
        assert!(matches!(parsed.moves[0].kind, ToolMoveKind::Rapid));
        assert!(matches!(parsed.moves[1].kind, ToolMoveKind::Feed(_)));
    }

    #[test]
    fn dangerous_records_are_line_numbered_errors() {
        for (src, needle) in [
            ("LOADTL/1\nFEDRAT/300\nGOTO/0,0,0\nLOADTL/2\n", "LOADTL"),
            ("CUTCOM/LEFT\n", "CUTCOM"),
            ("CIRCLE/0,0,0,0,0,1,10\n", "CIRCLE"),
            ("GOHOME\n", "GOHOME"),
            ("FEDRAT/MMPR,0.2\n", "per revolution"),
            ("GOTO/1,2,3\n", "FEDRAT"),
        ] {
            let err = parse_apt(src).unwrap_err();
            let msg = err.to_string();
            assert!(msg.contains("line"), "no line number in `{msg}`");
            assert!(
                msg.to_lowercase().contains(&needle.to_lowercase()),
                "`{msg}` does not mention {needle}"
            );
        }
    }

    #[test]
    fn inches_scale_positions_and_feed() {
        let parsed = parse_apt("UNITS/INCHES\nFEDRAT/60\nRAPID\nGOTO/1,0,0\n").unwrap();
        let t = &parsed.moves[0].targets[0];
        assert!((t.position.x - 0.0254).abs() < 1e-12);
        // 60 in/min = 1 in/s.
        let parsed = parse_apt("UNITS/INCHES\nFEDRAT/60\nGOTO/1,0,0\n").unwrap();
        let ToolMoveKind::Feed(feed) = parsed.moves[0].kind else {
            panic!()
        };
        assert!((feed - 0.0254).abs() < 1e-12);
    }
}
