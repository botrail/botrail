//! The handshake specification: every line between controllers in a baked
//! cycle — direction, both ends, waveform, who writes it and who waits on
//! it — rendered as Markdown. Integrators keep this sheet (the robot ⇔ PLC
//! interface list) by hand in a spreadsheet; here it is a projection of the
//! same derivation the I/O table uses, over the timeline that actually
//! ran, so it can be regenerated per scenario and diffed.
//!
//! What counts as a line: a handshake signal (rule ② — written on one
//! host, read on another), a robot's start / done / program-number
//! handshake (rule ⑥ — a robot driven from another controller), and a
//! device's command / in-position lines (rule ⑤). Sensors and coils are
//! field wiring and stay in the I/O table.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::iomap::{self, Aspect, IoDirection, IoError, IoPoint, IoSource, StepRef};
use crate::rollout::{BoolTrack, SequenceTimeline, StepSpan};
use crate::Scene;

/// Intervals a robot was driven by a motion, ramp or toolpath — its moves
/// merged where they touch or overlap. `None` for a robot the timeline
/// does not carry. This is the "busy" contact a robot controller would
/// show a PLC, synthesized from the bake (the robot has no signal lane).
pub fn robot_busy(timeline: &SequenceTimeline, robot: &str) -> Option<Vec<(f64, f64)>> {
    let track = timeline.robots.iter().find(|r| r.name == robot)?;
    Some(merge_spans(track.moves.iter().map(|m| (m.start, m.end))))
}

/// Merges `(start, end)` intervals that touch or overlap (to float noise),
/// in time order.
pub fn merge_spans(spans: impl IntoIterator<Item = (f64, f64)>) -> Vec<(f64, f64)> {
    let mut spans: Vec<(f64, f64)> = spans.into_iter().collect();
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (s, e) in spans {
        match out.last_mut() {
            Some(last) if s <= last.1 + 1e-12 => last.1 = last.1.max(e),
            _ => out.push((s, e)),
        }
    }
    out
}

/// `[(start, end)]` intervals a lane was high; an open interval closes at
/// `duration`.
pub fn high_spans(track: &BoolTrack, duration: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut open: Option<f64> = None;
    for &(t, v) in &track.edges {
        match (open, v) {
            (None, true) => open = Some(t),
            (Some(t0), false) => {
                out.push((t0, t));
                open = None;
            }
            _ => {}
        }
    }
    if let Some(t0) = open {
        out.push((t0, duration.max(t0)));
    }
    out
}

/// One line of the specification: a wire (or a robot / device handshake)
/// between two controllers, with everything the sheet prints about it.
#[derive(Debug, Clone, PartialEq)]
pub struct HandshakeLine {
    /// The signal, robot or device name; `label` adds the aspect.
    pub name: String,
    pub label: String,
    pub kind: HandshakeKind,
    /// The writing (or commanding) host and its channel, if bound.
    pub from: HandshakeEnd,
    /// The reading hosts, each with its channel if bound.
    pub to: Vec<HandshakeEnd>,
    pub writers: Vec<StepRef>,
    pub readers: Vec<StepRef>,
    /// The waveform, when the line has one: high spans of the signal /
    /// device lane, or the synthesized robot busy spans.
    pub waveform: Option<Waveform>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum HandshakeKind {
    /// A handshake signal between controllers.
    Signal,
    /// A robot's start / done / program-number handshake.
    Robot { aspect: Aspect },
    /// A device's command or in-position line.
    Device {
        aspect: Option<Aspect>,
        source: IoSource,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct HandshakeEnd {
    /// The controller — a declared node name, or an implicit host such as
    /// `<cell>` / `<robot>`; the device or robot itself on the far side.
    pub host: String,
    /// `node.channel [address]` when the point is bound.
    pub channel: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Waveform {
    /// High spans of a lane.
    High(Vec<(f64, f64)>),
    /// Robot start pulses (issue times, with the issuing step) and busy spans.
    Robot {
        starts: Vec<(f64, String)>,
        busy: Vec<(f64, f64)>,
    },
    /// Command issue times with the issuing step (a numeric / indexed
    /// command has no lane).
    Commands(Vec<(f64, String)>),
}

/// The lines of the specification, derived over the timeline's programs.
pub fn handshake_lines(
    scene: &Scene,
    timeline: &SequenceTimeline,
) -> Result<Vec<HandshakeLine>, IoError> {
    let names: Vec<&str> = timeline.sequences.iter().map(String::as_str).collect();
    let d = iomap::derive(scene, Some(&names))?;
    let mut lines = Vec::new();

    // Entry times of a step, from the timeline (a step may run more than
    // once when a branch re-enters it — every span counts).
    let entries = |r: &StepRef| -> Vec<f64> {
        timeline
            .step_spans
            .iter()
            .filter(|s| s.sequence == r.sequence && s.step == r.index)
            .map(|s| s.start)
            .collect()
    };
    // `DI2 [%IX0.2]` on the host's own card, `RIO1.DI2 [%IX1.2]` on an
    // uplinked station.
    let end_of = |p: &IoPoint| {
        let host = p.host.clone().unwrap_or_else(|| "(unhosted)".to_string());
        let channel = d.binding_of(p).map(|(_, n, c)| {
            let id = if n.name == host {
                c.id.clone()
            } else {
                format!("{}.{}", n.name, c.id)
            };
            match &c.address {
                Some(a) => format!("{id} [{a}]"),
                None => id,
            }
        });
        HandshakeEnd { host, channel }
    };
    let dedup = |mut v: Vec<StepRef>| {
        v.sort_by(|a, b| (&a.sequence, a.index).cmp(&(&b.sequence, b.index)));
        v.dedup_by(|a, b| a.sequence == b.sequence && a.index == b.index);
        v
    };

    // ---- handshake signals: one Out point, N In points, one lane.
    let mut by_signal: BTreeMap<String, (Vec<&IoPoint>, Vec<&IoPoint>)> = BTreeMap::new();
    for p in d.points.iter().filter(|p| p.source == IoSource::Handshake) {
        let slot = by_signal.entry(p.id.name.clone()).or_default();
        match p.id.direction {
            IoDirection::Output => slot.0.push(p),
            IoDirection::Input => slot.1.push(p),
        }
    }
    for (name, (outs, ins)) in by_signal {
        let Some(out) = outs.first() else { continue };
        let lane = timeline.signals.iter().find(|s| s.name == name);
        lines.push(HandshakeLine {
            label: name.clone(),
            name,
            kind: HandshakeKind::Signal,
            from: end_of(out),
            to: ins.iter().map(|p| end_of(p)).collect(),
            writers: dedup(out.writers.clone()),
            readers: dedup(ins.iter().flat_map(|p| p.readers.clone()).collect()),
            waveform: lane.map(|l| Waveform::High(high_spans(l, timeline.duration))),
        });
    }

    // ---- robot handshakes: start / done / program per (robot, aspect);
    // the driving host's point first, the controller's mirror on the far
    // side when declared.
    let mut by_robot: BTreeMap<(String, u8), Vec<&IoPoint>> = BTreeMap::new();
    for p in d.points.iter().filter(|p| {
        matches!(
            p.source,
            IoSource::RobotStart | IoSource::RobotDone | IoSource::RobotProgram
        )
    }) {
        let order = match p.id.aspect {
            Some(Aspect::Start) => 0,
            Some(Aspect::Done) => 1,
            _ => 2,
        };
        by_robot
            .entry((p.id.name.clone(), order))
            .or_default()
            .push(p);
    }
    for ((robot, _), points) in by_robot {
        let aspect = points[0].id.aspect.unwrap_or(Aspect::Start);
        // The driving side is the host that carries the point in its
        // natural direction (start: Out, done: In, program: Out); the
        // mirror is the other direction on the robot's controller.
        let natural = match aspect {
            Aspect::Done => IoDirection::Input,
            _ => IoDirection::Output,
        };
        let Some(driving) = points.iter().find(|p| p.id.direction == natural) else {
            continue;
        };
        let mirrors: Vec<&&IoPoint> = points
            .iter()
            .filter(|p| p.id.direction != natural)
            .collect();
        let track = timeline.robots.iter().find(|r| r.name == robot);
        let far_host = mirrors
            .first()
            .and_then(|p| p.host.clone())
            .unwrap_or_else(|| format!("<{robot}>"));
        let (from, to) = match aspect {
            // start / program flow driving host → robot controller.
            Aspect::Start | Aspect::Program => (
                end_of(driving),
                vec![mirrors.first().map(|p| end_of(p)).unwrap_or(HandshakeEnd {
                    host: far_host.clone(),
                    channel: None,
                })],
            ),
            // done flows robot controller → driving host.
            _ => (
                mirrors.first().map(|p| end_of(p)).unwrap_or(HandshakeEnd {
                    host: far_host.clone(),
                    channel: None,
                }),
                vec![end_of(driving)],
            ),
        };
        let waveform = track.map(|t| {
            let starts: Vec<(f64, String)> = t
                .moves
                .iter()
                .map(|m: &StepSpan| (m.start, step_name(timeline, m)))
                .collect();
            match aspect {
                Aspect::Program => {
                    Waveform::Commands(t.moves.iter().map(|m| (m.start, m.name.clone())).collect())
                }
                _ => Waveform::Robot {
                    starts,
                    busy: merge_spans(t.moves.iter().map(|m| (m.start, m.end))),
                },
            }
        });
        lines.push(HandshakeLine {
            label: driving.id.label(),
            name: robot.clone(),
            kind: HandshakeKind::Robot { aspect },
            from,
            to,
            writers: dedup(points.iter().flat_map(|p| p.writers.clone()).collect()),
            readers: dedup(points.iter().flat_map(|p| p.readers.clone()).collect()),
            waveform,
        });
    }

    // ---- device lines: run / command (host → device) and in-position
    // (device → host). The device is the far end.
    for p in d.points.iter().filter(|p| {
        matches!(p.source, IoSource::DeviceRun | IoSource::DeviceCommand | IoSource::DeviceDone)
            // A belt that runs from t = 0 and is never commanded is wired
            // on, not handshaken; a device no program touches has no line.
            && p.status != iomap::PointStatus::Constant
            && p.host.is_some()
    }) {
        let device_end = HandshakeEnd {
            host: p.id.name.clone(),
            channel: None,
        };
        let (from, to) = match p.id.direction {
            IoDirection::Output => (end_of(p), vec![device_end]),
            IoDirection::Input => (device_end, vec![end_of(p)]),
        };
        let waveform = match p.source {
            IoSource::DeviceRun => timeline
                .signals
                .iter()
                .find(|s| s.name == p.id.name)
                .map(|l| Waveform::High(high_spans(l, timeline.duration))),
            IoSource::DeviceCommand => Some(Waveform::Commands(
                p.writers
                    .iter()
                    .flat_map(|w| entries(w).into_iter().map(move |t| (t, w.to_string())))
                    .collect(),
            )),
            _ => None,
        };
        lines.push(HandshakeLine {
            label: p.id.label(),
            name: p.id.name.clone(),
            kind: HandshakeKind::Device {
                aspect: p.id.aspect,
                source: p.source,
            },
            from,
            to,
            writers: dedup(p.writers.clone()),
            readers: dedup(p.readers.clone()),
            waveform,
        });
    }
    Ok(lines)
}

fn step_name(timeline: &SequenceTimeline, m: &StepSpan) -> String {
    timeline
        .step_spans
        .iter()
        .find(|s| s.sequence == m.sequence && s.step == m.step)
        .map(|s| s.name.clone())
        .unwrap_or_else(|| m.name.clone())
}

fn fmt_spans(spans: &[(f64, f64)]) -> String {
    if spans.is_empty() {
        return "never".to_string();
    }
    let mut shown: Vec<String> = spans
        .iter()
        .take(SPAN_LIST_CAP)
        .map(|(a, b)| format!("{a:.3}–{b:.3}"))
        .collect();
    if spans.len() > SPAN_LIST_CAP {
        shown.push(format!("… (+{} more)", spans.len() - SPAN_LIST_CAP));
    }
    shown.join(", ")
}

const SPAN_LIST_CAP: usize = 24;

/// Step lists are capped in the sheet (a two-arm weld station issues
/// fifty starts a cycle); the I/O table carries every writer and reader.
const STEP_LIST_CAP: usize = 12;

fn fmt_steps(steps: &[StepRef]) -> String {
    if steps.is_empty() {
        return "—".to_string();
    }
    let mut shown: Vec<String> = steps
        .iter()
        .take(STEP_LIST_CAP)
        .map(|s| format!("`{s}`"))
        .collect();
    if steps.len() > STEP_LIST_CAP {
        shown.push(format!("… (+{} more)", steps.len() - STEP_LIST_CAP));
    }
    shown.join(", ")
}

fn fmt_events(events: &[(f64, String)]) -> String {
    if events.is_empty() {
        return "never".to_string();
    }
    let mut shown: Vec<String> = events
        .iter()
        .take(STEP_LIST_CAP)
        .map(|(t, s)| format!("{t:.3} (`{s}`)"))
        .collect();
    if events.len() > STEP_LIST_CAP {
        shown.push(format!("… (+{} more)", events.len() - STEP_LIST_CAP));
    }
    format!("{} s", shown.join(", "))
}

fn fmt_end(e: &HandshakeEnd) -> String {
    match &e.channel {
        Some(c) => format!("{} · {c}", e.host),
        None => e.host.clone(),
    }
}

/// The specification as Markdown: a summary table, then one block per line.
pub fn render_handshake_spec(
    scene: &Scene,
    timeline: &SequenceTimeline,
) -> Result<String, IoError> {
    let lines = handshake_lines(scene, timeline)?;
    let mut out = String::new();
    let programs = timeline.sequences.join(" + ");
    let scenario = timeline.scenario.as_deref().unwrap_or("baseline");
    let _ = writeln!(out, "# Handshake spec — {programs} ({scenario})\n");
    let signals = lines
        .iter()
        .filter(|l| l.kind == HandshakeKind::Signal)
        .count();
    let robots = lines
        .iter()
        .filter(|l| matches!(l.kind, HandshakeKind::Robot { .. }))
        .count();
    let devices = lines
        .iter()
        .filter(|l| matches!(l.kind, HandshakeKind::Device { .. }))
        .count();
    let _ = writeln!(
        out,
        "Cycle {:.2} s. {signals} handshake signal(s), {robots} robot line(s), {devices} device line(s).\n",
        timeline.duration
    );
    if lines.is_empty() {
        let _ = writeln!(
            out,
            "No lines: every program runs on the controller that owns what it drives, and \
             no device is commanded. Declare nodes (`add_io_node(programs=...)`) to split \
             the cell across controllers."
        );
        return Ok(out);
    }
    let _ = writeln!(
        out,
        "| line | kind | from | to | writers | readers | activity |"
    );
    let _ = writeln!(out, "|---|---|---|---|---|---|---|");
    for l in &lines {
        let kind = match &l.kind {
            HandshakeKind::Signal => "signal".to_string(),
            HandshakeKind::Robot { aspect } => format!("robot {}", aspect.as_str()),
            HandshakeKind::Device { source, .. } => source.as_str().to_string(),
        };
        let activity = match &l.waveform {
            Some(Waveform::High(spans)) => format!(
                "{} pulse(s), {:.2} s high",
                spans.len(),
                spans.iter().map(|(a, b)| b - a).sum::<f64>()
            ),
            Some(Waveform::Robot { starts, busy }) => match &l.kind {
                HandshakeKind::Robot {
                    aspect: Aspect::Done,
                } => format!(
                    "idle {:.2} s",
                    timeline.duration - busy.iter().map(|(a, b)| b - a).sum::<f64>()
                ),
                _ => format!(
                    "{} start(s), busy {:.2} s",
                    starts.len(),
                    busy.iter().map(|(a, b)| b - a).sum::<f64>()
                ),
            },
            Some(Waveform::Commands(cmds)) => format!("{} command(s)", cmds.len()),
            None => "—".to_string(),
        };
        let _ = writeln!(
            out,
            "| `{}` | {kind} | {} | {} | {} | {} | {activity} |",
            l.label,
            fmt_end(&l.from),
            l.to.iter().map(fmt_end).collect::<Vec<_>>().join("; "),
            l.writers.len(),
            l.readers.len(),
        );
    }
    for l in &lines {
        let _ = writeln!(
            out,
            "\n## `{}` — {} → {}\n",
            l.label,
            l.from.host,
            l.to.iter()
                .map(|e| e.host.clone())
                .collect::<Vec<_>>()
                .join(", ")
        );
        let _ = writeln!(out, "| | |\n|---|---|");
        let _ = writeln!(out, "| from | {} |", fmt_end(&l.from));
        let _ = writeln!(
            out,
            "| to | {} |",
            l.to.iter().map(fmt_end).collect::<Vec<_>>().join("; ")
        );
        // Who drives and who waits: a robot start / program line is
        // issued by the driving steps (nobody "waits" on it in the
        // program), a done line is what those steps wait on, an
        // in-position input is only waited on.
        match &l.kind {
            HandshakeKind::Robot {
                aspect: Aspect::Done,
            }
            | HandshakeKind::Device {
                source: IoSource::DeviceDone,
                ..
            } => {
                let _ = writeln!(out, "| waited by | {} |", fmt_steps(&l.readers));
            }
            HandshakeKind::Robot { .. } => {
                let _ = writeln!(out, "| issued by | {} |", fmt_steps(&l.writers));
            }
            _ => {
                let _ = writeln!(out, "| written by | {} |", fmt_steps(&l.writers));
                let _ = writeln!(out, "| waited by | {} |", fmt_steps(&l.readers));
            }
        }
        match &l.waveform {
            Some(Waveform::High(spans)) => {
                let _ = writeln!(out, "| high | {} s |", fmt_spans(spans));
            }
            Some(Waveform::Robot { starts, busy }) => {
                if matches!(
                    l.kind,
                    HandshakeKind::Robot {
                        aspect: Aspect::Done
                    }
                ) {
                    let _ = writeln!(out, "| busy | {} s |", fmt_spans(busy));
                    let idle = idle_spans(busy, timeline.duration);
                    let _ = writeln!(out, "| done (idle) | {} s |", fmt_spans(&idle));
                } else {
                    let _ = writeln!(out, "| start | {} |", fmt_events(starts));
                    let _ = writeln!(out, "| busy | {} s |", fmt_spans(busy));
                }
            }
            Some(Waveform::Commands(cmds)) => {
                let _ = writeln!(out, "| commanded | {} |", fmt_events(cmds));
            }
            None => {
                let _ = writeln!(
                    out,
                    "| waveform | — (no lane: the in-position input follows the device) |"
                );
            }
        }
    }
    Ok(out)
}

fn idle_spans(busy: &[(f64, f64)], duration: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut t = 0.0;
    for (a, b) in busy {
        if *a > t {
            out.push((t, *a));
        }
        t = t.max(*b);
    }
    if duration > t {
        out.push((t, duration));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::iomap::tests::{node, two_arm_cell, ur_channels};
    use crate::iomap::{IoNodeKind, Uplink};
    use crate::rollout::RolloutOptions;
    use nalgebra::Isometry3;

    /// A PLC runs the belt program, the two-arm pick stays on `<cell>`:
    /// two handshake signals cross, both arms get start / done lines, the
    /// belt gets a run line — each with the bake's waveform.
    #[test]
    fn spec_lists_signal_robot_and_device_lines_with_waveforms() {
        let mut scene = two_arm_cell();
        let mut plc = node("PLC1", IoNodeKind::Plc, &["belt"], ur_channels());
        plc.channels[0].address = Some("%IX0.0".into());
        scene.upsert_io_node(plc).unwrap();
        let tl = scene
            .simulate_sequences(&["pick", "belt"], &RolloutOptions::default())
            .unwrap();
        let lines = handshake_lines(&scene, &tl).unwrap();
        let labels: Vec<&str> = lines.iter().map(|l| l.label.as_str()).collect();
        assert_eq!(
            labels,
            [
                "belt_ok",
                "carrying",
                "far.start",
                "far.done",
                "far.program",
                "r.start",
                "r.done",
                "belt"
            ],
            "{labels:?}"
        );
        // belt_ok: PLC1 → <cell>, written by belt/run, waited by pick/carry.
        let belt_ok = &lines[0];
        assert_eq!(belt_ok.kind, HandshakeKind::Signal);
        assert_eq!(belt_ok.from.host, "PLC1");
        assert_eq!(
            belt_ok
                .to
                .iter()
                .map(|e| e.host.as_str())
                .collect::<Vec<_>>(),
            ["<cell>"]
        );
        assert_eq!(
            belt_ok
                .writers
                .iter()
                .map(|s| s.to_string())
                .collect::<Vec<_>>(),
            ["belt/run"]
        );
        assert_eq!(
            belt_ok
                .readers
                .iter()
                .map(|s| s.to_string())
                .collect::<Vec<_>>(),
            ["pick/carry"]
        );
        // Its waveform is the lane: set in the first scan, high to the end.
        assert_eq!(
            belt_ok.waveform,
            Some(Waveform::High(vec![(0.0, tl.duration)]))
        );
        // carrying goes the other way, and only rises at `carry`.
        let carrying = &lines[1];
        assert_eq!(
            (carrying.from.host.as_str(), carrying.to[0].host.as_str()),
            ("<cell>", "PLC1")
        );
        let carry_start = tl
            .step_spans
            .iter()
            .find(|s| s.sequence == "pick" && s.name == "pick/carry")
            .unwrap()
            .start;
        assert_eq!(
            carrying.waveform,
            Some(Waveform::High(vec![(carry_start, tl.duration)]))
        );
        // Robot lines: start flows <cell> → the arm's (implicit) controller,
        // done flows back; the waveform is synthesized from the moves.
        let far_start = &lines[2];
        assert_eq!(
            far_start.kind,
            HandshakeKind::Robot {
                aspect: Aspect::Start
            }
        );
        assert_eq!(
            (far_start.from.host.as_str(), far_start.to[0].host.as_str()),
            ("<cell>", "<far>")
        );
        let far_done = &lines[3];
        assert_eq!(
            (far_done.from.host.as_str(), far_done.to[0].host.as_str()),
            ("<far>", "<cell>")
        );
        let Some(Waveform::Robot { starts, busy }) = &far_start.waveform else {
            panic!("robot waveform");
        };
        assert_eq!(starts.len(), 2, "far_go then far_back");
        assert_eq!(starts[0].1, "pick/go");
        // far_back starts on the scan after far_go ends — a sub-tick gap,
        // so two busy spans, and the same seconds `busy_seconds` reports.
        assert_eq!(busy.len(), 2);
        assert!(busy[1].0 - busy[0].1 < 0.011 && busy[1].0 > busy[0].1);
        assert_eq!(robot_busy(&tl, "far").unwrap(), *busy);
        assert_eq!(robot_busy(&tl, "nobody"), None);
        let seconds: f64 = busy.iter().map(|(a, b)| b - a).sum();
        assert!((tl.busy_seconds("far").unwrap() - seconds).abs() < 1e-12);
        // The program word carries the motion names, in firing order.
        let Some(Waveform::Commands(cmds)) = &lines[4].waveform else {
            panic!("program waveform");
        };
        assert_eq!(
            cmds.iter().map(|(_, m)| m.as_str()).collect::<Vec<_>>(),
            ["far_go", "far_back"]
        );
        // The device line: PLC1 → belt, run lane high from the first scan.
        let belt = &lines[7];
        assert!(matches!(
            belt.kind,
            HandshakeKind::Device {
                source: IoSource::DeviceRun,
                ..
            }
        ));
        assert_eq!(belt.from.channel, None, "not bound yet");
        assert_eq!(belt.to[0].host, "belt");

        // Bind on the PLC and on an uplinked station: the ends name the
        // channel — bare on the host's own card, `RIO1.…` on the station.
        scene.auto_assign_io(None, false).unwrap();
        let mut rio = node("RIO1", IoNodeKind::RemoteIo, &[], vec![]);
        rio.uplink = Some(Uplink {
            parent: "PLC1".into(),
            bus: Some("PROFINET".into()),
        });
        rio.channels = vec![crate::iomap::IoChannel {
            id: "DI0".into(),
            kind: crate::iomap::ChannelKind::Di,
            port: None,
            address: Some("%IX1.0".into()),
            electrical: None,
        }];
        scene.upsert_io_node(rio).unwrap();
        scene
            .unbind_io(
                &crate::iomap::IoPointId::parse("carrying", IoDirection::Input),
                Some("PLC1"),
            )
            .unwrap();
        scene
            .bind_io(crate::iomap::IoBinding {
                point: crate::iomap::IoPointId::parse("carrying", IoDirection::Input),
                node: "RIO1".into(),
                channel: "DI0".into(),
                tag: None,
                field: None,
                invert: false,
                contact: None,
                safety: false,
                device: None,
                note: None,
                auto: false,
            })
            .unwrap();
        let lines = handshake_lines(&scene, &tl).unwrap();
        assert_eq!(
            lines[0].from.channel.as_deref(),
            Some("DO1"),
            "belt took DO0 first"
        );
        assert_eq!(lines[1].to[0].channel.as_deref(), Some("RIO1.DI0 [%IX1.0]"));

        let md = render_handshake_spec(&scene, &tl).unwrap();
        assert!(
            md.starts_with("# Handshake spec — pick + belt (baseline)\n"),
            "{md}"
        );
        assert!(
            md.contains("2 handshake signal(s), 5 robot line(s), 1 device line(s)."),
            "{md}"
        );
        assert!(
            md.contains(
                "| `carrying` | signal | <cell> | PLC1 · RIO1.DI0 [%IX1.0] | 1 | 1 | 1 pulse(s),"
            ),
            "{md}"
        );
        assert!(md.contains("## `far.start` — <cell> → <far>\n"), "{md}");
        assert!(
            md.contains("| issued by | `pick/go`, `pick/carry` |"),
            "{md}"
        );
        assert!(md.contains("| done (idle) |"), "{md}");
        assert!(
            md.contains("| written by | `belt/run` |\n| waited by | `pick/carry` |"),
            "{md}"
        );
    }

    /// One robot running its own program on its own controller: nothing
    /// crosses, and the sheet says so. Commanding a device adds the one
    /// device line; sensors and coils stay field wiring.
    #[test]
    fn a_single_controller_cell_has_no_lines() {
        use crate::rollout::tests::{joint_motion, sample_scene};
        use crate::seq::{Action, Condition, DeviceCommand, Sequence, Step};
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.5);
        scene.define_signal("vacuum", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![Step {
                name: "go".into(),
                actions: vec![
                    Action::StartMotion {
                        motion: "go".into(),
                    },
                    Action::Set {
                        signal: "vacuum".into(),
                        value: true,
                    },
                ],
                transition: Condition::Done,
                select: vec![],
            }],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert!(handshake_lines(&scene, &tl).unwrap().is_empty());
        let md = render_handshake_spec(&scene, &tl).unwrap();
        assert!(
            md.contains("No lines: every program runs on the controller"),
            "{md}"
        );

        scene.upsert_device(crate::seq::Device {
            name: "belt".into(),
            kind: crate::seq::DeviceKind::Conveyor {
                zone_pose: Isometry3::translation(5.0, 0.0, 0.0),
                zone_size: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                velocity: nalgebra::Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![Step {
                name: "go".into(),
                actions: vec![
                    Action::StartMotion {
                        motion: "go".into(),
                    },
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Start,
                    },
                ],
                transition: Condition::Done,
                select: vec![],
            }],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let lines = handshake_lines(&scene, &tl).unwrap();
        assert_eq!(
            lines.iter().map(|l| l.label.as_str()).collect::<Vec<_>>(),
            ["belt"]
        );
        assert_eq!(lines[0].from.host, "<r>");
        assert_eq!(
            lines[0].waveform,
            Some(Waveform::High(vec![(0.0, tl.duration)]))
        );
    }

    #[test]
    fn merge_and_high_spans() {
        assert_eq!(
            merge_spans([(1.0, 2.0), (0.0, 1.0), (3.0, 4.0), (3.5, 5.0)]),
            [(0.0, 2.0), (3.0, 5.0)]
        );
        assert!(merge_spans(std::iter::empty()).is_empty());
        let lane = BoolTrack {
            name: "x".into(),
            edges: vec![(0.0, false), (1.0, true), (2.0, false), (3.0, true)],
            kind: crate::rollout::LaneKind::Signal,
        };
        assert_eq!(high_spans(&lane, 4.0), [(1.0, 2.0), (3.0, 4.0)]);
        assert_eq!(idle_spans(&[(1.0, 2.0)], 4.0), [(0.0, 1.0), (2.0, 4.0)]);
    }
}
