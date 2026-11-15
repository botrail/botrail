//! Words and analog inputs (design-plc-resolution.md PF2): registers a
//! program compares and writes, and the 4–20 mA chain that turns a
//! measured word into the counts a controller reads.

use super::*;
use crate::iomap::{IoDirection, IoNode, IoNodeKind};
use crate::rollout::tests::sample_scene;
use crate::seq::{
    Analog, ArithOp, CompareOp, Fault, FaultKind, Logic, Operand, Rung, RungOutput, Scenario,
    Sequence, Step, WordExpr,
};

fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
    Step {
        name: name.into(),
        actions,
        transition,
        select: Vec::new(),
    }
}

fn sig(name: &str) -> Condition {
    Condition::Signal {
        name: name.into(),
        value: true,
    }
}

fn w(name: &str) -> Operand {
    Operand::Word(name.into())
}

fn k(x: f64) -> Operand {
    Operand::Const(x)
}

fn cmp(left: Operand, op: CompareOp, right: Operand) -> Condition {
    Condition::Compare { left, op, right }
}

fn calc(left: Operand, op: ArithOp, right: Operand) -> WordExpr {
    WordExpr {
        left,
        op: Some((op, right)),
    }
}

fn rung(comment: &str, condition: Condition, outputs: Vec<RungOutput>) -> Rung {
    Rung {
        comment: comment.into(),
        condition,
        outputs,
    }
}

fn coil(target: &str) -> RungOutput {
    RungOutput::Coil {
        target: target.into(),
    }
}

fn mov(word: &str, value: WordExpr) -> RungOutput {
    RungOutput::Move {
        word: word.into(),
        value,
    }
}

/// A filter whose differential pressure climbs while the pump runs: the
/// plant's own rung integrates it (`process`, on no controller), a
/// transmitter sends it to the PLC, whose rungs compare it with two
/// setpoints (`ctl`). An operator's script presses start.
fn filter() -> Scene {
    let mut scene = sample_scene();
    for s in ["pb_start", "pump", "warn", "alarm"] {
        scene.define_signal(s, false);
    }
    scene.define_word("dp_true", 10.0, false).unwrap();
    scene.define_word("dp_warn", 60.0, false).unwrap();
    scene.define_word("dp_limit", 80.0, false).unwrap();
    scene
        .upsert_analog(Analog {
            name: "dp".into(),
            source: "dp_true".into(),
            range: (0.0, 100.0),
            unit: "kPa".into(),
            raw: (0, 4000),
            place: None,
        })
        .unwrap();
    scene.upsert_logic(Logic {
        name: "ctl".into(),
        rungs: vec![
            rung(
                "pump",
                Condition::All(vec![
                    Condition::Any(vec![sig("pb_start"), sig("pump")]),
                    Condition::Not(Box::new(sig("alarm"))),
                ]),
                vec![coil("pump")],
            ),
            rung(
                "change the element soon",
                cmp(w("dp"), CompareOp::Ge, w("dp_warn")),
                vec![coil("warn")],
            ),
            rung(
                "clogged",
                cmp(w("dp"), CompareOp::Ge, w("dp_limit")),
                vec![RungOutput::Set {
                    signal: "alarm".into(),
                }],
            ),
        ],
        ..Default::default()
    });
    scene.upsert_logic(Logic {
        name: "process".into(),
        rungs: vec![rung(
            "the element clogs while it filters",
            sig("pump"),
            vec![mov("dp_true", calc(w("dp_true"), ArithOp::Add, k(0.5)))],
        )],
        ..Default::default()
    });
    scene.upsert_sequence(Sequence {
        name: "operator".into(),
        steps: vec![
            step("idle", vec![], Condition::Elapsed { seconds: 0.1 }),
            step(
                "press",
                vec![Action::Set {
                    signal: "pb_start".into(),
                    value: true,
                }],
                Condition::Elapsed { seconds: 0.2 },
            ),
            step(
                "release",
                vec![Action::Set {
                    signal: "pb_start".into(),
                    value: false,
                }],
                Condition::Elapsed { seconds: 2.5 },
            ),
        ],
    });
    scene
}

const NAMES: [&str; 3] = ["ctl", "process", "operator"];

fn word<'a>(tl: &'a SequenceTimeline, name: &str) -> &'a WordTrack {
    tl.words
        .iter()
        .find(|w| w.name == name)
        .unwrap_or_else(|| panic!("no word lane {name}"))
}

fn rise(tl: &SequenceTimeline, name: &str) -> f64 {
    tl.signals
        .iter()
        .find(|s| s.name == name)
        .unwrap()
        .edges
        .iter()
        .find(|e| e.1)
        .unwrap_or_else(|| panic!("{name} never rose"))
        .0
}

#[test]
fn a_measured_word_goes_through_the_loop_and_a_comparison_trips_on_it() {
    let scene = filter();
    let tl = scene
        .simulate_sequences(&NAMES, &RolloutOptions::default())
        .unwrap();
    // The process integrates 0.5 kPa a scan from the scan the pump runs.
    let dp_true = word(&tl, "dp_true");
    assert_eq!(dp_true.samples[0], (0.0, 10.0));
    assert!(!dp_true.integer);
    let (t1, v1) = dp_true.samples[1];
    assert_eq!(v1, 10.5);
    assert!((t1 - rise(&tl, "pump")).abs() < 1e-9);
    // The chain: 4 + 16·x/100 mA, 250 counts per mA, the controller scales
    // the counts back — read the scan after the process wrote.
    let raw = word(&tl, "dp.raw");
    assert!(raw.integer);
    assert_eq!(raw.samples[0], (0.0, 400.0));
    let scaled = word(&tl, "dp");
    assert_eq!(scaled.samples[0], (0.0, 10.0));
    for (t, x) in dp_true.samples.iter().skip(1).take(40) {
        let counts = (x * 40.0).round();
        assert_eq!(raw.value_at(t + 0.01), counts, "at {t}");
        assert_eq!(scaled.value_at(t + 0.01), counts / 40.0, "at {t}");
    }
    // The warning trips at the first scan the scaled value reads 60, the
    // alarm at 80 — and the alarm drops the pump, so the pressure stops.
    let warn = rise(&tl, "warn");
    assert_eq!(scaled.value_at(warn), 60.0);
    let alarm = rise(&tl, "alarm");
    assert_eq!(scaled.value_at(alarm), 80.0);
    assert!((alarm - warn - 0.4).abs() < 1e-9, "{warn} {alarm}");
    assert!(dp_true.samples.last().unwrap().1 < 82.0);
    // Deterministic, words included.
    let again = scene
        .simulate_sequences(&NAMES, &RolloutOptions::default())
        .unwrap();
    assert_eq!(format!("{:?}", tl.words), format!("{:?}", again.words));
}

#[test]
fn integer_words_round_and_divide_like_a_plc() {
    let mut scene = sample_scene();
    scene.define_signal("tick", false);
    scene.define_word("n", 0.0, true).unwrap();
    scene.define_word("half", 0.0, true).unwrap();
    scene.define_word("ratio", 0.0, false).unwrap();
    scene.define_word("rounded", 0.0, true).unwrap();
    scene.upsert_logic(Logic {
        name: "count".into(),
        rungs: vec![
            rung(
                "count",
                Condition::Rising {
                    name: "tick".into(),
                },
                vec![mov("n", calc(w("n"), ArithOp::Add, k(1.0)))],
            ),
            rung(
                "derive",
                Condition::Immediately,
                vec![
                    // INT / INT truncates; a REAL operand makes it REAL;
                    // a REAL written to an INT rounds.
                    mov("half", calc(w("n"), ArithOp::Div, k(2.0))),
                    mov("ratio", calc(w("n"), ArithOp::Div, k(2.5))),
                    mov("rounded", calc(w("n"), ArithOp::Mul, k(0.5))),
                ],
            ),
        ],
        ..Default::default()
    });
    let pulse = |n: usize| -> Vec<Step> {
        (0..n)
            .flat_map(|i| {
                [
                    step(
                        &format!("on{i}"),
                        vec![Action::Set {
                            signal: "tick".into(),
                            value: true,
                        }],
                        Condition::Elapsed { seconds: 0.05 },
                    ),
                    step(
                        &format!("off{i}"),
                        vec![Action::Set {
                            signal: "tick".into(),
                            value: false,
                        }],
                        Condition::Elapsed { seconds: 0.05 },
                    ),
                ]
            })
            .collect()
    };
    scene.upsert_sequence(Sequence {
        name: "pulses".into(),
        steps: pulse(3),
    });
    let tl = scene
        .simulate_sequences(&["count", "pulses"], &RolloutOptions::default())
        .unwrap();
    let last = |name: &str| word(&tl, name).samples.last().unwrap().1;
    assert_eq!(last("n"), 3.0);
    assert_eq!(last("half"), 1.0); // 3 / 2 truncated
    assert_eq!(last("ratio"), 1.2); // 3 / 2.5
    assert_eq!(last("rounded"), 2.0); // 1.5 rounds away from zero
    assert!(word(&tl, "n").integer && !word(&tl, "ratio").integer);
    // A step can move a word in on its entry, and wait on a comparison.
    scene.define_word("preset", 0.0, true).unwrap();
    scene.upsert_logic(Logic {
        name: "count".into(),
        rungs: vec![rung(
            "derive",
            Condition::Immediately,
            vec![mov("half", calc(w("preset"), ArithOp::Div, k(2.0)))],
        )],
        ..Default::default()
    });
    scene.upsert_sequence(Sequence {
        name: "pulses".into(),
        steps: vec![
            step(
                "preset",
                vec![Action::SetWord {
                    word: "preset".into(),
                    value: WordExpr::constant(7.0),
                }],
                cmp(w("half"), CompareOp::Eq, k(3.0)),
            ),
            step("tail", vec![], Condition::Elapsed { seconds: 0.05 }),
        ],
    });
    let tl = scene
        .simulate_sequences(&["count", "pulses"], &RolloutOptions::default())
        .unwrap();
    assert_eq!(word(&tl, "half").samples.last().unwrap().1, 3.0);
    assert!(tl.duration < 0.2);
    // A division by a word that reads zero fails the bake by name.
    scene.define_word("zero", 0.0, true).unwrap();
    scene.upsert_logic(Logic {
        name: "count".into(),
        rungs: vec![rung(
            "bad",
            Condition::Immediately,
            vec![mov("half", calc(w("n"), ArithOp::Div, w("zero")))],
        )],
        ..Default::default()
    });
    let err = scene
        .simulate_sequences(&["count", "pulses"], &RolloutOptions::default())
        .unwrap_err();
    assert!(err.to_string().contains("divides by zero"), "{err}");
}

#[test]
fn an_open_loop_reads_below_the_range_and_a_live_plant_takes_a_setpoint() {
    let mut scene = filter();
    // A broken wire: 0 mA, a quarter of the span under the bottom count.
    scene
        .upsert_scenario(Scenario {
            name: "wire".into(),
            signals: Vec::new(),
            obstacles: Vec::new(),
            joints: Vec::new(),
            faults: vec![Fault {
                target: "dp".into(),
                kind: FaultKind::Open,
            }],
        })
        .unwrap();
    let mut broken = scene.clone();
    broken.apply_scenario("wire").unwrap();
    let tl = broken
        .simulate_sequences(&NAMES, &RolloutOptions::default())
        .unwrap();
    assert_eq!(word(&tl, "dp.raw").samples.last().unwrap().1, -1000.0);
    assert_eq!(word(&tl, "dp").samples.last().unwrap().1, -25.0);
    // A stuck level means nothing on a current loop.
    let err = scene
        .upsert_scenario(Scenario {
            name: "stuck".into(),
            signals: Vec::new(),
            obstacles: Vec::new(),
            joints: Vec::new(),
            faults: vec![Fault {
                target: "dp".into(),
                kind: FaultKind::StuckAt(true),
            }],
        })
        .and_then(|_| scene.clone().apply_scenario("stuck"))
        .unwrap_err();
    assert!(err.to_string().contains("analog input"), "{err}");

    // Live: the plant of PLC1 in loopback. Keying a lower setpoint in
    // brings the warning forward; forcing the transmitter trips the alarm.
    scene
        .upsert_io_node(IoNode {
            name: "PLC1".into(),
            kind: IoNodeKind::Plc,
            programs: vec!["ctl".into()],
            uplink: None,
            channels: Vec::new(),
            place: None,
            model: None,
            monitor: false,
        })
        .unwrap();
    let options = RolloutOptions::default();
    let bake = scene.simulate_sequences(&NAMES, &options).unwrap();
    let mut live = scene
        .open_plant(&NAMES, &options, None, &["PLC1"], true, false, &[])
        .unwrap();
    live.write_word("dp_warn", 30.0).unwrap();
    while !live.finished() {
        live.tick().unwrap();
    }
    let tuned = live.finish();
    assert!(
        (rise(&bake, "warn") - rise(&tuned, "warn") - 0.6).abs() < 1e-9,
        "{} {}",
        rise(&bake, "warn"),
        rise(&tuned, "warn")
    );
    assert_eq!(rise(&bake, "alarm"), rise(&tuned, "alarm"));

    let mut live = scene
        .open_plant(&NAMES, &options, None, &["PLC1"], true, false, &[])
        .unwrap();
    live.force_word("dp", 90.0).unwrap();
    for _ in 0..3 {
        live.tick().unwrap();
    }
    live.unforce_lane("dp").unwrap();
    while !live.finished() {
        live.tick().unwrap();
    }
    let forced = live.finish();
    assert!(rise(&forced, "alarm") < 0.05);
    assert_eq!(word(&forced, "dp.raw").value_at(0.01), 3600.0);
    // Released, the transmitter reads the process again.
    assert_eq!(word(&forced, "dp").samples.last().unwrap().1, 10.0);
    // The counts are the module's: force the input, not them.
    let mut live = scene
        .open_plant(&NAMES, &options, None, &["PLC1"], true, false, &[])
        .unwrap();
    assert!(live.force_word("dp.raw", 5.0).is_err());
    assert!(live.write_word("dp", 5.0).is_err());
}

#[test]
fn words_are_points_cross_the_exchange_and_are_written_as_st() {
    let mut scene = filter();
    // The PLC also reports the margin left to the limit — a word it
    // writes and the plant-side program reads (a handshake word).
    scene.define_word("margin", 0.0, false).unwrap();
    scene.define_word("seen", 0.0, false).unwrap();
    let mut ctl = scene.logic("ctl").unwrap().clone();
    ctl.rungs.push(rung(
        "margin",
        Condition::Immediately,
        vec![mov("margin", calc(w("dp_limit"), ArithOp::Sub, w("dp")))],
    ));
    scene.upsert_logic(ctl);
    let mut process = scene.logic("process").unwrap().clone();
    process.rungs.push(rung(
        "echo",
        Condition::Immediately,
        vec![mov(
            "seen",
            WordExpr {
                left: w("margin"),
                op: None,
            },
        )],
    ));
    scene.upsert_logic(process);
    scene
        .upsert_io_node(IoNode {
            name: "PLC1".into(),
            kind: IoNodeKind::Plc,
            programs: vec!["ctl".into()],
            uplink: None,
            channels: Vec::new(),
            place: None,
            model: None,
            monitor: false,
        })
        .unwrap();
    let d = crate::iomap::derive(&scene, Some(&NAMES)).unwrap();
    let on_plc = |name: &str| {
        d.points
            .iter()
            .filter(|p| p.id.name == name && p.host.as_deref() == Some("PLC1"))
            .collect::<Vec<_>>()
    };
    // The analog input: one AI point carrying the counts, read by the
    // two comparison rungs and the margin rung.
    let dp = on_plc("dp");
    assert_eq!(dp.len(), 1);
    assert_eq!(dp[0].kind, crate::iomap::ChannelKind::Ai);
    assert_eq!(dp[0].id.direction, IoDirection::Input);
    assert_eq!(crate::iomap::natural_type(dp[0]), "INT");
    assert_eq!(dp[0].readers.len(), 3);
    // The setpoints are the controller's own registers.
    assert_eq!(
        on_plc("dp_warn")[0].status,
        crate::iomap::PointStatus::Internal
    );
    // The margin crosses: an output word on the PLC.
    let margin = on_plc("margin");
    assert_eq!(margin[0].id.direction, IoDirection::Output);
    assert_eq!(margin[0].kind, crate::iomap::ChannelKind::Word);
    assert_eq!(crate::iomap::natural_type(margin[0]), "REAL");
    // The measured word is the plant's, not the PLC's.
    assert!(on_plc("dp_true").is_empty());

    let options = RolloutOptions::default();
    let bake = scene.simulate_sequences(&NAMES, &options).unwrap();
    let mut live = scene
        .open_plant(&NAMES, &options, None, &["PLC1"], true, false, &[])
        .unwrap();
    let xc = live.exchange("PLC1").unwrap();
    let point = xc.points.iter().find(|p| p.id.name == "dp").unwrap();
    assert_eq!(point.value, crate::exchange::ValueKind::Int);
    assert_eq!(point.initial, crate::exchange::Value::Int(400));
    live.tick().unwrap();
    let outputs = live.outputs("PLC1").unwrap();
    let dp_out = outputs.iter().find(|(id, _)| id.name == "dp").unwrap();
    assert_eq!(dp_out.1, crate::exchange::Value::Int(400));
    while !live.finished() {
        live.tick().unwrap();
    }
    // The margin went out through the exchange and came back as a write.
    assert!(live
        .input_log()
        .iter()
        .any(|r| r.id.name == "margin" && r.value == crate::exchange::Value::Real(70.0)));
    let looped = live.finish();
    assert_eq!(
        format!("{:?}", looped.signals),
        format!("{:?}", bake.signals)
    );
    assert_eq!(format!("{:?}", looped.words), format!("{:?}", bake.words));
    assert_eq!(word(&bake, "seen").value_at(0.05), 70.0);

    let st = crate::st::render_st(
        &scene,
        &crate::st::StOptions {
            sequences: Some(NAMES.iter().map(|s| s.to_string()).collect()),
            host: "PLC1".into(),
            ..Default::default()
        },
    )
    .unwrap();
    for line in [
        "dp_eu := 0.0 + (INT_TO_REAL(dp) - 0.0) * 100.0 / 4000.0;",
        "warn := (dp_eu >= dp_warn);",
        "IF (dp_eu >= dp_limit) THEN alarm := TRUE; END_IF;",
        "IF TRUE THEN margin := dp_limit - dp_eu; END_IF;",
        "dp_warn : REAL := 60.0; (* word *)",
        "dp_eu : REAL; (* scaled *)",
        "dp : INT;",
        "margin : REAL;",
    ] {
        assert!(st.contains(line), "missing `{line}` in\n{st}");
    }
    // PLCopen carries contacts and coils only, and says so.
    let err = crate::plcopen::render_plcopen(&scene, &Default::default()).unwrap_err();
    assert!(err.to_string().contains("Structured Text"), "{err}");
}

#[test]
fn authoring_errors_are_named() {
    let mut scene = filter();
    let err = scene.define_word("pump", 0.0, false).unwrap_err();
    assert!(err.to_string().contains("already a signal"), "{err}");
    let err = scene
        .upsert_analog(Analog {
            name: "lvl".into(),
            source: "nothing".into(),
            range: (0.0, 1.0),
            unit: "m".into(),
            raw: (0, 4000),
            place: None,
        })
        .unwrap_err();
    assert!(err.to_string().contains("unknown word `nothing`"), "{err}");
    let bad = |scene: &mut Scene, outputs: Vec<RungOutput>, condition: Condition| {
        scene.upsert_logic(Logic {
            name: "ctl".into(),
            rungs: vec![rung("bad", condition, outputs)],
            ..Default::default()
        });
        scene
            .simulate_sequences(&NAMES, &RolloutOptions::default())
            .unwrap_err()
            .to_string()
    };
    // A measured word is read-only; an unknown one is named.
    let e = bad(
        &mut scene.clone(),
        vec![mov("dp", WordExpr::constant(1.0))],
        Condition::Immediately,
    );
    assert!(e.contains("read-only"), "{e}");
    let e = bad(
        &mut scene.clone(),
        vec![coil("warn")],
        cmp(w("ghost"), CompareOp::Gt, k(1.0)),
    );
    assert!(e.contains("unknown word `ghost`"), "{e}");
    let e = bad(
        &mut scene,
        vec![mov("dp_warn", calc(w("dp"), ArithOp::Div, k(0.0)))],
        Condition::Immediately,
    );
    assert!(e.contains("division by the constant 0"), "{e}");
}

/// A program marked as the equipment's own is left out of the I/O map:
/// the motor coil it reads is an output of the PLC that writes it, the
/// word it integrates is no point at all — and it still runs, in a bake
/// and beside an external controller alike.
#[test]
fn a_plant_program_is_physics_not_wiring() {
    let mut scene = filter();
    // The PLC drives the motor from its run relay; the process follows
    // the motor (the relay itself is the controller's own memory).
    scene.define_signal("motor", false);
    let mut ctl = scene.logic("ctl").unwrap().clone();
    ctl.rungs
        .push(rung("motor", sig("pump"), vec![coil("motor")]));
    scene.upsert_logic(ctl);
    let mut process = scene.logic("process").unwrap().clone();
    process.rungs[0].condition = sig("motor");
    scene.upsert_logic(process);
    scene
        .upsert_io_node(IoNode {
            name: "PLC1".into(),
            kind: IoNodeKind::Plc,
            programs: vec!["ctl".into()],
            uplink: None,
            channels: Vec::new(),
            place: None,
            model: None,
            monitor: false,
        })
        .unwrap();
    let hosted = |scene: &Scene, host: &str| {
        crate::iomap::derive(scene, Some(&NAMES))
            .unwrap()
            .points
            .into_iter()
            .filter(|p| p.host.as_deref() == Some(host))
            .map(|p| (p.label(), p.source))
            .collect::<Vec<_>>()
    };
    // As an ordinary program, `process` is a controller on the implicit
    // cell host with wires of its own: the motor is a handshake to it.
    assert!(hosted(&scene, crate::iomap::CELL_HOST)
        .iter()
        .any(|(label, _)| label == "motor"));
    assert!(
        hosted(&scene, "PLC1").contains(&("motor".to_string(), crate::iomap::IoSource::Handshake))
    );
    let before = scene
        .simulate_sequences(&NAMES, &RolloutOptions::default())
        .unwrap();

    let mut process = scene.logic("process").unwrap().clone();
    process.plant = true;
    scene.upsert_logic(process);
    let d = crate::iomap::derive(&scene, Some(&NAMES)).unwrap();
    assert!(d
        .program_hosts
        .contains(&("process".to_string(), crate::iomap::PLANT_HOST.to_string())));
    assert!(hosted(&scene, crate::iomap::CELL_HOST)
        .iter()
        .all(|(label, _)| label != "motor" && label != "dp_true"));
    assert!(hosted(&scene, crate::iomap::PLANT_HOST).is_empty());
    // The motor is now what it is on the drawing: an output coil of PLC1.
    assert!(
        hosted(&scene, "PLC1").contains(&("motor".to_string(), crate::iomap::IoSource::WriteOnly))
    );
    assert!(d.points.iter().all(|p| p.id.name != "dp_true"));
    // A word only the plant's program uses is in use, not "unreferenced".
    assert!(d
        .report
        .findings
        .iter()
        .all(|f| !f.message.contains("dp_true")));
    // It runs exactly as before.
    let after = scene
        .simulate_sequences(&NAMES, &RolloutOptions::default())
        .unwrap();
    assert_eq!(format!("{:?}", before.words), format!("{:?}", after.words));
    // And beside a PLC outside the plant: the controller's writes come in
    // through the exchange, the process integrates on its own.
    let mut live = scene
        .open_plant(
            &NAMES,
            &RolloutOptions::default(),
            None,
            &["PLC1"],
            false,
            false,
            &[],
        )
        .unwrap();
    let motor = crate::iomap::IoPointId {
        name: "motor".into(),
        aspect: None,
        direction: IoDirection::Output,
    };
    live.apply("PLC1", vec![(motor, crate::exchange::Value::Bool(true))])
        .unwrap();
    for _ in 0..10 {
        live.tick().unwrap();
    }
    let tl = live.finish();
    // Ten scans of flow from the scan the write landed in: 10 → 15 kPa.
    assert_eq!(word(&tl, "dp_true").samples.last().unwrap().1, 15.0);
}
