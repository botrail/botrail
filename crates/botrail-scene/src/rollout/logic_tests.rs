//! Rungs and gates (design-plc-resolution.md PF1): a program's continuous
//! half, scanned every scan after its step transitions.

use nalgebra::{Isometry3, Point3, Vector3};

use super::*;
use crate::iomap::{IoNode, IoNodeKind};
use crate::rollout::tests::sample_scene;
use crate::seq::{
    Device, Logic, Rung, RungOutput, Sensor, SensorKind, SensorWatch, Sequence, Step,
};
use crate::Geometry;

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

fn not(c: Condition) -> Condition {
    Condition::Not(Box::new(c))
}

fn set(signal: &str, value: bool) -> Action {
    Action::Set {
        signal: signal.into(),
        value,
    }
}

fn wait(seconds: f64) -> Condition {
    Condition::Elapsed { seconds }
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

/// A belt with a box on it and a beam across it; push buttons are relays
/// an operator's script (a program of its own) presses.
fn cell() -> Scene {
    let mut scene = sample_scene();
    scene
        .add_obstacle(
            "box",
            Geometry::Box {
                size: Vector3::new(0.04, 0.04, 0.04),
            },
            Isometry3::translation(-0.5, 0.5, 0.5),
        )
        .unwrap();
    scene.upsert_device(Device {
        name: "conv".into(),
        kind: DeviceKind::Conveyor {
            zone_pose: Isometry3::translation(-0.2, 0.5, 0.5),
            zone_size: Vector3::new(1.2, 0.3, 0.3),
            velocity: Vector3::new(0.25, 0.0, 0.0),
            running: false,
            encoder: false,
        },
    });
    scene
        .upsert_sensor(Sensor {
            name: "beam".into(),
            kind: SensorKind::Beam {
                from: Point3::new(0.0, 0.3, 0.5),
                to: Point3::new(0.0, 0.7, 0.5),
                radius: 0.005,
            },
            watch: SensorWatch::AllObjects,
            mount: None,
            ids: None,
        })
        .unwrap();
    for s in ["pb_start", "pb_stop", "run", "seen"] {
        scene.define_signal(s, false);
    }
    scene
}

/// The self-holding run relay and the belt's coil: `(start OR run) AND NOT
/// stop -( run )`, `run AND NOT beam -( conv )`.
fn run_circuit(scene: &mut Scene) {
    scene.upsert_logic(Logic {
        name: "common".into(),
        rungs: vec![
            rung(
                "run",
                Condition::All(vec![
                    Condition::Any(vec![sig("pb_start"), sig("run")]),
                    not(sig("pb_stop")),
                ]),
                vec![coil("run")],
            ),
            rung(
                "belt",
                Condition::All(vec![sig("run"), not(sig("beam"))]),
                vec![coil("conv")],
            ),
        ],
        ..Default::default()
    });
}

/// The operator: start pressed at 0.5 s for 0.2 s, then `tail` seconds on.
fn operator(scene: &mut Scene, tail: f64) {
    scene.upsert_sequence(Sequence {
        name: "operator".into(),
        steps: vec![
            step("idle", vec![], wait(0.5)),
            step("press", vec![set("pb_start", true)], wait(0.2)),
            step("release", vec![set("pb_start", false)], wait(tail)),
        ],
    });
}

fn lane<'a>(tl: &'a SequenceTimeline, name: &str) -> &'a [(f64, bool)] {
    &tl.signals.iter().find(|s| s.name == name).unwrap().edges
}

#[test]
fn a_rung_holds_itself_and_its_coil_runs_the_belt_until_the_beam() {
    let mut scene = cell();
    run_circuit(&mut scene);
    operator(&mut scene, 4.0);
    let tl = scene
        .simulate_sequences(&["common", "operator"], &RolloutOptions::default())
        .unwrap();
    // The relay latches the scan after the button (the operator scans
    // after the rungs) and holds when the button is let go.
    let run = lane(&tl, "run");
    assert_eq!(run.len(), 2);
    assert!((run[1].0 - 0.51).abs() < 1e-9 && run[1].1);
    // The belt runs from the latch until the box reaches the beam.
    let conv = lane(&tl, "conv");
    assert_eq!(
        conv.iter().map(|e| e.1).collect::<Vec<_>>(),
        [false, true, false]
    );
    let beam = lane(&tl, "beam");
    // Sensors are read before the programs scan: the coil drops in the
    // very scan the beam trips.
    assert!((conv[2].0 - beam[1].0).abs() < 1e-12);
    // The pure-ladder program has no step bands; the operator has three.
    assert_eq!(
        tl.step_spans
            .iter()
            .filter(|s| s.sequence == "common")
            .count(),
        0
    );
    assert_eq!(
        tl.step_spans
            .iter()
            .filter(|s| s.sequence == "operator")
            .count(),
        3
    );
    // Deterministic.
    let again = scene
        .simulate_sequences(&["common", "operator"], &RolloutOptions::default())
        .unwrap();
    assert_eq!(format!("{:?}", tl.signals), format!("{:?}", again.signals));
    assert_eq!(format!("{:?}", tl.objects), format!("{:?}", again.objects));
}

#[test]
fn nothing_but_rungs_is_refused_as_a_bake() {
    let mut scene = cell();
    run_circuit(&mut scene);
    let err = scene
        .simulate_sequence("common", &RolloutOptions::default())
        .unwrap_err();
    assert!(err.to_string().contains("nothing ends this bake"), "{err}");
}

#[test]
fn a_timer_delays_and_a_counter_counts_and_resets() {
    let mut scene = cell();
    scene.define_signal("pulse", false);
    scene.define_signal("clear", false);
    scene.upsert_logic(Logic {
        name: "common".into(),
        rungs: vec![
            rung(
                "delay",
                sig("pulse"),
                vec![RungOutput::Ton {
                    signal: "t_pulse".into(),
                    seconds: 0.3,
                }],
            ),
            rung(
                "count",
                sig("pulse"),
                vec![RungOutput::Ctu {
                    signal: "c_two".into(),
                    preset: 2,
                }],
            ),
            rung(
                "clear",
                sig("clear"),
                vec![RungOutput::Reset {
                    signal: "c_two".into(),
                }],
            ),
            rung(
                "latch",
                sig("t_pulse"),
                vec![RungOutput::Set {
                    signal: "seen".into(),
                }],
            ),
        ],
        ..Default::default()
    });
    // The timer's and the counter's signals are declared by their use.
    assert!(scene.signals().iter().any(|s| s.name == "t_pulse"));
    assert!(scene.signals().iter().any(|s| s.name == "c_two"));
    scene.upsert_sequence(Sequence {
        name: "operator".into(),
        steps: vec![
            step("a", vec![set("pulse", true)], wait(0.5)), // long: the timer fires
            step("b", vec![set("pulse", false)], wait(0.2)),
            step("c", vec![set("pulse", true)], wait(0.1)), // short: it does not
            step("d", vec![set("pulse", false)], wait(0.2)),
            step("e", vec![set("clear", true)], wait(0.1)),
            step("f", vec![set("clear", false)], wait(0.1)),
        ],
    });
    let tl = scene
        .simulate_sequences(&["common", "operator"], &RolloutOptions::default())
        .unwrap();
    // On 0.3 s after the rung came true (the first scan sees the pulse:
    // every program's first step is entered before any rung scans), off
    // the scan after the pulse drops; the short pulse never fires it.
    let t = lane(&tl, "t_pulse");
    assert_eq!(t.len(), 3);
    assert!((t[1].0 - 0.30).abs() < 1e-9 && t[1].1);
    assert!((t[2].0 - 0.51).abs() < 1e-9 && !t[2].1);
    // SET latches what the timer saw.
    assert!(lane(&tl, "seen").last().unwrap().1);
    // Two rising edges reach the preset; RST clears count and contact.
    let c = lane(&tl, "c_two");
    assert_eq!(
        c.iter().map(|e| e.1).collect::<Vec<_>>(),
        [false, true, false]
    );
    assert!((c[1].0 - 0.71).abs() < 1e-9);
    let count = &tl
        .words
        .iter()
        .find(|w| w.name == "c_two.count")
        .unwrap()
        .samples;
    // (the first edge lands at t = 0, over the initial sample)
    assert_eq!(
        count.iter().map(|s| s.1).collect::<Vec<_>>(),
        [1.0, 2.0, 0.0]
    );
}

/// A step chain under gates: it advances only while `auto`, a reset puts
/// it back in its first step, and it starts over at its end.
#[test]
fn gates_hold_reset_and_cycle_a_step_chain() {
    let mut scene = cell();
    scene.define_signal("auto", false);
    scene.define_signal("clear", false);
    scene.define_signal("out", false);
    scene.upsert_sequence(Sequence {
        name: "chain".into(),
        steps: vec![
            step("one", vec![set("out", false)], wait(0.2)),
            step("two", vec![set("out", true)], wait(0.2)),
        ],
    });
    scene.upsert_logic(Logic {
        name: "chain".into(),
        enable: Some(sig("auto")),
        reset: Some(sig("clear")),
        cycle: true,
        plant: false,
        rungs: vec![rung(
            "in two",
            Condition::StepActive {
                sequence: "chain".into(),
                step: "two".into(),
            },
            vec![coil("seen")],
        )],
    });
    scene.upsert_sequence(Sequence {
        name: "operator".into(),
        steps: vec![
            step("held", vec![], wait(1.0)), // not enabled: the chain waits in `one`
            step("go", vec![set("auto", true)], wait(0.5)), // two 1.01, one 1.21, two 1.41 …
            step("clear", vec![set("clear", true)], wait(0.3)),
            step("release", vec![set("clear", false)], wait(0.1)),
        ],
    });
    let tl = scene
        .simulate_sequences(&["chain", "operator"], &RolloutOptions::default())
        .unwrap();
    let spans: Vec<(&str, f64, f64)> = tl
        .step_spans
        .iter()
        .filter(|s| s.sequence == "chain")
        .map(|s| (s.name.rsplit('/').next().unwrap(), s.start, s.end))
        .collect();
    let names: Vec<&str> = spans.iter().map(|s| s.0).collect();
    assert_eq!(names, ["one", "two", "one", "two", "one", "two"]);
    // Held a second in `one` until enabled (its own timer long served).
    assert!((spans[0].2 - 1.01).abs() < 1e-9, "{spans:?}");
    // Cycled: two → one in the same scan.
    assert!((spans[1].2 - spans[2].1).abs() < 1e-12);
    // Reset out of `two` back into `one`, held there while it stands
    // (its timer long served), and on the scan after it is released.
    assert!(
        (spans[3].2 - 1.51).abs() < 1e-9 && (spans[4].1 - 1.51).abs() < 1e-9,
        "{spans:?}"
    );
    assert!((spans[4].2 - 1.81).abs() < 1e-9, "{spans:?}");
    // The rung on the step relay follows the step.
    let seen = lane(&tl, "seen");
    assert!((seen[1].0 - spans[1].1).abs() < 1e-12 && seen[1].1);
    assert!((seen[2].0 - spans[1].2).abs() < 1e-12 && !seen[2].1);
}

#[test]
fn authoring_errors_are_named() {
    let mut scene = cell();
    operator(&mut scene, 0.1);
    let bake = |scene: &Scene| {
        scene
            .simulate_sequences(&["common", "operator"], &RolloutOptions::default())
            .unwrap_err()
            .to_string()
    };
    let one = |r: Rung| Logic {
        name: "common".into(),
        rungs: vec![r],
        ..Default::default()
    };
    scene.upsert_logic(one(rung("t", wait(1.0), vec![coil("run")])));
    assert!(
        bake(&scene).contains("belong to a step"),
        "{}",
        bake(&scene)
    );
    scene.upsert_logic(one(rung("t", sig("pb_start"), vec![coil("nothing")])));
    assert!(
        bake(&scene).contains("unknown coil target `nothing`"),
        "{}",
        bake(&scene)
    );
    scene.upsert_logic(Logic {
        name: "common".into(),
        rungs: vec![
            rung("a", sig("pb_start"), vec![coil("run")]),
            rung("b", sig("pb_stop"), vec![coil("run")]),
        ],
        ..Default::default()
    });
    assert!(bake(&scene).contains("double coil"), "{}", bake(&scene));
    // Two programs may not drive one relay: the single-owner rule.
    scene.upsert_logic(one(rung("a", sig("pb_stop"), vec![coil("pb_start")])));
    assert!(
        bake(&scene).contains("commanded by both"),
        "{}",
        bake(&scene)
    );
    // An empty logic clears the program's rungs — and with no steps the
    // program is nothing.
    scene.upsert_logic(Logic {
        name: "common".into(),
        ..Default::default()
    });
    assert!(scene.logic("common").is_none());
}

/// The rungs on a PLC: their reads and coils are the host's points, the
/// loopback through the exchange is the bake, and the ST writes them.
#[test]
fn rungs_cross_the_exchange_and_are_written_as_st() {
    let mut scene = cell();
    run_circuit(&mut scene);
    operator(&mut scene, 4.0);
    scene
        .upsert_io_node(IoNode {
            name: "PLC1".into(),
            kind: IoNodeKind::Plc,
            programs: vec!["common".into()],
            uplink: None,
            channels: Vec::new(),
            place: None,
            model: None,
            monitor: false,
        })
        .unwrap();
    let names = ["common", "operator"];
    let d = crate::iomap::derive(&scene, Some(&names)).unwrap();
    let point = |name: &str| {
        d.points
            .iter()
            .find(|p| p.id.name == name && p.host.as_deref() == Some("PLC1"))
            .unwrap_or_else(|| panic!("no point {name}"))
    };
    // Rung k of a program with no steps has flat index k.
    assert_eq!(point("beam").readers[0].index, 1);
    assert_eq!(point("beam").readers[0].name, "belt");
    assert_eq!(point("conv").writers[0].index, 1);
    assert_eq!(
        point("pb_start").id.direction,
        crate::iomap::IoDirection::Input
    );

    let options = RolloutOptions::default();
    let bake = scene.simulate_sequences(&names, &options).unwrap();
    let mut live = scene
        .open_plant(&names, &options, None, &["PLC1"], true, false, &[])
        .unwrap();
    while !live.finished() {
        live.tick().unwrap();
    }
    let looped = live.finish();
    assert_eq!(
        format!("{:?}", looped.signals),
        format!("{:?}", bake.signals)
    );
    assert_eq!(
        format!("{:?}", looped.objects),
        format!("{:?}", bake.objects)
    );

    let st = crate::st::render_st(
        &scene,
        &crate::st::StOptions {
            sequences: Some(names.iter().map(|s| s.to_string()).collect()),
            host: "PLC1".into(),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(
        st.contains("(* rungs, every scan, top to bottom *)"),
        "{st}"
    );
    assert!(
        st.contains("run := ((pb_start OR run) AND NOT (pb_stop));"),
        "{st}"
    );
    assert!(st.contains("conv_run := (run AND NOT (beam));"), "{st}");
    // PLCopen writes step chains only, and says so.
    let err = crate::plcopen::render_plcopen(&scene, &Default::default()).unwrap_err();
    assert!(err.to_string().contains("Structured Text"), "{err}");
}
