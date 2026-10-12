//! The static shape: the rod at rest between its holders, frame by frame.
use botrail_rope::{
    pass::*,
    shape::{shape, StaticSettings},
    shape_bridge::ShapeData,
    PoseData, RopeLocation,
};
use rapier_rope::{
    CollisionSettings, NativeRopeMaterial, RopeSpec, SamplingSettings, SpringSettings,
};

const EI: f64 = 1.5e-3;
const DENSITY: f64 = 0.02;

fn pass(points: Vec<[f64; 3]>, spacing: f64) -> RopePass {
    RopePass {
        ground_z: None,
        preroll_s: 0.0,
        extra_colliders: Vec::new(),
        straight_rest: true,
        rod_hz: 0.0,
        spec: RopeSpec::new(
            "tube",
            points,
            NativeRopeMaterial::new(
                DENSITY,
                SpringSettings::new(500.0, 1.0),
                SpringSettings::new(20.0, 0.8),
            ),
            SamplingSettings::new(spacing),
            CollisionSettings::new(0.003),
        ),
        grippers: vec![],
        collision_links: vec![],
        obstacles: vec![],
        connectors: vec![],
        step_s: 1.0 / 240.0,
        pins: vec![],
        anchors: vec![],
        guides: vec![],
        color: None,
    }
}
fn link() -> LinkBinding {
    LinkBinding {
        robot: "robot".into(),
        link: "tool".into(),
    }
}
fn still(name: &str, at: [f64; 3], shapes: Vec<(PoseData, ShapeData)>, duration: f64) -> Proxy {
    let pose = PoseData {
        position: at,
        ..Default::default()
    };
    Proxy {
        name: name.into(),
        times: vec![0.0, duration],
        poses: vec![pose, pose],
        shapes,
    }
}
fn lengths(points: &[[f64; 3]]) -> Vec<f64> {
    points
        .windows(2)
        .map(|w| {
            (0..3)
                .map(|i| (w[1][i] - w[0][i]).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .collect()
}

#[test]
fn a_cantilever_droops_by_its_rigidity_and_weight() {
    // 0.02 m held straight along +x, 0.15 m free: the tip droops about
    // w L⁴ / 8 EI (small deflections), with w = ρ g.
    let mut p = pass(vec![[0.0, 0.0, 0.5], [0.17, 0.0, 0.5]], 0.005);
    p.anchors.push(Anchor {
        body: AnchorBody::Link(link()),
        location: RopeLocation::Start,
        length_m: 0.02,
        rail: None,
        roll: false,
    });
    let bake = Bake {
        duration_s: 1.0,
        proxies: vec![still("robot/tool", [0.0, 0.0, 0.5], vec![], 1.0)],
        signals: vec![],
    };
    let track = shape(&bake, &p, &StaticSettings::new(EI)).unwrap();
    let tip = track.points[0].last().unwrap();
    let expected = DENSITY * 9.81 * 0.15f64.powi(4) / (8.0 * EI);
    let droop = 0.5 - tip[2];
    assert!(
        (droop - expected).abs() < 0.2 * expected,
        "droop {droop:.4} vs {expected:.4}"
    );
    assert!((tip[1]).abs() < 1e-6, "in the vertical plane");
    // Held straight: the first free sample continues the held direction.
    let held = &track.points[0][4];
    let next = &track.points[0][5];
    assert!(
        (next[2] - held[2]).abs() < 0.1 * (next[0] - held[0]),
        "{held:?} -> {next:?}"
    );
    for (i, l) in lengths(&track.points[0]).iter().enumerate() {
        assert!((l - 0.005).abs() < 1e-6, "segment {i}: {l}");
    }
    // Still bodies: one solve, the same shape at every frame.
    assert_eq!(track.times[0], 0.0);
    assert_eq!(*track.times.last().unwrap(), 1.0);
    assert!(track.points.iter().all(|f| *f == track.points[0]));
    assert!(track.held.iter().all(|h| h.is_empty()));
    assert!(track.solver.contains("static"));
}

#[test]
fn the_rod_rides_its_holder_keeps_its_length_and_is_the_same_twice() {
    let mut p = pass(vec![[0.0, 0.0, 0.4], [0.5, 0.0, 0.4]], 0.5 / 32.0);
    p.anchors.push(Anchor {
        body: AnchorBody::Link(link()),
        location: RopeLocation::Start,
        length_m: 0.05,
        rail: None,
        roll: false,
    });
    p.pins.push((RopeLocation::End, [0.5, 0.0, 0.4]));
    let proxy = Proxy {
        name: "robot/tool".into(),
        times: vec![0.0, 0.5, 1.0],
        poses: vec![
            PoseData {
                position: [0.0, 0.0, 0.4],
                ..Default::default()
            },
            PoseData {
                position: [0.1, 0.0, 0.4],
                ..Default::default()
            },
            PoseData {
                position: [0.1, 0.0, 0.4],
                ..Default::default()
            },
        ],
        shapes: vec![],
    };
    let bake = Bake {
        duration_s: 1.0,
        proxies: vec![proxy],
        signals: vec![],
    };
    let settings = StaticSettings::new(EI);
    let a = shape(&bake, &p, &settings).unwrap();
    let b = shape(&bake, &p, &settings).unwrap();
    assert_eq!(a.points, b.points, "deterministic");
    let (start, end) = (a.positions_at(0.0), a.positions_at(1.0));
    for i in 0..4 {
        assert!(
            (end[i][0] - start[i][0] - 0.1).abs() < 1e-9 && (end[i][2] - start[i][2]).abs() < 1e-9,
            "held sample {i} rides the body: {:?} -> {:?}",
            start[i],
            end[i]
        );
    }
    assert!((end.last().unwrap()[0] - 0.5).abs() < 1e-9, "the pin holds");
    // The holder came 0.1 m closer: the surplus bulges, under gravity, down.
    let low = end.iter().map(|q| q[2]).fold(f64::INFINITY, f64::min);
    assert!(low < 0.4 - 0.05, "bulges: {low}");
    // Inextensible: within a tenth of a percent while the line is taut
    // (the first frames, the chord the line's length), and exactly once
    // it has slack.
    for frame in &a.points {
        for l in lengths(frame) {
            assert!((l - 0.5 / 32.0).abs() < 3e-5, "inextensible: {l}");
        }
    }
    for l in lengths(a.points.last().unwrap()) {
        assert!(
            (l - 0.5 / 32.0).abs() < 5e-6,
            "inextensible with slack: {l}"
        );
    }
    assert!(a.times.len() > 20, "30 frames a second: {}", a.times.len());
}

#[test]
fn the_rod_keeps_off_the_bodies_around_it() {
    // A floppy line (a hundredth of the tube's rigidity) pinned level at
    // both ends with 4 cm to spare, laid as an arch; a block under the
    // middle whose top is 2 cm under the chord: the arch drops onto the
    // block and rests a radius off it instead of sagging into it, and
    // hangs beside it.
    let mut p = pass(
        vec![[0.0, 0.0, 0.4], [0.25, 0.0, 0.5], [0.5, 0.0, 0.4]],
        0.01,
    );
    p.pins.push((RopeLocation::Start, [0.0, 0.0, 0.4]));
    p.pins.push((RopeLocation::End, [0.5, 0.0, 0.4]));
    let block = still(
        "obstacle/block",
        [0.25, 0.0, 0.33],
        vec![(PoseData::default(), ShapeData::Cuboid([0.05, 0.05, 0.05]))],
        0.5,
    );
    let bake = Bake {
        duration_s: 0.5,
        proxies: vec![block],
        signals: vec![],
    };
    let track = shape(&bake, &p, &StaticSettings::new(EI / 100.0)).unwrap();
    let top = 0.38;
    let over: Vec<_> = track.points[0]
        .iter()
        .filter(|q| (q[0] - 0.25).abs() < 0.05)
        .collect();
    assert!(!over.is_empty());
    for q in &over {
        assert!(q[2] >= top + 0.003 - 2e-4, "a radius off the block: {q:?}");
    }
    let lowest = over.iter().map(|q| q[2]).fold(f64::INFINITY, f64::min);
    assert!(lowest < top + 0.003 + 1e-3, "resting on it: {lowest}");
    assert!(
        track.points[0]
            .iter()
            .any(|q| (q[0] - 0.25).abs() > 0.06 && q[2] < 0.4 - 0.005),
        "and hangs beside it"
    );
}

#[test]
fn malformed_input_fails_closed() {
    let p = pass(vec![[0.0, 0.0, 0.4], [0.5, 0.0, 0.4]], 0.01);
    let bake = Bake {
        duration_s: 1.0,
        proxies: vec![],
        signals: vec![],
    };
    let err = |r: Result<botrail_rope::RopeTrack, botrail_rope::Error>| r.unwrap_err().to_string();
    assert!(err(shape(&bake, &p, &StaticSettings::new(0.0))).contains("rigidity"));
    let mut anchored = p.clone();
    anchored.anchors.push(Anchor {
        body: AnchorBody::Obstacle("clamp".into()),
        location: RopeLocation::End,
        length_m: 0.0,
        rail: None,
        roll: false,
    });
    assert!(err(shape(&bake, &anchored, &StaticSettings::new(EI)))
        .contains("missing proxy obstacle/clamp"));
    let proxied = Bake {
        duration_s: 1.0,
        proxies: vec![still("obstacle/clamp", [0.5, 0.0, 0.4], vec![], 1.0)],
        signals: vec![],
    };
    anchored.anchors[0].length_m = -0.01;
    assert!(
        err(shape(&proxied, &anchored, &StaticSettings::new(EI))).contains("nonnegative length")
    );
}
