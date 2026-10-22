"""PCB handling around a horizontal transport loop, reconstructed from a reference image.

Three Universal Robots arms, a white KUKA AGILUS in front, green PCB carriers,
yellow/black guarding, a blue magazine and an overhead inspection head.
The image does not identify exact models or hidden dimensions. Selected products:
UR5e, two UR3e, KR 6 R900-2, Schmalz PFYN 6 ESD cups and ATS SuperTrak HORIZON10.
Product substitutions and SI geometry are described in pcb_loop_demo.md.

The rear UR feeds a magazine, the central UR presents boards to the AOI camera,
the KUKA loads an electrical test clamp, and the right UR sorts good/reject boards.
Inspection inputs are simulated component presence and resistance measurements.
The finite batch drains completely; carriers index only with all tools clear.

Run with the published catalog (from botrail):
    .venv/bin/python examples/assembly/pcb_loop_demo.py --studio
Use --catalog-root DIR or BOTRAIL_CATALOG_BUILD for a local catalog build.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import _pcb_loop_equipment as eq
import _shuttle_line_tooling as tooling
import botrail as bt

UR5 = "universal_robots/ur/ur5e/r2"
UR3 = "universal_robots/ur/ur3e/r1"
KUKA = "kuka/agilus/kr6-r900-2/r1"
ARM_IDS = {"ur_aoi": UR5, "ur_feed": UR3, "ur_sort": UR3, "kuka_test": KUKA}
BASES = {
    "ur_aoi": (0.0, -0.40, 0.70),
    "ur_feed": (0.14, 0.84, 0.88),
    "ur_sort": (0.98, 0.46, 0.82),
    "kuka_test": (0.99, -0.86, 0.70),
}
STATIONS = {"ur_aoi": 0, "ur_feed": 3, "ur_sort": 4, "kuka_test": 6}
ROLES = {
    "feed": "ur_feed",
    "aoi": "ur_aoi",
    "electrical": "kuka_test",
    "sort": "ur_sort",
}
DELAYS = {"feed": 0, "aoi": 3, "electrical": 5, "sort": 7}
# Deliberate sample defects exercise both reject paths. All other boards are good.
MISSING_ICS = {2: 1}
RESISTANCE_OHMS = {5: 150_000.0}
RESISTANCE_LIMITS = (900.0, 1100.0)
DOWN = (1.0, 0.0, 0.0, 0.0)
# The close view intentionally crops the front and right, as the supplied image does.
REFERENCE_VIEW = ((0.70, -1.95, 2.20), (0.0, -0.30, 1.10))
OVERVIEW = ((3.7, -5.3, 3.9), (0.0, 0.0, 0.75))
PROCESS_VIEWS = {
    "feed": ((0.90, 0.15, 1.85), (-0.02, 1.07, 1.07)),
    "aoi": ((0.10, -1.10, 1.85), (-0.66, -0.22, 1.18)),
    "electrical": ((1.3, -2.2, 1.65), (0.65, -1.25, 0.99)),
    "sort": ((2.1, -0.4, 1.6), (1.11, 0.4, 0.99)),
}


def default_catalog_root():
    configured = os.environ.get("BOTRAIL_CATALOG_BUILD")
    return Path(configured).expanduser() if configured else None


def build(catalog_root=None, *, board_count=8):
    if not 1 <= board_count <= 8:
        raise ValueError("board_count must be between 1 and 8")
    configured = catalog_root or default_catalog_root()
    root = Path(configured).expanduser().resolve() if configured else None

    def local_package(pid):
        return root / pid if root and (root / pid / "manifest.yaml").is_file() else None

    def load(pid):
        return (
            bt.Robot.from_package(root / pid, catalog_root=root)
            if local_package(pid)
            else bt.Robot.from_catalog(pid)
        )

    arms = {
        name: tooling.vacuum_hand(load(pid), load, "s2")
        for name, pid in ARM_IDS.items()
    }
    scene = bt.Scene(arms["ur_aoi"], name="ur_aoi", base_position=BASES["ur_aoi"])
    for name in ("ur_feed", "ur_sort", "kuka_test"):
        scene.add_robot(arms[name], name=name, base_position=BASES[name])
    for name in ARM_IDS:
        scene.set_part(
            f"{name}/tool",
            kind="tool",
            manufacturer="System integrator",
            model="Two-cup PCB holder",
            category="gripper.vacuum",
            description="SI holder with two catalog Schmalz PFYN 6 ESD cups; mounting fit unqualified",
            cup_catalog=tooling.PCB_CUP,
            cups=2,
        )
    eq.surroundings(scene)
    loop = eq.Loop()
    rails = eq.track(scene, loop, root)
    eq.peripherals(scene, BASES)
    equipment = eq.process_equipment(scene, loop, board_count)
    for name in BASES:
        base_link = "base_link" if name == "kuka_test" else "base_link_inertia"
        scene.allow_link_obstacle_contact(base_link, f"stand/{name}/top", robot=name)
    # The controllers have their own real catalog envelopes.
    for i, name in enumerate(("ur_aoi", "ur_feed", "ur_sort")):
        pid = "universal_robots/control-box/e-series/r1"
        bt.parts.controller(
            scene,
            f"controller/{name}",
            [name],
            (0.85, 0.90 + i * 0.20, 0.05),
            catalog=local_package(pid) or pid,
            mount="floor",
            yaw=math.pi / 2,
        )
    pid = "kuka/kr-c5/cabinet/r1"
    bt.parts.controller(
        scene,
        "controller/kuka",
        ["kuka_test"],
        (0.95, -0.80, 0.05),
        catalog=local_package(pid) or pid,
        variant="micro",
        mount="cabinet",
    )
    movers, boards = [], []
    model = eq.carrier(loop, load(eq.SHUTTLE))
    for i in range(loop.count):
        name = f"shuttle{i:02d}"
        scene.add_robot(model, name=name, base_position=(0, 0, loop.top))
        scene.set_joint_positions(list(loop.q(i * loop.pitch)), robot=name)
        scene.set_part(
            name,
            kind="robot",
            catalog=eq.SHUTTLE,
            manufacturer="ATS Automation",
            model="SuperTrak HORIZON10 shuttle with SI PCB nest",
            category="conveyor.shuttle",
        )
        # Only physical rolling contact with the stator, not robot/guard collisions.
        for rail in rails:
            scene.allow_link_obstacle_contact("body", rail, robot=name)
        movers.append(name)
    for i, pose in enumerate(equipment["magazine"]):
        parts = eq.pcb(scene, f"pcb{i:02d}", pose, missing_ic=MISSING_ICS.get(i))
        boards.append(parts)
    poses = teach(scene, loop, equipment)
    for name, taught in poses.items():
        scene.set_joint_positions(taught["hover"], robot=name)
    for name, view in {
        "reference": REFERENCE_VIEW,
        "overview": OVERVIEW,
        **PROCESS_VIEWS,
    }.items():
        scene.add_camera(
            name,
            position=view[0],
            look_at=view[1],
            fov=45,
            resolution=(1038, 848),
            body_visible=False,
        )
    return scene, {
        "loop": loop,
        "movers": movers,
        "boards": boards,
        "poses": poses,
        "root": root,
        "equipment": equipment,
        "board_count": board_count,
    }


def teach(scene, loop, equipment):
    """Teach carrier, magazine, camera, fixture and output poses with real IK."""
    poses = {}
    for name, station in STATIONS.items():
        x, y, z = loop.pose(station * loop.pitch)[0]
        xyzs = {"hover": (x, y, z + 0.18), "work": (x, y, z + 0.002)}
        vertical_pairs = []
        if name == ROLES["feed"]:
            for i, (position, _q) in enumerate(equipment["magazine"]):
                mx, my, mz = position
                xyzs[f"magazine_hover{i}"] = (
                    mx,
                    my,
                    equipment["magazine"][0][0][2] + 0.13,
                )
                xyzs[f"magazine_pick{i}"] = (mx, my, mz + 0.002)
                vertical_pairs.append((f"magazine_pick{i}", f"magazine_hover{i}"))
        elif name == ROLES["aoi"]:
            xyzs.update(scan_a=(-0.68, -0.27, 1.12), scan_b=(-0.68, -0.19, 1.12))
        elif name == ROLES["electrical"]:
            fx, fy, fz = equipment["fixture"][0]
            xyzs.update(
                fixture_hover=(fx, fy, fz + 0.18), fixture_work=(fx, fy, fz + 0.002)
            )
        else:
            for result, slots in equipment["outputs"].items():
                for i, (position, _q) in enumerate(slots):
                    ox, oy, oz = position
                    xyzs[f"{result}_hover{i}"] = (ox, oy, slots[-1][0][2] + 0.12)
                    xyzs[f"{result}_place{i}"] = (ox, oy, oz + 0.002)
                    vertical_pairs.append((f"{result}_place{i}", f"{result}_hover{i}"))
        # Short IK segments keep rack insertion/lift vertical. One long joint
        # ramp bows sideways into the magazine's narrow guide clearance.
        for work, hover in vertical_pairs:
            for k in range(1, 5):
                xyzs[f"{work}_z{k}"] = tuple(
                    a + (b - a) * k / 5 for a, b in zip(xyzs[work], xyzs[hover])
                )
        seed = (
            [0, -1.2, 1.4, 0, 1.3, 0]
            if name == ROLES["electrical"]
            else [0, -1.45, 1.4, -1.5, -math.pi / 2, 0]
        )
        taught = {}
        for tag, xyz in xyzs.items():
            scene.set_joint_positions(seed, robot=name)
            result = scene.set_tcp_target(xyz, DOWN, robot=name, max_iters=300)
            if not result.converged:
                raise RuntimeError(
                    f"{name}/{tag} unreachable: {result.pos_error * 1000:.2f} mm at {xyz}"
                )
            seed = list(scene.joint_positions_of(name))
            taught[tag] = seed
        poses[name] = taught
    return poses


def ramp(scene, robot, q, duration):
    return bt.seq.ramp(
        dict(zip(scene.robot_of(robot).joint_names, q)), duration, robot=robot
    )


def board_results(info):
    """Deterministic test inputs, separated from motion and runtime sort conditions."""
    records = []
    for i, members in enumerate(info["boards"]):
        aoi_ok = all(f"pcb{i:02d}/ic{chip}" in members for chip in range(3))
        resistance = RESISTANCE_OHMS.get(i, 1000.0 + i * 3)
        electrical_ok = RESISTANCE_LIMITS[0] <= resistance <= RESISTANCE_LIMITS[1]
        records.append(
            {
                "board": f"pcb{i:02d}",
                "carrier": (3 + i) % info["loop"].count,
                "aoi_ok": aoi_ok,
                "resistance_ohms": resistance,
                "electrical_ok": electrical_ok,
                "destination": "good" if aoi_ok and electrical_ok else "reject",
            }
        )
    return records


def programs(scene, info):
    """One transfer owner, four distinct stations, persistent per-board results."""
    loop, movers, boards = info["loop"], info["movers"], info["boards"]
    count = info["board_count"]
    records = info["results"] = board_results(info)
    for signal in (
        "indexing",
        "batch_done",
        "aoi_busy",
        "electrical_busy",
        "fixture_clamped",
    ):
        scene.define_signal(signal)
    for name in STATIONS:
        scene.define_signal(f"{name}_clear", initial=True)
    for i in range(loop.count):
        scene.define_signal(f"carrier{i:02d}_occupied")
    for row in records:
        for flag in (
            "loaded",
            "aoi_done",
            "aoi_ok",
            "electrical_done",
            "electrical_ok",
            "good",
            "reject",
            "complete",
        ):
            scene.define_signal(f"{row['board']}_{flag}")
    sq = scene.sequence("pcb_cell")

    def move(role, tag, duration=0.55):
        robot = ROLES[role]
        return ramp(scene, robot, info["poses"][robot][tag], duration)

    def take(i, role, *, detached=False):
        return ([] if detached else [bt.seq.detach(p) for p in boards[i]]) + [
            bt.seq.attach(p, robot=ROLES[role], touch_links=tooling.CUP_CONTACTS)
            for p in boards[i]
        ]

    def release(i, *, carrier=False):
        actions = [bt.seq.detach(p) for p in boards[i]]
        if carrier:
            actions += [
                bt.seq.attach(p, robot=movers[records[i]["carrier"]], link="nest")
                for p in boards[i]
            ]
        return actions

    def vertical(builder, name, role, work, hover, *, up):
        tags = [work, *(f"{work}_z{k}" for k in range(1, 5)), hover]
        if not up:
            tags.reverse()
        for k, tag in enumerate(tags[1:]):
            builder.step(f"{name}_{k}", actions=[move(role, tag, 0.12)])

    for cycle in range(count + DELAYS["sort"]):
        active = {
            role: cycle - delay
            for role, delay in DELAYS.items()
            if 0 <= cycle - delay < count
        }
        sq.step(
            f"dock{cycle}",
            actions=[
                bt.seq.set_signal(f"{ROLES[role]}_clear", False) for role in active
            ],
        )
        sq.step(
            f"approach{cycle}",
            actions=[
                move(role, f"magazine_hover{i}" if role == "feed" else "hover")
                for role, i in active.items()
            ],
        )
        sq.step(
            f"pick{cycle}",
            actions=[
                move(role, "work") for role, i in active.items() if role != "feed"
            ],
        )
        if "feed" in active:
            i = active["feed"]
            vertical(
                sq,
                f"magazine_lower{cycle}",
                "feed",
                f"magazine_pick{i}",
                f"magazine_hover{i}",
                up=False,
            )
        sq.step(
            f"grasp{cycle}",
            actions=[
                a
                for role, i in active.items()
                for a in take(i, role, detached=role == "feed")
            ],
            transition=bt.seq.elapsed(0.10),
        )
        sq.step(
            f"lift{cycle}",
            actions=[
                move(role, "hover") for role, i in active.items() if role != "feed"
            ],
        )
        if "feed" in active:
            i = active["feed"]
            vertical(
                sq,
                f"magazine_lift{cycle}",
                "feed",
                f"magazine_pick{i}",
                f"magazine_hover{i}",
                up=True,
            )
        transfer = {"feed": "hover", "aoi": "scan_a", "electrical": "fixture_hover"}
        sq.step(
            f"present{cycle}",
            actions=[
                move(role, tag, 0.70)
                for role, tag in transfer.items()
                if role in active
            ],
        )
        sq.step(
            f"place{cycle}",
            actions=[
                move(role, tag)
                for role, tag in (("feed", "work"), ("electrical", "fixture_work"))
                if role in active
            ],
        )
        actions = []
        if "feed" in active:
            i = active["feed"]
            actions += release(i, carrier=True) + [
                bt.seq.set_signal(f"pcb{i:02d}_loaded"),
                bt.seq.set_signal(f"carrier{records[i]['carrier']:02d}_occupied"),
            ]
        if "electrical" in active:
            actions += release(active["electrical"])
        sq.step(f"deposit{cycle}", actions=actions, transition=bt.seq.elapsed(0.10))
        sq.step(
            f"withdraw{cycle}",
            actions=[
                move(role, "fixture_hover" if role == "electrical" else "hover", 0.70)
                for role in ("feed", "electrical")
                if role in active
            ],
        )
        if "electrical" in active:
            sq.step(
                f"fixture_clear{cycle}", actions=[move("electrical", "hover", 0.70)]
            )
            sq.step(
                f"clamp_close{cycle}",
                actions=[ramp(scene, "ict_clamp", [0.029, 0.029], 0.35)],
            )
            sq.step(
                f"clamped{cycle}",
                actions=[
                    bt.seq.set_signal("fixture_clamped"),
                    bt.seq.set_signal("electrical_busy"),
                ],
            )
        if "aoi" in active:
            sq.step(
                f"aoi_scan{cycle}",
                actions=[bt.seq.set_signal("aoi_busy"), move("aoi", "scan_b", 0.65)],
            )
        if "aoi" in active or "electrical" in active:
            sq.step(f"inspect{cycle}", transition=bt.seq.elapsed(0.45))
            actions = []
            for role, field, busy in (
                ("aoi", "aoi", "aoi_busy"),
                ("electrical", "electrical", "electrical_busy"),
            ):
                if role in active:
                    i = active[role]
                    actions += [
                        bt.seq.set_signal(
                            f"pcb{i:02d}_{field}_ok", records[i][f"{field}_ok"]
                        ),
                        bt.seq.set_signal(f"pcb{i:02d}_{field}_done"),
                        bt.seq.set_signal(busy, False),
                    ]
            sq.step(f"results{cycle}", actions=actions)
        if "electrical" in active:
            i = active["electrical"]
            sq.step(
                f"clamp_open{cycle}",
                actions=[
                    bt.seq.set_signal("fixture_clamped", False),
                    ramp(scene, "ict_clamp", [0, 0], 0.35),
                ],
            )
            sq.step(
                f"fixture_approach{cycle}",
                actions=[move("electrical", "fixture_hover", 0.70)],
            )
            sq.step(
                f"fixture_pick{cycle}", actions=[move("electrical", "fixture_work")]
            )
            sq.step(
                f"fixture_grasp{cycle}",
                actions=take(i, "electrical", detached=True),
                transition=bt.seq.elapsed(0.10),
            )
            sq.step(
                f"fixture_lift{cycle}", actions=[move("electrical", "fixture_hover")]
            )
        returning = [role for role in ("aoi", "electrical") if role in active]
        if returning:
            sq.step(
                f"return{cycle}",
                actions=[move(role, "hover", 0.70) for role in returning],
            )
            sq.step(
                f"replace{cycle}", actions=[move(role, "work") for role in returning]
            )
            sq.step(
                f"release{cycle}",
                actions=[
                    a for role in returning for a in release(active[role], carrier=True)
                ],
                transition=bt.seq.elapsed(0.10),
            )
            sq.step(
                f"retract{cycle}", actions=[move(role, "hover") for role in returning]
            )
        if "sort" in active:
            i = active["sort"]
            # Refuse an untested board. Routing uses persistent inspection signals,
            # rather than the cycle number or a preselected animation path.
            sq.step(
                f"sort_ready{cycle}",
                transition=bt.seq.all_of(
                    bt.seq.signal(f"pcb{i:02d}_aoi_done"),
                    bt.seq.signal(f"pcb{i:02d}_electrical_done"),
                ),
            )
            selection = sq.select(f"sort_decide{cycle}")
            for result, condition in (
                (
                    "good",
                    bt.seq.all_of(
                        bt.seq.signal(f"pcb{i:02d}_aoi_ok"),
                        bt.seq.signal(f"pcb{i:02d}_electrical_ok"),
                    ),
                ),
                ("reject", bt.seq.otherwise()),
            ):
                branch = selection.when(condition)
                branch.step(
                    f"sort_{result}_approach{cycle}",
                    actions=[move("sort", f"{result}_hover{i}", 0.70)],
                )
                vertical(
                    branch,
                    f"sort_{result}_place{cycle}",
                    "sort",
                    f"{result}_place{i}",
                    f"{result}_hover{i}",
                    up=False,
                )
                branch.step(
                    f"sort_{result}_release{cycle}",
                    actions=release(i)
                    + [
                        bt.seq.set_signal(f"pcb{i:02d}_{result}"),
                        bt.seq.set_signal(f"pcb{i:02d}_complete"),
                        bt.seq.set_signal(
                            f"carrier{records[i]['carrier']:02d}_occupied", False
                        ),
                    ],
                    transition=bt.seq.elapsed(0.10),
                )
                vertical(
                    branch,
                    f"sort_{result}_retract{cycle}",
                    "sort",
                    f"{result}_place{i}",
                    f"{result}_hover{i}",
                    up=True,
                )
            sq.step(f"sort_home{cycle}", actions=[move("sort", "hover", 0.70)])
        sq.step(
            f"clear{cycle}",
            actions=[bt.seq.set_signal(f"{name}_clear") for name in STATIONS],
        )
        sq.step(
            f"gate{cycle}",
            transition=bt.seq.all_of(
                *(bt.seq.signal(f"{name}_clear") for name in STATIONS),
                bt.seq.signal("fixture_clamped", False),
            ),
        )
        if cycle == count + DELAYS["sort"] - 1:
            break
        sq.step(f"depart{cycle}", actions=[bt.seq.set_signal("indexing")])
        cuts = {0.0, loop.pitch}
        for i in range(loop.count):
            start = (i - cycle) * loop.pitch
            cuts.update(
                round(start - p, 10) for p in loop.cuts(start - loop.pitch, start)[1:-1]
            )
        cut_list = sorted(cuts)
        for j, (a, b) in enumerate(zip(cut_list, cut_list[1:])):
            sq.step(
                f"index{cycle}_{j}",
                actions=[
                    ramp(
                        scene,
                        name,
                        loop.q((i - cycle) * loop.pitch - b),
                        max(0.12, (b - a) / 0.32 * 1.5),
                    )
                    for i, name in enumerate(movers)
                ],
            )
        sq.step(f"arrived{cycle}", actions=[bt.seq.set_signal("indexing", False)])
    sq.step(
        "batch_complete",
        actions=[bt.seq.set_signal("batch_done")],
        transition=bt.seq.all_of(
            *(bt.seq.signal(f"pcb{i:02d}_complete") for i in range(count))
        ),
    )
    return ["pcb_cell"]


def bake(catalog_root=None, *, board_count=8):
    scene, info = build(catalog_root, board_count=board_count)
    names = programs(scene, info)
    timeline = scene.simulate_sequences(names, dt=0.04, max_duration=400.0)
    if not timeline.signal("batch_done").value_at(timeline.duration):
        raise RuntimeError("PCB batch did not complete")
    return scene, timeline, info


def report(scene, tl, info):
    error = 0.0
    completed = []
    for i, record in enumerate(info["results"]):
        expected = info["equipment"]["outputs"][record["destination"]][i][0]
        actual = tl.object_pose(info["boards"][i][0], tl.duration)[0]
        error = max(error, math.dist(expected, actual))
        completed.append(
            {
                **record,
                "complete": tl.signal(f"pcb{i:02d}_complete").value_at(tl.duration),
            }
        )
    return {
        "duration_s": round(tl.duration, 3),
        "carriers": info["loop"].count,
        "boards": info["board_count"],
        "index_steps": len(tl.signal("indexing").high_spans()),
        "good": sum(
            tl.signal(f"pcb{i:02d}_good").value_at(tl.duration)
            for i in range(info["board_count"])
        ),
        "reject": sum(
            tl.signal(f"pcb{i:02d}_reject").value_at(tl.duration)
            for i in range(info["board_count"])
        ),
        "max_output_position_error_m": error,
        "initial_collisions": scene.check_collisions(),
        "catalog_root": str(info["root"]) if info["root"] else None,
        "results": completed,
        "process": "magazine -> geometry-driven AOI -> simulated resistance test clamp -> good/reject racks",
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("out", nargs="?", default=str(HERE / "pcb_loop.usdc"))
    parser.add_argument("--studio", action="store_true")
    parser.add_argument(
        "--catalog-root",
        type=Path,
        default=None,
        help="Use locally built packages when available (default: published catalog)",
    )
    parser.add_argument(
        "--boards",
        "--cycles",
        dest="boards",
        type=int,
        default=8,
        help="Magazine batch size (1..8); every board is processed and discharged",
    )
    parser.add_argument(
        "--overview",
        action="store_true",
        help="Show the entire cell instead of the reference crop",
    )
    parser.add_argument(
        "--report", type=Path, help="Write the machine-readable run report"
    )
    parser.add_argument(
        "--project", type=Path, help="Save an editable .botrail project"
    )
    args = parser.parse_args()
    scene, tl, info = bake(args.catalog_root, board_count=args.boards)
    result = report(scene, tl, info)
    print(json.dumps(result, indent=2))
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + "\n")
    tl.export_usd(args.out, fps=30)
    print(f"wrote {args.out}")
    if args.project:
        scene.save_project(args.project)
    if args.studio:
        bt.studio(scene, view=OVERVIEW if args.overview else REFERENCE_VIEW)


if __name__ == "__main__":
    main()
