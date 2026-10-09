"""A screwdriving cell built in MISUMI aluminium frame: base, guard, shutter.

The equipment most aluminium frame is bought for: a base of extrusions
with a plate on top, the robot and its fixtures on the plate, and a guard
over them — a box of profiles with clear panels set into their slots and a
loading window across the front, closed by a shutter that lifts. Here every
member is cut from MISUMI's 6 series pack (`misumi/hfs/6-series`, HFS6):
the base from 60 x 60 legs and 30 x 60 rails (`frame_unit(template=
"table")`), the guard from 30 x 30 (`frame_unit(template="enclosure")`),
both black-anodised — so the bill of materials is the cut list you order
by (`HFSB6-3030-905` x3 …), the brackets, bolts, nuts and caps counted
from the joints (one line per article across both frames), and the panels
and the shutter's leaf listed by size.

Inside, the arm, the tools, the joint and the teaching of
`cover_bolting_demo.py`: a UR5e with an OnRobot RG6 and Screwdriver 103961
on one wrist fits a gearbox cover over its dowels and drives six M5 screws
in the star order the joint derives — the work a little nearer the arm,
and the arm parked folded over its base rather than stretched out, so the
park does not set the guard's size. The guard adds the cell's own program
around it: the shutter closes, the arm starts only once the shutter's
closed switch is made and the E-stop is out, and the shutter opens when
the arm is home.

What the bake says, beyond the tightening sheet: the clearance from
everything the arm and what it carries do to every face of the guard
(`min_clearance(to=)` on the profiles and the panels of that face) against
the 50 mm the guard is designed to keep. `--fit` is how the guard got its
size: it builds a roomy guard 150 mm out on every face, bakes, measures
each face, moves each face to keep 50 mm (a 5 mm grid, and tall enough for
the shutter to lift clear), builds and bakes again to confirm. The guard
below is what it found: W 965 x D 1000 x H 760 mm on a 740 mm base.

What is handed over (`deliver`, next to the USD): the layout sheet, the
bill, the I/O list with the shutter's switches, the PLCopen file, the
interlock table, the tightening sheet and the cell report with the FAT
rows — the shutter jammed open (the arm never starts), the E-stop in, the
presenter empty, the START wire to the driver open (each must refuse the
cycle), a NOK retried (passes) and a NOK twice (halts).

Run with:  python examples/assembly/misumi_frame_cell_demo.py [out.usdc]
                 [--fit [MM]] [--studio]

Needs the catalog (`pip install botrail[catalog]`): the frame pack, the arm,
its tools and the cell's products are fetched from botrail/botrail-catalog.
The guard is sized to the program, not to the arm's reach: on the real cell
the arm's safety configuration (UR safety planes) keeps it inside.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import _cover_bolting_cell as appearance
import botrail as bt
import cover_bolting_demo as bolting

HERE = Path(__file__).resolve().parent
A = bt.assembly
S = bt.seq
ROBOT = bolting.ROBOT

# --------------------------------------------------------------- products
FRAME = "misumi/hfs/6-series"   # HFS6: profiles by the 0.5 mm, brackets, bolts, nuts, caps

# -------------------------------------------------------------- equipment
BASE_H = 0.74       # the base frame, to the top of its ring
PLATE = 0.020       # the plate the arm and the fixtures stand on
ADAPTER = 0.005     # the arm's adapter plate
WINDOW = 0.35       # the loading window across the guard's front, from the plate up
SHUTTER_SPEED = 0.25
CLEARANCE = 0.050   # what the guard keeps from everything the arm does
# Where the guard's faces stand, measured from the arm's base (it faces
# -y): left -x, right +x, front -y, back +y, and the top above the plate.
# `--fit` measured them.
GUARD = {"left": 0.475, "right": 0.49, "front": 0.84, "back": 0.16, "top": 0.76}
FACING = (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5))   # the arm's +x turned to -y
# Where the arm parks between cycles: folded over its base, the tools
# down — not cover_bolting's stretched park, which would set the guard's
# height and front by itself.
PARK = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# --------------------------------------------------------------- the work
# From the arm's base: the housing ahead of it, the cover's stock to its
# right, the presenter to its left.
HOUSING_XY = (0.0, -0.50)
STOCK_XY = (-0.28, -0.50)
FEEDER_XY = (0.28, -0.34)


# ------------------------------------------------------------------ build
def outline(guard: dict) -> tuple[float, float, float, float]:
    """The footprint the guard and the base share: width, depth and centre."""
    w, d = guard["left"] + guard["right"], guard["front"] + guard["back"]
    return w, d, (guard["right"] - guard["left"]) / 2, (guard["back"] - guard["front"]) / 2


def estop(scene: bt.Scene, at) -> str:
    """The XALK178F station on the base's front-right leg, at hand beside
    the window. Returns its E-stop lane."""
    panel = bt.parts.operator_panel(scene, "panel", at, buttons=("estop",), catalog=bolting.PANEL_CATALOG,
                                    watch_robots=[ROBOT], source_url=appearance.PANEL_SOURCE,
                                    contacts="2 NC; one abstract E-stop lane in this demo")
    scene.set_part("panel/estop", kind="sensor", manufacturer="Schneider Electric",
                   model="XALK178F actuator (included)", category="hmi.button",
                   description="included in the complete control station, not a separate purchase")
    return panel.sensors[0]


def build(guard: dict = GUARD):
    """The cell in its guard, taught and programmed. Returns the scene, the
    joint, the driver, the feeder, the placement, the fastening and the
    E-stop lane."""
    scene = bt.Scene(bolting.hand(), name=ROBOT)
    robot = scene.robot_of(ROBOT)
    scene.set_part(f"{ROBOT}/tool", electrical_route="Compute Box; not robot wrist power")
    scene.set_part(f"{ROBOT}/tool/tool2", mass_kg=bolting.DRIVER_MASS, rpm=bolting.RPM_RUN,
                   accessory_mass="unknown; the extender, the bit kit and cables are not in the 2.5 kg")
    scene.set_part(f"{ROBOT}/tool/tool3", part_number="102021",
                   electrical_route="Dual QC 109878 / Compute Box, not a single-QC wrist-power kit")
    figures = bolting.datasheet(scene, f"{ROBOT}/tool/tool2")
    tool = {"torque_nm": (figures["torque_min_nm"], figures["torque_max_nm"]),
            "screw_length_mm": figures["screw_length_mm"], "bit_mm": figures["bit_mm"]}
    stroke = robot.joint_limits[robot.joint_names.index(bolting.SHANK)][1]

    # -- the base: 6060 legs, 3060 rails, a plate on the ring ----------------
    w, d, cx, cy = outline(guard)
    bt.parts.frame_unit(scene, "base", (w, d, BASE_H), (cx, cy), catalog=FRAME, legs="6060", rails="3060",
                        top=PLATE, finish="black")
    (_x, _y, top), _ = scene.frame("base/top")
    scene.set_part("base/top", category="structure.frame.panel",
                   model=f"plate A5052 t{PLATE * 1e3:g} {w * 1e3:.0f}x{d * 1e3:.0f} (machined)",
                   description="custom: drilled for the arm, the fixtures and the guard")

    # -- the guard on the plate, its window across the front ------------------
    bt.parts.frame_unit(scene, "guard", (w, d, guard["top"]), (cx, cy, top), catalog=FRAME, template="enclosure",
                        finish="black", openings={"front": ((None, None), (None, WINDOW))}, door="front")
    # The shutter stands open — the operator has just loaded — lifted off
    # the window by its own height, over the panel above it. A switch at
    # each end of the cylinder's stroke reads where the leaf is: the
    # closed one is what the arm's program waits on.
    (lx, ly, lz), _ = scene.obstacle_pose("guard/door")
    (_a, ay, az), (_b, by, bz) = scene.obstacle_bounds("guard/door")
    scene.set_obstacle_pose("guard/door", (lx, ly, lz + WINDOW))
    scene.add_linear_axis("shutter", objects=["guard/door"], axis=(0.0, 0.0, 1.0), speed=SHUTTER_SPEED,
                          range=(0.0, WINDOW), position=WINDOW)
    scene.set_part("shutter", kind="device", category="axis.linear",
                   model=f"shutter drive (air cylinder, {WINDOW * 1e3:.0f} mm stroke)",
                   description="cylinder not selected: sized from the leaf and its stroke")
    for stop, edge in (("closed", az - 0.0025), ("open", bz + WINDOW + 0.0025)):
        scene.add_zone_sensor(f"shutter/{stop}", (lx, ly, edge), (0.02, by - ay + 0.01, 0.015), watch=["guard/door"])
        scene.set_part(f"shutter/{stop}", kind="sensor", category="sensor.proximity",
                       model="cylinder auto switch", description="on the shutter cylinder; not selected")

    # -- the arm on its adapter ------------------------------------------------
    scene.set_robot_base_pose((0.0, 0.0, top + ADAPTER), FACING, robot=ROBOT)
    appearance.box(scene, "base/adapter", (0.19, 0.19, ADAPTER), (0.0, 0.0, top + ADAPTER / 2),
                   appearance.METAL, metal=0.85)
    scene.set_part("base/adapter", manufacturer="botrail", model="UR5e adapter plate (custom)", category="adapter",
                   description="5 mm demo plate; mounting pattern requires engineering")
    scene.allow_link_obstacle_contact(robot.link_names[0], "base/top", robot=ROBOT)

    # -- the housing on its nest, the cover at its stock, the presenter -------
    hx, hy = HOUSING_XY
    sx, sy = STOCK_XY
    fx, fy = FEEDER_XY
    work = bt.parts.workpiece(scene, "set", (hx, hy, top), catalog=bolting.WORKPIECE_CATALOG, cover_at=(sx, sy, top))
    dowels = list(work.dowels)
    screw = A.iso4762(5, bolting.SCREW_LENGTH_MM)
    threads = A.BoltPattern(
        [A.Hole(f"h{i}", xy, "threaded", diameter_mm=5.0, depth_mm=bolting.THREAD_DEPTH_MM)
         for i, xy in enumerate(bolting.HOLES_MM)],
        [A.Locator(f"p{i}", xy, "pin", bolting.DOWEL[0] * 1e3, bolting.DOWEL[1] * 1e3)
         for i, xy in enumerate(bolting.DOWELS_MM)],
    )
    clearances = A.BoltPattern(
        [A.Hole(f"h{i}", xy, "clearance", diameter_mm=5.5, grip_mm=bolting.COVER[2] * 1e3)
         for i, xy in enumerate(bolting.HOLES_MM)],
        [A.Locator(f"p{i}", xy, "hole", bolting.DOWEL[0] * 1e3 + 0.1, 10.0) for i, xy in enumerate(bolting.DOWELS_MM)],
    )
    joint = A.joint(scene, "cover_joint", a=work.housing, b=work.cover, pattern_a=threads, pattern_b=clearances,
                    fastener=screw, seat=work.seat, thickness=bolting.COVER[2],
                    torque_nm=bolting.TORQUE_NM, min_engagement_mm=bolting.MIN_ENGAGEMENT_MM)
    for link in bolting.pads(robot):
        scene.allow_link_obstacle_contact(link, joint.b, robot=ROBOT)
    appearance.fixtures(scene, housing_xy=HOUSING_XY, stock_xy=STOCK_XY, top=top,
                        housing_size=bolting.HOUSING, cover_size=bolting.COVER)
    screws = [screw.place(scene, f"screw{i}", (fx, fy, top), manufacturer="botrail") for i in range(len(joint.order))]
    feeder = bt.parts.screw_feeder(scene, "feeder", (fx, fy, top), screws=screws, catalog=bolting.FEEDER_CATALOG)
    for name in feeder.screws:
        scene.allow_link_obstacle_contact(bolting.BIT, name, robot=ROBOT)

    # -- the E-stop beside the window; the controllers inside the base ---------
    lane = estop(scene, (cx + w / 2 - 0.03, cy - d / 2 - 0.026, BASE_H - 0.12))
    bt.parts.controller(scene, "controller", robots=[ROBOT], position=(cx - 0.15, cy + 0.05),
                        catalog=bolting.CONTROLLER_CATALOG, programs=["assemble"], cable_m=6)
    driver = A.driver(scene, "driver", robot=ROBOT, bit=bolting.BIT, shank=bolting.SHANK, stroke=stroke,
                      fastener=joint.fastener, torque_nm=bolting.TORQUE_SET, cycles=len(joint.order) + bolting.RETRY,
                      rpm_run=bolting.RPM_RUN, tool=tool, estop=lane,
                      channels=bt.io.channels("di", 8, "DI") + bt.io.channels("do", 8, "DO"))
    appearance.compute_box(scene, driver.node, (cx + 0.25, cy + 0.10, 0.0))
    scene.add_spin("driver/spin", driver.signal("run"), ROBOT, link=bolting.BIT)
    problems = [f for f in A.check(joint, driver) if f.severity == "error"]
    if problems:
        raise ValueError("; ".join(f.message for f in problems))

    close = bolting.teach(scene, joint, feeder, driver, ready=PARK)
    placement, fastening = program(scene, joint, driver, feeder, close, lane,
                                   touch=[joint.a, *dowels, "base/top"])
    wiring = scene.auto_assign_io(["assemble", driver.program])
    if wiring.errors():
        raise ValueError("wiring: " + "; ".join(f.message for f in wiring.errors()))

    # -- the FAT rows: each fault a scenario that must refuse the cycle --------
    scene.add_scenario("nok_once", signals={driver.signal("fault_first"): True})
    scene.add_scenario("nok_twice", signals={driver.signal("fault_first"): True, driver.signal("fault_retry"): True})
    scene.add_scenario("estop_pressed", faults=[bt.io.stuck(lane, True)])
    reject = (cx + w / 2 + 0.30, cy)
    scene.add_scenario("feeder_empty", obstacles={
        s: (reject[0], reject[1] + 0.02 * i, 0.0) for i, s in enumerate(feeder.screws)})
    scene.add_scenario("start_wire_open", faults=[bt.io.open(driver.signal("start"))])
    scene.add_scenario("shutter_jammed", faults=[bt.io.stuck("shutter/closed", False)])
    return scene, joint, driver, feeder, placement, fastening, lane


def program(scene: bt.Scene, joint: A.Joint, driver: A.Driver, feeder: bt.parts.ScrewFeeder, close: dict,
            estop_lane: str, *, touch: list[str]):
    """The arm's program inside the guard's: the shutter closes, the arm
    waits for its closed switch and the E-stop out, fits the cover and
    drives the screws (the cycle of `cover_bolting_demo`), goes home, and
    the shutter opens."""
    finger = scene.robot_of(ROBOT).joint_names[-1]
    sq = scene.sequence("assemble")
    sq.step("close_shutter", actions=[S.move_to("shutter", 0.0)], transition=S.device_done("shutter"))
    sq.step("guarded", transition=S.all_of(S.signal("shutter/closed"), S.signal(estop_lane, False)))
    placement = A.place(sq, joint.b, motions={
        "approach": "cover_approach", "down": "cover_down", "up": "cover_up",
        "carry": "cover_carry", "seat": "cover_seat", "clear": "cover_clear",
    }, close=close, open={finger: 0.0}, expected=joint.seat, touch=touch, robot=ROBOT)
    fastening = A.fasten(sq, joint, driver, feeder, motions={
        "to_pick": "screw_to_pick", "pick": "screw_pick", "lift": "screw_lift",
        "over": "screw_over_{hole}", "engage": "screw_engage_{hole}", "clear": "screw_clear_{hole}",
    }, retry=bolting.RETRY, robot=ROBOT, after=[placement.seated])
    sq.step("home", actions=[S.motion("home"), S.set_signal(driver.signal("finish"))])
    sq.step("open_shutter", actions=[S.move_to("shutter", WINDOW)], transition=S.device_done("shutter"))
    return placement, fastening


def bake(guard: dict = GUARD):
    scene, joint, driver, feeder, placement, fastening, lane = build(guard)
    tl = scene.simulate_sequences(["assemble", driver.program], max_duration=240.0)
    return scene, tl, joint, driver, feeder, placement, fastening


# -------------------------------------------------------------- the guard
FACES = ("left", "right", "front", "back", "top")


def faces(scene: bt.Scene, name: str = "guard") -> dict[str, list[str]]:
    """The guard's residents a robot can hit, by the face they stand in:
    a profile or a panel lying wholly within one section of a face belongs
    to it (a corner post to two). The shutter stands outside the front."""
    names = [n for n in scene.obstacle_names
             if n.startswith(f"{name}/") and scene.obstacle_enabled(n) and n != f"{name}/door"]
    bounds = {n: scene.obstacle_bounds(n) for n in names}
    lo = [min(b[0][k] for b in bounds.values()) for k in range(3)]
    hi = [max(b[1][k] for b in bounds.values()) for k in range(3)]
    reach = 0.031   # a 30 mm section, and a hair
    out: dict[str, list[str]] = {face: [] for face in FACES}
    for n, (a, b) in bounds.items():
        for face, inside in (("left", b[0] <= lo[0] + reach), ("right", a[0] >= hi[0] - reach),
                             ("front", b[1] <= lo[1] + reach), ("back", a[1] >= hi[1] - reach),
                             ("top", a[2] >= hi[2] - reach)):
            if inside:
                out[face].append(n)
    return out


def clearances(scene: bt.Scene, tl: bt.SequenceTimeline, dt: float = 0.02) -> dict[str, bt.Clearance]:
    """The tightest the arm and what it carries come to each face over the
    cycle."""
    return {face: tl.min_clearance(dt, to=names) for face, names in faces(scene).items()}


def fit(guard: dict, measured: dict, target: float = CLEARANCE) -> dict:
    """Moves each face of `guard` in (or out) so it keeps `target` from what
    was `measured`, on a 5 mm grid — and keeps the guard tall enough for the
    shutter to lift clear of the window under the top ring."""
    out = dict(guard)
    for face in FACES:
        slack = float(measured[face]) - target
        out[face] = math.ceil(round((guard[face] - slack) / 0.005, 6)) * 0.005
    out["top"] = max(out["top"], 2 * WINDOW + 0.06)
    return out


def frame_summary(scene: bt.Scene) -> dict:
    """The guard's and the base's outline and what they are cut from."""
    rows = [r for r in scene.bom().rows if any(n.startswith(("guard", "base")) for n in r["names"])]
    profiles = [r for r in rows if r["category"] == "structure.frame.profile"]
    length = sum(float(r["model"].rsplit("-", 1)[1]) * r["qty"] for r in profiles) / 1000.0
    mass = sum((r["attributes"].get("mass_kg") or 0.0) * r["qty"] for r in rows
               if r["category"].startswith("structure.frame.") and r["category"] != "structure.frame.panel")
    brackets = sum(r["qty"] for r in rows if r["model"] == "HBLFS6")
    return {"profiles_m": length, "profile_lines": len(profiles), "frame_kg": mass, "brackets": brackets}


# ------------------------------------------------------------- hand-over
def deliver(scene: bt.Scene, tl: bt.SequenceTimeline, fastening: A.Fastening, placement: A.Placement,
            out: Path):
    """The document set, written into `out` from the one source, and the
    report that hashes it. Returns `(report, runs)`."""
    out.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []
    stem = "misumi_frame_cell"

    def write(name: str, fn) -> Path:
        path = out / f"{stem}{name}"
        fn(path)
        files.append(path)
        return path

    rows = A.fastening_report(tl, fastening)
    fit_ = A.fit_report(tl, placement)
    write(".botrail", scene.save_project)
    write(".py", lambda p: p.write_text(scene.generate_python()))
    write("_bom.csv", scene.export_bom)
    write("_bom.md", scene.export_bom)
    write("_io.csv", scene.export_io_list)
    write("_topology.mmd", scene.export_topology)
    write("_handshake.md", tl.export_handshake_spec)
    write(".plcopen.xml", lambda p: scene.export_plcopen(p, name="misumi frame cell"))
    write("_interlocks.md", scene.export_interlocks)
    # The equipment's layout: the taught frames (holes, picks) stay off it.
    write("_layout.svg", lambda p: scene.export_layout(p, scale=120, frames=False, title="misumi frame cell"))
    write("_layout.dxf", lambda p: scene.export_layout(p, frames=False, title="misumi frame cell"))
    write("_tightening.md", lambda p: A.export_sheet(p, fastening, rows=rows, title="misumi frame cell"))
    write("_fastening.json", lambda p: p.write_text(json.dumps({"fit": fit_, "screws": rows}, indent=2)))
    runs = scene.simulate_scenarios(["assemble", fastening.driver.program], max_duration=tl.duration + 30.0)
    report = scene.cell_report({"baseline": tl}, scenarios=runs, deliverables=files, title="misumi frame cell",
                               sections=[A.report_section(fastening, rows, fit_)])
    report.save(out / f"{stem}_report.md")
    report.save(out / f"{stem}_report.json")
    return report, runs


def cut_list(scene: bt.Scene) -> list[str]:
    """The frames' lines of the bill: the profiles by length, the hardware,
    the sheets — what goes on the order."""
    kinds = ("structure.frame.profile", "structure.frame.hardware", "structure.frame.panel", "structure.door")
    out = []
    for row in scene.bom().rows:
        if row["category"] in kinds:
            kg = row["attributes"].get("mass_kg")
            out.append(f"  {row['model']:<40} x{row['qty']:<3}" + (f" {kg:.3f} kg each" if kg else ""))
    return out


def describe(scene: bt.Scene, tl: bt.SequenceTimeline, guard: dict) -> dict:
    """Prints the guard's outline, its clearances and its frame; returns the clearances."""
    w, d, _cx, _cy = outline(guard)
    measured = clearances(scene, tl)
    summary = frame_summary(scene)
    print(f"guard W {w * 1e3:.0f} x D {d * 1e3:.0f} x H {guard['top'] * 1e3:.0f} mm on a {BASE_H * 1e3:.0f} mm base "
          f"({w * d:.3f} m² of floor), {summary['profiles_m']:.2f} m of profile in {summary['profile_lines']} cuts, "
          f"{summary['brackets']} brackets, frame {summary['frame_kg']:.1f} kg")
    for face in FACES:
        c = measured[face]
        flag = "ok" if float(c) >= CLEARANCE - 1e-9 else f"under {CLEARANCE * 1e3:.0f} mm"
        print(f"  {face:<6} {guard[face] * 1e3:5.0f} mm from the arm, closest {float(c) * 1e3:6.1f} mm at {c.t:6.2f}s  {flag}")
    return measured


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "misumi_frame_cell.usdc"))
    parser.add_argument("--fit", nargs="?", type=float, const=CLEARANCE * 1e3, default=None, metavar="MM",
                        help="fit the guard to keep MM (default 50) from the arm, from a roomy one")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    guard = GUARD
    if args.fit is not None:
        roomy = {face: value + 0.15 for face, value in GUARD.items()}
        print("roomy guard:")
        scene, tl, *_ = bake(roomy)
        measured = describe(scene, tl, roomy)
        guard = fit(roomy, measured, args.fit / 1e3)
        print(f"fitted to {args.fit:g} mm: GUARD = {json.dumps({k: round(v, 3) for k, v in guard.items()})}")

    scene, tl, joint, driver, feeder, placement, fastening = bake(guard)
    print(f"misumi frame cell: cycle {tl.duration:.2f}s, {len(fastening.pairs)} screws in the order {joint.order}")
    for lane in ("shutter/closed", "shutter/open", driver.signal("run")):
        spans = ", ".join(f"{a:.2f}-{b:.2f}" for a, b in tl.signal(lane).high_spans())
        print(f"  {lane:<16} on: {spans or '-'}")
    describe(scene, tl, guard)
    print("cut list:")
    print("\n".join(cut_list(scene)))
    rows = A.fastening_report(tl, fastening)
    print(f"fastening: {sum(r['result'] == 'ok' for r in rows)}/{len(rows)} ok")

    warnings = tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}" + (f" ({warnings})" if warnings else ""))
    out_dir = Path(args.out).with_name("misumi_frame_cell_deliverables")
    report, runs = deliver(scene, tl, fastening, placement, out_dir)
    for name in ("baseline", *scene.scenario_names):
        verdict = runs.errors.get(name)
        print(f"  scenario {name:<16} {'ok' if verdict is None else 'refused — ' + verdict}")
    print(f"wrote the document set to {out_dir}/ ({len(report.deliverables)} files hashed in the report)")
    if args.studio:
        # The scenario matrix was the last thing baked; open on the nominal cycle.
        scene.show_timeline(tl)
        bt.studio(scene)


if __name__ == "__main__":
    main()
