"""Cables and tubes of a small UR cell — the routing MVP.

A UR5e on a pedestal with a suction hand, its control box by the fence,
the PLC cabinet in the corner, a valve manifold on the pedestal, two floor
ducts. Three lines run between them, declared once:

* the **robot cable**, base connector → control box: clipped down the
  pedestal's column (``rt.along`` lays the clamps — no way points written
  for the climb, and the clamps are a BOM line), then a floor duct;
* the **PLC link** (PROFINET), control box → cabinet, in a duct of its own
  beside it — a power line and a network line do not share one
  (``--shared-duct`` runs it through the robot cable's and the report says
  so: ``route_separation``);
* the **air tubes** to the hand, two of them, dressed the way a cobot's
  tool lines are in the field (a screwdriver cell's photo was the model):
  valve → a P-clip each on the robot's base → tied along the front of the
  upper arm and the underside of the forearm (``rt.along`` lays a clip
  every 20 cm, the two tubes side by side) → the ejector's elbow fittings.
  Free only in the loops the axes need — the base's turn, the elbow, the
  wrist — and replayed as tubes over the cycle.

Everything else is derived from where things stand: the centerline with
its bends (the spec's minimum bend radius as the fillet), the cut and the
purchase length, the cable list, the BOM rows, the checks, the layout
sheet's ROUTE layer and the USD `BasisCurves`. The fixed spans are
colliders too: the planner routes the arm around the robot cable, and a
bake that brushed it would say so. Move the control box
(``--move-box 3``) and the robot cable's length follows — past 6 m the
``cable_m`` requirement reports the standard cable as short (the e-Series
box is sold with a 6 m cable, 12 m as an option).

A short cycle swings the arm out to its side (the first axis 57°) and
turns the hand 90°; the air tubes' free spans are replayed as tubes
against it (``bt.routing.animate`` — each line straight at rest, the way it
comes off the reel, and a rod rather than a string, so a loop holds a
round shape between its clips; all of them side by side) and judged over
time — the
free length the clips need, the stroke, the twist, the tightest bend, the
sag, the nearest body — each finding with its time, which the studio's
ROUTING tab seeks to. The replay collides with the arm — the tubes rest on
it, never pass through it — and the report judges every link but a clip's
own mount. Each loop is as long as its axes ask and no longer: the base
loop takes the first axis's turn as twist and stroke (0.25 m between its
clip and the first tie on the upper arm, 12 cm up the link so the loop has
a chord to hang from), the elbow loop a 6° bend (it lies on the elbow's
housing, the link its own clip is on: a warning), the wrist loop the
hand's 90° turn and the stroke of the port swinging round the flange —
a drip loop under the wrist, from the forearm's last clip to the ejector's
elbow fittings, which point up the hand's axis so the hand's turn swings
them round without turning them. Ask for more and the loops outgrow their
room: a 75° swing of the first axis brings the base loop to 3 mm of the
casting (``route_clearance``); a 149° turn of the hand wants 0.41 m of
tube at the wrist for its twist alone (``route_twist`` with the 0.28 m it
has), and a loop that long hangs 140 mm and lies on the forearm — the cell
then takes a swivel at the flange or a retraction, not a longer loop.

The lines, the conduit, the clamps and the ducts are catalog articles —
SMC TU0604BU, Lapp ETHERLINE FESTOON PN Cat.5e, ABB Adaptaflex PAFS21 /
PRFS16, HellermannTyton H6P, OBO LK4 60100 — read with ``catalog=`` from
the botrail catalog (``--catalog-root DIR`` reads the same ids from a local
root, the builder's ``build/`` or ``dist/``).
``--dress-conduit pafs21`` dresses the two tubes in a corrugated conduit
instead of the ties along the arm, held 12 cm up the upper arm and at the
forearm's wrist end: one hose between them, replayed as one. ``--dress-conduit prfs16`` picks the smaller conduit: two
6 mm tubes take 53 % of its 11.7 mm bore and the bundle is reported
(``duct_fill``), the kit row short of its ``inner_mm``. The dress pack is
then a line of ``scene.requirements()``: what this arm and this cycle ask
of a kit — the bore, the hose length between the holders, the stroke a
retraction would take up, the twist per metre — next to what the declared
hose states (and what ``bt.catalog.search_for`` would look for).
``--sweep-holder`` (with the conduit) moves the forearm holder along the
link and tables the stroke and the twist at each place, without re-baking.

Run with:  python examples/engineering/routing_demo.py [out.usd] [--studio] [--move-box METERS] [--dress-conduit pafs21|prfs16] [--shared-duct] [--sweep-holder] [--solver physics|static] [--catalog-root DIR]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import botrail as bt

UR5E = "universal_robots/ur/ur5e/r2"
CONTROL_BOX = "universal_robots/control-box/e-series/r1"
CABINET = "nito/fz/standard"
BOX_AT = (1.3, -0.9)
CABINET_AT = (-1.4, -1.0)
HOME = [0.0, -1.3, 1.2, -1.5, -1.57, 0.0]
# The valve manifold stands on the pedestal's top plate beside the base,
# side-ported toward it, clear of the loop that rises from the base to the
# upper arm.
VALVE_AT = (0.135, 0.10, 0.55)
VALVE_SIZE = (0.05, 0.12, 0.10)
# Where the air tubes are tied: a clip each on the robot's base casting
# toward the upper arm's sweep — by the first axis, so the base's turn
# hardly changes the way up to the arm — (link, offset in the link's frame,
# and half the step to the second tube's clip beside it), then along the
# front face of the upper arm and the underside of the forearm (the links'
# -y faces at the home pose), a clip every 20 cm, the two tubes 7 mm either
# side of the run. The upper arm's run starts 12 cm up the link: the loop
# from the base needs that much chord, or it hangs onto the pedestal's
# plate. Every clip stands its own standoff and the tube's radius off the
# link's collision shape: a clip inside a link is a line through the arm
# (the routing report warns).
BASE_TIE = ("base_link_inertia", (0.014, -0.077, 0.065), (0.0, 0.0, 0.007))
BASE_TIE_DIRECTION = (0.0, -0.4, 1.0)
ARM_RUNS = [("upper_arm_link", "-y", (0.12, 0.0)), ("forearm_link", "-y", (0.0, 0.0))]
TIE_PITCH_M = 0.2
TIE_ASIDE_M = 0.007
# The loops between the tied runs, base → upper arm → forearm → hand (m
# beyond the measured run): the base's turn (the 60° between its holders as
# twist wants 0.17 m of tube, the stroke 0.25 m), the elbow (6°), and the
# wrist (the hand's 90° turn, and the port swinging round the flange).
SLACK = [0.05, 0.05, 0.12]
# The second tube's clips TIE_STAGGER_M further in from both ends of each
# run, and its loops a little longer. The tubes lie side by side across the
# way their loops jump (up to the arm, across the elbow, round to the
# hand), so equal loops at the same stations lie in one plane and run into
# each other; staggered, the outer tube's loop nests round the inner one's.
TIE_STAGGER_M = 0.03
SLACK_OUTER = [0.06, 0.07, 0.14]
# With --dress-conduit, the conduit's holders on the same two faces, the
# hose's radius and a clip's standoff off the links; the hose's slack (its
# 60 mm bend radius counts twice in the length it needs).
DRESS_HOLDERS = [("upper_arm_link", (-0.12, -0.076, 0.133)), ("forearm_link", (-0.33, -0.074, 0.01))]
DRESS_SLACK = [0.06, 0.16, 0.14]
REACH = [1.0, -0.9, 1.1, -1.8, -1.57, 1.57]
# The ejector block on the stem's side (in the holder's frame), and its two
# inlets: elbow fittings on the block's outer face, turned up toward the
# flange, 7 mm either side of its middle — side by side across the hand, in
# the order the tubes are tied along the arm, so the pair runs parallel from
# the forearm's clips to the fittings. Turned up, the fittings point the
# tubes along the hand's axis: the hand's turn swings them round the flange
# but does not turn them, and the loop keeps its hang. Fittings pointing
# out of the hand's side would turn with it, and the loop would be dragged
# round into a hook.
EJECTOR_AT = (0.0, 0.035, 0.035)
INLET_AT = (0.007, 0.019, -0.010)
INLET_EXIT = (0, 0, -1)
INLET_STRAIGHT_M = 0.02


def _box_xml(size, xyz, rgba="0.78 0.78 0.80 1") -> str:
    geometry = f'<geometry><box size="{size[0]} {size[1]} {size[2]}"/></geometry>'
    origin = f'<origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}"/>'
    return (f"<visual>{origin}{geometry}<material name=\"m{abs(hash((size, xyz)))}\"><color rgba=\"{rgba}\"/></material></visual>"
            f"<collision>{origin}{geometry}</collision>")


def suction_hand() -> bt.Robot:
    """An SI-made holder: a flange plate, a stem, a cup bar with two cups,
    and the ejector block on the stem's side — where the air comes in, by
    two elbow fittings turned up toward the flange (EJECTOR_AT)."""
    holder = (_box_xml((0.06, 0.06, 0.01), (0, 0, 0.005))
              + _box_xml((0.02, 0.02, 0.05), (0, 0, 0.035))
              + _box_xml((0.10, 0.02, 0.01), (0, 0, 0.065))
              + _box_xml((0.02, 0.02, 0.012), (0.04, 0, 0.076), "0.15 0.15 0.17 1")
              + _box_xml((0.02, 0.02, 0.012), (-0.04, 0, 0.076), "0.15 0.15 0.17 1"))
    ejector = _box_xml((0.03, 0.022, 0.045), (0, 0, 0), "0.35 0.37 0.40 1")
    xml = (
        '<robot name="suction_hand">'
        f'<link name="holder">{holder}</link>'
        f'<link name="ejector">{ejector}</link>'
        '<joint name="ejector_mount" type="fixed"><parent link="holder"/><child link="ejector"/>'
        f'<origin xyz="{EJECTOR_AT[0]} {EJECTOR_AT[1]} {EJECTOR_AT[2]}"/></joint>'
        '<link name="suction_tcp"/>'
        '<joint name="suction_tcp_joint" type="fixed"><parent link="holder"/><child link="suction_tcp"/>'
        '<origin xyz="0 0 0.082"/></joint>'
        "</robot>"
    )
    return bt.Robot.from_urdf_string(xml)


# The articles, by catalog id (public on the botrail catalog). `--catalog-root`
# reads the same ids from a local root (the builder's `build/` or `dist/`).
PACKS = {
    "tube": "smc/tube/tu0604bu/r1",
    "profinet": "lapp/cable/etherline-festoon-pn-cat5e/r1",
    "pafs21": "abb/conduit/adaptaflex-pafs21/r1",
    "prfs16": "abb/conduit/adaptaflex-prfs16/r1",
    "clip": "hellermanntyton/clamp/h6p-pa66hs-bk/r1",
    "duct": "obo_bettermann/duct/lk4-60100/r1",
}


def article(key: str, catalog_root) -> dict:
    """``catalog=`` for one article: the hub id, or the same id under a local root."""
    pid = PACKS[key]
    return {"catalog": pid if catalog_root is None else str(Path(catalog_root) / pid)}


def build(box_shift: float = 0.0, dress_conduit=None, shared_duct: bool = False,
          catalog_root=None) -> bt.Scene:
    arm = bt.Robot.from_catalog(UR5E).attach_tool(suction_hand(), tcp="suction_tcp")
    scene = bt.Scene(arm, name="arm")
    bt.parts.pedestal(scene, "pedestal", height=0.5, position=(0.0, 0.0), model="PD-500", manufacturer="MISUMI")
    scene.set_robot_base_pose(*scene.frame("pedestal/mount"))
    # The base casting stands on the pedestal's top plate: intended contact,
    # not a collision the planner should refuse.
    scene.allow_link_obstacle_contact("base_link_inertia", "pedestal/top", robot="arm")
    scene.set_joint_positions(HOME)
    scene.set_part("arm/tool", kind="tool", category="gripper.vacuum",
                   model="Suction holder, SCPMc-05 ejector, 2× ZP3 cups",
                   description="Holder authored in the demo; the ejector and the cups are articles to order")

    # The boxes the lines run to: the UR control box, the PLC cabinet (with
    # its node), the valve manifold on the pedestal's side.
    box_at = (BOX_AT[0] + box_shift, BOX_AT[1])
    bt.parts.controller(scene, "UR", robots=["arm"], catalog=CONTROL_BOX, position=box_at, cable_m=6)
    bt.parts.cabinet(scene, "cabinet", catalog=CABINET, size=(0.6, 0.4, 1.6), position=CABINET_AT, base_height=0.1)
    scene.add_io_node("PLC1", kind="plc", channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"),
                      model="R04CPU", place="cabinet/body")
    scene.add_box("valve", size=VALVE_SIZE, position=VALVE_AT, color=(0.55, 0.57, 0.60))
    scene.set_part("valve", category="pneumatic.valve", manufacturer="SMC", model="SY3000 manifold, 2 stations")

    # Two floor ducts: the robot cable's, pedestal foot → control box, and
    # the PLC link's, control box → cabinet, laid beside it (its glands 13 cm
    # off centre, so the ducts do not cross). --shared-duct is the cell
    # short of a duct: the link runs through the robot cable's to the
    # pedestal's foot and on to the cabinet in a second one.
    # A duct's run reaches half its width past the path's end: end the legs
    # half a width short of the back faces and they touch them.
    box_end = scene.obstacle_bounds("UR/body")[1][1] + 0.05
    cab_end = scene.obstacle_bounds("cabinet/body")[1][1] + 0.05
    duct = article("duct", catalog_root)
    bt.parts.duct(scene, "duct_ur", path=[(0.25, -0.45), (box_at[0], -0.45), (box_at[0], box_end)], **duct)
    plc_dx = 0.0 if shared_duct else 0.13
    plc_path = ([(-0.25, -0.45), (CABINET_AT[0], -0.45), (CABINET_AT[0], cab_end)] if shared_duct else
                [(box_at[0] - plc_dx, box_end), (box_at[0] - plc_dx, -0.58), (CABINET_AT[0] + plc_dx, -0.58),
                 (CABINET_AT[0] + plc_dx, cab_end)])
    bt.parts.duct(scene, "duct_plc", path=plc_path, **duct)

    # Ports: where each line plugs in and which way it leaves. The arm's
    # base connector, the glands on the back of the box and the cabinet,
    # the valve's outlets (on its side, toward the base), the ejector's
    # inlets on the hand.
    cx = bt.connections

    def back_face(name: str, dx: float, up: float) -> tuple[float, float, float]:
        """A gland on the back (+y) face of a box, `dx` off centre, `up`
        above its bottom edge — in the box's own frame."""
        lo, hi = scene.obstacle_bounds(name)
        return (dx, (hi[1] - lo[1]) / 2, -(hi[2] - lo[2]) / 2 + up)

    cx.port(scene, "arm/cable", "arm", "power", "load", at=("arm", "base_link", (0.0, -0.09, 0.04)),
            exit=(0, -1, 0), straight_m=0.08)
    cx.port(scene, "UR/robot_cable", "UR", "power", "supply", target_kind="io_node",
            at=("UR/body", back_face("UR/body", 0.0, 0.03)), exit=(0, 1, 0), straight_m=0.10)
    cx.port(scene, "UR/plc", "UR", "network", "peer", target_kind="io_node", protocol="PROFINET",
            at=("UR/body", back_face("UR/body", -plc_dx, 0.14)), exit=(0, 1, 0), straight_m=0.05)
    cx.port(scene, "PLC1/ur", "PLC1", "network", "peer", target_kind="io_node", protocol="PROFINET",
            at=("cabinet/body", back_face("cabinet/body", plc_dx, 0.15)), exit=(0, 1, 0), straight_m=0.05)
    cx.port(scene, "valve/out1", "valve", "pneumatic", "supply", pressure_bar=6, capacity_l_min=100,
            flow_reference="ANR", at=("valve", (-0.025, -0.012, 0.035)), exit=(-1, 0, 0), straight_m=0.03)
    cx.port(scene, "valve/out2", "valve", "pneumatic", "supply", pressure_bar=6, capacity_l_min=100,
            flow_reference="ANR", at=("valve", (-0.025, 0.012, 0.035)), exit=(-1, 0, 0), straight_m=0.03)
    # The ejector's two inlets (see EJECTOR_AT): the hand tube on the -y
    # side at the home pose, as it is tied along the arm. One above the
    # other, the pair would have to turn a quarter on the way and the two
    # tubes would cross in the loop.
    x, y, z = INLET_AT
    cx.port(scene, "arm/tool/air", "arm/tool", "pneumatic", "load", target_kind="tool", pressure_min_bar=4,
            pressure_max_bar=7, flow_l_min=30, flow_reference="ANR",
            at=("arm", "ejector", (x, y, z)), exit=INLET_EXIT, straight_m=INLET_STRAIGHT_M)
    cx.port(scene, "arm/tool/vac", "arm/tool", "pneumatic", "load", target_kind="tool", pressure_min_bar=4,
            pressure_max_bar=7, flow_l_min=10, flow_reference="ANR",
            at=("arm", "ejector", (-x, y, z)), exit=INLET_EXIT, straight_m=INLET_STRAIGHT_M)
    cx.connect(scene, "UR/robot_cable", "arm/cable", name="robot_cable")
    cx.connect(scene, "UR/plc", "PLC1/ur", name="plc_link")
    cx.connect(scene, "valve/out1", "arm/tool/air", name="air_hand")
    cx.connect(scene, "valve/out2", "arm/tool/vac", name="air_release")

    # The lines: values from the data sheets, not guessed.
    rt = bt.routing
    rt.spec(scene, "ur_cable", media=["power"], od_mm=10.5, bend_r_fixed_mm=60, bend_r_moving_mm=105,
            end_allowance_m=0.3, sold_lengths_m=[6, 12], manufacturer="Universal Robots", model="Robot cable")
    # The PROFINET cable's maximum run and the metre it is sold by are the
    # standard's and the shop's, not the pack's; the tube's dynamic bend
    # radius and torsion are authored (the pack states the fixed radius).
    rt.spec(scene, "profinet", **article("profinet", catalog_root), max_length_m=100, order_step_m=1.0,
            end_allowance_m=0.2)
    rt.spec(scene, "tu0604", **article("tube", catalog_root), bend_r_moving_mm=20, torsion_deg_m=360)
    # A route runs from its connection's source port to its target: the box
    # (supply) → the duct → up the pedestal's column, clipped every 25 cm
    # (`along` lays the clamps on the column's near face) → out under the
    # top plate's edge, far enough for the last two elbows' fillets → up to
    # the arm's base connector.
    rt.route(scene, "robot_cable", carries="robot_cable", spec="ur_cable",
             via=["duct_ur", rt.along("pedestal/column", pitch_m=0.25), (0.0, -0.30, 0.41)],
             orthogonal=True, slack_m=1.0, clamp=article("clip", catalog_root))
    rt.route(scene, "plc_link", carries="plc_link", spec="profinet", orthogonal=True,
             via=["duct_ur", "duct_plc"] if shared_duct else ["duct_plc"])
    # The air tubes, tied along the arm: a P-clip per tube on the base
    # (side by side), then `along` the upper arm and the forearm — the clips
    # laid every TIE_PITCH_M on the faces named, each tube its own run
    # TIE_ASIDE_M off the middle — and the loops between. With
    # --dress-conduit, a corrugated conduit held at two places on the arm
    # takes both tubes from the upper arm to the forearm instead — one hose,
    # a bundle checked against its bore. The pack states the bore and the
    # static bend radius; the dynamic radius and the mass are authored.
    clip = article("clip", catalog_root)
    link, at, beside = BASE_TIE
    for name, side in (("air_hand", -1), ("air_release", 1)):
        # A P-clip is rigid: it points the tube the way it is screwed on —
        # up the casting and a little outward here, clear of the shoulder
        # housing, toward the upper arm's run.
        rt.clamp(scene, f"{name}_base", at=("arm", link, tuple(a + side * b for a, b in zip(at, beside))),
                 direction=BASE_TIE_DIRECTION, **clip)
    if dress_conduit:
        rt.carrier(scene, "dress", holders=[("arm", link, offset) for link, offset in DRESS_HOLDERS],
                   **article(dress_conduit, catalog_root), bend_r_moving_mm=60, mass_kg_m=0.08)
    for name, side in (("air_hand", -1), ("air_release", 1)):
        stagger = TIE_STAGGER_M if side > 0 else 0.0
        runs = [rt.along(("arm", link), pitch_m=TIE_PITCH_M, face=face, offset_m=side * TIE_ASIDE_M,
                         inset_m=(inset[0] + stagger, inset[1] + stagger))
                for link, face, inset in ARM_RUNS]
        via = [f"{name}_base"] + (["dress"] if dress_conduit else runs)
        slack = DRESS_SLACK if dress_conduit else (SLACK_OUTER if side > 0 else SLACK)
        rt.route(scene, name, carries=name, spec="tu0604", via=via, clamp=clip, slack_m=slack)
    return scene


def build_cycle(scene: bt.Scene) -> str:
    """Out to the arm's side (the first axis 57°) with the hand turned
    90°, and back — the air tubes between the clips move with it."""
    scene.add_segment("reach", goal=REACH)
    scene.add_segment("home", goal=HOME)
    sq = scene.sequence("cycle")
    sq.step("reach", actions=[bt.seq.motion("reach")])
    sq.step("home", actions=[bt.seq.motion("home")])
    return sq.name


def describe(scene: bt.Scene, timeline=None) -> None:
    print(bt.routing.cable_list(scene).to_markdown())
    report = bt.routing.report(scene, timeline)
    for sp in report.spans:
        print(f"  span {sp['route']} {sp['from']} → {sp['to']}: line {sp['material_m']} m, holders "
              f"{sp['min_distance_m']}–{sp['max_distance_m']} m (stroke {sp['stroke_m']} m, needs {sp['required_free_m']} m), "
              f"twist {sp['twist_deg']}°, bend r ≥ {sp.get('min_bend_radius_m')} m, sag {sp.get('max_sag_m')} m, "
              f"clearance {sp.get('min_clearance_m')} m ({sp.get('nearest')}) — {sp['status']}")
    for f in report.findings:
        where = f" @ {f['s_m']:.2f} m" if f.get("s_m") is not None else ""
        when = f" @ t = {f['t_s']:.2f} s" if f.get("t_s") is not None else ""
        print(f"  {f['severity']:7} {f['code']:20} {f['route']}: {f['message']}{where}{when}")
    for row in scene.requirements():
        for q in row.requirements:
            if q.key == "cable_m":
                print(f"\n{row.names[0]} cable_m: needs {q.value} m ({q.basis}) — {q.status}")
    check = scene.check(timeline=timeline)
    print("check:", "ok" if check.ok else f"{len(check.errors())} error(s)",
          "/", len(check.warnings()), "warning(s)")
    bom = scene.bom()
    for r in bom.rows:
        if r["category"].startswith(("cable", "pneumatic")):
            attrs = r.get("attributes") or {}
            print(f"  BOM {r['category']:16} ×{r['qty']} {r.get('manufacturer') or ''} {r.get('model') or ''}"
                  f" {attrs.get('length_m', '')} {'m' if 'length_m' in attrs else ''}")
    # The dress pack as a line to choose: what the cell asks of it, what the
    # declared hose states, and whether that covers it.
    for row in scene.requirements(timeline=timeline).rows:
        if row.category != "cable.carrier":
            continue
        print(f"\n{row.target} ({row.model or 'unidentified'}) — what the kit must offer:")
        for q in row.requirements:
            states = f", states {q.provided:g}" if q.provided is not None else ", states nothing"
            print(f"  {q.key} {q.op} {q.value:g} — {q.status}{states}  ({q.basis})")
        for note in row.notes:
            print(f"  note: {note}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default="routing_cell.usda", help="USD stage to write")
    parser.add_argument("--studio", action="store_true", help="open the studio after writing the stage")
    parser.add_argument("--move-box", type=float, default=0.0, metavar="METERS",
                        help="shift the UR control box along +x and watch the robot cable's length follow")
    parser.add_argument("--dress-conduit", choices=["pafs21", "prfs16"], default=None,
                        help="dress the air tubes in a corrugated conduit on the arm instead of tying them to it; "
                             "prfs16 (bore 11.7 mm) no longer takes both tubes (duct_fill)")
    parser.add_argument("--catalog-root", type=Path, default=None, metavar="DIR",
                        help="read the articles from this local catalog root (<root>/<id>/manifest.yaml)")
    parser.add_argument("--shared-duct", action="store_true",
                        help="run the PLC link through the robot cable's duct: power with network (route_separation)")
    parser.add_argument("--solver", choices=["physics", "static"], default="physics",
                        help="how the free spans get their shape: the rope replay (physics), or the rod at rest "
                             "between its holders at every frame (static — drawn, not simulated)")
    parser.add_argument("--sweep-holder", action="store_true",
                        help="move the forearm holder along the link and table the hose's stroke and twist at each "
                             "place (with the conduit; pafs21 unless --dress-conduit says otherwise)")
    args = parser.parse_args(argv)
    if args.sweep_holder and args.dress_conduit is None:
        args.dress_conduit = "pafs21"

    scene = build(args.move_box, args.dress_conduit, args.shared_duct, args.catalog_root)
    cycle = build_cycle(scene)
    timeline = scene.simulate_sequence(cycle)
    # The moving spans as tubes, the fixed ones hung under gravity too.
    timeline = bt.routing.animate(scene, timeline, settle_fixed=True, solver=args.solver)
    describe(scene, timeline)
    if args.sweep_holder:
        # The forearm holder along the link (its -x), from the elbow toward
        # the wrist: nothing is re-baked, the kinematic report is read again
        # at every place.
        table = bt.routing.sweep_holder(scene, timeline, "dress", 1, x=[-0.21, -0.27, -0.33, -0.37])
        print("\nforearm holder along the link — the hose over the cycle at each place:")
        print(table.to_markdown())
        best = table.best("stroke_m", where=lambda r: r["errors"] == 0)
        if best is None:
            print("no place along the link is free of errors — the stroke is read, the holder stays")
        else:
            print(f"least stroke without an error at x = {best['x']:g} m: {best['stroke_m']} m (twist {best['twist_deg']}°)")
    out = Path(args.out)
    timeline.export_usd(out, fps=30.0)
    bt.routing.cable_list(scene).save(out.with_name(out.stem + "_cables.csv"))
    bt.routing.report(scene, timeline).save(out.with_name(out.stem + "_routing.md"))
    scene.export_layout(out.with_name(out.stem + "_layout.svg"))
    print(f"\ncycle {timeline.duration:.1f} s; wrote {out}, {out.stem}_cables.csv, {out.stem}_routing.md and {out.stem}_layout.svg")
    if args.studio:
        scene.show_timeline(timeline)
        bt.studio(scene)


if __name__ == "__main__":
    main(sys.argv[1:])
