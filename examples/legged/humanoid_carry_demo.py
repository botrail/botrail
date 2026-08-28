"""A humanoid carrying a part from one bench to another.

The cell sees a humanoid the way it sees a quadruped: a vehicle with legs.
It is sent with `bt.seq.goto`, arrives with `bt.seq.device_done`, and its
legs are a `bt.Gait` on the mount — two of them, with soles that land flat
and pointed where the body heads, a body that bobs and leans over each
planted leg, and arms that swing in step until the hands are full. What a
humanoid adds is what it is bought for: it *carries*. The arms are the
robot's own joints, so they are taught with ramps and the part is held
with the same `attach` an arm on a pedestal uses; the gait leaves a held
part's arms alone, and the part rides still in the hands through the walk.

The robot is a Unitree G1 (29 DOF, `unitree_ros`, BSD-3-Clause): the first
run fetches its URDF and the 35 STL meshes it references (~15 MB) into the
botrail cache. `--robot biped` runs the same cell on the primitive biped in
`examples/assets/biped_test.urdf`, with no download — that is what the tests use.

The cycle:

  * **受け取り** — standing at bench A, the arms go out and up to the sides,
    then swing in around the tote at chest height, and the hands close on
    it (`attach`); the arms lift it clear. Two ramps, not one: an arm
    raised straight ahead from the hip sweeps its forearm through a bench
    at hip height, and a ramp is not planned — its path is the author's,
    so the cell checks every arm ramp against the benches before it bakes
    (`ramp_contacts`).
  * **搬送** — `goto` bench B. The legs walk, the body sways, the arms do
    not swing: they are holding something.
  * **載置** — the arms lower the tote onto bench B and let go (`detach`);
    the arms rest, and the robot walks back.

Everything the legs do is derived — no walk is authored anywhere. The
benches are placed where the taught carry pose puts the hands, so one pose
serves both ends: the same inversion the AMR cell uses (teach the pose,
put the equipment where it reaches).

Run with:  python examples/legged/humanoid_carry_demo.py [out.usdc] [--robot g1|biped] [--studio]
"""

import os
import re
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import botrail as bt  # noqa: E402

ASSETS = Path(__file__).resolve().parents[1] / "assets"

# ------------------------------------------------------------- the machine
G1_REPO = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_description"
G1_URDF = "g1_29dof_rev_1_0.urdf"

SIDES = ("left", "right")


def _g1_stance() -> dict:
    # Knees bent enough to take a stride: a leg stood nearly straight has
    # no reach left for the foot that stays planted while the body moves
    # off (the first half-period of every walk). Soles flat: the ankle
    # pitch undoes hip + knee.
    # The arms hang at the sides (a G1's forearms point forward at zero —
    # straight into a bench at hip height).
    stance = {}
    for side, out in zip(SIDES, (0.15, -0.15)):
        stance.update({
            f"{side}_hip_pitch_joint": -0.4, f"{side}_hip_roll_joint": 0.0,
            f"{side}_hip_yaw_joint": 0.0, f"{side}_knee_joint": 0.8,
            f"{side}_ankle_pitch_joint": -0.4, f"{side}_ankle_roll_joint": 0.0,
            f"{side}_shoulder_pitch_joint": 0.15, f"{side}_shoulder_roll_joint": out,
            f"{side}_elbow_joint": 1.2,
        })
    return stance


# G1: 6-DOF legs whose foot link is the ankle roll link, sole 35 mm below
# it (the foot's collision spheres sit at z = -0.03, radius 5 mm). Arms
# swing at the shoulder pitch, opposite signs for opposite arms.
G1_GAIT = {
    "legs": {"L": "left_ankle_roll_link", "R": "right_ankle_roll_link"},
    "contact": "sole",
    "stance": _g1_stance(),
    "pattern": "biped", "period": 0.85, "lift": 0.05, "max_stride": 0.5, "foot_radius": 0.035,
    "arm_swing": {"left_shoulder_pitch_joint": -0.25, "right_shoulder_pitch_joint": 0.25},
    "bob": 0.015, "lateral": 0.02,
}
# Hands forward at waist height, a tote's width apart: the forearms point
# +x at zero, so the carry is mostly the shoulders.
G1_CARRY = {
    "left_shoulder_pitch_joint": -0.35, "right_shoulder_pitch_joint": -0.35,
    "left_shoulder_roll_joint": 0.12, "right_shoulder_roll_joint": -0.12,
    "left_elbow_joint": 0.1, "right_elbow_joint": 0.1,
}
G1_LIFT = {"left_shoulder_pitch_joint": -0.55, "right_shoulder_pitch_joint": -0.55}
# Between rest and carry: arms out to the sides and up, elbows nearly
# straight — the one way past a bench at hip height (the elbow folds the
# forearm *down*, so nothing raised in front clears the top).
G1_TUCK = {
    "left_shoulder_pitch_joint": 0.3, "right_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 1.5, "right_shoulder_roll_joint": -1.5,
    "left_shoulder_yaw_joint": 0.6, "right_shoulder_yaw_joint": -0.6,
    "left_elbow_joint": 0.3, "right_elbow_joint": 0.3,
}
G1_HAND = "right_rubber_hand"
G1_FOOTPRINT = (0.30, 0.40, 1.25)   # the body the aisle check drives; the tote rides ahead of it
G1_SPEED, G1_TURN = 0.45, 0.8

# The primitive biped: 0.35 m thigh and shank, soles 50 mm under the ankle.
BIPED_GAIT = {
    "legs": {"L": "L_foot", "R": "R_foot"},
    "contact": "sole",
    "stance": {f"{s}_{j}": v for s in ("L", "R")
               for j, v in (("hip_yaw_joint", 0.0), ("hip_roll_joint", 0.0), ("hip_pitch_joint", -0.4),
                            ("knee_joint", 0.8), ("ankle_pitch_joint", -0.4), ("ankle_roll_joint", 0.0))},
    "pattern": "biped", "period": 0.8, "lift": 0.05, "max_stride": 0.5, "foot_radius": 0.05,
    "arm_swing": {"L_shoulder_pitch_joint": 0.3, "R_shoulder_pitch_joint": -0.3},
    "bob": 0.02, "lateral": 0.015,
}
BIPED_CARRY = {"L_shoulder_pitch_joint": -1.0, "R_shoulder_pitch_joint": -1.0,
               "L_elbow_joint": -0.6, "R_elbow_joint": -0.6}
BIPED_LIFT = {"L_shoulder_pitch_joint": -1.2, "R_shoulder_pitch_joint": -1.2}
BIPED_TUCK = {"L_shoulder_pitch_joint": -0.2, "R_shoulder_pitch_joint": -0.2,
              "L_elbow_joint": -1.8, "R_elbow_joint": -1.8}
BIPED_HAND = "R_hand"
BIPED_FOOTPRINT = (0.40, 0.50, 1.30)
BIPED_SPEED, BIPED_TURN = 0.3, 0.8

# ---------------------------------------------------------------- the cell
# Both stations face +x with their bench in front. A parked vehicle faces
# the leg it arrived by (a path's end) or the leg it leaves by (its start),
# so bench A is the path's *end* and the robot starts there: leaving it is
# a step back (the vehicle may reverse), the turn is made clear of the
# bench, and every arrival is nose first with the next bench straight
# ahead.
BENCH_A, BENCH_B = (0.0, 0.0), (2.4, 2.0)
PATH = [BENCH_B, (-0.6, 2.0), (-0.6, 0.0), BENCH_A]
TOTE = (0.24, 0.16, 0.12)
SEAT_GAP = 0.006
STEEL = (0.32, 0.35, 0.40)
CRATE = (0.45, 0.33, 0.20)


def fetch_g1() -> Path:
    """Unitree's G1 description, fetched once into the botrail cache: the
    URDF and only the meshes it references, laid out the way it expects
    them (`meshes/` beside the URDF), so nothing needs rewriting."""
    cache = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
    dest = cache / "assets" / "g1"
    urdf = dest / G1_URDF
    if urdf.exists():
        return urdf
    (dest / "meshes").mkdir(parents=True, exist_ok=True)

    def fetch(rel: str) -> Path:
        target = dest / rel
        if not target.exists():
            print(f"downloading {rel} ...")
            part = target.with_suffix(target.suffix + ".part")
            urllib.request.urlretrieve(f"{G1_REPO}/{rel}", part)
            part.rename(target)
        return target

    part = urdf.with_suffix(".urdf.part")
    urllib.request.urlretrieve(f"{G1_REPO}/{G1_URDF}", part)
    xml = part.read_text()
    for rel in sorted(set(re.findall(r'filename="(meshes/[^"]+)"', xml))):
        fetch(rel)
    part.rename(urdf)
    return urdf


def walker_of(robot: str):
    """The humanoid named by `--robot`: model, gait, arm poses (tuck, carry,
    lift), hand link, footprint, rates."""
    if robot == "g1":
        return (bt.Robot.from_urdf(fetch_g1()), bt.Gait(**G1_GAIT), (G1_TUCK, G1_CARRY, G1_LIFT),
                G1_HAND, G1_FOOTPRINT, G1_SPEED, G1_TURN)
    if robot == "biped":
        return (bt.Robot.from_urdf(ASSETS / "biped_test.urdf"), bt.Gait(**BIPED_GAIT),
                (BIPED_TUCK, BIPED_CARRY, BIPED_LIFT), BIPED_HAND, BIPED_FOOTPRINT, BIPED_SPEED,
                BIPED_TURN)
    raise ValueError(f"unknown robot `{robot}` (g1 | biped)")


def with_joints(scene: bt.Scene, robot: str, base: list, targets: dict) -> list:
    names = scene.robot_of(robot).joint_names
    q = list(base)
    for joint, value in targets.items():
        q[names.index(joint)] = value
    return q


def ramp_contacts(scene: bt.Scene, a: list, b: list, samples: int = 25) -> list:
    """Link/obstacle pairs an arm ramp from `a` to `b` would touch, sampled
    along its cubic: a ramp is not planned, so its path is the author's to
    check. The vehicle's footprint is the body the legs stand in and does
    not count."""
    was = list(scene.joint_positions_of("walker"))
    hits = set()
    for k in range(samples + 1):
        u = k / samples
        ease = u * u * (3 - 2 * u)
        scene.set_joint_positions([x + (y - x) * ease for x, y in zip(a, b)], robot="walker")
        for (_, p), (_, q) in scene.check_collisions():
            if "walker/footprint" not in (p, q):
                hits.add((p, q))
    scene.set_joint_positions(was, robot="walker")
    return sorted(hits)


def build_scene(robot: str = "g1"):
    """The two benches and the humanoid standing at the first; returns the
    scene and the taught arm poses (stance, tuck, carry, lift)."""
    model, gait, (tuck, carry, lift), hand, footprint, speed, turn = walker_of(robot)
    scene = bt.Scene(model, name="walker")
    scene.add_box("walker/footprint", footprint, (BENCH_A[0], BENCH_A[1], footprint[2] / 2))
    scene.set_obstacle_visible("walker/footprint", False)
    scene.add_vehicle("legs", body=["walker/footprint"], path=PATH, stations={"b": 0, "a": 3},
                      speed=speed, turn_speed=turn, start="a", allow_reverse=True)
    scene.mount_robot("legs", robot="walker", gait=gait)   # stands it at bench A, facing +x

    # Teach the arms in joint space and put the benches where the hands
    # are; both stations face +x.
    stance = list(scene.joint_positions)
    q_tuck = with_joints(scene, "walker", stance, tuck)
    q_carry = with_joints(scene, "walker", stance, carry)
    q_lift = with_joints(scene, "walker", q_carry, lift)
    hand_p, _ = scene.link_pose_at(hand, q_carry, robot="walker")
    reach = hand_p[0] - BENCH_A[0]                 # how far ahead the hands are
    top = hand_p[2] - TOTE[2] / 2 - SEAT_GAP        # the tote's seat under the hands
    for name, (x, y) in (("bench_a", BENCH_A), ("bench_b", BENCH_B)):
        bt.parts.table(scene, name, size=(0.6, 1.0, top - 0.006), position=(x + reach + 0.22, y),
                       model="HFS8-600", manufacturer="Generic", color=STEEL)
    scene.add_box("tote", TOTE, (BENCH_A[0] + reach, BENCH_A[1], top + TOTE[2] / 2), color=CRATE)
    bt.parts.rack(scene, "rack", size=(1.2, 0.5, 2.0), position=(1.2, -1.3),
                  model="SR-2000", manufacturer="Generic", color=STEEL)
    return scene, (stance, q_tuck, q_carry, q_lift)


def build_cycle(scene: bt.Scene, poses) -> str:
    """Checks the arm ramps against the benches and writes the carry cycle;
    returns its name."""
    stance, q_tuck, q_carry, q_lift = poses
    names = scene.robot.joint_names
    for label, (a, b) in (("raise", (stance, q_tuck)), ("reach", (q_tuck, q_carry)),
                          ("lift", (q_carry, q_lift)), ("withdraw", (q_carry, q_tuck))):
        hits = ramp_contacts(scene, a, b)
        if hits:
            raise RuntimeError(f"the `{label}` ramp sweeps through the cell: {hits[:3]}")

    def ramp(q: list, duration: float, joints: dict):
        return bt.seq.ramp({j: q[names.index(j)] for j in joints}, duration, robot="walker")

    arms = {j: None for j in (set(G1_TUCK) | set(BIPED_TUCK) | set(G1_CARRY) | set(BIPED_CARRY)
                              | set(G1_LIFT) | set(BIPED_LIFT)) if j in names}
    sq = scene.sequence("carry")
    sq.step("raise", actions=[ramp(q_tuck, 1.2, arms)], transition=bt.seq.done())
    sq.step("reach", actions=[ramp(q_carry, 1.2, arms)], transition=bt.seq.done())
    sq.step("grasp", actions=[bt.seq.attach("tote", robot="walker")])
    sq.step("lift", actions=[ramp(q_lift, 0.8, arms)], transition=bt.seq.done())
    sq.step("to b", actions=[bt.seq.goto("legs", "b")], transition=bt.seq.device_done("legs"))
    sq.step("set down", actions=[ramp(q_carry, 0.8, arms)], transition=bt.seq.done())
    sq.step("release", actions=[bt.seq.detach("tote")])
    sq.step("withdraw", actions=[ramp(q_tuck, 1.2, arms)], transition=bt.seq.done())
    sq.step("rest", actions=[ramp(stance, 1.2, arms)], transition=bt.seq.done())
    sq.step("return", actions=[bt.seq.goto("legs", "a")], transition=bt.seq.device_done("legs"))
    sq.step("stand", transition=bt.seq.elapsed(1.5))
    return "carry"


def bake(robot: str = "g1"):
    scene, poses = build_scene(robot)
    name = build_cycle(scene, poses)
    return scene, scene.simulate_sequence(name, max_duration=120.0)


def main() -> None:
    args = sys.argv[1:]
    robot = args[args.index("--robot") + 1] if "--robot" in args else "g1"
    out = next((a for a in args if not a.startswith("--") and a != robot), "cell_humanoid.usdc")

    scene, poses = build_scene(robot)
    name = build_cycle(scene, poses)
    if "--studio" in args:
        bt.studio(scene)
        return
    try:
        tl = scene.simulate_sequence(name, max_duration=120.0)
    except ValueError as err:
        print(f"cycle failed: {err}")
        sys.exit(1)

    print(f"cycle time: {tl.duration:.2f}s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<10} {start:6.2f} – {end:6.2f}s")
    steps = tl.footfalls("walker")
    walking = tl.signal("legs").high_total()
    # The body's bob over the first walk, read back off the base track.
    to_b = tl.step_span("to b")
    heights = [tl.base_pose(to_b.start + k * 0.05, robot="walker")[0][2]
               for k in range(int((to_b.end - to_b.start) / 0.05))]
    tote = tl.object_pose("tote", tl.duration)[0]
    hand = scene.link_pose_at(G1_HAND if robot == "g1" else BIPED_HAND, poses[2], robot="walker")[0]
    print(f"{len(steps)} footfalls; walking {walking:.2f}s of {tl.duration:.2f}s; "
          f"body height {min(heights):.3f}–{max(heights):.3f} m on the way to B; "
          f"hands {hand[0] - BENCH_A[0]:.2f} m ahead at {hand[2]:.2f} m")
    print(f"tote ends at {tuple(round(v, 3) for v in tote)} — on bench B")

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
