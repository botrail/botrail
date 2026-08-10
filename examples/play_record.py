"""Play a baked USD recording back into the studio.

A recording is joint tracks addressed to robot *instances*, not to "the
robot": botrail exports each one under `/World/<instance name>`, and
playback looks the scene's robots up by that path. So a recording has to be
played onto the cell it was baked from — and putting the two-arm recording
on the single-arm cell is an error, not a degraded picture:

    recording import failed: cannot locate robot `near` in the recording
    (no `/World/near`); pass robot_roots with its prim path

which is also the escape hatch when a recording came from somewhere else
(Isaac Sim, say) and its prims are named differently: pass `robot_roots`.

How the robot plays depends on where it came from. USD-sourced robots
carry joint tracks (`joint_state` mode); URDF and `attach_tool` composite
robots have no stage behind them, so their bake is per-link world poses
and playback follows those (`transforms` mode). Either way the cell has to
be rebuilt first — the recording stores motion, not geometry.

Run with:  python examples/play_record.py [recording.usda] [--cell NAME]

A binary `.usdc` keeps its prim names out of reach of the text sniffing
below, so name its cell explicitly: `--cell line` / `line4` / `weld` / …
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import agv_cell_demo  # noqa: E402
import amr_demo  # noqa: E402
import botrail as bt  # noqa: E402
import demo  # noqa: E402
import dual_cell_demo  # noqa: E402
import weld_station_demo  # noqa: E402

DEFAULT = Path("cell_seq.usda")


def robot_instances(recording: Path) -> set:
    """The instance names a recording animates, read off the prim names
    botrail exported them under. Empty for a binary `.usd`/`.usdc`, which
    just means the cell has to be chosen by hand below."""
    try:
        text = recording.read_text()
    except (UnicodeDecodeError, OSError):
        return set()
    # Direct children of /World; `Env` is the static scenery, not a robot.
    return {
        name
        for name in re.findall(r'^    def Xform "([^"]+)"', text, re.M)
        if name != "Env"
    }


def marks(recording: Path, needle: str) -> bool:
    """Does the recording mention `needle`? Cells are told apart by a prim
    only one of them has."""
    try:
        return needle in recording.read_text()
    except (UnicodeDecodeError, OSError):
        return False


def has_vehicle(recording: Path) -> bool:
    """Does the recording animate an AGV? Its body is scenery, so it lands
    under `/World/Env`, not beside the robots — the cell still has to be
    rebuilt with the vehicle in it, or the body prims resolve to nothing
    and the AGV sits frozen at the warehouse while its cycle plays."""
    try:
        return '"agv"' in recording.read_text()
    except (UnicodeDecodeError, OSError):
        return False


def cell_for(recording: Path) -> bt.Scene:
    """Rebuilds the cell the recording was baked from."""
    names = robot_instances(recording)
    if {"near", "far"} <= names:
        print(f"{recording}: two-arm cell ({', '.join(sorted(names))})")
        return dual_cell_demo.build_cell()
    if {"lh_up", "lh_dn", "rh_up", "rh_dn"} <= names:
        # The weld cell's arms are composites (`attach_tool`: catalog arm
        # + catalog gun) — no USD stage behind them, so their bake is
        # per-link transforms and playback follows those directly
        # (`transforms` mode) instead of joint tracks.
        print(f"{recording}: weld station ({', '.join(sorted(names))})")
        return weld_station_demo.build_cell()[0]
    if {"st1_lh", "st1_rh", "st2_lh", "st2_rh"} <= names:
        import weld_line_demo

        print(f"{recording}: weld line ({', '.join(sorted(names))})")
        return weld_line_demo.build_line()[0]
    # The AMR carries its own arm and has no cell at all, so check it first:
    # its body prims are named like the AGV's.
    if marks(recording, '"stand_place"'):
        print(f"{recording}: AMR (arm riding the vehicle)")
        return amr_demo.build_scene()
    if has_vehicle(recording):
        print(f"{recording}: single-arm cell + AGV")
        return agv_cell_demo.build_scene()
    # Everything left falls through to the single-arm cell, which is safe
    # for *one* robot however it is named: playback structurally searches
    # the stage when there is only one to find, so an older bake whose
    # instance is `Robot` still lands. Two or more unrecognised instances
    # have no such escape — answering those with a Franka would show a
    # factory that has nothing to do with the recording, and the mismatch
    # would surface as `cannot locate robot ...` rather than as "I do not
    # know this cell". (A binary recording reads as no names at all;
    # nothing to check, so it still falls through.)
    if len(names) > 1:
        raise SystemExit(
            f"{recording} animates {', '.join(sorted(names))}, which is not a "
            "cell this script knows how to rebuild. Build that cell yourself "
            "and call scene.play_usd_animation() on it."
        )
    if not names:
        print(
            f"{recording}: no robot prims readable (a binary .usdc keeps its "
            f"names out of reach) — assuming the single-arm cell; pass "
            f"--cell {'/'.join(sorted(CELLS))} to say otherwise"
        )
    else:
        print(f"{recording}: single-arm cell")
    return demo.build_scene()


# Cells this script can rebuild, for `--cell`. A *binary* recording
# (`.usdc`) carries the same prims as a text one, but the sniffing below
# reads prim names out of the text — so a binary recording has to be told
# which cell it belongs to rather than guessed at.
CELLS = {
    "single": lambda: demo.build_scene(),
    "dual": lambda: dual_cell_demo.build_cell(),
    "weld": lambda: weld_station_demo.build_cell()[0],
    "line": lambda: _line_cell(2),
    "line4": lambda: _line_cell(4),
    "agv": lambda: agv_cell_demo.build_scene(),
    "amr": lambda: amr_demo.build_scene(),
}


def _line_cell(stations: int):
    import weld_line_demo

    weld_line_demo.set_stations(stations)
    return weld_line_demo.build_line()[0]


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    cell = None
    if "--cell" in sys.argv:
        cell = sys.argv[sys.argv.index("--cell") + 1]
        if cell not in CELLS:
            raise SystemExit(
                f"--cell takes one of: {', '.join(sorted(CELLS))}"
            )
    recording = Path(args[0]) if args else DEFAULT
    if not recording.exists():
        raise SystemExit(
            f"{recording} not found — bake one first:\n"
            "  python examples/sequence_demo.py      # -> cell_seq.usda\n"
            "  python examples/dual_cell_demo.py     # -> cell_dual.usda\n"
            "  python examples/weld_station_demo.py  # -> cell_weld.usda\n"
            "  python examples/weld_line_demo.py     # -> cell_line.usda"
        )

    scene = CELLS[cell]() if cell else cell_for(recording)
    server = bt.studio(scene, block=False)  # ブラウザが開く

    result = scene.play_usd_animation(recording)
    print(f"{result['mode']} {result['duration']:.2f}s")
    print(f"  robots:  {', '.join(scene.robots)}")
    print(f"  objects: {', '.join(result['object_tracks']) or '—'}")
    # A recording baked before a layout change leaves the new scenery
    # static and says so, once per prim — summarise rather than flood.
    warnings = result["warnings"]
    for warning in warnings[:3]:
        print(f"  warning: {warning}")
    if len(warnings) > 3:
        print(f"  warning: … and {len(warnings) - 3} more (re-bake the recording)")

    input("Enterで終了")
    server.stop()


if __name__ == "__main__":
    main()
