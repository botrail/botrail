"""The `botrail` command: bake, check and export a cell without writing a
line of Python — the entry an agent's iteration loop and a CI job share.

    botrail check cell.botrail                 # load, lint, count → JSON, exit 1 on errors
    botrail review cell.py --stage design      # missing information and unperformed evaluations
    botrail simulate cell.py --report r.json   # bake (+ scenarios) → the cell report
    botrail export cell.botrail --out deliverables/ --all
    botrail schema > project.schema.json       # the .botrail JSON Schema
    botrail studio cell.botrail                # open it in the browser

A *cell* is a `.botrail` project or a Python file: the file is run and its
top-level `scene` is taken, or its `build()` / `build_cell()` (returning a
`Scene`) is called — the two conventions the repository's examples use.
Everything the command prints is JSON by default (`--markdown` where a
human wants prose), so the output feeds straight back into whatever wrote
the cell. Exit codes: 0 ok, 1 findings with errors / a failed bake, 2 the
cell could not be loaded or the arguments were wrong.
"""

from __future__ import annotations

import argparse
import json
import runpy
import sys
import traceback
from pathlib import Path
from typing import Optional

from .deliverables import EXPORTS


class CliError(Exception):
    """A usage / load failure — reported as JSON, exit code 2."""


# ------------------------------------------------------------- loading


def load_cell(path: str):
    """A `Scene` from a `.botrail` project or a Python cell file."""
    import botrail as bt

    p = Path(path)
    if not p.exists():
        raise CliError(f"{path}: no such file")
    if p.suffix == ".botrail":
        return bt.Scene.load_project(p)
    if p.suffix == ".py":
        # Run the file as a module (not as `__main__`, so its `main()` guard
        # stays quiet), then take what it built.
        sys.path.insert(0, str(p.parent))
        try:
            namespace = runpy.run_path(str(p), run_name="__botrail_cell__")
        except Exception as e:
            raise CliError(f"{path}: {type(e).__name__}: {e}\n{traceback.format_exc()}") from e
        for key in ("scene",):
            if isinstance(namespace.get(key), bt.Scene):
                return namespace[key]
        for fn in ("build", "build_cell", "build_scene"):
            if callable(namespace.get(fn)):
                scene = namespace[fn]()
                if isinstance(scene, bt.Scene):
                    return scene
                raise CliError(f"{path}: {fn}() returned {type(scene).__name__}, not a Scene")
        raise CliError(
            f"{path}: define a top-level `scene` (a botrail Scene) or a `build()` / "
            "`build_cell()` function that returns one"
        )
    raise CliError(f"{path}: unknown cell type {p.suffix!r} — use a .botrail project or a .py file")


def _bake(scene, sequences: Optional[list], scenarios: bool, max_duration: float):
    """Bakes the programs — all of them together unless `sequences` names a
    set — and, with `scenarios`, the whole scenario matrix. Returns
    `(timelines dict, runs or None)`."""
    names = list(sequences) if sequences else list(scene.sequence_names)
    if not names:
        return {}, None
    if scenarios:
        runs = scene.simulate_scenarios(names, max_duration=max_duration)
        timelines = {name: tl for name, tl in runs.items()}
        return timelines, runs
    if len(names) == 1:
        tl = scene.simulate_sequence(names[0], max_duration=max_duration)
        return {names[0]: tl}, None
    tl = scene.simulate_sequences(names, max_duration=max_duration)
    return {"+".join(names): tl}, None


# ------------------------------------------------------------ commands


def cmd_check(args) -> int:
    from . import select

    scene = load_cell(args.cell)
    # The I/O lint, each sequence walked, unidentified lines with what the
    # cell asks of them, and the requirement comparison — `bt.select.check`.
    report = select.check(scene)
    findings = [f.to_dict() for f in report.findings]
    errors = sum(1 for f in findings if f["severity"] == "error")
    out = {
        "ok": errors == 0,
        "cell": args.cell,
        "robots": scene.robots,
        "counts": {
            "obstacles": len(scene.obstacle_names),
            "frames": len(scene.frames),
            "sensors": len(scene.sensor_names),
            "devices": len(scene.device_names),
            "sequences": len(scene.sequence_names),
            "scenarios": len(scene.scenario_names),
            "parts": len(scene.parts()),
            "bom_rows": len(scene.bom()),
        },
        "findings": findings,
        "requirements": report.to_dict()["requirements"],
    }
    print(json.dumps(out, indent=2))
    return 0 if out["ok"] else 1


def _report(scene, args, deliverables: Optional[list] = None):
    timelines, runs = _bake(scene, args.sequence, getattr(args, "scenarios", False), args.max_duration)
    report = scene.cell_report(
        timelines or None,
        scenarios=runs,
        deliverables=deliverables or None,
        clearance_dt=None if args.no_clearance else args.clearance_dt,
        title=args.title,
    )
    return report, timelines, runs


def cmd_simulate(args) -> int:
    scene = load_cell(args.cell)
    try:
        report, timelines, runs = _report(scene, args)
    except (ValueError, KeyError) as e:
        print(json.dumps({"ok": False, "cell": args.cell, "error": str(e)}, indent=2))
        return 1
    if args.usd:
        first = next(iter(timelines.values()), None)
        if first is None:
            raise CliError("--usd needs a sequence to bake; the cell has none")
        first.export_usd(args.usd, fps=args.fps)
    if args.report:
        report.save(args.report)
    if args.markdown:
        print(report.to_markdown())
    else:
        print(report.to_json())
    failed = bool(runs) and bool(runs.errors)
    return 1 if failed else 0


def cmd_review(args) -> int:
    from .review import review

    scene = load_cell(args.cell)
    config = {}
    if args.config:
        try:
            config = json.loads(Path(args.config).read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            raise CliError(f"review config: {e}") from e
        if not isinstance(config, dict) or set(config) - {"required", "totals", "annotations"}:
            raise CliError("review config must be an object with required, totals and/or annotations")
    if args.require:
        configured = config.get("required", [])
        if not isinstance(configured, list):
            raise CliError("review config required must be a list")
        config["required"] = [*configured, *args.require]
    cell_report = None
    if args.manifest and (args.simulate or args.scenarios):
        raise CliError("review: use --manifest or a new simulation, not both")
    if args.simulate or args.scenarios:
        try:
            cell_report, _, _ = _report(scene, args)
        except (ValueError, KeyError) as e:
            print(json.dumps({"ready": False, "cell": args.cell, "error": str(e)}, indent=2))
            return 1
    try:
        result = review(scene, report=cell_report, manifest=args.manifest,
                        sequences=args.sequence, stage=args.stage, **config)
        if args.report:
            result.save(args.report)
    except (ValueError, TypeError, OSError) as e:
        raise CliError(f"review: {e}") from e
    print(result.to_markdown() if args.markdown else result.to_json())
    return 0 if result.ready else 1


def cmd_export(args) -> int:
    from .deliverables import export_cell, verify_export

    scene = load_cell(args.cell)
    wanted = {name for name in EXPORTS if getattr(args, name)}
    try:
        manifest = export_cell(
            scene, args.out, name=args.name or Path(args.cell).stem,
            exports=None if args.all or not wanted else wanted,
            sequences=args.sequence, scenarios=args.scenarios, dt=args.dt,
            max_duration=args.max_duration, plan_resolution=args.plan_resolution,
            clearance_dt=None if args.no_clearance else args.clearance_dt,
            title=args.title, fps=args.fps, scale=args.scale, attachments=args.attach,
        )
    except (ValueError, KeyError, OSError) as e:
        print(json.dumps({"ok": False, "cell": args.cell, "error": str(e)}, indent=2))
        return 1
    verified = verify_export(manifest)
    files = [str(manifest.parent / row["path"]) for row in verified["files"]] + [str(manifest)]
    print(json.dumps({"ok": True, "cell": args.cell, "out": str(args.out), "files": files,
                      "manifest": str(manifest), "same_revision": verified["same_revision"],
                      "issues": verified["issues"]}, indent=2))
    return 0


def cmd_verify_export(args) -> int:
    from .deliverables import verify_export

    result = verify_export(args.manifest, scene=load_cell(args.cell) if args.cell else None)
    print(json.dumps(result, indent=2))
    return 0 if result["same_revision"] else 1


def cmd_connections(args) -> int:
    from .connections import report

    result = report(load_cell(args.cell))
    try:
        for path, table in ((args.report, "connections"), (args.csv, "connections"), (args.power, "power")):
            if path:
                result.save(path, table=table)
    except (ValueError, OSError) as e:
        print(json.dumps({"ok": False, "cell": args.cell, "error": str(e)}, indent=2))
        return 2
    print(result.to_markdown() if args.markdown else result.to_json())
    return 0 if result.ready else 1


def cmd_schema(args) -> int:
    import botrail as bt

    text = bt.project_schema()
    if args.out:
        Path(args.out).write_text(text)
        print(json.dumps({"ok": True, "out": args.out}))
    else:
        print(text)
    return 0


def cmd_capture(args) -> int:
    from . import capture

    scene = load_cell(args.cell)
    camera = args.camera
    if camera is None:
        names = scene.camera_names
        if len(names) != 1:
            have = ", ".join(names) if names else "none"
            raise CliError(f"--camera is required (cameras in the cell: {have})")
        camera = names[0]
    if args.depth:
        # A depth snapshot (or, for .ply, its point cloud): the scene as
        # authored, or --at seeks a bake.
        if args.at is not None:
            _bake(scene, args.sequence, scenarios=False, max_duration=args.max_duration)
        out = Path(args.out) if args.out else Path("cell_depth.npy")
        if out.suffix == ".ply":
            capture.capture_pointcloud(scene, camera, out, t=args.at)
        else:
            capture.capture_depth(scene, camera, out, t=args.at)
        print(json.dumps({"ok": True, "camera": camera, "t": args.at, "out": str(out)}))
        return 0
    # Bake the cycle to film. A cell with no sequences may still carry a
    # played recording; record_camera says so clearly if nothing arrives.
    _bake(scene, args.sequence, scenarios=False, max_duration=args.max_duration)
    out = capture.record_camera(scene, camera, args.out or "cell.webm", fps=args.fps)
    print(json.dumps({"ok": True, "camera": camera, "fps": args.fps, "out": str(out)}))
    return 0


def _merged_ply(path: Path, points: list) -> None:
    """All frames' world points as one binary little-endian PLY (the
    same shape `ScanFrame.save_ply` writes for a single sweep)."""
    import struct

    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.writelines(struct.pack("<3f", x, y, z) for x, y, z in points)


def cmd_scan(args) -> int:
    scene = load_cell(args.cell)
    lidar = args.lidar
    if lidar is None:
        names = scene.lidar_names
        if len(names) != 1:
            have = ", ".join(names) if names else "none"
            raise CliError(f"--lidar is required (lidars in the cell: {have})")
        lidar = names[0]
    if args.at is not None and args.sweep is not None:
        raise CliError("pass either --at or --sweep, not both")
    if args.at is not None or args.sweep is not None:
        # A timeline sweep needs the bake in place first.
        _bake(scene, args.sequence, scenarios=False, max_duration=args.max_duration)
    if args.sweep is not None:
        # The corridor survey: every frame's returns merged into one cloud.
        frames = scene.scan_sweep(lidar, fps=args.sweep, noise=args.noise, seed=args.seed)
        out = Path(args.out) if args.out else Path("cell_sweep.ply")
        points = [p for f in frames for p in f.points()]
        _merged_ply(out, points)
        print(
            json.dumps(
                {
                    "ok": True,
                    "lidar": lidar,
                    "frames": len(frames),
                    "points": len(points),
                    "out": str(out),
                }
            )
        )
        return 0
    frame = scene.lidar_scan(lidar, t=args.at, noise=args.noise, seed=args.seed)
    out = Path(args.out) if args.out else Path("cell_scan.ply")
    if out.suffix == ".csv":
        # The per-beam table: what the sweep measured, and what it hit —
        # the blind-spot debugging view.
        with open(out, "w", encoding="utf-8") as f:
            f.write("angle_deg,elevation_deg,range_m,hit\n")
            f.writelines(
                f"{a:.4f},{e:.4f},{r:.6f},{h or ''}\n"
                for a, e, r, h in zip(frame.angles, frame.elevations, frame.ranges, frame.hits)
            )
    else:
        frame.save_ply(out)
    returns = sum(1 for r in frame.ranges if r > 0)
    print(
        json.dumps(
            {
                "ok": True,
                "lidar": lidar,
                "beams": len(frame.ranges),
                "returns": returns,
                "out": str(out),
            }
        )
    )
    return 0


def cmd_studio(args) -> int:
    import botrail as bt

    scene = load_cell(args.cell)
    bt.studio(scene, port=args.port)
    return 0


# -------------------------------------------------------------- parser


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="botrail",
        description="Bake, check, review and export a robot cell (a .botrail project or a Python cell file).",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def add_bake_args(p, scenarios_help: str) -> None:
        p.add_argument("--sequence", action="append", help="program(s) to bake together (default: all)")
        p.add_argument("--scenarios", action="store_true", help=scenarios_help)
        p.add_argument("--max-duration", type=float, default=120.0, help="bake time limit, seconds (default 120)")
        p.add_argument("--clearance-dt", type=float, default=0.01, help="clearance scan step, seconds (default 0.01)")
        p.add_argument("--no-clearance", action="store_true", help="skip the clearance re-scan")
        p.add_argument("--title", help="report / sheet title")

    p = sub.add_parser("check", help="load the cell, lint it, count what is in it (JSON; exit 1 on errors)")
    p.add_argument("cell")
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("review", help="review missing design information and required evidence (exit 1 if unresolved)")
    p.add_argument("cell")
    p.add_argument("--stage", choices=("concept", "design"), default="concept")
    p.add_argument("--require", action="append", help="additional required group or item ID (repeatable)")
    p.add_argument("--config", help="JSON object with required, totals and/or annotations")
    p.add_argument("--simulate", action="store_true", help="also bake the selected programs for the review")
    p.add_argument("--manifest", help="verify a generated package and review its execution evidence")
    add_bake_args(p, "also bake the scenario matrix; expected-result acceptance remains a separate check")
    p.add_argument("--report", help="write the design information review (.json or .md)")
    p.add_argument("--markdown", action="store_true", help="print Markdown instead of JSON")
    p.set_defaults(func=cmd_review)

    p = sub.add_parser("simulate", help="bake the cell and print the cell report (JSON, or --markdown)")
    p.add_argument("cell")
    add_bake_args(p, "bake the whole scenario matrix (the report gets the table)")
    p.add_argument("--report", help="also write the report here (.json or .md)")
    p.add_argument("--usd", help="also write the (first) baked cycle as USD here")
    p.add_argument("--fps", type=float, default=30.0, help="USD frame rate (default 30)")
    p.add_argument("--markdown", action="store_true", help="print Markdown instead of JSON")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("export", help="write the document set (project, python, bom, io, topology, plc, interlocks, layout, usd, script, connections, report)")
    p.add_argument("cell")
    p.add_argument("--out", required=True, help="new or empty output directory for this revision")
    p.add_argument("--name", help="file stem (default: the cell file's stem)")
    for name in EXPORTS:
        p.add_argument(f"--{name}", action="store_true", help=f"write the {name}")
    p.add_argument("--all", action="store_true", help="write everything (the default when nothing is picked)")
    add_bake_args(p, "bake the scenario matrix for the report and the script")
    p.add_argument("--dt", type=float, default=0.01, help="simulation scan step in seconds (default 0.01)")
    p.add_argument("--plan-resolution", type=float, default=0.05, help="planner edge sampling stride (default 0.05)")
    p.add_argument("--attach", action="append", help="copy an external attachment with unverified provenance (repeatable)")
    p.add_argument("--fps", type=float, default=30.0, help="USD frame rate (default 30)")
    p.add_argument("--scale", type=float, default=100.0, help="layout SVG pixels per metre (default 100)")
    p.set_defaults(func=cmd_export)

    p = sub.add_parser("schema", help="print the .botrail JSON Schema")
    p.add_argument("--out", help="write it here instead of stdout")
    p.set_defaults(func=cmd_schema)

    p = sub.add_parser(
        "capture",
        help="record a camera's view as video, or --depth for a metric depth snapshot (headless browser)",
    )
    p.add_argument("cell")
    p.add_argument("--camera", help="camera name (default: the cell's only camera)")
    p.add_argument("--out", help="video: .webm/.mp4/.gif (default cell.webm); depth: .npy/.png/.ply (default cell_depth.npy)")
    p.add_argument("--fps", type=int, default=30, help="video frame rate (default 30)")
    p.add_argument("--depth", action="store_true", help="capture one metric depth image instead of video (.npy float32 meters, or 16-bit .png millimeters; a .json sidecar carries the intrinsics; .ply writes the world-space point cloud)")
    p.add_argument("--at", type=float, help="depth only: seek the baked cycle to this time, seconds (default: the scene as authored, no bake)")
    p.add_argument("--sequence", action="append", help="program(s) to bake together (default: all)")
    p.add_argument("--max-duration", type=float, default=120.0, help="bake time limit, seconds (default 120)")
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser(
        "scan",
        help="simulate one sweep of a lidar against the cell's colliders (no browser)",
    )
    p.add_argument("cell")
    p.add_argument("--lidar", help="lidar name (default: the cell's only lidar)")
    p.add_argument("--out", help=".ply point cloud (default cell_scan.ply), or .csv per-beam table (angle, range, hit); --sweep always writes the merged .ply (default cell_sweep.ply)")
    p.add_argument("--at", type=float, help="sweep at this instant of the baked cycle, seconds (default: the scene as authored, no bake)")
    p.add_argument("--sweep", type=float, metavar="FPS", help="sweep every frame of the baked cycle at this rate and merge the clouds")
    p.add_argument("--noise", type=float, default=0.0, metavar="SIGMA", help="Gaussian range noise, 1-sigma meters (deterministic per --seed; default 0 = exact)")
    p.add_argument("--seed", type=int, default=0, help="noise stream seed (default 0)")
    p.add_argument("--sequence", action="append", help="program(s) to bake together (default: all)")
    p.add_argument("--max-duration", type=float, default=120.0, help="bake time limit, seconds (default 120)")
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("verify-export", help="verify a document set's revision and file digests (exit 1 on mismatch or attachments)")
    p.add_argument("manifest", help="generated *_manifest.json")
    p.add_argument("--cell", help="also compare against this current cell definition")
    p.set_defaults(func=cmd_verify_export)

    p = sub.add_parser("connections", help="check physical interfaces and connected supply loads (exit 1 if unresolved)")
    p.add_argument("cell")
    p.add_argument("--report", help="save the connection report (.json or .md)")
    p.add_argument("--csv", help="save the interface requirements table (.csv)")
    p.add_argument("--power", help="save the power supply capacity table (.csv)")
    p.add_argument("--markdown", action="store_true", help="print Markdown instead of JSON")
    p.set_defaults(func=cmd_connections)

    p = sub.add_parser("studio", help="open the cell in the studio")
    p.add_argument("cell")
    p.add_argument("--port", type=int, default=0, help="port (default: any free)")
    p.set_defaults(func=cmd_studio)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except CliError as e:
        print(json.dumps({"ok": False, "error": str(e)}, indent=2))
        return 2


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
