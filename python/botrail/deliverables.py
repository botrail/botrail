"""Write the document set of a cell from one snapshot and one bake.

``export_cell`` bakes the selected programs once, writes every requested
document from that same snapshot into a staging directory, and moves the
files into place only when all of them succeeded — so a set never mixes an
old script with a new I/O list. A small manifest lists what was written,
with each file's SHA-256, and the conditions the bake ran under.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
import warnings
from pathlib import Path
from xml.etree import ElementTree as ET

EXPORTS = ("project", "python", "bom", "io", "topology", "plc", "interlocks", "layout", "usd", "script", "report")
SCHEMA_VERSION = 1


def _file(path):
    digest = hashlib.sha256()
    size = 0
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            size += len(chunk)
    return {"sha256": digest.hexdigest(), "bytes": size}


def _name(value):
    if not isinstance(value, str) or not re.fullmatch(r"[\w.-]+", value) or value in (".", ".."):
        raise ValueError("export name must contain only letters, numbers, underscore, dot or hyphen")
    return value


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


def _bake(scene, names, scenarios, dt, max_duration, plan_resolution):
    if not names:
        return {}, None
    options = {"dt": dt, "max_duration": max_duration, "plan_resolution": plan_resolution}
    if scenarios:
        runs = scene.simulate_scenarios(names, **options)
        return dict(runs.items()), runs
    timeline = scene.simulate_sequences(names, **options)
    return {"+".join(names): timeline}, None


def export_cell(scene, out: str | Path, *, name: str = "cell", exports=None,
                sequences=None, scenarios: bool = False, dt: float = 0.01,
                max_duration: float = 120., plan_resolution: float = 0.05,
                clearance_dt: float | None = 0.01, title: str | None = None,
                fps: float = 30., scale: float = 100., manifest: bool = True) -> Path:
    """Write the selected documents (and a manifest) from a fresh snapshot and bake.

    ``out`` is created when missing; a document set already there is
    overwritten file by file and anything else in the directory is left
    alone, so the set can be re-run in place. ``exports=None`` means all
    formats. Every program-dependent exporter uses ``sequences`` (all by
    default). Bakes are kinematic. A program that cannot compile to the
    robot dialect, a PLCopen block left as a stub, or a scenario that did
    not complete is recorded under ``issues`` in the manifest and the
    report rather than failing the export.

    Returns the manifest path — or, with ``manifest=False``, the directory.
    """
    from . import _core

    out = Path(out)
    name = _name(name)
    wanted = set(EXPORTS if exports is None else exports)
    if not wanted or wanted - set(EXPORTS):
        raise ValueError(f"exports must be a nonempty subset of {EXPORTS}")
    for key, value in {"dt": dt, "max_duration": max_duration, "plan_resolution": plan_resolution,
                       "fps": fps, "scale": scale}.items():
        _positive(value, key)
    if clearance_dt is not None:
        _positive(clearance_dt, "clearance_dt")
    if out.exists() and not out.is_dir():
        raise ValueError(f"export directory is a file: {out}")
    snapshot = scene._snapshot()
    names = list(snapshot.sequence_names if sequences is None else sequences)
    if len(set(names)) != len(names) or set(names) - set(snapshot.sequence_names):
        raise ValueError("sequences must name distinct programs present in the cell")
    bake = bool(wanted & {"usd", "script", "report"}) and bool(names)
    conditions = {
        "sequences": names,
        "scenarios": ["baseline", *snapshot.scenario_names] if scenarios and bake else (["baseline"] if bake else []),
        "dt": dt, "max_duration": max_duration, "plan_resolution": plan_resolution,
        "clearance_dt": clearance_dt if "report" in wanted and bake else None,
        "fps": fps, "layout_scale": scale, "title": title,
    }
    manifest_name = f"{name}_manifest.json"
    files, issues = [], []
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".botrail-export-", dir=out.parent) as temp:
        stage = Path(temp) / "package"
        stage.mkdir()

        def issue(code, message, path=None):
            issues.append({"code": code, "message": message, "path": path})

        def write(filename, kind, fn, *, program=None):
            path = stage / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists():
                raise ValueError(f"export filenames collide: {filename}")
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                try:
                    returned = fn(path)
                finally:
                    for warning in caught:
                        issue("export_warning", str(warning.message), filename)
            row = {"path": filename, "kind": kind, **_file(path)}
            if program is not None:
                row["sequence"] = program
            files.append(row)
            if isinstance(returned, list):
                for message in returned:
                    issue("export_warning", str(message), filename)
            # USD can emit an accompanying asset directory; list those files too.
            recorded = {r["path"] for r in files}
            for asset in sorted(stage.rglob("*")):
                relative = asset.relative_to(stage).as_posix()
                if asset.is_file() and relative not in recorded:
                    files.append({"path": relative, "kind": kind + "_asset", "parent": filename, **_file(asset)})

        timelines, runs = _bake(snapshot, names, scenarios, dt, max_duration, plan_resolution) if bake else ({}, None)
        if runs is not None:
            for scenario, error in runs.errors.items():
                issue("scenario_execution_failed", f"{scenario}: {error}")
        if "project" in wanted:
            write(f"{name}.botrail", "project", snapshot.save_project)
        if "python" in wanted:
            write(f"{name}.py", "python", lambda p: p.write_text(snapshot.generate_python(), encoding="utf-8"))
        if "bom" in wanted:
            for ext in ("csv", "md"):
                write(f"{name}_bom.{ext}", "bom", snapshot.export_bom)
        if "io" in wanted:
            write(f"{name}_io.csv", "io", lambda p: snapshot.export_io_list(p, sequences=names))
        if "topology" in wanted:
            write(f"{name}_topology.mmd", "topology", lambda p: snapshot.export_topology(p, sequences=names))
        if "plc" in wanted and names:
            filename = f"{name}.plcopen.xml"
            write(filename, "plc", lambda p: snapshot.export_plcopen(p, sequences=names, name=title or name))
            tree = ET.parse(stage / filename)
            stubs = [p.attrib["name"] for p in tree.iter() if p.tag.endswith("}pou")
                     and p.attrib.get("pouType") == "functionBlock"]
            if stubs:
                issue("plcopen_stubs", "Controller implementation required for: " + ", ".join(stubs), filename)
        if "interlocks" in wanted and names:
            for ext in ("md", "csv"):
                write(f"{name}_interlocks.{ext}", "interlocks", lambda p: snapshot.export_interlocks(p, sequences=names))
        if "layout" in wanted:
            write(f"{name}_layout.svg", "layout", lambda p: snapshot.export_layout(p, scale=scale, title=title))
            write(f"{name}_layout.dxf", "layout", lambda p: snapshot.export_layout(p, title=title))
        if "usd" in wanted:
            for cycle, timeline in timelines.items():
                safe = re.sub(r"[^\w.-]", "_", cycle)
                write(f"{name}_{safe}.usda", "usd", lambda p, tl=timeline: tl.export_usd(p, fps=fps))
        if "script" in wanted and timelines:
            compiled = runs if runs is not None else next(iter(timelines.values()))
            for program in names:
                suffix = "" if len(names) == 1 else "_" + re.sub(r"[^\w.-]", "_", program)
                filename = f"{name}{suffix}.script"
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter("always")
                    try:
                        script = compiled.to_script(sequence=program)
                    except ValueError as e:
                        issue("script_not_exported", f"{program}: {e}", filename)
                        script = None
                    finally:
                        for warning in caught:
                            issue("export_warning", str(warning.message), filename)
                if script is not None:
                    write(filename, "script", lambda p, script=script: p.write_text(script, encoding="utf-8"), program=program)
        if not names:
            for kind in sorted(wanted & {"plc", "interlocks", "usd", "script"}):
                issue("no_programs", f"{kind}: no programs selected; no file generated")
        elif "script" in wanted and not timelines:
            issue("script_not_exported", "No completed scenario available for script export")

        if "report" in wanted:
            report = snapshot.cell_report(timelines or None, scenarios=runs, sequences=names,
                                          clearance_dt=clearance_dt, title=title)
            data = json.loads(report.to_json())
            data["deliverables"] = [{k: row[k] for k in ("path", "kind", "sha256", "bytes")} for row in files]
            data["issues"] = list(issues)
            markdown = report.to_markdown() + _deliverables_markdown(files, issues)
            write(f"{name}_report.md", "report_markdown", lambda p: p.write_text(markdown, encoding="utf-8"))
            write(f"{name}_report.json", "report_json",
                  lambda p: p.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"))

        record = {"schema_version": SCHEMA_VERSION, "name": name, "botrail_version": _core.__version__,
                  "conditions": conditions, "files": files, "issues": issues}
        if manifest:
            (stage / manifest_name).write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        # Publish only after every exporter succeeded: move the staged files
        # over, replacing same-named files from an earlier run and leaving
        # the rest of the directory alone.
        out.mkdir(parents=True, exist_ok=True)
        for source in sorted(p for p in stage.rglob("*") if p.is_file()):
            target = out / source.relative_to(stage)
            target.parent.mkdir(parents=True, exist_ok=True)
            os.replace(source, target)
    return out / manifest_name if manifest else out


def _deliverables_markdown(files, issues):
    lines = ["\n## Deliverables\n", "| file | bytes | sha256 |", "|---|---|---|"]
    lines += [f"| {row['path']} | {row['bytes']} | {row['sha256']} |" for row in files]
    if issues:
        lines += ["\n## Export issues\n"]
        lines += [f"- **{i['code']}** {i['path'] or ''}: {i['message']}".replace("  ", " ") for i in issues]
    return "\n".join(lines) + "\n"
