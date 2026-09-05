"""Generate a document set from one isolated cell and verify its provenance.

The manifest records the serialized authored definition, observed geometry
assets, execution conditions and implementation digests. It is an integrity
record, not a signature or an engineering acceptance certificate.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
import tempfile
import warnings
from pathlib import Path
from xml.etree import ElementTree as ET

EXPORTS = ("project", "python", "bom", "io", "topology", "plc", "interlocks", "layout", "usd", "script", "connections", "report")
SCHEMA_VERSION = 1


def _json(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def _digest(value):
    return hashlib.sha256(_json(value).encode("utf-8")).hexdigest()


def _file(path):
    digest = hashlib.sha256()
    size = 0
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            size += len(chunk)
    return {"sha256": digest.hexdigest(), "bytes": size}


def _input(scene):
    project = json.loads(scene._project_json())
    assets = [{"path": str(Path(p).absolute()), **_file(p)} for p in scene._asset_paths()]
    catalog = []

    def source(value, target):
        if value["kind"] == "catalog":
            catalog.append({"target": target, "id": value["id"], "revision": value.get("revision")})
            source(value["inner"], target)
        elif value["kind"] == "composite":
            source(value["base"], target)
            source(value["tool"], target + "/tool")

    for robot in project["robots"]:
        source(robot["source"], robot["name"])
    for part in scene.parts():
        if part.get("catalog"):
            id, separator, revision = part["catalog"].partition("@")
            catalog.append({"target": part["target"], "id": id, "revision": revision if separator else None})
    return {"project": project, "assets": assets, "catalog": catalog}


def _generator():
    from . import _core

    root = Path(__file__).parent
    return {
        "botrail_version": _core.__version__,
        "validator": "botrail-export-manifest-v1",
        "core_sha256": _file(_core.__file__)["sha256"],
        "python_sha256": {p.relative_to(root).as_posix(): _file(p)["sha256"]
                          for p in sorted(root.rglob("*.py"))},
    }


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
                fps: float = 30., scale: float = 100., attachments=None) -> Path:
    """Write a manifest and selected documents from a fresh snapshot and bake.

    ``out`` must be absent or empty; publish into a new directory for each
    revision. ``exports=None`` means all formats. Every program-dependent
    exporter uses ``sequences`` (all by default). Bakes are kinematic.
    Existing timelines/scripts cannot be substituted for generated results.
    ``attachments`` are copied and explicitly remain unverified external inputs.
    The returned path is usable with :func:`verify_export` and ``bt.review``.
    Export warnings and omitted programs are recorded as unresolved issues.
    """
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
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError(f"export directory must be empty; use a new directory for this revision: {out}")
    snapshot = scene._snapshot()
    names = list(snapshot.sequence_names if sequences is None else sequences)
    if len(set(names)) != len(names) or set(names) - set(snapshot.sequence_names):
        raise ValueError("sequences must name distinct programs present in the cell")
    inputs = _input(snapshot)
    input_sha256 = _digest(inputs)
    generator = _generator()
    bake = bool(wanted & {"usd", "script", "report"}) and bool(names)
    conditions = {
        "sequences": names, "scenarios": ["baseline", *snapshot.scenario_names] if scenarios and bake else
        (["baseline"] if bake else []), "simulation_performed": bake,
        "dt": dt, "max_duration": max_duration, "plan_resolution": plan_resolution,
        "physics": "kinematic", "clearance_dt": clearance_dt if "report" in wanted and bake else None,
        "fps": fps, "layout_scale": scale, "ground_z": 0.02, "title": title, "name": name,
        "exports": sorted(wanted), "plc": {"cycle": True, "task_interval_ms": 10},
        "script": {"dialect": "urscript", "speed_scale": 1., "blend_radius": 0.,
                   "tcp_speed": 0.25, "tcp_accel": 1.2, "move_to_start": True},
    }
    run = {"input_sha256": input_sha256, "conditions": conditions, "generator": generator}
    run_sha256 = _digest(run)
    manifest_name = f"{name}_manifest.json"
    files, issues = [], []
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".botrail-export-", dir=out.parent) as temp:
        stage = Path(temp) / "package"
        stage.mkdir()

        def issue(code, message, path=None):
            issues.append({"code": code, "message": message, "path": path})

        def write(filename, kind, fn, *, origin="generated", program=None):
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
            row = {"path": filename, "kind": kind, "origin": origin, **_file(path)}
            if origin == "generated":
                row.update(input_sha256=input_sha256, run_sha256=run_sha256)
            if program is not None:
                row["sequence"] = program
            files.append(row)
            if isinstance(returned, list):
                for message in returned:
                    issue("export_warning", str(message), filename)
            # USD can emit an accompanying asset directory. Those files
            # are deliverables too, with the same input/run provenance.
            recorded = {r["path"] for r in files}
            for asset in sorted(stage.rglob("*")):
                relative = asset.relative_to(stage).as_posix()
                if asset.is_file() and relative not in recorded:
                    files.append({"path": relative, "kind": kind + "_asset", "origin": origin,
                                  "input_sha256": input_sha256, "run_sha256": run_sha256,
                                  "parent": filename, **_file(asset)})

        timelines, runs = _bake(snapshot, names, scenarios, dt, max_duration, plan_resolution) if bake else ({}, None)
        if runs is not None:
            for scenario, error in runs.errors.items():
                issue("scenario_execution_failed", f"{scenario}: {error}")
        for ref in inputs["catalog"]:
            if not ref["revision"]:
                issue("catalog_revision_unknown", f"{ref['target']}: {ref['id']} has no pinned catalog revision")
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
        physical = None
        if wanted & {"connections", "report"}:
            from .connections import report as connection_report

            physical = connection_report(snapshot)
        if "connections" in wanted:
            for ext in ("csv", "md", "json"):
                write(f"{name}_connections.{ext}", "connections", physical.save)
            write(f"{name}_power.csv", "power", lambda p: physical.save(p, table="power"))
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
        for index, attachment in enumerate(attachments or []):
            path = Path(attachment)
            write(f"attachments/{index}_{path.name}", "attachment", lambda p, src=path: shutil.copyfile(src, p),
                  origin="external_attachment")

        if "report" in wanted:
            report = snapshot.cell_report(timelines or None, scenarios=runs, sequences=names,
                                          clearance_dt=clearance_dt, title=title)
            data = json.loads(report.to_json())
            data["deliverables"] = list(files)
            data["provenance"] = {**run, "run_sha256": run_sha256, "manifest": manifest_name,
                                  "catalog": inputs["catalog"], "assets": inputs["assets"]}
            data["issues"] = list(issues)
            data["connections"] = physical.to_dict()
            connection_markdown = re.sub(r"^(#+) ", r"#\1 ", physical.to_markdown(), flags=re.MULTILINE)
            markdown = report.to_markdown() + "\n" + connection_markdown + _provenance_markdown(data)
            write(f"{name}_report.md", "report_markdown", lambda p: p.write_text(markdown, encoding="utf-8"))
            write(f"{name}_report.json", "report_json", lambda p: p.write_text(_json(data) + "\n", encoding="utf-8"))

        if _input(snapshot) != inputs:
            raise ValueError("cell or geometry assets changed during export; no package published")
        manifest = {"schema_version": SCHEMA_VERSION, "input": inputs, **run,
                    "run_sha256": run_sha256, "files": files, "issues": issues}
        manifest["manifest_sha256"] = _digest(manifest)
        (stage / manifest_name).write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        verified = verify_export(stage / manifest_name)
        if not verified["ok"]:
            raise ValueError("export verification failed: " + "; ".join(verified["errors"]))
        # Publish only after every exporter and asset check succeeds. Refuse
        # to combine this revision with existing/untracked files.
        if out.exists():
            out.rmdir()  # fails if a concurrent writer populated it
        stage.replace(out)
    return out / manifest_name


def _provenance_markdown(report):
    from .review import _md

    p = report["provenance"]
    lines = ["\n## Input revision\n", f"- Input SHA-256: `{p['input_sha256']}`",
             f"- Run SHA-256: `{p['run_sha256']}`", f"- Manifest: `{p['manifest']}`",
             f"- botrail: {p['generator']['botrail_version']}; validator: {p['generator']['validator']}",
             "\nGenerated files use one isolated cell and program scope. External attachments have unverified provenance.",
             "Verify the package before review; matching revisions do not establish engineering acceptance.",
             "\n### Execution conditions\n", "```json", json.dumps(p["conditions"], indent=2), "```",
             "\n### Catalog and geometry inputs\n", "| target / file | catalog revision / SHA-256 |", "|---|---|"]
    for ref in p["catalog"]:
        lines.append(f"| {_md(ref['target'] + ': ' + ref['id'])} | {_md(ref['revision'] or 'unknown')} |")
    for asset in p["assets"]:
        lines.append(f"| {_md(asset['path'])} | {asset['sha256']} |")
    if not p["catalog"] and not p["assets"]:
        lines.append("| Embedded definitions / primitives | Recorded in manifest input |")
    lines.extend(["\n## Deliverables\n", "| file | origin | bytes | SHA-256 |", "|---|---|---|---|"])
    for row in report["deliverables"]:
        lines.append(f"| {_md(row['path'])} | {row['origin']} | {row['bytes']} | {row['sha256']} |")
    lines.append("\n## Unresolved export issues\n")
    lines.extend(f"- **{i['code']}** {_md(i['path'] or '')}: {_md(i['message'])}" for i in report["issues"])
    if not report["issues"]:
        lines.append("No exporter issues recorded. Project acceptance criteria are evaluated separately.")
    return "\n".join(lines) + "\n"


def verify_export(manifest: str | Path, *, scene=None) -> dict:
    """Check manifest consistency and every file, optionally against a live cell.

    ``ok`` covers integrity and the optional current serialized input match.
    ``same_revision`` additionally requires no external attachments. Exporter
    warnings remain in ``issues``; neither boolean is engineering acceptance.
    Source assets need not be installed when checking a handed-over package.
    Unlisted files, missing files and paths outside the package are rejected.
    """
    path = Path(manifest)
    errors = []
    result = {"ok": False, "same_revision": False, "errors": errors, "files": [], "issues": []}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data["schema_version"] != SCHEMA_VERSION:
            raise ValueError("unsupported export manifest version")
        if data["manifest_sha256"] != _digest({k: v for k, v in data.items() if k != "manifest_sha256"}):
            errors.append("manifest digest mismatch")
        if data["input_sha256"] != _digest(data["input"]):
            errors.append("input fingerprint mismatch")
        run = {key: data[key] for key in ("input_sha256", "conditions", "generator")}
        if data["run_sha256"] != _digest(run):
            errors.append("run fingerprint mismatch")
        if scene is not None and data["input_sha256"] != _digest(_input(scene._snapshot())):
            errors.append("current cell differs from the recorded input revision")
        root = path.parent.resolve()
        listed = {path.name}
        for row in data["files"]:
            relative = Path(row["path"])
            file_path = root / relative
            if relative.is_absolute() or ".." in relative.parts or root not in file_path.resolve().parents:
                raise ValueError(f"file path escapes package: {relative}")
            if row["path"] in listed:
                raise ValueError(f"duplicate file path: {relative}")
            listed.add(row["path"])
            if row["origin"] == "generated":
                if row.get("input_sha256") != data["input_sha256"] or row.get("run_sha256") != data["run_sha256"]:
                    errors.append(f"{relative}: generated file has a different input/run revision")
            elif row["origin"] != "external_attachment":
                raise ValueError(f"unknown origin: {row['origin']}")
            try:
                if _file(file_path) != {key: row[key] for key in ("sha256", "bytes")}:
                    errors.append(f"{relative}: file digest/size mismatch")
                if row["kind"] == "report_json":
                    report = json.loads(file_path.read_text(encoding="utf-8"))
                    provenance = report["provenance"]
                    if any(provenance[key] != value for key, value in run.items()) or provenance["run_sha256"] != data["run_sha256"]:
                        errors.append(f"{relative}: report input/run revision mismatch")
                    documents = [r for r in data["files"] if r["kind"] not in ("report_json", "report_markdown")]
                    if report["deliverables"] != documents or report["issues"] != data["issues"]:
                        errors.append(f"{relative}: report deliverables/issues differ from manifest")
                    if report["io"] is not None and report["io"]["sequences"] != data["conditions"]["sequences"]:
                        errors.append(f"{relative}: report I/O program scope mismatch")
            except OSError as e:
                errors.append(f"{relative}: {e}")
        extra = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()} - listed
        if extra:
            errors.append(f"unlisted files in package: {sorted(extra)}")
        result.update(files=data["files"], issues=data["issues"], input_sha256=data["input_sha256"],
                      run_sha256=data["run_sha256"], conditions=data["conditions"])
        result["ok"] = not errors
        result["same_revision"] = not errors and bool(data["files"]) and all(r["origin"] == "generated" for r in data["files"])
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
        errors.append(f"invalid export package: {e}")
    return result
