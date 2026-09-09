"""Process setup and declared load checks for spot welding and radial deburring.

Setups belong to a workpiece, not to the immutable product catalog. They are
stored in its Part attributes so projects and generated Python preserve them.
This module checks declared inputs and geometric TCPs; it does not predict weld
quality, cutting forces, servo dynamics or certify a robot load envelope.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

_KEY = "botrail_process_setup_v1"


def _number(value):
    return (
        isinstance(value, (float, int))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _vector(value, length, label):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != length
        or not all(_number(x) for x in value)
    ):
        raise ValueError(f"process: {label} must contain {length} finite numbers")
    return list(value)


def _inertia(value):
    xx, yy, zz, xy, xz, yz = _vector(value, 6, "inertia_kg_m2 [xx, yy, zz, xy, xz, yz]")
    matrix = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    # The second-moment matrix must be positive semidefinite. This also checks
    # the principal-moment triangle inequalities, including rotated tensors.
    half_trace = (xx + yy + zz) / 2
    cov = [
        [(half_trace if i == j else 0) - matrix[i][j] for j in range(3)]
        for i in range(3)
    ]
    a, b, c = (cov[i][i] for i in range(3))
    d, e, f = cov[0][1], cov[0][2], cov[1][2]
    scale = max(abs(x) for row in cov for x in row)
    eps = max(scale, 1e-12) * 1e-10
    if min(a, b, c) < -eps or min(
        a * b - d * d, a * c - e * e, b * c - f * f
    ) < -eps * max(scale, 1e-12):
        raise ValueError(
            "process: inertia must be a physical tensor about the component COM"
        )
    if (
        a * b * c + 2 * d * e * f - a * f * f - b * e * e - c * d * d
        < -eps * max(scale, 1e-12) ** 2
    ):
        raise ValueError(
            "process: inertia must be a physical tensor about the component COM"
        )
    return matrix


def _validated(setup: dict) -> dict:
    setup = json.loads(json.dumps(setup, allow_nan=False))
    if not isinstance(setup, dict):
        raise TypeError("process: setup must be a dict")
    if setup.get("operation") not in ("spot_weld", "radial_deburr"):
        raise ValueError("process: operation must be spot_weld or radial_deburr")
    for key in ("robot", "tcp", "tool_frame", "flange"):
        if not isinstance(setup.get(key), str) or not setup[key]:
            raise ValueError(f"process: missing {key}")
    _vector(setup.get("tcp_offset_m"), 3, "tcp_offset_m")
    if setup.get("tcp_axis") is not None:
        axis = _vector(setup["tcp_axis"], 3, "tcp_axis")
        if abs(math.sqrt(sum(x * x for x in axis)) - 1) > 1e-6:
            raise ValueError("process: tcp_axis must be a unit vector")
    if not isinstance(setup.get("load_components_complete", False), bool):
        raise TypeError("process: load_components_complete must be a boolean")
    for key in ("facts", "settings"):
        if not isinstance(setup.get(key), dict):
            raise TypeError(f"process: {key} must be a dict")
    for name, fact in setup["facts"].items():
        if (
            not isinstance(fact, dict)
            or not {"value", "basis", "source"} <= fact.keys()
        ):
            raise ValueError(f"process: fact {name} needs value, basis and source")
        if not all(
            isinstance(fact[k], str) and fact[k].strip() for k in ("basis", "source")
        ):
            raise ValueError(f"process: fact {name} needs nonempty basis and source")
    if not isinstance(setup.get("load_components", []), list):
        raise TypeError("process: load_components must be a list")
    names = set()
    for component in setup.get("load_components", []):
        if not isinstance(component, dict):
            raise TypeError("process: load component must be a dict")
        if (
            not isinstance(component.get("name"), str)
            or not component.get("name")
            or component["name"] in names
        ):
            raise ValueError("process: load component names must be present and unique")
        names.add(component["name"])
        if not isinstance(component.get("frame"), str) or not component["frame"]:
            raise ValueError("process: load component needs a frame")
        mass = component.get("mass_kg")
        if mass is not None and (not _number(mass) or mass <= 0):
            raise ValueError("process: component mass_kg must be positive or None")
        if component.get("com_m") is not None:
            _vector(component["com_m"], 3, "com_m")
        if component.get("inertia_kg_m2") is not None:
            _inertia(component["inertia_kg_m2"])
    return setup


def configure(scene, target: str, setup: dict) -> None:
    """Save a setup on an existing, identified workpiece Part.

    ``operation`` is ``spot_weld`` or ``radial_deburr``. Required references:
    ``robot``, ``tcp``, ``tool_frame``, ``tcp_offset_m`` and ``flange``. ``facts``
    holds manufacturer/CAD values as ``{value, source, basis}``; ``settings``
    holds this cell's choices. ``load_components`` name frames with mass_kg,
    com_m and COM inertia_kg_m2 [xx, yy, zz, xy, xz, yz], all in SI units.
    Missing measured quantities stay None. No geometry-derived mass is assumed.
    """
    setup = _validated(setup)
    part = scene.part(target)
    if part is None:
        raise ValueError("process: identify the workpiece with scene.set_part first")
    attrs = dict(part.get("attributes") or {})
    attrs[_KEY] = json.dumps(setup, allow_nan=False, separators=(",", ":"))
    identity = {
        k: part[k]
        for k in (
            "kind",
            "catalog",
            "manufacturer",
            "model",
            "category",
            "description",
            "qty",
        )
        if part.get(k) is not None
    }
    scene.set_part(target, attributes=attrs, **identity)


def setup(scene, target: str) -> dict:
    """Return an independent copy of the saved setup."""
    part = scene.part(target)
    if part is None or _KEY not in part.get("attributes", {}):
        raise ValueError(f"process: no setup on {target!r}")
    return _validated(json.loads(part["attributes"][_KEY]))


def targets(scene) -> list[str]:
    return [p["target"] for p in scene.parts() if _KEY in p.get("attributes", {})]


def aggregate_load(
    scene, robot: str, flange: str, components: list[dict], *, complete: bool = False
) -> dict:
    """Combine declared component masses/tensors at the scene's current pose.

    COM and output inertia use flange axes; inertia is about the flange origin.
    Gravity moment uses world gravity (0, 0, -9.80665) m/s², expressed at flange.
    Unknown components invalidate totals instead of becoming zero. This is a
    static calculation, not the robot manufacturer's dynamic payload diagram.
    """
    from .select import _cross, _quat_mul, _rotate, _rotate_inverse

    if not isinstance(complete, bool):
        raise TypeError("process: complete must be a boolean")
    names = [c["name"] for c in components]
    if not all(names) or len(set(names)) != len(names):
        raise ValueError("process: load component names must be present and unique")
    base_p, base_q = scene.link_pose(flange, robot=robot)
    mass_sum = 0.0
    moment = [0.0] * 3
    tensor = [[0.0] * 3 for _ in range(3)]
    missing_mass, missing_com, missing_inertia = [], [], []
    for c in components:
        name, mass = c["name"], c.get("mass_kg")
        p, q = scene.link_pose(c["frame"], robot=robot)
        if mass is None:
            missing_mass.append(name)
            continue
        if not _number(mass) or mass <= 0:
            raise ValueError("process: mass must be finite and positive")
        mass_sum += mass
        if c.get("com_m") is None:
            missing_com.append(name)
            continue
        com = _vector(c["com_m"], 3, "com_m")
        world_com = [a + b for a, b in zip(p, _rotate(q, com))]
        r = _rotate_inverse(base_q, [a - b for a, b in zip(world_com, base_p)])
        for i in range(3):
            moment[i] += mass * r[i]
        if c.get("inertia_kg_m2") is None:
            missing_inertia.append(name)
            continue
        local = _inertia(c["inertia_kg_m2"])
        relative = _quat_mul((-base_q[0], -base_q[1], -base_q[2], base_q[3]), q)
        columns = [_rotate(relative, [int(k == j) for k in range(3)]) for j in range(3)]
        rotation = [[columns[j][i] for j in range(3)] for i in range(3)]
        for i in range(3):
            for j in range(3):
                rotated = sum(
                    rotation[i][a] * local[a][b] * rotation[j][b]
                    for a in range(3)
                    for b in range(3)
                )
                tensor[i][j] += rotated + mass * (
                    (sum(x * x for x in r) if i == j else 0) - r[i] * r[j]
                )
    mass_known = complete and bool(components) and not missing_mass
    com_known = mass_known and not missing_com
    inertia_known = com_known and not missing_inertia
    gravity = _rotate_inverse(base_q, (0, 0, -9.80665))
    return {
        "known_subtotal_kg": mass_sum,
        "mass_kg": mass_sum if mass_known else None,
        "com_m": [x / mass_sum for x in moment] if com_known else None,
        "inertia_at_flange_kg_m2": tensor if inertia_known else None,
        "gravity_moment_nm": list(_cross(moment, gravity)) if com_known else None,
        "components_complete": complete,
        "missing_mass": missing_mass,
        "missing_com": missing_com,
        "missing_inertia": missing_inertia,
        "robot": robot,
        "flange": flange,
    }


@dataclass
class ProcessReport:
    target: str
    checks: list[dict]
    load: dict
    inputs: dict

    @property
    def ready(self) -> bool:
        return bool(self.checks) and all(c["status"] == "pass" for c in self.checks)

    def to_dict(self) -> dict:
        return {
            "target": self.target,
            "ready": self.ready,
            "checks": self.checks,
            "load": self.load,
            "inputs": self.inputs,
        }

    def to_markdown(self) -> str:
        def cell(x):
            return str(x).replace("|", "\\|").replace("\n", " ")

        lines = [
            f"# Process setup: {self.target}\n",
            "Declared process inputs: " + ("complete" if self.ready else "incomplete"),
            "\nGeometric and declared-input checks only; no prediction of weld quality, cutting forces or dynamic load acceptance.",
            "\n| Check | Status | Detail |",
            "| --- | --- | --- |",
        ]
        lines += [
            f"| {cell(c['id'])} | {c['status']} | {cell(c['message'])} |"
            for c in self.checks
        ]
        lines += [
            "\n## Load at the current pose\n",
            "```json",
            json.dumps(self.load, indent=2),
            "```",
        ]
        lines += [
            "\n## Sources\n",
            "| Input | Value | Basis | Source |",
            "| --- | --- | --- | --- |",
        ]
        lines += [
            f"| {cell(k)} | {cell(v['value'])} | {cell(v['basis'])} | {cell(v['source'])} |"
            for k, v in self.inputs["facts"].items()
        ]
        return "\n".join(lines) + "\n"

    def save(self, path: str | Path) -> None:
        path = Path(path)
        if path.suffix not in (".json", ".md"):
            raise ValueError("process report extension must be .json or .md")
        path.write_text(
            json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n"
            if path.suffix == ".json"
            else self.to_markdown()
        )


def report(scene, target: str) -> ProcessReport:
    """Check the saved setup against live frame poses and declared product facts."""
    from .select import _rotate, _rotate_inverse

    scene = scene._snapshot()
    profile = setup(scene, target)
    facts, settings = profile["facts"], profile["settings"]
    checks = []

    def add(name, status, message):
        checks.append({"id": name, "status": status, "message": message})

    def fact(name):
        return facts.get(name, {}).get("value")

    unbounded = object()

    def compare(name, actual, lower=unbounded, upper=unbounded):
        values = [v for v in (actual, lower, upper) if v is not unbounded]
        if any(v is not None and not _number(v) for v in values):
            add(name, "fail", f"{name}: expected finite numeric inputs")
        elif any(v is None for v in values):
            add(name, "unknown", f"{name}: required or selected value is missing")
        else:
            ok = (
                lower is unbounded
                or actual >= lower
                or math.isclose(actual, lower, rel_tol=1e-9, abs_tol=0)
            ) and (
                upper is unbounded
                or actual <= upper
                or math.isclose(actual, upper, rel_tol=1e-9, abs_tol=0)
            )
            bounds = ["unbounded" if v is unbounded else str(v) for v in (lower, upper)]
            add(
                name,
                "pass" if ok else "fail",
                f"{name}: {actual}; allowed {'..'.join(bounds)} (declared inputs)",
            )

    try:
        p, q = scene.link_pose(profile["tool_frame"], robot=profile["robot"])
        tcp, tcp_q = scene.link_pose(profile["tcp"], robot=profile["robot"])
        offset = _rotate_inverse(q, [a - b for a, b in zip(tcp, p)])
        error = math.dist(offset, profile["tcp_offset_m"])
        add(
            "tcp_geometry",
            "pass" if error < 1e-6 else "fail",
            f"TCP offset error {error:.6g} m against selected tool geometry",
        )
        if profile.get("tcp_axis") is not None:
            axis = _rotate_inverse(q, _rotate(tcp_q, (0, 0, 1)))
            error = math.dist(axis, profile["tcp_axis"])
            add(
                "tcp_axis",
                "pass" if error < 1e-6 else "fail",
                f"TCP +Z axis error {error:.6g} against selected tool frame",
            )
    except ValueError as exc:
        add("tcp_geometry", "fail", str(exc))
    add(
        "tcp_calibration",
        "pass" if profile.get("tcp_calibration_ref") else "unknown",
        profile.get("tcp_calibration_ref")
        or "Nominal/CAD TCP; measured calibration record is missing",
    )
    try:
        load = aggregate_load(
            scene,
            profile["robot"],
            profile["flange"],
            profile.get("load_components", []),
            complete=profile.get("load_components_complete", False),
        )
        for field in ("mass_kg", "com_m", "inertia_at_flange_kg_m2"):
            add(
                "load_" + field,
                "pass" if load[field] is not None else "unknown",
                f"{field}: {load[field]}; all mounted parts and moving links must be covered",
            )
        capacity = fact("robot_payload_kg")
        if _number(capacity) and load["known_subtotal_kg"] > capacity:
            add(
                "payload_kg",
                "fail",
                f"Known subtotal {load['known_subtotal_kg']} kg already exceeds declared capacity {capacity} kg",
            )
        else:
            compare("payload_kg", load["mass_kg"], 0, capacity)
        add(
            "robot_load_envelope",
            "pass" if profile.get("load_acceptance_ref") else "unknown",
            profile.get("load_acceptance_ref")
            or "Manufacturer moment/inertia envelope and motion load acceptance not supplied",
        )
    except ValueError as exc:
        load = {}
        add("load_frames", "fail", str(exc))
    if profile["operation"] == "radial_deburr":
        compare("feed_mps", settings.get("feed_mps"), 1e-12)
        compare(
            "collet_shank_mm",
            settings.get("collet_mm"),
            fact("shank_mm"),
            fact("shank_mm"),
        )
        compare(
            "motor_pressure_bar",
            settings.get("motor_pressure_bar"),
            fact("motor_pressure_bar"),
            fact("motor_pressure_bar"),
        )
        compare(
            "compliance_pressure_bar",
            settings.get("compliance_pressure_bar"),
            fact("compliance_pressure_min_bar"),
            fact("compliance_pressure_max_bar"),
        )
        compare("bur_speed_rating_rpm", fact("bur_rated_rpm"), fact("idle_rpm"))
        diameter = fact("bur_diameter_mm")
        compare(
            "radial_engagement_mm",
            settings.get("radial_engagement_mm"),
            0,
            diameter * 0.3 if _number(diameter) else None,
        )
        # Pressure alone cannot establish the air supply's volumetric capacity.
        flow_basis = settings.get("motor_flow_reference")
        if not flow_basis or flow_basis != fact("motor_flow_reference"):
            add(
                "motor_air_flow",
                "unknown",
                "Supply and consumption need the same documented volumetric reference conditions",
            )
        else:
            compare(
                "motor_air_flow",
                settings.get("motor_capacity_l_min"),
                fact("motor_flow_l_min"),
            )
        add(
            "radial_compliance_model",
            "pass" if profile.get("compliance_validation_ref") else "unknown",
            profile.get("compliance_validation_ref")
            or "Nominal rigid tool path; pneumatic compliance/contact force not simulated",
        )
        compare(
            "valve_voltage_v",
            settings.get("valve_voltage_v"),
            fact("valve_voltage_v"),
            fact("valve_voltage_v"),
        )
        compare("air_filter_um", settings.get("filter_um"), 1e-9, fact("filter_max_um"))
        compare(
            "motor_lubrication_drops_min",
            settings.get("lubrication_drops_min"),
            fact("lubrication_min_drops_min"),
            fact("lubrication_max_drops_min"),
        )
        for name in ("separate_regulators", "compliance_air_clean_dry"):
            value = settings.get(name)
            add(
                name,
                "unknown" if value is None else "pass" if value is True else "fail",
                f"{name}: {value} (declared installation)",
            )
        add(
            "bur_insertion",
            "pass" if profile.get("bur_insertion_ref") else "unknown",
            profile.get("bur_insertion_ref")
            or "Exposed projection does not establish sufficient shank insertion in the collet",
        )
    else:
        compare(
            "electrode_force_n",
            settings.get("electrode_force_n"),
            1e-12,
            fact("electrode_force_max_n"),
        )
        compare("weld_current_ka", settings.get("weld_current_ka"), 1e-12)
        compare("weld_time_s", settings.get("weld_time_s"), 1e-12)
        compare("hold_time_s", settings.get("hold_time_s"), 1e-12)
        compare(
            "coolant_l_min",
            settings.get("coolant_l_min"),
            fact("transformer_reference_coolant_l_min"),
        )
        add(
            "whole_gun_cooling",
            "pass" if profile.get("cooling_validation_ref") else "unknown",
            profile.get("cooling_validation_ref")
            or "Series transformer flow is not a complete gun/arm/electrode cooling-circuit requirement",
        )
        add(
            "weld_power_controller",
            "pass" if profile.get("power_controller_ref") else "unknown",
            profile.get("power_controller_ref")
            or "Inverter, primary supply and gun-drive controller compatibility remain unspecified; kVA is not mains current",
        )
    add(
        "process_qualification",
        "pass" if profile.get("qualification_ref") else "unknown",
        profile.get("qualification_ref")
        or "Material, tool and process settings need a measured qualification record",
    )
    return ProcessReport(target, checks, load, profile)
