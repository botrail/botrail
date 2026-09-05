"""Physical interface requirements over existing equipment and I/O assignments.

Ports describe declared interfaces; they do not execute the cell. Ratings
are for one endpoint in total. Unknown consumption is never counted as zero.
"""

from __future__ import annotations

import csv
import io
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

__all__ = ["ConnectionReport", "connect", "disconnect", "port", "remove_port", "report", "restore"]


def _plan(scene):
    return json.loads(scene._connection_plan_json())


def restore(scene, plan: dict) -> None:
    """Restore a serialized connection plan, including unresolved references.

    The typed project schema validates its structure. :func:`report` checks
    its design meaning and names references broken by equipment/port removal.
    """
    scene._set_connection_plan_json(json.dumps(plan, allow_nan=False))


def port(scene, name: str, target: str, medium: str, role: str, *, target_kind=None,
         required: bool | None = None, io=None, terminal=None, reference=None, **specs) -> str:
    """Define or replace a named interface on an existing resident.

    ``medium`` is power/pneumatic/signal/network. Roles are supply/load for
    utilities, output/input for signals, peer for networks. Supply ports are
    optional by default; other ports require a connection. ``io`` references
    an existing controller assignment: ``{point, direction, node, aspect?}``.
    ``terminal`` and ``reference`` are external drawing/specification labels.

    Power specs: voltage_v (or voltage_min_v/max_v), current_a (load),
    capacity_a (supply). Pneumatic specs: pressure_bar (or min/max),
    flow_l_min / capacity_l_min and flow_reference. Signal specs:
    signal_type (digital/safe_digital/analog/word), voltage range and logic
    (pnp/npn). Networks declare protocol. Values apply to this endpoint's
    total load or capacity; no diversity factor or W/A conversion is inferred.
    """
    kind = scene._part_target_kind(target, target_kind)
    value = {"name": name, "target": target, "target_kind": kind, "medium": medium, "role": role,
             "required": role != "supply" if required is None else required, "specs": specs,
             "io": io, "terminal": terminal, "reference": reference}
    plan = _plan(scene)
    index = next((i for i, p in enumerate(plan["ports"]) if p["name"] == name), None)
    if index is None:
        plan["ports"].append(value)
    else:
        plan["ports"][index] = value
    restore(scene, plan)
    return name


def connect(scene, source: str, target: str, *, name: str | None = None,
            cable: str | None = None, reference: str | None = None) -> str:
    """Connect declared ports. Reusing a connection name replaces that link.

    Invalid medium/direction pairs are kept for review, like incompatible
    I/O assignments. Cable identifiers are references, not added BOM lines.
    """
    plan = _plan(scene)
    names = {p["name"] for p in plan["ports"]}
    if source not in names or target not in names:
        raise ValueError("connect: both endpoints must be declared ports")
    name = name if name is not None else f"{source} -> {target}"
    value = {"name": name, "source": source, "target": target, "cable": cable, "reference": reference}
    index = next((i for i, c in enumerate(plan["links"]) if c["name"] == name), None)
    if index is None:
        plan["links"].append(value)
    else:
        plan["links"][index] = value
    restore(scene, plan)
    return name


def disconnect(scene, name: str) -> None:
    """Remove a named connection; required endpoints become unconnected."""
    plan = _plan(scene)
    if not any(c["name"] == name for c in plan["links"]):
        raise ValueError(f"unknown connection: {name}")
    plan["links"] = [c for c in plan["links"] if c["name"] != name]
    restore(scene, plan)


def remove_port(scene, name: str) -> None:
    """Remove a port, retaining links so broken references remain visible."""
    plan = _plan(scene)
    if not any(p["name"] == name for p in plan["ports"]):
        raise ValueError(f"unknown connection port: {name}")
    plan["ports"] = [p for p in plan["ports"] if p["name"] != name]
    restore(scene, plan)


def _status(values):
    values = list(values)
    return next((s for s in ("fail", "unknown", "not_run") if s in values),
                "pass" if "pass" in values else "not_applicable")


def _number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0


def _cell(value):
    if isinstance(value, float):
        return format(value, ".8g")
    return str(value if value is not None else "unknown").replace("|", "\\|").replace("\n", "<br>")


@dataclass
class ConnectionReport:
    """Connection checks, resolved endpoint specs and source capacity tables.

    ``ready`` applies only to declared requirements and the missing utility
    declarations identifiable from Part attributes; it is not installation
    approval. No declared requirements is ``not_run``.
    """
    ports: list[dict]
    connections: list[dict]
    supplies: list[dict]
    checks: list[dict]

    @property
    def ready(self) -> bool:
        return _status(c["status"] for c in self.checks) in ("pass", "not_applicable")

    def to_dict(self) -> dict:
        return {"ready": self.ready, "ports": self.ports, "connections": self.connections,
                "supplies": self.supplies, "checks": self.checks}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def to_markdown(self) -> str:
        lines = ["# Equipment connections\n", "Declared requirements: " + ("resolved" if self.ready else "unresolved"),
                 "\nRatings are declared endpoint totals. Electrical protection, cable sizing and dynamic behaviour are not evaluated.",
                 "\n## Connection requirements\n", "| port | equipment | medium / role | required | status | reference |",
                 "|---|---|---|---|---|---|"]
        for p in self.ports:
            lines.append("| " + " | ".join(_cell(v) for v in (
                p["name"], p["target"], p["medium"] + " / " + p["role"], p["required"], p["status"],
                p.get("reference") or "")) + " |")
        lines.extend(["\n## Connections\n", "| name | from | to | medium | status | cable / reference |", "|---|---|---|---|---|---|"])
        for c in self.connections:
            lines.append("| " + " | ".join(_cell(c.get(k)) for k in ("name", "source", "target", "medium", "status"))
                         + " | " + _cell(" / ".join(filter(None, (c.get("cable"), c.get("reference"))))) + " |")
        lines.extend(["\n## Supply capacity\n", "| supply | medium | capacity | known subtotal | known / connected loads | missing | status |",
                      "|---|---|---|---|---|---|---|"])
        for s in self.supplies:
            lines.append("| " + " | ".join(_cell(v) for v in (
                s["source"], s["medium"], f"{_cell(s['capacity'])} {s['unit']}",
                f"{_cell(s['known_subtotal'])} {s['unit']}", f"{s['known_loads']} / {s['total_loads']}",
                ", ".join(s["missing_loads"]), s["status"])) + " |")
        lines.extend(["\n## Checks\n", "| status | target | finding | basis / next action |", "|---|---|---|---|"])
        for c in self.checks:
            lines.append("| " + " | ".join(_cell(c[k]) for k in ("status", "target", "message", "basis")) + " |")
        return "\n".join(lines) + "\n"

    def to_csv(self, table: str = "connections") -> str:
        """Requirements (one row per port) or power supply capacity CSV."""
        if table == "connections":
            keys = ["name", "target", "target_kind", "medium", "role", "required", "status", "connected_to",
                    "terminal", "reference", "io", "specs", "resolved", "links"]
            rows = self.ports
        elif table == "power":
            keys = ["source", "target", "capacity", "known_subtotal", "unit", "known_loads", "total_loads",
                    "missing_loads", "loads", "status", "voltage"]
            rows = [s for s in self.supplies if s["medium"] == "power"]
        else:
            raise ValueError("connection CSV table must be connections or power")
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(row[k], ensure_ascii=False) if isinstance(row.get(k), (dict, list))
                             else row.get(k) for k in keys})
        return stream.getvalue()

    def save(self, path: str | Path, *, table: str = "connections") -> None:
        path = Path(path)
        if path.suffix == ".json":
            text = self.to_json()
        elif path.suffix == ".md":
            text = self.to_markdown()
        elif path.suffix == ".csv":
            text = self.to_csv(table)
        else:
            raise ValueError("connection report extension must be .json, .md or .csv")
        path.write_text(text, encoding="utf-8")


def report(scene) -> ConnectionReport:
    """Check physical requirements for the entire cell, independent of bake scope.

    A single port on a qty=1 equipment target can inherit corresponding Part
    attributes. Multiple ports of the same medium/role, and grouped qty>1
    targets, require explicit port totals. I/O terminals read their existing
    channel kind/electrical data. Every resolved value records its origin.
    """
    scene = scene._snapshot()
    plan = _plan(scene)
    ports = {p["name"]: {**p, "resolved": {}, "connected_to": [], "links": []} for p in plan["ports"]}
    checks, rows, supplies = [], [], []
    parts = {(p["kind"], p["target"]): p for p in scene.parts()}
    counts = Counter((p["target_kind"], p["target"], p["medium"], p["role"]) for p in ports.values())
    iomap = json.loads(scene.io_map().to_json())
    nodes = {n["name"]: n for n in iomap.get("nodes", [])}
    port_checks = {name: [] for name in ports}

    def check(id, target, status, message, basis, *, related=(), **evidence):
        entry = {"id": id, "target": target, "status": status, "message": message, "basis": basis, "evidence": evidence}
        checks.append(entry)
        for name in related:
            port_checks[name].append(status)
        return entry

    if not ports:
        check("requirements:none", "cell", "not_run", "No physical interface requirements declared",
              "Declare the interfaces required by the equipment specifications; omitted interfaces are not inferred")
    for name, p in ports.items():
        try:
            scene._part_target_kind(p["target"], p["target_kind"])
            exists = True
        except ValueError:
            exists = False
        check(f"port:{name}:target", name, "pass" if exists else "fail",
              "Equipment reference exists" if exists else f"Missing {p['target_kind']}: {p['target']}",
              "Existing Scene resident; ports do not add equipment", related=[name])
        identity = parts.get((p["target_kind"], p["target"]))
        attrs = (identity or {}).get("attributes") or {}
        single = counts[p["target_kind"], p["target"], p["medium"], p["role"]] == 1 and (identity or {}).get("qty", 1) == 1
        applicable = {
            "power": {"voltage_v", "voltage_min_v", "voltage_max_v", "capacity_a" if p["role"] == "supply" else "current_a"},
            "pneumatic": {"pressure_bar", "pressure_min_bar", "pressure_max_bar", "flow_reference",
                          "capacity_l_min" if p["role"] == "supply" else "flow_l_min"},
            "signal": {"voltage_v", "voltage_min_v", "voltage_max_v", "signal_type", "logic"},
            "network": {"protocol"},
        }[p["medium"]]
        for key, value in p["specs"].items():
            origin = "port" if value is not None else "unknown"
            attribute = key
            if value is None and key in applicable and single and exists:
                if p["medium"] == "power" and p["role"] == "supply":
                    attribute = {"voltage_v": "output_v", "capacity_a": "output_a"}.get(key, key)
                if attribute in attrs:
                    candidate = attrs[attribute]
                    textual = key in ("logic", "protocol", "signal_type", "flow_reference")
                    if (isinstance(candidate, str) and candidate.strip()) if textual else _number(candidate):
                        value, origin = candidate, "part"
            p["resolved"][key] = {"value": value, "source": origin, "attribute": attribute,
                                   "reference": p.get("reference")}
        if p.get("io"):
            _io_reference(scene, p, iomap, nodes, check)
        for dimension, unit in (("voltage", "v"), ("pressure", "bar")):
            nominal = _value(p, f"{dimension}_{unit}")
            low, high = (_value(p, f"{dimension}_{bound}_{unit}") for bound in ("min", "max"))
            contradictory = (low is not None and high is not None and low > high) or (
                nominal is not None and ((low is not None and nominal < low) or (high is not None and nominal > high)))
            if contradictory:
                check(f"port:{name}:{dimension}:definition", name, "fail", f"Contradictory {dimension} specification",
                      "Check the declared nominal value and bounds", related=[name])
        for key, allowed in (("signal_type", {"digital", "safe_digital", "analog", "word"}), ("logic", {"pnp", "npn"})):
            value = _value(p, key)
            if value is not None and value not in allowed:
                check(f"port:{name}:{key}:unsupported", name, "unknown", f"Unsupported {key}: {value}",
                      "No compatibility rule is implemented for this value", related=[name])

    assignments = Counter(tuple(p["io_assignment"]) for p in ports.values() if p.get("io_assignment"))
    for name, p in ports.items():
        if p.get("io_assignment") and assignments[tuple(p["io_assignment"])] > 1:
            check(f"port:{name}:duplicate_io", name, "fail", "Several declared ports reference the same I/O channel",
                  "Use a single controller port and explicit connections", related=[name])

    for link in plan["links"]:
        a, b = ports.get(link["source"]), ports.get(link["target"])
        before = len(checks)
        known = [p["name"] for p in (a, b) if p]
        if a is None or b is None:
            check(f"link:{link['name']}:reference", link["name"], "fail", "Connection references a missing port",
                  "Restore or explicitly remove the dangling connection", related=known, connection=link)
        else:
            for p, other in ((a, b), (b, a)):
                p["connected_to"].append(other["name"])
                p["links"].append(link)
            compatible = a["name"] != b["name"] and a["medium"] == b["medium"] and (a["role"], b["role"]) in (
                ("supply", "load"), ("output", "input"), ("peer", "peer"))
            check(f"link:{link['name']}:type", link["name"], "pass" if compatible else "fail",
                  "Endpoint types and direction agree" if compatible else "Incompatible medium, direction or self-connection",
                  "Power/air supply → load; signal output → input; network peer ↔ peer", related=known)
            if compatible:
                medium = a["medium"]
                if medium == "power" or (medium == "signal" and not all(_value(p, "signal_type") == "word" for p in (a, b))):
                    _range_check(link["name"], a, b, "voltage", "v", check)
                elif medium == "pneumatic":
                    _range_check(link["name"], a, b, "pressure", "bar", check)
                if medium == "network":
                    _equal_check(link["name"], a, b, "protocol", check)
                elif medium == "signal":
                    _equal_check(link["name"], a, b, "signal_type", check)
                    if any(_value(p, "signal_type") in ("digital", "safe_digital") for p in (a, b)):
                        _equal_check(link["name"], a, b, "logic", check)
                    for controller, field in ((a, b), (b, a)):
                        point = controller.get("io_point")
                        if point and ((field["target_kind"] == "sensor" and point["source"] == "sensor") or
                                      (field["target_kind"] == "device" and point["source"].startswith("device"))):
                            check(f"link:{link['name']}:field", link["name"], "pass" if field["target"] == point["name"] else "fail",
                                  f"Field equipment {field['target']} / simulated I/O source {point['name']}",
                                  "Existing sensor/device I/O identity", related=known)
        rows.append({**link, "medium": a["medium"] if a else None,
                     "status": _status(c["status"] for c in checks[before:])})

    for name, p in ports.items():
        links = [c for c in plan["links"] if name in (c["source"], c["target"])]
        required = p["required"]
        status = "pass" if links else "unknown" if required else "not_applicable"
        check(f"port:{name}:connection", name, status,
              "Connection recorded" if links else "Required interface is unconnected" if required else "Optional interface is unconnected",
              "Declared interface requirement, not inferred from the operating sequence", related=[name])
        # One physical terminal/input has one upstream connection. Fan-out
        # belongs at supply/output ports, not at a multiply-fed load.
        if p["role"] in ("load", "input", "peer") and len(links) > 1:
            check(f"port:{name}:multiple", name, "fail", "Multiple connections terminate at one input/port",
                  "Declare distinct ports for multiple feeds or network sockets", related=[name])
        if p["role"] == "load" and (required or links):
            key = "current_a" if p["medium"] == "power" else "flow_l_min"
            demand = _value(p, key)
            check(f"port:{name}:demand", name, "pass" if _number(demand) else "unknown",
                  f"Declared endpoint consumption: {demand} ({key})", "Total steady demand at this endpoint; supply an explicit value if unknown",
                  related=[name], specification=p["resolved"][key])
        if p["role"] == "supply" and counts[p["target_kind"], p["target"], p["medium"], p["role"]] > 1:
            check(f"port:{name}:shared_capacity", name, "unknown", "Equipment has multiple supply ports; aggregate/shared capacity is not evaluated",
                  "Confirm independent ratings or model a common supply port with fan-out", related=[name])

    # Known utility needs must not disappear merely because no port was authored.
    for (kind, target), part in parts.items():
        attrs = part.get("attributes") or {}
        needed = []
        if part.get("category") == "power_supply" or "output_a" in attrs:
            needed.append(("power", "supply"))
        if "current_a" in attrs:
            needed.append(("power", "load"))
        if "flow_l_min" in attrs:
            needed.append(("pneumatic", "load"))
        for medium, role in needed:
            if not any(p["target_kind"] == kind and p["target"] == target and p["medium"] == medium and p["role"] == role
                       for p in ports.values()):
                check(f"requirements:{kind}:{target}:{medium}:{role}", target, "unknown",
                      f"Part declares a {medium} {role} but has no interface port", "Declare its connection and applicable specifications")

    for node in nodes.values():
        if node.get("uplink"):
            parent = node["uplink"]["parent"]
            represented = any(a["medium"] == "network" and b["medium"] == "network" and
                              a["target_kind"] == b["target_kind"] == "io_node" and
                              {a["target"], b["target"]} == {node["name"], parent}
                              for c in plan["links"] if (a := ports.get(c["source"])) and (b := ports.get(c["target"])))
            if not represented:
                check(f"requirements:uplink:{node['name']}", node["name"], "unknown",
                      f"Uplink to {parent} has no network interface requirements",
                      "Declare endpoint protocols; the existing uplink bus label is not a capability specification")

    for name, p in ports.items():
        if p["role"] != "supply":
            continue
        loads = list(dict.fromkeys(c["target"] for c in plan["links"] if c["source"] == name))
        key, capacity_key, unit = ("current_a", "capacity_a", "A") if p["medium"] == "power" else (
            "flow_l_min", "capacity_l_min", "L/min")
        known, missing = [], []
        for load_name in loads:
            load = ports.get(load_name)
            value = _value(load, key) if load else None
            valid = load is not None and load["role"] == "load" and load["medium"] == p["medium"] and _number(value)
            if p["medium"] == "pneumatic":
                valid = valid and bool(_value(p, "flow_reference")) and _value(p, "flow_reference") == _value(load, "flow_reference")
            if valid:
                known.append({"port": load_name, "value": value, "origin": load["resolved"][key]})
            else:
                missing.append(load_name)
        overflow = False
        try:
            subtotal = math.fsum(k["value"] for k in known) if known else None
        except OverflowError:
            subtotal, overflow = None, True
        capacity = _value(p, capacity_key)
        if not loads:
            status, message = "not_applicable", "No connected loads; supply capacity not evaluated"
        elif overflow or (subtotal is not None and _number(capacity) and subtotal > capacity and
                         not math.isclose(subtotal, capacity, rel_tol=1e-12, abs_tol=1e-12)):
            status, message = "fail", "Known connected demand exceeds supply capacity"
        elif missing or not _number(capacity):
            status, message = "unknown", "Supply capacity or connected consumption is incomplete"
        else:
            status, message = "pass", "Declared connected demand fits the supply capacity"
        check(f"supply:{name}:capacity", name, status, message,
              "Only directly connected endpoint totals; no demand factor, peak estimate or unit conversion" +
              ("; flow reference conditions must match" if p["medium"] == "pneumatic" else ""), related=[name],
              capacity=capacity, known_subtotal=subtotal, missing_loads=missing, loads=known, unit=unit)
        supply_status = _status([*port_checks[name], *(s for n in loads for s in port_checks.get(n, ["fail"]))])
        if not loads and supply_status == "pass":
            supply_status = "not_applicable"
        supplies.append({"source": name, "target": p["target"], "target_kind": p["target_kind"], "medium": p["medium"],
                         "capacity": capacity, "known_subtotal": subtotal, "unit": unit,
                         "known_loads": len(known), "total_loads": len(loads), "missing_loads": missing, "loads": known,
                         "voltage": _interval(p, "voltage", "v"),
                         "status": supply_status})
    for name, p in ports.items():
        p["status"] = _status(port_checks[name])
        if p["status"] == "pass" and not p["required"] and not p["links"]:
            p["status"] = "not_applicable"
    for row in rows:
        row["status"] = _status([row["status"], *(ports.get(n, {}).get("status", "fail") for n in (row["source"], row["target"]))])
    return ConnectionReport(list(ports.values()), rows, supplies, checks)


def _value(port, key):
    return (port or {}).get("resolved", {}).get(key, {}).get("value")


def _interval(port, dimension, unit):
    low, high = (_value(port, f"{dimension}_{bound}_{unit}") for bound in ("min", "max"))
    if low is None and high is None:
        low = high = _value(port, f"{dimension}_{unit}")
    return [low, high]


def _range_check(name, source, target, dimension, unit, check):
    a, b = _interval(source, dimension, unit), _interval(target, dimension, unit)
    known = all(_number(v) for v in [*a, *b])
    status = "unknown" if not known else "pass" if b[0] <= a[0] <= a[1] <= b[1] else "fail"
    check(f"link:{name}:{dimension}", name, status,
          f"{dimension}: supplied {a}, accepted {b} {unit}",
          "The entire declared supply range must fit the accepted range; nominal-only means an exact declared value",
          related=[source["name"], target["name"]], supplied=a, accepted=b)


def _equal_check(name, source, target, key, check):
    a, b = _value(source, key), _value(target, key)
    status = "unknown" if not a or not b else "pass" if a.strip().casefold() == b.strip().casefold() else "fail"
    check(f"link:{name}:{key}", name, status, f"{key}: {a or 'unknown'} → {b or 'unknown'}",
          "Declared interface compatibility; timing, throughput and protocol execution are not evaluated",
          related=[source["name"], target["name"]], source=a, destination=b)


def _io_reference(scene, port, iomap, nodes, check):
    ref, name = port["io"], port["name"]
    expected_role = ref["direction"]
    node = nodes.get(ref["node"])
    binding = next((b for b in iomap.get("bindings", []) if b["point"]["name"] == ref["point"]
                    and b["point"]["direction"] == ref["direction"] and b["point"].get("aspect") == ref.get("aspect")
                    and b["node"] == ref["node"]), None)
    valid = ref["node"] == port["target"] and port["role"] == expected_role and node is not None
    try:
        points = json.loads(scene.io_list("json"))["points"]
        point = next((p for p in points if p["name"] == ref["point"] and p["direction"] == ref["direction"] and
                      p.get("aspect") == ref.get("aspect")), None)
        present = point is not None
    except ValueError:
        present = False
    channel = next((c for c in (node or {}).get("channels", []) if binding and c["id"] == binding["channel"]), None)
    status = "fail" if not valid or not present else "unknown" if channel is None else "pass"
    check(f"port:{name}:io", name, status, "I/O assignment resolved" if status == "pass" else "I/O point, node, direction or assignment is unresolved",
          "Existing IoMap binding; no copied channel table", related=[name], io=ref, binding=binding)
    if channel is None:
        return
    direction_ok = channel["kind"] in (("di", "safe_di", "ai", "word") if expected_role == "input" else
                                       ("do", "safe_do", "ao", "word"))
    if not direction_ok:
        check(f"port:{name}:io:direction", name, "fail", "Assigned channel direction differs from the interface",
              "Existing channel kind must support the declared direction", related=[name])
    port["io_assignment"] = [ref["node"], binding["channel"]]
    if present:
        port["io_point"] = point
    kind = {"di": "digital", "do": "digital", "safe_di": "safe_digital", "safe_do": "safe_digital",
            "ai": "analog", "ao": "analog", "word": "word"}.get(channel["kind"])
    electrical = channel.get("electrical") or {}
    for key, value in {"signal_type": kind, "voltage_v": electrical.get("voltage"), "logic": electrical.get("logic")}.items():
        declared = _value(port, key)
        if declared is not None and value is not None and declared != value:
            check(f"port:{name}:io:{key}", name, "fail", f"Port {key} differs from its I/O channel",
                  "Update the interface declaration or the existing channel specification", related=[name], declared=declared, channel=value)
        if value is not None:
            port["resolved"][key] = {"value": value, "source": "io_channel", "attribute": key,
                                     "node": ref["node"], "channel": binding["channel"]}
