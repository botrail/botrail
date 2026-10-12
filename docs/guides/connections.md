# Equipment connections

The [I/O map](io-map.md) is the signal side of a cell: which contact lands
on which channel. The utilities are the other side — *which* supply feeds
*which* load, which valve feeds which actuator, which controller input
reads which sensor — and most cells never have to say it: `scene.requirements()`
already sums the `current_a` of the parts at each supply's voltage. When a
cell does have to say it (two 24 V supplies, a drive on its own 48 V unit,
an air main shared by several grippers), the **connection plan** is where:
ports declared on existing equipment, connections between them, and a
report that checks the pairing.

```python
import botrail as bt

cx = bt.connections

cx.port(scene, "24V", "psu", "power", "supply")                 # capacity from the part: output_v / output_a
cx.port(scene, "eye.power", "eye", "power", "load")             # demand from the part: voltage_v / current_a
cx.port(scene, "air", "valve", "pneumatic", "supply", pressure_bar=6, capacity_l_min=100, flow_reference="ANR")
cx.port(scene, "grip.air", "grip", "pneumatic", "load", pressure_min_bar=4, pressure_max_bar=7,
        flow_l_min=20, flow_reference="ANR")

cx.connect(scene, "24V", "eye.power", name="W1", cable="CBL-001", reference="E-001")
cx.connect(scene, "air", "grip.air", name="P1")

report = cx.report(scene)
assert report.ready, report.to_markdown()
report.save("connections.md")
```

Ports describe declared interfaces; they do not run the cell. Nothing here
changes a bake, a derived I/O point or a BOM line.

## Ports

[`port`][botrail.connections.port] defines (or replaces) a named interface on
a resident that already exists — a robot, device, sensor, I/O node, obstacle
or group, found by name like [`set_part`](parts-and-bom.md) finds its target
(`target_kind=` disambiguates). A port has a **medium** and a **role**:

| medium | roles | specs |
|---|---|---|
| `power` | `supply` / `load` | `voltage_v` (or `voltage_min_v` / `voltage_max_v`), `capacity_a` on a supply, `current_a` on a load |
| `pneumatic` | `supply` / `load` | `pressure_bar` (or min / max), `capacity_l_min` on a supply, `flow_l_min` on a load, `flow_reference` (the reference conditions both numbers are stated at) |
| `signal` | `output` / `input` | `signal_type` (`digital`, `safe_digital`, `analog`, `word`), a voltage range, `logic` (`pnp` / `npn`) |
| `network` | `peer` | `protocol` |

Values are the **total at that endpoint** — a load's whole steady demand, a
supply's whole rating. Nothing is converted (no W to A, no diversity
factor), and a value nobody states stays unknown. A port with a single
counterpart on a `qty=1` resident inherits what its [part](parts-and-bom.md)
already says — `voltage_v` and `current_a` on a load, `output_v` and
`output_a` on a power supply — so the two lines in the example above carry
no numbers of their own. Every resolved value records where it came from.

Supply ports are optional by default; a load, input or peer is `required`
and reports `unknown` until something connects to it. `terminal=` and
`reference=` carry drawing labels and never mean a channel address. A
signal port on an I/O node may point at an existing assignment of the I/O
map — `io={"point": "eye", "direction": "input", "node": "PLC1"}` — and the
report then checks that the point still derives, that it is bound, and that
the channel's kind and electrical data agree with the port.

## Connections

[`connect`][botrail.connections.connect] joins two declared ports. The name
defaults to `"source -> target"`; reusing a name replaces that link.
Incompatible pairs (two supplies, a signal into a power port) are **kept and
reported**, like a wrong I/O assignment, rather than rejected on the spot —
the report is where the cell is reviewed. `cable=` is a reference to the
cable this connection runs on (a part number, an ECAD id); it does not add a
BOM line, and no length is computed from it.

[`disconnect`][botrail.connections.disconnect] removes a link;
[`remove_port`][botrail.connections.remove_port] removes a port but keeps
the links that named it, so a dangling reference shows up in the report
instead of vanishing. The whole plan rides the `.botrail`
[project](projects.md) and comes back from `generate_python` as a
`bt.connections.restore` call.

## The report

[`report`][botrail.connections.report] checks the whole cell, independent of
what was baked, and returns a [`ConnectionReport`][botrail.connections.ConnectionReport]
with four tables — the ports with their resolved specs, the connections, the
supplies with their known subtotals, and the checks — rendered as Markdown,
JSON or CSV (`to_markdown()`, `to_json()`, `to_csv(table="connections" |
"power")`, `save(path)`). Each check has an `id`, a `target`, a `status`
and the `basis` it was decided on:

| status | meaning |
|---|---|
| `pass` | the declared values agree |
| `fail` | they contradict: a supply range outside what the load accepts, two supplies into one load, a missing port or resident, known demand over a supply's capacity, a part whose I/O channel disagrees with its port |
| `unknown` | something needed is not declared — a load without `current_a`, a required port nobody connected, a part that declares `output_a` or `current_a` but has no port, a controller uplink with no network ports |
| `not_run` / `not_applicable` | no requirements declared at all / an optional port left unconnected |

`ready` is true when nothing failed or stayed unknown. It means the declared
requirements are resolved; it is not an installation approval. What the
report does **not** evaluate is stated on every table it prints: electrical
protection, cable sizing, pressure drop and dynamic behaviour are outside it,
and the connection plan is its own report — it is not part of
`scene.check()`.

## From the plan to the wiring

A port says what an interface is; give it a place (`at=` the resident and
offset it sits on, `exit=` the direction the line leaves it) and a
[route](routing.md) can run between two ports — the physical line, its
spec, its way through ducts and clamps — with its length, BOM row and
checks derived from where the equipment stands. A connection whose ports
both have a place and that no route carries is the wiring to-do
(`route_unrouted`). The plan stays the connection table; the route is the
run of one of its lines.

A port's place can come from the product itself: a catalog pack that
declares `ports:` (where each line plugs in, which way it leaves, how much
stays straight) answers `bt.connections.ports_of(scene, target)`, and
`port(scene, name, target, from_catalog="robot_cable")` takes that place,
exit and straight length — and the port's medium and role — as defaults.
