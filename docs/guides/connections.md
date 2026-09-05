# Equipment connections and supply capacity

Declare the interfaces each piece of equipment needs, connect them, and
check the resulting supply loads and compatibility. Ports refer to existing
Scene equipment or I/O nodes. They survive project save/load and generated
Python, and do not add BOM rows or change the operating sequence.

```python
import botrail as bt

scene = bt.Scene()
bt.parts.power_supply(scene, "PS24", (0, 0, 0), size=(.1, .1, .2),
                      model="Example supply", output_v=24, output_a=2)
scene.add_beam_sensor("eye", frm=(.2, 0, .1), to=(.6, 0, .1))
scene.set_part("eye", model="Example sensor", voltage_v=24, current_a=.1)

bt.connections.port(scene, "PS24.out", "PS24", "power", "supply",
                    terminal="X1:+/-", reference="E-001")
bt.connections.port(scene, "eye.power", "eye", "power", "load")
bt.connections.connect(scene, "PS24.out", "eye.power", cable="W-01")

result = bt.connections.report(scene)
print(result.to_markdown())
result.save("connections.csv")
result.save("power.csv", table="power")
```

This illustrative supply has **0.1 A of connected demand against 2 A of
capacity**. Another supply receives only its own connected loads. An
unconnected component never enters either subtotal.

## Declaring interfaces

`port(scene, name, target, medium, role, ...)` defines a named endpoint.
Reusing a name replaces its declaration. `target_kind=` disambiguates
resident names, using the same kinds as `set_part`. Equipment may have
several endpoints: for example, separate control and drive power inputs.

| medium | roles | specifications |
|---|---|---|
| `power` | `supply` → `load` | `voltage_v` or `voltage_min_v` / `voltage_max_v`; supply `capacity_a`, load `current_a` |
| `pneumatic` | `supply` → `load` | `pressure_bar` or `pressure_min_bar` / `pressure_max_bar`; supply `capacity_l_min`, load `flow_l_min`; both `flow_reference` |
| `signal` | `output` → `input` | `signal_type`: `digital`, `safe_digital`, `analog`, `word`; voltage as above; digital `logic`: `pnp`, `npn` |
| `network` | `peer` ↔ `peer` | `protocol`, compared without case sensitivity |

Consumer, signal and network ports require a connection by default. Supply
ports are optional by default. Set `required=False` for a spare interface.
Supply/output fan-out is supported; multiple connections terminating at a
load, input or network socket fail. Model separate sockets as separate ports.

`terminal`, `cable` and `reference` are drawing/specification references.
They do not create terminals or cables as equipment. Use `disconnect(scene,
name)` to remove a connection. Removing a port with `remove_port`, or deleting
equipment, retains dangling connections so the report can name the missing
references. `restore(scene, plan)` accepts the project's typed
`connection_plan` object, including unresolved references.

## Where specifications come from

An explicit port value takes precedence. Otherwise a port can read the exact
Part pinned to its target when the target has `qty=1` and only one port of
that medium and role. Power supplies read `output_v` / `output_a` as
`voltage_v` / `capacity_a`; other attributes use the names above. Editing
the Part changes the next report. A merged BOM row is never used to infer
an individual endpoint's consumption.

For quantities greater than one or multiple ports of the same medium/role,
declare values on each port. These are **endpoint totals**, without quantity
multiplication. Several supply ports on one equipment target leave a shared
capacity question `unknown`; the current model cannot establish whether
their ratings are independent. A common source port with fan-out permits
checking its combined load.

The JSON report records each resolved value and its origin: `port`, `part`,
`io_channel` or `unknown`. No catalog download or current-product lookup
occurs during checking. Non-numeric, negative and non-finite Part values
remain unknown. Invalid port field names, units-as-strings and numeric values
are rejected when authored.

## Reusing an I/O assignment

Declare the field interface and reference the existing assignment from the
controller port. Reassigning `DI0` to `DI1` updates the next connection report.

```python
scene.add_io_node("PLC", channels=bt.io.di16(voltage=24, logic="pnp"))
scene.declare_io("eye", role="input", kind="di")
scene.bind_input("eye", "PLC", "DI0")

bt.connections.port(scene, "eye.output", "eye", "signal", "output",
                    signal_type="digital", voltage_v=24, logic="pnp")
bt.connections.port(scene, "PLC.eye", "PLC", "signal", "input",
                    io={"point": "eye", "direction": "input", "node": "PLC"})
bt.connections.connect(scene, "eye.output", "PLC.eye")
```

Channel kind, voltage and logic remain authoritative. Conflicting port
values fail. Missing assignments remain unresolved; two declared ports
cannot alias the same physical channel. Where an existing I/O point names
a sensor/device, connecting a different sensor/device fails. An existing
I/O uplink also needs declared network endpoints: its bus label alone does
not specify both devices' protocol capabilities.

## Reading the result

The report includes one requirements row per port, connection rows, supply
capacity rows and individual checks. `ready` requires the declared checks
to be resolved. Empty declarations are `not_run`. Part attributes revealing
power/air consumption or power supply capacity also expose missing interface
declarations. Other omitted interfaces cannot be inferred automatically.

| condition | result |
|---|---|
| Missing equipment/port, wrong medium/direction, multiple feeds or incompatible known specifications | `fail` |
| Required port unconnected, missing specs or unknown connected consumption | `unknown` |
| Optional unconnected port | `not_applicable` |
| Complete compatible declared interfaces and sufficient capacity | `pass` |

The entire supply voltage/pressure range must fit within the accepted load
range. A nominal-only value means that exact declared value; no tolerance
is invented. A partly specified range stays unknown. Word signals do not
require electrical voltage checks.

Power budgets sum only directly connected loads' steady `current_a`.
`known_subtotal`, `known_loads`, `total_loads` and `missing_loads` keep partial
information explicit. All loads unknown yields a null subtotal; explicit
zero remains known. A known subtotal exceeding capacity fails even if other
loads are unknown. Air flow contributes only when both endpoints state the
same `flow_reference` conditions; no conversion is performed.

These checks cover declared steady interface requirements. Protection,
cable sizing, AC/DC and phase compatibility, transient/inrush behaviour,
demand factors, pneumatic dynamics, analog transfer ranges, network timing
and safety performance require separate evaluations.

## Review and deliverables

`scene.check()` includes physical failures as errors and unknowns as warnings.
[`bt.review(scene, stage="design")`](design-review.md) also treats unresolved
physical connections as review blockers. Power supply specification checks
use connected budgets; the previous whole-BOM current requirement is removed.

```bash
botrail connections examples/engineering/cell_connections_demo.py \
  --report connections.md --csv connections.csv --power power.csv
botrail export examples/engineering/cell_connections_demo.py \
  --out deliverables/connections-r1 --project --python --connections --report
```

The example includes separate 24 V / 48 V supplies, a sensor assignment,
air service and a network uplink. Its ratings are illustrative. Calling
`build(unknown_valve_current=True)` leaves one load unknown: the 24 V subtotal
is 0.1 A with one missing load; the 48 V budget stays complete at 2 A.

The CLI exits 0 for resolved declared requirements, 1 for unresolved findings,
and 2 for invalid input/output arguments. `--markdown` prints Markdown;
otherwise stdout is JSON. No simulation bake is needed.

Batch export's `--connections` (included in `--all`) writes
`<name>_connections.csv`, `.md`, `.json` and `<name>_power.csv`. The main batch
report also includes the connection results. These use the same snapshot and
[manifest revision](layout-and-report.md#the-document-set-as-one-thing) as the
other files; changing a connection invalidates verification against the old
package. Physical requirements always cover the whole cell, even when a
subset of operating programs is selected. Direct `scene.cell_report()`
continues to describe simulation results; use the batch export or
`bt.connections.report()` for the physical connection tables.
