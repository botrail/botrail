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


## Catalog connection configurations

A manufacturer's support applies to a purchase SKU and a particular host
configuration. `compatibility.connections` records those conditions separately
from program listings (`compatibility.programs`), mechanical mounting and model
validation. UR+ listing alone does not establish connector or software support.

`bt.connections.configure(scene, target, profile, values=...)` records the
settings for an attached catalog kit. `target` is the kit path in
`bt.mounting.report(scene).kits`, such as `robot/tool`. Product and directly
attached host catalog IDs are pinned automatically. `remove_configuration`
clears the selection. Replacing the kit or host requires selecting its own
configuration; an old selection cannot qualify a different product.

```python
bt.connections.configure(scene, "robot/tool", "ur5e-es062-polyscope5", values={
    "wrist_connector": "m8_8pin_male",
    "controller": "ur_e_series",
    "voltage_v": 24,
    "protocol": "modbus_rtu_rs485",
    "software_family": "polyscope_5",
    # Supply capacities, pinout verification and installed releases are unknown.
})
print(bt.connections.report(scene).to_markdown())
scene.save_project("cell.botrail")
```

The report and Studio's mounting inspector show **configuration**, **electrical**,
**communication**, and **software** results separately. The stored declarations
and package evidence survive project save/load and generated Python. For offline
Python replay of every public host and tool, load the portable project and use
`scene.generate_python(embed_catalog=True)`. Earlier projects load with unknown
connection conditions.

| Input | Meaning |
|---|---|
| `wrist_connector`, `pinout` | Exact connector and documented pinout identifiers from the profile; a pinout identifier records the user's verification against that drawing |
| `voltage_v` | Configured host supply voltage in volts |
| `current_a`, `peak_current_a` | Available supply capacity in amperes; continuous and peak remain separate |
| `protocol`, `signal_logic` | Exact declared protocol and PNP/NPN mode |
| `controller`, `software_family` | Exact controller and software family identifiers |
| `software_version`, `plugin`, `plugin_version` | Installed release and plugin identifiers; versions match only explicitly recorded strings |

Unknown input or missing documentary conditions produce `unknown`, even when
the user supplies a plausible value. A different connector or software family
produces `fail`. A release absent from the documentary list remains `unknown`. A profile for another host is an invalid selection (`fail`);
absence of a manufacturer's claim remains `unknown`. Axes outside an explicitly
documented route can be `not_applicable`. These results compare declarations;
continue to use `port` and `connect` to check the physical wiring plan. Required
purchases stay in `order.requires`; included components stay in `order.includes`.

For a product that has no model yet, use
`bt.connections.evaluate(manifest_dict_or_package_path, exact_host_id,
profile="...", values={...})`. A catalog reference is also accepted and is
resolved before evaluation. The evaluator itself does not access the network.
The builder's `examples/connections/` contains separate Zimmer HRC-03-118505
(UR e-Series, NPN) and HRC-03-116787 (CRX, PNP) metadata examples. Their new
geometry and mounting models are not included in these examples.

### Local UR5e / ES-062 example

Build the new kit revision in the catalog-builder checkout:

```bash
bcb build recipes/robotiq/2f-85-ur-es-062-kit-r3.yaml --work-dir /tmp/urplus-catalog
```

In an environment with `botrail[catalog]`, copy the three existing packages into
the same local root. This preparation downloads the catalog packages once:

```python
from pathlib import Path
import shutil
import botrail as bt

root = Path("/tmp/urplus-catalog")
for product in ("universal_robots/ur/ur5e/r2",
                "robotiq/coupling/grp-es-cpl-062/r1",
                "robotiq/2f/2f-85/r2"):
    shutil.copytree(bt.catalog_package(product), root / product)
```

Then run in the Botrail checkout:

```bash
python examples/engineering/urplus_connections.py --catalog-root /tmp/urplus-catalog --studio
python examples/engineering/urplus_connections.py --catalog-root /tmp/urplus-catalog --connector m8_8pin_female --out /tmp/urplus-wrong-connector
```

The ES-062 example retains unresolved continuous current, exact PolyScope
release and installation data. Its electrical result becomes `fail` when the
female wrist is selected. It does not substitute the ES-077 purchase
configuration. The example uses a local r3 package and does not require r3 to
have been published to the catalog.

### Product configurations and tool loads

The catalog also includes Hand-E and 2F-85 kits with ES-077, the four
robot-specific Zimmer HRC-03 variants, and RG6 / VGC10 with a separately
purchased robot-side Quick Changer. CRX-20iA/L and M1509 have their own robot
packages. Try the complete configurations with:

```bash
python examples/engineering/urplus_products.py --configuration hand-e --studio
python examples/engineering/urplus_products.py --configuration zimmer-crx --format usd --out /tmp/hrc-crx
python examples/engineering/urplus_products.py --configuration vgc10 --out /tmp/vgc10
```

Available configurations are `hand-e`, `2f85-es077`, `zimmer-ur` (118506,
female wrist), `zimmer-ur-old` (118505, male wrist), `zimmer-crx` (116787),
`zimmer-doosan` (126895), `rg6`, and `vgc10`. Use `--revision` to pin a public
catalog commit, or `--catalog-root` to load staged packages. The output includes
a portable project, generated Python, BOM, mounting and connection reports,
and a tool-load report. Example settings leave unverified wiring and releases
unset.

Individually purchased tools appear in `bt.mounting.report(scene).products`.
A connection profile's `via` lists exact adapters in order from the host toward
the tool. Botrail compares this with the actual attachment path; missing,
additional or reordered adapters and changed mounting poses invalidate the
selected configuration. Studio shows these products and their required items
in the mounting inspector and assembly comparison.

```python
loads = bt.select.tool_loads(scene)
for load in loads:
    print(load["robot"], load["known_mass_kg"], load["total_mass_kg"])
    print(load["missing_mass"], load["unresolved_requirements"])
```

`known_mass_kg` sums documented masses of attached purchase units on that
robot. A kit counts once; its included parts are not additional purchases.
`total_mass_kg` remains `None` when a mass or required item is missing. Required
quantities need matching catalog IDs and part numbers when both are supplied;
the same attached item cannot satisfy two required quantities. This report
does not include workpieces or calculate center of gravity, inertia, or dynamic
payload suitability.

The authored tool shapes are reference geometry. Detailed fastening, cable
routing, and real gripper control require separate verification. VGC10 has no
actuated simulation joint and its four 30 mm cups do not inherit the rated load
of the manufacturer's three 40 mm cup configuration. Hand-E and RG6 report
opposing finger contact at full closure. The reused 2F-85 model reports internal
knuckle/pad interference at full closure; that endpoint is unsuitable as a
collision-free planning goal. The M1509's upstream collision meshes are not
watertight, so detailed collision geometry remains unverified.
