# Parts and the bill of materials

A cell is more than geometry and behavior: every conveyor, sensor, controller
and fence panel in it is *a product somebody buys*. botrail keeps that identity
on the residents themselves — a **part** is what a thing is commercially — and
**derives** the bill of materials from the scene, the way the
[I/O list](io-map.md) is derived from the sequences. You author what is in the
cell and where; the parts list falls out, and it can never disagree with the
scene it was read from.

```python
scene.set_part("belt", manufacturer="MISUMI", model="GVL-1200", mass_kg=45)
scene.set_part("eye", catalog="keyence/pz-g61n", model="PZ-G61N")
scene.set_part("fence", category="structure.fence", model="FP-2000", qty=12)

bom = scene.bom()
print(bom.to_markdown())
scene.export_bom("bom.csv")
```

## What a part is

[`Scene.set_part`][botrail.Scene.set_part] pins identity to a resident **by
name** — a robot, a device, a sensor, an I/O node, an obstacle, or an obstacle
*group* (everything under `name/`: an imported USD subtree, a generated
assembly). Every field is optional:

| field | meaning |
|---|---|
| `catalog` | a [catalog](robots.md#the-model-catalog) reference — `"id"`, `"id@revision"`, or `(id, revision)` |
| `manufacturer`, `model` | maker and model / part number |
| `category` | the BOM category (`conveyor`, `sensor.photoelectric`, `structure.fence`, ...); overrides the one the resident's kind implies |
| `description` | free text |
| `qty` | how many the target stands for — one by default; a fence group generated as twelve panels says 12 |
| any other keyword, or `attributes={...}` | free attributes: numbers (`mass_kg=45`, `power_w=200`, a price) are summed by `bom().total(key)`, text (`finish="RAL 7035"`) is carried along |

A part carries no geometry and no behavior. The shape stays on the obstacle,
the motion on the device, the wiring on the I/O map; the part is only the
name and the count. That is deliberate — botrail does not model shapes, and it
does not size anything: it counts and checks.

Names live in separate name spaces (a conveyor device and its belt slab may
both be `belt`), so when a name resolves to several things `set_part` asks for
`kind=` (`"robot"`, `"device"`, `"sensor"`, `"io_node"`, `"obstacle"`,
`"group"`). Re-pinning a target replaces its part; removing the resident
removes the pin.

## Generated structures come identified

The [standard parts](standard-parts.md) — `bt.parts.fence`, `table`,
`pedestal`, `conveyor`, `pallet`, `light_curtain`, `photoelectric` — pin their parts as they
build: a fence is one line for the panels (its quantity the panel count),
one for the posts, one for the door; a conveyor's identity sits on the
device, its body being geometry. Give them `model=` / `manufacturer=` /
attributes and the BOM line is complete the moment the thing exists.

## Catalog robots need no authoring

A robot loaded with [`Robot.from_catalog`][botrail.Robot.from_catalog] already
knows what it is: the package's manifest names the maker, the product, the
category and the headline specs, and botrail keeps them on the model's
provenance record — so a catalog arm is an identified BOM line with its
`id@revision`, `payload_kg`, `reach_mm` and `mass_kg` the moment it enters
the scene, and a tool welded on with `attach_tool` is its own line. A part
pinned to the robot's instance name overlays that (a description, a price,
a different category); it does not replace it.

## Tools in the stack

A tool welded onto the robot with `attach_tool` is a line of its own,
named by its place in the stack — `arm/tool` for the first, `arm/tool2`
for the next, `arm/tool/tool3` for one riding a tool that is itself a
stack. A catalog tool brings its identity; one made from a URDF string
(a `bt.tools.multi_tool` bracket, say) does not, and gets it pinned by
that row name:

```python
scene.set_part("arm/tool", kind="tool", category="tool.multi",
               catalog="botrail/hand/mph3/r1", manufacturer="botrail", model="MPH-3", mass_kg=0.3)
```

The pin follows the robot through a rename and rides the project; on a
catalog tool's row it is the last word.

## The bill of materials

[`Scene.bom`][botrail.Scene.bom] lists every piece of **equipment** the scene
holds — robots and their tools, conveyors, axes and vehicles, sensors,
controller boxes — whether or not it has been identified, plus every obstacle
or group a part was pinned to. Bare obstacles are geometry (scenery, stock, a
workpiece pool) and are not parts unless you say so; a `Source`/`Sink` pair
models an endless line and is not equipment either.

Identical products merge into one row with the quantity summed and the
resident names kept for traceability; a row that names nothing yet — no
catalog reference, maker or model — stays under its own resident name, so
`bom.unidentified()` is the purchasing to-do list.

```text
| # | category             | manufacturer | model     | catalog              | qty | names            | mass_kg |
|---|----------------------|--------------|-----------|----------------------|-----|------------------|---------|
| 1 | manipulator          | FANUC        | M-20iD/25 | fanuc/m/m-20id-25@…  | 1   | arm              | 250     |
| 2 | conveyor             | MISUMI       | GVL-1200  |                      | 1   | belt             | 45      |
| 3 | sensor.photoelectric | KEYENCE      | PZ-G61N   | keyence/pz-g61n      | 1   | eye              |         |
| 4 | plc                  |              | R04CPU    |                      | 1   | PLC1             |         |
| 5 | part                 |              | HFS8-1200 |                      | 2   | table_a; table_b | 30      |
| 6 | structure.fence      |              | FP-2000   |                      | 12  | fence            | 8       |

Totals: mass_kg = 451
```

The [`Bom`][botrail.Bom] renders as CSV, Markdown or JSON (`to_csv()`,
`to_markdown()`, `to_json()`, `save(path)`; `scene.export_bom(path)` picks the
format from the extension), and its numbers are pytest material like
everything else botrail bakes:

```python
bom = build_cell().bom()
assert bom.unidentified() == []                # everything has a number
assert bom.total("mass_kg") <= 1500            # None, not 0, when nobody says
assert bom.total("power_w") <= 20_000
```

The generated Python and the `.botrail` project carry the pins, so a
reloaded cell — or the script `generate_python()` writes — produces the same
table.

## What it is not

The BOM is a *derived table*, not a purchasing system: botrail holds the
slot for a price and sums it if you fill it in, but it does not know prices,
lead times or stock, and it does not validate attributes. Nothing here sizes
a frame, a supply or a valve — a part's `mass_kg` is a number you wrote down.

## What the cell asks of each line

A BOM line also carries *requirements* — the payload, reach, stroke, span,
load the cell implies for it — and the check compares them with the part's
attributes. See [Selecting parts](selection.md): `scene.requirements()`,
`scene.check()` and `bt.catalog.search`.

[Physical interface ports](connections.md) reference these same equipment
targets. Supply capacity is compared with connected loads, using exact Part
attributes where a single endpoint's rating can be resolved. Ports and cable
references do not create additional BOM lines.
