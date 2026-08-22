# Standard parts and CAD geometry

Every cell has scenery nobody wants to model: the fence, the tables, the
pedestal under the robot, the racks, the conveyor's body, the pallets. `bt.parts`
generates them from parameters — as ordinary residents (boxes under a name
prefix, a frame where the next thing mounts, a device or a sensor where one
belongs) with their [part identity](parts-and-bom.md) already pinned, so the
BOM counts them and the [layout sheet](layout-and-report.md) labels each
assembly once. Change a parameter and the geometry, the BOM line and the
drawing change together.

```python
fence = bt.parts.fence(scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)],
                       height=2.0, panel_pitch=1.0, door=(0, 2),
                       model="ST20", manufacturer="TROAX", mass_kg=12)
ped = bt.parts.pedestal(scene, "pedestal", height=0.5, position=(0, 0), model="PD-500")
scene.set_robot_base_pose(*scene.frame("pedestal/mount"))
conv = bt.parts.conveyor(scene, "conv", length=2.0, width=0.4, position=(0, 1.2, 0.7),
                         direction=(1, 0), speed=0.2, model="GVL-2000")
bt.parts.table(scene, "table", size=(1.2, 0.8, 0.75), position=(1.0, 0.0), model="HFS8-1200")
bt.parts.pallet(scene, "pallet", position=(-1.2, 0.0))
rack = bt.parts.rack(scene, "rack", size=(1.2, 0.6, 1.8), position=(-1.2, 1.2), levels=4)
bt.parts.light_curtain(scene, "lc", frm=(-1, -2), to=(1, -2), model="SL-V")
```

## What each generator makes

| generator | obstacles | frame | device / sensor | parts pinned |
|---|---|---|---|---|
| [`fence`][botrail.parts.fence] | panels under `<name>/panels/`, posts under `<name>/posts/`, the door as `<name>/door` | — | — | `<name>` (`structure.fence`, qty = panels), `<name>/posts` (`structure.fence.post`, qty = posts), the door (`structure.door`) |
| [`table`][botrail.parts.table] | `<name>/top`, four legs | `<name>/top` (centre of the top face) | — | `<name>` (`structure.table`) |
| [`pedestal`][botrail.parts.pedestal] | `<name>/base`, `<name>/column`, `<name>/top` | `<name>/mount` (the robot's base pose) | — | `<name>` (`structure.pedestal`) |
| [`conveyor`][botrail.parts.conveyor] | `<name>/belt`, side rails, legs | `<name>/infeed`, `<name>/outfeed` | the conveyor device `<name>`, its zone on the belt | the *device* (`conveyor`) — the body is its geometry, not a second product |
| [`rack`][botrail.parts.rack] | four uprights under `<name>/uprights/`, a board per level under `<name>/shelves/` | `<name>/level0` … upwards (the centre of each deck) | — | `<name>` (`structure.rack`), and with a catalog `<name>/shelves` (`structure.rack.shelf`, qty = levels) |
| [`pallet`][botrail.parts.pallet] | bottom boards, blocks, deck boards | `<name>/top` | — | `<name>` (`pallet`, `EPAL 1` by default) |
| [`light_curtain`][botrail.parts.light_curtain] | two columns | — | the beam sensor `<name>` (trips on the robot) | the *sensor* (`sensor.light_curtain`) |

Every generator takes `model=`, `manufacturer=` and free attributes
(`mass_kg=…`) for the part it pins — or `catalog=`, the id of a spec pack, and
then the dimensions, part numbers and mass come from the catalog and the
generator refuses a size nobody sells (see
[the model catalog](robots.md#the-model-catalog)). Each returns a
[`Built`][botrail.parts.Built] naming what it made — `built.frames`,
`built.devices`, `built.obstacles` — with `built.remove(scene)` to take the
whole thing down again.

## Drawn, and what it hits

A catalog part is drawn the way it looks: a mesh panel as a tube frame with a
grid of wire in it, a conveyor with its rollers and drive, a rack with its
beams and braces. All of that is **decoration** — added with collision off,
under `<name>/trim/` — while the massing underneath (the panel slab, the belt,
the uprights) keeps collision and stops being drawn where the detail stands in
for it. Changing `detail` therefore never changes what a robot can hit, what
the BOM says, or what a plan gives you; it costs scene entries and nothing
else. `detail="plain"` is the bare massing, and is what a generator called
without a catalog does — there are no real sections to draw from.
[`examples/equipment_cell_demo.py`](https://github.com/botrail/botrail/blob/main/examples/equipment_cell_demo.py)
builds a cell whose fence, conveyor and rack all come from the catalog, and
prints the bill it can be ordered from.

Where the drawing comes from is the product's business, not the generator's: a
pack can name a file of primitives per part (`components[].trim`, a URDF or
xacro), and the generator expands it to the size at hand with
[`load_urdf`][botrail._core.Scene.load_urdf] instead of drawing its own
shapes. One parametric file covers every size a product is sold in, so making
a fence look like *that* maker's fence is an edit to the catalog, not to
botrail. Parts the pack says nothing about keep the built-in look.

A fence's edge is split into panels of *about* `panel_pitch` (stretched so an
edge takes a whole number of them) with a post at every corner and between
panels; `door=(edge, panel)` makes that panel the door. Because the counts
come from the geometry, `panel_pitch=0.5` on the same path is twice the
panels on the BOM and twice the panels on the sheet — the same edit, seen
by every document.

Names live under the generator's `name`, so a device and its body share
one name (`conv` the device, `conv/belt` the slab): `scene.set_part("conv",
…)` then needs `kind="device"` to say which — the generators do this for
you.

## Bringing shapes from CAD — the Geometry Provider pattern

botrail does not model shapes, and will not: no sketches, no features, no
CAD kernel. A fence is *panels of a pitch along a path* and a table is *a
top on legs* — the meaning is what a cell verifies and what its documents
need, and the few centimetres a real profile differs by change nothing the
verifier measures.

Anything with a shape of its own comes in from the tool that owns it. The
pattern is always the same three steps:

1. **Generate or export the shape elsewhere** — CadQuery, FreeCAD, Blender,
   the vendor's CAD — as a mesh (OBJ / STL) or a USD stage.
2. **Load it as geometry** — `scene.add_mesh(name, path, position=…)`
   for one body, `scene.load_usd(stage, prefix=…)` for an assembly (its
   prims become obstacles under the prefix, its Xforms named frames).
3. **Pin what it is** — `scene.set_part(name, manufacturer=…, model=…,
   catalog=…)`, on the mesh or on the group the stage came in as.

```python
# CadQuery makes the bracket; botrail places it and knows what it is.
import cadquery as cq
bracket = cq.Workplane("XY").box(0.12, 0.08, 0.01).faces(">Z").hole(0.008)
cq.exporters.export(bracket, "bracket.obj")

scene.add_mesh("bracket", "bracket.obj", position=(0.4, 0.2, 0.75))
scene.set_part("bracket", model="BR-120", manufacturer="ACME", mass_kg=0.35)
```

The mesh collides (a cached convex decomposition), draws in the studio with
its `mtllib` colours, projects onto the layout sheet as its bounding
rectangle, and lands on the BOM as one identified line — exactly like a
generated part. What botrail keeps is the *meaning* (this is a bracket,
model BR-120, one of them, 0.35 kg, standing there); the shape stays the
provider's business, and can be regenerated there without the cell
noticing anything but the geometry.

The [model catalog](robots.md#the-model-catalog) is the same pattern with
the identity already attached: `Robot.from_catalog` (and
`catalog_package()` for non-robot packages) brings the maker's mesh *and*
its manifest, so the BOM line writes itself.
