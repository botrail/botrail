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
bt.parts.photoelectric(scene, "eye", frm=(0.0, 1.0, 0.75), to=(0.0, 1.4, 0.75),
                       watch=["part"], model="E3Z-D62")
```

## What each generator makes

| generator | obstacles | frame | device / sensor | parts pinned |
|---|---|---|---|---|
| [`fence`][botrail.parts.fence] | panels under `<name>/panels/`, posts under `<name>/posts/`, the door as `<name>/door` | — | — | `<name>` (`structure.fence`, qty = panels), `<name>/posts` (`structure.fence.post`, qty = posts), the door (`structure.door`) |
| [`table`][botrail.parts.table] | `<name>/top`, four legs | `<name>/top` (centre of the top face) | — | `<name>` (`structure.table`), and with a catalog `<name>/top` (the board, where the maker sells it as its own article) |
| [`pedestal`][botrail.parts.pedestal] | `<name>/base`, `<name>/column`, `<name>/top` | `<name>/mount` (the robot's base pose) | — | `<name>` (`structure.pedestal`) |
| [`conveyor`][botrail.parts.conveyor] | `<name>/belt`, side rails, legs | `<name>/infeed`, `<name>/outfeed` | the conveyor device `<name>`, its zone on the belt | the *device* (`conveyor`) — the body is its geometry, not a second product |
| [`rack`][botrail.parts.rack] | four uprights under `<name>/uprights/`, a board per level under `<name>/shelves/` | `<name>/level0` … upwards (the centre of each deck) | — | `<name>` (`structure.rack`), and with a catalog `<name>/shelves` (`structure.rack.shelf`, qty = levels) |
| [`cabinet`][botrail.parts.cabinet] | `<name>/body`, its plinth as `<name>/base`, the mounting plate standing inside as `<name>/plate` | `<name>/front` (centre of the door face at floor level — where an operator stands) | — | `<name>` (`structure.cabinet`), and with a catalog `<name>/base` and `<name>/plate` (`structure.cabinet.base` / `.plate` — the plinth and the plate are articles of their own) |
| [`controller`][botrail.parts.controller] | the box `<name>/body` | `<name>/front` (centre of the door face at the box's level — where somebody stands to service it) | the `robot_controller` I/O node `<name>` driving `robots=`, its `place` the box (a node the cell wired first is adopted, wiring kept) | the *node* (`robot_controller`, carrying `mount` and the robot cable length `cable_m`) — one line for the box, the node and the [controller every arm needs](parts-and-bom.md#the-bill-of-materials); with a catalog the enclosure variant's part number, mass and the maker's service clearance |
| [`pallet`][botrail.parts.pallet] | bottom boards, blocks, deck boards | `<name>/top` | — | `<name>` (`pallet`, `EPAL 1` by default) |
| [`light_curtain`][botrail.parts.light_curtain] | two columns, `<name>/column_a|b` | — | the beam sensor `<name>`, spanning the gap between the lens faces (trips on anything in the field) | the *sensor* (`sensor.light_curtain`) — with a catalog the emitter/receiver pair's model number, and the range of the resolution chosen |
| [`photoelectric`][botrail.parts.photoelectric] | the sensor body `<name>/body` behind its lens; a through-beam pair adds `<name>/receiver`, a retroreflective one `<name>/reflector` | — | the beam sensor `<name>` (trips on `watch`, and on the robot if asked) | the *sensor* (`sensor.photoelectric`), and with a catalog the reflector where the maker sells it separately |
| [`proximity`][botrail.parts.proximity] | the threaded barrel `<name>/body` behind the sensing face | — | the beam sensor `<name>`, as long as the switch's sensing range (a few millimetres) | the *sensor* (`sensor.proximity`) — with a catalog the model of the size, shield and output chosen, and its range |
| [`power_supply`][botrail.parts.power_supply] | the box `<name>/body` on its rail | — | — | `<name>` (`power_supply`, carrying `output_v` / `output_a` — what the `current_a` of the parts at its voltage is checked against) |
| [`remote_io`][botrail.parts.remote_io] | the coupler `<name>/coupler`, a box per terminal unit `<name>/di{i}` / `<name>/do{i}` | — | the I/O node `<name>` (`remote_io`, a channel per point, hung off `uplink=`) | the coupler (`io.remote`, with `di` / `do` counts), and every unit as a line of its own |
| [`wall`][botrail.parts.wall] | a pier per solid stretch under `<name>/e{edge}_{i}`, the wall over each opening under `<name>/head/`, a column at each shared corner | `<name>/opening{edge}_{i}` (on the floor at each doorway, facing along the wall) | — | `<name>` (`structure.wall`, carrying the run's length, height and thickness) |
| [`machine_tool`][botrail.parts.machine_tool] | the enclosure under `<name>/shell/`, `<name>/bed`, `<name>/column`, `<name>/saddle`, `<name>/table`, `<name>/head`, the door leaves under `<name>/side_door/` and `<name>/front_door/`, the panel's plate | `<name>/table`, `<name>/entry`, `<name>/door/side/handle`, the panel's | the side door as a linear axis `<name>/side_door` (servo / air) with the stops `closed` / `open` as lanes, a zone per button | `<name>` (`machine_tool.vmc`), `<name>/side_door` (`machine_tool.door`, drive and stroke), the panel and its buttons — with a catalog the pack's envelope, options, door times and interface |
| [`operator_panel`][botrail.parts.operator_panel] | `<name>/plate`, a cap per button | `<name>`, `<name>/<button>`, `<name>/<button>/press` (+Z into the panel) | a zone sensor `<name>/<button>` inside each cap, as deep as the stroke | `<name>` (`hmi.panel`), each button (`hmi.button`, head, travel, force) — with a catalog the box by its positions, the buttons and the E-stop by article |
| [`vise`][botrail.parts.vise] | `<name>/body`, `<name>/jaw_fixed`, `<name>/jaw_moving` | `<name>/jaw` (the jaw floor between the jaws) | — | `<name>` (`fixture.vise`, jaw width and opening) — with a catalog the jaw width matched against the ones sold, the maximum opening from the pack |
| [`lathe`][botrail.parts.lathe] | the enclosure under `<name>/shell/`, `<name>/bed`, `<name>/rear`, `<name>/headstock`, `<name>/turret` (and `<name>/tailstock`), the front door leaf under `<name>/front_door/`, the panel's plate | `<name>/spindle` (the nose, +Z along the axis toward the tailstock), `<name>/entry`, `<name>/door/front/handle`, the panel's | the front door as a linear axis `<name>/front_door` (servo / air) with the stops `closed` / `open`, or a loose leaf with two limit switches; a zone per button | `<name>` (`machine_tool.lathe`), `<name>/front_door` (`machine_tool.door`, drive and stroke), the panel and its buttons |
| [`chuck`][botrail.parts.chuck] | `<name>/body`, `<name>/jaw0` … (proud of the face around the gripping diameter) | `<name>/face` (the face centre, +Z out along the spindle axis — a load comes in along -Z) | — | `<name>` (`fixture.chuck`, diameter, opening and jaw count) — with a catalog the diameter matched against the ones sold, the maximum opening from the pack |
| [`pallet_stand`][botrail.parts.pallet_stand] | two sections either side of the vehicle's path: `<name>/rail_l|r`, posts `<name>/post_{l|r}{f|b}`, the far-end stops `<name>/stop_l|r` | `<name>/pallet` (the seat, `support` over the floor), `<name>/entry` (on the floor at the open end, +X into the stand — a docking heading) | — | `<name>` (`structure.pallet_stand`, support and inner width; with a catalog the maker's article and the pallet it takes) |
| [`charging_station`][botrail.parts.charging_station] | `<name>/housing`, the charging plate on the floor `<name>/plate` | `<name>/dock` (on the floor at the plate's edge, +X away from the housing — where the vehicle's charging face stands) | — | `<name>` (`vehicle.charger`; with a catalog the article, mass and the current at the supply chosen) |
| [`pallet_rack`][botrail.parts.pallet_rack] | posts and beams under `<name>/unit/` (the first bay) and `<name>/ext/{i}/` (each further bay) | `<name>/bay{i}/level{j}` (the centre of every pallet seat: j = 0 the floor, then each beam top) | — | `<name>` (`structure.rack`); with a catalog `<name>/unit` (the 単体 article) and `<name>/ext` (the 連結 extensions, qty = bays − 1) |
| [`stairs`][botrail.parts.stairs] | a walkable checker-plate tread per step under `<name>/tread…`, a plate stringer and support leg each side, the handrail under `<name>/handrails/` | `<name>/foot`, `<name>/top` (author the vehicle path's z between them) | — | `<name>` (`structure.stairs`), and with a catalog `<name>/handrails` (`structure.stairs.rail`, qty = 2 sides) |
| [`tray`][botrail.parts.tray] | the tray `<name>` (collides), its foam insert `<name>/insert` (a picture), and in full detail a bent grip each side under `<name>/trim/` | `<name>/seat` (the insert's top centre — where a part sets down) | — | `<name>` (`tray`) |
| [`stage`][botrail.parts.stage] | the block `<name>` (collides; hidden in full detail behind the plate and legs under `<name>/trim/`), `<name>/insert` | `<name>/seat` | — | `<name>` (`fixture`) |
| [`carton`][botrail.parts.carton] | one box `<name>`, drawn as the library's carton | — | — | `<name>` (`workpiece`, the RSC size as its model, `mass_kg` when given) |
| [`unit_load`][botrail.parts.unit_load] | the envelopes `<name>/pallet` and `<name>/load` (collide, hidden in full detail), the timber, cartons, film and labels under `<name>/visual/` | — | — | nothing: stock is not a purchase |
| [`marking`][botrail.parts.marking] | paint out of collision — `<name>/0`…`<name>/3` round a rect, `<name>` for a line, `<name>/0`… for its dashes | — | — | nothing (the layout sheet draws them on its ground layer) |
| [`person`][botrail.parts.person] | one box `<name>` (collides) | — | — | nothing |
| [`gantry`][botrail.parts.gantry] | `<name>/post_l`, `<name>/post_r`, `<name>/beam` (collide) | `<name>/beam` (the beam's centre underside — a camera's mount) | — | `<name>` (`structure.gantry`) |

The last seven are *props*: the generic things a cell is full of and nobody
orders by part number — the tray a part waits in, the stage under a camera,
the carton, the stock on a pallet, the paint on the floor, the person a
scenario stands in the gate, the portal a camera hangs from. They are
generated from their dimensions like everything else, drawn from the
[shape library](#shapes-a-box-cannot-draw-the-shape-library) where a box
cannot draw the thing, and `detail="plain"` keeps just the massing.

Every generator takes `model=`, `manufacturer=` and free attributes
(`mass_kg=…`) for the part it pins — or `catalog=`, the id of a spec pack, and
then the dimensions, part numbers and mass come from the catalog and the
generator refuses a size nobody sells (see
[the model catalog](robots.md#the-model-catalog)). Each returns a
[`Built`][botrail.parts.Built] naming what it made — `built.frames`,
`built.devices`, `built.obstacles` — with `built.remove(scene)` to take the
whole thing down again.

## Drawn, and what it hits

The machining centre, vise, operator panel, table and pedestal assign simple
surface finishes by role: painted covers, exposed metal, plastic buttons and
rubber feet. Conveyor belts and rails also have separate finishes. These are
authored visual defaults, not measured manufacturer data or friction values.
Colours and geometry stay independent of the finish. Override a surface with
`scene.set_obstacle_material(name, metalness=..., roughness=...)`; overrides
survive project save/load and generated Python. These defaults do not override
imported catalog trim; its appearance follows the existing
[import path](scene-and-obstacles.md). In `machine_tool(detail="full")`, the
doors have framed transparent panes; the hidden full leaves remain the
collision and switch-sensing envelopes, and all moving trim belongs to
`door_objects`. The skirt, the accent band, the stack light and the
operator panel's bezel are authored visual details that scale with the
generator's dimensions. They do not claim manufacturer CAD accuracy.

A catalog part is drawn the way it looks: a mesh panel as a tube frame with a
grid of wire in it, a conveyor with its rollers and drive, a rack with its
beams and braces. All of that is **decoration** — added with collision off,
under `<name>/trim/` — while the massing underneath (the panel slab, the belt,
the uprights) keeps collision and stops being drawn where the detail stands in
for it. Changing `detail` therefore never changes what a robot can hit, what
the BOM says, or what a plan gives you; it costs scene entries and nothing
else. `detail="plain"` is the bare massing, and is what a generator called
without a catalog does — there are no real sections to draw from.
[`examples/engineering/equipment_cell_demo.py`](https://github.com/botrail/botrail/blob/main/examples/engineering/equipment_cell_demo.py)
builds a cell whose fence, conveyor and rack all come from the catalog, and
prints the bill it can be ordered from;
[`examples/basics/demo.py`](https://github.com/botrail/botrail/blob/main/examples/basics/demo.py)
equips the tutorial cell the same way, next to a USD layer that keeps the
layout and the teach frames.

Where the drawing comes from is the product's business, not the generator's: a
pack can name a file of primitives and/or meshes per part (`components[].trim`, a URDF or
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

## Shapes a box cannot draw — the shape library

Some of what every cell has is neither a product nor a box: a carton with
its folded lids and tape, a pressed tray, a perforated basket, the rubber
foot under a bench leg, a bent handle, a drain hose, a machined part with
its bores. botrail ships a small library of these forms — `bt.parts.SHAPES`,
nine unit-box USD layers under `botrail/_shapes/`, authored in
[botrail-assets](https://github.com/botrail/botrail-assets) (`workshop-shapes/`)
and vendored by `scripts/sync_shapes.py` — and two ways to use them:

```python
scene.add_box("part", (0.06, 0.06, 0.04), (0.4, 0.2, 0.9))
bt.parts.appearance(scene, "part", "workpiece", (0.06, 0.06, 0.04))   # drawn as the machined blank
bt.parts.shaped_box(scene, "tray/grip", "handle", (0.09, 0.012, 0.045),
                    (0.4, 0.34, 0.8), quaternion=q)                   # decoration, out of collision
```

[`appearance`][botrail.parts.appearance] binds the layer's mesh to an
existing obstacle, scaled to `size` — the box the shape fills — so the
collision stays the box the cell was taught against while the picture
moves, attaches, saves and exports with the resident
([an appearance over a box](scene-and-obstacles.md#an-appearance-over-a-box)).
The layer's own finishes (kraft, tape, brushed steel, rubber …) are used;
`tint=True` paints the whole shape the obstacle's colour instead, which is
what the plain `panel` is for. [`shaped_box`][botrail.parts.shaped_box] is
`add_box` out of collision plus `appearance`: the picture of a handle or a
foot beside a resident that keeps its own massing.

A shape is a form, never a dimension: the same file draws a 200 mm tray and
a 600 mm one, and nothing a cell verifies — a set-down height, a grip — lives
in it. Scale it anisotropically within reason (a tube's thickness is its
smallest side) and keep `workpiece` for near-cubic boxes, since its bores
scale with the box.

| shape | draws | finishes |
|---|---|---|
| `carton` | a shipping carton: folded lids, seam, tape, label and barcode | kraft, packing tape, label paper and ink |
| `workpiece` | a chamfered machined blank with a stepped through-bore and four mounting bores | machined aluminium |
| `tray` | a pressed tray with drawn sides | brushed steel |
| `rim` | the rolled rim of a tank (open inside) | brushed steel |
| `basket` | a perforated sheet, its holes real (you see through it) | brushed steel |
| `adjuster` | a levelling foot: rubber pad, disc and threaded stem | rubber, brushed steel |
| `panel` | a laminate board with rounded corners — tint it | laminate |
| `handle` | a bent-tube grip, its opening along −Z | brushed steel |
| `hose` | a hanging drain hose | rubber |

## Series-specific equipment trims

A catalog fence's `height` / `height_mm` is the **panel** height; a pack's
`configuration.rules.floor_gap_mm` lifts the panel off the floor and the
posts reach panel height plus that gap — in the collision slabs, the
full-detail trims and the built-in frame-and-wire fallback alike. Fence,
conveyor and cabinet trims are handed the resolved pack parameters
(including string choices such as `post_finish`) on top of their metre
arguments; a fence panel trim's origin is its **bottom centre**, already
raised, and a post trim gets the installed height.

A pack's `components[].dimensions_mm` can add conservative collision
envelopes, in both detail modes and without touching the BOM:

| Component | Fields | Effect |
| --- | --- | --- |
| conveyor `unit` | `rail_rise` | Rise above belt top; default 40, use 0 for a flat standard frame |
| conveyor `unit` | `drive_length`, `drive_drop`, `drive_overhang` | Center-drive box below the frame, overhang on local -Y; declare all three together |
| conveyor `unit` | `mid_tension_after_length`, `mid_tension_length`, `mid_tension_drop` | Long-run tension box at local X = length/4, after the declared length threshold |
| conveyor `stand` | `inset` | Distance from each belt end to the first/last support center |
| cabinet `body` | `lifting_eye_height` | Conservative top slab above the enclosure, including any base offset |
| table `frame` | `inset_length`, `inset_width` | The leg centres this far in from the board's edge, the way a maker's frame is inset; the collision legs move with them |
| table `frame` | `top_thickness` | The board that comes with the frame (a `top` component is a board sold separately) |

A field left out keeps the generator's own massing; handles, fasteners,
wire openings and foot covers need no colliders of their own.
`detail="plain"` hides the trim but keeps these envelopes, the chosen
dimensions and the BOM.

A `table` frame trim that draws the board names it `top`, and the
generator hides its own board massing behind it. An `operator_panel` pack
can ship a `box` trim, drawn in place of the plate (the operators are still
the generator's, so a press stays a press), and an operator the pack sells
no article for — a complete station's E-stop — gets no BOM line of its own.
A `screw_feeder` pack can ship a `feeder` trim, drawn in place of the body
box; its rail and nest stay the generator's.

A part that is *carried* — a workpiece — is one resident, collision and
all, so its look is not decoration beside it but a picture bound to it:
`components[].visual` names one USD prim (`<layer>#<prim path>`, authored
in the part's own frame at its size) and [`workpiece`][botrail.parts.workpiece]
binds it to the housing and the cover the way
[an appearance over a box](scene-and-obstacles.md#an-appearance-over-a-box)
is bound. The picture moves, attaches, saves and exports with the part;
`detail="plain"` keeps the generator's massing.
