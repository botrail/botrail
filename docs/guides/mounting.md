# Mechanical mounting

`attach_tool` records an assembly: which product sits on which flange, and
at what transform. `bt.mounting.report` reads what the loaded products'
installation documents say about that assembly:

```python
import botrail as bt

result = bt.mounting.report(robot)   # also accepts a Scene
print(result.to_markdown())
result.save("mounting.json")
for item in result.blockers():
    print(item.target, item.status, item.message, item.next_action)
```

Each item has an `id`, its `target` (the same instance names the BOM uses),
a `status` of `pass`, `fail` or `unknown`, a `message`, a `next_action` and
the `evidence` it was read from. `ready` is true when no item failed or
stayed unknown, `blockers()` lists the ones that did, and the report renders
as Markdown or JSON (`to_markdown()`, `to_json()`, `save(path)`) alongside
the `assemblies` it walked and the `kits` it found. `scene.check()` includes
failed items as warnings; this report also retains unknowns and their evidence.

## What is checked

Product and assembly declarations:

* **`required:<id>`** — a part the product's installation documents require
  between it and the robot flange, and whether that part is actually on the
  attachment path. A bare Robotiq 2F-85 needs its ES-062 coupling
  ([manual, section 6.1.1](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190315.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm));
  translating the gripper by the coupling's thickness does not supply it,
  and a coupling attached somewhere else does not count.
* **`kit_host` / `kit_composition`** — whether a purchase kit is documented
  for the robot it is mounted on, and whether it is still assembled the way
  the manufacturer's part number describes.

These checks are evaluated alongside the drawing checks below. Having the
right part or a manufacturer-supported kit does not resolve missing hole,
fastener or clearance data. Payload and electrical compatibility are separate.

## Drawing-level fit

Each actual tool or arm attachment, including a rigid vehicle-mounted arm,
also produces these items:

| Item | What it checks |
| --- | --- |
| `requirements_complete` | An evidenced declaration that the installation lists all required separate parts. Individual `required:<id>` checks still examine their actual path and quantity. |
| `interface` | Agreement of the two evidenced bare-interface identifiers; matching labels do not replace dimensional checks or manufacturer host approval. |
| `declared_pose` | The actual flange-to-mount transform against explicit, evidenced `allowed_poses`. Omission is unknown. |
| `geometry` | Coincident, opposed mating planes; corresponding clearance/threaded holes and cylindrical locating features across their diameter, position-tolerance and depth bounds. |
| `fasteners` | Selected screws: nominal thread diameter, pitch and handedness, head standard, property class, declared length/torque limits, engagement and bottoming. |
| `clearance` | Declared conservative body boxes and tool-access boxes for the two adjacent parts in the actual assembly pose. |

These consume the catalog-builder vocabulary on `mounting.interfaces[]`:
`geometry`, `fasteners`, `clearance` and `requirements_complete`. No dimensions
are inferred from model meshes. Sources must be cited by index and document
section. Drawing frames require `frame_verified: true`; `complete: true`
declares coverage, not a substitute for individual missing fields or evidence.
Missing declarations produce `unknown`, including in old projects. A part
check or vehicle `mount_pose` can pass while the overall report stays unresolved.

Drawing lengths use **millimetres**, torque uses **N·m**, and
`allowed_poses.position` uses **metres**. Dimension ranges are `{min, max}`;
they are kept as ranges through loading, saving and generated Python. A pass
requires every value in the stated bounds to satisfy the condition. Disjoint
incompatible bounds fail; partial overlap is unknown. No default position
tolerance, thread pitch, minimum engagement or torque is invented.

Holes are paired by transformed position and complementary type, not their
names or list order. Uncertain correspondence stays unknown and a feature
cannot satisfy two holes. Pin/hole and boss/recess checks include radial fit
and usable depth. This is a planar threaded-hole/clearance-hole contract;
nut fastening, clamps, interference fits and strength calculations are outside
its scope.

For a selected screw on the clearance-hole side:

```text
tip depth  = screw length - grip - washer thickness
engagement = tip depth - unthreaded lead
```

`grip_mm` is bearing-surface-to-mating-face thickness, excluding washers.
`depth_mm` is usable tip depth including the drawing's bottom allowance.
Both faces' `fastener_rules` apply. The one documented default is omitted
`thread_start_mm` meaning zero unthreaded lead; missing usable depth or grip
remains unknown. The selected `fasteners[]` records one screw for each listed
clearance hole, with its length, washer, thread, grade and tightening torque.
This checks a declared assembly selection, not measured installation torque.

`clearance.solids` conservatively covers each body excluding the separately
checked mating features. `clearance.access` reserves tool space. Overlap means
a violation of these declared envelopes, not proof that the detailed CAD
surfaces collide. Both tool-access directions are checked; touching box
boundaries are allowed. Nonadjacent parts, approach motion and the surrounding
cell require separate collision checks.

JSON evidence includes the loaded identities, complete face declarations,
source references and individual checks with numerical bounds. `ready` now
requires these drawing items to pass as well as the part and kit checks;
existing callers that used part presence alone must inspect that item's
status instead. It remains limited to the report's scope and is not hardware
certification or a strength/load assessment.

## Vehicle-mounted arms

Give `mount_robot` the loaded carrier to record its catalog identity,
revision, source documents and mounting frame:

```python
carrier = bt.Robot.from_catalog("rb-kairos", format="usd")
# After adding the arm and the vehicle named "amr":
scene.mount_robot("amr", carrier=carrier)
print(bt.mounting.report(scene).to_markdown())
```

With no offset arguments, the first mount-side `allowed_poses` entry is
used when declared; otherwise the carrier's declared flange and the arm's
declared mount (or root) coincide, including their rotations. The vehicle
frame must be the carrier model's root frame. Only frames fixed to their
model roots are accepted. `flange=` and `mount=` accept exact link names;
selecting a different frame from the declared one remains unverified.

An explicit `offset_position` / `offset_quaternion` keeps the existing
meaning: vehicle frame to arm root. It is retained even if it differs from
the catalog frame, so an existing layout can be audited without moving it:

```python
scene.mount_robot("amr", carrier=carrier,
                 offset_position=(-0.2423, -0.1765, 0.7041))
for item in bt.mounting.report(scene).items:
    if item.key == "mount_pose":
        print(item.status, item.message, item.evidence)
```

`mount_pose` reports `pass` for a matching reference pose and `fail` for a
position or orientation discrepancy. It uses the same `allowed_poses`
format, source checks and quaternion comparison as tool mounting; an
explicit pose declaration without supporting evidence remains `unknown`.
Its evidence includes the expected
and actual offsets, translation/rotation errors, and captured carrier
provenance. For multiple permitted poses, a match uses that pose; a failure
reports the difference from the first pose and lists all permitted poses.
Missing carrier information, a non-catalog carrier or arm, or missing
sources for the carrier or a distinct arm mounting face produces `unknown`;
changing a BOM label cannot supply this information. An arm's model root
is a defined coordinate origin and can be compared without a separate
mounting-face declaration. Evidence labels that basis as `model_root` and
retains the arm's sources; it does not verify that origin as a physical
mounting face. Existing offset-only mounts still load and move as before.
Gait and rotor-spin mounts describe locomotion rather than a bolted arm and
are excluded from this check.

This item's scope is **catalog frame alignment**. A deliberate adapter or
custom bracket may offset those frames; a failure does not mean such an
assembly is physically impossible. A pass does not establish approved host
combinations, hole/fastener fit or load capacity. The numerical comparison
tolerance is not a manufacturing tolerance. `ready` only means no reported
item is failed or unknown.

The reference survives snapshots, `.botrail` files and generated Python,
including `embed_catalog=True`. It contains frame data and provenance, so
replaying it needs no carrier download or duplicate carrier geometry.
The captured mounting declarations and order requirements participate in the
same drawing and required-part checks as tools. If the carrier is itself an
uncaptured composite or kit, or an older snapshot lacks path completeness,
absence of a required part on that incomplete path stays unknown. To review
each adapter's two interfaces, model that chain with actual `Robot` attachments.
The AMR example uses the catalog frame directly, with no corner offset or
invented adapter plate. Its RB-KAIROS arm root is at `(0, 0, 0.688620001)` m
in the carrier frame. The bench, deck and belt poses and travelling fold
are taught for that placement, including the unrotated holonomic arrival.
The CLI reports `mount_pose` and the overall unresolved mounting count
separately: matching the frame does not supply missing mechanical drawings.

## Manufacturer kits

A `kind: kit` catalog entry is a purchase unit. Loading it assembles its
component packages with their mounting frames and TCP:

```python
arm = bt.Robot.from_catalog("universal_robots/ur/ur5e/r2")
kit = bt.Robot.from_catalog("robotiq/2f/2f-85-ur-es-062-kit/r2")
robot = arm.attach_tool(kit)   # the coupling and the hand, once each, under `kit_`
scene = bt.Scene(robot)

purchase = scene.bom().rows[-1]
print(purchase["qty"], purchase["order"]["part_number"])   # 1 kit SKU
print(purchase["order"]["includes"])                       # per kit, not extra purchases

for entry in bt.mounting.report(scene).kits:
    print(entry["target"], entry["host"],
          entry["manufacturer_support"]["status"], entry["composition"]["status"])
```

A kit lands under the `kit_` prefix unless you pass `prefix=` (its coupling
exposes a `flange` link, as the arm does), so its links read
`kit_gripper/tcp` and so on. The BOM shows the kit as one row with its `order.part_number` and
`order.includes`; the included parts are not additional purchases. Each entry
of `report.kits` names its `target`, `catalog` ID and `host`, with
`manufacturer_support` (`status`, `message`) and `composition` (`status`).

The ES-062 kit is `AGC-ES-UR-KIT-85` as documented in the
[Robotiq quick-start guide](https://blog.robotiq.com/hubfs/support-files/Quick_start_2Finger_e-Series_nocropmarks_EN.pdf);
its coupling is `GRP-ES-CPL-062`, and the bare gripper keeps its own catalog
ID. Robotiq distinguishes
[ES-062 and ES-077 wrists](https://blog.robotiq.com/knowledge/couplings-and-cables-for-universal-robots-robots),
so check which one the robot has.
