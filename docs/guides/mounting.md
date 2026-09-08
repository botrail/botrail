# Review mechanical tool mounting

`attach_tool` records an assembly and its transform. To review the products
and mounting declarations in that assembly, use:

```python
import botrail as bt

result = bt.mounting.report(robot)  # also accepts a Scene
print(result.to_markdown())
result.save("mounting.json")

scene = bt.Scene(robot)
review = bt.review(scene, required=["mounting"])
for item in review.blockers():
    print(item.target, item.status, item.message, item.next_action)
```

Both APIs use the same Rust checker. A scene also compares authored
`set_part(catalog=...)` identity with the product actually loaded. A catalog
label on a different model cannot satisfy a required adapter.

## What is checked

| Item | Current comparison |
|---|---|
| Required parts | The catalog product and quantity referenced by `order.requires`, a documented `catalog_alternatives` product, or an explicitly permitted `interface_pair` adapter must occur on this tool's upstream attachment path. Interface-pair alternatives must use both specified faces on that path. A spare elsewhere does not count. |
| Bare mating interfaces | Explicit `interface_id` declarations on the two connected faces are compared when supported by cited manufacturer/standard/official source records. Legacy `flange_standard` or compatibility text is not converted into a mating-face declaration. |
| Assembly pose | The recorded transform is compared with explicitly documented `allowed_poses`. Translation is in metres; quaternions are unit XYZW. The comparison tolerance, `1e-8`, handles numerical roundoff and is not a manufacturing tolerance. |
| Product identity | A conflicting catalog ID or pinned dataset revision in `set_part` is a failure. Free-form BOM names do not prove product identity. |
| Dimensions | Planar mating faces, screw-hole positions, clearance/threaded pairing, cylindrical pilots/pins, diameters, depths and positional tolerances. |
| Fasteners | One selected screw per required hole; thread diameter, pitch and handedness; prescribed head, property class, length, tightening torque and usable engagement. |
| Assembly space | Declared conservative body envelopes and tool-access boxes of the two adjacent mating parts at their attachment pose. |

Missing declarations, unknown brackets and insufficient evidence remain
`unknown`. Community mesh provenance alone does not verify a mechanical
interface. A matching interface identifier is a partial comparison: it does
not establish the fit of holes, threads, pilots or the model geometry.

For example, a bare Robotiq 2F gripper needs a coupling in the assembly.
Manufacturer UR kits include the appropriate coupling; translating the bare
gripper by the coupling thickness does not supply that part. The
manufacturer documents this requirement in [section 6.1.1 of its manual](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190315.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm).

The EMSF-3060K spindle needs a holder: its four M4 mounting screws on a 39 mm
bolt circle do not directly fit the RV-5AS-D's four M5 holes on a 31.5 mm bolt
circle ([spindle manual, Fig. 1 and section 10](https://en.nakanishi-spindle.com/wp-content/uploads/existing/industrial-eng/download/manual/sale/OM-KK0978EN000_Motor-Spindle_EMSF-3060K_Manual_EN_no-collet.pdf),
[robot manual, Fig. 2-3](https://dl.mitsubishielectric.com/dl/fa/document/manual/robot/bfp-a3727/bfp-a3727p.pdf)).
An offset cannot supply that holder. The reference spindle's simplified flange
also differs from the drawing; nominal hole coordinates do not establish their
location on that mesh.

For the reconstructed X16005 weld gun, the [manufacturer's specification](https://heron-welder.com/x-type-robotic-welding-gun-with-servo-actuator/)
lists attachment sides but does not identify this model's robot-side bracket.
Its robot-brand statement concerns servo-motor integration. Select the gun
configuration, mounting side, bracket and robot flange option before recording
a manufacturer-supported assembly. A custom bracket needs its own drawings.

**An attached assembly remains unresolved while required detailed checks
are `unknown` or `not_run`.** `result.ready` covers all mounting items.
`bt.review` adds the mounting group to its required scope only when requested;
explicit failures always block either review stage. `scene.check()` reports
known mounting contradictions as errors and missing information as warnings.
A robot with no declared tool attachment has a `not_applicable` observation;
that observation does not establish whether the scene includes all equipment.

## Attach a drawing to a custom part

```python
bracket = bt.Robot.from_urdf("bracket.urdf").with_mounting("bracket.mounting.yaml")
robot = arm.attach_tool(bracket, flange="tool0", mount="mount", prefix="bracket_")
robot = robot.attach_tool(tool, flange="bracket_out")
result = bt.mounting.report(robot)
```

Attach the document to each individual part before assembling it. URDF and USD
sources are supported. Use existing link/frame names; a USD leaf name is accepted
only when it is unique. This API adds declarations to a new immutable `Robot`;
it does not add geometry or change the default TCP or attachment frames.

A drawing document has `schema_version: "1"`, a nonempty `revision`, `sources`
and `mounting`. Optional `order` follows the catalog order format. For example:

```yaml
schema_version: "1"
revision: bracket-drawing-A
sources:
  - kind: user_drawing
    url: drawing:bracket-A
    ref: A
mounting:
  interfaces:
    - frame: mount
      role: mount
      interface_id: project:robot-face
      evidence: [{source: 0, section: "Drawing A, face M"}]
      geometry:
        normal_z: -1
        frame_verified: false
        complete: false
        holes:
          - id: mounting-screw-1
            position_mm: [20, 20]
            kind: clearance
            diameter_mm: {min: 6.4, max: 6.5}
            grip_mm: {min: 5, max: 5.1}
        evidence: [{source: 0, section: "Drawing A, face M"}]
```

This is an incomplete syntax example, not a verified bracket. Add a `flange`
interface for the bracket's output frame and use your actual drawing values.
Manufacturer drawings, standards, `user_drawing` and `measurement` records can
support declarations; these record who supplied the evidence, not independent
certification. A source URL is recorded, not automatically downloaded or verified.

Each face in a document replaces the declaration for the same frame and role.
Other catalog faces and **all existing required-part obligations remain**.
Document requirements are appended with `document:` IDs and rebased source/order
indices. Calling `with_mounting` again replaces that part's previous custom
document. Changing or deleting the YAML file never changes an existing robot;
explicitly load the new revision to use it.

## Drawing and fastener fields

All geometric dimensions below are **millimetres**. `allowed_poses.position`
and `attach_tool(offset_position=...)` remain **metres**. A face uses its frame's
XY plane at Z=0 and an outward `normal_z` of +1 or -1. The two normals must oppose
one another after applying the assembly transform.

| Field | Meaning |
|---|---|
| `geometry.frame_verified` | The cited drawing coordinates and mating plane have been checked against this model frame. Default false. |
| `geometry.complete` | Every required mating feature and dimensional/positional tolerance is covered. Default false. Equal min/max values alone do not assert a complete drawing. |
| `holes[]` | Local `id`, XY `position_mm`, `kind` (`clearance` or `threaded`), optional `position_tolerance_mm` (radial bound), `diameter_mm`, `thread`, `depth_mm`, `thread_start_mm`, `grip_mm` and `fastener_rules`. Hole names/order do not establish a match. |
| `thread` | `diameter_mm`, optional `pitch_mm`, and `left_hand` (default false). Missing pitch stays unknown. |
| `locators[]` | Local `id`, XY position/tolerance, `kind` (`boss`, `recess`, `pin`, `hole`), diameter and projection/usable recess depth. |
| `fastener_rules` | Drawing requirements: optional `thread`, `head_standard`, `property_class`, `length_mm`, `min_engagement_mm` and `torque_nm`. |
| `fasteners[]` | Selected screws on the clearance side: `holes` (one screw per listed ID), `thread`, `head_standard`, `property_class`, `length_mm`, `washer_mm`, `torque_nm` and `evidence`. Explicitly set washer thickness to zero if absent. |
| `requirements_complete` | The cited installation declaration covers all required mounting parts. An empty list with this flag means no additional adapter; the default false means unconfirmed completeness. |

Dimensions use `{min: ..., max: ...}` except coordinates, thread sizes and minimum
engagement. Tip penetration is screw length minus the bearing stack (`grip_mm`)
minus washers. Engagement additionally subtracts the receiver's unthreaded lead,
`thread_start_mm`. Omitted lead means zero for compatibility with existing
drawings; supply it explicitly for a recessed thread. Compare engagement with
the required minimum and tip penetration with **usable** `depth_mm`, both depths
measured from the mating plane. Account for counterbores in the bearing stack
and for bottom/tip allowances in usable depth. For example, 6 mm penetration
with a 2 mm unthreaded lead gives only 4 mm engagement. A maximum flange
penetration is not the total screw length.
Head and property-class strings are compared exactly; the checker does not infer
that a different class or head is an acceptable substitution.

Range checks pass only when the whole stated range satisfies the constraint;
partial overlap stays unknown. This contract covers planar joints with screws
through clearance holes into threaded receivers. Through-bolts with nuts,
tapered/clamped interfaces and arbitrary CAD feature extraction need another
contract and cannot receive a complete fit result from these declarations.

## Assembly envelopes and adapter alternatives

`clearance` carries `frame_verified`, `complete`, `evidence`, `solids` and `access`.
Each box has `id`, `center_mm: [x, y, z]` and positive `size_mm: [x, y, z]` in the
face frame. Solids conservatively cover the non-mating body; any excluded mating
features must be covered by the dimensional checks. Access boxes reserve the
space needed for screws, sockets or other assembly tools. `complete: true`
asserts that this coverage is sufficient for this joint; empty access means the
source requires no additional access reserve. An omitted declaration does not
make that assertion.

Penetrating envelopes produce `fail` for the declared clearance constraint.
Envelope overlap can be conservative and does not prove actual CAD interference.
Face touching is allowed. These checks cover adjacent parts at the assembled pose;
approach motion, nonadjacent parts, external equipment, strength, payload and
process suitability require their own checks. An approximate visual mesh alone
cannot satisfy drawing or envelope evidence.

Where the installation drawing explicitly permits a mechanical alternative,
use `requirements[].interface_pair: {mount: project:robot-face, flange: project:tool-face}`
with evidence. This accepts a custom adapter using those two faces on the actual
path, while keeping all dimension, screw and clearance checks. Omit it for a
product-specific obligation such as the electronics-bearing Robotiq coupling.

For explicitly documented product variants, add `catalog_alternatives` to the
requirement instead:

```yaml
catalog_alternatives:
  - catalog: robotiq/coupling/agc-cpl-062-002/r1
    evidence: [{source: 1, section: "20181130 manual, pp.123-124 and p.152"}]
```

The referenced source must be present in that part's `sources`; the requirement
also needs its own evidence. Only exact loaded catalog IDs count. A present
alternative without supported evidence remains unknown. Quantities count each
physical instance once, and all mating-interface and detailed fit checks still
apply. The AGC example is a legacy variant; its declarations cannot be transferred
to GRP or GRP-ES models solely because all use a nominal ISO50 input.

Detailed comparisons are available in each item's `evidence["checks"]`, with
their statuses and input dimensions. JSON preserves these inputs and source
references; Markdown includes the individual comparison names and results.

## Results and replay

The report provides `items`, `assemblies`, `simulation`, `ready`, `blockers()`, `scope`,
`validator_version` and `input_hash`, plus JSON/Markdown export. Each assembly
names its base, tool, original product frames, relative pose and upstream
part instances. Item IDs use the same instance names as the BOM.

### Simulation routes and detailed checks

A manufacturer kit is one way to establish a mounting route. Individual catalog
parts and custom parts with `with_mounting` drawing documents can also establish
one. `result.simulation` separates the connection method from its evidence:

```python
result = bt.mounting.report(robot)
print(result.simulation["ready"])  # mounting eligibility for simulation
for connection in result.simulation["connections"]:
    print(connection["target"], connection["method"], connection["basis"])
print(result.ready)  # all required detailed mounting checks
```

| Field | Values |
|---|---|
| `method` | `direct`, `catalog_adapter`, `custom_adapter`, or `unknown` |
| `basis` | `manufacturer_kit`, `interface_declarations`, `dimensional_checks`, or `unknown` |
| `evidence_items` | IDs of passed observations; follow their evidence to the recorded sources |

The method follows the actual upstream attachment path. `catalog_adapter` means
the intervening parts have catalog identities, not that Botrail has verified
their commercial availability. A path containing an intervening part without a
catalog identity is `custom_adapter`. Manufacturer support is a separate result;
neither catalog registration nor a user drawing creates a manufacturer claim.

For `interface_declarations`, both bare faces must have the same explicitly
documented identifier, the actual pose must match a documented permitted pose,
and the complete required-part declaration must be satisfied on that path.
Legacy flange-standard text and matching names alone are insufficient.
`dimensional_checks` can instead use passing dimensional and fastener checks,
together with the permitted pose and complete part requirements; it does not
require inventing a common interface name.

These routes can be applied while detailed dimensions, fastening or assembly
access remain `unknown` / `not_run`. Their findings are retained, and any explicit
failure still blocks application. Supported evidence on one connection cannot
cover another connection. `simulation["blockers"]` lists mounting finding IDs;
`preview(...).can_apply` additionally checks scene references and revision state.
The strict `ready`, `blockers()` and `bt.review(..., required=["mounting"])`
contracts are unchanged. None of these results establish payload, strength,
electrical compatibility or process performance.

Studio shows the method and evidence for each connection in both the inspection
and assembly comparison views, alongside the existing detailed findings.

`input_hash` is a deterministic FNV-1a fingerprint of the evaluated declarations,
assembly paths and catalog annotations, not a security digest or a hardware
measurement. It is useful together with `validator_version` when comparing reports.

Catalog mounting conditions, order requirements and cited source records are
embedded in the immutable model source and `.botrail` project. Project load
reuses the embedded model and needs no catalog lookup. Generated Python loads
the pinned catalog model and restores the saved declaration snapshot, including
absent metadata from old projects. Changes in later catalog data do not silently
upgrade an old project's mounting evidence.

Locally built packages loaded with `Robot.from_package` retain their manifest
declarations and a local content digest. Their generated Python restores the
embedded source, since a local digest is not a downloadable dataset commit.

Packages without mounting metadata still load. Their attached tools report
missing information. New declarations become available through a rebuilt
catalog package and its dataset revision; editing a builder recipe alone does
not change an already loaded or published package.

An adapter's presence and its fit are separate results. For example, the ROS-I
mesh used by `robotiq/coupling/gripper-coupling/r1` has a 40 mm bolt circle,
corresponding to the [ISO40 coupling drawing, Fig.6-8](https://assets.robotiq.com/website-assets/support_archives/document_en/2F-85_2F-140_Instruction_Manual_PDF_20181130.pdf#page=126).
A UR5e has an [ISO50 face](https://www.universal-robots.com/media/1833559/ur5e-tool-drawing.jpg). Declarations
that record these faces produce an interface `fail` even when the required
coupling is present. The failure message names both interfaces. Older snapshots
that lack the corrected declaration keep their original `unknown` result.

Use the robot-specific coupling model and its installation documents when
building a verified assembly. The coupling's outside thickness is not necessarily
the distance between its recessed bearing planes, and a mesh measurement does
not supply manufacturing tolerances or a fastener specification.

## Inspecting the connection in Studio

Open **Robot → Inspect mounting** to inspect the current assembly. The connection
strip follows the attached components; choose a connection to bring its mating
frames and downstream tool into view. A kit keeps its purchase grouping while
showing its internal connections. Manufacturer support, composition, model
correspondence and detailed fit remain separate results.

The inspector provides five views:

- **Assembled** shows the model at its actual mounting pose, with inspection colors.
- **Exploded** separates the selected connection for viewing. Adjust separation,
  make the tool transparent, or temporarily hide the flange geometry.
- **Hole overlay** projects documented features into the flange coordinate system.
  Select holes in the drawing or the feature selector to see their dimensions,
  the checker's paired hole and the source evidence.
- **Screw section** shows a schematic bearing stack, engagement, unthreaded lead
  and allowed tip depth. Intervals and depth limits are drawn as bands or dashed
  lines. Screw heads and the physical bore bottom are not reconstructed.
- **Tool access** shows the declared mating-part solids and required access
  volumes, highlighting overlaps reported by the checker.

Drawing features appear on the model only when the declared frame mapping has
supporting evidence. An unconfirmed side retains its separate drawing and missing
information; geometry is not guessed to make it look connected. Wire shafts are
dimensioned overlays, not additional purchased fasteners. An empty attachment
list is shown as **No mounting connection to inspect**.

The inspector uses a separate camera and rendering materials. These controls
leave the Scene, TCP, BOM, trajectories and mounting input hash unchanged.
The report and frame poses form a read-only snapshot; **Refresh** takes another
snapshot. Composition and part-annotation updates invalidate an open report.
**Report ↓** downloads the Rust mounting report, including its input hash and
evidence. Its checks match `bt.mounting.report(scene)`; Python additionally
exposes its review annotation and derived `blocking` fields.

Native and WASM sessions handle the same `inspect_mounting` request through the
shared Rust checker. This view does not extend the check to nearby cell equipment
or validate an assembly approach path.

## Manufacturer kits

A `kind: kit` catalog entry identifies a purchase configuration. Loading it
assembles the referenced component packages with their mounting frames and TCP:

```python
kit = bt.Robot.from_catalog("robotiq/2f/2f-85-ur-es-062-kit/r2")
robot = arm.attach_tool(kit)
scene = bt.Scene(robot)

purchase = scene.bom().rows[-1]
print(purchase["qty"], purchase["order"]["part_number"])  # 1 kit SKU
print(purchase["order"]["includes"])  # quantities per kit, not extra purchases

result = bt.mounting.report(scene)
for kit_result in result.kits:
    for key in ("manufacturer_support", "composition", "model_correspondence", "detailed_fit"):
        print(key, kit_result[key]["status"])
```

`Robot.from_catalog(kit_id, revision=...)` loads publicly distributed kits and
all dependencies at the same dataset commit. For a locally built kit,
`catalog_root` contains the component directories at their full catalog IDs;
it can be omitted when the kit itself occupies `<root>/<full-id>`.
Missing packages, cycles, inconsistent included quantities and mismatched IDs
are errors. Local kit revisions include the component package fingerprints.
Saved projects bundle the assembly's model assets for offline replay.

The report separates documented mechanical support for the actual directly
attached host, agreement with the kit assembly, model correspondence, and
detailed fit. Missing dimensional inputs do not negate a manufacturer's
compatibility statement. Conversely, that statement cannot override an
incorrect component or a failed dimensional check. `ready` continues to mean
that **all required detailed checks** have passed. Electrical compatibility is
not inferred from mechanical support.

For simulation, `bt.mounting.preview(...).can_apply` also accepts an unchanged
manufacturer-supported kit when internal dimensional or fastening inputs are
missing. The claim covers only that kit's connections and its actual host
mount: the declared mounting frames and either an explicitly permitted pose
or their default coincident placement must match. This placement convention
does not verify the model's seating datum or TCP calibration.

The detailed results remain `unknown` / `not_run`, and `result.ready` retains
its strict meaning. Known mismatches, missing required components, and
unresolved connections outside the supported assembly still block application.
`proposal.mounting_blockers` lists the finding IDs that prevent simulation;
`proposal.blockers` lists scene-reference problems. Studio shows manufacturer
support and model correspondence separately, with detailed findings expandable.

The ES-062 entry identifies `AGC-ES-UR-KIT-85` as documented in the 2021
[Robotiq quick-start guide](https://blog.robotiq.com/hubfs/support-files/Quick_start_2Finger_e-Series_nocropmarks_EN.pdf).
Its included coupling is `GRP-ES-CPL-062`; the gripper's existing standalone
catalog ID still represents the bare hand. This kit uses a reference gripper
model, and the protector and installation hardware are recorded in the BOM
without separate geometry. Verify the actual wrist connector generation:
Robotiq distinguishes [ES-062 and ES-077 configurations](https://blog.robotiq.com/knowledge/couplings-and-cables-for-universal-robots-robots).

The same purchase model applies to process tools. ATI documents the
`9150-COB-CRX10-RCV250-01` spindle kit for CRX-10iA: robot interface plate
`3700-50-9210`, side mounting bracket kit `9005-50-6091`, and spindle
`9150-RCV-250` ([ATI CRX documentation bundle](https://www.ati-ia.com/library/documents/ATI_MR_CRX.zip),
manual `9610-50-1049-03`, Table 2.5; assembly drawing `9640-50-1041`).
Represent both adapters as components of the purchased kit, with the spindle
attached to the bracket's output frame. Missing either adapter or changing an
internal attachment invalidates the recorded kit configuration. Mechanical
support does not establish cutting-tip calibration, pneumatic connections or
compliance behavior. Prebuilt public reference models are available:

```python
arm = bt.Robot.from_catalog("fanuc/crx/crx10ia/r1")
kit = bt.Robot.from_catalog("ati/rcv/rcv-250-crx10-kit/r1")
scene = bt.Scene(arm.attach_tool(kit))
```

The authored plate models preserve the measured hole positions and support
planes. The kit's screws, pins, cutting bit and pneumatic equipment are BOM-only;
its TCP is a mounting reference. No manufacturer CAD build is required.
To inspect the assembly in Studio, run:

```bash
uv run python examples/machining/spindle_mounting_demo.py --studio
```

## Compare and reuse assemblies

Keep composition explicit: build another `Robot` with `attach_tool` / `mount`,
then inspect the change before replacing the robot in the cell:

```python
# arm, adapter and tool are loaded catalog products or local packages.
candidate = arm.attach_tool(adapter, prefix="adapter_").attach_tool(
    tool, flange="adapter_out", prefix="tool_"
)
proposal = bt.mounting.preview(scene, candidate, robot="arm")
print(proposal.route, proposal.can_apply)
print(proposal.before["tcp"], proposal.after["tcp"])
print(proposal.after["mass"], proposal.after["bom"])
print(proposal.mounting_blockers, proposal.blockers, proposal.revalidation)
proposal.save("tool-assembly.botrail")

# Preserve the findings; apply only when the simulation conditions are met.
if proposal.can_apply:
    proposal.apply()

# Reuse the actual saved model and inspect it against the destination scene.
reused = bt.Scene.load_project("tool-assembly.botrail").robot
comparison = bt.mounting.preview(other_scene, reused)
```

Use the actual frame names declared by the products in place of `adapter_out`.
`bt.mounting.candidates(scene, [candidate_a, candidate_b])` compares supplied
models and orders them by **direct mounting evidence**, **adapter route
evidence**, and **needs design / information**. These categories describe the
route's declarations or manufacturer kit support; they do not override a failed
screw/dimension check. The function retains unknown and failing candidates.
It does not search every catalog product or design a bracket automatically.

`proposal.scene` is the separate candidate scene at the transferred pose.
Saving it uses the existing project format, including asset bundling and
catalog/kit provenance. No confirmation flag is saved. Loading and inspecting
recomputes the result. A kit remains one purchase unit; included components
are not additional purchasing rows. Mass comes from the BOM's declared
`mass_kg`: `known_kg` is a **subtotal** if `missing` lists any components, and
`null` means no declared mass was available.

Apply rebuilds the model, collision geometry and source graph. It transfers
joint values only when the joint name, physical definition, ancestor chain,
and available catalog identity match. Other joints start at neutral. Existing
scene references are resolved by name; missing attachment frames or motions
whose joint identities changed block the replacement. Application checks both
snapshots again: changed poses, scene content or candidate inputs require a
new preview. Keep the original `Robot` objects for any independent scripts;
`scene.robot` / `scene.robot_of(name)` return the current scene model.

Affected motions, toolpaths and sequences appear in
`scene.mounting_revalidation`, which survives project save/load. Rerun the
motion planner, toolpath checker or sequence simulation; successful evaluations
clear the corresponding entries. Previously retained recordings, rollouts and
Studio playback are invalidated, including jobs started before the replacement.
Self-collision exclusions are recomputed and inter-robot exclusions touching
the replaced robot are cleared; review authored process-contact allowances.

### Studio workflow

Open **Robot → Inspect mounting → Compare assemblies**:

1. Select a catalog query, a package directory on the server, or a saved JSON
   project. Multi-robot projects require selecting one robot. The tool's explicit
   unmet product requirements also appear as candidate shortcuts.
2. Choose to replace the assembly, attach a part, or insert an adapter at a
   selected existing tool connection. Specify frames when the product does not
   declare them. Insertion keeps the downstream tool and its original prefix.
3. Preview the current and proposed assemblies at the same scale, inspect a
   connection in assembled or exploded view, and compare TCP, declared mass,
   purchasing BOM and mounting findings. Multiple proposals remain selectable.
4. **Save proposal** keeps unresolved alternatives. **Apply checked assembly**
   requires resolved mounting checks and valid scene references, and performs
   the snapshot checks again on the server.

Studio downloads assembly projects as JSON, with references to available assets.
For portable bundled projects, use `proposal.save(...)` in Python. The browser
file picker currently accepts JSON projects; load ZIP projects with
`Scene.load_project` in Python. In the standalone WASM session, embedded URDF
projects can be compared and applied. Catalog fetching, server package paths and
USD re-import require the Python Studio server; the UI does not offer these as
available WASM capabilities.
