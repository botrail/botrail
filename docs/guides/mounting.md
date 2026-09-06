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
| Required parts | The catalog product and quantity referenced by `order.requires`, or an explicitly permitted `interface_pair` adapter, must occur on this tool's upstream attachment path. The alternative must use both specified faces on that path. A spare elsewhere does not count. |
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

For example, a Robotiq 2F gripper requires a separate coupling; translating
the gripper by the coupling thickness does not supply that part. The
manufacturer documents this requirement in [section 6.1.1 of its manual](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190315.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm).
The catalog's EMSF spindle reference model and reconstructed X16005 weld gun
retain unknown robot-side brackets and mounting details.

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
| `holes[]` | Local `id`, XY `position_mm`, `kind` (`clearance` or `threaded`), optional `position_tolerance_mm` (radial bound), `diameter_mm`, `thread`, `depth_mm`, `grip_mm` and `fastener_rules`. Hole names/order do not establish a match. |
| `thread` | `diameter_mm`, optional `pitch_mm`, and `left_hand` (default false). Missing pitch stays unknown. |
| `locators[]` | Local `id`, XY position/tolerance, `kind` (`boss`, `recess`, `pin`, `hole`), diameter and projection/usable recess depth. |
| `fastener_rules` | Drawing requirements: optional `thread`, `head_standard`, `property_class`, `length_mm`, `min_engagement_mm` and `torque_nm`. |
| `fasteners[]` | Selected screws on the clearance side: `holes` (one screw per listed ID), `thread`, `head_standard`, `property_class`, `length_mm`, `washer_mm`, `torque_nm` and `evidence`. Explicitly set washer thickness to zero if absent. |
| `requirements_complete` | The cited installation declaration covers all required mounting parts. An empty list with this flag means no additional adapter; the default false means unconfirmed completeness. |

Dimensions use `{min: ..., max: ...}` except coordinates, thread sizes and minimum
engagement. Engagement is screw length minus the bearing stack (`grip_mm`) minus
washers, compared with the minimum engagement and **usable** receiver depth.
Account for counterbores in the bearing stack and for bottom/tip allowances in
usable depth. A maximum flange penetration is not the total screw length.
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

Detailed comparisons are available in each item's `evidence["checks"]`, with
their statuses and input dimensions. JSON preserves these inputs and source
references; Markdown includes the individual comparison names and results.

## Results and replay

The report provides `items`, `assemblies`, `ready`, `blockers()`, `scope`,
`validator_version` and `input_hash`, plus JSON/Markdown export. Each assembly
names its base, tool, original product frames, relative pose and upstream
part instances. Item IDs use the same instance names as the BOM.

`input_hash` is a deterministic FNV-1a fingerprint of the evaluated declarations,
assembly paths and catalog annotations, not a security digest or a hardware
measurement. It is useful together with `validator_version` when comparing reports.

Catalog mounting conditions, order requirements and cited source records are
embedded in the immutable model source and `.botrail` project. Project load
reuses the embedded model and needs no catalog lookup. Generated Python loads
the pinned catalog model and restores the saved declaration snapshot, including
absent metadata from old projects. Changes in later catalog data do not silently
upgrade an old project's mounting evidence.

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
