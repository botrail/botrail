# Mechanical tool mounting

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
the `assemblies` it walked and the `kits` it found. `scene.check()` does not
include these items; read them from this report.

## What is checked

Two kinds of item, both read from the products' documents:

* **`required:<id>`** — a part the product's installation documents require
  between it and the robot flange, and whether that part is actually on the
  attachment path. A bare Robotiq 2F-85 needs its ES-062 coupling
  ([manual, section 6.1.1](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190315.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm));
  translating the gripper by the coupling's thickness does not supply it,
  and a coupling attached somewhere else does not count.
* **`kit_host` / `kit_composition`** — whether a purchase kit is documented
  for the robot it is mounted on, and whether it is still assembled the way
  the manufacturer's part number describes.

Nothing is measured or inferred from geometry: a `pass` repeats a
manufacturer statement about the products actually loaded, and `unknown`
means the loaded products carry no such statement. Holes, threads, payload
and electrical compatibility are not part of this report.

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
