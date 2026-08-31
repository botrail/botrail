# Standard parts (`bt.parts`)

Generators for the structures every cell has — fences, walls, tables,
pedestals, racks, conveyor bodies, pallets, light curtains, stairs — built
from ordinary residents
(boxes, frames, a device or a sensor) with their [part](../../guides/parts-and-bom.md)
identity pinned, so the BOM counts them and the layout sheet labels them.
See [Standard parts and CAD geometry](../../guides/standard-parts.md).

```python
bt.parts.fence(scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)],
               height=2.0, panel_pitch=1.0, door=(0, 2), model="ST20")
ped = bt.parts.pedestal(scene, "pedestal", height=0.5, position=(0, 0))
scene.set_robot_base_pose(*scene.frame(ped.frames[0]))
```

::: botrail.parts
