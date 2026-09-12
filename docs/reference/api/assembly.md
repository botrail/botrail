# Assembly and fastening (`bt.assembly`)

Bolted joints as a drawing states them, the screwdriver's program, and
what a bake says about every screw. See [Assembly and fastening](../../guides/assembly.md).

```python
screw = bt.assembly.iso4762(5, 20)
joint = bt.assembly.joint(scene, "cover_joint", a="housing", b="cover", pattern_a=threads,
                          pattern_b=clearances, fastener=screw, seat=seat, thickness=0.010,
                          torque_nm=(4.0, 5.0), min_engagement_mm=10)
driver = bt.assembly.driver(scene, "driver", robot="arm", bit="drv_bit", shank="drv_shank",
                            stroke=0.055, fastener=screw, torque_nm=4.5, cycles=7)
fastening = bt.assembly.fasten(sq, joint, driver, feeder, motions={...})
```

::: botrail.assembly
