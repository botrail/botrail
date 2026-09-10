# Catalog search (`bt.catalog`)

Real products to choose from: the catalog's published index, filtered the
way a requirement reads. See [Selecting parts](../../guides/selection.md).

```python
cands = bt.catalog.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)
cands[0].identify(scene, "ur5e/tool")
bt.catalog.search_for(scene.requirements()["eye"], level="V2")
```

## Public UR Series and UR+ reference models

The r2 packages for UR8 Long, UR15, UR18, UR20, UR30, OnRobot RG6 and
Ewellix LIFTKIT-UR are prebuilt public models. They load without running
catalog-builder or installing ROS:

```python
import botrail as bt

arm = bt.Robot.from_catalog("universal_robots/ur/ur20/r2")
gripper = bt.Robot.from_catalog("onrobot/rg/rg6/r2")
lift = bt.Robot.from_catalog("ewellix/liftkit/liftkit-ur/r2")
# Also available: ur8-long, ur15, ur18 and ur30 in universal_robots/ur/*/r2.
# Use format="usd" to choose USD instead of the default URDF.
```

These are reference shells: the UR arms keep the official kinematics, joint
limits, inertia and `tool0` frame under approximate covers; the RG6 is a
one-drive parallel linkage (`0` rad opens about 160 mm, `1.3` rad closes it
until the pads touch — aim a little short of that for an empty grasp); the
LIFTKIT is the 620 variant with its 800 mm stroke (`0`–`0.4` m of drive, the
second stage following, 0.905–1.705 m of mount height, 0.08 m/s over both
stages). Mounting holes, adapters and communication are not in them, and
`compatibility.model_fidelity` in each manifest says how far to trust the
geometry and the masses. The `r1` packages stay `recipe_only`; pin the full
id and a `revision` to reproduce a cell, and `save_project` bundles the
geometry for replay without the catalog.

::: botrail.catalog
