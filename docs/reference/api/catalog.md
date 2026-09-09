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

These packages use independently authored reference geometry. UR keeps the
official default kinematics, joint limits, inertia and `tool0` frame; its
shells and collision shapes are approximate. RG6 uses a parallel linkage
with one drive: `0` rad opens approximately 160 mm and `1.3` rad closes it.
At full closure the opposing pads touch; use a smaller angle for an empty,
collision-free motion goal. Its fixed TCP retains the previous datum.
LIFTKIT is the UR20/UR30 **620 variant with 800 mm stroke**: the drive ranges
from `0` to `0.4` m and the second stage follows it, giving a mount height
of 0.905–1.705 m. Its two stage speed limits total 0.08 m/s.

Mounting holes, robot-specific adapters and hardware communication are not
provided by these reference models. Detailed fit and per-link dynamics of
RG6/LIFTKIT remain unverified. Read `compatibility.model_fidelity` in each
manifest and its `sources/<asset>/README.md` before using model geometry or
catalog mass values for engineering decisions.

The original `r1` packages remain `recipe_only`. Pin both the full product
ID and a dataset `revision` to reproduce an existing project. Saving with
`scene.save_project(...)` bundles the loaded geometry for offline replay.

::: botrail.catalog
