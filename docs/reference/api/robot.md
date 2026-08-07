# Robot

A kinematic model loaded from URDF, Xacro, or a USD articulation. A `Robot` is
immutable and holds no scene state — joint values live in the
[`Scene`](scene.md) that owns it, so the same model can be added to a scene more
than once.

```python
import botrail as bt

robot = bt.Robot.from_urdf("arm.urdf")
robot = bt.Robot.from_xacro("arm.urdf.xacro")          # expanded without ROS
robot = bt.Robot.from_usd("franka.usd")                # Isaac Sim articulations
```

::: botrail.Robot

## IkResult

Returned by [`Robot.ik`][botrail.Robot.ik] and
[`Scene.set_tcp_target`][botrail.Scene.set_tcp_target]. The solver is
best-effort: it applies the closest configuration it reached, so always check
`converged` before trusting the pose.

::: botrail.IkResult

## catalog_package

The catalog holds more than robots. A `workpiece` or a fixture is meshes and a
manifest, with no articulation to build a `Robot` from, so this resolves the
same product ids to a downloaded package directory and leaves the loading to
the caller — see [the catalog section](../../guides/robots.md#the-model-catalog).

::: botrail.catalog_package
