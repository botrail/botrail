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
