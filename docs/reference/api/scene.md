# Scene

The cell: one or more robots, the obstacles around them, the frames they mount
on, the devices and sensors that give the environment behavior, the named
motions, and the sequences that drive it all.

Units are **meters**, and the world is **Z-up**. Orientations are quaternions in
`(x, y, z, w)` order. Wherever a method takes `robot=None`, it acts on the
scene's first robot.

A scene is also the live link to the studio: state changes made from Python are
pushed to connected browsers, and edits made in the browser are visible here.

```python
import botrail as bt

scene = bt.Scene(bt.Robot.from_urdf("arm.urdf"))
scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))
scene.load_usd("cell.usda", prefix="env")
scene.set_robot_base_pose(*scene.frame("env/World/mount"))
```

::: botrail.Scene
