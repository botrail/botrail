# Scene and obstacles

The [`Scene`][botrail.Scene] is the cell. Everything in it shares three
conventions: lengths are **meters**, the world is **Z-up**, and orientations
are quaternions in **`(x, y, z, w)`** order. Wherever a method takes
`quaternion=None`, identity is assumed; wherever it takes `robot=None`, the
scene's first robot is meant.

## Primitives

```python
scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))
scene.add_sphere("dome", radius=0.1, position=(0.0, 0.5, 0.3))
scene.add_cylinder("post", radius=0.04, length=0.8, position=(0.5, 0.5, 0.4))
```

Box `size` is full extents; cylinders follow the URDF convention (axis along
local +z). Every `add_*` returns the final name — a taken name is uniquified
rather than rejected, so use the return value if you generate names in a loop.

## Meshes

```python
scene.add_mesh("fixture", "fixture.stl", position=(0.5, 0.0, 0.0),
               scale=(0.001, 0.001, 0.001))   # a mm-unit STL
```

STL and OBJ. The studio renders the original mesh; the **collision shape is a
VHACD convex decomposition**, computed on first load (about a second per mesh)
and cached on disk — subsequent runs are instant. See
[Collision checking](collision.md) for why.

## Posing, recoloring, removing

```python
scene.set_obstacle_pose("table", (0.5, 0.0, 0.0))
scene.set_obstacle_color("table", (0.8, 0.2, 0.2))   # linear RGB, display only
scene.obstacle_pose("table")                          # ((x,y,z), (x,y,z,w))
scene.obstacle_names
scene.remove_obstacle("table")
```

Two switches are worth knowing:

```python
scene.set_obstacle_enabled("cleat_3", False)   # out of collision, still rendered
```

Disabled obstacles keep rendering and keep riding conveyors — they are scenery
that happens to move. The dual-arm demo's belt cleats work exactly this way.

## Named frames

A frame is a named pose — a mount point, a teach point, a fixture datum:

```python
scene.add_frame("fixture_datum", (0.5, 0.2, 0.1))
scene.frames                    # {name: ((x,y,z), (x,y,z,w))}
scene.frame("fixture_datum")    # one pose, unpackable:
scene.set_robot_base_pose(*scene.frame("/World/MountFrame"))
```

Frames mostly arrive from [USD import](usd-import.md) — every leaf Xform in
the stage becomes one — which is what lets a layout file carry its own mount
and teach points.

## The scene is live

Everything above is mirrored to any connected studio immediately, and edits
made in the studio land back in this object. There is one scene; Python and
the browser are two views of it. `bt.studio(scene, block=False)` keeps your
prompt while you work from both sides.

## Where the rest lives

| Cell ingredient | Guide |
| --- | --- |
| USD stages as environments | [USD import](usd-import.md) |
| Collision queries and the ACM | [Collision checking](collision.md) |
| Motions and constraints | [Motion planning](motion-planning.md) |
| Sensors, conveyors, sources, axes | [Sensors and devices](sensors-and-devices.md) |
| Grasping and conveyor tracking | [Attach and tracking](attach-and-tracking.md) |
| Saving the whole cell | [Projects](projects.md) |
