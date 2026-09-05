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

Colour says what a surface *is*; a material says how it takes light, and the
two are separate — bare steel and a painted panel can share a grey and still
look nothing alike:

```python
scene.set_obstacle_material("panel", metalness=0.85, roughness=0.42)  # bare steel
scene.set_obstacle_material("cabinet", metalness=0.0, roughness=0.55)  # paint
scene.set_obstacle_material("window", metalness=0.0, roughness=0.16, opacity=0.24)
scene.set_obstacle_material("panel")                  # back to the viewer's choice
scene.obstacle_material("panel")                      # (metalness, roughness) | None
```

`opacity` is an optional value from 0 (transparent) to 1 (opaque). It survives
project save/load, generated Python and USD export. `obstacle_opacity(name)`
reads the override; `obstacle_material(name)` retains its `(metalness, roughness)`
return value. This models a thin transparent cover without refraction. It
does not change collisions or sensor detection.

Both knobs use the 0–1 metallic/roughness convention of glTF, USD Preview
Surface and three.js. Studio applies them to primitives and meshes. Colour
changes and collision highlighting preserve the mesh's other material
channels, including textures. An explicit metallic/roughness pair converts
legacy OBJ/MTL shading to PBR while keeping its shared surface channels;
MTL shininess and specular maps have no direct equivalent in that workflow.

Finishes survive project save/load, generated Python, and USD export as
`UsdPreviewSurface` metallic/roughness inputs. Imported USD meshes retain
their original normals, UVs, material subsets and textures for display;
an explicit scene finish overrides their metallic/roughness values. Clear
the override to restore the source material. Metal is what
makes an unpainted body read as *metal* rather than as grey plastic: it
reflects its surroundings instead of carrying a diffuse colour of its own.
Appearance never touches collision or planning.

Two switches are worth knowing:

```python
scene.set_obstacle_enabled("cleat_3", False)   # out of collision, still rendered
scene.set_obstacle_visible("proxy_7", False)   # still collides, not rendered
```

The two are independent, and the pair is what lets a workpiece carry both a
display mesh and its own collision shape. Convex decomposition fills a body
shell's door and window apertures — and a welding gun works *through* those
— so a catalog `workpiece` ships a display shell alongside a set of authored
convex pieces that keep the openings open. Load the shell with collision off
and the pieces with rendering off, and the scene both looks right and
collides right:

```python
scene.add_mesh("body/shell", "…/visual/biw.obj", (0, 0, 0.78))
scene.set_obstacle_enabled("body/shell", False)      # looks right
for piece in pieces:                                  # …collides right
    scene.add_mesh(f"body/{piece}", f"…/collision/{piece}.stl", (0, 0, 0.78))
    scene.set_obstacle_visible(f"body/{piece}", False)
```

Disabled obstacles keep rendering and keep riding conveyors — they are scenery
that happens to move. The dual-arm demo's belt cleats work exactly this way.

!!! note "z = 0 is the floor"

    The studio draws the shop floor at `z = 0`, so geometry below it is
    behind the floor and never appears — a cell laid out around a robot
    base at the origin looks half-missing. Build upward instead: floor at
    zero, the robot's mounting plane on top of its pedestal
    (`bt.Scene(robot, base_position=(0, 0, 0.74))`), everything else
    measured from there.

Seating a workpiece on a fixture wants the mesh's own dimensions, not a number
measured off it once. [`obstacle_bounds`][botrail.Scene.obstacle_bounds]
returns the world-frame `(min, max)` of anything already in the scene, so a
cell can ask where the underside is and lift it onto the pallet:

```python
low, high = scene.obstacle_bounds("body/floor_pan")
scene.set_obstacle_pose("body/floor_pan", (0, 0, PALLET_TOP - low[2]))
```

That keeps the cell correct when the asset is rebuilt — the trap being that a
hard-coded lift is silently wrong the day the mesh's origin moves, and a
workpiece a centimetre into its fixture reads as a permanent collision.

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
