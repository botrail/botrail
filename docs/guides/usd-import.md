# USD import

botrail's environment format is USD. A stage becomes obstacles and named
frames in one call; a `UsdPhysics` articulation becomes a robot. Both are
normalized on the way in, so downstream code never thinks about units or up
axes again.

## Stages as environments

```python
names = scene.load_usd("cell.usda", prefix="env")
```

* **Formats**: `usda`, `usdc`, `usdz`. References, variants, and instancing
  are composed — you get the flattened result of the stage as authored.
* **Normalization**: everything arrives in **meters, Z-up**, whatever the
  stage's `metersPerUnit` and up axis say.
* **What imports**: the *static geometry*, as obstacles. This is deliberate —
  behavior (belts that run, sensors that trip) is authored in botrail, where
  it can be simulated deterministically. See
  [Sensors and devices](sensors-and-devices.md).
* **Names**: prim paths, optionally prefixed (`env/World/Table`). The call
  returns the added obstacle names.

The studio draws each imported prim as authored — normals, UVs, material
subsets and textures — while collision keeps its own decomposed geometry.
Moving the obstacle moves its appearance with it; `enabled`, `visible`,
`walkable` and a `set_obstacle_material` override stay scene state on top.
The appearance and the layers and images it references travel with a saved
project and with an exported USD's asset directory.

## Frames come along for free

Leaf `Xform`/`Scope` prims become named frames — poses with no geometry. This
is the load-bearing feature: author your mount points and teach points *into
the layout file*, and the cell logic reads them by name.

```python
scene.load_usd("factory.usda")
scene.set_robot_base_pose(*scene.frame("/World/MountFrame"))

pick  = scene.frame("/World/Conveyor/PickFrame")
place = scene.frame("/World/Pallet/PlaceFrame")
```

Move the pedestal prim in the USD and the robot moves with it; move
`PickFrame` and the pick re-teaches itself. The
[tutorial cells](../tutorials/pose-and-plan.md) run entirely on this pattern.

## Cameras come along too

A `Camera` prim in the stage becomes a [camera](sensors-and-devices.md#cameras)
of the scene — a world fixture named like the prim, standing where the prim
stands, with the authored optics: the film back (`focalLength` /
`horizontalAperture`) is the horizontal field of view, the aperture aspect
the image aspect, `clippingRange` the near and far clip in meters. USD
cameras carry no pixel count; a stage botrail exported says it in
`botrail:resolution` and imports back pixel-exact, anything else gets a
1280-pixel width and the aperture aspect. Omniverse's far clips of ten
million meters are capped at 100 m, with an import notice.

```python
scene.load_usd("cell.usda", prefix="env")
scene.camera_names            # ['env/World/Overview', ...]
scene.remove_camera("env/World/Overview")   # scene state like any other
```

A camera Omniverse hid with `visibility = "invisible"` still imports — that
only hides its gizmo, the camera still films. Orthographic cameras are
skipped with a notice; botrail's cameras are pinhole.

## Robots from USD

Articulations load through [`Robot.from_usd`][botrail.Robot.from_usd] — see
the [Robots guide](robots.md#three-ways-in) for the details (prim-path names,
degree/unit conversion, `articulation_root`, `search_paths`). USD-sourced
robots keep a pointer to their stage, which the exporter uses to reference the
original asset at full visual fidelity. A `Camera` prim under one of its
rigid bodies — an Isaac asset's head or wrist camera — is the robot's own:
adding the robot mounts it on that link as `<robot>/<prim name>`, riding the
joints, without the generic housing (the robot's geometry is the housing).
Remove one you do not want; the project and the generated script keep it
removed.

A tool that came from USD keeps its authored prims after `attach_tool`,
mapped onto the composite's links; `Robot.from_catalog(id, format="usd")`
picks a catalog product's USD where the package ships one. Materials are
read as `UsdPreviewSurface` — MDL or MaterialX networks will not look the
same — and layer and image references should be relative
(`../textures/panel.png`) so the asset directory an export writes next to
the layer stays portable.

A stage with rigid bodies but no physics joints — a coupling, a fingertip, a
static fixture — imports too: the bodies weld together at their stage poses
and the model comes out with zero DOF, a perfectly good scene citizen for
collision checking and [tool mounting](robots.md#mounting-a-tool).

## Round trip

USD is also how animation leaves and re-enters botrail:

```python
tl.export_usd("cycle.usda", fps=60)          # bake a cycle out
scene.play_usd_animation("cycle.usda")       # play a recording back
```

The exported layer plays in usdview, Omniverse, or Blender with no botrail
installed; recordings — botrail's own, or an Isaac Sim capture — play back
into the studio through the same pipeline. Details in
[Export](export.md) and the
[Export and replay USD](../tutorials/replay-usd.md) tutorial.

!!! tip "Isaac Sim assets work as-is"

    The examples load NVIDIA's official Franka USD unmodified, and the factory
    cell is a hand-authored `usda` layer. If you already have Omniverse
    assets, they are your botrail cells too.
