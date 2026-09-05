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

Studio renders each original USD gprim with its authored normals, UVs,
material subsets and supported PBR channels. The collision geometry remains
separate. Moving an obstacle moves its appearance with it; `enabled`,
`visible`, `walkable` and botrail material overrides remain scene state.
Imported appearances and their layer/image dependencies travel with saved
projects and exported USD asset directories.

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

## Robots from USD

To refine the appearance of an existing robot while keeping its validated
mechanics, install display shapes from a compatible USD model before attaching
tools:

```python
arm = bt.Robot.from_catalog("rv-5as-d")
arm = arm.with_visuals(bt.Robot.from_usd("rv-5as-d.usdc"))
```

Each visual link's final name segment must match exactly one existing link.
Link origins at zero joint coordinates must agree within 20 µm; differing
joint-frame orientations are compensated in the display transform. Unmatched
existing links retain their visuals. Joints, collision shapes, TCP, planning
groups and catalog identity remain those of `arm`. The display source travels
with projects, generated Python and USD exports. This is also how the machine
tending example's `--visual-dir` applies local RV-5AS-D and MPH-3 refinements.

Articulations load through [`Robot.from_usd`][botrail.Robot.from_usd] — see
the [Robots guide](robots.md#three-ways-in) for the details (prim-path names,
degree/unit conversion, `articulation_root`, `search_paths`). USD-sourced
robots keep a pointer to their stage, which the exporter uses to reference the
original asset at full visual fidelity.

Attaching a USD-sourced tool also retains each component's source gprims,
mapped to the combined robot's links. To use a catalog product's USD
appearance, select `Robot.from_catalog("product-id", format="usd")` for
that component. URDF remains the catalog default when available; choosing
USD does not add material information absent from the source package.

Appearance support follows Studio's USD loader, centered on
`UsdPreviewSurface`; arbitrary MDL/MaterialX networks are not guaranteed to
render identically. For portable assets, use relative layer and image
references, including paths such as `../textures/panel.png`. Preserve the
exported asset directory alongside the USD file. USD export of scene
overrides supports constant or directly texture-connected Preview Surface
inputs; unsupported override graphs produce an error.

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
