# Cloth (`bt.cloth`)

A T-shirt or a sheet on a table, handled by the robots' grippers and simulated
against the baked cycle. See the [cloth guide](../../guides/cloth.md) for what
it answers and how a fold is taught.

```python
shirt = bt.cloth.tshirt("shirt", on="table/top")
marks = bt.cloth.landmarks(scene, shirt)
bt.cloth.add(scene, shirt, grippers=[bt.cloth.gripper("grip", radius=0.05)])
timeline = scene.simulate_sequence("fold")
timeline.cloth("shirt").position("cuff_left", timeline.duration)
```

## Reference

::: botrail.cloth
    options:
      # Pure Python, so static analysis works here and keeps the source order.
      force_inspection: false
      members_order: source

## ClothTrack

Returned by `SequenceTimeline.cloth(name)`: one cloth over the baked cycle.

::: botrail.ClothTrack
