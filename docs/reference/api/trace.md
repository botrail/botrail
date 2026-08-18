# Traces (`bt.trace`)

Controller I/O logs as traces, and the diff against a bake — the offline
commissioning check. See
[Offline commissioning](../../guides/offline-commissioning.md).

```python
trace = bt.trace.load("plc_log.csv", io=scene.io_map())
d = tl.diff(trace, tolerance=0.05, align_on="beam_pick")
assert d.ok, d.to_markdown()
```

::: botrail.trace
