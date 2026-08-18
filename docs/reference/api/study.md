# Parameter studies (`bt.sweep`, `bt.optimize`)

Sweeps and deterministic optimisation over a cell authored as a function of
its parameters. `bt.sweep` and `bt.optimize` are the module's `sweep` and
`optimize`; the results are `bt.study.Sweep` and `bt.study.Optimum`. See the
[Parameter sweeps](../../tutorials/parameter-sweep.md) tutorial.

```python
result = bt.sweep(build_cell, grid={"velocity": [0.1, 0.2, 0.3], "lane_y": [0.4, 0.5, 0.6]},
                  metrics=lambda tl: {"cycle": tl.duration, "clearance": float(tl.min_clearance())})
print(result.pivot("lane_y", "velocity", "cycle"))
best = bt.optimize(build_cell, space={"velocity": (0.1, 0.4, 0.05), "lane_y": (0.3, 0.7, 0.05)},
                   objective="cycle", constraints={"clearance": (">=", 0.4)}, method="descent")
```

::: botrail.study
