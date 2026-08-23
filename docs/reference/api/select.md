# Requirements and the check (`bt.select`)

What the cell asks of every bill-of-materials line, compared with what the
chosen part says — and every static check in one report. botrail derives and
compares; it does not choose. See
[Selecting parts](../../guides/selection.md).

```python
req = scene.requirements()                 # bt.select.requirements(scene)
print(req.to_markdown())
req["ur5e/tool"].minimum                   # {"payload_kg": 2.3, "stroke_mm": 150.0}

report = scene.check()                     # bt.select.check(scene)
assert report.ok, report.to_markdown()
```

::: botrail.select
