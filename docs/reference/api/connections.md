# Equipment connections (`bt.connections`)

Which supply feeds which load, which valve feeds which actuator, which
controller input reads which sensor: ports declared on existing equipment,
the connections between them, and a report that checks the pairing. The
plan rides the `.botrail` project. See
[Equipment connections](../../guides/connections.md).

```python
import botrail as bt

cx = bt.connections
cx.port(scene, "24V", "psu", "power", "supply")
cx.port(scene, "eye.power", "eye", "power", "load")
cx.connect(scene, "24V", "eye.power", name="W1", cable="CBL-001")

report = cx.report(scene)
assert report.ready, report.to_markdown()
```

::: botrail.connections
