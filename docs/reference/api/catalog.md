# Catalog search (`bt.catalog`)

Real products to choose from: the catalog's published index, filtered the
way a requirement reads. See [Selecting parts](../../guides/selection.md).

```python
cands = bt.catalog.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)
cands[0].identify(scene, "ur5e/tool")
bt.catalog.search_for(scene.requirements()["eye"], level="V2")
```

::: botrail.catalog
