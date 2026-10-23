# SimReady Central (`bt.simready`)

NVIDIA's open library of simulation-ready USD assets as botrail scenery:
one package to disk at its commit, with its license and publisher, for
[`bt.parts.simready`][botrail.parts.simready] to stand in a cell. See
[USD import](../../guides/usd-import.md#simready-central-assets).

```python
pkg = bt.simready.fetch("https://huggingface.co/datasets/nvidia/simready-assets/"
                        "blob/653d91e182011f357cf1644013cc94ddf194d349/box_cardboard_a01/"
                        "simready_usd/sm_box_cardboard_a01_01.usd")
bt.parts.simready(scene, "carton", (1.2, 0.4), pkg, dynamic=True)
pkg.license, pkg.source                      # 'CC-BY-4.0', the URL at its commit
```

::: botrail.simready
