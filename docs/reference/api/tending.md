# Machine tending (`bt.tending`)

The machine's side of a tending handshake, authored as a program of its
own — FANUC Robot Interface 2 signalling, or a machine with no interface
worked through its panel. See [Machine tending](../../guides/machine-tending.md).

```python
vmc = bt.parts.machine_tool(scene, "vmc", door="servo")
hs = bt.tending.fanuc_ri2(scene, vmc, cycle_s=42.0)
tl = scene.simulate_sequences(["tend", hs.program])
```

::: botrail.tending
