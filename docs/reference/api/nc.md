# NC part programs (`bt.nc`)

A machine tool's axes as a mechanism, and a part program read the way the
machine's control runs it: against its work offsets and its tool table,
timed by its rates, refused by line where it cannot be read — and, with
the tools entered as bodies, cut: the stock carved, what a tool met named
by line, the part measured against its design, the setup written down. A
program that works the cell with M codes is the source of the machine's
own program (`MCode`, `sequence`). See
[Machine tending](../../guides/machine-tending.md#a-machine-that-runs-its-program)
and [NC machining](../../guides/nc-machining.md).

```python
vf2 = bt.parts.machine_tool(scene, "vf2", catalog="haas/vf/vf-2", axes=True)
bt.nc.set_work_offset(scene, vf2, "G54", at=part_top)
holder = bt.nc.Holder(diameter=0.040, length=0.075)
bt.nc.set_tool(scene, vf2, 1, bt.nc.end_mill(0.010, flutes=0.022, stickout=0.035, holder=holder))
bt.nc.program(scene, "op10", Path("op10.nc"))
run = bt.nc.check(scene, vf2, "op10", stock="part", design="part.stl")     # the dry run
print(run.summary(), run.machining.summary(), sep="\n")
hs = bt.tending.haas_mfin(scene, vf2, "op10", parts=["part", "blank"])  # the program runs the cell
tl = scene.simulate_sequences(["tend", hs.program])
shown, cut = bt.nc.animate(scene, tl, vf2, "part", design="part.stl")      # … cut, in the cell
```

::: botrail.nc
