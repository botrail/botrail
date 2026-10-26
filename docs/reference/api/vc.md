# Virtual commissioning (`bt.vc`)

The cell as the plant of a controller outside it: the exchange derived
from the I/O map, a plant session driven through it, loopback and replay,
and the Modbus / OPC UA links to a controller on the wire. See
[Virtual commissioning](../../guides/virtual-commissioning.md) for the
guide and [Offline commissioning](../../guides/offline-commissioning.md)
for the file-based half.

```python
xc = bt.vc.exchange(scene, "plc1")                 # what the controller writes and reads
with bt.vc.loopback(scene, ["pick"], host="plc1") as s:
    s.run()
    assert s.timeline().duration == tl.duration     # the bake, bit for bit
    rec = s.recording()                             # every input, with its time

cases = [bt.vc.TestCase("beam open", faults=[bt.io.open("beam")],       # the FAT: tests as a table
                        expect=[bt.vc.never("vacuum", during=10.0)]),
         *bt.vc.interlock_tests(scene, "pick"), *bt.vc.safety_tests(scene)]
fat = bt.vc.run_fat(scene, cases)                   # offline; links={"plc1": link} for a PLC
fat.save("cell_fat.md"); assert fat.ok

link = bt.vc.ursim(bt.vc.exchange(scene, "UR"), robot="arm")    # the robot controller in the loop (URSim, RTDE)
uop = bt.vc.interfaces.fanuc_uop_pns(programs={"pick": 1})      # a vendor interface as data
uop.apply(scene, "arm", "RC1")

bt.parts.cylinder(scene, "press", (0.2, 0, 0.55), axis=(0, 0, -1), stroke=0.03, solenoid="double")
scene.add_linear_axis("feed", ["strip"], axis=(1, 0, 0), speed=0.25, range=(0, 0.4), encoder=True)
scene.add_io_node("PLC1", kind="plc", programs=["index"],
                  channels=bt.io.di8() + bt.io.do8() + bt.io.word(4, base="%QW100", unit="mm"))
xc = bt.vc.exchange(scene, "PLC1")                 # press/extend, press/retract, press/extended, press/retracted,
                                                   # feed.position (INT, mm), feed (done), feed.encoder (INT, mm)
tl.word("feed.encoder").max()                      # the word lanes the plant records
s.word("feed.encoder")                             # … and reads live
```

::: botrail.vc

## Interfaces (`bt.vc.interfaces`)

::: botrail.vc.interfaces
