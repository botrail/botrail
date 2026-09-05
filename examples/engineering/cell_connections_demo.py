"""Declared 24 V / 48 V feeds, an I/O terminal, air and a controller uplink.

All ratings are illustrative engineering inputs, not vendor specifications.
Run `botrail connections examples/engineering/cell_connections_demo.py` or
`botrail studio examples/engineering/cell_connections_demo.py`.
"""

import botrail as bt


def build(*, unknown_valve_current=False) -> bt.Scene:
    scene = bt.Scene()
    for name, voltage, capacity, x in (("PS24", 24, 2, -.4), ("PS48", 48, 5, -.1)):
        bt.parts.power_supply(scene, name, (x, .3, .05), size=(.15, .15, .25),
                              output_v=voltage, output_a=capacity, model=f"Example {voltage}V supply")
        bt.connections.port(scene, name + ".out", name, "power", "supply", terminal="+ / -",
                            reference="EXAMPLE-E01")

    scene.add_beam_sensor("eye", frm=(.2, -.3, .2), to=(.6, -.3, .2))
    scene.set_part("eye", model="Example photo-eye", voltage_v=24, current_a=.1, sensing_range_mm=500)
    scene.add_box("field/eye_body", size=(.04, .04, .08), position=(.2, -.3, .2), color=(.2, .5, .8))
    scene.add_box("field/drive", size=(.2, .18, .18), position=(.65, .3, .14), color=(.3, .4, .5))
    scene.set_part("field/drive", model="Example drive", voltage_v=48, current_a=2)
    scene.add_box("field/valve", size=(.15, .08, .08), position=(.2, .05, .09), color=(.4, .5, .6))
    scene.set_part("field/valve", model="Example valve", voltage_v=24,
                   **({} if unknown_valve_current else {"current_a": .35}))
    for target, source in (("eye", "PS24"), ("field/valve", "PS24"), ("field/drive", "PS48")):
        bt.connections.port(scene, target + ".power", target, "power", "load")
        bt.connections.connect(scene, source + ".out", target + ".power", cable="W-" + target,
                               reference="EXAMPLE-E01")

    scene.add_io_node("PLC", channels=bt.io.di16(voltage=24, logic="pnp"), model="Example PLC")
    scene.declare_io("eye", role="input", kind="di")
    scene.bind_input("eye", "PLC", "DI0")
    bt.connections.port(scene, "eye.output", "eye", "signal", "output",
                        signal_type="digital", voltage_v=24, logic="pnp")
    bt.connections.port(scene, "PLC.eye", "PLC", "signal", "input",
                        io={"point": "eye", "direction": "input", "node": "PLC"}, terminal="X2:1")
    bt.connections.connect(scene, "eye.output", "PLC.eye", cable="W-eye-signal")

    scene.add_io_node("RIO", kind="remote_io", uplink=("PLC", "EtherCAT"), model="Example remote I/O")
    for target in ("PLC", "RIO"):
        bt.connections.port(scene, target + ".net", target, "network", "peer", protocol="EtherCAT")
    bt.connections.connect(scene, "PLC.net", "RIO.net", cable="N-01")

    scene.add_box("utilities/air", size=(.12, .12, .2), position=(-.3, -.3, .15), color=(.3, .6, .7))
    scene.set_part("utilities/air", model="Example air service")
    bt.connections.port(scene, "air.out", "utilities/air", "pneumatic", "supply", pressure_bar=6,
                        capacity_l_min=100, flow_reference="ANR", reference="EXAMPLE-P01")
    bt.connections.port(scene, "valve.air", "field/valve", "pneumatic", "load", pressure_min_bar=4,
                        pressure_max_bar=7, flow_l_min=60, flow_reference="ANR")
    bt.connections.connect(scene, "air.out", "valve.air", cable="TUBE-01")
    return scene


if __name__ == "__main__":
    print(bt.connections.report(build()).to_markdown())
