"""`bt.trace.read_mtconnect` / `to_mtconnect` — a machine tool's MTConnect
stream against the bake (T4 of design/design-machine-tending.md): the
standard's vocabulary (`Execution`, `DoorState`, `ChuckState`,
`EmergencyStop`) read as levels on the handshake's lanes, and the bake
written back out in the same vocabulary as the expected stream."""

from pathlib import Path

import botrail as bt
import pytest
from test_tending import cell

STREAM = """<?xml version="1.0" encoding="UTF-8"?>
<MTConnectStreams xmlns="urn:mtconnect.org:MTConnectStreams:2.0">
<Header creationTime="2026-09-04T00:00:00Z" sender="agent" instanceId="1" version="2.0.0" bufferSize="1024"
        nextSequence="9" firstSequence="1" lastSequence="8"/>
<Streams><DeviceStream name="vmc" uuid="vmc-1"><ComponentStream component="Controller" name="cont" componentId="c">
<Events>
<Execution dataItemId="exec" timestamp="2026-09-04T10:00:00.000Z" sequence="1">UNAVAILABLE</Execution>
<Execution dataItemId="exec" timestamp="2026-09-04T10:00:00.500Z" sequence="2">ACTIVE</Execution>
<DoorState dataItemId="door" name="side_door" timestamp="2026-09-04T10:00:00.500Z" sequence="3">CLOSED</DoorState>
<EmergencyStop dataItemId="estop" timestamp="2026-09-04T10:00:00.500Z" sequence="4">ARMED</EmergencyStop>
<Execution dataItemId="exec" timestamp="2026-09-04T10:00:03.000Z" sequence="5">READY</Execution>
<DoorState dataItemId="door" name="side_door" timestamp="2026-09-04T10:00:03.200Z" sequence="6">UNLATCHED</DoorState>
<DoorState dataItemId="door" name="side_door" timestamp="2026-09-04T10:00:04.000Z" sequence="7">OPEN</DoorState>
<EmergencyStop dataItemId="estop" timestamp="2026-09-04T10:00:05.000Z" sequence="8">TRIGGERED</EmergencyStop>
</Events></ComponentStream></DeviceStream></Streams></MTConnectStreams>
"""


def test_events_read_as_levels_by_id_name_or_type(tmp_path: Path) -> None:
    # Keys may be a data item id (`exec`), its name (`side_door`) or its
    # type (`EmergencyStop`); a door is two lanes; UNAVAILABLE is skipped;
    # the clock starts at the first observation kept.
    items = {"exec": "vmc/running", "side_door": ("vmc/side_door/closed", "vmc/side_door/open"),
             "EmergencyStop": "vmc/panel/estop"}
    trace = bt.trace.read_mtconnect(STREAM, items)
    assert trace.names == ["vmc/panel/estop", "vmc/running", "vmc/side_door/closed", "vmc/side_door/open"]
    assert trace.signals["vmc/running"] == [(0.0, True), (2.5, False)]
    assert trace.signals["vmc/side_door/closed"] == [(0.0, True), (2.7, False), (3.5, False)]
    assert trace.signals["vmc/side_door/open"] == [(0.0, False), (2.7, False), (3.5, True)]
    assert trace.edges("vmc/panel/estop") == ([4.5], [])
    # `t0` pins the clock: an ISO stamp, or the seconds the first sample sits at.
    assert bt.trace.read_mtconnect(STREAM, items, t0="2026-09-04T10:00:00Z").signals["vmc/running"][0] == (0.5, True)
    assert bt.trace.read_mtconnect(STREAM, items, t0=10.0).signals["vmc/running"][0] == (10.0, True)
    # A file works as well as text; nothing matched is an empty trace.
    path = tmp_path / "sample.xml"
    path.write_text(STREAM)
    assert bt.trace.read_mtconnect(path, items).names == trace.names
    assert bt.trace.read_mtconnect(STREAM, {"nothing": "x"}).names == []


def test_the_bake_round_trips_through_the_standard_vocabulary() -> None:
    scene, vmc = cell()
    hs = bt.tending.fanuc_ri2(scene, vmc, cycle_s=3.0, notice_s=1.0, clamp_s=0.2)
    sq = scene.sequence("tend")
    sq.step("wait", transition=bt.seq.signal(hs.signal("service_req")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("leave", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False)], transition=bt.seq.elapsed(0.5))
    sq.step("ok", actions=[bt.seq.set_signal(hs.signal("service_ok"))], transition=bt.seq.signal(hs.signal("door_closed")))
    sq.step("home", actions=[bt.seq.set_signal(hs.signal("service_ok"), False)])
    tl = scene.simulate_sequences(["tend", "vmc"])
    items = hs.mtconnect_items()
    assert items == {"Execution": "vmc/running", "DoorState": ("vmc/side_door/closed", "vmc/side_door/open"),
                     "EmergencyStop": "vmc/panel/estop", "ChuckState": "vmc/clamp"}
    xml = bt.trace.to_mtconnect(tl, items, start="2026-09-04T00:00:00Z", device="vmc")
    # The door is written three-valued: closed, unlatched while moving, open.
    assert xml.count("<DoorState") >= 4 and ">UNLATCHED<" in xml and ">OPEN<" in xml and ">CLOSED<" in xml
    assert '<Execution dataItemId="Execution" timestamp="2026-09-04T00:00:00Z" sequence=' in xml
    # The perfect stream diffs clean; a door that never reports CLOSED again
    # is a missing edge, by name and second.
    assert tl.diff(bt.trace.read_mtconnect(xml, items)).ok
    late = xml.replace(">CLOSED<", ">UNLATCHED<", 1)
    d = tl.diff(bt.trace.read_mtconnect(late, items))
    assert not d.ok and "vmc/side_door/closed" in d.to_markdown()
    with pytest.raises(ValueError):
        tl.diff(bt.trace.read_mtconnect(xml, {"Execution": "vmc/running"}), signals=["vmc/clamp"])
