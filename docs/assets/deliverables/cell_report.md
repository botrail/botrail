# pick cell — cell report

| | |
|---|---|
| Robots | simple_arm (6 DOF, cabinet UR) |
| Cycle time | baseline: 11.75 s, ng_part: 10.97 s |
| Min clearance | 0.175 m at 0.00 s (baseline) |
| Footprint | 2.46 × 1.66 m (4.1 m²), height 1.80 m |
| I/O | 4 points (2 DI, 2 DO), 0 unbound, 0 finding(s) |
| BOM | 11 lines, 0 unidentified, cable_m 6, mass_kg 258 |
| Scenarios | 2/3 passed |
| Deliverables | 12 files hashed |

## Cycle `baseline`

Programs: `pick`. Duration **11.75 s**.

| robot | busy (s) | utilization |
|---|---|---|
| simple_arm | 9.38 | 80 % |

Min clearance 0.175 m at 0.00 s.

Branches taken: pick/judge → arm 0.

| step | start (s) | end (s) |
|---|---|---|
| feed | 0.00 | 3.49 |
| await part | 3.49 | 5.55 |
| halt | 5.55 | 5.55 |
| grip | 5.55 | 5.85 |
| hold | 5.85 | 5.85 |
| judge | 5.85 | 5.85 |
| place | 5.85 | 7.99 |
| release | 7.99 | 7.99 |
| return | 7.99 | 11.75 |

## Cycle `ng_part` (scenario `ng_part`)

Programs: `pick`. Duration **10.97 s**.

| robot | busy (s) | utilization |
|---|---|---|
| simple_arm | 8.29 | 76 % |

Min clearance 0.175 m at 0.00 s.

Branches taken: pick/judge → arm 1.

| step | start (s) | end (s) |
|---|---|---|
| feed | 0.00 | 3.49 |
| await part | 3.49 | 5.55 |
| halt | 5.55 | 5.55 |
| grip | 5.55 | 5.85 |
| hold | 5.85 | 5.85 |
| judge | 5.85 | 5.85 |
| to chute | 5.85 | 6.69 |
| drop | 6.69 | 6.99 |
| return | 6.99 | 10.97 |

## I/O

4 points over `pick`: 4 bound, 0 unbound, 0 internal, 0 safety.

| kind | points |
|---|---|
| DI | 2 |
| DO | 2 |

| node | kind | bound / channels |
|---|---|---|
| UR | robot_controller | 4 / 16 |


## Scenarios

| scenario | result | cycle (s) |
|---|---|---|
| baseline | ok | 11.75 |
| ng_part | ok | 10.97 |
| beam_stuck | **failed** — timed out after 30s waiting in step 1 (`await part`) — forced: part_at_pick=false |  |

## Bill of materials

11 lines, 0 unidentified.

| category | qty |
|---|---|
| bin | 1 |
| conveyor | 1 |
| robot | 1 |
| robot_controller | 1 |
| sensor.photoelectric | 1 |
| structure.cabinet | 1 |
| structure.cabinet.base | 1 |
| structure.cabinet.plate | 1 |
| structure.door | 1 |
| structure.fence | 7 |
| structure.fence.post | 8 |

Totals: cable_m = 6, mass_kg = 258.


## Footprint

x -1.23 … 1.23 m, y -0.63 … 1.03 m — 2.46 × 1.66 m, 4.08 m², tallest item 1.80 m.


## Deliverables

| file | bytes | sha256 |
|---|---|---|
| ../../docs/assets/deliverables/cell.botrail | 6600 | bce422b887d9d4a5927473249d7f11d1b301dc9767d2c2a1d1cf7a7b4c3623dd |
| ../../docs/assets/deliverables/cell.py | 21199 | 0c1c698449f5213f29b7e8ca319bb864418b604b64e6fc934bf77d12758d2b71 |
| ../../docs/assets/deliverables/cell_bom.csv | 1208 | 00ee01ac073385158d6881b43663d3bbb21199445c6138de7e2333f698b181d9 |
| ../../docs/assets/deliverables/cell_bom.md | 1828 | 403cf7b6469b92022341d44a27996a31a999f828f4d0506c159304e2c995bd30 |
| ../../docs/assets/deliverables/cell_io.csv | 572 | 368d863f9c1c7193a1b707b3d1bc495760f5b4de99e187e50ffad28fa5d9d47c |
| ../../docs/assets/deliverables/cell_topology.mmd | 421 | a88549a0bb2f8ea31c57399127d4b6082016f4513b1cd58bb8bd6ff4618bd097 |
| ../../docs/assets/deliverables/cell.plcopen.xml | 13834 | 1f120e1b7647852bf774e30f01fe28001e0c37b3f5077d4fcba38eaccda31e4c |
| ../../docs/assets/deliverables/cell_interlocks.md | 1305 | 38251f69527bea4d4b6cb5953a63a365062f112bc35db0ec10128a64f07cfdfb |
| ../../docs/assets/deliverables/cell_layout.svg | 7623 | 332e3eb1c6a4c633549c0c1d3a26e9eb1e5425fed8b6016b1132bb256a6468f4 |
| ../../docs/assets/deliverables/cell_layout.dxf | 12810 | 8d45398712928abbd737885915590948b076fec60be98858d322ddd9f529621f |
| ../../docs/assets/deliverables/cell_cycle.usda | 826009 | 6f232e08b1bfa98d2eeeb2f7d17619a38ad95af908cf9673def8f2c721d494c5 |
| ../../docs/assets/deliverables/pick_cell.script | 1209 | 1bd8db12bf396c71610900df19f1e8bf2d1dc6b22bf30811d2fcca7e5c4264d2 |
