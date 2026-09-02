# pick cell — cell report

| | |
|---|---|
| Robots | simple_arm (6 DOF) |
| Cycle time | baseline: 11.75 s, ng_part: 10.97 s |
| Min clearance | 0.239 m at 4.59 s (baseline) |
| Footprint | 2.46 × 1.66 m (4.1 m²), height 1.80 m |
| I/O | 4 points (2 DI, 2 DO), 0 unbound, 0 finding(s) |
| BOM | 11 lines, 0 unidentified, mass_kg 258 |
| Scenarios | 2/3 passed |
| Deliverables | 11 files hashed |

## Cycle `baseline`

Programs: `pick`. Duration **11.75 s**.

| robot | busy (s) | utilization |
|---|---|---|
| simple_arm | 9.38 | 80 % |

Min clearance 0.239 m at 4.59 s.

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

Min clearance 0.239 m at 4.59 s.

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

Totals: mass_kg = 258.


## Footprint

x -1.23 … 1.23 m, y -0.63 … 1.03 m — 2.46 × 1.66 m, 4.08 m², tallest item 1.80 m.


## Deliverables

| file | bytes | sha256 |
|---|---|---|
| ../../docs/assets/deliverables/cell.botrail | 28669 | 8b3e860e3d86b268544d2a6368aac87a72486c4eb8c9b94588e01e36363416c7 |
| ../../docs/assets/deliverables/cell.py | 13750 | e425ba7152ba78d1c79914ed9eb7973cd975c9af0a7c32d011d6f5c6846c9517 |
| ../../docs/assets/deliverables/cell_bom.csv | 1166 | 6b908a8a8fafee0dc907e3c6ebb565eefbdb693512f48c556aa7c7685f23f5a9 |
| ../../docs/assets/deliverables/cell_bom.md | 1717 | 7db9a68217597f7b9de052c82f11068db04bf25e111c4150640b8b3546dfe7e2 |
| ../../docs/assets/deliverables/cell_io.csv | 484 | f19f8a948e1778a41142209d68489783625ed9505d3804336050bfe1aad7fc2d |
| ../../docs/assets/deliverables/cell_topology.mmd | 421 | a88549a0bb2f8ea31c57399127d4b6082016f4513b1cd58bb8bd6ff4618bd097 |
| ../../docs/assets/deliverables/cell.plcopen.xml | 13834 | a4bb95528a67dacf382224c77626184a0a84e8d0f23b43babef5172710c5c685 |
| ../../docs/assets/deliverables/cell_layout.svg | 5442 | 8c83f6fe067cfc3b08f34c937f75b54bb2452f1c1acd7df3d54e65538d0767ae |
| ../../docs/assets/deliverables/cell_layout.dxf | 8586 | 4b1821cc07be3ba0e97ba9f68b46b794571f9d11709abc60a1378cbd3a9b954d |
| ../../docs/assets/deliverables/cell_cycle.usda | 593339 | d3a29a464c69fa00dfbf4973dd8411fdd4eadbb099ef4bbd014d58610626149a |
| ../../docs/assets/deliverables/pick_cell.script | 1209 | 1bd8db12bf396c71610900df19f1e8bf2d1dc6b22bf30811d2fcca7e5c4264d2 |
