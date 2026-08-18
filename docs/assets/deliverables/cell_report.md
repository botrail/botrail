# pick cell — cell report

| | |
|---|---|
| Robots | simple_arm (6 DOF) |
| Cycle time | baseline: 11.75 s, ng_part: 10.97 s |
| Min clearance | 0.239 m at 4.59 s (baseline) |
| Footprint | 2.46 × 1.66 m (4.1 m²), height 1.80 m |
| I/O | 4 points (2 DI, 2 DO), 0 unbound, 0 finding(s) |
| BOM | 8 lines, 0 unidentified, mass_kg 144 |
| Scenarios | 2/3 passed |
| Deliverables | 10 files hashed |

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

8 lines, 0 unidentified.

| category | qty |
|---|---|
| bin | 1 |
| conveyor | 1 |
| robot | 1 |
| robot_controller | 1 |
| sensor.photoelectric | 1 |
| structure.door | 1 |
| structure.fence | 7 |
| structure.fence.post | 8 |

Totals: mass_kg = 144.


## Footprint

x -1.23 … 1.23 m, y -0.63 … 1.03 m — 2.46 × 1.66 m, 4.08 m², tallest item 1.80 m.


## Deliverables

| file | bytes | sha256 |
|---|---|---|
| docs/assets/deliverables/cell.botrail | 24534 | 17c80b6294e63418d2a34d2e61bb9ad8ca09ae8d4dc25935e6861adf6229f7b6 |
| docs/assets/deliverables/cell.py | 11544 | 96e008c3797bcbca4eb027de9e0cb76bfeb875d5fb9f67fc282f8ba53a8ad6fe |
| docs/assets/deliverables/cell_bom.csv | 412 | 6a14b49558a47f6aa8ff8479463b241df8c240cb2e78a674ba5c481fa447a18f |
| docs/assets/deliverables/cell_bom.md | 671 | dfcb2a209816105a5972e00f65f2a198c8554b1fc8553082d09af0fec5a69f7f |
| docs/assets/deliverables/cell_io.csv | 484 | f19f8a948e1778a41142209d68489783625ed9505d3804336050bfe1aad7fc2d |
| docs/assets/deliverables/cell_topology.mmd | 421 | a88549a0bb2f8ea31c57399127d4b6082016f4513b1cd58bb8bd6ff4618bd097 |
| docs/assets/deliverables/cell_layout.svg | 4818 | ce23b533db392f7d6613687b481d9c6247d96541be2994ec80706d71a4b0c0ca |
| docs/assets/deliverables/cell_layout.dxf | 7374 | 6239a3debcf902346d00dba17eb5010f97303a10847f6405b884decc5c249e93 |
| docs/assets/deliverables/cell_cycle.usda | 590838 | dd8071f69704c9aa97778761fa6bb2cc1bb023d24e2edc18929923a32ac58b41 |
| docs/assets/deliverables/pick_cell.script | 1209 | 1bd8db12bf396c71610900df19f1e8bf2d1dc6b22bf30811d2fcca7e5c4264d2 |
