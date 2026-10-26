# pick cell — cell report

| | |
|---|---|
| Robots | simple_arm (6 DOF, cabinet UR) |
| Cycle time | baseline: 11.35 s, ng_part: 12.22 s |
| Min clearance | 0.000 m at 6.16 s (baseline) |
| Footprint | 2.46 × 2.16 m (5.3 m²), height 1.80 m |
| I/O | 4 points (2 DI, 2 DO), 0 unbound, 0 finding(s) |
| BOM | 15 lines, 0 unidentified, cable_m 6, mass_kg 258 |
| Scenarios | 2/3 passed |
| Deliverables | 17 files hashed |

## Cycle `baseline`

Programs: `pick`. Duration **11.35 s**.

| robot | busy (s) | utilization |
|---|---|---|
| simple_arm | 7.08 | 62 % |

Min clearance 0.000 m at 6.16 s.

Branches taken: pick/judge → arm 0.

| step | start (s) | end (s) |
|---|---|---|
| feed | 0.00 | 1.19 |
| await part | 1.19 | 5.13 |
| halt | 5.13 | 5.13 |
| descend | 5.13 | 5.85 |
| grip | 5.85 | 6.15 |
| hold | 6.15 | 6.15 |
| judge | 6.15 | 6.15 |
| place | 6.15 | 9.09 |
| release | 9.09 | 9.72 |
| return | 9.72 | 11.35 |

## Cycle `ng_part` (scenario `ng_part`)

Programs: `pick`. Duration **12.22 s**.

| robot | busy (s) | utilization |
|---|---|---|
| simple_arm | 7.96 | 65 % |

Min clearance 0.000 m at 6.16 s.

Branches taken: pick/judge → arm 1.

| step | start (s) | end (s) |
|---|---|---|
| feed | 0.00 | 1.19 |
| await part | 1.19 | 5.13 |
| halt | 5.13 | 5.13 |
| descend | 5.13 | 5.85 |
| grip | 5.85 | 6.15 |
| hold | 6.15 | 6.15 |
| judge | 6.15 | 6.15 |
| to reject | 6.15 | 9.52 |
| drop | 9.52 | 10.70 |
| return | 10.70 | 12.22 |

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
| baseline | ok | 11.35 |
| ng_part | ok | 12.22 |
| beam_stuck | **failed** — timed out after 30s waiting in step 1 (`await part`) — forced: part_at_pick=false |  |

## Bill of materials

15 lines, 0 unidentified.

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
| structure.pedestal | 1 |
| structure.table | 2 |
| tray | 1 |

Totals: cable_m = 6, mass_kg = 258.


## Footprint

x -1.23 … 1.23 m, y -0.63 … 1.53 m — 2.46 × 2.16 m, 5.31 m², tallest item 1.80 m.


## Deliverables

| file | bytes | sha256 |
|---|---|---|
| ../../docs/assets/deliverables/cell.botrail | 61800 | cae01c0e5207ac5ea2b61973208f7528f897206374711b38086140668fa3bb8c |
| ../../docs/assets/deliverables/cell.py | 33380 | d4f2cd37c1f08fce4a17017ca10ff73cbeba5b4a8ca4cf74aba6507838faaa96 |
| ../../docs/assets/deliverables/cell_bom.csv | 1427 | 1017a569718b5e53fd657e869c112f4baa24499616d471fd2ec78f61808f737d |
| ../../docs/assets/deliverables/cell_bom.md | 2219 | 006d521078b714641ff392e267f698facc29b077c2bcce449026109819a51cd9 |
| ../../docs/assets/deliverables/cell_io.csv | 572 | 368d863f9c1c7193a1b707b3d1bc495760f5b4de99e187e50ffad28fa5d9d47c |
| ../../docs/assets/deliverables/cell_topology.mmd | 421 | a88549a0bb2f8ea31c57399127d4b6082016f4513b1cd58bb8bd6ff4618bd097 |
| ../../docs/assets/deliverables/cell.plcopen.xml | 14887 | 82f007a0c3c7447ba8120b0881667a138e58b327bd301148d5a5a3782af63282 |
| ../../docs/assets/deliverables/cell_interlocks.md | 1577 | 265301c3fd9f2d8e8ac86409ccb09058527a61f82c6c20125fdaf61aa8e1e0a4 |
| ../../docs/assets/deliverables/cell_layout.svg | 12781 | 5334e287fcf1d2a6886cc9b396b232ae17ad724195e1595a450bff7deb6f4d35 |
| ../../docs/assets/deliverables/cell_layout.dxf | 23178 | 5a84117234d5acc2aa39d445085ad80d79da291fe60a10eda6e479b191d6f802 |
| ../../docs/assets/deliverables/cell_cycle.usda | 1842276 | cd60893297a0bfb9d9c0a5ff8ba5f8e54866fe7162b19fd983140c46e489c3a2 |
| ../../docs/assets/deliverables/cell_cycle_assets/appearances/source_0/tray.usda | 152782 | 74cee1025277247ab14a5346cd4ac06c25a2160d348d1ba92339ea816f39fef1 |
| ../../docs/assets/deliverables/cell_cycle_assets/appearances/source_1/panel.usda | 24834 | 931b3abc41b25c875500c9d63708f3d15de02a33cbdad63a1300181920af53c7 |
| ../../docs/assets/deliverables/cell_cycle_assets/appearances/source_2/handle.usda | 171389 | c149c6d34736fe889230a17e1e53f70a50c470b96cd4dd65da5658de9961861b |
| ../../docs/assets/deliverables/cell_cycle_assets/appearances/source_3/tote.usda | 136637 | f71e1f939762b771e23303bfe61b8116e2c1b6d57d5a45891201f0fd517164b4 |
| ../../docs/assets/deliverables/cell_cycle_assets/meshes/lifting_eye_lf.usdc | 9726 | 1b8d932957ba1ec0ab8f2a922ff12e94785bc02edaed53f2e380b82e7d2bbab7 |
| ../../docs/assets/deliverables/pick_cell.script | 1877 | 82656f8024cc84975ae0a31ea9ceb484e9017ddb4a83e6d97eaf6ea0dfb9be47 |
