# Examples

Every demo is a runnable script: `python examples/<group>/<name>.py` from the
repository root. Most open the studio in the browser; the ones that take an
output path bake a USD recording instead. Demos that order equipment from the
catalog fetch it from the Hugging Face dataset on first run (cached after).
The `.usdc` files sitting beside some demos are pre-baked recordings of them.

| Group | Contents |
| --- | --- |
| `assets/` | Shared assets: `factory.usda` (the factory cell every basic demo loads), `simple_arm.urdf`, the offline `biped_test.urdf` / `quad_test.urdf` walkers, spray gun and hood meshes. |
| `basics/` | Start here. `demo.py` — Franka + factory cell: pose, plan, play. `sequence_demo.py` — 13-step cell: conveyor feed → tracked pick. `sfc_chart_demo.py` — the sequence as an SFC chart. `sweep_demo.py` — parameter sweep: belt speed × lane position. |
| `export/` | `export_urscript.py` — a pick cell exported as URScript with its I/O list. `export_animation.py` — bake the demo cell into an animated USD. `play_record.py` — replay a baked `.usdc` recording inside a rebuilt cell. |
| `welding/` | `weld_station_demo.py` — one spot-weld station. `weld_line_demo.py` — the four-station body-in-white line. `line_balance_sweep.py` — move a spot between stations and watch the takt. |
| `machining/` | `machining_demo.py` — robot milling with staged stock removal. `machine_tending_demo.py` — a cobot tending a machining centre by hand: a three-tool bracket (gripper, pin, fork) slides the side door, swaps the part in the vise and presses the panel's buttons; writes the hand-over set (interlock table, PLCopen with the CNC's own resource, FAT scenarios in the report); `--catalog` orders the machine and the vise from their packs. `two_machine_cell_demo.py` — one arm between two machining centres, taught with the same code prefixed per machine: utilization of the arm and of each spindle, three programs on the interlock table. |
| `painting/` | `painting_demo.py` — spray cell basics. `painting_hood_demo.py` — coating a hood section mesh. |
| `multi_robot/` | `dual_cell_demo.py` — two arms sharing one infeed, arbitrated by interlocks. `dual_arm_demo.py` — one robot with two arms (two UR5e on a torso): a kitting cell with a contested bin, a handover, a two-handed carry, and a 6-axis URScript per arm. |
| `vehicles/` | `agv_cell_demo.py` — an AGV crossing the factory cell. `agv_sweep_demo.py` — sweeping its variants. `amr_demo.py` — a mobile manipulator assembled from catalog items. `lift_demo.py` — an AMR riding an elevator between floors. |
| `legged/` | `legged_patrol_demo.py` — quadruped patrol. `humanoid_carry_demo.py` — humanoid carry. `stairs_delivery_demo.py` — a quadruped climbing catalog stairs. `building_delivery_demo.py` — the same dog delivering B1F→5F through a six-storey building, on the stairs, with the lift unused. |
| `drone/` | `drone_survey_demo.py` — warehouse cell: UR12e case palletizing beside a drone cycle-counting the racks. |
| `engineering/` | `cell_deliverables_demo.py` — the whole document set derived from one cell source. `equipment_cell_demo.py` — fence, conveyor and rack ordered from the catalog. |

Demos that build on another one (`sequence_demo` on `demo`, `agv_cell_demo` on
the factory cell, `stairs_delivery_demo` and `building_delivery_demo` on the
patrol robot) put the sibling
group on `sys.path` themselves, so each script also runs standalone from any
working directory.
