# Commercial tooling in the machine-tending demo

The UR12e carries a Robotiq Hand-E with its documented ES-077 coupling. It
grips the workpiece and the vertical door handle directly. Three Robotiq
Button Activators stay on the machine's panel. The former botrail MPH-3
bracket, lateral push pin and door fork are removed.

This follows the manufacturer's [Machine Tending Solution](https://robotiq.com/solutions/machine-tending).
It is a simulation using commercial components, not a claim that the entire
cell is a standard Robotiq kit or a commissioned installation.

| Component | Selected identity | Representation |
|---|---|---|
| Arm | Universal Robots UR12e, `universal_robots/ur/ur12e/r1` | Existing catalog robot |
| Hand and coupling | `robotiq/hand-e/hand-e-ur-es-077-kit/r1` | Existing kit, including GRP-ES-CPL-077; selected female-wrist route |
| Fingers | HND-FIN-MLD-KIT | Existing Hand-E model, 50 mm maximum opening |
| Panel pushers | Robotiq Button Activator × 3 | Independently authored runtime reference; exact order number and mounting dimensions unverified |

The manufacturer documents the solution for Universal Robots. The demo uses
a UR12e in place of the previous Mitsubishi ASSISTA to give the shorter
Hand-E enough reach while keeping the wrist away from its folded region.
Robotiq confirms [UR12e compatibility with UR10e solutions](https://blog.robotiq.com/knowledge/understanding-the-difference-between-ur10e-and-ur12e).
The stand is 600 mm outside the entry frame; the stocker is 1150 mm outside.
These layout dimensions remain editable Python parameters. The workpiece is
40 × 40 × 60 mm, leaving 5 mm approach clearance on either side with standard
Hand-E fingers. The gripper geometry is unchanged. Its catalog TCP is 12.5 mm
ahead of the pad centres; teaching compensates for this offset. Door travel,
opening and handle size remain the machine model's own dimensions. The hand
grasps 40 mm above the handle centre and approaches obliquely from above.
The bar participates in collision checks; its mounting stubs remain visual-only.

The fixed actuator feet move into the original button zones. Their signals
are measured from the geometry, not written directly by the robot program.
The robot remains stationary throughout each press. Each device has its own
linear axis; travel is the modeled button stroke plus a 1 mm rest gap. Body,
mounting bracket and tube geometry is estimated from the public product
photograph; it is not vendor CAD. The 20 mm/s cylinder speed is a simulation
setting, not a manufacturer timing guarantee.

The standard solution includes one Button Activator. This example uses two
additional units and requires three independently controlled pneumatic
channels. The catalog builder also contains the metadata-only
`robotiq/button-activator/sol-mt-but-act-kit/r1` accessory pack. The manufacturer
brochure identifies SOL-MT-BUT-ACT-KIT as a second-button add-on (one actuator,
5 m tube and pneumatic components); it does not establish third-channel support.
The live demo uses the product name and source until that pack is published.
The supplied valve arrangement, extra valves, controller mapping,
status-light sensors, URCap versions and physical connection plan remain
unverified. The demo does not represent pneumatic failure detection or the
complete Robotiq control stack. Assembly loads/CoG/inertia, door effort,
gripping friction, base mounting, hardware tolerances and commissioning also
remain outside the simulation validation.

Run:

```sh
python examples/machining/machine_tending_demo.py /tmp/machine_tending.usdc
python examples/machining/machine_tending_demo.py /tmp/machine_tending.usdc --catalog
python examples/machining/two_machine_cell_demo.py /tmp/two_machine_cell.usdc
```

The handover includes a portable `.botrail` project, BOM, PLCopen, button and
door interlocks, and the baseline plus three fault scenarios. Those checks
validate the simulated sequence; they are not a hardware acceptance test.
