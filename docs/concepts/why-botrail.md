# Why botrail

**The claim in one sentence: a robot cell should be something you can diff,
test, and hand over — so botrail makes the cell text, the motions planned, and
the simulation deterministic.**

## The gap it sits in

Tools that author robot cells split into two camps, and both leave the same
thing on the table.

**Commercial cell software** — offline programming and process-simulation
suites — has the vocabulary a cell needs: conveyors, sensors, sequences of
operations. But the cell lives in a proprietary project file behind a desktop
license. You cannot `git diff` it, you cannot run it in CI, and the motions in
it are largely *taught*: move a pallet 10 cm and the teaching breaks, which is
how cells become assets nobody dares touch.

**Open simulators and planning stacks** — physics simulators, motion-planning
frameworks — are open but speak robot, not cell. Conveyor feeds, photo-eyes,
and step advance are yours to script from scratch; the heavyweight ones bring
ROS or a GPU as table stakes; and physics-based simulation is not
deterministic in the way regression testing needs.

The empty quadrant: **author the cell in open formats, verify it
deterministically, ship it in open formats**. That is the position botrail
takes:

* The cell is **text** — Python, `.botrail` JSON, USD. It diffs, reviews, and
  versions like the code it is.
* Motions are **planned**, so a layout edit re-solves instead of breaking
  teaching. The environment becomes data you can move.
* The bake is **deterministic** — same input, bit-identical timeline — so
  cycle time, sensor timing, and clearance are pytest assertions. *A layout
  change that costs a second fails a test.* No other cell tool supports that
  workflow; it is botrail's core claim.
* Deliverables are **open**: USD animation, CSV/JSON, vendor robot programs,
  generated Python.

## Where it stands next to Isaac Sim

Complement, not competitor. botrail reads Isaac's USD assets as-is and writes
USD that Omniverse opens. If you have those assets, botrail is the light loop
beside the heavy one: layout studies and cycle verification in a
`pip install`-and-browser footprint, without booting a full physics simulator
to answer "does the cycle close, and in how many seconds?"

## Where it stands next to MoveIt and friends

Below the cell layer, botrail is also a ROS-free planning stack: pip-one-shot
install, RRT-Connect and time parameterization, IK, collision — with an
editable 3D UI on top, which the pip-installable planning libraries don't
have. Two details matter in practice: Xacro loads **without ROS** (most real
robot descriptions are xacro, and this is where ROS-free tools usually give
up), and the whole studio also runs [in the browser](../guides/browser-only.md)
with no install at all.

## What botrail does *not* claim

The limits are part of the position — determinism is bought by refusing
physics, and honesty about that is what makes the numbers trustworthy.

* **No dynamics, no contact physics.** botrail answers *does it reach, does
  it clear, how many seconds* — not whether the part slips in the gripper or
  tips on the belt.
* **No throughput statistics.** One deterministic cycle, not stochastic
  production simulation — botrail is a cell verifier, not a factory
  simulator.
* **Per-robot planning.** With several arms, each plans with the others
  frozen; there is no cooperative planning. Execution-time interference is
  caught by tick checking, and separation is authored with interlocks — the
  way a PLC cell actually does it.
* **PLC vocabulary, not PLC connectivity.** Steps, signals, and scan cycles
  are the mental model; there is no OPC-UA link to real hardware.
* **USD in, not CAD in.** STEP/JT conversion is an upstream job for other
  tools.

If those trade-offs match your problem — and for cycle-time and layout
verification they usually do — the rest of the documentation shows the
workflow end to end, starting with
[Your first cell](../getting-started/first-cell.md).
