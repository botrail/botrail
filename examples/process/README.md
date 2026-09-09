# Real-product process examples

These examples extend the existing mounting assemblies into a workpiece cycle.
They use the published prebuilt catalog; no local geometry build is required.
Fixtures are ordinary Python boxes and the ATI bur is a cylindrical envelope.

```bash
uv run python examples/welding/nimak_spot_welding_demo.py --studio
uv run python examples/machining/ati_deburring_demo.py --studio
```

Add `--output <directory>` to save `cell.botrail`, `setup.json`, a process report
in JSON/Markdown and a BOM. ATI also exports its physical connection requirements.
Edit the exported setup and pass `--setup <directory>/setup.json` to rebuild the
example. The saved project retains the setup, geometry, paths and sequences.
In Studio, use the timeline to inspect approach, process and release; the SFC
shows the start permissives and inhibited branch. Names prefixed `sim_` are
scenario inputs, not verified feedback from installed devices.

| Example | Product configuration | Demonstrated behavior |
| --- | --- | --- |
| NIMAK | Kawasaki BX250L-B001 + 95.020.516/P3U r4 | Two spots on overlapping 1 mm steel sheets: approach, close with 1 mm clearance on each side, simulated 0.2 s request, hold, open, withdraw |
| ATI | FANUC CRX-10iA + RCV-250 CRX kit + 9150-RC-B-24645 | Tangential lead-in, 100 mm aluminum edge, lead-out; 8 mm/s nominal feed, 0.2 mm radial engagement |

These are geometric/process-sequence examples. NIMAK applies no electric current
or squeeze force. ATI does not simulate cutting forces, pneumatic compliance,
flutes or material removal. Timings include planning and SFC scan timing; they
are not measured production cycle times. Only the ATI `bur_cutter` link is
exempted against stock, and toolpath rapids retain contact checks. NIMAK retains
all gun/workpiece collision checks.

## Product inputs and cell settings

`ati_rcv250.json` and `nimak_95_020_516.json` separate `facts` (value, source,
basis), chosen `settings`, physical `control` inputs, and `load_components`.
Missing measurements use JSON `null`. Editing a setting changes its checks;
ATI projection/feed/engagement and NIMAK sheet thickness/opening/timing also
change the generated geometry or motion. Do not change a manufacturer limit to
make an incompatible setting pass. The examples compute the ATI nominal TCP
from the selected collet nose, exposed shank and head length.

The [ATI RCV-250 manual, 9610-50-1043-04](https://www.ati-ia.com/app_content/Documents/9610-50-1043.pdf)
supplies the bit dimensions and process limits. The aluminum-cut 24645 is an
additional purchase; the kit's included 24061 is a standard-cut bit. The selected
bur has a 6.35 mm shank, 9.525 mm head diameter and 15.875 mm cutting length.
The 10 mm exposed shank is a setup choice, not a verified insertion depth.
The nominal collet nose comes from the public reference model derived from
RCV-250-E CAD; the exact purchased collet and TCP need measurement.

Motor and compliance use separate regulated air circuits: nominal motor supply
6.2 bar; compliance 1.0–4.1 bar. Motor air requires filtration and lubrication;
compliance air is clean and dry. The reported 7.1–14.2 L/s consumption does not
specify volumetric reference conditions, so it is not silently treated as NL/min.
The bur needs a documented speed rating suitable for 40,000 rpm idle operation.
20,000 rpm is approximate working speed, not a closed-loop speed command.

The [ATI CRX kit manual, 9610-50-1049-03](https://www.ati-ia.com/library/documents/ATI_MR_CRX.zip)
identifies the air preparation kit and separate 24 V motor/compliance solenoids.
The example declares their required physical ports using `bt.connections`.
Actual outlets, terminal addresses, driver polarity/current and feedback devices
remain to be selected and connected; the simulated permissive is independent of
that wiring. `air_missing` prevents the cycle from starting. A known invalid
setup, such as 4 bar motor pressure, also takes the inhibited branch.

For NIMAK, the selected configurator record supplies the 5,500 N force limit;
2,500 N is only a demo input and is not mechanically applied. The
[NIMAK multiframegun series page](https://www.nimak.com/en/spotweldinggun/multiframegun/)
provides transformer-level reference values (130 kVA and 6 L/min), not a verified
primary supply or the complete configured gun's cooling requirement. Current,
inverter, drive controller, water pressure/temperature and qualified welding
schedule remain unspecified. CAD labels a KUKA KRC4 drive, whose integration
with the Kawasaki host is unresolved. `cooling_missing` inhibits the dry cycle.
The existing jaw window of 0–20 degrees and speed of 0.2 rad/s remain simulation
settings. Cartesian transfers use the catalog's joint timing limits.

These are **start permissives**. The example does not implement continuous
fault monitoring or a safety controller. Unknown hardware qualification does
not prevent inspecting the nominal simulation.

## Process API and loads

```python
import json
import botrail as bt

profile = json.loads(open("setup.json").read())
# The workpiece already has a Part identity.
bt.process.configure(scene, "fixture/stock", profile)
report = bt.process.report(scene, "fixture/stock")
report.save("process.json")
print(report.to_markdown())
# Also included in scene.check() and bt.review(scene).
```

Checks compare the live TCP position and optional +Z axis with the selected tool
frame, combine declared masses and tensors, check scalar payload and process
inputs, and retain missing calibration/qualification records as `unknown`.
`pass` means this declared-input check passed; `ready` means all listed checks
passed, not hardware approval or a prediction of process quality. References
such as `qualification_ref` record an externally established result, which this
module does not independently verify.

Each load component declares its own frame, mass in kg, local COM in metres and
COM inertia tensor in kg m² as `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`. The off-diagonal
values are tensor entries; account for sign conventions in CAD exports. Split
fixed and moving NIMAK assemblies so opening the jaw changes the aggregate.
Include fasteners and the moving portion of the dress pack without double
counting components already included in an assembly measurement.

```python
load = bt.process.aggregate_load(
    scene, profile["robot"], profile["flange"],
    profile["load_components"], complete=profile["load_components_complete"],
)
```

Set `load_components_complete` only when all mounted load is represented.
The output reports COM in flange axes, rotated inertia **about the flange
origin** (including parallel-axis terms), and the static gravity moment at the
current pose. A missing mass does not become zero; a known subtotal exceeding
payload still fails even if other masses are missing. Moment/inertia envelope
acceptance and dynamic loads require robot-specific engineering. Declaring
these data does not overwrite URDF inertials or change native motion dynamics.

The NIMAK configured gun's mass/COM/inertia are still missing. ATI's documented
1.71 kg is for the tool **without adapters** (an older drawing says 1.73 kg),
not the complete mounted kit. Both examples therefore keep aggregate load and
measured TCP calibration incomplete. Populate the supplied component records
from a supplier mass-properties report or measurements; retain their source.
