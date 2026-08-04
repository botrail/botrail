# Sequences (`bt.seq`)

The process vocabulary: a sequence is a list of steps, and a step is *entry
actions* plus a *transition condition*, evaluated on a fixed scan cycle — the
PLC step-advance model.

```python
sq = scene.sequence("cycle")
sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
sq.step("stop", actions=[bt.seq.stop("belt")])
sq.step("pick", actions=[bt.seq.motion("approach"), bt.seq.attach("crate")])
```

Omitting `transition` supplies the obvious default: a step that starts a motion
waits for it (`done()`), a step that starts nothing falls through
(`immediately()`).

## Actions at a glance

| Action | Effect |
| --- | --- |
| `motion(name)` | Start a named motion; await it with `done()` |
| `ramp(targets, duration)` | Drive joints to targets over a fixed duration — gripper open/close |
| `attach(obj)` / `detach(obj)` | Grasp, and release where the robot holds it |
| `track(obj)` / `untrack()` | Latch taught poses onto a moving part, and let go |
| `set_signal(name, value)` | Write an internal signal (declared with `scene.define_signal`) |
| `start(device)` / `stop(device)` | Run or halt a device — a conveyor or a source |
| `set_speed(device, speed)` | Rescale a conveyor's velocity, direction kept |
| `move_to(device, position)` | Command a linear axis; await with `device_done` |

## Conditions at a glance

| Condition | Advances when |
| --- | --- |
| `immediately()` | Always, on the next scan |
| `done()` | This step's motion has finished |
| `robot_done(robot)` | A named robot is idle — the inter-robot handshake |
| `elapsed(seconds)` | A timer expires |
| `signal(name, value)` | A signal, sensor, or device state matches |
| `device_done(device)` | A device reached its commanded target |
| `all_of(...)` / `any_of(...)` | Composed conditions hold |

## Reference

::: botrail.seq
    options:
      # Pure Python, so static analysis works here and keeps the source
      # order: actions first, then conditions, then the builder.
      force_inspection: false
      members_order: source
