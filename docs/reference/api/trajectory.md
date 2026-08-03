# Trajectory

A time-parameterized path for one robot: joint positions and velocities on a
time base, produced by [`Scene.plan`][botrail.Scene.plan],
[`Scene.plan_to_pose`][botrail.Scene.plan_to_pose], and
[`Scene.plan_motion`][botrail.Scene.plan_motion].

```python
traj = scene.plan_to_pose((0.35, -0.2, 0.35), seed=0)

traj.duration                      # seconds
traj.sample(0.5)                   # joint positions at t = 0.5 s
traj.export_csv("motion.csv", dt=0.008)
traj.export_script("prog.script", dialect="urscript")
```

::: botrail.Trajectory
