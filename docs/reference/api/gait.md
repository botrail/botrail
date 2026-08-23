# Gaits (`bt.Gait`)

How a robot mounted on a vehicle walks it: the feet, the stance, the
rhythm. Hand one to `scene.mount_robot(..., gait=...)` and the legs walk
whenever the vehicle drives. See [Legged robots](../../guides/legged.md).

```python
gait = bt.Gait(
    legs={"FL": "FL_foot", "FR": "FR_foot", "RL": "RL_foot", "RR": "RR_foot"},
    stance={...}, pattern="trot", period=0.45, lift=0.07, max_stride=0.45,
    foot_radius=0.022,
)
scene.mount_robot("walker", robot="go2", gait=gait)
timeline.footfalls("go2")        # every step: (leg, lift, land, (x, y, z))
```

::: botrail.gait
