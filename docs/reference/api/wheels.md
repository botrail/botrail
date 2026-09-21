# Wheels (`bt.Wheels`)

How a robot mounted on a vehicle rolls it: which joints are the wheels, how
big they are, which frame stands on the floor. Hand one to
`scene.mount_robot(..., wheels=...)` and the wheel joints turn by exactly
what the vehicle drives. See
[The robot is the vehicle](../../guides/vehicles-and-amr.md#the-robot-is-the-vehicle-a-semi-humanoid).

```python
wheels = bt.Wheels({"left_wheel_joint": 0.085, "right_wheel_joint": 0.085},
                   base_frame="base_footprint")
scene.add_vehicle("base", body=[], path=..., stations=..., drive=wheels.vehicle_drive)
scene.mount_robot("base", robot="g1d", wheels=wheels)

wheels = bt.Wheels.from_catalog("unitree/g1/g1-d")     # a vehicle.mobile_manipulator package
```

::: botrail.wheels
