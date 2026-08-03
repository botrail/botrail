# Quickstart

Load a robot, put something in its way, plan around it, and open the studio.
Every snippet below runs as-is once you have the sample arm from the next
section — no downloads, no ROS.

## The sample arm

Save this as `arm.urdf` next to your script. It is a primitive-only 4-DOF arm,
so nothing has to be fetched. If you have your own URDF, use it instead — the
only thing that changes is the number of joints.

??? example "arm.urdf"

    ```xml
    <?xml version="1.0"?>
    <robot name="arm">
      <link name="base_link">
        <visual>
          <origin xyz="0 0 0.04"/>
          <geometry><cylinder radius="0.075" length="0.08"/></geometry>
        </visual>
      </link>

      <joint name="pan" type="revolute">
        <parent link="base_link"/><child link="shoulder_link"/>
        <origin xyz="0 0 0.08"/><axis xyz="0 0 1"/>
        <limit lower="-3.1416" upper="3.1416" effort="50" velocity="2.0"/>
      </joint>
      <link name="shoulder_link">
        <visual><geometry><sphere radius="0.065"/></geometry></visual>
      </link>

      <joint name="lift" type="revolute">
        <parent link="shoulder_link"/><child link="upper_arm_link"/>
        <origin xyz="0 0 0.06"/><axis xyz="0 1 0"/>
        <limit lower="-2.2" upper="2.2" effort="50" velocity="2.0"/>
      </joint>
      <link name="upper_arm_link">
        <visual>
          <origin xyz="0 0 0.15"/>
          <geometry><cylinder radius="0.05" length="0.30"/></geometry>
        </visual>
      </link>

      <joint name="elbow" type="revolute">
        <parent link="upper_arm_link"/><child link="forearm_link"/>
        <origin xyz="0 0 0.30"/><axis xyz="0 1 0"/>
        <limit lower="-2.6" upper="2.6" effort="30" velocity="2.5"/>
      </joint>
      <link name="forearm_link">
        <visual>
          <origin xyz="0 0 0.125"/>
          <geometry><cylinder radius="0.04" length="0.25"/></geometry>
        </visual>
      </link>

      <joint name="wrist" type="revolute">
        <parent link="forearm_link"/><child link="wrist_link"/>
        <origin xyz="0 0 0.25"/><axis xyz="0 1 0"/>
        <limit lower="-3.1416" upper="3.1416" effort="10" velocity="3.0"/>
      </joint>
      <link name="wrist_link">
        <visual><geometry><sphere radius="0.045"/></geometry></visual>
      </link>

      <joint name="tool_mount" type="fixed">
        <parent link="wrist_link"/><child link="tool0"/>
        <origin xyz="0 0 0.06"/>
      </joint>
      <link name="tool0">
        <visual>
          <origin xyz="0 0 0.02"/>
          <geometry><box size="0.03 0.08 0.04"/></geometry>
        </visual>
      </link>
    </robot>
    ```

## Load a robot

```python
import botrail as bt

robot = bt.Robot.from_urdf("arm.urdf")

print(robot.dof)          # 4
print(robot.joint_names)  # ['pan', 'lift', 'elbow', 'wrist']
print(robot.tcp_link)     # 'tool0'  — the deepest link, used as the tool frame
print(robot.joint_limits) # [(-3.1416, 3.1416), (-2.2, 2.2), ...]
```

Xacro files (`bt.Robot.from_xacro`) are expanded without ROS, and USD
articulations — including Isaac Sim assets — load with
[`bt.Robot.from_usd`][botrail.Robot.from_usd].

## Build a scene

A [`Scene`][botrail.Scene] is the robot plus everything around it. Lengths are
meters, and the world is Z-up.

```python
scene = bt.Scene(robot)
scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))
```

`add_sphere`, `add_cylinder`, and `add_mesh` (STL/OBJ) work the same way.
Whole USD stages come in with one call, which also exposes their Xform prims as
named frames you can mount the robot on:

```python
scene.load_usd("cell.usda", prefix="env")
scene.set_robot_base_pose(*scene.frame("env/World/mount"))
```

## Pose it, and ask about clearance

`set_tcp_target` runs inverse kinematics toward a pose and applies the result:

```python
ik = scene.set_tcp_target((0.3, 0.1, 0.5))
print(ik.converged, ik.pos_error)   # True 2e-06

scene.in_collision()                # False
scene.min_obstacle_distance()       # 0.025 — the table edge is 2.5 cm from the base
```

Pass a `quaternion` to constrain orientation too; leaving it out matches
position only, which gives IK more freedom.

## Plan a motion

`plan_to_pose` solves IK for the goal, then plans a collision-free path with
RRT-Connect and time-parameterizes it:

```python
traj = scene.plan_to_pose((0.35, -0.2, 0.35), seed=0)

print(traj.duration)      # seconds, after time parameterization
print(traj.joint_names)
traj.export_csv("motion.csv", dt=0.008)
```

Passing `seed` makes the planner reproducible. The result is a
[`Trajectory`][botrail.Trajectory]: sample it (`traj.sample(t)`), export it
(`export_csv`, `export_json`), or turn it into a robot program
(`traj.export_script("prog.script", dialect="urscript")`).

## Open the studio

```python
bt.studio(scene)
```

This serves the 3D studio on `127.0.0.1` and opens your browser. Drag the TCP
gizmo and IK follows live; the joint sliders, collision highlighting, and
trajectory playback all act on the same scene your script holds. Anything you
change in the browser is visible from Python, and vice versa:

```python
scene.set_tcp_target((0.3, 0.1, 0.5))   # pushed to connected browsers
```

`bt.studio()` blocks until <kbd>Ctrl</kbd>+<kbd>C</kbd>. To keep working in the
same script — a notebook, say — pass `block=False` and keep the returned
handle:

```python
server = bt.studio(scene, block=False)
print(server.url)
...
server.stop()
```

## Save the cell

```python
scene.save_project("cell.botrail")   # meshes and USD stages bundled when needed
print(scene.generate_python())       # a script that rebuilds this scene
```

## Next

The scene so far is static geometry. Give it behavior — a conveyor, a sensor,
and a sequence that produces a cycle time you can assert on — in
[Your first cell](first-cell.md).
