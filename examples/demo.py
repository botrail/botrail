"""Minimal botrail demo: load a URDF and open the studio in a browser.

Run with:  python examples/demo.py
"""

from pathlib import Path

import botrail as bt

robot = bt.Robot.from_urdf(Path(__file__).parent / "simple_arm.urdf")
print(robot)
print("joints:", robot.joint_names)

scene = bt.Scene(robot)
bt.studio(scene)
