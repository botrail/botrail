# Multi-purpose hands (`bt.tools`)

A bracket carrying several tools — a gripper mount, pins, forks — each
with its own tip frame, built as a joint-less robot model to weld on with
`Robot.attach_tool`. See [Machine tending](../../guides/machine-tending.md#the-multi-purpose-hand).

```python
bracket = bt.tools.multi_tool("hand", [bt.tools.Mount("gripper"), bt.tools.Pin("pusher"), bt.tools.Fork("fork")])
hand = bracket.attach_tool(coupling, flange="hand_gripper").attach_tool(gripper)
robot = arm.attach_tool(hand)
```

::: botrail.tools
