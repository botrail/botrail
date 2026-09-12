# Multi-purpose hands (`bt.tools`)

A bracket carrying several tools — a gripper mount, pins, forks — each
with its own tip frame, built as a joint-less robot model to weld on with
`Robot.attach_tool`; and the process-tool envelopes that go on it, a
rotary bur and a screwdriver with its own stroke. See
[Machine tending](../../guides/machine-tending.md#the-multi-purpose-hand)
and [Assembly and fastening](../../guides/assembly.md#the-hand-and-the-presenter).

```python
bracket = bt.tools.multi_tool("hand", [bt.tools.Mount("gripper"), bt.tools.Pin("pusher"), bt.tools.Fork("fork")])
hand = bracket.attach_tool(coupling, flange="hand_gripper").attach_tool(gripper)
robot = arm.attach_tool(hand)
```

::: botrail.tools
