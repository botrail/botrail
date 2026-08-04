# Collision checking

Collision is checked between the robot and itself, the robot and every enabled
obstacle, and — with several robots — robot against robot. The same geometry
serves the live queries here, the [planner](motion-planning.md), and the
sequence rollout's tick checks.

## The queries

```python
scene.in_collision()             # bool, the fast yes/no
scene.check_collisions()         # who: [(('link', 'forearm_link'), ('obstacle', 'wall')), ...]
scene.min_obstacle_distance()    # tightest robot-obstacle distance in meters
                                 # 0 when colliding, None with no obstacles
```

`check_collisions` names pairs as `(kind, name)` tuples with kind `"link"` or
`"obstacle"` — the same names the studio uses to highlight the offending
geometry in red.

Over a whole baked cycle, the equivalent of `min_obstacle_distance` is
[`SequenceTimeline.min_clearance`][botrail.SequenceTimeline.min_clearance] —
see [Timeline assertions](timeline-assertions.md).

## What the shapes are

Primitives collide as themselves. Meshes collide as **VHACD convex
decompositions**: triangle-mesh-vs-triangle-mesh testing misses containment
and mis-reports distance, so every mesh is decomposed into convex pieces
(about a second per mesh on first load, cached on disk afterwards). The studio
still renders the original mesh — only the collision layer sees the
decomposition.

Anything that could not be prepared for collision is listed, not silently
dropped:

```python
scene.collision_warnings         # link shapes skipped for collision checking
```

## Self-collision, and the inter-robot ACM

A robot's own allowed-contact matrix (adjacent links, permanently-touching
pairs) is generated automatically by sampling. **Inter-robot pairs are never
inferred**: whether two arms may touch depends on where their bases stand,
which no sampler can know — so it is the author's call:

```python
scene.allow_inter_robot_collision("near", "/panda/panda_link0",
                                  "far",  "/panda/panda_link0")
```

the escape hatch for arms that share a mount plate or are meant to touch.

## Switching obstacles out

```python
scene.set_obstacle_enabled("cleats", False)
```

Disabled obstacles are excluded from every query above but keep rendering —
moving scenery, dress geometry, or a fixture you want visible but not planned
around.

## Grasped objects

An [attached](attach-and-tracking.md) obstacle collides *as part of the
robot*: it is checked against the environment, excluded against its
`touch_links` (the gripper pads that are supposed to touch it), and carried
through planning. Detach it and it goes back to being an obstacle where it
stands.
