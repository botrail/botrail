# Studio

`bt.studio(scene)` serves the 3D studio on `127.0.0.1` and opens a browser. The
UI and your Python session share one scene: dragging the TCP gizmo runs IK and
the new joint values are visible from Python, and `scene.set_tcp_target(...)`
moves the robot in the browser.

```python
bt.studio(scene)                          # blocks until Ctrl-C

server = bt.studio(scene, block=False)    # or keep working in the same script
print(server.url)
server.stop()
```

!!! note "The studio assets must be present"

    Wheels bundle the built UI. In a source checkout, run
    `./scripts/build_studio.sh` first, or point `BOTRAIL_STUDIO_DIR` at a built
    studio `dist` directory.

::: botrail.studio

## StudioServer

The handle returned by `studio(..., block=False)`. The server also stops when
the handle is garbage-collected.

::: botrail.StudioServer
