# Cables and tubes (`bt.routing`)

Routes between the ports of the [connection plan](connections.md): the
line each is made of (a *spec*), the clamps and ducts it passes, and what
is derived from them — centerline, lengths, cable list, BOM rows and the
checks. See [Cables and tubes](../../guides/routing.md).

```python
import botrail as bt

rt = bt.routing
rt.spec(scene, "tu0604", kind="tube", media=["pneumatic"], od_mm=6,
        bend_r_fixed_mm=15, bend_r_moving_mm=20, order_step_m=20,
        manufacturer="SMC", model="TU0604BU-20")
rt.clamp(scene, "clip", at=("arm", "forearm_link", (-0.2, 0.0, 0.05)), length_m=0.03)
rt.route(scene, "air_hand", carries="air_hand", spec="tu0604", via=["clip"], slack_m=0.15)

rt.geometry(scene, "air_hand")["cut_length_m"]
rt.cable_list(scene).save("cables.csv")
assert rt.report(scene).ok
```

::: botrail.routing
