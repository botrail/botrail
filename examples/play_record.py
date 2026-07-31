import sys
sys.path.insert(0, "examples")

import botrail as bt
from demo import build_scene

scene = build_scene()
server = bt.studio(scene, block=False)   # ブラウザが開く

res = scene.play_usd_animation("cell_seq.usda")
print(res["mode"], f"{res['duration']:.2f}s", res["object_tracks"])

input("Enterで終了")
server.stop()

