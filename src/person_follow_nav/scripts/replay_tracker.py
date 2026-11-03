#!/usr/bin/env python3
"""Phat lai bag qua target_tracker_node THAT (stub ROS): /person_reid/target + /scan + /odom -> /follow/target (03/11).

    TR_SET="camera_height_m=0.70" python3 replay_tracker.py <bag.pkl> <pkg_dir> <out.json>

bag.pkl tu dump_bag.py (phai co /person_reid/target). Ghi moi nhip 20 Hz: valid, source, distance_m (d), cam_dist_m
(cd), elev_bias_deg (eb), odom_x/y. Vd. 03/11: so camera_height_m 0.34 / 0.70 tren bag 15/10 — ghep nham mat thung
xop lam chan nguoi giam mot nua khi do cao dung.
"""
import json, math, os, pickle, sys, types
import numpy as np
pkl, PKG, out = sys.argv[1:4]
stub = open(os.path.join(PKG, "scripts/test_sim.py")).read().split("sys.path.insert")[0]
exec(compile(stub, "stubs", "exec"))
sys.path.insert(0, PKG)
import yaml  # noqa
import person_follow_nav.target_tracker_node as TT  # noqa
from person_follow_nav import geometry as G  # noqa
CLK = types.SimpleNamespace(t=0.0)
TT.time = types.SimpleNamespace(time=lambda: CLK.t, monotonic=lambda: CLK.t, sleep=lambda s: None)
CFG = yaml.safe_load(open(os.path.join(PKG, "config/follow_nav.yaml")))
tr = TT.TargetTrackerNode()
tr._p.update(CFG["target_tracker_node"]["ros__parameters"])
for kv in [x for x in os.environ.get("TR_SET", "").split(",") if x.strip()]:
    k, v = kv.split("=", 1)
    tr._p[k.strip()] = yaml.safe_load(v)
tr._read_params()
tr.filter = G.AlphaBeta2D(alpha=tr.ab_alpha, beta=tr.ab_beta, max_speed=tr.max_person_speed)
cap = []
class Cap:
    def publish(self, m):
        if hasattr(m, "data"):
            cap.append((CLK.t, m.data))
for name in dir(tr):
    if name.startswith("pub"):
        setattr(tr, name, Cap())
D = pickle.load(open(pkl, "rb"))
C = D
ev = [(ts, 0, "scan", d) for ts, d in D["/scan"]] + [(ts, 1, "odom", d) for ts, d in D["/odom"]] + \
     [(ts, 2, "cam", d) for ts, d in C["/person_reid/target"]]
ev.sort(key=lambda e: (e[0], e[1]))
t0 = min(D["/odom"][0][0], D["/scan"][0][0])
dtu = 1.0 / tr.update_hz
nxt = ev[0][0] + 0.2
rows = []
for ts, _, kind, d in ev:
    while nxt <= ts:
        CLK.t = nxt
        cap.clear()
        tr._update()
        for (_t, data) in cap:
            try:
                j = json.loads(data)
            except Exception:
                continue
            if "distance_m" in j:
                rows.append(dict(t=round(nxt - t0, 3), valid=j.get("valid"), source=j.get("source"),
                                 d=j.get("distance_m"), cd=j.get("cam_dist_m"), eb=j.get("elev_bias_deg"),
                                 ox=j.get("odom_x"), oy=j.get("odom_y")))
                break
        nxt += dtu
    CLK.t = ts
    if kind == "scan":
        tr._scan_cb(types.SimpleNamespace(**d))
    elif kind == "odom":
        pos = types.SimpleNamespace(x=d["x"], y=d["y"], z=0.0)
        ori = types.SimpleNamespace(x=d["qx"], y=d["qy"], z=d["qz"], w=d["qw"])
        tw = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0), angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))
        tr._odom_cb(types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=pos, orientation=ori)),
                                          twist=types.SimpleNamespace(twist=tw), header=types.SimpleNamespace(frame_id="odom")))
    else:
        tr._cam_cb(types.SimpleNamespace(data=d))
json.dump(rows, open(out, "w"))
print(len(rows), "nhip")
