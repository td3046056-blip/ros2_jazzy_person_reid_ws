#!/usr/bin/env python3
"""Phat lai bag (pickle tu dump_bag.py) qua follow_planner_node THAT (stub ROS nhu test_sim.py) — VONG HO: tu the xe
lay tu /odom cua bag, nen chi cho biet planner SE RA LENH GI o dung cac tu the that, khong cho biet sau do xe di dau.
Bat / tat bam theo truong 'enabled' cua /follow/planner_status trong bag. Kiem tra cong cu: chay voi ban package da
ghi bag -> trang thai khop ban ghi ~97 % (bag 15/10, ban d271164).

    python3 replay_planner.py <bag.pkl> <pkg_dir> <out.json> [t_tu t_den]
    PL_SET="ten=gia_tri,..." de ghi de tham so planner.

Ghi moi nhip dieu khien: t (s tu dau bag), state/note/cmd cua planner phat lai (state, note, v, w) va cua planner
trong bag (r_state, r_note, r_v, r_w), tu the xe, nguon / khoang cach muc tieu.
"""
import json, math, os, pickle, sys, types
import numpy as np
pkl, PKG, out = sys.argv[1], sys.argv[2], sys.argv[3]
tf = float(sys.argv[4]) if len(sys.argv) > 4 else -1e9
tt = float(sys.argv[5]) if len(sys.argv) > 5 else 1e9
stub = open(os.path.join(PKG, "scripts/test_sim.py")).read().split("sys.path.insert")[0]
exec(compile(stub, "stubs", "exec"))
sys.path.insert(0, PKG)
import yaml  # noqa
import person_follow_nav.follow_planner_node as FP  # noqa
CLK = types.SimpleNamespace(t=0.0)
FP.time = types.SimpleNamespace(time=lambda: CLK.t, monotonic=lambda: CLK.t, sleep=lambda s: None)
CFG = yaml.safe_load(open(os.path.join(PKG, "config/follow_nav.yaml")))
D = pickle.load(open(pkl, "rb"))
pl = FP.FollowPlannerNode()
pl._p.update(CFG["follow_planner_node"]["ros__parameters"])
for kv in [x for x in os.environ.get("PL_SET", "").split(",") if x.strip()]:
    k, v = kv.split("=", 1)
    pl._p[k.strip()] = yaml.safe_load(v)
pl._read_params()
pl.t_arr = np.arange(1, pl.horizon_steps + 1, dtype=np.float64) * pl.horizon_dt
cap = {}
class Cap:
    def __init__(self, k): self.k = k
    def publish(self, m): cap[self.k] = m
pl.pub_cmd = Cap("cmd"); pl.pub_status = Cap("st"); pl.pub_markers = Cap("mk")
ev = []
for ts, d in D.get("/scan", []):
    ev.append((ts, 0, "scan", d))
for ts, d in D.get("/odom", []):
    ev.append((ts, 1, "odom", d))
for ts, d in D.get("/follow/target", []):
    ev.append((ts, 2, "tgt", d))
for ts, d in D.get("/rssi/bearing", []):
    ev.append((ts, 3, "rssi", d))
st_rec = sorted(D.get("/follow/planner_status", []))
ev.sort(key=lambda e: (e[0], e[1]))
t0 = min(e[0] for e in ev)
dtc = 1.0 / pl.control_hz
next_tick = t0 + 0.5
rows = []
j = 0
rec_enabled = False
rec_last = None
def mkscan(d):
    return types.SimpleNamespace(**d)
def mkodom(d):
    pos = types.SimpleNamespace(x=d["x"], y=d["y"], z=0.0)
    ori = types.SimpleNamespace(x=d["qx"], y=d["qy"], z=d["qz"], w=d["qw"])
    return types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=pos, orientation=ori)))
for ts, _, kind, d in ev + [(ev[-1][0] + 0.2, 9, "end", None)]:
    while next_tick <= ts:
        CLK.t = next_tick
        while j < len(st_rec) and st_rec[j][0] <= next_tick:
            try:
                rec_last = json.loads(st_rec[j][1]); rec_enabled = bool(rec_last.get("enabled"))
            except Exception:
                pass
            j += 1
        if rec_enabled and not pl.enabled:
            pl._enable_cb(None, types.SimpleNamespace())
        elif not rec_enabled and pl.enabled:
            pl._disable_cb(None, types.SimpleNamespace())
        cap.pop("st", None)
        try:
            pl._control()
        except Exception as e:
            import traceback; traceback.print_exc(); sys.exit(1)
        rel = next_tick - t0
        if tf <= rel <= tt:
            s = json.loads(cap["st"].data) if "st" in cap else {}
            rows.append(dict(t=round(rel, 3), state=s.get("state"), note=s.get("note"), v=s.get("cmd_v"), w=s.get("cmd_w"),
                             rx=round(pl.robot_x, 3), ry=round(pl.robot_y, 3), ryaw=round(pl.robot_yaw, 3),
                             r_state=(rec_last or {}).get("state"), r_note=(rec_last or {}).get("note"),
                             r_v=(rec_last or {}).get("cmd_v"), r_w=(rec_last or {}).get("cmd_w"),
                             src=(rec_last or {}).get("target_source"), dist=(rec_last or {}).get("target_distance_m")))
        next_tick += dtc
    if kind == "end":
        break
    CLK.t = ts
    if kind == "scan":
        pl._scan_cb(mkscan(d))
    elif kind == "odom":
        pl._odom_cb(mkodom(d))
    elif kind == "tgt":
        pl._target_cb(types.SimpleNamespace(data=d))
    elif kind == "rssi" and hasattr(pl, "_rssi_cb"):
        try:
            pl._rssi_cb(types.SimpleNamespace(data=d))
        except Exception:
            pass
json.dump(rows, open(out, "w"))
print(len(rows), "nhip")
