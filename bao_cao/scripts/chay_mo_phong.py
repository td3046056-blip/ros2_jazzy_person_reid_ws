#!/usr/bin/env python3
"""Chay MOT kich ban sim_follow.py (planner + tracker THAT) va luu quy dao + the gioi ra JSON de ve hinh.

    python3 chay_mo_phong.py <kich_ban> <ra.json>            # env cua kich ban (DOOR, OBOX, START...) dat truoc
    PKG_DIR=<thu muc goi person_follow_nav cu> python3 chay_mo_phong.py ...   # chay ban cu

Ghi them (neu ban planner co): duong ban do moi ~0.5 s (khung odom = khung the gioi trong mo phong) va moi
ke hoach THOAT KET (tu the luc lap + chuoi dong tac). Moi kich ban phai chay o tien trinh rieng vi sim_follow
dung the gioi tu bien moi truong luc import.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import SCRIPTS_NAV  # noqa: E402

sys.path.insert(0, SCRIPTS_NAV)
import sim_follow as S  # noqa: E402

ten, ra = sys.argv[1], sys.argv[2]
FPN = S.FP.FollowPlannerNode
snap = {"nav": [], "esc": []}

_ctrl0 = FPN._control


def _control(self, *a, **k):
    out = _ctrl0(self, *a, **k)
    t = S.CLK.t - 1000.0
    nc = getattr(self, "nav_cache", None)
    if nc and nc.get("path_odom") and (not snap["nav"] or t - snap["nav"][-1]["t"] >= 0.5):
        snap["nav"].append({"t": round(t, 2), "path": [[round(x, 3), round(y, 3)] for x, y in nc["path_odom"]],
                            "pose": [self.robot_x, self.robot_y, self.robot_yaw]})
    return out


FPN._control = _control

if hasattr(FPN, "_esc_begin"):
    _esc0 = FPN._esc_begin

    def _esc_begin(self, plan, why, mode):
        x, y, th = self.robot_x, self.robot_y, self.robot_yaw
        poses = [[x, y, th]]
        for (kind, v, w, T) in plan:            # tich phan chuoi dong tac (mo hinh 2 banh vi sai)
            n = max(2, int(T / 0.1))
            for _ in range(n):
                dt = T / n
                x += v * math.cos(th) * dt
                y += v * math.sin(th) * dt
                th += w * dt
            poses.append([x, y, th])
        snap["esc"].append({"t": round(S.CLK.t - 1000.0, 2), "why": why, "mode": mode,
                            "plan": [[k, v, w, T] for (k, v, w, T) in plan], "poses": poses})
        return _esc0(self, plan, why, mode)

    FPN._esc_begin = _esc_begin

f, T = S.SCEN[ten][0], S.SCEN[ten][1]
intr = S.SCEN[ten][2] if len(S.SCEN[ten]) > 2 else None
rows, hit = S.run(ten, f, T, report_every=0.1, verbose=False, intruder=intr)

n_box = 4 * len(S.LOW_BOXES) + 4 * len(S.OBOXES)
walls = S.WALLS[:len(S.WALLS) - n_box] if n_box else list(S.WALLS)
keep = ("t", "rx", "ry", "yaw", "px", "py", "ix", "iy", "st", "note", "v", "w", "src", "d", "ex", "ey")
out = {
    "kich_ban": ten,
    "pkg": os.environ.get("PKG_DIR", ""),
    "env": {k: os.environ[k] for k in ("DOOR", "SIDE_DOOR", "WALL_T", "TALLWALL", "LOWBOX", "OBOX", "START", "WP",
                                       "CAM_HALF", "SEED", "LIDAR_NOISE", "START_TRAIL") if k in os.environ},
    "walls": [list(map(float, w)) for w in walls],
    "low_boxes": [list(map(float, b)) for b in S.LOW_BOXES],
    "oboxes": [[float(cx), float(cy), float(L), float(W), float(a), float(h)] for (cx, cy, L, W, a, h) in S.OBOXES],
    "cam_h": S.CAM_H,
    "rows": [{k: r.get(k) for k in keep} for r in rows],
    "hit": {k: (float(v) if isinstance(v, (int, float)) else None) for k, v in hit.items() if not isinstance(v, dict)},
    "mot": {k: float(v) for k, v in hit.get("mot", {}).items()} if isinstance(hit.get("mot"), dict) else {},
    "nav": snap["nav"],
    "esc": snap["esc"],
}
with open(ra, "w") as fh:
    json.dump(out, fh)
print("xong", ten, "->", ra, "| so dong", len(rows), "| thoat ket", len(snap["esc"]))
