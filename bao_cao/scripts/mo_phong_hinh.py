#!/usr/bin/env python3
"""Chay cac kich ban mo phong dung de ve hinh "truoc / sau" cho bao cao, voi tung PHIEN BAN code (commit / tag git).

    python3 mo_phong_hinh.py                 # chay het, bo qua ca da co JSON
    python3 mo_phong_hinh.py cua_ben bay_U   # chi vai ca
    python3 mo_phong_hinh.py --lai ...       # chay lai ca da co

Cau hinh kich ban lay DUNG tu regress_follow.py (nhom, nhan) de khop so lieu hoi quy trong CLAUDE.md.
Ban cu: git archive <ref> src/person_follow_nav -> ~/bao_cao_cache/pkg_<ref> (ngoai repo).
Ket qua: bao_cao/du_lieu/mo_phong/<ca>__<phien_ban>.json
"""
import importlib.util
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import WS, SCRIPTS_NAV, DU_LIEU, CACHE  # noqa: E402

OUT = os.path.join(DU_LIEU, "mo_phong")

# phien ban -> git ref ("" = cay lam viec hien tai = main = ban-tot-nhat-0711)
REF = {
    "bc3dcdf": "bc3dcdf",          # 26/09 (chay that 29/09) — truoc sua dung im giua cua 0.81 m (7.2-AD)
    "6ddbe62": "6ddbe62",          # 22/09 — truoc sua cua truoc 0.81 m (7.2-Z)
    "799d643": "799d643",          # 23/09 — truoc sua re vao cua ben hong (7.2-AA/AB)
    "6d0aa54": "6d0aa54",          # 07/10 — truoc ban do luoi + vat thap (7.2-AJ/AK)
    "2a979a8": "2a979a8",          # 09/10 — truoc sua cua bep (7.2-AQ)
    "d271164": "d271164",          # 14/10 — truoc sua tu lanh / thung xop / goc ket (7.2-AR)
    "0311": "ban-tot-nhat-0311",   # 03/11 — truoc tui hep (7.2-AS)
    "0611": "ban-tot-nhat-0611",   # 05/11 — truoc cho hep (7.2-AT)
    "0711": "",                    # ban cuoi
}

# ca -> (nhom, nhan trong regress_follow, [phien ban], mam)
CA = {
    "cua_giua": ("door_near_0.02", "3.365:-0.62:-90", ["bc3dcdf", "0711"], 1),
    "cua_ben": ("regress", "cua_ben_cham+day", ["799d643", "0711"], 1),
    "vat_thap": ("lowbox", "L5:px2.4:0:0.35:5", ["6d0aa54", "0711"], 1),
    "u_trap": ("hard", "U_trap:px2.6", ["6d0aa54", "0711"], 1),
    "cua_bep": ("cua_bep", "tuong_trai:thang:nz0.01", ["2a979a8", "0711"], 1),
    "tu_lanh": ("tu_lanh", "tu_lanh_trai:re_trai:v0.17", ["d271164", "0711"], 2),
    "tui_hep": ("tui_hep", "w0.85:giua:v0.25", ["0311", "0711"], 1),
    "bay_U": ("bay_U", "W1.00:sau1.3:cao:thang", ["0611", "0711"], 1),
    "bay_goc": ("bay_goc", "goc_phai:truoc0.15:ben0.08:yaw+45", ["0611", "0711"], 1),
    "bay_khe": ("bay_khe", "khe0.60:y+0.00:yaw+0:cao", ["0311", "0711"], 1),
    "chen": ("chen", "dung:2.2:5.0:0.0", ["0711"], 1),
}


def pkg_of(ver):
    ref = REF[ver]
    if not ref:
        return os.path.join(WS, "src", "person_follow_nav")
    d = os.path.join(CACHE, "pkg_" + ver)
    p = os.path.join(d, "src", "person_follow_nav")
    if not os.path.isdir(p):
        os.makedirs(d, exist_ok=True)
        arch = subprocess.run(["git", "-C", WS, "archive", ref, "src/person_follow_nav"], check=True,
                              capture_output=True).stdout
        subprocess.run(["tar", "-x", "-C", d], input=arch, check=True)
    return p


def jobs_rf():
    sys_argv = sys.argv
    sys.argv = ["x"]
    spec = importlib.util.spec_from_file_location("rf", os.path.join(SCRIPTS_NAV, "regress_follow.py"))
    rf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rf)
    sys.argv = sys_argv
    return {(j["group"], j["label"]): j for j in rf.JOBS}


def chay(ca, ver, job, seed, lai):
    os.makedirs(OUT, exist_ok=True)
    ra = os.path.join(OUT, f"{ca}__{ver}.json")
    if os.path.exists(ra) and not lai:
        return ca, ver, "da co"
    env = dict(os.environ)
    env.update(job["env"])
    if "SEED" in job["env"]:
        env["SEED"] = str(int(job["env"]["SEED"]) + 100 * (seed - 1))
    else:
        env["SEED"] = str(seed)
    env["PKG_DIR"] = pkg_of(ver)
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["OMP_NUM_THREADS"] = "1"
    p = subprocess.run([sys.executable, os.path.join(HERE, "chay_mo_phong.py"), job["scen"], ra], env=env,
                       capture_output=True, text=True, timeout=1800, cwd=SCRIPTS_NAV)
    tail = (p.stdout + p.stderr).strip().splitlines()[-1:] if (p.stdout + p.stderr).strip() else [""]
    return ca, ver, ("OK " if p.returncode == 0 else "LOI ") + tail[0][:150]


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    lai = "--lai" in sys.argv
    J = jobs_rf()
    todo = []
    for ca, (g, lab, vers, seed) in CA.items():
        if args and ca not in args:
            continue
        for v in vers:
            pkg_of(v)                    # giai nen truoc (tranh 2 tien trinh cung giai nen)
            todo.append((ca, v, J[(g, lab)], seed))
    with ThreadPoolExecutor(max_workers=int(os.environ.get("JOBS", "12"))) as ex:
        for ca, v, msg in ex.map(lambda a: chay(*a, lai), todo):
            print(f"{ca:10s} {v:8s} {msg}")
