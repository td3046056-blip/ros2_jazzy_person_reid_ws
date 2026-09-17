import math, sys, numpy as np
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from person_follow_nav.geometry import wrap_pi

print("="*72)
print("TEST 7 — BO NHO GOC TUONG DOI (code cu) vs BO NHO ODOM (code moi)")
print("="*72)
print("""Kich ban: nguoi dung yen cach xe 2.0m, goc +0 do (thang truoc mat).
Mot nguoi la chen ngang, camera mat hinh. Xe ne sang TRAI, quay 35 do trong 1.2s.
Cau hoi: sau khi ne xong, xe tuong nguoi o dau?
""")

d, b0 = 2.0, 0.0                      # nguoi: 2.0m, goc 0
px, py = d*math.cos(b0), d*math.sin(b0)   # vi tri THAT (co dinh trong odom)

print(f"{'t(s)':>5} {'yaw xe':>8} {'goc THAT':>10} {'code CU':>9} {'sai so':>8} {'code MOI':>10} {'sai so':>8}")
print("-"*72)
w = math.radians(35)/1.2
rx=ry=yaw=0.0; v=0.18
maxerr_old=maxerr_new=0.0
for k in range(13):
    t=k*0.1
    # goc THAT toi nguoi trong base_link hien tai
    dx,dy = px-rx, py-ry
    true_b = wrap_pi(math.atan2(dy,dx)-yaw)
    old_b  = b0                                   # code cu: giu nguyen goc cu
    new_b  = true_b                               # code moi: tinh lai tu odom
    e_old = abs(math.degrees(wrap_pi(old_b-true_b)))
    e_new = abs(math.degrees(wrap_pi(new_b-true_b)))
    maxerr_old=max(maxerr_old,e_old); maxerr_new=max(maxerr_new,e_new)
    if k%2==0:
        print(f"{t:5.1f} {math.degrees(yaw):7.1f}° {math.degrees(true_b):9.1f}° "
              f"{math.degrees(old_b):8.1f}° {e_old:7.1f}° {math.degrees(new_b):9.1f}° {e_new:7.1f}°")
    rx+=v*math.cos(yaw)*0.1; ry+=v*math.sin(yaw)*0.1; yaw=wrap_pi(yaw+w*0.1)

fov=62.0
print("-"*72)
print(f"Sai so lon nhat — code CU: {maxerr_old:.1f}°   code MOI: {maxerr_new:.1f}°")
print(f"Nua truong nhin camera: ±{fov/2:.0f}°")
print()
if maxerr_old > fov/2:
    print(f"=> Code CU sai {maxerr_old:.0f}°, VUOT nua truong nhin ({fov/2:.0f}°).")
    print("   Xe quay theo goc nho sai -> nguoi ra khoi khung hinh -> mat han.")
    print("   DAY LA LY DO XE KHONG THE VUA NE VUA BAM DUOC NGUOI.")
print(f"=> Code MOI sai {maxerr_new:.1f}° vi tinh lai tu odom moi chu ky.")
print("="*72)
