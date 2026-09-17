import sys, os, math, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from person_follow_nav.geometry import scan_to_base_points, self_filter_mask, rect_clearance

print("="*72); print("TEST 8 — Self-filter: mo phong dung ca cua ban (tia 0.128m)"); print("="*72)
n=360; inc=2*math.pi/n
ranges=np.full(n,3.0)
# cot do than xe: 3 tia lien tiep o 0.128m (giong het truong hop cua ban)
for i in (200,201,202): ranges[i]=0.128
ranges[90]=0.60          # thung carton truoc mui xe

pts,bear,rng,ldeg = scan_to_base_points(ranges,0.0,inc,0.1,10.0,0,0,
                                        math.radians(-90.0),1.0,8.0)
F,R,HW=0.30,0.30,0.24
print(f"\nTong so diem: {pts.shape[0]}")
c=rect_clearance(pts[:,0],pts[:,1],F,R,HW)
n_collide=int((c<=0.0).sum())
print(f"So diem bi coi la VA CHAM (clearance=0): {n_collide}")
print(f"  => admissible = min_clear > margin_hard(0.07) se FALSE cho MOI quy dao")
print(f"  => _dwa() tra None  =>  xe KET VINH VIEN o BLOCKED\n")

keep = self_filter_mask(pts, ldeg, F,R,HW, margin=0.03, blind_sectors_deg=None)
p2=pts[keep]
c2=rect_clearance(p2[:,0],p2[:,1],F,R,HW)
print(f"Sau self-filter: giu {p2.shape[0]}/{pts.shape[0]} diem, bo {int((~keep).sum())}")
print(f"So diem va cham con lai: {int((c2<=0.0).sum())}")
print(f"Clearance nho nhat: {c2.min():.3f}m  (margin_hard=0.07)")

# thung carton co con khong
i=int(np.argmin(np.hypot(p2[:,0],p2[:,1])))
d=math.hypot(p2[i,0],p2[i,1]); b=math.degrees(math.atan2(p2[i,1],p2[i,0]))
print(f"\nVat can gan nhat con lai: {d:.3f}m tai goc {b:+.1f} do")
ok = (c2>0).all() and abs(d-0.60)<0.02 and abs(b)<3
print(f"  (thung carton 0.60m truoc mui xe — VAN GIU DUOC)")
print(f"\n=> {'DAT' if ok else 'LOI'}")
print("="*72)
