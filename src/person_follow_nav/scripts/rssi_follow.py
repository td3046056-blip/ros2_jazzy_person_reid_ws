#!/usr/bin/env python3
"""
rssi_follow.py — BAM THEO nguoi deo beacon bang RSSI + LiDAR (KHONG camera).

VI SAO KHONG PHAI "CHI RSSI"
----------------------------
Do tren xe 30/09-01/10: RSSI chi cho huong khi xe XOAY TAI CHO (~8 s moi lan, sai ~15 do), xe dung yen thi
khong phan biet duoc trai/phai, va khong do duoc khoang cach. Chi RSSI thi xe phai dung-xoay-di tung doan
(rssi_seek.py). De BAM LIEN TUC, chia viec:

  RSSI    NHAN CHU   xoay do 1 vong -> huong beacon. Chi nguoi deo beacon moi co huong nay.
  LiDAR   GIU BAM    khoa vao nhom chan gan huong do nhat roi bam tung vong quet (10 Hz), dung cac cong chan
                     nhay / chan vat che cua target_tracker_node.
  planner LAI XE     follow_planner_node NGUYEN BAN: chon khe + DWA, ne vat can, chui cua, giu cach 1 m.
                     Planner ghi vao /cmd_vel_follow; script nay chuyen tiep sang /cmd_vel khi dang bam.

Mat dau (bi che, ra khoi tam LiDAR) -> lai toi gan cho thay cuoi -> RSSI do lai -> khoa lai DUNG CHU. Trong 3 s dau sau
khi mat dau, neu gan cho du doan co DUNG MOT nhom chan DANG DI va khong co ai khac quanh do -> khoa lai ngay, khong xoay
do (kiem tra bang RSSI o lan dung yen dau tien); cho dong nguoi thi khong khoa nhanh.
LiDAR chi bam "nhom chan", khong biet chan AI: khi xe va muc tieu cung dung yen ma RSSI chua xac nhan qua
--verify-sec (20 s, lan sau gap doi), hoac muc tin hieu beacon doi han trong khi muc tieu dung yen
-> do lai; huong beacon lech muc tieu > 40 do hai lan lien tiep (hoac > 75 do) thi bo, khoa lai theo beacon.
Chua thay ai theo huong beacon trong tam LiDAR -> planner lai ve huong do (chi doan dang trong) roi do lai;
huong do bi vat chan ngay truoc thi dung cho nguoi ra cho thoang.

CHON DUNG NGUOI GIUA DO DAC (sai so huong ~15 do nen trong cung +-35 do thuong co ca do dac "giong chan nguoi")
  - Vat MOI xuat hien = nguoi vua di toi. Nen (canh vat dung yen) lay tu CA VONG XOAY do (du 360 do, ke ca cung mu
    sau duoi); nhom chan o cho lan truoc con trong duoc uu tien manh. => Luc bat dau: bam Enter roi MOI di ra vi tri.
  - Do xong ma co nhom chan moi xuat hien so voi nua dau lan do = co nguoi DI CHUYEN luc xe dang xoay -> huong vua do
    khong tin duoc (nguoi dang di thi 30 % lan do sai > 30 do) -> do lai. Tru khi chi co DUNG MOT nhom moi, lech huong
    beacon <= 25 do va da dung yen o do: chinh la nguoi deo beacon vua di toi -> nhan ket qua, uu tien nhom do.
  - Muc tieu chua he nhuc nhich tu luc khoa (co the la do dac), hoac dang dung dung cho lan quet truoc co vat dung yen
    (LiDAR da truot sang vat / nguoi dung yen), ma co nguoi moi di toi trong cung +-35 do quanh huong beacon -> chuyen.
  - RSSI KHONG phan biet duoc hai vat DUNG YEN cach nhau < ~40 do nhin tu xe, va khong do duoc khoang cach: vat nam
    gan duong ngam toi chu va gan xe hon (nhat la che kin chan chu) co the bi khoa nham cho toi khi chu di chuyen.

KHI XE KHONG TIEN DUOC / KHONG XOAY DUOC
  - Planner dung im hoac lac tai cho khi chua toi gan muc tieu (loi da biet cua planner, CLAUDE.md 13.12) -> script
    lui ra + quay mat ve muc tieu roi bat lai planner (toi da 2 lan), sau do kiem tra lai muc tieu bang RSSI.
  - Het cho xoay do (vat trong 0.53 m quanh tam quay) -> lui ra theo cho vua di qua; khong lui duoc thi nho planner
    nhich toi cho thoang hon; van khong duoc thi dung cho roi thu lai (khong thoat chuong trinh).

CAN 2 TERMINAL (deu da source ROS + install/setup.bash cua repo nay, source ~/rssi_env.sh)
-------------------------------------------------------------------------------------------
  T1: ros2 launch person_follow_nav rssi_follow.launch.py lidar_port:=$PL ports:="$PA,$PB,$PC"
  T2: python3 rssi_follow.py

AN TOAN
-------
  - /cmd_vel chi co MOT nguon: script nay. Planner bi doi topic sang /cmd_vel_follow (trong launch); script tu
    choi chay neu thay node khac ghi /cmd_vel.
  - Khi bam: moi lenh la cua planner (da loc va cham bang footprint chu nhat). Khi do: chi xoay tai cho va chi
    khi quanh tam quay trong > 0.53 m. Lenh rieng cua script chi co: xoay tai cho, va LUI THANG 0.06 m/s toi da
    0.45 m — chi lui khi dai LiDAR khong thay sau duoi xe nam tron trong vung than xe vua di qua (60 s gan day) va
    hai ben duoi xe khong co diem LiDAR nao.
  - /scan cu qua 0.6 s, mat /odom, mat beacon, planner ngung -> DUNG.
  - DUNG KHAN: Ctrl-C o T2 (nhanh nhat). Tu terminal khac:  ros2 service call /follow/stop std_srvs/srv/Trigger {}
    (co tac dung MOI LUC: o rssi_follow.launch.py script nhan /follow/stop, service dung cua planner doi ten thanh
    /rssi_follow/planner_stop). /rssi_follow/stop cung vay.
  - LiDAR cao 18 cm: KHONG thay mat ban, bac them, day dien; MU thang phia sau.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # goc package
from rssi_seek import B, CLEAR_MIN_M, FRONT_LEN, G, HALF_WIDTH, LIDAR_X, R, REAR_LEN, X, Y, Seeker, Stop  # noqa: E402
from person_follow_nav.geometry import AlphaBeta2D, cluster_points  # noqa: E402
from person_follow_nav.rssi_df import wrap  # noqa: E402

CMD_FOLLOW = "/cmd_vel_follow"      # planner ghi vao day (tham so cmd_vel_topic trong rssi_follow.launch.py)
SRC = "rssi+lidar"                  # nhan nguon trong /follow/target (khong bat dau bang "camera")

# Bam nhom chan — cac gia tri cua target_tracker_node trong config/follow_nav.yaml (da chay tren xe o nguon
# camera+lidar / lidar_track). Doi o do thi doi o day.
CLUSTER_GAP_M = 0.18
PERSON_W_MIN, PERSON_W_MAX = 0.08, 0.90
PERSON_R_MIN = 0.30
ASSOC_WINDOW_DEG = 10.0
ASSOC_RADIUS_M = 0.55
OCCLUDER_MARGIN_M = 0.30
TRACK_OCCLUDER_MARGIN_M = 0.15
AB_ALPHA, AB_BETA, MAX_PERSON_SPEED = 0.55, 0.10, 1.8
VEL_DECAY_SEC = 1.0
MEAS_HOLD_SEC = 0.3
FOLLOW_DISTANCE_M = 1.0             # follow_distance_m cua planner — chi dung de dat diem dich cho planner

# Rieng cho bam CHI BANG LiDAR (khong co camera sua sai) — rut ra tu mo phong vong kin 02/10:
LEG_MERGE_M = 0.32                  # hai cum cach nhau duoi ngan nay = hai chan cua MOT nguoi (hoac mot chan bi vo doi)
MIN_UPDATE_DT = 0.10                # hai phep do sat nhau hon chu ky LiDAR: he so beta/dt bien 10 cm rung thanh ~1 m/s
TRACK_KEEP_SEC = 0.32               # bam tren 3 vong quet GOP lai: mot vong don mat ~40 % tia lam TUONG vo thanh nhieu
#                                     manh "giong chan nguoi", muc tieu truot doc theo tuong (thay trong mo phong)
GATE_MIN_M, GATE_GROW_MPS = 0.30, 0.5   # cong ghep quanh du doan: 0.30 m + 0.5 m/s x thoi gian chua co phep do (<= 0.55)
GOTO_LAST_MAX_SEC = 8.0             # mat dau: lai toi gan cho thay cuoi toi da ngan nay roi moi do lai
BLOCK_KEEP_M = 0.9                  # lai theo huong beacon: dung cach vat chan it nhat ngan nay (con cho xoay do)
LEVEL_JUMP_DB = 8.0                 # xe + muc tieu dung yen ma >= 2 board doi qua ngan nay trong 3 s = beacon da di cho khac
NEW_OBJ_M = 0.22                    # nhom cach moi diem cua NEN (lan quet truoc) hon ngan nay = vat MOI xuat hien = nguoi vua di toi
BG_MASK_M = 0.18                    # khi bam: cum trung voi NEN (do dac dung yen) trong ngan nay thi khong tinh la chan nguoi
BG_EXCLUDE_M = 0.30                 # luc chup nen: bo cac diem quanh muc tieu trong ngan nay (do la chan nguoi, khong phai nen)
BG_MATCH_MIN = 0.80                 # so nen: ti le diem khop toi thieu (chi tren phan nen da nhin thay) — xem new_vs_background
SEED_MIN_M, SEED_MAX_M = 0.28, 0.45  # chi lay cac cum trong ban kinh nay quanh du doan lam phep do (no dan khi chua co phep do)
REJECT_M = 0.50                     # khong khoa lai vat da bi RSSI bac bo (trong ban kinh nay, khung odom)
REJECT_KEEP_SEC = 180.0             # ... trong ngan nay giay (het han thi duoc xet lai: biet dau lan bac bo do la do RSSI sai)
PLANNER_TICK_SEC = 0.25             # > 2 chu ky dieu khien cua planner (15 Hz)
# Be rong vung xet "canh bong" ngoai moi mep nhom. Manh tuong nhin qua khe giua hai chan nguoi luon nam SAT mep hai
# chan (khe chan ~7 cm) -> 3 do du ca khi mat 2 tia o mep. 6 do (truoc 03/10) loai luon CHU dung lot giua hai vat gan
# xe hon cach mep chan ~5 do moi ben (mo phong: hai thung tren duong toi chu o 3.2 m) -> chi con thung de chon.
SHADOW_SIDE_RAD = math.radians(3.0)
# Lui ra khi het cho xoay do (vat trong 0.53 m quanh tam quay): CHI lui thang, cham, trong vung than xe VUA quet qua
BACK_V = 0.06                       # = |v_min| cua planner
BACK_MAX_M = 0.45                   # ~ gap_back_max_m cua planner (0.40): LiDAR MU thang phia sau
BACK_STEP_M = 0.05                  # moi nhip xet truoc doan lui ngan nay
BLIND_HALF_RAD = math.radians(24.0 + 4.0)   # nua cung mu phia sau (blind_sectors_deg 246..294 = +-24 do) + 4 do du phong
TRAIL_STEP_M, TRAIL_KEEP_SEC = 0.04, 60.0
REJECT_NOW_DEG = 75.0               # huong beacon lech muc tieu qua ngan nay -> bo ngay; 40..75 do thi do lai lan nua da
VERIFY_RETRY_SEC = 20.0             # khong kiem tra duoc (het cho xoay, muc tieu dang di): thu lai sau ngan nay
VOX_M, ACC_STATIC_SEC = 0.06, 0.8    # nen 360 do gom tu vong xoay do: o luoi 6 cm, phai duoc thay trai ra >= 0.8 s
# Da toi noi (cach muc tieu ~1 m, muc tieu dung yen, planner da ra lenh dung): GIU XE DUNG IM cho toi khi muc tieu lech
# han / di tiep. Khong lam the thi xe cu nhuc nhich xoay qua lai sau lung nguoi: planner giu huong trong +-4 do (de giu
# nguoi giua khung hinh camera — o day khong can) ma lenh xoay nho nhat cua driver la ~0.2 rad/s nen lan nao cung troi
# qua; trong phong nhieu do dac dong tac chui khe con bat nham "khe" giua hai mon do roi xoay ve do (mo phong 03/10).
HOLD_IN = (FOLLOW_DISTANCE_M + 0.15, math.radians(6.0), 0.10)     # vao: cach <=, lech huong <=, toc do muc tieu <
HOLD_OUT = (FOLLOW_DISTANCE_M + 0.22, math.radians(12.0), 0.15)   # ra:  mot trong ba vuot nguong
STALL_SEC = 5.0                     # planner ra lenh dung im lau hon ngan nay khi chua toi gan muc tieu = ket (CLAUDE.md 13.12)
STALL_WIN_SEC = 8.0                 # ... hoac suot ngan nay giay xe khong roi khoi cho dang dung qua 12 cm (lac tai cho)
STALL_BACK_M = 0.25                 # go ket: lui it nhat ngan nay (neu duoc phep) roi quay mat ve muc tieu, bat lai planner
STALL_MAX_TRIES = 2                 # so lan go ket lien tiep cho mot muc tieu; qua thi dung cho nguoi / vat can doi cho
# KHOA LAI NHANH sau khi mat dau (chay that 06/10: 3 lan mat dau, moi lan ton 18-34 s di toi cho thay cuoi + xoay do lai,
# trong khi nguoi van dang di ngay gan do — vd. di lai sat xe roi vong doc hong xe ra sau). Vua mat dau ma gan cho du doan
# co DUNG MOT nhom chan DANG DI va khong co ai khac de nham -> khoa lai luon, kiem tra bang RSSI o lan dung yen dau tien.
# Cho dong nguoi gan nhu luon co nguoi khac quanh do -> khong khoa nhanh, xoay do nhu cu.
REACQ_SEC = 3.0                     # chi xet trong ngan nay giay sau khi mat dau
# Tim quanh DOAN DUONG nguoi co the da di: tu cho thay cuoi, keo dai theo van toc luc do (nguoi bi che thuong van di tiep:
# mo phong 06/10, tim quanh cho du doan da giam toc thi nguoi di 0.4 m/s ra khoi vung tim truoc khi kip khoa).
REACQ_R_M, REACQ_R_GROW, REACQ_R_MAX_M = 0.6, 0.25, 1.6   # ban kinh tim: 0.6 m + 0.25 m moi giay tu lan cuoi thay, <= 1.6
REACQ_PATH_SEC = 3.0                # doan duong keo dai toi da ngan nay giay chuyen dong (xa hon thi vung tim qua rong)
REACQ_GUARD_M = 0.8                 # ngoai vung tim them ngan nay KHONG duoc co nhom chan dang di nao khac
REACQ_NEAR_M = 0.6                  # trong ngan nay quanh cho thay cuoi / quanh nhom do khong co nhom 'giong nguoi' dung yen
#                                     (co the chinh la chu dang DUNG, vua bi nguoi khac di ngang che mat)
REACQ_HOLD_SEC = 0.6                # cac dieu kien tren phai dung lien tuc ngan nay (nguoi bi che co the hien ra ngay sau)
REACQ_MAX = 2                       # toi da ngan nay lan khoa nhanh lien tiep chua co RSSI xac nhan; mat lan nua thi xoay do
MOVE_BACK_SEC = (1.2, 3.0)          # "dang di" = cho nhom dang dung bay gio, cach day 1.2-3 s con TRONG (tia di xuyen qua)
MOVE_OCC_M = 0.25                   # luc do co diem LiDAR trong ngan nay quanh cho do = da co vat o day (khong phai dang di)
# Do xong ma co DUNG MOT nhom chan moi xuat hien trong luc do, no lech huong beacon trong ngan nay va da DUNG YEN o do (it
# nhat ~0.8 s) -> do chinh la nguoi deo beacon vua di toi: nhan ket qua do, uu tien nhom do. Truoc 06/10 luon do lai —
# chay that ton them 24 s cho 3 lan do lai ma ca 3 lan nhom moi chi lech huong beacon 1-17 do. Co >= 2 nhom moi, nhom moi
# lech xa, hoac nhom do van dang di (nguoi khac di ngang qua) -> van do lai nhu cu.
MOVED_OK_DEG = 25.0
MOVED_STILL_SEC = (0.8, 1.6)        # "da dung yen": cach day ngan nay giay cho do da co vat (xem moving())


class Follower(Seeker):

    def __init__(self, args) -> None:
        super().__init__(args, name="rssi_follow")
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String

        self.String = String
        self.flt = AlphaBeta2D(AB_ALPHA, AB_BETA, MAX_PERSON_SPEED)
        self.trk = False                 # dang giu mot muc tieu
        self.t_fix = 0.0                 # lan cuoi LiDAR khop duoc nhom chan
        self.t_verified = 0.0            # lan cuoi RSSI xac nhan muc tieu nay (luc khoa cung tinh)
        self.verify_wait = float(args.verify_sec)
        self.level_wait = 8.0
        self.n_ambig = 0                 # so vong quet co >= 2 nhom "giong nguoi" quanh muc tieu (de nham sang nguoi khac)
        self.goal = None                 # diem dich tam (odom) cho planner khi chua co muc tieu: (x, y, nhan nguon)
        self.bg = None                   # nen de biet vat nao MOI xuat hien: (dam diem odom, vi tri xe luc lay) — lay tu ca
        #                                  vong xoay do (du 360 do), khong co thi chup tai cho
        self.bg_mask = None              # nen de KHONG BAM NHAM do dac: chup luc xe DUNG YEN (diem khong bi nhoe)
        self.rejected = []               # (ox, oy, luc bac bo): vat tung khoa nham, RSSI da bac bo
        self.verify_hold = 0.0           # khong thu kiem tra lai truoc thoi diem nay (lan truoc khong kiem tra duoc)
        self.n_stall = 0                 # so lan da go ket cho muc tieu nay ma chua toi gan duoc no
        self.acc = None                  # dang xoay do: cac vong quet (t, diem odom) de dung nen 360 do
        self.scan_cloud = None           # (dam diem tinh 360 do, vi tri xe) cua lan do vua xong
        self.scan_first = None           # ... chi nua dau lan do (de biet co ai di chuyen trong luc do)
        self.lock_new = False            # muc tieu dang bam luc khoa la "vat MOI xuat hien" (chac la nguoi vua di toi)
        self.just_rejected = False       # vua bac bo muc tieu -> lan chon ke tiep khong tru diem "vat cu" (pick_candidate)
        self.last_lost = None            # vi tri (odom) muc tieu luc mat dau
        self.ambig_lock = False          # luc khoa co >= 2 ung vien va khong biet cai nao moi -> kiem tra som
        self.scan_trk0 = None            # vi tri muc tieu luc bat dau mot lan do kiem tra
        self.hold_xy = None              # diem muc tieu dang phat cho planner (co do tre 8 cm)
        self.lock_xy = None              # vi tri luc khoa; moved = muc tieu da tu di xa nhat bao nhieu ke tu do
        self.moved = 0.0
        self.strike = 0                  # so lan kiem tra lien tiep thay huong beacon lech muc tieu
        self.scan_xy = None              # cho do thanh cong gan nhat (o do du cho xoay)
        self.block_wait = 6.0
        self.trail = []                  # cac tu the xe DA dung qua: (x, y, yaw, t), moi 4 cm / 8 do
        self.relay = False               # dang chuyen lenh planner -> /cmd_vel
        self.hold_still = False          # da toi noi: thay lenh planner bang lenh DUNG (xem HOLD_IN)
        self.t_pcmd = 0.0
        self.pstat = None
        self.t_pstat = 0.0
        self.cands = []
        self.recent = deque(maxlen=36)   # cac vong quet ~3.5 s gan nhat (t, x, y, yaw, diem base_link) — biet nhom nao DANG DI
        self.prefer = None               # (ox, oy): nhom chan moi duy nhat vua xuat hien trong lan do, dung huong beacon
        self.reacq = None                # (ox, oy): nhom chan vua tim duoc de khoa lai nhanh sau khi mat dau
        self.fix_xy, self.fix_v = (0.0, 0.0), (0.0, 0.0)   # vi tri / van toc muc tieu o lan LiDAR khop cuoi (cho lost_path)
        self.n_quick = 0                 # so lan khoa nhanh lien tiep tu lan khoa / xac nhan bang RSSI gan nhat
        self.pub_tgt = self.node.create_publisher(String, "/follow/target", 10)
        self.node.create_subscription(Twist, CMD_FOLLOW, self._pcmd, 10)
        self.node.create_subscription(String, "/follow/planner_status", self._pstat_cb, 10)
        self.en_cli = self.node.create_client(self.Trigger, "/follow/enable")
        self.dis_cli = self.node.create_client(self.Trigger, "/follow/disable")
        # DUNG KHAN tu terminal khac, co tac dung MOI LUC. Service dung cua planner chi dung duoc xe khi planner dang
        # lai (dang bam); luc script tu xoay do / lui thi planner von da tat -> goi no khong doi duoc gi (mo phong 03/10:
        # xe chay tiep 2 m). Nen rssi_follow.launch.py doi ten service do thanh /rssi_follow/planner_stop va script
        # nhan /follow/stop — ten lenh dung khan quen thuoc cua du an.
        self.node.create_service(self.Trigger, "/follow/stop", self._stop_srv)
        self.node.create_service(self.Trigger, "/rssi_follow/stop", self._stop_srv)

    # ── callback ─────────────────────────────────────────────────────────
    def _odom(self, m) -> None:
        super()._odom(m)
        if self.trail:
            lx, ly, lyaw, _lt = self.trail[-1]
            if math.hypot(self.x - lx, self.y - ly) < TRAIL_STEP_M and abs(wrap(self.yaw - lyaw)) < math.radians(8.0):
                return
        self.trail.append((self.x, self.y, self.yaw, self.t_odom))
        if len(self.trail) > 600:
            del self.trail[:150]

    def swept(self, bx: float, by: float) -> bool:
        """Diem (base_link hien tai) co nam trong vung THAN XE da dung qua trong TRAIL_KEEP_SEC gan day khong.
        Cho than xe vua chiem thi luc do chac chan trong."""
        ox, oy = self.to_odom(bx, by)
        now = time.time()
        for tx, ty, tyaw, tt in reversed(self.trail):
            if now - tt > TRAIL_KEEP_SEC:
                break
            c, s = math.cos(tyaw), math.sin(tyaw)
            dx, dy = ox - tx, oy - ty
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            if -REAR_LEN - 0.02 <= lx <= FRONT_LEN + 0.02 and abs(ly) <= HALF_WIDTH + 0.02:
                return True
        return False

    def back_out(self, why: str, min_dist: float = 0.0, watch: bool = False) -> bool:
        """LUI THANG cham cho toi khi quanh tam quay trong lai (va da lui it nhat min_dist). True = da du cho xoay.

        LiDAR MU thang phia sau (cung 246-294 do = +-24 do quanh duoi xe). Nen moi nhip, truoc khi lui them 5 cm:
          - dai giua sau duoi xe (phan LiDAR khong thay) phai nam TRON trong vung than xe da dung qua trong 60 s gan
            day — cung gia dinh "cho vua di qua thi trong" ma dong tac chui cua cua planner dung;
          - hai ben duoi xe (phan LiDAR thay) khong duoc co diem nao.
        Toi da 0.45 m, 0.06 m/s. Khong du dieu kien thi KHONG lui.
        watch=True (lui de KIEM TRA muc tieu dang bam): muc tieu bat dau di / mat dau thi thoi ngay, bam tiep."""
        self.state = "LUI RA"
        x0, y0, t0 = self.x, self.y, time.time()
        ref = (self.flt.x, self.flt.y)
        self.say(f"{why} — lui cham theo cho xe vua di qua.", Y)
        ok, stop = False, "da du"
        yb = min(HALF_WIDTH, (REAR_LEN + BACK_STEP_M + LIDAR_X) * math.tan(BLIND_HALF_RAD))
        while True:
            self.guard()
            done = math.hypot(self.x - x0, self.y - y0)
            if done >= min_dist and self.rotate_clear()[1] >= CLEAR_MIN_M + 0.03:
                ok = True
                break
            if watch and (time.time() - self.t_fix > self.a.lost_sec
                          or math.hypot(self.flt.x - ref[0], self.flt.y - ref[1]) > 0.35):
                stop = "muc tieu dang di chuyen"
                break
            if done >= BACK_MAX_M or time.time() - t0 > BACK_MAX_M / BACK_V + 4.0:
                stop = "da lui het muc cho phep"
                break
            p = self.points()
            if len(p) and ((p[:, 0] < -0.05) & (p[:, 0] > -(REAR_LEN + 0.25)) & (np.abs(p[:, 1]) <= HALF_WIDTH + 0.06)).any():
                stop = "LiDAR thay vat o hai ben duoi xe"
                break
            if not all(self.swept(-(REAR_LEN + BACK_STEP_M), float(v)) for v in np.linspace(-yb, yb, 7)):
                stop = "phia sau khong phai cho xe vua di qua (LiDAR mu phia sau)"
                break
            self.cmd(-BACK_V, 0.0)
        self.halt()
        self.say(f"da lui {math.hypot(self.x - x0, self.y - y0):.2f} m — "
                 + ("du cho xoay." if ok else f"VAN chua du cho xoay ({stop})."), G if ok else Y)
        return ok

    def _stop_srv(self, _req, res):
        self.stop_req = True             # spin() nem KeyboardInterrupt o cho an toan -> main() dung xe, tat planner
        res.success, res.message = True, "rssi_follow: DUNG KHAN"
        return res

    def _pcmd(self, m) -> None:
        self.t_pcmd = time.time()
        if self.relay and self.pub is not None:
            # lenh cua planner di thang ra xe, khong sua — tru khi da toi noi thi thay bang lenh DUNG (luon an toan)
            self.pub.publish(self.Twist() if self.hold_still else m)

    def _pstat_cb(self, m) -> None:
        try:
            self.pstat = json.loads(m.data)
            self.t_pstat = time.time()
        except json.JSONDecodeError:
            pass

    def call(self, cli) -> bool:
        if not cli.service_is_ready():
            return False
        fut = cli.call_async(self.Trigger.Request())
        t = time.time()
        while not fut.done() and time.time() - t < 1.0:
            self.spin(0.02)
        return fut.done()

    # ── hinh hoc ─────────────────────────────────────────────────────────
    def to_base(self, ox: float, oy: float):
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        dx, dy = ox - self.x, oy - self.y
        return c * dx - s * dy, s * dx + c * dy

    def to_odom(self, bx: float, by: float):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * bx - s * by, self.y + s * bx + c * by

    def person_groups(self, pts: np.ndarray, b0: float, half: float, min_pts: int):
        """Cac nhom diem 'giong nguoi' trong cung b0 +- half -> [(cx, cy, khoang cach, be rong, so diem, cac cum thanh
        vien [(x, y, so diem, khoang cach)])] (base_link).

        1. Gom cum lien tuc (cluster_points) roi GOP cac cum cach nhau < LEG_MERGE_M thanh mot nhom: hai chan nguoi,
           hoac mot chan bi vo lam doi vi mat tia. Bam tung cum rieng le thi diem do nhay giua chan trai / chan phai
           (+-10 cm moi vong quet) va cum hep hon 8 cm bi loai -> mat dau gia. Xet be rong tren CA NHOM: chan ghe don
           le (3-4 cm) van bi loai; tuong (ke ca tuong nhin cheo, diem thua) noi thanh chuoi dai > 0.9 m nen bi loai.
        2. Bo nhom bi KEP giua hai vat o gan xe hon (ngay ngoai hai mep nhom deu co diem gan hon han): do la mot MANH
           NEN (tuong, tu) nhin thay qua khe giua hai vat che — vd. khuc tuong giua hai chan nguoi — chu khong phai
           mot vat dung rieng. Khoa vao no thi muc tieu "truot" doc tuong khi xe di chuyen (thay trong mo phong).
        Xoay khung cho b0 ve 0 truoc khi gom: cluster_points sap theo goc tho, cung vat qua +-180 do thi bi cat doi."""
        if not len(pts):
            return []
        c, s = math.cos(-b0), math.sin(-b0)
        q = np.stack([c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]], 1)
        bear, rng = np.arctan2(q[:, 1], q[:, 0]), np.hypot(q[:, 0], q[:, 1])
        cl = [k for k in cluster_points(q, bear, rng, 0.0, half, gap_threshold_m=CLUSTER_GAP_M, min_points=1)
              if k.width_m <= PERSON_W_MAX]
        parent = list(range(len(cl)))

        def find(i: int) -> int:
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        for i in range(len(cl)):
            for j in range(i + 1, len(cl)):
                if math.hypot(cl[i].cx - cl[j].cx, cl[i].cy - cl[j].cy) < LEG_MERGE_M:
                    parent[find(i)] = find(j)
        out = []
        c, s = math.cos(b0), math.sin(b0)
        for g in set(find(i) for i in range(len(cl))):
            mem = [cl[i] for i in range(len(cl)) if find(i) == g]
            n = sum(k.count for k in mem)
            cx = sum(k.cx * k.count for k in mem) / n
            cy = sum(k.cy * k.count for k in mem) / n
            width = max(math.hypot(u.cx - v.cx, u.cy - v.cy) + 0.5 * (u.width_m + v.width_m) for u in mem for v in mem)
            if n < min_pts or not (PERSON_W_MIN <= width <= PERSON_W_MAX):
                continue
            r_g = math.hypot(cx, cy)
            lo = min(k.bearing - (0.5 * k.width_m + 0.03) / max(0.3, k.range_m) for k in mem)
            hi = max(k.bearing + (0.5 * k.width_m + 0.03) / max(0.3, k.range_m) for k in mem)
            nearer = rng < r_g - 0.25
            if (nearer & (bear > hi) & (bear <= hi + SHADOW_SIDE_RAD)).any() and \
                    (nearer & (bear < lo) & (bear >= lo - SHADOW_SIDE_RAD)).any():
                continue
            out.append((c * cx - s * cy, s * cx + c * cy, r_g, width, n,
                        [(c * k.cx - s * k.cy, s * k.cx + c * k.cy, k.count, k.range_m) for k in mem]))
        return out

    # ── bam nhom chan bang LiDAR (moi vong quet) ─────────────────────────
    def is_background(self, bx: float, by: float) -> bool:
        """Diem (base_link) co trung voi do dac dung yen da chup lam nen khong.

        Dung anh chup luc xe DUNG YEN (bg_mask), khong dung nen 360 do gom luc xe dang xoay: LiDAR that quet mot vong
        mat 0.1 s, xe xoay 0.86 rad/s thi moi vat bi nhoe toi ~5 do (0.17 m o 2 m) — de nhan "vat moi" thi khong sao
        (chi lam than trong hon), nhung lam mat na khi bam thi vung "do dac" phinh ra, nguoi di sat do dac la mat dau."""
        if self.bg_mask is None:
            return False
        ox, oy = self.to_odom(bx, by)
        return bool(np.hypot(self.bg_mask[0][:, 0] - ox, self.bg_mask[0][:, 1] - oy).min() < BG_MASK_M)

    def moving(self, ox: float, oy: float, back=MOVE_BACK_SEC):
        """Cho (ox, oy) odom — noi mot nhom chan dang dung bay gio — cach day `back` giay ra sao:
          True  = it nhat 2 vong quet thay cho do TRONG (tia LiDAR di xuyen qua, gap vat o xa hon) va khong vong nao thay
                  vat o do -> nhom nay vua DI TOI (dang di);
          False = luc do da co vat o day (do dac, nguoi dung yen);
          None  = luc do khong nhin thay cho nay (bi che, cung mu sau duoi xe, ngoai tam).
        Moi vong quet cu xet theo CHINH tu the xe luc do (xe dang chay / xoay). Do dac vua lot vao tam nhin (truoc do bi
        che / nam trong cung mu) -> None chu khong phai True: khong bao gio bi coi la "dang di"."""
        now = time.time()
        n_free = 0
        for t, x, y, yaw, p in self.recent:
            if not (back[0] <= now - t <= back[1]) or not len(p):
                continue
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = ox - x, oy - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy           # cho do trong base_link cua vong quet cu
            if (np.hypot(p[:, 0] - lx, p[:, 1] - ly) < MOVE_OCC_M).any():
                return False
            dg = math.hypot(lx - LIDAR_X, ly)
            if dg < PERSON_R_MIN:
                continue
            rx, ry = p[:, 0] - LIDAR_X, p[:, 1]
            cone = np.abs(wrap(np.arctan2(ry, rx) - math.atan2(ly, lx - LIDAR_X))) <= max(
                math.radians(2.0), math.atan2(0.10, dg))
            rp = np.hypot(rx[cone], ry[cone])
            if len(rp) and (rp > dg + MOVE_OCC_M).any() and not (rp < dg - MOVE_OCC_M).any():
                n_free += 1
        return True if n_free >= 2 else None

    def reacq_candidate(self, seg, r_m: float):
        """Vua mat dau: nhom chan de KHOA LAI NGAY (khong xoay do) -> (ox, oy) odom | None.
        seg = ((ax, ay), (bx, by)) odom: doan tu cho thay cuoi toi cho nguoi se toi neu cu di tiep nhu luc do (lost_path);
        r_m = ban kinh tim quanh doan do.

        Chi khi chac la MOT nguoi dang di o do va khong co ai khac de nham:
          - nhom 'giong nguoi' DANG DI (moving), cach doan duong <= r_m, khong phai do dac da biet (bg_mask), khong gan xe
            hon diem gan nhat cua doan duong qua OCCLUDER_MARGIN_M (vat CHE: nguoi khac di ngang giua xe va chu);
          - trong r_m + REACQ_GUARD_M quanh doan duong KHONG co nhom dang di nao khac (hai nguoi cung di -> khong biet ai);
          - trong REACQ_NEAR_M quanh cho thay cuoi / quanh nhom do KHONG co nhom 'giong nguoi' nao khong di chuyen (chu co
            the dang DUNG o do — vua bi nguoi khac di ngang che mat).
        Van co the sai khi chu bi che kin ma dung mot nguoi khac di qua gan duong di cua chu -> sau khi khoa nhanh, kiem
        tra bang RSSI ngay lan dung yen dau tien (ambig_lock)."""
        (ax, ay), (bx, by) = seg
        L2 = (bx - ax) ** 2 + (by - ay) ** 2

        def near_pt(ox: float, oy: float):          # diem gan nhat tren doan duong
            k = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((ox - ax) * (bx - ax) + (oy - ay) * (by - ay)) / L2))
            return ax + k * (bx - ax), ay + k * (by - ay)
        mx0, my0 = self.to_base(0.5 * (ax + bx), 0.5 * (ay + by))
        mov, still = [], []
        for cx, cy, rng, _w, _n, mem in self.person_groups(self.points(TRACK_KEEP_SEC), math.atan2(my0, mx0), math.pi, 2):
            ox, oy = self.to_odom(cx, cy)
            qx, qy = near_pt(ox, oy)
            d = math.hypot(ox - qx, oy - qy)
            if d > r_m + REACQ_GUARD_M or rng < PERSON_R_MIN:
                continue
            if all(self.is_background(mx, my) for mx, my, _c, _r in mem):
                continue                                    # do dac da chup luc xe dung yen
            if any(math.hypot(ox - rx, oy - ry) < REJECT_M for rx, ry, _rt in self.rejected):
                continue
            rq = math.hypot(*self.to_base(qx, qy))
            (mov if self.moving(ox, oy) else still).append((ox, oy, d, rng, rq))
        cand = [g for g in mov if g[2] <= r_m and g[3] >= g[4] - OCCLUDER_MARGIN_M]
        if len(cand) != 1 or len(mov) > 1:
            return None
        ox, oy = cand[0][0], cand[0][1]
        if any(math.hypot(g[0] - ax, g[1] - ay) <= REACQ_NEAR_M or math.hypot(g[0] - ox, g[1] - oy) <= REACQ_NEAR_M
               for g in still):
            return None
        return ox, oy

    def on_scan(self, pts: np.ndarray) -> None:
        if self.scans:
            self.recent.append(self.scans[-1])
        if self.acc is not None and len(pts):               # dang xoay do: gom diem (odom) de dung nen 360 do
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.acc.append((time.time(), np.stack([self.x + c * pts[:, 0] - s * pts[:, 1],
                                                    self.y + s * pts[:, 0] + c * pts[:, 1]], 1)))
        if not self.trk or not self.flt.initialized:
            return
        now = time.time()
        px, py = self.flt.predict(now)
        tx, ty = self.to_base(px, py)
        r0 = math.hypot(tx, ty)
        if r0 < 0.15:
            return
        age = now - self.t_fix
        gate = min(ASSOC_RADIUS_M, GATE_MIN_M + GATE_GROW_MPS * age)
        seed = min(SEED_MAX_M, SEED_MIN_M + GATE_GROW_MPS * age)
        window = max(math.radians(ASSOC_WINDOW_DEG), math.atan2(ASSOC_RADIUS_M + LEG_MERGE_M, max(0.3, r0)))
        # Vat gan xe hon du doan = vat CHE truoc nguoi (nguoi khac di ngang / chen vao), khong phai nguoi dang bam.
        # Nguong co gian theo toc do nguoi — nhu _nearest_cluster_to cua target_tracker_node.
        margin = min(OCCLUDER_MARGIN_M, TRACK_OCCLUDER_MARGIN_M + 0.25 * math.hypot(self.flt.vx, self.flt.vy))
        # Phep do = trong tam cac CUM (chan) nam trong ban kinh `seed` quanh du doan, thuoc nhom "giong nguoi", khong
        # trung voi nen. Khong lay trong tam ca NHOM: nguoi dang bam di sat nguoi khac / do dac thi chan hai ben noi
        # thanh mot nhom, trong tam roi vao giua va muc tieu truot sang ben kia (thay trong mo phong).
        sx = sy = sn = 0.0
        n_near = 0
        for _cx, _cy, _rng, _w, _n, mem in self.person_groups(self.points(TRACK_KEEP_SEC), math.atan2(ty, tx), window, 2):
            used = False
            for mx, my, cnt, mrng in mem:
                d = math.hypot(mx - tx, my - ty)
                if mrng < r0 - margin or self.is_background(mx, my):
                    continue
                used |= d < ASSOC_RADIUS_M
                if d < seed:
                    sx, sy, sn = sx + mx * cnt, sy + my * cnt, sn + cnt
            n_near += used
        if n_near >= 2:
            # Co nhom khac sat muc tieu (nguoi thu hai dung canh / di ngang, do dac la): LiDAR co the truot sang no.
            # Khong biet duoc ngay -> lan dung yen toi phai do lai bang RSSI som, khong cho het khoang gian thua.
            self.n_ambig += 1
            self.verify_wait = min(self.verify_wait, float(self.a.verify_sec))
        if sn < 3 or math.hypot(sx / sn - tx, sy / sn - ty) > gate:
            return
        ox, oy = self.to_odom(sx / sn, sy / sn)
        if self.flt.last_time is not None and now - self.flt.last_time < MIN_UPDATE_DT:
            self.flt.last_time = now - MIN_UPDATE_DT
        self.flt.update(ox, oy, now)
        self.t_fix = now
        self.fix_xy, self.fix_v = (self.flt.x, self.flt.y), (self.flt.vx, self.flt.vy)
        if self.lock_xy is not None:
            self.moved = max(self.moved, math.hypot(self.flt.x - self.lock_xy[0], self.flt.y - self.lock_xy[1]))

    def do_scan(self):
        self.scan_trk0 = (self.flt.x, self.flt.y) if self.trk else None
        # Dang bam (do de KIEM TRA) ma het cho xoay: khong dung cho lau — muc tieu co the di mat; lui ra roi do (xem run)
        self.block_wait = 2.0 if self.trk else 6.0
        # Gom diem LiDAR suot vong xoay: xe xoay tron nen cung mu sau duoi (+-24 do) quet qua moi huong -> dung duoc
        # NEN DU 360 DO (chup mot cho thi thieu cung mu, nguoi dung dung huong do se khong bao gio duoc coi la "moi").
        self.acc, t0 = [], time.time()
        try:
            phi = super().do_scan()
        finally:
            acc, self.acc = self.acc, None
        t1 = time.time()
        self.scan_cloud = self.scan_first = None
        if phi is not None:
            full, first = self.static_cloud(acc, t0, t1), self.static_cloud(acc, t0, t0 + 0.55 * (t1 - t0))
            self.scan_cloud = None if full is None else (full, (self.x, self.y))
            self.scan_first = None if first is None else (first, (self.x, self.y))
        return phi

    @staticmethod
    def static_cloud(acc, t_lo: float, t_hi: float):
        """Dam diem cua cac vat DUNG YEN (odom, N x 2) tu cac vong quet trong [t_lo, t_hi]: luoi 6 cm, chi giu o duoc thay
        trai ra >= 0.8 s (nguoi di ngang qua o nao thi chi o do ~0.15 s -> khong thanh "nen ma")."""
        sel = [(t, q) for t, q in acc if t_lo <= t <= t_hi]
        if not sel:
            return None
        xy = np.concatenate([q for _t, q in sel])
        tt = np.concatenate([np.full(len(q), t) for t, q in sel])
        key = np.round(xy / VOX_M).astype(np.int64)
        _u, inv = np.unique(key[:, 0] * 1000003 + key[:, 1], return_inverse=True)
        n = int(inv.max()) + 1
        t_min, t_max = np.full(n, np.inf), np.full(n, -np.inf)
        np.minimum.at(t_min, inv, tt)
        np.maximum.at(t_max, inv, tt)
        cnt = np.bincount(inv, minlength=n)
        keep = (t_max - t_min >= ACC_STATIC_SEC) & (cnt >= 3)
        if keep.sum() < 30:
            return None
        return np.stack([np.bincount(inv, xy[:, 0], n)[keep] / cnt[keep], np.bincount(inv, xy[:, 1], n)[keep] / cnt[keep]], 1)

    def fresh_bg(self, exclude=None):
        """NEN cho lan sau: dam diem 360 do cua vong xoay do vua xong; khong co thi chup tai cho (thieu cung mu).
        exclude = (ox, oy): bo cac diem quanh muc tieu (chan nguoi khong phai la nen)."""
        if self.scan_cloud is not None:
            q, pos = self.scan_cloud
            if exclude is not None:
                q = q[np.hypot(q[:, 0] - exclude[0], q[:, 1] - exclude[1]) > BG_EXCLUDE_M]
            if len(q) >= 30:
                return q, pos
        return self.snapshot(exclude)

    def scene_changed(self):
        """Sau mot lan do: co nhom 'giong nguoi' nao MOI xuat hien so voi NUA DAU cua lan do khong -> (x, y) base_link | None.

        Thuat toan do huong gia dinh nguoi deo beacon DUNG YEN suot ~8 s xe xoay. Mo phong voi mau do that (318 lan do):
        nguoi dung yen thi sai > 30 do chi 1 %; nguoi DANG DI trong luc do thi 30 % (toi 64 do) — va corr/margin
        KHONG lo ra dieu do. LiDAR thi thay: do xong ma co nguoi dung o cho nua dau con trong = ho vua di toi."""
        ch = self.scene_changes()
        return ch[0] if ch else None

    def scene_changes(self):
        """Nhu scene_changed() nhung tra ve TAT CA cac nhom MOI xuat hien -> [(x, y)] base_link."""
        if self.scan_first is None:
            return []
        groups = [g for g in self.person_groups(self.points(), 0.0, math.pi, 3)
                  if PERSON_R_MIN + 0.05 <= g[2] <= self.a.cand_range]
        return [(g[0], g[1]) for g, new in zip(groups, self.new_vs_background(groups, self.scan_first)) if new]

    def abort_scan(self) -> bool:
        """Dang do de KIEM TRA ma muc tieu di chuyen / mat dau: ket qua RSSI se sai (thuat toan gia dinh nguoi dung yen)
        va cang do lau cang tut lai phia sau -> bo lan do, bam tiep ngay."""
        if not self.trk or self.scan_trk0 is None:
            return False
        if time.time() - self.t_fix > self.a.lost_sec:
            self.say("mat dau trong luc do kiem tra — bo lan do nay.", Y)
            return True
        if math.hypot(self.flt.x - self.scan_trk0[0], self.flt.y - self.scan_trk0[1]) > 0.35:
            self.say("muc tieu di chuyen trong luc do kiem tra — bo lan do nay, bam tiep.", Y)
            return True
        return False

    def coast(self, now: float) -> None:
        """Khong co phep do: van toc giam dan de du doan dung gan cho thay cuoi (nhu target_tracker_node)."""
        if (self.trk and now - self.t_fix > MEAS_HOLD_SEC and self.flt.initialized
                and self.flt.last_time is not None):
            k = math.exp(-max(0.0, now - self.flt.last_time) / VEL_DECAY_SEC)
            self.flt.vx *= k
            self.flt.vy *= k
            self.flt.step(now)

    def lost_path(self, now: float):
        """Muc tieu vua mat dau -> (doan duong odom, ban kinh tim) cho reacq_candidate: tu cho LiDAR thay lan cuoi, keo dai
        theo van toc luc do (KHONG giam dan nhu coast(): nguoi bi che thuong van di tiep), ban kinh no dan theo thoi gian."""
        dt = max(0.0, now - self.t_fix)
        (ax, ay), (vx, vy) = self.fix_xy, self.fix_v
        k = min(dt, REACQ_PATH_SEC)
        return ((ax, ay), (ax + vx * k, ay + vy * k)), min(REACQ_R_MAX_M, REACQ_R_M + REACQ_R_GROW * dt)

    def snapshot(self, exclude=None):
        """(dam diem LiDAR trong khung odom, vi tri xe) luc xe dung yen — lam NEN: de biet vat nao MOI xuat hien o lan
        chon muc tieu sau, va de khong bam nham do dac dung yen. exclude = (ox, oy): bo cac diem quanh muc tieu (chan
        nguoi khong phai la nen)."""
        p = self.points()
        if len(p) < 30:
            return None
        # Giu TAT CA cac diem: bo bot diem thi vat manh (chan ban, chan ghe: 2-4 tia) co the mat khoi nen va lan sau
        # bi coi la "vat moi xuat hien".
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        q = np.stack([self.x + c * p[:, 0] - s * p[:, 1], self.y + s * p[:, 0] + c * p[:, 1]], 1)
        if exclude is not None:
            q = q[np.hypot(q[:, 0] - exclude[0], q[:, 1] - exclude[1]) > BG_EXCLUDE_M]
        return (q, (self.x, self.y)) if len(q) >= 30 else None

    def new_vs_background(self, groups, ref=None):
        """Voi tung nhom (base_link): True = vat MOI (lan truoc cho do TRONG, tia LiDAR di xuyen qua) -> nguoi vua di toi;
        False = da co tu truoc (do dac, hoac nguoi van dung cho cu); None = khong biet (lan truoc bi che / khong nhin toi).
        ref = nen de so (mac dinh self.bg).

        Odom dem thieu goc xoay ~1-3 % nen sau mot vong do, nen bi lech vai do: tim goc xoay quanh xe (+-15 do) cho hai
        dam diem khop nhau nhat roi moi so."""
        cur = self.snapshot()
        ref = self.bg if ref is None else ref
        if ref is None or cur is None:
            return [None] * len(groups)
        bg, (bx, by) = ref
        rel = cur[0] - np.array([self.x, self.y])
        rel = rel[:: len(rel) // 300 + 1]                   # chi de KHOP GOC thi lay thua diem HIEN TAI cho nhe (nen giu du)
        r_bg = np.hypot(bg[:, 0] - bx, bg[:, 1] - by)
        th_bg = np.arctan2(bg[:, 1] - by, bg[:, 0] - bx)
        # Chi so tren phan ma luc chup nen NHIN THAY duoc: vat gan nhat cua nen trong tung o goc 1 do (+-1 o), nhin tu cho
        # chup nen. Diem hien tai o huong ma luc do bi che (nen co vat gan hon), nam trong cung mu sau duoi, hoac khong co
        # diem nen nao -> khong tinh. Truoc 06/10 tinh ca, nguong 0.45: chay that, nguoi dung sat hong xe luc bam Enter che
        # mot mang tam nhin, xe lai da quay 129 do (cung mu doi cho) -> chi khop 0.43-0.46 -> moi ung vien thanh "?".
        # Nguong moi BG_MATCH_MIN chon tren cac cap vong quet cua bag that 06/10 (dung odom that vs co y lam lech):
        #   nen = anh chup 5 vong (260 cap): nhan dung 94 % (cu 93 %), nhan nham khi lech 25 do 2 % (cu 8 %), dich 0.3 m 5 % (cu 15 %)
        #   nen = dam diem 360 do luc xoay do (200 cap): 98 % (cu 99 %), lech 25 do 36 % (cu 56 %), dich 0.3 m 34 % (cu 66 %),
        #   lech 40 do 0 % (cu 9 %). Lan chay 1 that: 1.00 (cu 0.44). Ha nguong xuong 0.45 voi cach tinh moi thi nhan nham
        #   toi 93 % — DUNG ha.
        near = np.full(360, np.inf)
        np.minimum.at(near, np.floor(np.degrees(th_bg)).astype(int) % 360, r_bg)
        near = np.minimum(np.minimum(near, np.roll(near, 1)), np.roll(near, -1))

        def score(deg: float) -> float:
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            q = np.stack([self.x + c * rel[:, 0] - s * rel[:, 1], self.y + s * rel[:, 0] + c * rel[:, 1]], 1)
            hit = np.sqrt(((q[:, None, :] - bg[None, :, :]) ** 2).sum(2)).min(1) < 0.12
            nk = near[np.floor(np.degrees(np.arctan2(q[:, 1] - by, q[:, 0] - bx))).astype(int) % 360]
            seen = np.isfinite(nk) & (nk >= np.hypot(q[:, 0] - bx, q[:, 1] - by) - 0.12)
            m = int((hit | seen).sum())
            return float(hit.sum()) / m if m >= 40 else 0.0
        best = max(np.arange(-15.0, 15.1, 1.0), key=score)
        best = max(np.arange(best - 0.75, best + 0.76, 0.25), key=score)
        if score(best) < BG_MATCH_MIN:                      # canh vat doi qua nhieu / lech qua xa: khong tin phep so
            return [None] * len(groups)
        c, s = math.cos(self.yaw + math.radians(best)), math.sin(self.yaw + math.radians(best))
        out = []
        for g in groups:
            cx, cy = g[0], g[1]
            gx, gy = self.x + c * cx - s * cy, self.y + s * cx + c * cy
            if np.hypot(bg[:, 0] - gx, bg[:, 1] - gy).min() <= NEW_OBJ_M:
                out.append(False)                           # lan truoc da co vat o day
                continue
            dg = math.hypot(gx - bx, gy - by)
            cone = np.abs(wrap(th_bg - math.atan2(gy - by, gx - bx))) <= max(math.radians(3.0), math.atan2(0.15, max(0.3, dg)))
            # lan truoc tia LiDAR di XUYEN QUA cho nay (thay vat o xa hon) -> cho nay tung trong -> gio co vat = vat moi
            out.append(True if (cone.any() and (r_bg[cone] > dg + NEW_OBJ_M).any()
                                and not (r_bg[cone] < dg - NEW_OBJ_M).any()) else None)
        return out

    def pick_candidate(self, phi: float, learn: bool = True, after_reject: bool = False):
        """Nhom 'giong nguoi' hop voi huong beacon phi (odom) nhat trong tam LiDAR -> (cx, cy, lech do, moi?) base_link,
        hoac None.

        Sai so huong ~15 do nen trong cung +-35 do thuong co ca do dac. Phan biet bang: (1) vat MOI xuat hien so voi
        lan quet truoc = nguoi vua di toi; (2) nhom o ngay cho muc tieu vua mat; (3) bo vat da bi RSSI bac bo.
        learn=False: chi xem nhanh co ai khong (dang di), khong so nen va khong cap nhat nen.
        after_reject=True: vua bac bo muc tieu (bam nham). Trong luc xe bam nham, chu thuong DUNG YEN -> o lan quet nay
        chu la "vat cu" -> khong tru diem "vat cu" (mo phong 03/10: khong tru thi chon nham manh do dac mong)."""
        b0 = wrap(phi - self.yaw)
        groups = self.person_groups(self.points(), b0, math.radians(self.a.cand_win_deg + 25.0), 3)
        fresh = self.new_vs_background(groups) if (groups and learn) else [None] * len(groups)
        now = time.time()
        self.rejected = [r for r in self.rejected if now - r[2] < REJECT_KEEP_SEC]
        best, best_s, n_ok = None, 1e9, 0
        self.cands = []                                    # de ghi nhat ky: vi sao khoa / khong khoa
        for (cx, cy, rng, width, n, _mem), new in zip(groups, fresh):
            d = math.degrees(wrap(math.atan2(cy, cx) - b0))
            ox, oy = self.to_odom(cx, cy)
            rej = any(math.hypot(ox - rx, oy - ry) < REJECT_M for rx, ry, _rt in self.rejected)
            near_lost = self.last_lost is not None and math.hypot(ox - self.last_lost[0], oy - self.last_lost[1]) < 0.5
            # nhom chan moi DUY NHAT vua di toi trong lan do vua xong, dung huong beacon, da dung yen (xem MOVED_OK_DEG)
            pref = self.prefer is not None and math.hypot(ox - self.prefer[0], oy - self.prefer[1]) < 0.5
            self.cands.append([round(d, 1), round(rng, 2), round(width, 2), n,
                               "moi" if new else ("cu" if new is False else "?"), "bo" if rej else ""])
            # Vat MOI xuat hien la bang chung manh (phong tinh thi chi nguoi moi doi cho) -> cho lech huong beacon rong hon
            win = self.a.cand_win_deg + (25.0 if new or pref else 0.0)
            if not (PERSON_R_MIN + 0.05 <= rng <= self.a.cand_range) or abs(d) > win or rej:
                continue
            n_ok += 1
            # Diem = (lech huong beacon / 20 do)^2, cong tru theo bang chung khac:
            #   vat MOI xuat hien / dung cho vua mat dau -> uu tien manh;  vat da co tu lan quet truoc -> tru diem;
            #   be rong khong giong hai chan nguoi (do that 01/10: 0.21-0.29 m) -> tru nhe;  ngang nhau thi lay nhom gan.
            # 20 do chu khong 15: sai so do huong co sigma ~17 do; (d/15)^2 doc gap ~2.5 lan mo hinh do nen bang chung
            # "vat moi" bi lan at (chu moi di toi lech 33 do thua mot do dac cu lech 6 do). Cham lai 138 lan khoa trong
            # mo phong 03/10: 117 -> 121 lan chon dung chu (122 khi them after_reject), khong lan nao xau di.
            s = ((d / 20.0) ** 2 + rng / 6.0 + (1.2 if width < 0.16 else (0.6 if width > 0.60 else 0.0))
                 - (2.5 if new else 0.0) + (2.0 if new is False and not near_lost and not after_reject else 0.0)
                 - (1.0 if near_lost else 0.0) - (3.0 if pref else 0.0))
            if s < best_s:
                best, best_s = (cx, cy, d, bool(new)), s
        if learn:
            self.ambig_lock = best is not None and n_ok >= 2 and not best[3]
            # nen cho lan sau; nhom duoc chon la chan nguoi nen khong tinh vao nen
            excl = None if best is None else self.to_odom(best[0], best[1])
            self.bg, self.bg_mask = self.fresh_bg(excl), self.snapshot(excl)
        return best

    def new_candidate(self, phi: float, avoid):
        """Nhom 'giong nguoi' MOI xuat hien (so voi nen lan truoc), hop huong beacon phi, KHONG phai muc tieu dang bam
        (avoid = vi tri odom cua no) -> (cx, cy, lech do, True) base_link | None. Khong cap nhat nen."""
        b0 = wrap(phi - self.yaw)
        groups = self.person_groups(self.points(), b0, math.radians(self.a.cand_win_deg), 3)
        best = None
        for (cx, cy, rng, _width, _n, _mem), new in zip(groups, self.new_vs_background(groups) if groups else []):
            ox, oy = self.to_odom(cx, cy)
            if (not new or not (PERSON_R_MIN + 0.05 <= rng <= self.a.cand_range)
                    or math.hypot(ox - avoid[0], oy - avoid[1]) < 0.6
                    or any(math.hypot(ox - rx, oy - ry) < REJECT_M for rx, ry, _rt in self.rejected)):
                continue
            d = math.degrees(wrap(math.atan2(cy, cx) - b0))
            if best is None or abs(d) < abs(best[2]):
                best = (cx, cy, d, True)
        return best

    def on_background(self, ox: float, oy: float) -> bool:
        """Diem (odom) co trung voi mot vat DUNG YEN cua nen lan quet truoc khong. Nen luon bo vung quanh muc tieu luc chup
        (luc khoa / luc xac nhan), nen muc tieu ma nay dung DUNG CHO mot vat cua nen = LiDAR da truot tu nguoi dang di
        sang vat / nguoi dung yen o do (vd. chu di sat qua nguoi thu hai: mo phong 03/10)."""
        if self.bg is None:
            return False
        return bool(np.hypot(self.bg[0][:, 0] - ox, self.bg[0][:, 1] - oy).min() <= NEW_OBJ_M)

    def free_along(self, phi: float) -> float:
        """Khoang trong (m, tinh tu mui xe neu xe quay ve huong do) trong hanh lang doc huong phi (odom)."""
        p = self.points()
        if not len(p):
            return float("inf")
        b0 = wrap(phi - self.yaw)
        c, s = math.cos(-b0), math.sin(-b0)
        return self.front_gap(np.stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]], 1))

    def lock(self, cand, why: str = "") -> None:
        """cand = (cx, cy, lech huong beacon (do), vat moi?) base_link. why: ly do khoa (thay cho dong 'lech huong beacon')."""
        now = time.time()
        ox, oy = self.to_odom(cand[0], cand[1])
        self.flt.reset()
        self.flt.update(ox, oy, now)
        self.fix_xy, self.fix_v = (ox, oy), (0.0, 0.0)
        if self.bg_mask is not None:
            # Muc tieu khong bao gio duoc nam trong NEN: on_scan bo moi cum trung voi nen -> khoa xong la mat dau ngay
            keep = np.hypot(self.bg_mask[0][:, 0] - ox, self.bg_mask[0][:, 1] - oy) > BG_EXCLUDE_M
            self.bg_mask = (self.bg_mask[0][keep], self.bg_mask[1]) if keep.sum() >= 30 else None
        self.hold_xy = None
        self.n_stall, self.verify_hold, self.lock_new = 0, 0.0, bool(cand[3])
        self.lock_xy, self.moved, self.strike = (ox, oy), 0.0, 0
        self.trk, self.t_fix, self.t_verified, self.n_ambig = True, now, now, 0
        # Co nhieu ung vien ma khong biet cai nao la nguoi vua di toi -> kiem tra bang RSSI ngay lan dung yen dau tien
        self.verify_wait, self.level_wait = (0.0 if self.ambig_lock else float(self.a.verify_sec)), 8.0
        detail = why or (f"lech huong beacon {cand[2]:+.0f} do" + (", vat MOI xuat hien" if cand[3] else "")
                         + (", con ung vien khac -> se kiem tra som" if self.ambig_lock else ""))
        self.say(f"KHOA: nhom chan o {math.degrees(math.atan2(cand[1], cand[0])):+.0f} do so voi mui xe, cach "
                 f"{math.hypot(cand[0], cand[1]):.2f} m ({detail})", G, lock=[round(ox, 3), round(oy, 3)], cands=self.cands)

    # ── /follow/target ───────────────────────────────────────────────────
    def pub_target(self, now: float) -> None:
        out = {"stamp": now, "status": "ok", "odom_frame": "odom", "measured_this_tick": False,
               "robot": {"x": round(self.x, 4), "y": round(self.y, 4), "yaw": round(self.yaw, 4)}}
        if self.goal is not None:                           # diem dich tam (chua co muc tieu that)
            ox, oy, src = self.goal
            valid, conf, age, vx, vy = True, 0.5, 0.0, 0.0, 0.0
        else:
            age = now - self.t_fix
            valid = bool(self.trk and self.flt.initialized and age <= self.a.lost_sec)
            ox, oy = self.flt.predict(now)
            # Nguoi dung yen: diem do van rung 2-3 cm moi vong quet; o cach 1 m du de vuot vung chet huong 4 do cua
            # planner -> xe cu xoay nhe qua lai. Chi doi diem da phat khi muc tieu lech khoi no qua 8 cm.
            # Chi lam the khi xe da toi gan: luc con xa, diem dich dung im tuyet doi lam DWA de ket o cuc tieu dia phuong.
            if (self.hold_xy is not None and math.hypot(ox - self.hold_xy[0], oy - self.hold_xy[1]) < 0.08
                    and math.hypot(ox - self.x, oy - self.y) <= FOLLOW_DISTANCE_M + 0.35):
                ox, oy = self.hold_xy
            else:
                self.hold_xy = (ox, oy)
            src = SRC if age <= MEAS_HOLD_SEC else "predicted"
            conf = 0.9 if age <= MEAS_HOLD_SEC else max(0.2, 1.0 - age / max(0.2, self.a.lost_sec))
            vx, vy = self.flt.vx, self.flt.vy
        if valid:
            bx, by = self.to_base(ox, oy)
            b = math.atan2(by, bx)
            out.update({"valid": True, "source": src, "confidence": round(conf, 3), "age_since_fix_sec": round(age, 3),
                        "odom_x": round(ox, 4), "odom_y": round(oy, 4), "vx": round(vx, 3), "vy": round(vy, 3),
                        "speed": round(math.hypot(vx, vy), 3), "base_x": round(bx, 4), "base_y": round(by, 4),
                        "distance_m": round(math.hypot(bx, by), 3), "bearing_rad": round(b, 4),
                        "bearing_deg": round(math.degrees(b), 2), "in_camera_fov": False})
        else:
            out.update({"valid": False, "source": "none", "confidence": 0.0, "age_since_fix_sec": None,
                        "odom_x": None, "odom_y": None, "vx": 0.0, "vy": 0.0, "speed": 0.0, "base_x": None,
                        "base_y": None, "distance_m": None, "bearing_rad": None, "bearing_deg": None,
                        "in_camera_fov": False})
        self.pub_tgt.publish(self.String(data=json.dumps(out)))

    def trace(self, now: float) -> None:
        if now - self.t_rec < 0.5:
            return
        self.t_rec = now
        st = self.pstat or {}
        rec = {"pst": st.get("state"), "note": st.get("note"), "v": st.get("cmd_v"), "w": st.get("cmd_w"),
               "lv": (self.brg or {}).get("levels")}
        if self.trk and self.flt.initialized:
            ox, oy = self.flt.predict(now)
            rec.update({"trk": [round(ox, 3), round(oy, 3)], "age": round(now - self.t_fix, 2)})
        if self.goal is not None:
            rec["goal"] = [round(self.goal[0], 3), round(self.goal[1], 3), self.goal[2]]
        self.write(rec)

    # ── cac pha ──────────────────────────────────────────────────────────
    def planner_guard(self, now: float, t_en: float):
        """Loi cua planner trong luc dang chuyen tiep lenh -> ly do, hoac None."""
        if now - self.t_pstat > 1.0:
            raise Stop("planner khong con gui /follow/planner_status", 0.0)
        if now - t_en > 1.0 and (self.pstat or {}).get("enabled") is False:
            return "bi tat"                          # ai do goi /follow/stop hoac /follow/disable
        if now - self.t_pcmd > 0.5:
            self.pub.publish(self.Twist())           # planner ngung gui lenh -> giu xe dung
        return None

    def planner_idle(self) -> bool:
        st = self.pstat or {}
        return abs(st.get("cmd_v") or 0.0) < 1e-3 and abs(st.get("cmd_w") or 0.0) < 1e-3

    def end_relay(self) -> None:
        self.relay = self.hold_still = False
        self.halt()
        self.call(self.dis_cli)
        self.spin(PLANNER_TICK_SEC)      # cho planner chay it nhat mot nhip o trang thai TAT truoc khi bat lai

    def do_follow(self) -> str:
        """Chuyen lenh planner ra xe trong khi LiDAR bam nhom chan.
        Tra ve 'mat' | 'kiem tra' | 'muc doi' | 'ket' | 'bi tat'."""
        self.state = "BAM"
        if not self.call(self.en_cli):
            raise Stop("khong goi duoc /follow/enable (planner chua chay?)", 0.0)
        t_en = time.time()
        t_still = t_dev = base = t_stall = None
        t_warn = 0.0
        hist = deque([(t_en, self.x, self.y)])       # vi tri xe trong STALL_WIN_SEC gan day (moi 0.25 s)
        self.relay = True
        try:
            while True:
                self.guard()
                self.spin(0.05)
                now = time.time()
                self.coast(now)
                # Kiem tra TRUOC khi phat: khong bao gio de planner (dang bat) thay muc tieu "khong hop le" — no se tu
                # vao SEARCH cua rieng no, va /follow/enable lan sau gap dung trang thai do thi node planner CHET
                # (last_target_odom = None, xem CLAUDE.md 13.32).
                if now - self.t_fix > self.a.lost_sec:
                    return "mat"
                self.pub_target(now)
                self.trace(now)
                why = self.planner_guard(now, t_en)
                if why:
                    return why
                near = math.hypot(self.flt.x - self.x, self.flt.y - self.y) <= FOLLOW_DISTANCE_M + 0.35
                idle = self.planner_idle()
                bx, by = self.to_base(self.flt.x, self.flt.y)
                arr = (math.hypot(bx, by), abs(math.atan2(by, bx)), math.hypot(self.flt.vx, self.flt.vy))
                if self.hold_still:
                    self.hold_still = all(v <= lim for v, lim in zip(arr, HOLD_OUT))
                else:
                    self.hold_still = idle and all(v <= lim for v, lim in zip(arr, HOLD_IN))
                idle = idle or self.hold_still
                if near:
                    self.n_stall = 0
                # KET: chua toi gan muc tieu ma planner (a) ra lenh dung im qua STALL_SEC — loi da biet cua planner
                # (CLAUDE.md 13.12: DWA chon v = w = 0 canh vat can, khong co co che thoat), hoac (b) lac / xoay tai cho
                # ma suot STALL_WIN_SEC xe khong roi khoi cho dang dung qua 12 cm (dong tac chui khe bat nham giua do dac).
                if near:
                    hist.clear()                     # chi dem thoi gian xe LE RA phai tien: dang o gan muc tieu thi xe
                    hist.append((now, self.x, self.y))   # dung yen la dung
                elif now - hist[-1][0] >= 0.25:
                    hist.append((now, self.x, self.y))
                while now - hist[0][0] > STALL_WIN_SEC:
                    hist.popleft()
                t_stall = (t_stall or now) if (idle and not near and now - t_en > 2.0) else None
                stuck = (not near and now - hist[0][0] > STALL_WIN_SEC - 1.0
                         and max(math.hypot(self.x - hx, self.y - hy) for _ht, hx, hy in hist) < 0.12)
                if stuck or (t_stall is not None and now - t_stall > STALL_SEC):
                    if self.n_stall < STALL_MAX_TRIES:
                        return "ket"
                    # Het cach go. Biet dau muc tieu la do dac khong toi duoc (khoa nham) -> van phai kiem tra bang RSSI
                    if (self.a.verify_sec > 0 and now >= self.verify_hold
                            and now - self.t_verified >= max(self.verify_wait, float(self.a.verify_sec))):
                        return "kiem tra"
                    if now - t_warn > 15.0:
                        t_warn = now
                        self.say("xe khong toi gan duoc muc tieu (ket canh vat can?) — cho nguoi / vat can doi cho.", Y)
                if self.a.verify_sec <= 0 or now < self.verify_hold:
                    continue
                # KIEM TRA LAI bang RSSI: LiDAR chi bam "nhom chan", khong biet la chan AI. Chi lam khi ca xe lan muc
                # tieu dang dung yen (do mat ~8 s xoay tai cho).
                still = idle and near and math.hypot(self.flt.vx, self.flt.vy) < 0.15
                if not still:
                    t_still = t_dev = base = None
                    self.level_wait = 8.0
                    continue
                t_still = t_still or now
                if now - t_still < 2.0:
                    continue
                if now - self.t_verified >= self.verify_wait:
                    return "kiem tra"
                # Muc tieu dung yen ma muc tin hieu beacon doi han = beacon dang o cho khac (dang bam nham do dac /
                # nguoi khac, hoac chu vua di khoi trong khi LiDAR giu mot vat dung yen)
                lv = (self.brg or {}).get("levels") or {}
                if base is None:
                    base = dict(lv) if len(lv) == 3 else None
                    continue
                n_dev = sum(abs(lv[k] - base[k]) > LEVEL_JUMP_DB for k in base if k in lv)
                t_dev = (t_dev or now) if n_dev >= 2 else None
                if t_dev is not None and now - t_dev >= 3.0 and now - self.t_verified >= self.level_wait:
                    return "muc doi"
        finally:
            self.end_relay()

    def reposition(self, watch: bool = False) -> bool:
        """Khong du cho xoay ma cung khong lui duoc: nho PLANNER (co loc va cham) nhich xe toi cho thoang hon — chon
        huong trong +-90 do quanh mui xe co hanh lang trong it nhat 0.9 m va lech XA vat gan nhat. True = da du cho xoay.
        watch=True (nhich de KIEM TRA muc tieu dang bam): muc tieu bat dau di / mat dau thi thoi ngay."""
        _ok, _d, bdeg = self.rotate_clear()
        best, best_s = None, -1.0
        for deg in range(-90, 91, 15):
            free = self.free_along(self.yaw + math.radians(deg))
            if free < 0.9:
                continue
            s = abs(wrap(math.radians(deg - bdeg)))          # cang lech xa huong vat gan nhat cang tot
            if s > best_s:
                best, best_s = deg, s
        if best is None:
            self.say("khong co huong nao du trong de nhich ra cho thoang.", Y)
            return False
        self.say(f"nhich toi cho thoang hon (huong {best:+d} do so voi mui xe, toi da 0.6 m, planner lai).", Y)
        self.state = "NHICH RA"
        th = self.yaw + math.radians(best)
        res = self.drive_to(self.x + (0.6 + FOLLOW_DISTANCE_M) * math.cos(th), self.y + (0.6 + FOLLOW_DISTANCE_M) * math.sin(th),
                            "predicted", 12.0, 0.6, until_clear=True, watch=(self.flt.x, self.flt.y) if watch else None)
        return res == "du cho" or (res != "muc tieu di" and self.rotate_clear()[0])

    def drive_to(self, ox: float, oy: float, src: str, max_sec: float, max_move: float, phi=None,
                 until_clear: bool = False, watch=None, reacq: float = 0.0) -> str:
        """Dat diem dich tam (odom) de planner lai toi cach no follow_distance (co ne vat can).
        until_clear: dung ngay khi quanh tam quay da du trong de xoay do.
        watch = vi tri muc tieu dang bam luc bat dau: no di chuyen / mat dau thi dung ngay ('muc tieu di').
        reacq > 0 (vua mat dau): trong ngan nay giay dau, moi vong quet tim nhom chan de khoa lai ngay (reacq_candidate);
        tim duoc va giu duoc REACQ_HOLD_SEC thi dung ('khoa lai', vi tri odom o self.reacq).
        Tra ve 'toi' | 'sat vat' | 'du cho' | 'muc tieu di' | 'het buoc' | 'het gio' | 'ket' | 'thay nguoi' | 'khoa lai' |
        'bi tat'."""
        self.goal = (ox, oy, src)
        x0, y0, t0 = self.x, self.y, time.time()
        t_idle, t_look, armed = None, 0.0, False
        t_chk, hold = 0.0, None
        if not self.call(self.en_cli):
            raise Stop("khong goi duoc /follow/enable (planner chua chay?)", 0.0)
        self.relay = True
        try:
            while True:
                self.guard()
                self.spin(0.05)
                now = time.time()
                self.pub_target(now)
                self.trace(now)
                why = self.planner_guard(now, t0)
                if why:
                    return why
                if reacq > 0.0 and now - t0 <= reacq and self.t_scan > t_chk:
                    t_chk = self.t_scan
                    g = self.reacq_candidate(*self.lost_path(now))
                    if g is None:
                        hold = None
                    elif hold is None or math.hypot(g[0] - hold[0], g[1] - hold[1]) > 0.45:
                        hold = (g[0], g[1], now)
                    else:
                        hold = (g[0], g[1], hold[2])
                        if now - hold[2] >= REACQ_HOLD_SEC:
                            self.reacq = (g[0], g[1])
                            return "khoa lai"
                moved = math.hypot(self.x - x0, self.y - y0)
                # Doan lai nay la de toi cho DO LAI -> khong duoc dung o cho khong xoay duoc (vat trong 0.53 m quanh tam
                # quay): vua thay sap het cho xoay thi dung ngay tai do.
                clr = self.rotate_clear()[1]
                if watch is not None and (now - self.t_fix > self.a.lost_sec
                                          or math.hypot(self.flt.x - watch[0], self.flt.y - watch[1]) > 0.35):
                    return "muc tieu di"
                if until_clear and moved > 0.05 and clr >= CLEAR_MIN_M + 0.06:
                    return "du cho"
                if clr >= CLEAR_MIN_M + 0.08:
                    armed = True
                elif armed and clr < CLEAR_MIN_M + 0.04:
                    return "sat vat"
                if math.hypot(ox - self.x, oy - self.y) <= FOLLOW_DISTANCE_M + 0.15 and not (reacq > 0.0 and now - t0 <= reacq):
                    return "toi"                     # (vua mat dau: xe thuong da o ngay gan cho thay cuoi -> doi het luc tim)
                if moved >= max_move:
                    return "het buoc"
                if now - t0 > max_sec:
                    return "het gio"
                t_idle = (t_idle or now) if (now - t0 > 1.5 and self.planner_idle()) else None
                if t_idle is not None and now - t_idle > 2.5:
                    return "ket"                     # planner dung im ma chua toi: bi chan / het duong
                if phi is not None and moved > 0.3 and now - t_look > 0.5:
                    t_look = now
                    if self.pick_candidate(phi, learn=False) is not None:
                        return "thay nguoi"
        finally:
            self.goal = None
            self.end_relay()

    def idle(self, sec: float) -> None:
        t_end = time.time() + sec
        while time.time() < t_end:
            self.guard()
            self.pub.publish(self.Twist())
            self.spin(0.2)

    # ── chay ─────────────────────────────────────────────────────────────
    def run(self) -> int:
        t = time.time()
        while time.time() - t < 1.5 or (time.time() - t < 6.0 and (
                self.yaw is None or not self.scans or self.brg is None or self.pstat is None)):
            self.spin(0.1)
        if self.yaw is None:
            print(f"{R}Khong co /odom — T1 da chay chua, khung xe da bat nguon chua?{X}")
            return 1
        if not self.scans:
            print(f"{R}Khong co /scan — LiDAR khong gui du lieu (rut cam lai LiDAR, launch lai T1).{X}")
            return 1
        if self.brg is None:
            print(f"{R}Khong co /rssi/bearing — T1 phai la rssi_follow.launch.py (co 2 node RSSI).{X}")
            return 1
        if self.pstat is None:
            print(f"{R}Khong co /follow/planner_status — T1 phai la rssi_follow.launch.py (co planner).{X}")
            return 1
        if self.node.count_publishers("/cmd_vel") > 0:
            print(f"{R}Dang co node ghi thang /cmd_vel. Voi script nay planner phai ghi vao {CMD_FOLLOW}: dung "
                  f"rssi_follow.launch.py, KHONG dung follow_nav_real / test_avoid_only.{X}")
            return 1
        if self.node.count_publishers(CMD_FOLLOW) != 1:
            print(f"{R}Can dung 1 node ghi {CMD_FOLLOW} (planner), dang co {self.node.count_publishers(CMD_FOLLOW)}.{X}")
            return 1
        if self.brg.get("reason") == "khong co file mau":
            print(f"{R}rssi_bearing_node khong co file mau — chay rssi_rotate.py --calib roi launch lai T1.{X}")
            return 1
        if not self.brg.get("beacon_ok", False):
            print(f"{R}Khong nghe thay beacon — beacon da bat chua? (ros2 topic echo /rssi/status --field data --full-length){X}")
            return 1
        if not (self.en_cli.wait_for_service(timeout_sec=2.0) and self.dis_cli.wait_for_service(timeout_sec=2.0)):
            print(f"{R}Khong thay service /follow/enable, /follow/disable cua planner.{X}")
            return 1
        self.reset_cli.wait_for_service(timeout_sec=2.0)
        if "/rssi_follow/planner_stop" not in dict(self.node.get_service_names_and_types()):
            print(f"{Y}Canh bao: planner khong chay tu rssi_follow.launch.py (service dung cua planner chua doi ten) -> "
                  f"/follow/stop co the KHONG dung duoc xe luc dang xoay do. Dung khan: Ctrl-C, hoac /rssi_follow/stop.{X}")
        self.call(self.dis_cli)                          # planner TAT cho toi khi da khoa duoc nguoi
        self.t0 = time.time()
        # NEN ban dau: canh vat luc nguoi con dung canh xe. Het dem nguoc, nhom nao "moi" o huong beacon chinh la nguoi.
        self.bg = self.snapshot()
        if self.a.delay > 0:
            print(f"{B}Di toi vi tri va DUNG YEN (deo beacon sau that lung, quay lung ve xe). Xe bat dau xoay sau "
                  f"{self.a.delay:.0f} s...{X}", flush=True)
            t_end = time.time() + self.a.delay
            while time.time() < t_end:
                self.spin(min(2.0, max(0.05, t_end - time.time())))
                lv = self.brg.get("levels", {})
                print(f"   con {max(0.0, t_end - time.time()):3.0f} s   muc A/B/C: "
                      + "/".join(f"{lv[k]:.0f}" if k in lv else "--" for k in "ABC") + " dBm", flush=True)
        self.pub = self.node.create_publisher(self.Twist, "/cmd_vel", 10)
        self.t0 = time.time()
        self.say("Bat dau bam bang RSSI + LiDAR. DUNG KHAN: Ctrl-C, hoac ros2 service call /follow/stop std_srvs/srv/Trigger {}",
                 B, args=vars(self.a))
        fails = n_wait = n_back = n_move = 0
        for n in range(1, self.a.max_scans + 1):
            try:
                verifying = self.trk
                self.say(f"--- lan do {n}: xoay tim huong beacon" + (" (KIEM TRA muc tieu dang bam)" if verifying else "") + " ---")
                phi = self.do_scan()
                tracked = time.time() - self.t_fix <= self.a.lost_sec
                if phi is None and not self.rotate_clear()[0] and (tracked or not verifying):
                    # Het cho xoay (vat trong 0.53 m quanh tam quay) — ca luc tim lan luc kiem tra: lui ra roi do lai.
                    # Khong lam the thi xe dau canh do dac voi mot muc tieu SAI se khong bao gio kiem tra lai duoc.
                    n_back += 1
                    if n_back <= 2 and self.back_out("khong du cho xoay o day", watch=verifying):
                        continue
                    if verifying:
                        # Khong lui duoc ma muc tieu van dung yen: nhich ra cho thoang de kiem tra. Bo qua thi mot muc tieu
                        # SAI nam canh do dac se khong bao gio bi phat hien.
                        still = (time.time() - self.t_fix <= self.a.lost_sec and self.scan_trk0 is not None
                                 and math.hypot(self.flt.x - self.scan_trk0[0], self.flt.y - self.scan_trk0[1]) <= 0.35)
                        if still and n_back <= 3 and self.reposition(watch=True):
                            continue
                    else:
                        # Khong lui duoc: nho planner nhich toi cho thoang hon. Van khong duoc thi dung cho roi thu lai
                        # (nguoi / vat can co the doi cho) — KHONG thoat chuong trinh chi vi thieu vai cm cho xoay.
                        if n_back <= 4 and self.reposition():
                            continue
                        w = min(30.0, 5.0 * 2 ** max(0, n_back - 4))
                        self.say(f"van khong du cho xoay — dung cho {w:.0f} s roi thu lai (dua xe / vat can ra cho thoang).", Y)
                        self.idle(w)
                        continue
                if phi is None and not verifying:
                    fails += 1
                    if fails >= 3:
                        self.say("3 lan lien tiep khong tim duoc huong — dung.", R)
                        return 2
                    continue
                fails = 0
                if phi is not None:
                    self.scan_xy, n_back = (self.x, self.y), 0
                    mvs = [] if verifying else self.scene_changes()
                    if len(mvs) == 1:
                        g = mvs[0]
                        dev = math.degrees(wrap(math.atan2(g[1], g[0]) - wrap(phi - self.yaw)))
                        go = self.to_odom(g[0], g[1])
                        if abs(dev) <= MOVED_OK_DEG and self.moving(go[0], go[1], MOVED_STILL_SEC) is False:
                            # Chinh la nguoi deo beacon vua di toi roi dung lai: nhan ket qua, uu tien nhom do khi chon
                            self.prefer = go
                            self.say(f"do xong; trong luc do co DUNG MOT nhom chan moi xuat hien (o {math.degrees(math.atan2(g[1], g[0])):+.0f} "
                                     f"do, cach {math.hypot(g[0], g[1]):.1f} m), lech huong beacon {dev:+.0f} do va nay da dung yen = nguoi "
                                     "deo beacon vua di toi -> nhan ket qua, uu tien nhom do (kiem tra lai bang RSSI o lan dung yen "
                                     "dau tien).", G, moved_ok=[round(g[0], 2), round(g[1], 2)])
                            mvs = []
                    if mvs and n_move < 2:
                        n_move += 1
                        mv = mvs[0]
                        self.say(f"do xong nhung co nhom chan MOI xuat hien trong luc do (o {math.degrees(math.atan2(mv[1], mv[0])):+.0f} "
                                 f"do, cach {math.hypot(mv[0], mv[1]):.1f} m) = co nguoi di chuyen luc xe dang xoay -> huong vua do "
                                 "khong tin duoc — do lai.", Y, moved_during_scan=[round(mv[0], 2), round(mv[1], 2)])
                        continue
                    n_move = 0
                if verifying and phi is None:
                    self.say(f"khong kiem tra duoc lan nay — bam tiep muc tieu cu, {VERIFY_RETRY_SEC:.0f} s nua thu lai.", Y)
                    self.verify_hold, n_back = time.time() + VERIFY_RETRY_SEC, 0
                elif verifying:
                    ox, oy = self.flt.predict(time.time())
                    d = math.degrees(wrap(phi - math.atan2(oy - self.y, ox - self.x)))
                    # KHONG co luat "chuyen sang nhom khac khop huong beacon hon": da thu (02/10) — mot lan do lech
                    # 25-31 do (chuyen thuong gap) la xe bo DUNG chu de sang do dac ben canh. RSSI khong phan biet duoc
                    # hai vat dung yen cach nhau < ~40 do; do lai tai cho cung cho dung do lech do (sai so lap lai theo
                    # vi tri). Chi chuyen khi co bang chung cua LiDAR rang muc tieu dang bam co the KHONG phai nguoi:
                    #   - luc khoa no khong phai vat moi va tu do chua he nhuc nhich (co the la do dac), hoac
                    #   - no dang dung dung cho ma lan quet truoc co vat dung yen (LiDAR da truot sang vat / nguoi do)
                    # trong khi co nhom chan MOI xuat hien (nguoi vua di toi) trong cung +-35 do quanh huong beacon.
                    # Nhom moi KHONG can khop huong beacon hon muc tieu cu (mo phong: chuyen dung ca khi lech -18 so voi
                    # -12 do). Khong dua muc tieu cu vao danh sach bo.
                    alt, why = None, ""
                    if abs(d) < REJECT_NOW_DEG:
                        if not self.lock_new and self.moved < 0.3:
                            why = "chua he nhuc nhich tu luc khoa (co the la do dac)"
                        elif self.on_background(ox, oy):
                            why = "dang dung dung cho lan quet truoc co vat dung yen (LiDAR co the da truot sang vat do)"
                        if why:
                            alt = self.new_candidate(phi, (ox, oy))
                    if alt is not None:
                        self.say(f"muc tieu dang bam {why}, ma co nhom chan MOI xuat hien dung huong beacon (lech {alt[2]:+.0f} "
                                 f"do; muc tieu cu lech {d:+.0f} do) — chuyen sang nhom moi.", Y, verify=round(d, 1),
                                 switch=round(alt[2], 1))
                        self.ambig_lock = True               # vua doi muc tieu: kiem tra lai ngay lan dung yen dau tien
                        self.lock(alt)
                        self.n_quick = 0
                    elif abs(d) <= self.a.verify_tol_deg:
                        self.say(f"XAC NHAN: beacon o dung huong muc tieu dang bam (lech {d:+.0f} do) — bam tiep.", G, verify=round(d, 1))
                        self.t_verified = time.time()
                        self.strike = self.n_quick = 0
                        # xe vua xoay 1 vong (odom lech vai do): dung lai ca hai nen
                        self.bg, self.bg_mask = self.fresh_bg((ox, oy)) or self.bg, self.snapshot((ox, oy)) or self.bg_mask
                        # Dung yen lau thi thua dan cac lan kiem tra: --verify-sec roi gap doi. Muc tieu da tung tu di
                        # chuyen (chac chan la nguoi, LiDAR bam lien tuc tu luc RSSI xac nhan): toi da 8 lan; chua he
                        # nhuc nhich tu luc khoa (co the la do dac canh chu): toi da 2 lan.
                        vs = float(self.a.verify_sec)
                        self.verify_wait = min(max(vs, 2.0 * self.verify_wait), (8.0 if self.moved >= 0.3 else 2.0) * vs)
                        self.level_wait = min(2.0 * self.level_wait, 64.0)
                    elif abs(d) < REJECT_NOW_DEG and self.strike == 0:
                        # Mot lan do RSSI co the sai > 40 do (du lieu that: toi ~45 do). Bo nham CHU roi khoa sang do dac
                        # thi te hon nhieu so voi ton them 8 s -> do lai lan nua, hai lan lien tiep cung lech moi bo.
                        self.strike = 1
                        self.say(f"huong beacon lech muc tieu dang bam {d:+.0f} do — chua du chac, do lai lan nua.", Y,
                                 verify=round(d, 1))
                        continue
                    else:
                        self.say(f"Muc tieu dang bam KHONG o huong beacon (lech {d:+.0f} do"
                                 + (", hai lan lien tiep" if self.strike else "") + ") — bo, khoa lai theo beacon.", Y,
                                 verify=round(d, 1))
                        self.trk = False
                        self.strike = 0
                        self.just_rejected = True
                        if self.moved < 0.3:             # tu luc khoa chua he nhuc nhich = do dac: tam thoi khong khoa lai no
                            self.rejected.append((ox, oy, time.time()))
                        if self.scene_changed() is not None:
                            # Nguoi deo beacon (khong phai muc tieu vua bo) co the vua di chuyen trong luc xe xoay ->
                            # huong vua do du de BO muc tieu cu nhung khong du tin de KHOA cai moi
                            self.say("co nguoi di chuyen trong luc do — do lai roi moi khoa.", Y)
                            continue
                if not self.trk:
                    if abs(wrap(phi - self.yaw)) > math.radians(100.0):
                        # Huong beacon o phia SAU: cung mu cua LiDAR (+-24 do quanh duoi xe) che mat nguoi -> quay mat ve do
                        self.do_turn(phi)
                        self.spin(0.5)
                    cand = self.pick_candidate(phi, after_reject=self.just_rejected)
                    self.just_rejected = False
                    if self.prefer is not None and cand is not None:
                        self.ambig_lock = True           # huong do luc co nguoi di chuyen: kiem tra lai som bang RSSI
                    self.prefer = None
                    if cand is None:
                        free = self.free_along(phi)
                        d = min(self.a.step, free - BLOCK_KEEP_M)
                        if d < 0.3:
                            n_wait += 1
                            w = min(30.0, 5.0 * 2 ** (n_wait - 1))       # 5, 10, 20, 30 s: khong xoay lien tuc khi nguoi con khuat
                            self.say(f"chua thay ai theo huong beacon, va huong do bi vat chan cach mui xe {free:.2f} m (nguoi khuat "
                                     f"sau vat?) — dung cho nguoi ra cho thoang, {w:.0f} s nua do lai.", Y, cands=self.cands)
                            self.idle(w)
                            continue
                        self.say(f"chua thay ai theo huong beacon trong tam LiDAR — lai ve huong do {d:.1f} m (planner ne vat "
                                 "can) roi do lai.", cands=self.cands)
                        self.state = "DI THEO HUONG"
                        res = self.drive_to(self.x + (d + FOLLOW_DISTANCE_M) * math.cos(phi),
                                            self.y + (d + FOLLOW_DISTANCE_M) * math.sin(phi), "rssi_bearing",
                                            d / 0.08 + 8.0, d - 0.1, phi)
                        if res == "bi tat":
                            self.say("planner bi tat tu ngoai (/follow/stop) — dung.", Y)
                            return 0
                        self.say(f"dung lai ({res}) — do lai.")
                        continue
                    self.lock(cand)
                    n_wait = self.n_quick = 0
                while True:                          # mat dau ma khoa lai nhanh duoc thi bam tiep luon, khong do lai
                    while True:
                        res = self.do_follow()
                        if res != "ket":
                            break
                        # Go ket cho planner: lui ra (neu duoc phep) roi quay mat ve muc tieu, bat lai planner tu the moi
                        self.n_stall += 1
                        self.say(f"xe khong tien duoc toi muc tieu (planner dung im / lac tai cho canh vat can) — go ket lan "
                                 f"{self.n_stall}/{STALL_MAX_TRIES}.", Y, stall=self.n_stall)
                        self.back_out("go ket", STALL_BACK_M)
                        if time.time() - self.t_fix <= self.a.lost_sec and self.rotate_clear()[0]:
                            self.do_turn(math.atan2(self.flt.y - self.y, self.flt.x - self.x))
                    if res == "bi tat":
                        self.say("planner bi tat tu ngoai (/follow/stop) — dung.", Y)
                        return 0
                    if res == "mat":
                        self.trk = False
                        self.last_lost = (self.flt.x, self.flt.y)
                        self.say(f"MAT DAU (LiDAR khong thay nhom chan {self.a.lost_sec:.1f} s) — lai toi gan cho thay cuoi "
                                 f"(trong {REACQ_SEC:.0f} s dau: thay dung mot nguoi dang di o gan thi khoa lai ngay), khong "
                                 "thay thi do lai bang RSSI.", Y, lost=True)
                        self.state = "TOI CHO THAY CUOI"
                        res = self.drive_to(self.flt.x, self.flt.y, "predicted", GOTO_LAST_MAX_SEC, 9.0,
                                            reacq=REACQ_SEC if self.n_quick < REACQ_MAX else 0.0)
                        if res == "bi tat":
                            self.say("planner bi tat tu ngoai (/follow/stop) — dung.", Y)
                            return 0
                        if res == "khoa lai":
                            bx, by = self.to_base(*self.reacq)
                            self.cands = []
                            self.n_quick += 1
                            self.ambig_lock = True       # chua co RSSI xac nhan -> kiem tra ngay lan dung yen dau tien
                            self.lock((bx, by, 0.0, True), why="KHOA LAI NHANH: nhom chan DANG DI duy nhat gan cho vua mat "
                                      "dau, khong co ai khac quanh do; se kiem tra bang RSSI o lan dung yen dau tien")
                            continue
                    elif res == "muc doi":
                        self.say("muc tieu dung yen nhung muc tin hieu beacon doi han — do lai xem beacon con o do khong.")
                    else:
                        self.say(f"xe dang dung, RSSI xac nhan muc tieu nay lan cuoi cach day {time.time() - self.t_verified:.0f} s "
                                 f"(co {self.n_ambig} vong quet thay vat khac sat muc tieu) — do lai de chac la nguoi deo beacon.")
                        self.n_ambig = 0
                    break
            except Stop as e:
                self.relay = False
                self.halt()
                self.trk = False
                self.say(f"DUNG: {e.why}", R)
                if e.wait <= 0.0:
                    return 2
                self.state = "CHO PHUC HOI"
                if not self.wait_recover(e.wait):
                    self.say(f"cho {e.wait:.0f} s van chua phuc hoi — dung han.", R)
                    return 2
                self.say("da phuc hoi — do lai.", G)
        self.say("Het so lan do cho phep (--max-scans) — ket thuc.", B)
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Bam theo nguoi deo beacon: RSSI nhan chu, LiDAR giu bam, planner lai xe")
    ap.add_argument("--step", type=float, default=1.2, help="chua thay nguoi: lai ve huong beacon toi da ngan nay roi do lai (m)")
    ap.add_argument("--lost-sec", type=float, default=1.5, help="LiDAR khong thay nhom chan lau hon -> mat dau (s)")
    ap.add_argument("--verify-sec", type=float, default=20.0,
                    help="xe + muc tieu dung yen va RSSI chua xac nhan lau hon ngan nay -> do lai de kiem tra dung chu "
                         "(s; moi lan dung gap doi, toi da 8 lan; 0 = tat)")
    ap.add_argument("--verify-tol-deg", type=float, default=40.0, help="huong beacon lech muc tieu qua ngan nay -> bo muc tieu")
    ap.add_argument("--cand-win-deg", type=float, default=35.0, help="chi nhan nhom chan lech huong beacon trong ngan nay (do)")
    ap.add_argument("--cand-range", type=float, default=4.5, help="tam xa nhat nhan nhom chan luc khoa (m)")
    ap.add_argument("--w-scan", type=float, default=0.9, help="toc do xoay do (rad/s) — 0.9 da kiem tren xe (that ~0.82)")
    ap.add_argument("--min-scan-deg", type=float, default=330.0, help="xoay it nhat ngan nay moi tin huong (do)")
    ap.add_argument("--max-scan-deg", type=float, default=760.0, help="xoay toi da ngan nay ma chua co huong thi bo (do)")
    ap.add_argument("--max-scans", type=int, default=60)
    ap.add_argument("--max-sec", type=float, default=900.0, help="tu dung sau ngan nay giay (0 = khong gioi han)")
    ap.add_argument("--delay", type=float, default=8.0, help="dem nguoc truoc khi xe bat dau (s) — de di toi vi tri")
    ap.add_argument("--log", default=time.strftime("rssi_follow_%H%M%S.jsonl"))
    args = ap.parse_args()
    args.w_scan = min(max(args.w_scan, 0.3), 1.0)
    if args.max_sec <= 0:
        args.max_sec = 1e9

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    s = Follower(args)

    def _stop(*_):                       # chi dat co; spin() nem KeyboardInterrupt o cho an toan (xem rssi_seek.py)
        s.stop_req = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    rc = 1
    try:
        rc = s.run()
    except KeyboardInterrupt:
        print(f"\n{Y}Ctrl-C / DUNG KHAN — dung xe{X}")
        rc = 130
    finally:
        s.relay = False
        s.halt()
        s.stop_req = False
        try:
            s.call(s.dis_cli)
        except BaseException:
            pass
        if s.log:
            print(f"Nhat ky: {os.path.abspath(args.log)}  (+ .csv mau tho)")
            s.log.close()
            s.raw.close()
        s.node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
