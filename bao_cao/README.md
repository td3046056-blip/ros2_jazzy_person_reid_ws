# Hình cho báo cáo khóa luận

Mọi hình trong `hinh/` được **sinh ra từ chính code và dữ liệu của dự án** — không vẽ tay:

- **Mô phỏng:** dùng `sim_follow.py` chạy **đúng code planner và tracker thật**. Mỗi bản cũ được lấy bằng `git archive` theo commit/tag.
- **Thuật toán:** gọi thẳng các hàm `_dwa`, `_choose_heading`, `_nav_plan` của `follow_planner_node`.
- **Chạy thật:** lấy từ bag ghi trên xe ngày 07/11 và log RSSI đo trên xe ngày 01/10.
- **Biểu đồ tổng hợp:** dùng số liệu đã ghi trong `CLAUDE.md` (mục 7.2, 8). Script tự kiểm tổng khớp 1212 / 1523 / 1658 / 1836.

PNG 300 dpi, rộng 6,4 in (vừa khổ A4 trừ lề). Sơ đồ kiến trúc có thêm bản `.svg`.

## Danh sách hình và chú thích gợi ý

### Chương: Tổng quan hệ thống

| Hình | Chú thích gợi ý | Nguồn |
|---|---|---|
| `he_thong_kien_truc_ros2.png` (.svg) | Kiến trúc phần mềm ROS 2: các node, topic chính và phần cứng. `follow_planner_node` là nguồn duy nhất ghi `/cmd_vel`. | `CLAUDE.md` mục 5 |
| `he_thong_hinh_hoc_xe.png` | Hình học xe BW-DR03: footprint chữ nhật 0,47 × 0,60 m, `base_link` ở trục bánh trước. Bán kính quét khi xoay tại chỗ: góc trước 0,331 m, góc sau 0,446 m (đuôi văng rộng). Kèm cung mù LiDAR sau đuôi và trường nhìn camera 114°. | Số đo mục 2, 3 |

### Chương: Hiệu chỉnh cảm biến và cơ cấu chấp hành

| Hình | Chú thích gợi ý | Nguồn |
|---|---|---|
| `hieu_chinh_loc_than_xe.png` | Lọc tia LiDAR đập vào thân xe, trên 51 vòng quét thật lúc xe đứng yên. Điểm bị loại có hai loại: nằm trong footprint thu nhỏ 3 cm, hoặc nằm trong cung mù [246°, 294°]. Thiếu bộ lọc này thì mọi quỹ đạo bị loại và xe kẹt `BLOCKED` vĩnh viễn. | bag `follow_1107_104423`, 5 s đầu |
| `hieu_chinh_toc_do_driver.png` | (a) Tốc độ thật tuyến tính theo % PWM, ra `max_linear` = 0,49 m/s ở 60 %. (b) Kiểm chứng sau khi sửa: thước đo / odom so với lệnh. | `measure_speed.py`, `CALIBRATION.md` |
| `hieu_chinh_mo_hinh_camera.png` | Mô hình góc camera KINGSEN: mắt cá đều f = 324 px (FOV 114,4°) so với giả định pinhole 62° cũ. Người lệch giữa khung bị báo thiếu gần một nửa góc. | đo thước 5 vạch 30/09 |

### Chương: Thuật toán

| Hình | Chú thích gợi ý | Nguồn |
|---|---|---|
| `thuat_toan_bo_nho_odom.png` | Nhớ vị trí người trong khung odom thay vì nhớ góc tương đối. Xe né, quay 35° trong 1,2 s: cách cũ sai tới 36,9°, vượt nửa FOV; cách mới tính lại từ odom nên sai 0°. | kịch bản `test_memory.py` |
| `thuat_toan_dwa_quat_quy_dao.png` | Hai tầng của planner. Tầng chọn khe (nhìn xa 1,6 m) cho hướng ngắm. DWA thử khoảng 119 cặp (v, ω) trong 1,2 s và kiểm footprint chữ nhật ở từng tư thế. Quỹ đạo xoay gắt bị loại vì đuôi xe quét vào chân ghế phía sau. | `_choose_heading`, `_dwa`, `_admissible` thật |
| `thuat_toan_ban_do_luoi_dijkstra.png` | Bản đồ lưới 6 cm quanh xe: ô cấm khi tâm xe cách vật < 0,30 m, chi phí tăng dần trong lề an toàn. Khe 0,5 m giữa hai thùng (hẹp hơn xe 0,6 m) bị cấm, Dijkstra chọn đường vòng bên trống tới chỗ đứng tốt nhất. | `_nav_plan` thật |
| `thuat_toan_ke_hoach_thoat_ket.png` | Kế hoạch thoát kẹt A* trên (x, y, hướng) trong túi chữ U rộng 1,0 m: lùi 8 + 8 + 16 cm theo cung, rồi xoay tại chỗ 165°. Kế hoạch đúng như planner lập ra trong mô phỏng; đường đen là tâm xe thật khi chạy kế hoạch. | mô phỏng `bay_U`, bản cuối |
| `rssi_xoay_do_huong.png` | RSSI "xoay dò hướng". (a) Mức tín hiệu 3 board theo góc người khi xe xoay 2 vòng (mẫu hiệu chỉnh). (b) Điểm khớp mẫu của một lần đo. (c) Sai số 11 lần đo thật: 11/11 trong ±30°, trung bình 14,5°. | `rssi_logs/q2_*`, hàm `rssi_df.est_template` thật |
| `camera_reid_truoc_sau.png` | Camera / ReID trước và sau sửa: (a) Rank-1 Market-1501 39,8 → 75,5 % (lỗi nạp BGR); (b) khoá nhầm người lạ 8,6 → 0,1 %; (c) thời gian xử lý 70,6 → 21 ms/khung; (d) bám đúng chủ 47,5 → 59,4 %. | `CLAUDE.md` 7.2-AG, AP |

### Chương: Kết quả mô phỏng (trước / sau từng đợt sửa)

Mỗi hình gồm 2 panel cùng một kịch bản: bản code trước đợt sửa và bản cuối 07/11. Dòng chữ nhỏ trên mỗi panel ghi các chỉ số: khoảng cách cuối tới người, trạng thái cuối, khe hở nhỏ nhất tới vật, tỉ lệ camera thấy người, thời gian đứng im lâu nhất.

| Hình | Kịch bản | Bản trước → bản cuối |
|---|---|---|
| `mp_cua_giua.png` | Xe gần giữa cửa 0,81 m tường dày, LiDAR nhiễu 2 cm | 26/09: đứng giằng co ở cửa (`AVOID`, đổi tiến/lùi 19 lần), cách người 1,68 m → qua cửa, `ARRIVED` 0,99 m |
| `mp_cua_ben_hong.png` | Người rẽ vào cửa bên hông 0,81 m | 23/09: kẹt ở khung cửa 1,82 m → chui cửa, 1,14 m |
| `mp_vat_thap.png` | Người đứng sau thùng thấp, thùng thứ hai cách 0,5 m | 07/10: đứng `AVOID` 1,38 m → vòng bên trống, `ARRIVED` 1,12 m |
| `mp_vat_chu_U.png` | Người sau vật hình chữ U | 07/10: chui vào lòng chữ U, `BLOCKED` → vòng ngoài, `ARRIVED` 1,03 m |
| `mp_cua_bep.png` | Cửa bếp 0,80 m dựng lại từ bag chạy thật 14/10 | 09/10: tiến lùi ở miệng cửa (41 lần), 2,50 m → qua cửa, 1,04 m |
| `mp_tu_lanh.png` | Qua cửa có tủ lạnh sát sau tường, người rẽ ngay | 14/10: mất người, `IDLE`, đứng im 26 s → `ARRIVED` 1,14 m |
| `mp_tui_hep.png` | Túi hẹp: khe chỉ người lọt | 03/11: kẹt rồi mất người (`SEARCH`, 2,84 m) → thoát ra, 1,11 m |
| `mp_bay_U_sau.png` | Xe xuất phát trong túi chữ U sâu 1,3 m, người đi xa | 05/11: đứng im 27 s, 3,88 m → lùi theo vết, quay đầu, 1,26 m |
| `mp_bay_goc_tuong.png` | Mũi xe vào góc tường cao | 05/11: bò tới cách người 0,43 m, `BLOCKED` → giữ 0,87 m |
| `mp_bay_khe_060.png` | Khe 0,60 m hẹp hơn xe, tường cao | 03/11: tiến–lùi 141 lần, hở còn 1 cm → thoát kẹt, 1,11 m |
| `mp_dien_bien_bay_U.png` | Diễn biến theo thời gian của `mp_bay_U_sau` (bản cuối) | khoảng cách, trạng thái, lệnh v/ω; vạch đỏ = lúc bắt đầu thoát kẹt |
| `mp_nguoi_thu_hai_chen.png` | Người thứ hai bước vào đứng chen giữa xe và người đang bám | Xe dừng chờ, tracker không nhận nhầm (sai số < 0,25 m) |
| `hoi_quy_3_ban_theo_nhom.png` | Hồi quy toàn bộ 612 kịch bản × 3 mầm, 3 bản tốt nhất | 1212 → 1523 → 1658 / 1836 lượt đạt, không lượt nào chạm vật |
| `hoi_quy_tien_do_cac_dot.png` | Tỉ lệ đạt trước/sau mỗi đợt sửa (07/10 → 07/11) | bộ hồi quy được mở rộng dần, bản sau luôn cao hơn |

Lưu ý khi viết: các nhóm hỗn loạn (tủ lạnh, lộn xộn, cửa bếp, người chen) đổi kết quả theo mầm ngẫu nhiên. Hình trước/sau chỉ là **một** lượt minh hoạ; con số tổng hợp nằm ở hai biểu đồ hồi quy.

### Chương: Thực nghiệm trên xe thật (07/11)

| Hình | Chú thích gợi ý |
|---|---|
| `xe_that_ban_do_1107_104423.png` | Quỹ đạo xe thật trong 2 phút (bag `follow_1107_104423`, 330–450 s). Nền là điểm LiDAR cộng dồn theo odom, vệt tô màu theo trạng thái planner, chấm xanh là vị trí người do tracker ước lượng, sao đỏ là lúc bắt đầu thoát kẹt. |
| `xe_that_ban_do_1107_112843.png` | Như trên, bag `follow_1107_112843`, 150–240 s: 90 s, xe đi 9,1 m, 3 lần thoát kẹt. |
| `xe_that_dien_bien_1107_104423.png` | Diễn biến cùng đoạn: khoảng cách tới người, trạng thái, lệnh v/ω, nguồn đo của tracker. |

Bản đồ nền bị nhoè nhẹ là do odom trôi (2–5 % quãng đường, mục 12) — xe không dùng SLAM.

## Về RViz2 và Gazebo

- **Gazebo:** gói `robot_simulation` (Gazebo + Nav2) thuộc giai đoạn trước, dùng cho các controller cũ (VFH, Nav2 MPPI). Package `person_follow_nav` hiện tại **không** được kiểm chứng bằng Gazebo. Toàn bộ đánh giá dùng mô phỏng vòng kín riêng (`sim_follow.py`): mô hình LiDAR thiếu tia và có nhiễu, camera có FOV và che khuất, driver có trễ. Nhờ vậy chạy được khoảng 5 700 lượt trong ~80 phút, điều Gazebo không làm được. Nên viết rõ điều này trong báo cáo, thay vì đưa ảnh Gazebo không phản ánh cách đã kiểm chứng.
- **RViz2:** ảnh chụp màn hình RViz cần mở cửa sổ đồ hoạ, nên bạn tự chụp. Đã chuẩn bị sẵn công cụ phát lại bag thật:

  ```bash
  cd ~/ros2_ws/ros2_jazzy_person_reid_ws
  bash bao_cao/rviz/xem_bag.sh ~/bags/follow_1107_104423 330 0.5   # tu giay 330, toc do x0.5
  ```

  RViz hiện vòng quét hiện tại (đen), điểm LiDAR cộng dồn 20 s (xám), footprint xe tô màu theo trạng thái, vệt xe, người (trụ màu theo nguồn đo) và dòng chữ trạng thái/note/lệnh. Script:
  - chạy ở `ROS_DOMAIN_ID=77`, nên không lẫn với xe đang chạy;
  - không phát `/cmd_vel` từ bag và không chạy planner.

  Chụp bằng phím `PrtSc` hoặc `gnome-screenshot -w`. Đoạn đáng chụp: bag `follow_1107_104423` 330–450 s, bag `follow_1107_112843` 150–240 s (có thoát kẹt).

  **Phát hiện khi làm:** các launch file của xe phát TF `base_link → laser` với góc 0, trong khi code dùng `lidar_yaw_offset_deg = -90`. Planner/tracker không dùng TF nên xe chạy đúng, nhưng RViz mở theo launch sẽ vẽ vòng quét **lệch 90°**. `xem_bag.sh` tự phát TF đúng (yaw −90°) và bỏ `/tf_static` của bag. Launch chưa sửa.

## Tạo lại hình

```bash
bash bao_cao/scripts/tao_lai_hinh.sh              # dùng JSON mô phỏng có sẵn trong du_lieu/
bash bao_cao/scripts/tao_lai_hinh.sh --mo-phong   # chạy lại cả mô phỏng các bản cũ (~10 phút)
```

| Script | Sinh ra |
|---|---|
| `scripts/hinh_kien_truc.py` | sơ đồ kiến trúc (graphviz) |
| `scripts/hinh_he_thong.py` | hình học xe, lọc thân xe, tốc độ driver, mô hình camera |
| `scripts/hinh_thuat_toan.py` | bộ nhớ odom, DWA, bản đồ lưới, thoát kẹt |
| `scripts/hinh_rssi.py` | RSSI xoay dò hướng |
| `scripts/hinh_tong_hop.py` | hồi quy 3 bản, tiến độ, ReID |
| `scripts/mo_phong_hinh.py` → `chay_mo_phong.py` | chạy mô phỏng theo phiên bản, ghi `du_lieu/mo_phong/*.json` |
| `scripts/hinh_mo_phong.py` | các hình `mp_*` |
| `scripts/hinh_xe_that.py` | các hình `xe_that_*` (cần bag → pickle ở `~/bao_cao_cache`) |
| `rviz/xem_bag.sh`, `rviz/rviz_markers.py`, `rviz/bao_cao.rviz` | phát lại bag trong RViz2 |

Bag (`~/bags`) và bản chép code cũ (`~/bao_cao_cache`) nằm **ngoài repo**. Ảnh ReID ở `~/reid_data` không được đưa vào repo. Muốn chèn ảnh có người vào báo cáo thì bạn tự quyết.
