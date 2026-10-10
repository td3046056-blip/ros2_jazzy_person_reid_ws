# robot_web — trang điều khiển robot qua trình duyệt

Trang web chạy **trên máy chạy robot** (bây giờ là laptop trên xe, sau này là mini PC). Người điều khiển mở bằng trình duyệt điện thoại cùng mạng Wi-Fi, không cần cài gì. Từ trang có thể khởi động / tắt hệ thống, học người, bắt đầu / tạm dừng bám, dừng khẩn, ghi bag, xem pin và dòng sự kiện.

Node chỉ **đọc topic và gọi service có sẵn**. Nó **không bao giờ ghi `/cmd_vel`**, nên vẫn giữ bất biến "chỉ một nguồn ra lệnh cho xe" (`follow_planner_node`).

## Chạy

```bash
cd ~/ros2_reid_from_github
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run robot_web web_server
```

Log khởi động in ra:
- **Địa chỉ trang**, ví dụ `Trang dieu khien: http://192.168.1.173:8080`. IP laptop có thể đổi khi router cấp lại, nên mỗi lần xem dòng này, hoặc đặt IP cố định cho laptop trong trang quản lý router.
- **Mật khẩu**: lần đầu node tự tạo mã 6 số và lưu ở `~/.config/robot_web.conf`, nằm ngoài repo vì repo công khai. Muốn đổi thì sửa file đó rồi khởi động lại node.

**Lần đầu trên một máy:** mở cổng tường lửa, chỉ cho mạng nhà.

```bash
sudo ufw allow from 192.168.1.0/24 to any port 8080 proto tcp
```

Điện thoại vào **cùng Wi-Fi** với máy robot. Không dùng mạng "khách" (guest), vì mạng khách thường chặn các thiết bị nói chuyện với nhau. Mở địa chỉ trong log, nhập mật khẩu.

Tắt node web (Ctrl-C) **không** tắt hệ thống robot và **không** dừng ghi bag, vì cả hai chạy trong nhóm tiến trình riêng. Bật lại web thì web tự nhận ra hệ thống đang chạy và bag đang ghi; nút "Tắt hệ thống" và "Dừng ghi" vẫn dùng được.

## Giao diện (thiết kế cho điện thoại)

Từ trên xuống:
- **Thanh trên cùng:** tên trang, **REC 3:12** đỏ khi đang ghi bag (chạm để tới thẻ ghi bag), trạng thái kết nối. Mạng chậm (> 400 ms) thì báo vàng; mất kết nối quá 1.5 s thì hiện bảng đỏ.
- **Cảnh báo quan trọng** (chỉ hiện khi có): 2 node cùng ghi `/cmd_vel`, pin máy ≤ 20 %, pin khung xe dưới ngưỡng, ổ đĩa < 1 GB.
- **Thẻ "bước tiếp theo":** một câu trạng thái + **một nút lớn** cho việc cần làm bây giờ: Khởi động → Học người → Xong → Bắt đầu bám → Tạm dừng bám. Nút bị khoá thì ghi lý do ngay dưới. Khi hệ thống đang tắt, hai tuỳ chọn "Dùng RSSI" và "Tự ghi bag" nằm ngay dưới nút Khởi động.
- **Camera trực tiếp:** video kèm nhãn trạng thái và % học. Nút ⤢ phóng to video kín màn hình mà **vẫn để nút DỪNG** ở đáy. Camera mất tín hiệu thì khung cũ bị che mờ, ghi "Chưa có ảnh camera".
- **Ô trạng thái nhanh:** LiDAR, Xe, Camera, Lệnh, Beacon, Pin, Pin xe — chấm màu + số. Chạm để sổ ra chi tiết: tần số từng cảm biến, 3 board RSSI, video, pin.
- **Ba bước ①②③ dạng gập / mở:** chạm tiêu đề để mở. Tiêu đề luôn hiện trạng thái. Bước xong có dấu ✓, bước hiện tại tô xanh và tự mở một lần khi vừa chuyển bước. Kết quả kiểm tra, nhật ký hệ thống và hướng dẫn nằm trong mục sổ xuống riêng. Trạng thái gập / mở được nhớ trên từng máy.
- **Ghi dữ liệu (bag):** một nút **Ghi / Dừng ghi**, thời gian, dung lượng, số topic. "Chi tiết · bag gần đây" sổ xuống.
- **Sự kiện:** 3 sự kiện mới nhất, "Xem tất cả" để sổ cả danh sách.
- **Nút DỪNG** cố định ở đáy màn hình.

Màn hình rộng từ 980 px trở lên (laptop, máy tính bảng ngang) thì chia 2 cột và mở sẵn ba bước. Sáng hoặc tối theo cài đặt của máy. Mọi chữ hiện trên trang đều là tiếng Việt có dấu (máy chủ trả câu không dấu, trang tự dịch). Mọi thứ nằm trong một file, không tải gì từ Internet, nên dùng được cả trên Wi-Fi riêng của robot.

## Các nút

Máy chủ tự kiểm điều kiện của từng nút, không chỉ dựa vào trang. Nút đang bị khoá thì trang ghi lý do ngay bên dưới.

| Nút | Làm gì | Chỉ bấm được khi |
|---|---|---|
| Kiểm tra | `run_full.sh --check`, hiện từng mục OK / CẢNH BÁO / HỎNG | hệ thống chưa chạy |
| Khởi động | `run_full.sh` (bỏ chọn "Dùng RSSI" thì thêm `--no-rssi`). **Không bao giờ** dùng `--force`: có mục HỎNG thì không chạy. Chọn "Tự ghi bag" thì bắt đầu ghi ngay khi qua kiểm tra | hệ thống chưa chạy, và không có planner nào khác đang chạy |
| Tắt hệ thống | `/follow/stop` → chờ 0.5 s → Ctrl-C cả nhóm tiến trình launch (sau 25 s chưa tắt thì SIGTERM, rồi SIGKILL) | hệ thống đang chạy |
| Học người | `/person_reid/start_enroll`; camera tự xong ở 100 % | camera đang gửi dữ liệu, xe không đang bám |
| Xong | `/person_reid/finish_enroll` (thẻ trên cùng chỉ cho bấm "Xong sớm" khi đủ 80 mẫu) | đang học |
| Xoá người đã học | `/person_reid/reset` | xe không đang bám |
| Bắt đầu bám | `/follow/enable` | planner đang chạy và **chưa** bật bám, đã học xong, LiDAR và `/odom` còn gửi dữ liệu, đúng 1 node ghi `/cmd_vel` |
| Tạm dừng | `/follow/disable` | luôn bấm được |
| Ghi | `run_full.sh --bag` (xem mục Ghi bag) | hệ thống đang chạy (hoặc có planner đang chạy), chưa ghi, ổ đĩa còn ≥ 1 GB |
| Dừng ghi | Ctrl-C tiến trình ghi (ros2 bag đóng file, ghi `metadata.yaml`) | đang ghi |
| **DỪNG** | `/follow/stop`; không gọi được thì `/follow/disable` | luôn bấm được, **không cần đăng nhập** |

"Tắt hệ thống" phải dừng xe trước rồi mới tắt các node, vì khi driver tắt thì khung xe giữ lệnh cuối thêm ~3 s (CLAUDE.md 7.3).

## Ghi bag

Nút **Ghi** chạy đúng lệnh `run_full.sh --bag` như khi ghi tay ở terminal, nên danh sách topic luôn trùng: `/scan /odom /cmd_vel /follow/target /follow/planner_status /person_reid/target /rssi/bearing /rssi/status /rssi/raw /tf /tf_static`. **Không ghi ảnh camera** (quá nặng). Bag lưu ở `~/bags/follow_<thángngày_giờphútgiây>` **trên máy robot**, khoảng 4–5 MB/phút.

- Thẻ ghi bag hiện thời gian, dung lượng và số topic đã có, ví dụ `9/11 topic`. "Chi tiết" ghi rõ topic nào **chưa có**: tắt RSSI thì thiếu 3 topic `/rssi/*`; camera chưa chạy thì thiếu `/person_reid/target`.
- **Tự dừng** 2 s sau khi hệ thống tắt (bấm Tắt hệ thống, hoặc hệ thống tự tắt vì lỗi), nên bag có cả đoạn cuối. Ổ đĩa còn < 0.3 GB thì cũng tự dừng.
- **Bag gần đây:** 6 bag mới nhất, có giờ, thời lượng, dung lượng. Bag ghi **"thiếu metadata"** là bag chưa đóng (đang ghi) hoặc bị tắt ngang; sửa bằng `ros2 bag reindex <thư mục>`.
- Có `ros2 bag record` chạy từ terminal thì web cũng nhận ra ("ngoài web") và dừng được.
- Web không xoá bag. Xoá bag cũ bằng tay trên máy robot.

## Pin

- **Pin máy robot:** đọc từ `/sys/class/power_supply/BAT*` mỗi 5 s: %, đang sạc / dùng pin / cắm điện không sạc, công suất, thời gian còn lại ước tính. Pin ≤ 20 % (vàng) hoặc ≤ 10 % (đỏ) khi đang dùng pin thì có cảnh báo trên cùng và một dòng sự kiện. Mini PC không có pin thì ghi "không có pin".
- **Pin khung xe:** số `/bw_dr03/power` do driver BW-DR03 phát, lấy trung vị 10 s vì điện áp tụt khi động cơ kéo. Driver đặt tên trường này là `power_v` nhưng **[CẦN XÁC NHẬN]** đơn vị và loại pin. Vì vậy trang chỉ hiện **số thô** và mặc định **không cảnh báo**. Biết đơn vị và ngưỡng rồi thì thêm tham số, ví dụ pin 12 V: `-p chassis_power_unit:=V -p chassis_power_warn:=11.8 -p chassis_power_bad:=11.3`.

## Dòng sự kiện

Máy chủ so trạng thái mỗi 0.5 s và ghi lại những gì thay đổi, có giờ:
- hệ thống: kiểm tra xong (đạt / mấy mục hỏng), khởi động, đang chạy, tắt, lỗi;
- nút đã bấm: nút gì, được hay không (kèm lý do), từ IP nào;
- xe bắt đầu / ngừng bám; học xong người (số mẫu), huỷ học, xoá người đã học;
- **đợt** của planner khi đang bám, có thời gian kéo dài: né vật cản, người khuất camera, tìm người (→ "thấy lại người" hay "ngừng bám"), bị chặn đường, planner dừng xe, đứng chờ không thấy người;
- khi hệ thống đang chạy: LiDAR / khung xe / camera / planner / RSSI ngừng gửi dữ liệu (và gửi lại sau bao lâu); 2 node cùng ghi `/cmd_vel`; mất tín hiệu beacon;
- ghi bag: bắt đầu, đã lưu (thời lượng, dung lượng, số topic, lý do dừng), lỗi;
- pin yếu.

Trạng thái chập chờn dưới 0.5–1 s không được ghi. Một đợt lặp lại trong 4 s mà chưa có sự kiện nào xen giữa thì gộp vào đợt cũ ("2 lần"). Mọi sự kiện được ghi **cùng file** `run_logs/web_<thángngày>.log` với nút bấm (cột 2 là `su_kien`), nên sau buổi chạy đọc lại được cả diễn biến. Trên trang giữ 200 sự kiện gần nhất từ lúc node web khởi động.

## Video camera

Trang hiện **video trực tiếp** (luồng MJPEG): máy chủ đẩy khung ngay khi camera có khung mới. Video chạy theo tốc độ camera, khoảng 14–15 hình/s, tối đa `image_max_fps`, rộng 480 px. Video chỉ được nhận khi có người đang xem: đóng trang hoặc tắt "Hiện video" quá 5 s thì node thôi nhận ảnh. Tối đa 3 người xem cùng lúc (`max_streams`); người thứ 4 bị từ chối. Nếu luồng video lỗi 3 lần liền, trang tự chuyển sang xin từng ảnh trong 60 s.

Góc trên ảnh có **nhãn chữ to** báo trạng thái camera:

| Nhãn | Ý nghĩa |
|---|---|
| **ĐANG HỌC NGƯỜI 45%** (+ số mẫu đã học, thanh xanh dương ở mép dưới ảnh) | đang học người |
| ĐANG HỌC — CHƯA THẤY NGƯỜI 52% (vàng) | đang học nhưng camera không thấy người: bước vào khung hình |
| ĐÃ HỌC XONG 100% (thanh xanh lá) | học xong, bấm được "Bắt đầu bám" |
| ĐANG THẤY CHỦ (xanh) / NGHI NGỜ — ĐANG XÁC MINH (vàng) / KHÔNG THẤY CHỦ (đỏ) | lúc đã học xong |

**Cách tính %:** camera chỉ học xong khi đủ **cả hai** điều kiện: ≥ 80 mẫu **và** ≥ 30 s (`enroll_min_samples`, `enroll_seconds` của node camera). Vì vậy % là phần chậm hơn trong hai cái, và 100% đúng là lúc học xong. Đủ 80 mẫu mà vẫn 99% là đang chờ cho đủ 30 s; % đứng yên lâu là camera không thấy người để lấy mẫu.

**Video trên web không ảnh hưởng tới việc bám người:** camera và ReID trên xe vẫn chạy đủ tốc độ, ảnh trên web chỉ là bản xem.

## An toàn

- **Nút DỪNG là dừng mềm qua Wi-Fi**, có thể trễ hoặc mất. Công tắc cắt nguồn động cơ trên xe vẫn là lớp an toàn cuối.
- Gọi được cả `/follow/stop` lẫn `/follow/disable` đều không được thì trang báo rõ: **dùng công tắc nguồn động cơ**.
- Mất kết nối quá 1.5 s thì cả trang chuyển đỏ: "MẤT KẾT NỐI — các nút không còn tác dụng". Xe **vẫn bám bình thường** (người dùng chọn 08/10). Video đang phóng to thì có dải đỏ ở mép dưới.
- Lệnh bấm phải có header `X-Robot-Web`, nên trang web lạ mở trong cùng trình duyệt không gọi hộ được.
- Mỗi lần bấm nút và mỗi sự kiện được ghi vào `run_logs/web_<thángngày>.log`: giờ, IP máy bấm, nút, kết quả.

## Tham số

`ros2 run robot_web web_server --ros-args -p ten:=gia_tri`

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `port` | 8080 | cổng web |
| `host` | `0.0.0.0` | địa chỉ nghe |
| `ws_dir` | tự tìm | thư mục workspace (chứa `src/person_follow_nav/scripts/run_full.sh`) |
| `password_file` | `~/.config/robot_web.conf` | file mật khẩu, dạng `{"password": "..."}` |
| `image_width` | 480 | bề ngang video gửi lên trang |
| `image_max_fps` | 15 | số hình mỗi giây tối đa của video. Hạ xuống 8 để đỡ CPU trên máy yếu |
| `jpeg_quality` | 65 | chất lượng JPEG |
| `max_streams` | 3 | số người xem video cùng lúc tối đa |
| `stop_wait_sec` | 0.5 | chờ sau `/follow/stop` rồi mới tắt các node |
| `probe_period_sec` | 0.25 | chu kỳ lấy tin mới nhất của các topic trạng thái (không áp cho video) |
| `bag_dir` | `~/bags` | thư mục bag để liệt kê; **phải trùng** `$HOME/bags` mà `run_full.sh --bag` ghi vào |
| `bag_min_free_gb` | 1.0 | ổ đĩa còn ít hơn thì không cho bắt đầu ghi |
| `bag_stop_free_gb` | 0.3 | đang ghi mà ổ đĩa còn ít hơn thì tự dừng ghi |
| `power_supply_dir` | `/sys/class/power_supply` | nơi đọc pin máy tính |
| `chassis_power_unit` | (trống) | đơn vị số `/bw_dr03/power`, ví dụ `V` — **[CẦN XÁC NHẬN]** |
| `chassis_power_warn` / `chassis_power_bad` | 0 (tắt) | ngưỡng cảnh báo / nguy hiểm pin khung xe |

## CPU

Đo 08/10 bằng `test_web.py`. Robot giả phát đúng tần số như xe thật: `/odom` 50 Hz; planner, camera và lệnh xe 15 Hz; LiDAR 10 Hz; ảnh 640×480 ở 15 Hz.

| | Một nhân CPU |
|---|---|
| Không ai mở trang | ~2 % |
| Mở trang, tắt video (kể cả dòng sự kiện, pin, quét tiến trình ghi bag) | 2.2–2.5 % |
| Xem video 15 hình/s (robot giả) | 8.7–9.1 % |
| Xem video với camera thật (webcam laptop, 14 hình/s, bản trước) | 10.1 % |
| Xem video, `image_max_fps:=8` | ~5.7 % |

**Cách làm:**
- **Topic trạng thái:** nằm ở node phụ `robot_web_probe`. Mỗi 0.25 s node chỉ lấy **tin mới nhất** của từng topic (QoS giữ 1 tin), và tính tần số từ số thứ tự tin nhắn. rclpy tốn ~0.75 ms cho mỗi tin, nên nhận đủ mọi tin thì tốn 8.2 %.
- **Video:** nằm ở node riêng `robot_web_video`, có luồng riêng. Mỗi khung mới chỉ mã hoá JPEG **một lần**, mọi người xem dùng chung; video không chen vào luồng xử lý nút bấm.
- **OpenCV trong node web chỉ dùng 1 luồng:** để mặc định thì các luồng phụ chạy quay vòng chờ việc, xem video tốn 20–24 %.
- **Thu nhỏ ảnh bằng `INTER_LINEAR`:** 0.5 ms mỗi khung, so với 1.9 ms của `INTER_AREA`.
- **Tìm tiến trình ngoài web** (launch và `ros2 bag record`): đọc `/proc` **một lần** mỗi 2 s cho cả hai, ~6 ms (0.3 %).
- **Dòng sự kiện:** trang chỉ tải lại danh sách khi số phiên bản (`ev.rev` trong `/api/status`) đổi.

## Kiểm tra (không cần xe)

```bash
cd ~/ros2_reid_from_github && source install/setup.bash
python3 src/robot_web/scripts/test_web.py      # ~2.5 phút, dòng cuối phải "=> DAT"
```

Bài kiểm chạy node web thật trong miền ROS riêng (`ROS_DOMAIN_ID` 77, chỉ localhost), cùng một robot giả, một `run_full.sh` giả (kèm `--bag` giả) và thư mục pin giả. Kết quả 08/10: **92/92 ĐẠT**. Các nhóm kiểm:
- đăng nhập;
- kiểm tra, khởi động, tắt hệ thống — có mục HỎNG thì không chạy; tắt thì `/follow/stop` đi trước Ctrl-C 0.51 s; nhận ra và tắt được hệ thống bật ngoài web;
- luật của từng nút;
- ghi bag: bị chặn khi hệ thống chưa chạy; ghi 1 nút, đọc tên bag + 3/4 topic, dung lượng tăng dần; dừng ghi đọc được `metadata.yaml`; tự ghi khi Khởi động và tự dừng **2.5 s sau** khi hệ thống tắt; lỗi ghi; nhận ra và dừng được ghi ngoài web;
- dòng sự kiện: thứ tự trạng thái hệ thống, nút bấm (tên, kết quả, IP), học xong, bắt đầu / ngừng bám, đợt "tìm người" 2.5 s (gộp khi lặp lại), LiDAR ngừng 2 s, 2 node ghi `/cmd_vel`, ghi vào file nhật ký;
- pin máy 15 % → 8 % (cảnh báo rồi nguy hiểm, không báo lặp), pin khung xe 12.34 → 11.0 (dưới ngưỡng);
- ảnh camera và video: ≥ 12 hình/s, rộng 480 px, tối đa 3 người xem, đóng trang thì thôi nhận ảnh;
- nút DỪNG khi mất service;
- cả 3 node (`robot_web`, `robot_web_probe`, `robot_web_video`) không publish topic nào;
- CPU khi mở trang và khi xem video.

Giao diện còn được kiểm bằng cách chạy JavaScript của trang với 15 trạng thái mẫu và 33 loại sự kiện (không lỗi, mọi câu máy chủ đều được dịch), và chụp ảnh Chrome headless ở khổ điện thoại 390 px, sáng / tối, và màn hình rộng.

## Ghi chú

- **Chạy nền bằng `&` trong script:** shell đặt SIGINT ở trạng thái *bỏ qua* cho tiến trình nền, và trạng thái đó truyền xuống cả hệ thống robot do web khởi động. Node tự đặt lại SIGINT nên vẫn tắt được bằng Ctrl-C / `kill -INT`. SIGTERM (systemd) cũng tắt gọn.
- Trang dùng HTTP thường, không có HTTPS. Chỉ dùng trong mạng tin cậy: Wi-Fi nhà, hoặc sau này là Wi-Fi riêng của robot.
- **Chưa làm:** tự chạy khi bật máy (systemd) và Wi-Fi riêng của robot. Đó là bước chuyển sang mini PC.
- **Sửa `web_node.py` thì phải khởi động lại node web; sửa `index.html` chỉ cần tải lại trang** (máy chủ đọc file mỗi lần tải).
- Bag do web bắt đầu mà node web khởi động lại giữa chừng thì thành bag "ngoài web": vẫn dừng được bằng nút, nhưng **không** tự dừng khi tắt hệ thống.
- **[CẦN XÁC NHẬN]** người dùng đã mở bản đầu trên điện thoại qua Wi-Fi nhà; chưa xem giao diện mới trên điện thoại, chưa điều khiển xe thật, chưa ghi bag bằng web trên xe; đơn vị của `/bw_dr03/power`.
