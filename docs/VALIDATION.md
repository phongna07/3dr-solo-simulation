# Kết quả kiểm tra thực tế

Kiểm thử ngày **09/10/2026**, trong Ubuntu WSL2 / Docker Desktop 29.8.2, Compose 5.5.1, Linux amd64. Docker được cấp 14 CPU và khoảng 15,37 GiB RAM. WSLg hoạt động; `glxinfo -B` nhận diện **llvmpipe, Accelerated: no**, nên camera dùng rendering phần mềm.

## Build và phiên bản

Cả `solo-simulation:gd0` và `solo-companion:gd0` build thành công. Simulation khoảng 6,98 GB; companion khoảng 3,33 GB theo Docker image inspect, chưa tính build cache và dữ liệu run. Image ID xuất vào `artifacts/simulation-image.json`; registry chưa được publish.

| Thành phần runtime | Kết quả |
|---|---|
| ROS 2 | Jazzy / Ubuntu 24.04 |
| MAVROS | Source overlay 2.15.1, prefix `/opt/mavros_ws/install/mavros` |
| ArduCopter | 4.5.3, FCU báo `040503ff (ee14ff38)` cho cả ba UAV |
| Gazebo | gz-sim 8.15.0 / Harmonic |
| Python ROS | 3.12.3 |
| Python Waf | 3.11.16, venv riêng |

Source commits và base image digests nằm trong `locks/versions.json`. APT manifests, submodule commits và geoid checksum nằm trong image, được xuất bởi `./scripts/dev lock`. APT lấy package từ kho hiện hành khi rebuild; dùng cùng image digest/archive để thống nhất tuyệt đối giữa các máy.

## Các kiểm tra đã pass

- 16 pytest trong container simulation và companion: fleet/ports, world transformation, state isolation/resume, chặn flight trên hardware hoặc health cũ, và khởi tạo ROS flight node.
- Bash syntax và shellcheck cho entrypoint/CLI; Compose overrides baseline, WSLg, DRM, NVIDIA, hardware serial parse hợp lệ. Parse không xác nhận GPU/serial thật.
- World ba UAV `control` và `reference` sinh từ plugin đã pin được `gz sdf -k` chấp nhận. Có warning `gz_frame_id` extension từ upstream.
- Khởi động bằng `./scripts/dev sim`, xuất manifests bằng `./scripts/dev lock`, dừng qua `./scripts/dev stop` đã chạy thực tế. CLI tự chọn WSLg override.
- Ba namespace/SYSID/FCU TCP và JSON UDP độc lập; mỗi run có thư mục SITL riêng và phiên mới không dùng lại eeprom của phiên trước.
- Telemetry IDs 30/32 ACK, pose hữu hạn liên tục; frame runtime xác nhận `uav1/map`, `uav2/map`, `uav3/map`.
- Plugin YAML được áp qua parameter services trước readiness. Key upstream cũ như `timesync_avg_alpha`, `tf.send_fcu` không còn được plugin khai báo được bỏ qua với warning. Giá trị bị từ chối trên parameter còn được khai báo là lỗi khởi tạo.

## Profile control: bài bay đủ ba UAV

Phiên `20261009T110032Z-27038` đạt readiness sau khoảng 90 giây wall clock, rồi lần lượt GUIDED → ARM → takeoff 3 m, có xác nhận độ cao từ pose.

| UAV | Độ cao hover quan sát | Pose rate wall clock | Frame |
|---|---:|---:|---|
| uav1 | 2,981 m | 9,940 Hz | uav1/map |
| uav2 | 2,982 m | 9,939 Hz | uav2/map |
| uav3 | 2,977 m | 9,939 Hz | uav3/map |

Mẫu pose dài 15 giây. RTF mẫu 5 giây sau takeoff, GUI đóng: trung bình **0,99965**, khoảng **0,99797–1,00124**. Đây là số đo trên máy kiểm thử, không bảo đảm cho máy khác hay workload MPC tương lai.

Velocity `(0.5, 0, 0)` được phát cho ba UAV trong 6 giây, sau đó zero velocity 2 giây. Position UAV1 `(3, 0, 3)` được phát 10 giây; pose sau bài là **(3,022; 0,013; 3,000)** m trong ENU local. LAND cho cả ba UAV có xác nhận disarm, rồi container dừng sạch qua `dev stop`.

Bằng chứng local: `artifacts/takeoff-release.log`, `control-release-airborne.json`, `control-release-stats.txt`, `position-release-observed.yaml`, `stop-release.log`; run lưu world/config/ROS logs/flight logs. `artifacts/` và `runs/` được gitignore để không đưa dữ liệu lớn vào source control.

## Profile reference: camera/gimbal nguyên bản

Phiên `20261009T110529Z-27913` đạt readiness cả ba UAV sau khoảng 220 giây. Gazebo có ba topic camera image/camera_info riêng, gimbal command topics và video ports độc lập. Camera upstream 640×480, update rate 10 Hz theo simulation time. Camera zoom command cũng được cấu hình riêng theo UAV.

RTF mẫu ban đầu khoảng **0,23**. Mẫu pose ổn định 15 giây cho cả ba UAV khoảng **1,92 Hz wall clock**. Có giai đoạn tải render làm pose giảm xuống khoảng **0,22 Hz**, heartbeat hết timeout, readiness chuyển false rồi tự khôi phục. Không ARM trong bài reference này.

`reference` chạy được và giữ camera/gimbal, nhưng **chưa đạt môi trường điều khiển ổn định trên renderer phần mềm này**. Dùng `control` làm baseline develop; nghiệm thu lại reference trên GPU phù hợp hoặc cấu hình camera giảm tải có ghi nhận thay đổi. Tăng timeout không chứng minh đã đáp ứng tần số controller.

GUI client kết nối với server qua WSLg và load scene/plugins; đóng client không dừng simulation server. Đã bổ sung `libqt5svg5` và kiểm tra lại, không còn warning thiếu định dạng SVG. Chưa đánh giá hình ảnh trực quan bằng screenshot, chưa xác nhận hardware GPU acceleration. Topic camera không thay thế kiểm tra video decode/stream hoặc bridge ảnh vào ROS.

## Giới hạn của nghiệm thu

Chưa chạy GitHub Actions trên remote runner, image ARM64 trên Pi 4, kết nối FCU Solo thật, firmware upgrade, driver LD19, GoPro vừa stream vừa record, MPC, common ENU hoặc failsafe bay thật. Các bước chuyển phần cứng và thông tin còn thiếu nằm trong [HARDWARE.md](HARDWARE.md). Kiểm tra simulation không xác nhận các hạng mục này.

Sau nghiệm thu, container simulation được dừng; image, source, `.env`, logs và manifests vẫn giữ để chạy lại.
