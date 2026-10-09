# Đường chuyển sang Pi 4 / 3DR Solo / LD19 / GoPro

Phần này là nền triển khai và kế hoạch nghiệm thu; **chưa xác nhận trên thiết bị thật**. Hardware launch hiện chỉ chạy MAVROS + readiness; cấu hình sensor disabled sẽ báo rõ nếu người dùng bật khi chưa tích hợp driver. Không flash firmware hay ARM tự động.

## Image companion

Image target `companion` có ROS Jazzy, MAVROS, readiness, rosbag và thư viện ROS camera cơ bản; không có Gazebo, SITL, Waf hoặc venv Python 3.11. Chưa cài MPC/LD19/GoPro SDK vì chưa có source MPC và model GoPro.

Trên Pi 4 dùng OS 64-bit, ưu tiên Ubuntu Server 24.04 ARM64 nếu thiết bị/driver đáp ứng. Kernel, UART, udev, mạng, đồng bộ giờ và cấp nguồn thuộc host. Camera/SDK có yêu cầu OS riêng phải kiểm tra trước khi chọn ảnh host cuối cùng.

Build native trên Pi:

```bash
cp .env.example .env
./scripts/dev build companion
```

Hoặc build ARM64 trên máy hỗ trợ Buildx:

```bash
docker buildx build --platform linux/arm64 --target companion \
  -f docker/Dockerfile -t solo-companion:gd0-arm64 --load .
```

Build emulation không xác nhận hiệu năng runtime. Phải test binary và ROS nodes trên Pi thật; dependency amd64 không tự hoạt động trên ARM64.

## Bước trung gian Pi controller ↔ laptop SITL

Đưa controller/companion lên Pi, simulation ở laptop, trước khi nối Solo. Baseline container không expose FCU endpoint nên cần cấu hình riêng:

1. Chọn đường MAVLink LAN và địa chỉ Pi/laptop; xác định đúng endpoint TCP hoặc UDP, port, SYSID.
2. Nếu dùng TCP 5760, cổng SITL phải listen trên interface container phù hợp và Compose phải publish; kết nối từ Pi dùng IP LAN laptop, không `127.0.0.1`. Kiểm tra Windows firewall/WSL/Docker routing thực tế.
3. Mỗi link FCU do một MAVROS làm nguồn điều khiển. Tắt MAVROS tương ứng trên laptop hoặc thêm MAVLink router với policy rõ ràng, tránh tranh chấp trên cùng cổng TCP.
4. Nếu dùng ROS qua LAN, đổi discovery range/config DDS, kiểm tra multicast và QoS. Telemetry từ FCU có thể dùng MAVLink riêng để không phụ thuộc ROS discovery cho mọi thứ.

Đây là mở rộng networking cần triển khai và kiểm tra theo mạng thật; Compose baseline localhost chưa tự thực hiện test Pi ↔ laptop.

## Solo nguyên bản

Stock hardware không đồng nghĩa firmware stock. Đọc version FCU, Solo/OpenSolo, board/bootloader và backup parameters trước khi quyết định nâng cấp. OpenSolo 4 có hỗ trợ stock cubes, và firmware server có `CubeSolo` 4.5.3; các thông tin này chưa thay thế kiểm tra tương thích hệ thống Solo hiện tại.

- [OpenSolo release notes](https://github.com/OpenSolo/OpenSolo/releases)
- [CubeSolo 4.5.3](https://firmware.ardupilot.org/Copter/stable-4.5.3/CubeSolo/)
- [Solo developer documentation](https://ardupilot.org/dev/docs/solo.html)

ArduPilot chạy trên FCU; Pi chạy ROS/MAVROS/MPC. Giữ Solo i.MX6/controller link hoạt động khi thêm Pi. Chọn UART hoặc MAVLink routing qua mạng sau khi xác minh cổng/dây/ownership. Không áp ngay TELEM2 mapping chung lên Solo mà chưa kiểm tra carrier.

Ví dụ sau khi host đã có thiết bị serial thật được đặt tên `/dev/solo-fcu`, chỉnh baud/SYSID trong `config/hardware.yaml` cho khớp FCU, rồi:

```bash
export COMPOSE_FILE=compose.yaml:compose.hardware-serial.yaml
export DIALOUT_GID=$(getent group dialout | cut -d: -f3)
./scripts/dev companion
docker compose --profile hardware logs -f companion
./scripts/dev companion-stop
```

Hardware profile dùng host network trên Linux Pi, device mapping hẹp và dialout group; không dùng `privileged`. Chạy trên Windows/WSL có thiết bị USB cần USB passthrough riêng; không coi đó là cấu hình Pi chuẩn.

Hardware readiness yêu cầu local pose giống simulation; trong môi trường không có GPS/localization, nó có thể không ready dù MAVROS connected. LiDAR scan hay camera image không tự tạo pose EKF. Cần triển khai estimator/GPS/VIO/SLAM và luồng fusion tương ứng.

`companion-stop` chỉ dừng phần mềm Pi, **không LAND** drone. Flight helper của repo từ chối mọi run hardware. Chính sách timeout/failsafe phải được thiết kế và test trên FCU/companion trước khi điều khiển bay thật.

## LD19

Driver chính thức: [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2). Thêm vào workspace bằng commit pin sau khi build/test trên Jazzy ARM64. Nên thử USB–serial trước để kiểm tra dữ liệu độc lập. Udev rule đặt tên ổn định dựa trên VID/PID và serial adapter; không đưa rule chung đoán thiết bị vào host tự động.

Khi tích hợp, mở rộng Compose device mapping `/dev/ld19` và hardware launch để chạy `ld19.launch.py` với topic/frame đúng. Đo rate, mất packet, timestamp và TF extrinsics so với `base_link`.

LD19 là LiDAR 2D; mặt phẳng scan nghiêng theo drone. Không coi scan là bản đồ vật cản 3D hay nguồn pose đầy đủ. Model Gazebo LD19 tương lai phải mô phỏng scan plane/range/rate/noise theo datasheet và cách lắp thực tế.

## GoPro vừa ghi vừa truyền ảnh

Yêu cầu: lưu video trên thẻ GoPro đồng thời truyền ảnh vào Pi. Chưa biết model HERO, nên chưa chọn driver/giao thức và chưa bảo đảm concurrent recording + streaming.

Theo [Open GoPro](https://gopro.github.io/OpenGoPro/) và [streaming FAQ](https://gopro.github.io/OpenGoPro/docs/faq/), USB/Wi-Fi/streaming modes phụ thuộc model/firmware. USB có thể là network-over-USB, không phải UVC `/dev/video0`. HDMI capture cũng cần kiểm tra camera có output/chế độ tương ứng.

Camera adapter tương lai:

```text
GoPro → video trên SD
      → stream/capture → decode → ROS Image + CameraInfo → perception
```

Bắt đầu stream xử lý độ phân giải thấp, giữ recording chất lượng cao trên GoPro nếu model hỗ trợ đồng thời. Đo latency end-to-end, buffer, frame drops và CPU khi MPC/LD19 hoạt động. Arrival timestamp không tự bằng exposure timestamp; localization cần đồng bộ thời gian thật. Nếu dùng gimbal/stabilization, extrinsics và calibration có thể thay đổi; không gán fixed camera TF khi góc gimbal đang đổi.

## Tiêu chí trước khi bay

1. Companion ARM64 boot/restart đúng, không ARM tự động; điện nguồn và tản nhiệt Pi ổn định dưới tải.
2. Telemetry/SYSID/mode và đường điều khiển Solo không tranh chấp với hệ thống có sẵn.
3. Cảm biến có timestamp/frame/calibration đúng; nguồn định vị cho MPC đã xác nhận.
4. Solver p95/p99, tuổi state, jitter setpoint, timeout và mất kết nối được đo trên Pi.
5. Bench với cánh quạt tháo: telemetry, modes, reconnect và failsafe.
6. Bài bay vị trí đơn giản trước, rồi MPC một UAV, sau cùng nhiều UAV/common ENU.
