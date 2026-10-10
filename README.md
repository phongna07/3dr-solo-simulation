# 3DR Solo — Docker SITL / ROS 2 / Gazebo

Môi trường develop cho **ArduCopter 4.5.3 + ROS 2 Jazzy + MAVROS 2.15.1 + Gazebo Harmonic**, một world với ba Iris. Chạy toàn bộ stack bằng một lệnh, không phải mở bảy terminal thủ công.

Thiết kế có image `companion` riêng để tiến tới Raspberry Pi 4 trên 3DR Solo nguyên bản. Hiện tại companion cung cấp MAVROS và readiness; **chưa port MPC, chưa tích hợp driver LD19/GoPro, chưa xác nhận bay trên Solo thật**. GoPro cần xác định model và chế độ hỗ trợ vừa record vừa stream.

## Bắt đầu trên Windows 11 + Ubuntu WSL2

1. Bật WSL2 và WSLg; trong Docker Desktop bật Linux containers, WSL 2 backend và WSL Integration cho Ubuntu. Không cần cài ROS/Gazebo lên host.
2. Giữ repo trong Linux filesystem, ví dụ `~/projects/3dr-solo`, không dùng `/mnt/c/...` làm workspace build.
3. Chạy trong terminal Ubuntu WSL:

```bash
cp .env.example .env
./scripts/dev doctor
./scripts/dev build sim
./scripts/dev sim
```

Build đầu tải nhiều package và compile ArduPilot/plugin; các lần chạy sau dùng lại image. `sim` chạy nền và chờ heartbeat + telemetry + ít nhất ba pose có timestamp khác nhau trên mọi UAV. Không tự ARM. Profile mặc định `control` không có gimbal/camera để giảm tải.

`connected=False` trong các giây đầu có thể là trạng thái đang khởi tạo. Lệnh theo dõi container và báo lỗi kèm log nếu stack đã dừng; không tiếp tục chờ trên health cũ. Khi đã `connected=True` nhưng `pose=0`, tiếp tục chờ EKF theo readiness.

Đã nghiệm thu ba UAV với takeoff/setpoint/LAND trong Docker tại workspace này; xem [kết quả và giới hạn kiểm thử](docs/VALIDATION.md). Camera/gimbal trên renderer phần mềm có thể làm readiness gián đoạn, nên cần đo lại trước khi dùng profile `reference` cho controller.

Xem GUI ở terminal khác:

```bash
./scripts/dev gui
```

Hoặc `./scripts/dev sim --gui` để đợi ready rồi mở GUI. Script tự thêm `compose.wslg.yaml` khi thấy WSLg. Đóng GUI không dừng server. Nếu GUI lỗi, simulation vẫn có thể hoạt động qua ROS; xem [WSL/GPU](docs/WSL_GPU.md).

## Bài bay kiểm tra

Chỉ chạy sau khi `sim` báo ready; các lệnh sau **chỉ dùng cho simulation**:

```bash
./scripts/dev status
./scripts/dev flight takeoff --altitude 3
./scripts/dev flight velocity --xyz 0.5 0 0 --seconds 6
./scripts/dev flight position --uav uav1 --xyz 3 0 3 --seconds 10
./scripts/dev flight land
./scripts/dev stop
```

Takeoff chuyển GUIDED, xác nhận mode thực tế, ARM/takeoff liền nhau từng UAV rồi xác nhận độ cao. Velocity phát song song ở 10 Hz, cuối lệnh gửi zero velocity hai giây. Position là tọa độ ENU **local riêng UAV đã chọn**, không phải world chung. Dừng publisher khác trước khi đổi loại setpoint hoặc LAND.

`stop` yêu cầu LAND và chờ disarm trước khi dừng container. Nếu stack chưa boot, đã crash hoặc LAND thất bại, dùng `./scripts/dev stop --force` để dừng tiến trình simulation; lệnh này không hạ cánh. Không dùng flight helper để điều khiển drone thật.

## Hai profile

Checkpoint một UAV trên máy mới, sau khi dừng phiên cũ:

```bash
FLEET_CONFIG=/workspace/config/fleet-one.yaml ./scripts/dev sim
```

Không đặt FLEET_CONFIG lâu dài thì lần chạy tiếp mặc định dùng ba UAV. World một UAV tên `iris_one`; đổi tên topic stats tương ứng.

```bash
./scripts/dev stop
./scripts/dev sim --profile reference
```

| Profile | Mục đích |
|---|---|
| `reference` | Iris với gimbal/camera như hướng dẫn gốc; tách topic gimbal và video port cho từng UAV; cần EGL/rendering cho camera |
| `control` | Giữ Iris, rotor physics, IMU, NavSat và ArduPilotPlugin; bỏ gimbal/camera và hệ thống rendering sensor |

Profile `control` thay đổi khối lượng/cấu hình do tháo gimbal; cần đo lại khi so sánh động lực học. Cả hai đều là Iris, **chưa phải model Solo đã hiệu chỉnh**. Profile reference chưa bridge ảnh camera/LiDAR vào ROS. Các bài kiểm tra controller cần chọn rõ profile trong kết quả.

## Cấu hình và dữ liệu

- [config/fleet.yaml](config/fleet.yaml): instance, SYSID, namespace và spawn. TCP/JSON port được suy ra từ instance theo ArduPilot 4.5.3.
- [docker/Dockerfile](docker/Dockerfile): targets `simulation`, `companion`.
- [compose.yaml](compose.yaml): container, user, môi trường, volume, healthcheck.
- [ros_ws/src/solo_bringup](ros_ws/src/solo_bringup): ROS launch, world generator, readiness và CLI bài bay.
- [locks](locks): commit, digest nền và Python lock có hash.
- `runs/<run-id>/`: world, fleet snapshot, parameter/runtime SITL, ROS logs, health và provenance.
- `runs/latest`: symlink tới phiên simulation mới nhất. Phiên hardware dùng `runs/hardware/latest` riêng.

Mặc định mỗi lần chạy tạo state mới; không xóa log cũ, không dùng chung eeprom giữa UAV. Muốn dùng lại parameter đã lưu:

```bash
./scripts/dev sim --profile control --resume <run-id>
```

Resume yêu cầu cùng fleet và profile; vẫn phải bật telemetry/chờ readiness lại. Bảo quản image cùng run để tái lập môi trường đầy đủ.

## Develop và kiểm tra

```bash
./scripts/dev shell
./scripts/dev logs
./scripts/dev test
./scripts/dev lock
```

Source bind mount tại `/workspace`; entrypoint ưu tiên Python module trong repo và launch file cũng đọc trực tiếp từ repo. Sửa Python/launch/config rồi restart phiên, không cần build lại image. Khi thêm executable mới, đổi `setup.py`, thêm dependency hoặc C++ package, cần build image hoặc tạo colcon overlay. Xem [DEVELOPMENT.md](docs/DEVELOPMENT.md).

Không chạy `sudo pip` trên host hoặc trộn Python 3.11 build Waf với Python 3.12 ROS. Repo này chưa có source/dependency của project MPC mà hướng dẫn gốc nhắc tới; không tự cài package MPC suy đoán.

## Tài liệu đầy đủ

- [Vận hành, ROS CLI, log và troubleshooting](docs/OPERATIONS.md)
- [Windows WSLg và GPU](docs/WSL_GPU.md)
- [Dependencies, image digest và chia sẻ cho nhóm](docs/REPRODUCIBILITY.md)
- [Kiến trúc develop và nối MPC](docs/DEVELOPMENT.md)
- [Raspberry Pi 4, Solo, LD19 và GoPro](docs/HARDWARE.md)
- [Kết quả kiểm tra thực tế](docs/VALIDATION.md)
- [Hướng dẫn cài thủ công và lịch sử debug](HUONG_DAN_GD0_SITL_ROS2_GAZEBO_3_UAV.md)

## Nguồn chính

- [ArduPilot Gazebo SITL](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [Plugin commit được pin](https://github.com/ArduPilot/ardupilot_gazebo/tree/082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5)
- [Gazebo Harmonic installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [ROS Jazzy Ubuntu installation](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)
