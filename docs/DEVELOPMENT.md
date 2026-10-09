# Kiến trúc develop và mở rộng

## Phân chia

```text
simulation.launch.py: world + Gazebo + SITL + MAVROS + readiness
hardware.launch.py:                         MAVROS + readiness
                         ↓ cùng topics/interfaces
               controller / estimator / perception (bổ sung)
```

`solo_bringup` là ament_python package. `configuration.py` kiểm tra fleet, `world.py` sinh world từ pinned upstream, `runtime.py` tạo run và MAVROS frame config. `launching.py` dùng chung MAVROS và shutdown policy cho hai môi trường.

`parameters.py` áp cấu hình plugin bằng `ListParameters`/`SetParameters` vì plugin nodes của commit MAVROS này không dùng global CLI arguments. Readiness đợi cấu hình được áp thành công; không chỉ kiểm tra YAML sinh ra trên đĩa. Frame của UAS được đặt riêng qua `base_link_frame`, `map_frame`, `odom_frame` trên `mavros_node`.

Docker Compose quản lý container và thiết bị; Bash `scripts/dev` là CLI thao tác ngắn; ROS 2 launch viết bằng Python quản lý các tiến trình bên trong. Cách này dùng được namespace/parameters, log theo tiến trình và event shutdown của ROS, không phải tự xây process supervisor bằng `subprocess` hoặc mở terminal bằng script. GUI là client riêng; tmux chỉ dùng khi muốn xem nhiều bảng log cùng lúc. Node readiness và flight dùng API ROS có kiểu thay vì ghép nhiều lệnh `ros2 service call` rồi parse chuỗi.

Repo hiện tại không có MPC source. Tránh xây controller placeholder tự tích phân state rồi coi là pose thật. Thêm solver/planner bằng ROS package riêng khi đã có source và xác định dependencies.

## Sửa source

Container bind mount repo vào `/workspace`; Python imports ưu tiên source sống. Các console entries và ament index đã được build trong image. Sửa module/launch/config hiện có: restart simulation. Thêm entry point hoặc đổi package metadata: rebuild image. Không sửa code host ArduPilot/plugin nằm ngoài repo.

Để build package mới trong container:

```bash
./scripts/dev shell
cd /workspace/ros_ws
colcon build --symlink-install
source install/setup.bash
```

Overlay này chỉ có hiệu lực trong terminal đã source. Nếu muốn stack dùng overlay, thêm source tương ứng vào entrypoint sau khi đã kiểm tra dependencies. Giữ thứ tự ROS → `/opt/mavros_ws` (MAVROS pin) → project overlay. APT/image thay đổi cần cập nhật release; không dựa vào cài tay trong container đang chạy vì container mới sẽ mất các thay đổi đó.

## Interface MPC

- Input pose/velocity lấy từ MAVROS, tuổi dữ liệu có kiểm soát.
- Sensor interface ổn định: `/uavN/scan`, `/uavN/camera/image_raw`, `/uavN/camera/camera_info` khi bổ sung adapters.
- Output reference qua bộ phát setpoint riêng; solver không trực tiếp khóa vòng publish.
- State machine: disconnected → initializing → ready → guided → flying → landing; chính sách lỗi rõ ràng.
- Frame: `world`, `uavN/map`, `uavN/base_link`, `uavN/laser`, camera optical frame. Namespace không tự giải quyết transform.

Trước khi nối MPC, hiệu chuẩn `world → uavN/map`; spawn không tự chứng minh offset EKF. Chuyển position bằng rotation+translation, velocity/acceleration chỉ rotation. Chuẩn ENU/NED và `type_mask` phải được kiểm chứng trên firmware mục tiêu.

## Clock

Baseline ROS wall time giống hướng dẫn gốc. Muốn simulation time, thêm ros_gz bridge `/clock`, cấu hình toàn bộ nodes liên quan và kiểm tra MAVROS timestamp/timesync với FCU. Chỉ đặt `use_sim_time=true` mà không có `/clock` sẽ làm timer đứng. Không coi solver thời gian 100 ms wall và 100 ms simulation là cùng tiêu chí khi RTF khác 1.

## Kiểm thử

`./scripts/dev test` kiểm tra fleet uniqueness/ports và world transformation giữ IMU scope, loại gimbal đúng profile và tách video ports. Smoke test thực tế kiểm tra heartbeat/pose trên ba UAV. Flight CLI kiểm tra GUIDED/ARM/takeoff và xác nhận LAND/disarm. Không gọi test ARM/takeoff tự động trên hardware.

CI nên chia hai lớp: unit/build trên runner thường; smoke simulation profile control trên Linux runner đủ tài nguyên. Khi đã có camera/LiDAR simulation, thêm bài cảm biến và GPU runner riêng. Benchmark solver trên Pi thật trước khi đặt deadline được bảo đảm.
