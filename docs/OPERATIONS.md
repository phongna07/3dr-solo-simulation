# Vận hành simulation

## Topology

Một container `sim` chứa Gazebo server, ba SITL, ba MAVROS và readiness node. TCP/UDP trao đổi trên localhost **trong container**. Compose không publish cổng FCU ra host, ROS discovery mặc định localhost. Dùng ROS CLI qua `./scripts/dev shell` hoặc `docker compose exec`, không cần discovery qua NAT của WSL/Docker Desktop.

| UAV | Instance | SYSID | JSON UDP | MAVLink TCP | Topic prefix |
|---|---:|---:|---:|---:|---|
| uav1 | 0 | 1 | 9002 | 5760 | /uav1/mavros |
| uav2 | 1 | 2 | 9012 | 5770 | /uav2/mavros |
| uav3 | 2 | 3 | 9022 | 5780 | /uav3/mavros |

Launch kiểm tra port chưa bị chiếm, tạo world từ model/plugin đã pin, chạy binary trực tiếp với hai defaults tuyệt đối. Không dùng launcher `sim_vehicle.py --count` có bug đường dẫn của tag cũ.

## Readiness

Node sử dụng subscription/service ROS có kiểu, không parse output CLI. Với mỗi UAV:

1. Chờ state connected.
   Đồng thời đọc và áp dụng những giá trị YAML khác hiện tại qua ROS parameter services, xác nhận phản hồi trước khi ready. Commit MAVROS đã pin đặt `use_global_arguments(false)` cho plugin nodes, nên file YAML trên CLI không tự áp vào các node này. Các key YAML upstream không còn được plugin khai báo được bỏ qua với warning; parameter được khai báo mà từ chối giá trị sẽ làm stack dừng.
2. Yêu cầu `LOCAL_POSITION_NED` ID 32, `ATTITUDE` ID 30 ở 10 Hz, kiểm tra service response. Các yêu cầu được thực hiện lần lượt trên mỗi FCU.
3. Chờ ít nhất ba pose hữu hạn có timestamp khác nhau; lần nhận mới nhất dưới 10 giây wall clock.
4. Ghi `health.json`, publish `/solo/readiness` và cập nhật Docker health.

State phải mới trong 20 giây wall clock. Khi disconnect, telemetry/pose counters reset và được yêu cầu lại khi reconnect. Deadline khởi tạo mặc định 600 giây; readiness timeout khiến ROS launch dừng stack. Sau khi từng ready, monitor tiếp tục sống và báo mất readiness, không tự ARM/restart FCU.

`dev sim`/`dev wait` kiểm tra container mỗi hai giây. Nếu stack đã dừng, lệnh thoát ngay kèm log gần nhất, kể cả Docker exit code là 0; không tiếp tục lặp `connected=False` từ file health cũ tới hết 600 giây. Health của phiên trước khi restart/resume không được dùng để báo ready cho container mới.

Readiness chỉ xác nhận luồng telemetry/pose, chưa xác nhận prearm, chất lượng EKF, calibration, common ENU hay an toàn bài bay. Với RTF rất thấp, tăng timeout chưa giải quyết việc không đạt tần số control.

## Quan sát và ROS CLI

```bash
./scripts/dev status
./scripts/dev logs
./scripts/dev shell
```

Trong shell container (đã source ROS):

```bash
ros2 topic echo /uav1/mavros/state --once
ros2 topic echo /uav1/mavros/local_position/pose --qos-reliability best_effort --once
ros2 topic hz /uav1/mavros/local_position/pose
ros2 topic echo /solo/readiness --once
timeout 5s gz topic -e -t /world/iris_3uav/stats
gz sdf -k /workspace/runs/latest/world.sdf
```

`timeout` trả 124 khi kết thúc thời gian giới hạn là bình thường. Tất cả tiến trình ROS launch xuất screen và log vào `runs/<id>/logs`. Binary SITL lưu eeprom/flight log trong `sitl/<instance>`. `docker compose logs` có prefix service; ROS launch có prefix process để nhận diện UAV.

Nhiều terminal không cần chạy lại môi trường: mỗi terminal dùng `./scripts/dev shell`. Có thể dùng tmux bên trong một shell, tạo pane cho state, pose, Gazebo stats và CLI. Đừng chạy thêm MAVROS vào cùng FCU chỉ để xem dữ liệu.

## Clock, frame và setpoint

Phiên mặc định giữ hành vi hướng dẫn gốc: ROS `use_sim_time=false`, không bridge `/clock`. SITL JSON physics vẫn theo Gazebo. Tần số 10 Hz được yêu cầu từ FCU có thể thấp hơn trên wall clock khi RTF thấp.

Các frame MAVROS được prefix như `uav1/map`, `uav1/base_link` để tránh trùng TF. Đây vẫn là origin riêng của FCU; không tạo transform `world → uavN/map` từ spawn một cách tự động. Muốn MPC formation, phải hiệu chuẩn common ENU và rotation/translation.

Flight helper chỉ dùng simulation, xác nhận service ACK **và** state/pose cho mode/takeoff. `position` yêu cầu chọn một UAV. Velocity/position dùng ENU và stamp ROS wall time. Một publisher điều khiển cho mỗi loại/UAV; tắt controller/publisher khác trước bài CLI.

```bash
./scripts/dev flight takeoff --uav uav1 --altitude 3
./scripts/dev flight velocity --uav uav1 --xyz 0.5 0 0 --seconds 6
./scripts/dev flight position --uav uav1 --xyz 3 0 3 --seconds 10
./scripts/dev flight land
```

Nếu takeoff thất bại giữa fleet, các UAV trước đó có thể đã bay: đọc state rồi thực hiện LAND. Không tắt arming checks để che lỗi. Flight helper không tự bay lại sau lỗi.

## Dừng và restart

```bash
./scripts/dev stop
./scripts/dev sim
```

Stop gửi LAND cho mọi UAV, chờ disarm, rồi dừng ROS launch/container. Ctrl+C khi tail log chỉ dừng tail. Đóng GUI chỉ đóng client. `stop --force` dừng tiến trình ngay, dùng khi stack boot lỗi/crash hoặc muốn kết thúc mô phỏng cưỡng bức. Không sử dụng lệnh này như chính sách dừng drone thật.

ROS launch dừng cả stack khi Gazebo, SITL, MAVROS hoặc readiness process exit. Không tự respawn FCU giữa thí nghiệm. Compose `init: true` hỗ trợ thu hồi child processes; stop grace period 30 giây.

## Lỗi thường gặp

| Dấu hiệu | Kiểm tra |
|---|---|
| Docker socket permission denied | Docker Desktop đang chạy, bật WSL Integration; không đổi quyền socket bằng chmod 777 |
| MAVROS runtime khác 2.15.1 | Kiểm tra đã chạy solo-entrypoint và source `/opt/mavros_ws/install/setup.bash`; bản APT chỉ làm nền dependency |
| `connected=False` mãi, log có `setpoint_attitude ... invalid allocator` | Cập nhật source bootstrap: chỉ set parameter khác giá trị hiện tại. Không dùng bản cũ ghi lại `use_quaternion=false` mỗi lần boot; xem DEVELOPMENT.md |
| TCP opened nhưng disconnected | Log SITL: defaults/PANIC/JSON sensor; cổng mở không chứng minh FCU đã boot |
| Connected, chưa có pose | Telemetry ACK, EKF và RTF; đọc readiness summary |
| PreArm Gyros inconsistent | World ổn định, model không chồng nhau, IMU scope đúng, RTF; không force ARM |
| Reference lỗi EGL/OpenGL | Chạy control; kiểm tra GPU/software rendering theo WSL_GPU.md |
| Pose chậm | Đo sim_time/real_time, tải CPU/render; không tăng speedup để che lỗi |
| Container unhealthy sau từng ready | Pose/state quá cũ hoặc disconnect; health.json là live status, ready.json chỉ là lần ready gần nhất |
| Run ID đã tồn tại | Chọn run mới hoặc resume rõ ràng; tránh ghi đè eeprom/log |

## Rosbag

Trong shell container, đặt output mới bên trong run:

```bash
ros2 bag record -s mcap -o /workspace/runs/latest/telemetry \
  /uav1/mavros/state /uav1/mavros/local_position/pose \
  /uav2/mavros/state /uav2/mavros/local_position/pose \
  /uav3/mavros/state /uav3/mavros/local_position/pose /solo/readiness
```

Kết thúc recorder bằng Ctrl+C trước stop. Pose sensor QoS cần đối chiếu nếu bag không nhận; dùng QoS override khi cần. Đừng ghi video raw lên Pi trước khi đo storage/CPU.
