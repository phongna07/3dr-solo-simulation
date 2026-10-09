# Windows 11 / WSL2 / GUI và GPU

Repo ở `/home/<user>/...` của Ubuntu WSL2. VS Code Windows + extension WSL có thể mở bằng `code .`. Docker Desktop dùng Linux containers và bật integration cho distro. Không chạy Compose từ PowerShell với repo Linux này.

## Không GUI

```bash
./scripts/dev sim --profile control
```

Control bỏ camera/gimbal và rendering sensors; đây là đường kiểm tra cơ bản không phụ thuộc GUI/GPU. Gazebo server vẫn có physics/IMU/NavSat. Có thể mở GUI sau khi ready.

## WSLg

`scripts/dev` tự thêm `compose.wslg.yaml` khi có `/mnt/wslg` và `/tmp/.X11-unix`. Override map X11/WSLg sockets và DISPLAY vào container; không dùng `xhost +`.

```bash
./scripts/dev gui
```

Entrypoint sử dụng runtime directory writable của container. X11 là đường GUI cơ bản; Wayland socket có thể không được sử dụng bởi cấu hình Qt đó. WSLg mount không tự cấp GPU hay đảm bảo render acceleration. Các driver GPU thuộc Windows/WSL/Docker host, không được đóng gói hoàn toàn trong image.

Nếu GUI/reference không render được, thử fallback phần mềm:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./scripts/dev sim --profile reference --gui
```

Dừng phiên cũ trước khi chạy lệnh trên. Software rendering có thể rất chậm; benchmark RTF. Kiểm tra từ shell:

```bash
glxinfo -B
gz sim --versions
```

## Native Linux Intel/AMD

Nếu Docker host có `/dev/dri`, dùng:

```bash
export COMPOSE_FILE=compose.yaml:compose.gpu.yaml
export RENDER_GID=$(getent group render | cut -d: -f3)
export VIDEO_GID=$(getent group video | cut -d: -f3)
./scripts/dev sim
```

Để GUI trên native Linux, tạo override map X11 socket + DISPLAY và Xauthority tương ứng host. Không map `/mnt/wslg` nếu nó không tồn tại. `compose.gpu.yaml` chỉ cấp DRM, không cấu hình X11.

## NVIDIA

Chỉ dùng sau khi xác nhận NVIDIA Container Toolkit hoặc Docker Desktop GPU support hoạt động:

```bash
export COMPOSE_FILE=compose.yaml:compose.wslg.yaml:compose.nvidia.yaml
./scripts/dev sim --profile reference
```

NVIDIA override không thay thế việc cài host driver/toolkit. Không mặc định `/dev/dri` của Ubuntu WSL tồn tại trong Docker Desktop VM. Bản setup này chưa xác nhận hardware GPU acceleration; xem kết quả đo cụ thể ở VALIDATION.md.

## Networking

Simulation dùng bridge network và localhost discovery bên trong một container, tránh yêu cầu Docker Desktop host networking. Truy cập ROS qua `dev shell`. Khi test Pi ↔ laptop, localhost TCP trong container không dùng được từ Pi; phải tạo endpoint MAVLink UDP/TCP rõ ràng và routing/port mapping cho địa chỉ LAN. Xem HARDWARE.md.
