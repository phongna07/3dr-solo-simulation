# Dependencies và tái lập môi trường

## Đã cố định

- ROS base multi-platform index digest trong Dockerfile và `locks/versions.json`.
- uv image digest, Python build 3.11.16.
- ArduPilot tag/commit và submodule được checkout từ commit đó.
- Plugin Gazebo commit chính xác.
- MAVROS upstream 2.15.1 build từ commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e` vào `/opt/mavros_ws`. Entrypoint source overlay này trước project workspace.
- Python build dependencies trực tiếp và gián tiếp trong `ardupilot-py311.lock`, có SHA256.

`build-backend.lock` khóa setuptools/wheel dùng để build package nguồn như empy. Image bootstrap hai package này trước rồi cài lock chính với `--no-build-isolation`, tránh việc pip/uv tự tải một build backend ngoài lock. Nếu đổi setuptools/wheel trong `.in`, cập nhật cùng version/hash tương ứng trong backend lock.

Gazebo dùng Harmonic/gz-sim8. Patch version, dependency APT, dataset tải ngoài được **ghi nhận sau build**, chưa khóa bằng snapshot mirror. `dpkg-query` tạo `/opt/solo/apt-packages.lock`; GeographicLib geoid có checksum. Base image pin không làm kho APT bất biến.

Kho ROS tại lúc triển khai đã cung cấp MAVROS 2.16.0. Image cài bản APT để lấy dependencies, sau đó build/source overlay MAVROS 2.15.1, libmavconn, msgs và extras cùng commit. Vì vậy APT manifest có thể ghi 2.16.0 trong khi `ros2 pkg prefix mavros` trỏ `/opt/mavros_ws/install/mavros`. Kiểm tra runtime version từ `package.xml` của prefix đang dùng. Không source chỉ `/opt/ros/jazzy` rồi bỏ overlay này.

**Build lại Dockerfile ở ngày khác có thể nhận patch APT khác.** Quy trình chuẩn cho nhóm là build/kiểm tra một image rồi phân phối cùng digest. Không tuyên bố Dockerfile hiện tại là build bit-for-bit từ internet bất kỳ thời điểm nào.

## Xuất thông tin image hiện tại

```bash
./scripts/dev lock
```

Lệnh ghi manifests vào `artifacts/` và image inspect JSON. Mỗi run còn sao chép package lock, submodule lock và hashes source/config. Script `dev sim` ghi image ID thực tế tại `runs/<run-id>.image-id.json`. Local image ID không thay thế registry digest để pull từ máy khác.

## Phát hành cho nhóm

Ví dụ registry placeholder, thay bằng registry bạn có quyền sử dụng:

```bash
docker tag solo-simulation:gd0 registry.example.org/solo/simulation:gd0-v1
docker push registry.example.org/solo/simulation:gd0-v1
docker buildx imagetools inspect registry.example.org/solo/simulation:gd0-v1
```

Ghi digest được trả về vào `.env` của thành viên:

```dotenv
SIM_IMAGE=registry.example.org/solo/simulation@sha256:<actual-digest>
```

Sau đó `docker compose pull sim` và `./scripts/dev sim`; không build image riêng để chạy cùng baseline. Không publish registry tự động bởi script. Các update dependency được thực hiện thành release môi trường mới, chạy lại nghiệm thu.

Không có registry, dùng `docker save`/`docker load` và kiểm tra SHA256 archive:

```bash
docker save solo-simulation:gd0 -o solo-simulation-gd0.tar
sha256sum solo-simulation-gd0.tar
# Máy khác:
docker load -i solo-simulation-gd0.tar
```

Archive image lớn: giữ ngoài repo. Chia sẻ source bằng Git, không commit build/install/eeprom/rosbag.

## Cần build tái lập lâu dài từ source

Lưu toàn bộ `.deb` hoặc mirror/snapshot của Ubuntu, ROS và OSRF; giữ Python artifacts tương ứng hashes, Python runtime download và GeographicLib dataset. Dùng manifest APT đầy đủ trong đúng kiến trúc, không chỉ package cấp cao. Giữ compiler/build flags nếu cần so sánh binary. Có thể bổ sung kiểm tra SHA256 binary/asset cho release.

Không lấy Debian lock amd64 để áp thẳng cho arm64: version/build hoặc package suffix có thể khác. Phát hành image companion ARM64 riêng, ghi manifest riêng; nếu dùng multi-platform registry index, ghi thêm platform đã chạy.

## Thay Python build dependency

Chỉnh `.in`, regenerate bằng uv đã pin, review lock rồi rebuild:

```bash
UV_CACHE_DIR=/tmp/solo-uv-cache uv pip compile locks/ardupilot-py311.in \
  --python-version 3.11 --generate-hashes --output-file locks/ardupilot-py311.lock
```

ROS runtime sử dụng Python hệ thống 3.12 + packages APT, không cài bộ dependency Waf vào ROS. Khi thêm MPC, tạo lock Python 3.12 riêng tương thích arm64 và tránh thay package ROS nền bằng pip.
