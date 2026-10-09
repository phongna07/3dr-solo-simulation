# GĐ0 – SITL: ArduCopter 4.5.3 + ROS 2 Jazzy + MAVROS + Gazebo Harmonic cho 3 UAV

Ngày tổng hợp: **09/10/2026**. Môi trường: **Ubuntu 24.04 Noble, shell zsh**.

Tài liệu này tổng hợp quy trình cài đặt, các lỗi đã gặp và cách chạy cuối cùng sau khi debug. Các khối lệnh vận hành dùng phiên bản đã sửa lỗi; không dùng lại launcher đa UAV bị lỗi đường dẫn. Có thể đọc tài liệu độc lập, không cần lấy script từ `/tmp` của cuộc hội thoại.

**Phạm vi đã xác nhận:** build ArduCopter 4.5.3 và plugin Gazebo; một UAV cất cánh và điều khiển qua Gazebo theo phản hồi của người dùng; ba UAV kết nối MAVROS và xuất pose qua kiểm tra trực tiếp. Các lệnh bay ba UAV được hướng dẫn bên dưới, nhưng việc ba UAV bám quỹ đạo MPC chưa được thực hiện/xác nhận. Controller của repository chưa được port thành node ROS trong công việc này.

Tài liệu chỉ thêm hướng dẫn. Không sửa source của MPC, ArduPilot hay plugin Gazebo. Các script trong tài liệu, khi người dùng chạy, tạo cấu hình riêng tại `~/sitl_runs/`.

## Mục lục

1. [Mục tiêu GĐ0 và tính khả thi](#1-mục-tiêu-gđ0-và-tính-khả-thi)
2. [Version và cách chia môi trường](#2-version-và-cách-chia-môi-trường)
3. [Cài môi trường hệ thống và ROS 2](#3-cài-môi-trường-hệ-thống-và-ros-2)
4. [Cài và build ArduCopter 4.5.3](#4-cài-và-build-arducopter-453)
5. [Cài Gazebo và build ardupilot_gazebo](#5-cài-gazebo-và-build-ardupilot_gazebo)
6. [Kiểm tra một UAV](#6-kiểm-tra-một-uav)
7. [Tạo world ba UAV](#7-tạo-world-ba-uav)
8. [Khởi chạy ba UAV bằng nhiều terminal](#8-khởi-chạy-ba-uav-bằng-nhiều-terminal)
9. [Telemetry, EKF và pose](#9-telemetry-ekf-và-pose)
10. [GUIDED, ARM, takeoff và điều khiển](#10-guided-arm-takeoff-và-điều-khiển)
11. [Hạ cánh, dừng và khởi động lại](#11-hạ-cánh-dừng-và-khởi-động-lại)
12. [Các lỗi đã gặp và cách xử lý](#12-các-lỗi-đã-gặp-và-cách-xử-lý)
13. [Hiệu năng và thời gian mô phỏng](#13-hiệu-năng-và-thời-gian-mô-phỏng)
14. [Hướng nối thuật toán MPC](#14-hướng-nối-thuật-toán-mpc)
15. [Tiêu chí hoàn thành và tài liệu tham khảo](#15-tiêu-chí-hoàn-thành-và-tài-liệu-tham-khảo)

## 1. Mục tiêu GĐ0 và tính khả thi

Theo `plan_3dr.xlsx`, GĐ0 gồm:

| Mục | Nội dung | Điều kiện nghiệm thu |
|---|---|---|
| 0.1 | ArduCopter SITL + ROS 2 Jazzy + MAVROS cấu hình APM | MAVROS kết nối, state có `connected: true` |
| 0.2 | Port MPC/CasADi, planner, safe corridor và occupancy grid thành node `rclpy`; chốt ENU chung | Node nhận pose và xuất setpoint |
| 0.3 | Setpoint GUIDED: pos + vel + acc bước đầu dự báo, ít nhất 10 Hz; yaw từ vector vận tốc | SITL bám quỹ đạo MPC |
| 0.4 | Controller trên Pi, SITL trên laptop; benchmark solver | Thời gian giải nhỏ hơn chu kỳ, ví dụ 100 ms |
| 0.5 | Chốt FOV | Có kích thước footprint, độ cao bay và chiều cao vật cản |

**Đánh giá:** chạy ba FCU và MAVROS trong Gazebo là khả thi và đã xác nhận được kết nối/pose. Phần port MPC cũng khả thi, nhưng cần adapter trạng thái, cảm biến, hệ tọa độ và bộ phát setpoint riêng. Không thể chỉ chạy `main.py` rồi tự động điều khiển UAV Gazebo: vòng lặp hiện tại vẫn mô phỏng động học và LiDAR trong Python.

Theo tài liệu Solo đính kèm, hướng này cần giữ **ArduCopter 4.5.3** để nhất quán với cấu hình mục tiêu. Iris trong Gazebo là model dùng để kiểm thử luồng điều khiển; chưa phải model động lực học được hiệu chỉnh theo 3DR Solo.

Luồng dữ liệu:

```mermaid
flowchart LR
    R[Controller ROS 2 / lệnh CLI] <-->|Topic và service| M[MAVROS riêng cho từng UAV]
    M <-->|MAVLink qua TCP| A[ArduCopter SITL riêng cho từng UAV]
    A <-->|Servo và cảm biến JSON qua UDP| G[Gazebo: 3 model trong 1 world]
```

Giai đoạn kiểm tra CLI không cần `ros_gz_bridge`. Sau này cần bridge nếu muốn đưa camera, LiDAR hoặc `/clock` từ Gazebo sang ROS; MAVROS đã cung cấp trạng thái FCU qua MAVLink. Plugin ArduPilot–Gazebo tự kết nối với SITL qua JSON, độc lập với ROS. [Tài liệu ArduPilot–Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html).

## 2. Version và cách chia môi trường

### 2.1. Version đã kiểm tra trên máy

| Thành phần | Version / cấu hình |
|---|---|
| Ubuntu | 24.04 Noble |
| ROS 2 | Jazzy |
| Python hệ thống | `/usr/bin/python3`, 3.12.3 tại thời điểm kiểm tra |
| Python cho Waf/SITL launcher cũ | 3.11; phiên cài đặt đã dùng 3.11.16 |
| ArduPilot | tag `Copter-4.5.3` |
| Commit ArduPilot | `ee14ff3822161368ec403700bf96d02b84def06c` |
| MAVROS | 2.15.1, gói `ros-jazzy-mavros` |
| Gazebo | Harmonic, họ thư viện `gz-sim8`; bản đã kiểm tra 8.15.0 |
| CMake | 3.28.3, hỗ trợ `--fresh` |
| Commit plugin Gazebo đã build | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` |

Version vá của các gói APT có thể thay đổi theo kho. Lưu lại phiên bản thực tế và commit khi làm thí nghiệm; không coi branch `main/master` là một phiên bản cố định. Dòng `Built-in MAVLink package version: 2026.8.8` là version thư viện MAVLink phía MAVROS, không phải version firmware ArduCopter.

### 2.2. Ba loại môi trường

| Công việc | Môi trường dùng |
|---|---|
| Build ArduPilot bằng Waf, chạy `sim_vehicle.py` cũ khi cần | `~/venvs/ardupilot453-py311` |
| ROS 2, MAVROS, configure/build plugin Gazebo với ament | Python hệ thống 3.12; source `/opt/ros/jazzy/setup.zsh` |
| MPC offline hiện có | Venv Python 3.12 riêng cho repository |
| Chạy binary `arducopter` đã build | Không cần kích hoạt venv Python |

Python 3.12 đã bỏ module `imp`; Waf được pin trong ArduPilot 4.5.3 còn dùng module này. Vì vậy build bằng Python 3.11. Ngược lại, các gói Python ROS Jazzy trên Noble được cài cho Python hệ thống 3.12. [Thông báo của Python](https://docs.python.org/3.12/library/imp.html).

Mở terminal mới cho từng môi trường. Nếu terminal đang có venv, thoát trước khi dùng ROS:

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi

source /opt/ros/jazzy/setup.zsh
```

Trong các terminal ROS ở phần vận hành, chạy đoạn trên thay vì kích hoạt venv 3.11. Tất cả terminal ROS phải dùng cùng `ROS_DOMAIN_ID` và cấu hình DDS. Quy trình này dùng mặc định nếu bạn chưa tùy biến.

### 2.3. Cấu trúc thư mục

```text
~/motion_planning_MPC/             repository thuật toán
~/ardupilot_453/                   ArduPilot tag Copter-4.5.3
~/ardupilot_gazebo/                source và build plugin
~/venvs/ardupilot453-py311/         Python 3.11 cho Waf
~/venvs/mpc-py312/                 tùy chọn: MPC offline
~/sitl_runs/one_uav/               SITL không Gazebo
~/sitl_runs/gazebo_uav1/           SITL một UAV với Gazebo
~/sitl_runs/gazebo_3uav/
    create_world.py
    iris_3uav.sdf
    sitl/0/                       trạng thái, eeprom và log UAV 1
    sitl/1/                       trạng thái, eeprom và log UAV 2
    sitl/2/                       trạng thái, eeprom và log UAV 3
```

Đường dẫn dùng `$HOME`, không cần thay username. Không dùng chung thư mục trạng thái giữa các UAV hoặc giữa model quad nội bộ và model Gazebo.

## 3. Cài môi trường hệ thống và ROS 2

Phần cài đặt chỉ cần làm một lần. Nếu máy đã cài đúng môi trường, dùng lệnh kiểm tra rồi chuyển tới phần vận hành; không cần cài lại.

### 3.1. Kiểm tra Ubuntu và cài công cụ

```zsh
lsb_release -a
/usr/bin/python3 --version

sudo apt update
sudo apt install -y \
  build-essential ccache gawk git make cmake pkg-config \
  curl wget gnupg lsb-release software-properties-common locales \
  python3-dev python3-venv python3-pip \
  libtool libtool-bin libxml2-dev libxslt1-dev \
  libsfml-dev libcsfml-dev xterm screen
```

Danh sách này dành cho SITL trên máy Ubuntu; không yêu cầu compiler ARM để build firmware thật.

Kiểm tra locale:

```zsh
locale
```

Nếu chưa có UTF-8:

```zsh
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
```

### 3.2. Thêm kho ROS 2 trên máy mới

Bỏ qua bước thêm kho nếu ROS Jazzy đã được cài và `apt update` đang đọc kho ROS 2 bình thường. Cách dưới đây dùng gói `ros2-apt-source` của ROS, theo [hướng dẫn kho APT chính thức](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).

```zsh
sudo add-apt-repository -y universe

MPC_ROS_APT_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | /usr/bin/python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')

curl -fL \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${MPC_ROS_APT_VERSION}/ros2-apt-source_${MPC_ROS_APT_VERSION}.noble_all.deb" \
  -o /tmp/ros2-apt-source.deb

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

### 3.3. Cài ROS Jazzy và MAVROS

```zsh
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
  geographiclib-tools python3-catkin-pkg-modules

source /opt/ros/jazzy/setup.zsh

sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

Kiểm tra:

```zsh
ros2 pkg prefix mavros
ros2 interface show mavros_msgs/srv/MessageInterval

/usr/bin/python3 -c \
  "import rclpy; from catkin_pkg.package import parse_package_string; print('ROS Python OK')"

ls /usr/share/GeographicLib/geoids/egm96-5.pgm
```

Nếu dataset nằm trong `/usr/local/share/GeographicLib`, kiểm tra thư mục đó thay cho `/usr/share`. Dataset geoid thiếu có thể làm MAVROS không khởi động được. [README MAVROS](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md).

Jazzy dùng Ubuntu Noble. Với zsh, source file `.zsh`; với bash, thay bằng `.bash`. [Hướng dẫn cài ROS Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst).

## 4. Cài và build ArduCopter 4.5.3

### 4.1. Checkout riêng đúng tag

Nếu chưa có checkout:

```zsh
git clone \
  --branch Copter-4.5.3 \
  --recurse-submodules \
  https://github.com/ArduPilot/ardupilot.git \
  ~/ardupilot_453
```

Nếu `~/ardupilot_453` đã tồn tại, không clone đè. Kiểm tra:

```zsh
git -C ~/ardupilot_453 describe --tags --exact-match
git -C ~/ardupilot_453 rev-parse HEAD
git -C ~/ardupilot_453 submodule update --init --recursive
```

Tag mong đợi là `Copter-4.5.3`. Không dùng nhầm checkout `~/ardupilot` chạy firmware dev khác version.

### 4.2. Python 3.11 qua uv

Nếu chưa có uv:

```zsh
curl -LsSf https://astral.sh/uv/install.sh -o /tmp/install-uv.sh
sh /tmp/install-uv.sh
```

Sau đó dùng đường dẫn uv trực tiếp để không phụ thuộc việc shell đã cập nhật PATH:

```zsh
~/.local/bin/uv python install 3.11

mkdir -p ~/venvs

~/.local/bin/uv venv \
  --python 3.11 \
  --seed \
  ~/venvs/ardupilot453-py311

source ~/venvs/ardupilot453-py311/bin/activate

python -m pip install --upgrade pip wheel "setuptools<81"

python -m pip install \
  future lxml pymavlink pyserial pexpect \
  "empy==3.3.4" ptyprocess dronecan \
  "numpy<2" pyparsing psutil PyYAML intelhex

python --version
python -c "import imp, pymavlink, em; print('ArduPilot Python OK')"
python -m pip check
```

Nếu venv đã tồn tại và dùng được, chỉ activate và kiểm tra; không tạo đè. `setuptools<81` giữ khả năng tương thích với các dependency cũ dùng `pkg_resources`; `empy==3.3.4` là pin của toolchain cũ. Không cài dependency ArduPilot vào Python hệ thống bằng `sudo pip`.

Lệnh kiểm tra `import imp` trên 3.11 có thể có cảnh báo deprecated; đó không phải lỗi import. [Cài uv](https://docs.astral.sh/uv/getting-started/installation/), [quản lý Python bằng uv](https://docs.astral.sh/uv/guides/install-python/).

MAVProxy không bắt buộc trong quy trình này: dùng MAVROS trực tiếp qua TCP và binary SITL trực tiếp. Venv trên cũng đủ cho launcher `sim_vehicle.py` nếu dùng `--no-mavproxy`.

### 4.3. Build bằng đúng interpreter

```zsh
source ~/venvs/ardupilot453-py311/bin/activate
cd ~/ardupilot_453

python ./waf configure --board sitl
python ./waf copter -j4

ls -lh ~/ardupilot_453/build/sitl/bin/arducopter
```

Thành công khi thấy `build finished successfully` và có binary `arducopter`.

### 4.4. Cách đã xử lý hai lỗi môi trường

- **`Unable to locate package python-argparse`:** installer trong tag 4.5.3 chưa xử lý đầy đủ Noble, thêm package Python 2 cũ. `argparse` đã có trong Python 3. Quy trình này dùng APT thủ công ở mục 3.1 và venv ở mục 4.2, không chạy lại installer lỗi.
- **`ModuleNotFoundError: No module named 'imp'`:** Waf chạy nhầm Python 3.12. Dùng `python ./waf` sau khi activate venv 3.11. Không cần sửa source Waf.

## 5. Cài Gazebo và build ardupilot_gazebo

### 5.1. Gazebo Harmonic

Nếu kho OSRF đã có trên máy, bỏ qua đoạn thêm kho và chỉ kiểm tra/cài package cần thiết.

```zsh
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /tmp/pkgs-osrf-archive-keyring.gpg

sudo install -m 0644 /tmp/pkgs-osrf-archive-keyring.gpg \
  /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

sudo apt install -y \
  gz-harmonic libgz-sim8-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

Kiểm tra Gazebo, rồi đóng world thử trước khi chạy world UAV:

```zsh
gz sim --versions
gz sim -v4 -r shapes.sdf
```

Harmonic hỗ trợ Ubuntu 24.04 và dùng họ `gz-sim8`. [Hướng dẫn cài Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/).

### 5.2. Plugin official và commit tái lập

Chỉ chạy block clone này khi thư mục chưa có. Nó pin về commit của plugin đã build thành công trong phiên làm việc:

```zsh
git clone https://github.com/ArduPilot/ardupilot_gazebo.git \
  ~/ardupilot_gazebo

git -C ~/ardupilot_gazebo checkout \
  082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
```

Nếu đã có plugin đang chạy tốt, giữ checkout đó và ghi lại commit:

```zsh
git -C ~/ardupilot_gazebo rev-parse HEAD
git -C ~/ardupilot_gazebo status --short
```

### 5.3. Configure bằng Python hệ thống

Mở terminal mới cho ROS/Gazebo hoặc thoát venv 3.11:

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi

source /opt/ros/jazzy/setup.zsh
export GZ_VERSION=harmonic

/usr/bin/python3 -c \
  "from catkin_pkg.package import parse_package_string; print('catkin_pkg OK')"

cmake --fresh \
  -S ~/ardupilot_gazebo \
  -B ~/ardupilot_gazebo/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/usr/bin/python3

cmake --build ~/ardupilot_gazebo/build -j4

ls -lh ~/ardupilot_gazebo/build/libArduPilotPlugin.so
```

`--fresh` làm mới cache configure trong thư mục build; không xóa source. `Python3_EXECUTABLE` ép CMake dùng interpreter đúng. [Tài liệu CMake](https://cmake.org/cmake/help/latest/module/FindPython3.html).

Log mong đợi:

```text
Found Python3: /usr/bin/python3 (found version "3.12.x")
Compiling against Gazebo Harmonic
Configuring done
Generating done
```

Lỗi đã gặp `ModuleNotFoundError: No module named 'catkin_pkg'` xuất hiện vì CMake chọn `/home/phong/.local/bin/python3.11` khi build dưới ament. Package `catkin_pkg` đã có cho hệ thống 3.12, nên giải pháp là chọn interpreter và làm mới cache, không cài thêm ROS dependency vào venv 3.11.

### 5.4. Biến môi trường runtime

Trong **mỗi terminal chạy Gazebo**, đặt:

```zsh
export GZ_VERSION=harmonic

export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}"
```

Quy trình dùng plugin từ thư mục build, không cần `sudo make install` và không cần source một colcon workspace cho plugin này. [README plugin official](https://github.com/ArduPilot/ardupilot_gazebo/blob/082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5/README.md).

## 6. Kiểm tra một UAV

Phần này là checkpoint trước khi lên ba UAV. Không chạy đồng thời với phần 8 vì trùng instance 0/cổng 5760.

### 6.1. Tùy chọn: SITL không Gazebo

Terminal SITL:

```zsh
mkdir -p ~/sitl_runs/one_uav
cd ~/sitl_runs/one_uav

~/ardupilot_453/build/sitl/bin/arducopter \
  -S --model + --speedup 1 \
  -I 0 --sysid 1 \
  --defaults "$HOME/ardupilot_453/Tools/autotest/default_params/copter.parm"
```

Đây là physics nội bộ SITL; chưa có UAV trong Gazebo. `Waiting for connection` có thể xuất hiện cho tới khi MAVROS mở TCP.

Terminal MAVROS:

```zsh
source /opt/ros/jazzy/setup.zsh

ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5760 \
  namespace:=uav1 \
  tgt_system:=1 \
  tgt_component:=1
```

Kiểm tra state/telemetry như mục 9, chỉ dùng `uav1`. Sau checkpoint, dừng SITL/MAVROS trước khi chuyển sang Gazebo.

### 6.2. Một UAV trong Gazebo

Terminal Gazebo: đặt biến môi trường ở mục 5.4, rồi chạy:

```zsh
gz sim -v4 -r iris_runway.sdf
```

Terminal SITL:

```zsh
mkdir -p ~/sitl_runs/gazebo_uav1
cd ~/sitl_runs/gazebo_uav1

~/ardupilot_453/build/sitl/bin/arducopter \
  -S --model JSON --speedup 1 \
  --sim-address=127.0.0.1 \
  -I 0 --sysid 1 \
  --defaults "$HOME/ardupilot_453/Tools/autotest/default_params/copter.parm,$HOME/ardupilot_453/Tools/autotest/default_params/gazebo-iris.parm"
```

Terminal MAVROS dùng lệnh của mục 6.1. Bật telemetry, chờ EKF có pose, chuyển GUIDED, ARM và takeoff theo mục 9–10 với `uav1`.

Khác biệt cần giữ: `--model JSON` nhận physics từ Gazebo và phải nạp thêm `gazebo-iris.parm` cho frame Iris X. Model `+` của checkpoint không thay thế cho JSON khi muốn nhìn UAV được điều khiển trong Gazebo.

## 7. Tạo world ba UAV

### 7.1. Port và vị trí ban đầu

| UAV | Model | Instance | SYSID | UDP plugin | TCP MAVROS | Namespace | Pose spawn Gazebo |
|---|---|---:|---:|---:|---:|---|---|
| 1 | `uav1` | 0 | 1 | 9002 | 5760 | `/uav1` | `(0, 0, 0.195)` m |
| 2 | `uav2` | 1 | 2 | 9012 | 5770 | `/uav2` | `(6, 0, 0.195)` m |
| 3 | `uav3` | 2 | 3 | 9022 | 5780 | `/uav3` | `(12, 0, 0.195)` m |

ArduPilot 4.5.3 cộng `10 × instance` vào cổng: TCP cơ sở 5760, cổng JSON gửi servo cơ sở 9002. Tăng namespace ROS không tự thay cổng Gazebo hay SYSID.

### 7.2. Script đầy đủ, không phụ thuộc file tạm

Chạy block sau **một lần** để ghi script vào thư mục cấu hình riêng. Khi sửa world, phải dừng Gazebo rồi khởi chạy lại để nạp file mới.

```zsh
mkdir -p ~/sitl_runs/gazebo_3uav

cat > ~/sitl_runs/gazebo_3uav/create_world.py <<'PY'
#!/usr/bin/env python3
"""Tạo world 3 UAV riêng từ model/world official đã cài."""
import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    '--output', type=Path,
    default=Path.home() / 'sitl_runs/gazebo_3uav',
)
args = parser.parse_args()
source = Path.home() / 'ardupilot_gazebo'
tree = ET.parse(source / 'worlds/iris_runway.sdf')
world = tree.getroot().find('world')
template = ET.parse(
    source / 'models/iris_with_gimbal/model.sdf'
).getroot().find('model')
if world is None or template is None:
    raise RuntimeError('Không tìm thấy world hoặc model Iris')

old_uavs = [
    element for element in world.findall('include')
    if element.findtext('uri') == 'model://iris_with_gimbal'
]
if len(old_uavs) != 1:
    raise RuntimeError('World gốc phải có đúng một include Iris')
world.remove(old_uavs[0])
world.set('name', 'iris_3uav')

for index in range(3):
    model = copy.deepcopy(template)
    model.set('name', f'uav{index + 1}')
    pose = model.find('pose')
    if pose is None:
        pose = ET.SubElement(model, 'pose')
    pose.set('degrees', 'true')
    pose.text = f'{6 * index} 0 0.195 0 0 90'

    plugin = model.find("plugin[@name='ArduPilotPlugin']")
    if plugin is None:
        raise RuntimeError('Không tìm thấy ArduPilotPlugin')
    port = plugin.find('fdm_port_in')
    if port is None:
        raise RuntimeError('Không tìm thấy fdm_port_in')
    port.text = str(9002 + 10 * index)
    world.append(model)

args.output.mkdir(parents=True, exist_ok=True)
destination = args.output / 'iris_3uav.sdf'
ET.indent(tree, space='  ')
tree.write(destination, encoding='utf-8', xml_declaration=True)
print(f'Created {destination}')
for index in range(3):
    print(
        f'uav{index + 1}: x={6 * index} m, '
        f'UDP={9002 + 10 * index}, '
        f'instance={index}, TCP={5760 + 10 * index}, SYSID={index + 1}'
    )
PY

/usr/bin/python3 ~/sitl_runs/gazebo_3uav/create_world.py
```

Script giữ tên nested model `iris_with_standoffs`, nên các đường dẫn IMU/joint trong plugin vẫn đúng và được resolve trong phạm vi từng model `uav1/uav2/uav3`. Giữ cấu hình lockstep, hướng trục và gimbal từ model đã chạy một UAV.

Đây là cấu hình ba UAV cho điều khiển; chưa tách topic điều khiển gimbal hay cổng video cho ba camera. Chưa bật streaming camera trong các lệnh ở tài liệu này.

### 7.3. Kiểm tra SDF

```zsh
export SDF_PATH="$HOME/ardupilot_gazebo/models:${SDF_PATH}"

gz sdf -k ~/sitl_runs/gazebo_3uav/iris_3uav.sdf
```

Kết quả cần có `Valid.`. `gz sdf` dùng `SDF_PATH` để resolve `model://`; chỉ đặt `GZ_SIM_RESOURCE_PATH` chưa đủ cho công cụ kiểm tra này. Có thể thấy warning extension `gz_frame_id` không nằm trong spec SDF; với model/commit này, warning đó vẫn đi cùng kết quả hợp lệ.

## 8. Khởi chạy ba UAV bằng nhiều terminal

### 8.1. Bố trí chín terminal

| Terminal | Vai trò | Giữ chạy? |
|---|---|---|
| T1 | Gazebo với world ba UAV | Có |
| T2 | SITL UAV 1 | Có |
| T3 | SITL UAV 2 | Có |
| T4 | SITL UAV 3 | Có |
| T5 | MAVROS UAV 1 | Có |
| T6 | MAVROS UAV 2 | Có |
| T7 | MAVROS UAV 3 | Có |
| T8 | Telemetry, mode, ARM, takeoff và setpoint | Dùng tương tác |
| T9 | Theo dõi state/pose/rate | Dùng tương tác |

Đóng phiên một UAV hoặc phiên launcher cũ trước khi bắt đầu. Không để tiến trình cũ chiếm cùng cổng. Lệnh kiểm tra chỉ đọc:

```zsh
ps -C arducopter -o pid,stat,args
ss -ltnp '( sport = :5760 or sport = :5770 or sport = :5780 )'
```

### 8.2. T1 – Gazebo

```zsh
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}"

gz sim -v4 -r ~/sitl_runs/gazebo_3uav/iris_3uav.sdf
```

Phải thấy ba UAV đứng cách nhau. `-r` cho world chạy ngay; không pause trong lúc chờ boot, telemetry hay bay.

### 8.3. T2 – SITL UAV 1

```zsh
mkdir -p ~/sitl_runs/gazebo_3uav/sitl/0
cd ~/sitl_runs/gazebo_3uav/sitl/0

~/ardupilot_453/build/sitl/bin/arducopter \
  -S \
  --model JSON \
  --speedup 1 \
  --sim-address=127.0.0.1 \
  -I 0 \
  --sysid 1 \
  --defaults "$HOME/ardupilot_453/Tools/autotest/default_params/copter.parm,$HOME/ardupilot_453/Tools/autotest/default_params/gazebo-iris.parm"
```

### 8.4. T3 – SITL UAV 2

```zsh
mkdir -p ~/sitl_runs/gazebo_3uav/sitl/1
cd ~/sitl_runs/gazebo_3uav/sitl/1

~/ardupilot_453/build/sitl/bin/arducopter \
  -S \
  --model JSON \
  --speedup 1 \
  --sim-address=127.0.0.1 \
  -I 1 \
  --sysid 2 \
  --defaults "$HOME/ardupilot_453/Tools/autotest/default_params/copter.parm,$HOME/ardupilot_453/Tools/autotest/default_params/gazebo-iris.parm"
```

### 8.5. T4 – SITL UAV 3

```zsh
mkdir -p ~/sitl_runs/gazebo_3uav/sitl/2
cd ~/sitl_runs/gazebo_3uav/sitl/2

~/ardupilot_453/build/sitl/bin/arducopter \
  -S \
  --model JSON \
  --speedup 1 \
  --sim-address=127.0.0.1 \
  -I 2 \
  --sysid 3 \
  --defaults "$HOME/ardupilot_453/Tools/autotest/default_params/copter.parm,$HOME/ardupilot_453/Tools/autotest/default_params/gazebo-iris.parm"
```

Không cần activate Python 3.11 cho T2–T4: đây là binary C++. Nếu thấy `Waiting for connection`, mở MAVROS ở các bước kế tiếp; không chờ vô hạn tại đây rồi mới khởi chạy MAVROS.

### 8.6. T5 – MAVROS UAV 1

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi
source /opt/ros/jazzy/setup.zsh

ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5760 \
  namespace:=uav1 \
  tgt_system:=1 \
  tgt_component:=1
```

### 8.7. T6 – MAVROS UAV 2

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi
source /opt/ros/jazzy/setup.zsh

ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5770 \
  namespace:=uav2 \
  tgt_system:=2 \
  tgt_component:=1
```

### 8.8. T7 – MAVROS UAV 3

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi
source /opt/ros/jazzy/setup.zsh

ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5780 \
  namespace:=uav3 \
  tgt_system:=3 \
  tgt_component:=1
```

Mỗi SITL phải nạp được hai file defaults bằng đường dẫn tuyệt đối. Mỗi MAVROS cần có:

```text
CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

MAVROS tự tạo UAS prefix theo `tgt_system`: `/uas1`, `/uas2`, `/uas3`. Giữ SYSID và target system khớp bảng cổng; tránh ba MAVROS cùng target system 1. [Mã nguồn mavros_node](https://github.com/mavlink/mavros/blob/ros2/mavros/src/mavros_node.cpp).

## 9. Telemetry, EKF và pose

### 9.1. T8 – Kiểm tra state

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi
source /opt/ros/jazzy/setup.zsh

for uav_id in 1 2 3; do
  echo "State of UAV ${uav_id}"
  timeout 30s ros2 topic echo \
    "/uav${uav_id}/mavros/state" --once
done
```

Mỗi UAV cần có `connected: true`. `armed: false`, `mode: STABILIZE` là trạng thái bình thường trước khi thử bay. `ros2 topic list` có topic chưa chứng minh topic đã có dữ liệu cảm biến.

Nếu chưa có heartbeat, dừng ở đây và dùng mục 12.4; chưa gửi ARM.

### 9.2. Yêu cầu hai message cần thiết

```zsh
for uav_id in 1 2 3; do
  ros2 service call \
    "/uav${uav_id}/mavros/set_message_interval" \
    mavros_msgs/srv/MessageInterval \
    "{message_id: 32, message_rate: 10.0}"

  ros2 service call \
    "/uav${uav_id}/mavros/set_message_interval" \
    mavros_msgs/srv/MessageInterval \
    "{message_id: 30, message_rate: 10.0}"
done
```

| MAVLink ID | Message | Mục đích |
|---:|---|---|
| 32 | `LOCAL_POSITION_NED` | Vị trí/vận tốc local; MAVROS chuyển sang ENU |
| 30 | `ATTITUDE` | Attitude cho IMU/orientation |

Cả sáu service cần trả `success: true`. Lặp lại sau khi restart SITL. Không có MAVProxy yêu cầu stream mặc định; trong một phiên debug, `SR0_POSITION` và `SR0_EXTRA1` đều bằng 0.

`success: true` xác nhận FCU chấp nhận cấu hình gửi; chưa bảo đảm message 32 đã có dữ liệu hợp lệ. [Yêu cầu dữ liệu MAVLink](https://ardupilot.org/dev/docs/mavlink-requesting-data.html).

### 9.3. Chờ EKF theo trạng thái, không chỉ theo số giây

Trong T5–T7, các log thường xuất hiện theo tiến trình:

```text
EKF3 IMU0 initialised
EKF3 IMU0 tilt alignment complete
EKF3 IMU0 MAG0 initial yaw alignment complete
GPS 1: detected as u-blox ...
EKF3 IMU0 origin set
EKF3 IMU0 is using GPS
```

`origin set` hoặc GPS được phát hiện chưa bảo đảm đã có vị trí/vận tốc local để phát pose. Tiêu chí thực tế là **nhận pose đang cập nhật**.

Trong lần chạy ba UAV, Gazebo chỉ đạt trung bình khoảng `0.13×`: 58,7 giây mô phỏng mất khoảng 448 giây thực. EKF của các UAV mất gần năm phút thực từ lúc có heartbeat tới lúc dùng GPS. Vì vậy không dùng mốc 30–60 giây thực như một cam kết cho mọi máy.

### 9.4. T9 – Kiểm tra pose có thời gian chờ đủ

```zsh
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi
source /opt/ros/jazzy/setup.zsh

for uav_id in 1 2 3; do
  echo "Pose of UAV ${uav_id}"
  if timeout 120s ros2 topic echo \
    "/uav${uav_id}/mavros/local_position/pose" \
    --qos-reliability best_effort \
    --once; then
    echo "UAV ${uav_id}: received pose"
  else
    echo "UAV ${uav_id}: chưa nhận pose; kiểm tra telemetry, EKF và tốc độ Gazebo"
  fi
done
```

Nếu hết hạn trong lúc EKF vẫn khởi tạo, đọc log rồi chạy lại sau khi có `is using GPS`; không kết luận mất kết nối chỉ từ timeout.

Xem liên tục một UAV:

```zsh
ros2 topic echo /uav1/mavros/local_position/pose \
  --qos-reliability best_effort
```

Nhấn `Ctrl+C` để kết thúc riêng `echo`. Đổi namespace để xem UAV khác.

Kiểm tra publisher/QoS và tần số:

```zsh
ros2 topic info /uav1/mavros/local_position/pose --verbose
timeout 30s ros2 topic hz /uav1/mavros/local_position/pose
```

`timeout` thường trả mã 124 khi hết hạn; với lệnh `hz`, đó là chủ động kết thúc phép đo, không phải lỗi FCU. Trong lần đo cuối cuộc hội thoại, mỗi UAV có 14 pose trong 12 giây thực, khoảng 1,2 Hz thực, dù yêu cầu gửi là 10 Hz theo thời gian FCU/mô phỏng.

### 9.5. Vì sao service thành công nhưng pose vẫn im lặng?

Trong ArduPilot 4.5.3, `send_local_position()` chỉ gửi khi AHRS có **cả vị trí relative-to-origin và vận tốc NED**. Nếu một trong hai chưa có, hàm trả về mà không gửi message 32. Đây là nguyên nhân quan sát được sau khi telemetry đã được bật; chưa phải lỗi của `ros2 topic echo`.

Trong phiên trước đó còn có nguyên nhân khác: chưa bật message stream. Kiểm tra hai điều kiện độc lập: **đã yêu cầu stream** và **EKF đã có state hợp lệ**.

Pose gần `(0, 0, 0)` ở cả ba UAV là bình thường: từng EKF có gốc local riêng. Nó không phủ nhận khoảng cách spawn 6 m trong Gazebo. Các tên frame `map` giống nhau cũng chưa tạo ra một ENU chung; xem mục 14.3.

## 10. GUIDED, ARM, takeoff và điều khiển

Chỉ thực hiện sau khi cả ba UAV có heartbeat, pose cập nhật và EKF sẵn sàng. Nếu `PreArm: Gyros inconsistent` còn xuất hiện và ARM bị từ chối, xem mục 12.7 trước khi thử tiếp.

### 10.1. T8 – Chuyển GUIDED

```zsh
for uav_id in 1 2 3; do
  ros2 service call \
    "/uav${uav_id}/mavros/set_mode" \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'GUIDED'}"
done

for uav_id in 1 2 3; do
  echo "Mode of UAV ${uav_id}"
  timeout 30s ros2 topic echo \
    "/uav${uav_id}/mavros/state" --once
done
```

Phải thấy mode thực tế là `GUIDED` cho cả ba. `mode_sent: true` chỉ xác nhận yêu cầu đã gửi, không bảo đảm FCU đã chuyển mode. Trong ArduCopter dùng `GUIDED`; không chuyển sang mode `OFFBOARD` của PX4.

### 10.2. ARM và takeoff liền nhau, từng UAV

Để tránh tiếp tục takeoff khi ARM bị từ chối, dùng hàm kiểm tra response sau. Hàm không sửa source hay tạo node controller lâu dài; đây là tiện ích CLI trong T8.

```zsh
takeoff_uav() {
  local uav_id="$1"
  local arm_response
  local takeoff_response

  arm_response=$(timeout 30s ros2 service call \
    "/uav${uav_id}/mavros/cmd/arming" \
    mavros_msgs/srv/CommandBool \
    "{value: true}" 2>&1)
  local arm_exit=$?
  print -r -- "$arm_response"

  if [[ "$arm_exit" -ne 0 || "$arm_response" != *"success=True, result=0"* ]]; then
    echo "ARM UAV ${uav_id} chưa thành công; dừng và đọc log FCU."
    return 1
  fi

  takeoff_response=$(timeout 30s ros2 service call \
    "/uav${uav_id}/mavros/cmd/takeoff" \
    mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 3.0}" 2>&1)
  local takeoff_exit=$?
  print -r -- "$takeoff_response"

  if [[ "$takeoff_exit" -ne 0 || "$takeoff_response" != *"success=True, result=0"* ]]; then
    echo "Takeoff UAV ${uav_id} chưa thành công; kiểm tra mode và armed."
    return 1
  fi
}

for uav_id in 1 2 3; do
  takeoff_uav "$uav_id" || break
done
```

Hàm dùng `print -r` và cú pháp zsh, đúng shell của quy trình. Kiểm tra chuỗi response tương ứng `mavros_msgs` 2.15.1 đã thấy trong phiên này. `result=0` là MAVLink accepted; vẫn cần quan sát state và độ cao để xác nhận UAV thực sự bay.

Chờ cả ba UAV lên khoảng 3 m. Kiểm tra pose từng UAV ở T9, kiểm tra `armed: true` và `mode: GUIDED` bằng state. Không gửi setpoint di chuyển trước khi takeoff hoàn tất.

**ARM thành công rồi tự disarm:** log `Arming motors` rồi `Disarming motors` khoảng 10 giây sau có thể là tự disarm khi UAV vẫn trên mặt đất và throttle thấp. Trong bản 4.5.3, mặc định delay này là 10 giây theo thời gian FCU. Vì vậy ARM và takeoff từng UAV liền nhau; không ARM cả ba rồi mất thời gian kiểm tra trước khi takeoff.

**Takeoff bị `success=False, result=4`:** result 4 là `MAV_RESULT_FAILED`. Trong phiên đã gặp, UAV vẫn ở `STABILIZE`; phải xác nhận `GUIDED` trước khi ARM/takeoff. [Các lệnh Guided của ArduPilot](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html), [MAV_RESULT](https://mavlink.io/en/messages/common.html#MAV_RESULT).

### 10.3. Thử vận tốc cho cả ba UAV

Phép thử vận tốc không cần dùng cùng gốc vị trí local. Topic mặc định của MAVROS nhận vận tốc **ENU**: x Đông, y Bắc, z lên. Plugin tự chuyển sang NED cho FCU. [Plugin setpoint_velocity](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_velocity.cpp).

Trong T8, định nghĩa:

```zsh
send_velocity_all() {
  local velocity_x="$1"
  local message_count="$2"
  local uav_id
  local publisher_pid
  local failed=0
  local -a publisher_pids

  for uav_id in 1 2 3; do
    ros2 topic pub \
      --rate 10 \
      --times "$message_count" \
      --print 0 \
      --max-wait-time-secs 5 \
      "/uav${uav_id}/mavros/setpoint_velocity/cmd_vel" \
      geometry_msgs/msg/TwistStamped \
      "{header: {stamp: now, frame_id: 'map'}, twist: {linear: {x: ${velocity_x}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" &
    publisher_pids+=("$!")
  done

  for publisher_pid in "${publisher_pids[@]}"; do
    wait "$publisher_pid" || failed=1
  done
  return "$failed"
}
```

Chạy lần lượt hai lệnh; luôn gửi bước dừng sau phép thử:

```zsh
send_velocity_all 0.5 60
send_velocity_all 0.0 20
```

Ba publisher chạy song song ở 10 Hz. Lệnh đầu phát 60 message, khoảng 6 giây **thực** sau khi bắt đầu publish; lệnh sau phát vận tốc bằng không khoảng 2 giây thực. UAV bay cùng hướng Đông và dừng lại. Đây chưa phải thuật toán giữ đội hình hay tránh va chạm.

Không mặc định quãng đường là 3 m khi RTF thấp: 6 giây thực không tương đương 6 giây mô phỏng. Quan sát pose/Gazebo thay vì suy ra quãng đường chỉ từ số message.

Nếu đang chạy một UAV, không dùng hàm ba UAV này; dùng một publisher với namespace tương ứng.

### 10.4. Thử setpoint vị trí một UAV

Sau khi UAV 1 đã takeoff, trong T8:

```zsh
ros2 topic pub \
  --rate 10 \
  --print 10 \
  /uav1/mavros/setpoint_position/local \
  geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Đích `(3, 0, 3)` m là vị trí ENU tương đối với gốc EKF của **UAV 1**, không phải dịch thêm 3 m từ vị trí hiện tại và không mặc định là tọa độ world Gazebo. Quaternion hợp lệ `w=1` đặt yaw 0 trong ENU. Nhấn `Ctrl+C` dừng publisher trước khi dùng loại setpoint khác hoặc LAND.

Với UAV 2/3, đổi namespace và xác định đích trong hệ local của UAV đó. Không gửi cùng tọa độ world vào mọi topic local khi chưa có transform. `frame_id: 'map'` không tự yêu cầu MAVROS tìm TF để đổi hệ; plugin position nhận các giá trị như tọa độ local ENU. [Plugin setpoint_position](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_position.cpp).

## 11. Hạ cánh, dừng và khởi động lại

### 11.1. LAND

Trong T8, sau khi dừng/kết thúc publisher điều khiển:

```zsh
for uav_id in 1 2 3; do
  ros2 service call \
    "/uav${uav_id}/mavros/set_mode" \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'LAND'}"
done
```

Quan sát UAV xuống đất, kiểm tra state chuyển LAND rồi `armed: false`. Với một UAV, gọi service tương tự cho `uav1`.

### 11.2. Dừng sạch

1. Kết thúc các publisher CLI ở T8.
2. LAND và chờ UAV hạ cánh/disarm.
3. `Ctrl+C` ba MAVROS T5–T7.
4. `Ctrl+C` ba binary SITL T2–T4.
5. `Ctrl+C` Gazebo T1, đóng GUI nếu còn mở.

Khi restart: chạy lại T1–T7, yêu cầu telemetry lại, chờ EKF/pose rồi mới GUIDED/ARM/takeoff. Không phải clone, cài package hoặc build lại mỗi lần.

Thư mục `sitl/0`, `sitl/1`, `sitl/2` giữ `eeprom.bin` và log. Parameter đã lưu có thể ghi đè defaults ở lần chạy sau. Quy trình thông thường không xóa eeprom và không dùng `-w`. Nếu cần thử cấu hình hoàn toàn mới, dùng bộ thư mục runtime mới để giữ lại dữ liệu cũ.

## 12. Các lỗi đã gặp và cách xử lý

### 12.1. `python-argparse` trên Noble

**Nguyên nhân:** installer của ArduPilot 4.5.3 thêm package cũ do nhánh xử lý Ubuntu chưa bao phủ Noble.

**Cách dùng trong hướng dẫn:** APT thủ công ở mục 3.1 + venv 3.11 ở mục 4.2. Không cần sửa `argparse`, vốn thuộc thư viện chuẩn Python 3.

### 12.2. Waf báo thiếu `imp`

**Nguyên nhân:** dùng Python 3.12 cho Waf cũ.

**Fix:** activate venv 3.11 và chạy `python ./waf`. Các `SyntaxWarning: invalid escape sequence` không phải nguyên nhân của lỗi import này. Không đổi interpreter hệ thống hay sửa Waf để ép build.

### 12.3. CMake báo thiếu `catkin_pkg`

**Nguyên nhân:** CMake chọn Python uv/venv 3.11, trong khi dependency ament đã cài cho hệ thống 3.12.

**Fix:** lệnh configure ở mục 5.3, giữ cả `--fresh` và `-DPython3_EXECUTABLE=/usr/bin/python3`. Thoát venv thôi có thể chưa đủ vì cache vẫn giữ interpreter cũ.

### 12.4. MAVROS TCP mở được nhưng `connected: false`

Trong phiên lỗi, các TCP 5760/5770/5780 vẫn listen và MAVROS báo `link opened successfully`, nhưng không có heartbeat. SITL đã không nạp được defaults và đi vào PANIC; socket mở không chứng minh FCU hoạt động.

Hai lỗi launcher 4.5.3 đã xác định:

1. Chạy một instance với `--auto-sysid` có thể thêm một cấp `../` dù không tạo thư mục con instance.
2. Chạy nhiều instance với frame có **nhiều file defaults**: launcher ghép các file bằng dấu phẩy rồi thêm `../` vào cả chuỗi; chỉ file đầu được điều chỉnh.

Trong trường hợp thứ hai, `gazebo-iris.parm` bị trỏ nhầm tới `~/sitl_runs/ardupilot_453/...`, không tồn tại. ArduPilot báo `PANIC: Failed to load defaults`.

**Fix vận hành đã dùng:** chạy binary trực tiếp với hai đường dẫn tuyệt đối và SYSID rõ ràng, như T2–T4. Vì vậy tài liệu không đưa lệnh `sim_vehicle.py --count 3 --auto-sysid` làm đường chạy chính.

Nếu vẫn `connected: false`, kiểm tra theo thứ tự:

```zsh
ps -C arducopter -o pid,stat,args
ss -ltnp '( sport = :5760 or sport = :5770 or sport = :5780 )'

ls -l \
  ~/ardupilot_453/Tools/autotest/default_params/copter.parm \
  ~/ardupilot_453/Tools/autotest/default_params/gazebo-iris.parm
```

Sau đó đọc trực tiếp T2–T4: có `Loaded defaults`, có `PANIC` hoặc đang `No JSON sensor message received` không? Nếu chờ JSON, kiểm tra T1 có chạy đúng world, không pause, ba cổng plugin đúng và IMU được tìm thấy.

Thông báo `your FCU don't support AUTOPILOT_VERSION` khi không có heartbeat chỉ là hậu quả request không được trả lời; chưa chứng minh firmware không hỗ trợ. Warning `plugin_denylist pattern ... does not match` trong phiên này cũng không phải nguyên nhân mất heartbeat.

### 12.5. Đoạn code sửa bug defaults nếu sau này muốn sửa launcher

**Đoạn này để tham khảo, không phải bước bắt buộc và chưa áp dụng vào source.** Đường chạy bằng binary ở mục 8 đã tránh bug. Nếu sau này chủ động sửa `Tools/autotest/sim_vehicle.py`, trong nhánh xử lý `default_params_filename` có thể dùng đường dẫn tuyệt đối cho từng file:

```python
paths = stuff["default_params_filename"]
if not isinstance(paths, list):
    paths = [paths]
paths = [os.path.abspath(os.path.join(autotest_dir, p)) for p in paths]
for filename in paths:
    if not os.path.isfile(filename):
        print("The parameter file (%s) does not exist" % filename)
        sys.exit(1)
path = ",".join(paths)
progress("Using defaults from (%s)" % path)
```

Đoạn này thay cả phần tạo relative paths và phần thêm `../` cho danh sách defaults. Khi mọi phần tử là absolute, không cần phụ thuộc thư mục làm việc hay count. Không đưa lệnh patch tự động vào quy trình để giữ nguyên checkout đã pin.

### 12.6. Pose không có output

Các nhánh cần phân biệt:

| Dấu hiệu | Nguyên nhân cần kiểm tra | Hành động |
|---|---|---|
| State `connected: false` | FCU chưa boot/không heartbeat | Mục 12.4 |
| Connected, chưa yêu cầu ID 32 | Telemetry chưa được bật | Mục 9.2 |
| Service ID 32 thành công, log mới `origin set` | EKF chưa có vị trí/vận tốc | Chờ pose, đọc log GPS/EKF |
| `--once` hết hạn, RTF rất thấp | Chưa đủ thời gian mô phỏng | Đo stats, chờ theo trạng thái |
| Topic tồn tại nhưng publisher count 0 | Sai namespace/plugin chưa chạy | `topic info --verbose` |
| Có publisher nhưng QoS không khớp | Subscriber không nhận được | Echo với best effort như mục 9.4 |

Trong kiểm tra một UAV trước đó, nguyên nhân là stream chưa bật. Trong kiểm tra ba UAV sau khi đã bật stream, nguyên nhân là EKF chưa ready trong lúc Gazebo chạy chậm. Cuối phiên, cả ba đã xuất pose; không cần sửa code ROS để xử lý hai tình huống này.

### 12.7. `PreArm: Gyros inconsistent`

Thông báo này nghĩa là kiểm tra INS thấy các gyro không nhất quán, có thể chặn ARM. Nó đã xuất hiện trong lúc khởi tạo của cả ba UAV; sau đó pose vẫn xuất được. Vì vậy không đồng nhất lỗi prearm này với việc topic pose không có dữ liệu.

Nếu lỗi tiếp tục xuất hiện khi ARM:

1. Đọc log FCU hiện tại để biết lỗi còn hoạt động hay chỉ là dòng cũ khi boot.
2. Kiểm tra cả ba UAV đứng ổn định trong Gazebo, không đè lên nhau, không pause.
3. Kiểm tra tốc độ mô phỏng và log IMU đúng scope model; không sửa tên nested IMU tùy ý.
4. Nếu restart, dừng sạch và khởi chạy đủ Gazebo + ba SITL + ba MAVROS rồi chờ EKF.

Cuộc hội thoại chưa xác định một bản vá gyro cụ thể cần áp dụng. Tài liệu không hướng dẫn tắt `ARMING_CHECK` để che lỗi hoặc force ARM.

### 12.8. ARM rồi disarm; takeoff result 4

ARM `success=True, result=0` là đã được chấp nhận. Tự disarm sau khi không cất cánh không có nghĩa lệnh ARM ban đầu thất bại. Dùng GUIDED, ARM/takeoff liền nhau cho từng UAV. Nếu result 4, kiểm tra mode thực tế, armed và prearm log; không suy ra một nguyên nhân duy nhất từ result code.

## 13. Hiệu năng và thời gian mô phỏng

### 13.1. Đo Real Time Factor

Ở terminal có Gazebo chạy:

```zsh
timeout 5s gz topic -e -t /world/iris_3uav/stats
```

Quan sát:

- `sim_time`: thời gian mô phỏng.
- `real_time`: thời gian thực từ lúc server chạy.
- `real_time_factor`: tốc độ mô phỏng tại thời điểm đo; có thể dao động mạnh.
- `paused`: nếu true, mô phỏng đang pause; giá trị false có thể không được in bởi protobuf.

Ước lượng trung bình bằng `sim_time / real_time` khi phiên chạy liên tục từ đầu. Trong phiên đã đo: `58.7 / 448 ≈ 0.13`. Khi RTF thấp, heartbeat, EKF và message scheduler theo đồng hồ FCU cũng chậm trên thời gian thực.

### 13.2. Hướng giảm tải

Ba model hiện tại đều có gimbal/camera luôn bật. Chưa chứng minh camera là nguyên nhân duy nhất làm RTF thấp, nhưng đây là thành phần nên đo trước khi port MPC.

Các thử nghiệm tiếp theo, mỗi lần chỉ đổi một yếu tố và đo lại:

1. Dùng world riêng với model Iris không gimbal/camera nếu mục tiêu chỉ là điều khiển vị trí.
2. Giảm rate/resolution camera trong bản sao cấu hình khi cần camera.
3. Chạy server không GUI để so sánh tải render; không coi headless là bảo đảm camera không render.
4. Kiểm tra GPU/driver và phần mềm render nếu GUI/server chiếm CPU bất thường.

Giữ nguyên world đã chạy để đối chiếu. Không tăng `SIM_SPEEDUP` chỉ để che vấn đề RTF hoặc tăng `max_step_size` mà chưa kiểm tra tác động tới physics/control.

### 13.3. Đồng hồ cho controller tương lai

Các lệnh CLI ở tài liệu dùng thời gian thực của ROS CLI; header `stamp: now` được cập nhật mỗi lần publish. Tần số yêu cầu FCU và thời gian vật lý trong Gazebo không nhất thiết tương đương với tần số đo trên wall clock khi RTF thấp.

Khi xây adapter MPC, phải chốt cách dùng đồng hồ, dt và tuổi dữ liệu. Không chỉ đặt `use_sim_time=true` nếu ROS chưa có `/clock`. Cần cấu hình bridge `/clock` và kiểm tra đồng bộ MAVROS/FCU nếu chọn simulation time. Bài kiểm tra CLI hiện tại chưa dùng cơ chế này.

## 14. Hướng nối thuật toán MPC

Phần này là hướng triển khai tiếp GĐ0, không phải code adapter đã hoàn thành.

### 14.1. Môi trường MPC offline

Nếu cần cài từ đầu, dùng venv 3.12 riêng và dependency của repository:

```zsh
sudo apt install -y libgmp-dev

/usr/bin/python3 -m venv ~/venvs/mpc-py312
source ~/venvs/mpc-py312/bin/activate
python -m pip install --upgrade pip setuptools wheel

cd ~/motion_planning_MPC
python -m pip install -r requirements.txt
python -m pip check
python -c "import casadi, pathfind, pydecomp; print('MPC dependencies OK')"
```

`pydecomp==2.0.0` cần `pycddlib<3` theo requirements/README hiện tại; cài GMP trước. Venv này dành cho MPC offline, không mặc định chứa `rclpy`. Khi tạo adapter ROS sau này, dùng Python 3.12 tương thích ROS và cấu hình dependency riêng.

Repository hiện có các scenario `1`, `2`, `3`, `big1000`; cấu hình mặc định `9` và ví dụ README `6` không có file tương ứng trong checkout đã đọc. Chọn scenario tồn tại khi kiểm tra:

```zsh
cd ~/motion_planning_MPC
python config.py big1000
```

Lệnh này báo cấu hình/validation, chưa điều khiển Gazebo. Số UAV trong scenario Python lấy từ `starts`, độc lập với ba model SITL; phải tạo/chọn scenario ba UAV khi thực hiện port, không mặc định YAML hiện có khớp world.

### 14.2. Các phần cần tách

| Phần hiện tại | Cách dùng khi port |
|---|---|
| `mpc_problem.py`: state phẳng `[px, py, vx, vy, ax, ay]`, control `[ux, uy]` | Giữ solver, cấp state từ estimator; hiệu chỉnh lag/drag theo FCU |
| `robot_jps.py`: `computeControlSignal()` gọi scan và cập nhật state | Tách quyết định điều khiển khỏi bước tích phân mô phỏng |
| `updateState()` tích phân model Python | Không dùng làm nguồn pose thật của UAV Gazebo |
| `lidar.py`: LiDAR analytic/march | Thay bằng dữ liệu cảm biến/adapter, hoặc map tổng hợp có kiểm soát ở giai đoạn đầu |
| `planner_grid.py` và `planner_jps.py` | Giữ interface map/reference; mặc định hiện tại `PLANNER=local` là Dijkstra, không mặc định JPS |
| Target trajectory/Kalman | Cấp target measurement có cùng frame/time |
| Formation/neighbour predictions | Cập nhật từ cả ba UAV trong ENU chung; kiểm soát tuổi dữ liệu |

Không dùng state dự đoán/integrate của Python để ghi đè measurement MAVROS. Tách logging, dịch horizon và cập nhật neighbour prediction khỏi thao tác tích phân.

### 14.3. ENU chung và ENU local từng UAV

Tại lần đọc cuối, ba pose local đều gần `(0, 0, 0)` dù Gazebo spawn ở x = 0, 6, 12 m. Tên `map` giống nhau chỉ là tên frame, chưa bảo đảm có cùng origin.

Nếu hướng trục đã căn cùng ENU, một mapping đã được hiệu chuẩn có dạng:

```text
p_common_i = p_local_i + offset_i
p_local_goal_i = p_common_goal_i - offset_i
```

Không gán cứng offset chỉ từ pose spawn mà chưa đối chiếu measurement. Có thể xác định offset từ ground truth Gazebo tại lúc khởi tạo hoặc từ tọa độ GPS/origin của từng FCU so với origin chung. Nếu còn khác hướng trục, dùng transform đầy đủ gồm rotation và translation.

Vận tốc cũng phải quay về cùng hướng trục khi rotation khác nhau; translation không áp dụng cho vận tốc. Khi dùng TF, đặt frame rõ như `world`, `uav1/map`, `uav1/base_link`, v.v., rồi cấu hình frame của các node; namespace ROS không tự namespace mọi TF frame.

### 14.4. Setpoint và thời gian giải

Lộ trình triển khai:

1. Adapter đọc pose/vận tốc cho một UAV; chạy reference đơn giản bằng position/velocity.
2. Mở rộng cả ba UAV, kiểm chứng transform và khoảng cách.
3. Nối MPC với dữ liệu ngoài; ban đầu cố định độ cao, map/target đơn giản.
4. Thử `/mavros/setpoint_raw/local` với pos + vel + acc và yaw theo yêu cầu mục 0.3; kiểm tra `coordinate_frame`, `type_mask`, ENU/NED và phần nào ArduCopter thực sự sử dụng.
5. Đối chiếu reference, actual state, solver status và tuổi dữ liệu qua rosbag/log.

Không suy ra `control U` của MPC là motor PWM hoặc thrust. U hiện tại là commanded acceleration trong mô hình lag; FCU có vòng điều khiển riêng. Phải kiểm chứng mapping, saturation và bám quỹ đạo trước khi sử dụng trên Solo.

Tách **solver** khỏi **bộ phát setpoint định kỳ**. Solver chậm hoặc infeasible không được làm mất vòng publish. Cần chính sách cho stale state/command, timeout solver và trạng thái giữ vị trí/phanh khi không có nghiệm mới. Đo mean, p95, p99, max thời gian solve và toàn bộ chu kỳ ba UAV; không chỉ dựa vào trung bình một UAV.

Mục 0.4 yêu cầu controller trên Pi, SITL trên laptop. Tới bước đó mới cấu hình mạng/DDS/MAVLink giữa hai máy và benchmark thực tế trên Pi; quy trình localhost hiện tại chưa xác nhận thời gian dưới 100 ms trên Pi.

### 14.5. FOV và độ cao

Với camera nhìn thẳng xuống mặt phẳng, một footprint lý tưởng có nửa cạnh:

```text
half_width  = h × tan(HFOV / 2)
half_height = h × tan(VFOV / 2)
```

`h` là khoảng cách tới mặt phẳng mục tiêu, không luôn bằng altitude MSL. Cần thêm biên cho góc gimbal, dao động độ cao, sai số pose và che khuất. `VIEWING_RADIUS` của mô hình hiện tại là nửa cạnh footprint vuông; không mặc định là bán kính tròn. FOV camera Gazebo trong model cũng cần được đối chiếu với camera mục tiêu trước khi nghiệm thu 0.5.

## 15. Tiêu chí hoàn thành và tài liệu tham khảo

### 15.1. Điều đã xác nhận trong cuộc hội thoại

| Hạng mục | Trạng thái |
|---|---|
| ArduCopter 4.5.3 build với Python 3.11 | Đã thành công |
| Plugin Gazebo build với Python hệ thống 3.12 | Đã thành công |
| Một UAV trong Gazebo, ARM/takeoff/điều khiển qua ROS | Người dùng xác nhận chạy tốt |
| World ba UAV và SDF hợp lệ | Đã kiểm tra |
| Ba FCU kết nối MAVROS | Đã kiểm tra `connected: true` |
| Ba pose cập nhật | Đã đo mỗi UAV 14 bản tin trong 12 giây thực |
| RTF đủ gần 1 và pose 10 Hz trên wall clock | Chưa đạt trong phiên đo; cần tối ưu/đo lại |
| Ba UAV bay/bám reference MPC | Chưa xác nhận |
| ROS adapter cho MPC, common ENU, benchmark Pi và FOV | Công việc tiếp theo |

### 15.2. Checklist cho mỗi lần chạy

- [ ] Đã dừng phiên cũ, không có xung đột cổng.
- [ ] T1 chạy đúng world, ba UAV đứng tách nhau, không pause.
- [ ] T2–T4 dùng đúng binary 4.5.3 và hai defaults absolute.
- [ ] T5–T7 dùng đúng TCP, SYSID và namespace.
- [ ] Cả ba state `connected: true`.
- [ ] Đã yêu cầu ID 32/30 sau lần boot hiện tại.
- [ ] Cả ba pose đang cập nhật; đã xem RTF nếu chờ lâu.
- [ ] Mode thực tế `GUIDED`, prearm không chặn ARM.
- [ ] ARM/takeoff từng UAV liền nhau, kiểm tra bay thực tế.
- [ ] Đã dừng publisher, LAND và xác nhận disarm trước khi kết thúc.

### 15.3. Nguồn chính

- Kế hoạch `plan_3dr.xlsx`, mục GĐ0 0.1–0.5, và tài liệu `3DR Solo.pdf` người dùng cung cấp.
- Source/tag ArduPilot 4.5.3: `Tools/autotest/sim_vehicle.py`, `libraries/AP_HAL_SITL/SITL_cmdline.cpp`, `libraries/GCS_MAVLink/GCS_Common.cpp`, `libraries/AP_Arming/AP_Arming.cpp`.
- Source MPC hiện tại: `robot_jps.py`, `mpc_problem.py`, `dynamics.py`, `config.py`, `requirements.txt`.
- [ROS Jazzy: Ubuntu deb installation](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst).
- [Gazebo Harmonic: Ubuntu installation](https://gazebosim.org/docs/harmonic/install_ubuntu/).
- [ArduPilot: Gazebo SITL](https://ardupilot.org/dev/docs/sitl-with-gazebo.html).
- [ArduPilot: requesting telemetry](https://ardupilot.org/dev/docs/mavlink-requesting-data.html).
- [ArduPilot: Copter Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html).
- [MAVROS ROS 2](https://github.com/mavlink/mavros/tree/ros2).
- [Python: removal of imp](https://docs.python.org/3.12/library/imp.html).
- [CMake: FindPython3](https://cmake.org/cmake/help/latest/module/FindPython3.html).
