#!/bin/bash
set -euo pipefail

echo "========================================"
echo " com1 새 PC 자동 설치 시작"
echo "========================================"

# --------------------------------------------------
# 0. 기본 경로 / 버전
# --------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COM1_DIR="$SCRIPT_DIR"
PX4_DIR="$COM1_DIR/PX4-Autopilot"
PX4_MSGS_WS="$COM1_DIR/ws_px4_msgs"
MICROXRCE_DIR="$COM1_DIR/Micro-XRCE-DDS-Agent"

# run7.sh가 현재 ~/gz_ws를 사용하므로 그대로 맞춤
GZ_WS="$HOME/gz_ws"

CUSTOM_PX4_DIR="$COM1_DIR/custom_px4"
GAZEBO_DEB_DIR="$COM1_DIR/gazebo_debs"

PX4_COMMIT="2253701d6a8aa771d6b436400007daefd51a4806"
ROS_GZ_COMMIT="9d7f8c721c233a9ac8b43950129d51e67905523e"
MICROXRCE_COMMIT="73622810d984349b80bbac0ef55fc0b694d62222"

echo
echo "[경로]"
echo "COM1       : $COM1_DIR"
echo "PX4        : $PX4_DIR"
echo "PX4_MSGS   : $PX4_MSGS_WS"
echo "MicroXRCE  : $MICROXRCE_DIR"
echo "ros_gz     : $GZ_WS"

# --------------------------------------------------
# 1. Ubuntu 버전 확인
# --------------------------------------------------

echo
echo "===== 1. Ubuntu 버전 확인 ====="

if [ ! -f /etc/os-release ]; then
    echo "ERROR: /etc/os-release 없음"
    exit 1
fi

source /etc/os-release

echo "OS: $PRETTY_NAME"

if [ "${VERSION_ID:-}" != "22.04" ]; then
    echo
    echo "ERROR:"
    echo "이 프로젝트는 정상 PC와 동일한 Ubuntu 22.04 기준입니다."
    echo "현재 버전: ${VERSION_ID:-unknown}"
    exit 1
fi

echo "Ubuntu 22.04 확인 완료"

# --------------------------------------------------
# 2. 기본 개발 도구
# --------------------------------------------------

echo
echo "===== 2. 기본 개발 도구 설치 ====="

sudo apt update

sudo apt install -y \
    git \
    git-lfs \
    build-essential \
    cmake \
    ninja-build \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    curl \
    wget \
    unzip \
    rsync \
    pkg-config \
    gnome-terminal

# --------------------------------------------------
# 3. ROS2 Humble 확인
# --------------------------------------------------

echo
echo "===== 3. ROS2 Humble 확인 ====="

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo
    echo "ERROR:"
    echo "/opt/ros/humble/setup.bash 가 없습니다."
    echo
    echo "ROS2 Humble이 먼저 설치되어 있어야 합니다."
    echo "정확한 정상 PC 환경 재현을 위해 여기서 중단합니다."
    exit 1
fi

source /opt/ros/humble/setup.bash

echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"

if [ "${ROS_DISTRO:-}" != "humble" ]; then
    echo "ERROR: ROS2 Humble 환경이 아닙니다."
    exit 1
fi

echo "ROS2 Humble 확인 완료"

# --------------------------------------------------
# 4. Gazebo 백업 DEB 무결성 확인
# --------------------------------------------------

echo
echo "===== 4. Gazebo DEB 무결성 확인 ====="

if [ ! -d "$GAZEBO_DEB_DIR" ]; then
    echo "ERROR: $GAZEBO_DEB_DIR 없음"
    exit 1
fi

if [ ! -f "$GAZEBO_DEB_DIR/SHA256SUMS" ]; then
    echo "ERROR: SHA256SUMS 없음"
    exit 1
fi

cd "$GAZEBO_DEB_DIR"

sha256sum -c SHA256SUMS

echo "Gazebo DEB SHA256 확인 완료"

# --------------------------------------------------
# 5. 정상 PC Gazebo / ROS-GZ 버전 설치
# --------------------------------------------------

echo
echo "===== 5. 정상 PC Gazebo 버전 설치 ====="

cd "$GAZEBO_DEB_DIR"

sudo apt install -y ./*.deb

sudo ldconfig

echo
echo "Gazebo 설치 결과:"
gz sim --version || true

echo
echo "핵심 패키지 버전:"
dpkg-query -W \
    gz-sim8-cli \
    libgz-sim8 \
    libgz-transport13 \
    libgz-msgs10 \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ros-gz-interfaces

# --------------------------------------------------
# 6. 핵심 Gazebo 패키지 HOLD
# --------------------------------------------------

echo
echo "===== 6. Gazebo 핵심 패키지 버전 고정 ====="

sudo apt-mark hold \
    gz-sim8-cli \
    libgz-sim8 \
    libgz-sim8-dev \
    libgz-sim8-plugins \
    libgz-transport13 \
    libgz-transport13-dev \
    libgz-msgs10 \
    libgz-msgs10-dev \
    python3-gz-sim8 \
    python3-gz-transport13 \
    python3-gz-msgs10 \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ros-gz-interfaces

# --------------------------------------------------
# 7. PX4 다운로드
# --------------------------------------------------

echo
echo "===== 7. PX4 다운로드 ====="

if [ ! -d "$PX4_DIR/.git" ]; then

    git clone \
        https://github.com/PX4/PX4-Autopilot.git \
        "$PX4_DIR"

fi

cd "$PX4_DIR"

git fetch --all --tags

git checkout "$PX4_COMMIT"

git submodule sync --recursive

git submodule update \
    --init \
    --recursive

echo
echo "PX4 commit:"
git rev-parse HEAD

# --------------------------------------------------
# 8. PX4 개발 환경 설치
# Gazebo는 설치하지 않음
# --------------------------------------------------

echo
echo "===== 8. PX4 개발 도구 설치 ====="

cd "$PX4_DIR"

bash Tools/setup/ubuntu.sh \
    --no-sim-tools \
    --no-nuttx

# --------------------------------------------------
# 9. 사용자 PX4 코드 적용
# --------------------------------------------------

echo
echo "===== 9. 사용자 PX4 코드 적용 ====="

if [ ! -d "$CUSTOM_PX4_DIR/New_code" ]; then
    echo "ERROR: custom_px4/New_code 없음"
    exit 1
fi

if [ ! -d "$CUSTOM_PX4_DIR/navigation" ]; then
    echo "ERROR: custom_px4/navigation 없음"
    exit 1
fi

if [ ! -d "$CUSTOM_PX4_DIR/Tools/simulation/gz" ]; then
    echo "ERROR: custom Gazebo 폴더 없음"
    exit 1
fi

# 새로 clone한 PX4 내부만 수정
rm -rf "$PX4_DIR/New_code"
rm -rf "$PX4_DIR/navigation"
rm -rf "$PX4_DIR/Tools/simulation/gz"

cp -a \
    "$CUSTOM_PX4_DIR/New_code" \
    "$PX4_DIR/"

cp -a \
    "$CUSTOM_PX4_DIR/navigation" \
    "$PX4_DIR/"

mkdir -p "$PX4_DIR/Tools/simulation"

cp -a \
    "$CUSTOM_PX4_DIR/Tools/simulation/gz" \
    "$PX4_DIR/Tools/simulation/"

echo "사용자 PX4 파일 적용 완료"

# --------------------------------------------------
# 10. MicroXRCE-DDS-Agent
# --------------------------------------------------

echo
echo "===== 10. MicroXRCE-DDS-Agent 설치 ====="

if [ ! -d "$MICROXRCE_DIR/.git" ]; then

    git clone \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        "$MICROXRCE_DIR"

fi

cd "$MICROXRCE_DIR"

git fetch --all --tags

git checkout "$MICROXRCE_COMMIT"

echo
echo "MicroXRCE commit:"
git rev-parse HEAD

rm -rf build
mkdir build
cd build

cmake ..
make -j"$(nproc)"

if [ ! -x "$MICROXRCE_DIR/build/MicroXRCEAgent" ]; then
    echo "ERROR: MicroXRCEAgent 빌드 실패"
    exit 1
fi

echo "MicroXRCEAgent 빌드 완료"

# --------------------------------------------------
# 11. ros_gz 정확한 버전 빌드
# --------------------------------------------------

echo
echo "===== 11. ros_gz workspace 구성 ====="

mkdir -p "$GZ_WS/src"

if [ ! -d "$GZ_WS/src/ros_gz/.git" ]; then

    git clone \
        https://github.com/gazebosim/ros_gz.git \
        "$GZ_WS/src/ros_gz"

fi

cd "$GZ_WS/src/ros_gz"

git fetch --all --tags

git checkout "$ROS_GZ_COMMIT"

echo
echo "ros_gz commit:"
git rev-parse HEAD

cd "$GZ_WS"

source /opt/ros/humble/setup.bash

# rosdep이 Gazebo 최신 버전을 건드리는 위험을 줄이기 위해
# Gazebo 핵심 DEB 설치 + hold 이후 실행
sudo rosdep init 2>/dev/null || true
rosdep update

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y \
    --rosdistro humble

rm -rf build install log

colcon build

if [ ! -f "$GZ_WS/install/setup.bash" ]; then
    echo "ERROR: ros_gz 빌드 실패"
    exit 1
fi

echo "ros_gz build 완료"

# --------------------------------------------------
# 12. px4_msgs 빌드
# --------------------------------------------------

echo
echo "===== 12. ws_px4_msgs 빌드 ====="

if [ ! -d "$PX4_MSGS_WS/src" ]; then
    echo
    echo "ERROR:"
    echo "$PX4_MSGS_WS/src 가 없습니다."
    echo "GitHub 저장소에 ws_px4_msgs가 포함되어 있는지 확인하세요."
    exit 1
fi

cd "$PX4_MSGS_WS"

source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build

if [ ! -f "$PX4_MSGS_WS/install/setup.bash" ]; then
    echo "ERROR: ws_px4_msgs build 실패"
    exit 1
fi

echo "ws_px4_msgs build 완료"

# --------------------------------------------------
# 13. Python 패키지
# --------------------------------------------------

echo
echo "===== 13. Python 패키지 설치 ====="

python3 -m pip install --user \
    numpy==1.26.4 \
    opencv-contrib-python==4.8.1.78 \
    pandas==2.3.3

# scipy 1.8.0은 Ubuntu 22.04 기본 Python 3.10 환경에서
# 시스템 패키지와 충돌 가능성이 있으므로 apt 우선 사용
sudo apt install -y \
    python3-scipy \
    python3-yaml

# --------------------------------------------------
# 14. 실행 환경 검사
# --------------------------------------------------

echo
echo "===== 14. 최종 환경 검사 ====="

echo
echo "[PX4]"
cd "$PX4_DIR"
git rev-parse HEAD

echo
echo "[MicroXRCE]"
cd "$MICROXRCE_DIR"
git rev-parse HEAD

echo
echo "[ros_gz]"
cd "$GZ_WS/src/ros_gz"
git rev-parse HEAD

echo
echo "[Gazebo]"
gz sim --version || true

echo
echo "[ROS2]"
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null
echo "ROS2 Humble OK"

echo
echo "[ros_gz overlay]"
source "$GZ_WS/install/setup.bash"
ros2 pkg prefix ros_gz_bridge || true
ros2 pkg prefix ros_gz_image || true

echo
echo "[px4_msgs]"
source "$PX4_MSGS_WS/install/setup.bash"
ros2 interface list | grep px4_msgs | head || true

echo
echo "[MicroXRCEAgent]"
ls -lh "$MICROXRCE_DIR/build/MicroXRCEAgent"

# --------------------------------------------------
# 15. 정상 PC에서 확인한 핵심 버전 검증
# --------------------------------------------------

echo
echo "===== 15. 핵심 버전 검증 ====="

CHECK_GZ_SIM="$(dpkg-query -W -f='${Version}' gz-sim8-cli)"
CHECK_GZ_TRANSPORT="$(dpkg-query -W -f='${Version}' libgz-transport13)"
CHECK_GZ_MSGS="$(dpkg-query -W -f='${Version}' libgz-msgs10)"

echo "gz-sim8-cli      : $CHECK_GZ_SIM"
echo "gz-transport13   : $CHECK_GZ_TRANSPORT"
echo "gz-msgs10        : $CHECK_GZ_MSGS"

if [[ "$CHECK_GZ_SIM" != 8.12.0-* ]]; then
    echo "ERROR: Gazebo Sim 버전 불일치"
    exit 1
fi

if [[ "$CHECK_GZ_TRANSPORT" != 13.5.0-* ]]; then
    echo "ERROR: gz-transport13 버전 불일치"
    exit 1
fi

if [[ "$CHECK_GZ_MSGS" != 10.3.2-* ]]; then
    echo "ERROR: gz-msgs10 버전 불일치"
    exit 1
fi

# --------------------------------------------------
# 완료
# --------------------------------------------------

echo
echo "========================================"
echo " 설치 완료"
echo "========================================"

echo
echo "PX4:"
echo "$PX4_DIR"

echo
echo "MicroXRCEAgent:"
echo "$MICROXRCE_DIR/build/MicroXRCEAgent"

echo
echo "ros_gz:"
echo "$GZ_WS"

echo
echo "px4_msgs:"
echo "$PX4_MSGS_WS"

echo
echo "다음 실행:"
echo "cd $COM1_DIR"
echo "./run7.sh"
echo
