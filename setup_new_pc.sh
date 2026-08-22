#!/bin/bash
set -eo pipefail

echo "========================================"
echo " com1 새 PC 자동 설치 시작"
echo "========================================"

# --------------------------------------------------
# 0. 기본 경로 / 고정 버전
# --------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COM1_DIR="$SCRIPT_DIR"
PX4_DIR="$COM1_DIR/PX4-Autopilot"
PX4_MSGS_WS="$COM1_DIR/ws_px4_msgs"
MICROXRCE_DIR="$COM1_DIR/Micro-XRCE-DDS-Agent"
GZ_WS="$HOME/gz_ws"

CUSTOM_PX4_DIR="$COM1_DIR/custom_px4"
GAZEBO_DEB_DIR="$COM1_DIR/gazebo_debs"

PX4_COMMIT="2253701d6a8aa771d6b436400007daefd51a4806"
PX4_MSGS_COMMIT="a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8"
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

source /etc/os-release

echo "OS: $PRETTY_NAME"

if [ "${VERSION_ID:-}" != "22.04" ]; then
    echo "ERROR: Ubuntu 22.04 전용 스크립트입니다."
    exit 1
fi

# --------------------------------------------------
# 2. 기본 패키지 설치
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
    echo "ERROR: ROS2 Humble이 설치되어 있지 않습니다."
    exit 1
fi

source /opt/ros/humble/setup.bash

if [ "${ROS_DISTRO:-}" != "humble" ]; then
    echo "ERROR: ROS_DISTRO가 humble이 아닙니다."
    exit 1
fi

# --------------------------------------------------
# 4. Gazebo DEB 무결성 확인
# --------------------------------------------------

echo
echo "===== 4. Gazebo DEB 무결성 확인 ====="

if [ ! -d "$GAZEBO_DEB_DIR" ]; then
    echo "ERROR: gazebo_debs 폴더 없음"
    exit 1
fi

cd "$GAZEBO_DEB_DIR"

sha256sum -c SHA256SUMS

# --------------------------------------------------
# 5. Gazebo / ROS-GZ 고정 버전 설치
# --------------------------------------------------

echo
echo "===== 5. Gazebo 고정 버전 설치 ====="

sudo apt install -y ./*.deb

sudo ldconfig

# --------------------------------------------------
# 6. Gazebo 핵심 패키지 hold
# --------------------------------------------------

echo
echo "===== 6. Gazebo 버전 고정 ====="

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
# 7. PX4 clone + exact commit
# --------------------------------------------------

echo
echo "===== 7. PX4 설치 ====="

if [ ! -d "$PX4_DIR/.git" ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi

cd "$PX4_DIR"

git fetch --all --tags
git checkout "$PX4_COMMIT"

git submodule sync --recursive
git submodule update --init --recursive

echo "PX4 commit:"
git rev-parse HEAD

# --------------------------------------------------
# 8. PX4 개발 환경
# Gazebo는 설치하지 않음
# --------------------------------------------------

echo
echo "===== 8. PX4 개발 도구 설치 ====="

cd "$PX4_DIR"

bash Tools/setup/ubuntu.sh \
    --no-sim-tools \
    --no-nuttx

# --------------------------------------------------
# 9. custom PX4 파일 적용
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
    echo "ERROR: custom Gazebo 파일 없음"
    exit 1
fi

rm -rf "$PX4_DIR/New_code"
rm -rf "$PX4_DIR/navigation"
rm -rf "$PX4_DIR/Tools/simulation/gz"

cp -a "$CUSTOM_PX4_DIR/New_code" "$PX4_DIR/"
cp -a "$CUSTOM_PX4_DIR/navigation" "$PX4_DIR/"

mkdir -p "$PX4_DIR/Tools/simulation"

cp -a \
    "$CUSTOM_PX4_DIR/Tools/simulation/gz" \
    "$PX4_DIR/Tools/simulation/"

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

# --------------------------------------------------
# 11. ros_gz exact commit 빌드
# --------------------------------------------------

echo
echo "===== 11. ros_gz 빌드 ====="

mkdir -p "$GZ_WS/src"

if [ ! -d "$GZ_WS/src/ros_gz/.git" ]; then
    git clone \
        https://github.com/gazebosim/ros_gz.git \
        "$GZ_WS/src/ros_gz"
fi

cd "$GZ_WS/src/ros_gz"

git fetch --all --tags
git checkout "$ROS_GZ_COMMIT"

echo "ros_gz commit:"
git rev-parse HEAD

cd "$GZ_WS"

source /opt/ros/humble/setup.bash

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

# --------------------------------------------------
# 12. px4_msgs exact commit 구성 및 빌드
# --------------------------------------------------

echo
echo "===== 12. px4_msgs 구성 및 빌드 ====="

mkdir -p "$PX4_MSGS_WS/src"

if [ ! -d "$PX4_MSGS_WS/src/px4_msgs/.git" ]; then
    rm -rf "$PX4_MSGS_WS/src/px4_msgs"

    git clone \
        https://github.com/PX4/px4_msgs.git \
        "$PX4_MSGS_WS/src/px4_msgs"
fi

cd "$PX4_MSGS_WS/src/px4_msgs"

git fetch --all --tags
git checkout "$PX4_MSGS_COMMIT"

echo "px4_msgs commit:"
git rev-parse HEAD

cd "$PX4_MSGS_WS"

source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build

if [ ! -f "$PX4_MSGS_WS/install/setup.bash" ]; then
    echo "ERROR: ws_px4_msgs 빌드 실패"
    exit 1
fi

# --------------------------------------------------
# 13. Python 패키지
# --------------------------------------------------

echo
echo "===== 13. Python 패키지 설치 ====="

python3 -m pip install --user \
    numpy==1.26.4 \
    opencv-contrib-python==4.8.1.78 \
    pandas==2.3.3

sudo apt install -y \
    python3-scipy \
    python3-yaml

# --------------------------------------------------
# 14. 핵심 버전 검증
# --------------------------------------------------

echo
echo "===== 14. 버전 검증 ====="

CHECK_GZ_SIM="$(dpkg-query -W -f='${Version}' gz-sim8-cli)"
CHECK_GZ_TRANSPORT="$(dpkg-query -W -f='${Version}' libgz-transport13)"
CHECK_GZ_MSGS="$(dpkg-query -W -f='${Version}' libgz-msgs10)"

echo "gz-sim8-cli    : $CHECK_GZ_SIM"
echo "transport13    : $CHECK_GZ_TRANSPORT"
echo "msgs10         : $CHECK_GZ_MSGS"

if [[ "$CHECK_GZ_SIM" != 8.12.0-* ]]; then
    echo "ERROR: gz-sim8 버전 불일치"
    exit 1
fi

if [[ "$CHECK_GZ_TRANSPORT" != 13.5.0-* ]]; then
    echo "ERROR: transport13 버전 불일치"
    exit 1
fi

if [[ "$CHECK_GZ_MSGS" != 10.3.2-* ]]; then
    echo "ERROR: msgs10 버전 불일치"
    exit 1
fi

# --------------------------------------------------
# 15. Git commit 검증
# --------------------------------------------------

echo
echo "===== 15. Git commit 검증 ====="

PX4_NOW="$(git -C "$PX4_DIR" rev-parse HEAD)"
PX4_MSGS_NOW="$(git -C "$PX4_MSGS_WS/src/px4_msgs" rev-parse HEAD)"
ROS_GZ_NOW="$(git -C "$GZ_WS/src/ros_gz" rev-parse HEAD)"
MICROXRCE_NOW="$(git -C "$MICROXRCE_DIR" rev-parse HEAD)"

echo "PX4        : $PX4_NOW"
echo "px4_msgs   : $PX4_MSGS_NOW"
echo "ros_gz     : $ROS_GZ_NOW"
echo "MicroXRCE  : $MICROXRCE_NOW"

if [ "$PX4_NOW" != "$PX4_COMMIT" ]; then
    echo "ERROR: PX4 commit 불일치"
    exit 1
fi

if [ "$PX4_MSGS_NOW" != "$PX4_MSGS_COMMIT" ]; then
    echo "ERROR: px4_msgs commit 불일치"
    exit 1
fi

if [ "$ROS_GZ_NOW" != "$ROS_GZ_COMMIT" ]; then
    echo "ERROR: ros_gz commit 불일치"
    exit 1
fi

if [ "$MICROXRCE_NOW" != "$MICROXRCE_COMMIT" ]; then
    echo "ERROR: MicroXRCE commit 불일치"
    exit 1
fi

# --------------------------------------------------
# 16. 실행 환경 최종 확인
# --------------------------------------------------

echo
echo "===== 16. 실행 환경 확인 ====="

source /opt/ros/humble/setup.bash
source "$GZ_WS/install/setup.bash"
source "$PX4_MSGS_WS/install/setup.bash"

echo
echo "[Gazebo]"
gz sim --version || true

echo
echo "[ros_gz_bridge]"
ros2 pkg prefix ros_gz_bridge || true

echo
echo "[ros_gz_image]"
ros2 pkg prefix ros_gz_image || true

echo
echo "[MicroXRCEAgent]"
ls -lh "$MICROXRCE_DIR/build/MicroXRCEAgent"

echo
echo "[PX4]"
ls -ld "$PX4_DIR"

echo
echo "[px4_msgs]"
ls -ld "$PX4_MSGS_WS/install"

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
echo "px4_msgs:"
echo "$PX4_MSGS_WS"

echo
echo "MicroXRCEAgent:"
echo "$MICROXRCE_DIR/build/MicroXRCEAgent"

echo
echo "ros_gz:"
echo "$GZ_WS"

echo
echo "다음 실행:"
echo "cd $COM1_DIR"
echo "./run7.sh"
echo
