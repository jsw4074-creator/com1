#!/bin/bash
set -eo pipefail

echo "========================================"
echo " com1 재현 환경 자동 설치"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COM1_DIR="$SCRIPT_DIR"
PX4_DIR="$COM1_DIR/PX4-Autopilot"
PX4_MSGS_WS="$COM1_DIR/ws_px4_msgs"
MICROXRCE_DIR="$COM1_DIR/Micro-XRCE-DDS-Agent"
GZ_WS="$HOME/gz_ws"

CUSTOM_PX4_DIR="$COM1_DIR/custom_px4"
GAZEBO_DEB_DIR="$COM1_DIR/gazebo_debs"
HARMONIC_MANIFEST="$COM1_DIR/environment_snapshot/gazebo_harmonic_manifest.tsv"

PX4_COMMIT="2253701d6a8aa771d6b436400007daefd51a4806"
PX4_MSGS_COMMIT="a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8"
ROS_GZ_COMMIT="9d7f8c721c233a9ac8b43950129d51e67905523e"
MICROXRCE_COMMIT="73622810d984349b80bbac0ef55fc0b694d62222"

die()
{
    echo
    echo "ERROR: $1"
    exit 1
}

echo
echo "[경로]"
echo "COM1       : $COM1_DIR"
echo "PX4        : $PX4_DIR"
echo "PX4_MSGS   : $PX4_MSGS_WS"
echo "MicroXRCE  : $MICROXRCE_DIR"
echo "ros_gz     : $GZ_WS"

# --------------------------------------------------
# 1. OS 확인
# --------------------------------------------------

echo
echo "===== 1. Ubuntu 확인 ====="

source /etc/os-release

echo "$PRETTY_NAME"

[ "${VERSION_ID:-}" = "22.04" ] || \
    die "Ubuntu 22.04가 아닙니다."

# --------------------------------------------------
# 2. 기본 도구
# --------------------------------------------------

echo
echo "===== 2. 기본 개발 도구 ====="

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
# 3. ROS2 Humble
# --------------------------------------------------

echo
echo "===== 3. ROS2 Humble ====="

[ -f /opt/ros/humble/setup.bash ] || \
    die "ROS2 Humble이 설치되어 있지 않습니다."

source /opt/ros/humble/setup.bash

[ "${ROS_DISTRO:-}" = "humble" ] || \
    die "ROS_DISTRO가 humble이 아닙니다."

# --------------------------------------------------
# 4. Harmonic 백업 검사
# --------------------------------------------------

echo
echo "===== 4. Gazebo Harmonic 백업 검사 ====="

[ -d "$GAZEBO_DEB_DIR" ] || \
    die "gazebo_debs 폴더가 없습니다."

[ -f "$GAZEBO_DEB_DIR/SHA256SUMS" ] || \
    die "SHA256SUMS가 없습니다."

[ -f "$HARMONIC_MANIFEST" ] || \
    die "gazebo_harmonic_manifest.tsv가 없습니다."

cd "$GAZEBO_DEB_DIR"

sha256sum -c SHA256SUMS || \
    die "Gazebo DEB SHA256 검증 실패"

# Garden이 manifest에 들어가 있으면 중단
if grep -Eq \
'^(gz-garden|gz-sim7|gz-launch6|gz-transport12|libgz-gui7|libgz-launch6|libgz-msgs9|libgz-physics6|libgz-rendering7|libgz-sensors7|libgz-sim7|libgz-transport12|python3-gz-sim7)' \
"$HARMONIC_MANIFEST"
then
    die "Harmonic manifest에 Garden 패키지가 포함되어 있습니다."
fi

# --------------------------------------------------
# 5. Harmonic manifest의 정확한 DEB만 선택
# --------------------------------------------------

echo
echo "===== 5. Harmonic 정확한 DEB 선택 ====="

HARMONIC_DEBS=()
HARMONIC_PACKAGES=()

while IFS=$'\t' read -r pkg ver
do
    [ -z "$pkg" ] && continue

    clean_pkg="${pkg%%:*}"

    found=""

    for deb in "$GAZEBO_DEB_DIR"/*.deb
    do
        deb_pkg="$(dpkg-deb -f "$deb" Package)"
        deb_ver="$(dpkg-deb -f "$deb" Version)"

        if [ "$deb_pkg" = "$clean_pkg" ] && \
           [ "$deb_ver" = "$ver" ]
        then
            found="$deb"
            break
        fi
    done

    [ -n "$found" ] || \
        die "DEB 없음: $clean_pkg $ver"

    HARMONIC_DEBS+=("$found")
    HARMONIC_PACKAGES+=("$clean_pkg")

done < "$HARMONIC_MANIFEST"

echo "설치 대상 Harmonic 패키지: ${#HARMONIC_DEBS[@]}"

# --------------------------------------------------
# 6. Harmonic exact version 설치
# --------------------------------------------------

echo
echo "===== 6. Gazebo Harmonic exact version 설치 ====="

sudo apt install -y \
    --allow-downgrades \
    "${HARMONIC_DEBS[@]}"

sudo ldconfig

# --------------------------------------------------
# 7. Harmonic 전체 버전 확인
# --------------------------------------------------

echo
echo "===== 7. Harmonic 전체 버전 검증 ====="

while IFS=$'\t' read -r pkg expected
do
    [ -z "$pkg" ] && continue

    clean_pkg="${pkg%%:*}"

    actual="$(dpkg-query -W -f='${Version}' "$clean_pkg" 2>/dev/null || true)"

    if [ "$actual" != "$expected" ]; then
        echo "패키지 : $clean_pkg"
        echo "기대값 : $expected"
        echo "실제값 : $actual"
        die "Gazebo Harmonic 버전 불일치"
    fi

done < "$HARMONIC_MANIFEST"

echo "OK: Harmonic 전체 버전 일치 (${#HARMONIC_PACKAGES[@]}개)"

# 이후 apt / rosdep가 GZ를 바꾸지 못하도록 고정
echo
echo "===== 8. Harmonic 패키지 hold ====="

sudo apt-mark hold "${HARMONIC_PACKAGES[@]}"

# --------------------------------------------------
# 9. PX4
# --------------------------------------------------

echo
echo "===== 9. PX4 exact commit ====="

# GitHub 배포본에 들어있는 flattened PX4를 제거하고
# 공식 PX4 Git repository로 교체
if [ -d "$PX4_DIR" ] && [ ! -d "$PX4_DIR/.git" ]; then
    echo "비-Git PX4 배포 폴더를 공식 PX4 clone으로 교체합니다."
    rm -rf "$PX4_DIR"
fi

if [ ! -d "$PX4_DIR/.git" ]; then
    git clone \
        https://github.com/PX4/PX4-Autopilot.git \
        "$PX4_DIR"
fi

cd "$PX4_DIR"

git fetch --all --tags
git checkout --force "$PX4_COMMIT"

git submodule sync --recursive
git submodule update --init --recursive

PX4_NOW="$(git rev-parse HEAD)"

[ "$PX4_NOW" = "$PX4_COMMIT" ] || \
    die "PX4 commit 불일치"

echo "PX4: $PX4_NOW"

# --------------------------------------------------
# 10. PX4 개발 의존성
# Gazebo 설치는 금지
# --------------------------------------------------

echo
echo "===== 10. PX4 개발 의존성 ====="

cd "$PX4_DIR"

bash Tools/setup/ubuntu.sh \
    --no-sim-tools \
    --no-nuttx

# 다시 한번 Gazebo 버전 확인
while IFS=$'\t' read -r pkg expected
do
    [ -z "$pkg" ] && continue

    clean_pkg="${pkg%%:*}"
    actual="$(dpkg-query -W -f='${Version}' "$clean_pkg" 2>/dev/null || true)"

    [ "$actual" = "$expected" ] || \
        die "PX4 setup 이후 Gazebo 버전이 변경되었습니다: $clean_pkg"

done < "$HARMONIC_MANIFEST"

# --------------------------------------------------
# 11. 사용자 PX4 코드
# --------------------------------------------------

echo
echo "===== 11. 사용자 PX4 코드 적용 ====="

[ -d "$CUSTOM_PX4_DIR/New_code" ] || \
    die "custom_px4/New_code 없음"

[ -d "$CUSTOM_PX4_DIR/navigation" ] || \
    die "custom_px4/navigation 없음"

[ -d "$CUSTOM_PX4_DIR/Tools/simulation/gz" ] || \
    die "custom_px4/Tools/simulation/gz 없음"

rm -rf "$PX4_DIR/New_code"
rm -rf "$PX4_DIR/navigation"
rm -rf "$PX4_DIR/Tools/simulation/gz"

cp -a "$CUSTOM_PX4_DIR/New_code" \
      "$PX4_DIR/"

cp -a "$CUSTOM_PX4_DIR/navigation" \
      "$PX4_DIR/"

mkdir -p "$PX4_DIR/Tools/simulation"

cp -a "$CUSTOM_PX4_DIR/Tools/simulation/gz" \
      "$PX4_DIR/Tools/simulation/"

# --------------------------------------------------
# 12. MicroXRCE-DDS-Agent
# --------------------------------------------------

echo
echo "===== 12. MicroXRCE-DDS-Agent ====="

if [ -d "$MICROXRCE_DIR" ] && \
   [ ! -d "$MICROXRCE_DIR/.git" ]
then
    rm -rf "$MICROXRCE_DIR"
fi

if [ ! -d "$MICROXRCE_DIR/.git" ]; then
    git clone \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        "$MICROXRCE_DIR"
fi

cd "$MICROXRCE_DIR"

git fetch --all --tags
git checkout --force "$MICROXRCE_COMMIT"

git submodule sync --recursive
git submodule update --init --recursive

MICRO_NOW="$(git rev-parse HEAD)"

[ "$MICRO_NOW" = "$MICROXRCE_COMMIT" ] || \
    die "MicroXRCE commit 불일치"

rm -rf build
mkdir build
cd build

cmake ..
make -j"$(nproc)"

[ -x "$MICROXRCE_DIR/build/MicroXRCEAgent" ] || \
    die "MicroXRCEAgent 빌드 실패"

# --------------------------------------------------
# 13. ros_gz exact commit
# --------------------------------------------------

echo
echo "===== 13. ros_gz exact commit ====="

mkdir -p "$GZ_WS/src"

if [ -d "$GZ_WS/src/ros_gz" ] && \
   [ ! -d "$GZ_WS/src/ros_gz/.git" ]
then
    rm -rf "$GZ_WS/src/ros_gz"
fi

if [ ! -d "$GZ_WS/src/ros_gz/.git" ]; then
    git clone \
        https://github.com/gazebosim/ros_gz.git \
        "$GZ_WS/src/ros_gz"
fi

cd "$GZ_WS/src/ros_gz"

git fetch --all --tags
git checkout --force "$ROS_GZ_COMMIT"

ROS_GZ_NOW="$(git rev-parse HEAD)"

[ "$ROS_GZ_NOW" = "$ROS_GZ_COMMIT" ] || \
    die "ros_gz commit 불일치"

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

# rosdep 이후 Harmonic 재검증
while IFS=$'\t' read -r pkg expected
do
    [ -z "$pkg" ] && continue

    clean_pkg="${pkg%%:*}"
    actual="$(dpkg-query -W -f='${Version}' "$clean_pkg" 2>/dev/null || true)"

    [ "$actual" = "$expected" ] || \
        die "rosdep 이후 Gazebo 버전 변경: $clean_pkg"

done < "$HARMONIC_MANIFEST"

rm -rf build install log

colcon build

[ -f "$GZ_WS/install/setup.bash" ] || \
    die "ros_gz 빌드 실패"

# --------------------------------------------------
# 14. px4_msgs
# --------------------------------------------------

echo
echo "===== 14. px4_msgs exact commit ====="

mkdir -p "$PX4_MSGS_WS/src"

if [ -d "$PX4_MSGS_WS/src/px4_msgs" ] && \
   [ ! -d "$PX4_MSGS_WS/src/px4_msgs/.git" ]
then
    rm -rf "$PX4_MSGS_WS/src/px4_msgs"
fi

if [ ! -d "$PX4_MSGS_WS/src/px4_msgs/.git" ]; then
    git clone \
        https://github.com/PX4/px4_msgs.git \
        "$PX4_MSGS_WS/src/px4_msgs"
fi

cd "$PX4_MSGS_WS/src/px4_msgs"

git fetch --all --tags
git checkout --force "$PX4_MSGS_COMMIT"

PX4_MSGS_NOW="$(git rev-parse HEAD)"

[ "$PX4_MSGS_NOW" = "$PX4_MSGS_COMMIT" ] || \
    die "px4_msgs commit 불일치"

cd "$PX4_MSGS_WS"

source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build

[ -f "$PX4_MSGS_WS/install/setup.bash" ] || \
    die "px4_msgs 빌드 실패"

# --------------------------------------------------
# 15. Python
# --------------------------------------------------

echo
echo "===== 15. Python 환경 ====="

python3 -m pip install --user \
    numpy==1.26.4 \
    opencv-contrib-python==4.8.1.78 \
    pandas==2.3.3

sudo apt install -y \
    python3-scipy \
    python3-yaml

# --------------------------------------------------
# 16. Python exact version 확인
# --------------------------------------------------

echo
echo "===== 16. Python 버전 검증 ====="

python3 - <<'PY'
import sys
import importlib.metadata
import numpy
import cv2
import pandas
import yaml
import scipy
import rclpy

expected = {
    "Python": "3.10.12",
    "numpy": "1.26.4",
    "opencv": "4.8.1",
    "pandas": "2.3.3",
    "PyYAML": "5.4.1",
    "scipy": "1.8.0",
    "rclpy": "3.3.21",
}

actual = {
    "Python": ".".join(map(str, sys.version_info[:3])),
    "numpy": numpy.__version__,
    "opencv": cv2.__version__,
    "pandas": pandas.__version__,
    "PyYAML": yaml.__version__,
    "scipy": scipy.__version__,
    "rclpy": importlib.metadata.version("rclpy"),
}

failed = False

for key in expected:
    print(f"{key:10s}: {actual[key]}")

    if actual[key] != expected[key]:
        print(
            f"ERROR: {key} "
            f"expected={expected[key]} "
            f"actual={actual[key]}"
        )
        failed = True

if failed:
    raise SystemExit(1)
PY

[ "$?" -eq 0 ] || \
    die "Python 버전 불일치"

# --------------------------------------------------
# 17. 전체 Git commit 최종 검증
# --------------------------------------------------

echo
echo "===== 17. Git commit 최종 검증 ====="

PX4_NOW="$(git -C "$PX4_DIR" rev-parse HEAD)"
PX4_MSGS_NOW="$(git -C "$PX4_MSGS_WS/src/px4_msgs" rev-parse HEAD)"
ROS_GZ_NOW="$(git -C "$GZ_WS/src/ros_gz" rev-parse HEAD)"
MICRO_NOW="$(git -C "$MICROXRCE_DIR" rev-parse HEAD)"

echo "PX4       : $PX4_NOW"
echo "px4_msgs  : $PX4_MSGS_NOW"
echo "ros_gz    : $ROS_GZ_NOW"
echo "MicroXRCE : $MICRO_NOW"

[ "$PX4_NOW" = "$PX4_COMMIT" ] || \
    die "PX4 최종 commit 불일치"

[ "$PX4_MSGS_NOW" = "$PX4_MSGS_COMMIT" ] || \
    die "px4_msgs 최종 commit 불일치"

[ "$ROS_GZ_NOW" = "$ROS_GZ_COMMIT" ] || \
    die "ros_gz 최종 commit 불일치"

[ "$MICRO_NOW" = "$MICROXRCE_COMMIT" ] || \
    die "MicroXRCE 최종 commit 불일치"

# --------------------------------------------------
# 18. 최종 실행 검사
# --------------------------------------------------

echo
echo "===== 18. 최종 실행 환경 검사 ====="

source /opt/ros/humble/setup.bash
source "$GZ_WS/install/setup.bash"
source "$PX4_MSGS_WS/install/setup.bash"

GZ_VERSION="$(gz sim --version | head -1)"

echo "$GZ_VERSION"

echo "$GZ_VERSION" | grep -q "8.12.0" || \
    die "Gazebo Sim 8.12.0이 아닙니다."

ros2 pkg prefix ros_gz_bridge >/dev/null || \
    die "ros_gz_bridge 없음"

ros2 pkg prefix ros_gz_image >/dev/null || \
    die "ros_gz_image 없음"

[ -x "$MICROXRCE_DIR/build/MicroXRCEAgent" ] || \
    die "MicroXRCEAgent 없음"

[ -f "$COM1_DIR/run7.sh" ] || \
    die "run7.sh 없음"

chmod +x "$COM1_DIR/run7.sh"

echo
echo "========================================"
echo " SETUP SUCCESS"
echo "========================================"
echo
echo "Gazebo Harmonic : 8.12.0"
echo "PX4             : $PX4_COMMIT"
echo "px4_msgs        : $PX4_MSGS_COMMIT"
echo "ros_gz          : $ROS_GZ_COMMIT"
echo "MicroXRCE       : $MICROXRCE_COMMIT"
echo
echo "실행:"
echo "cd $COM1_DIR"
echo "./run7.sh"
echo
