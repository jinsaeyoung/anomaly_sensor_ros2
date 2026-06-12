#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 드론 센서 데이터 수집 환경 자동 설치 스크립트
# 사용법: bash install.sh
# ══════════════════════════════════════════════════════════════════════════════

set -e  # 에러 발생 시 중단

ROS_DISTRO=humble
WS=~/ros2_ws

echo "=========================================="
echo " 드론 센서 환경 설치 시작"
echo "=========================================="

# ── 1. ROS2 환경 확인 ─────────────────────────────────────────────────────
echo "[1/7] ROS2 환경 확인..."
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "ERROR: ROS2 $ROS_DISTRO 가 설치되어 있지 않습니다."
    echo "https://docs.ros.org/en/humble/Installation.html 를 참고하세요."
    exit 1
fi
source /opt/ros/$ROS_DISTRO/setup.bash
echo "✅ ROS2 $ROS_DISTRO 확인 완료"

# ── 2. 시스템 의존성 설치 ────────────────────────────────────────────────
echo "[2/7] 시스템 의존성 설치..."
sudo apt update -qq
sudo apt install -y \
    python3-pip \
    python3-pyaudio \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-mavros-msgs
echo "✅ 시스템 의존성 설치 완료"

# ── 3. Python 의존성 설치 ────────────────────────────────────────────────
echo "[3/7] Python 의존성 설치..."
pip install pyusb pyaudio numpy pyserial --break-system-packages -q
echo "✅ Python 의존성 설치 완료"

# ── 4. GeographicLib 데이터 설치 (mavros 필수) ───────────────────────────
echo "[4/7] GeographicLib 데이터 설치..."
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
echo "✅ GeographicLib 설치 완료"

# ── 5. udev 규칙 설정 (USB 장치 권한) ───────────────────────────────────
echo "[5/7] udev 규칙 설정..."
# ReSpeaker
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/60-respeaker.rules > /dev/null
# 시리얼 포트 권한
sudo usermod -aG dialout $USER
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✅ udev 규칙 설정 완료"

# ── 6. ROS2 워크스페이스 빌드 ────────────────────────────────────────────
echo "[6/7] ROS2 워크스페이스 빌드..."
mkdir -p $WS/src
cd $WS

# 현재 스크립트 위치의 패키지들을 src로 복사 (git clone 방식일 경우)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for pkg in respeaker thl100_sensor wcm6800_sensor drone_state drone_sensors; do
    if [ -d "$SCRIPT_DIR/$pkg" ]; then
        cp -r $SCRIPT_DIR/$pkg $WS/src/
        echo "  - $pkg 복사 완료"
    elif [ -d "$WS/src/$pkg" ]; then
        echo "  - $pkg 이미 존재"
    else
        echo "  WARNING: $pkg 패키지를 찾을 수 없습니다"
    fi
done

colcon build --symlink-install
echo "✅ 빌드 완료"

# ── 7. bashrc 설정 ───────────────────────────────────────────────────────
echo "[7/7] bashrc 설정..."

# ROS2 자동 source
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

# workspace 자동 source
if ! grep -q "source $WS/install/setup.bash" ~/.bashrc; then
    echo "source $WS/install/setup.bash" >> ~/.bashrc
fi

# 편의 alias 등록
if ! grep -q "alias start_drone" ~/.bashrc; then
    cat >> ~/.bashrc << 'ALIAS'

# 드론 센서 편의 명령어
alias start_drone='pkill -f mavros_node 2>/dev/null; sleep 1; ros2 launch drone_sensors drone_sensor_launch.py'
alias stop_drone='pkill -f mavros_node; pkill -f drone_sensor_launch'
alias check_topics='ros2 topic list | grep -E "drone|mavros|respeaker|thl100|wcm6800"'
alias check_usb='ls -la /dev/serial/by-id/'
ALIAS
fi

source ~/.bashrc
echo "✅ bashrc 설정 완료"

echo ""
echo "=========================================="
echo " 설치 완료!"
echo "=========================================="
echo ""
echo "사용 방법:"
echo "  새 터미널 열기 (bashrc 적용)"
echo ""
echo "  start_drone     — 전체 센서 실행"
echo "  stop_drone      — 전체 센서 종료"
echo "  check_topics    — 토픽 목록 확인"
echo "  check_usb       — USB 장치 확인"
echo ""
echo "⚠️  로그아웃 후 재로그인 필요 (dialout 그룹 적용)"
echo "=========================================="
