#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 드론 센서 데이터 수집 환경 자동 설치 스크립트
# 사용법: bash install.sh
# ══════════════════════════════════════════════════════════════════════════════

set -e

ROS_DISTRO=humble
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # 스크립트 위치 = 워크스페이스

echo "=========================================="
echo " 드론 센서 환경 설치 시작"
echo " 워크스페이스: $WS"
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
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-mavros-msgs
echo "✅ 시스템 의존성 설치 완료"

# ── 3. pip 업그레이드 및 PATH 설정 ───────────────────────────────────────
echo "[3/7] pip 업그레이드..."
pip3 install --upgrade pip -q
export PATH=$HOME/.local/bin:$PATH
if ! grep -q 'local/bin' ~/.bashrc; then
    echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
fi
echo "✅ pip 업그레이드 완료"

# ── 4. Python 의존성 설치 ────────────────────────────────────────────────
echo "[4/7] Python 의존성 설치..."
pip3 install \
    pyusb \
    pyaudio \
    "numpy<2" \
    pyserial \
    pandas \
    matplotlib \
    -q
echo "✅ Python 의존성 설치 완료"

# ── 5. GeographicLib 데이터 설치 ─────────────────────────────────────────
echo "[5/7] GeographicLib 데이터 설치..."
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
echo "✅ GeographicLib 설치 완료"

# ── 6. udev 규칙 설정 ────────────────────────────────────────────────────
echo "[6/7] udev 규칙 설정..."
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/60-respeaker.rules > /dev/null
sudo usermod -aG dialout $USER
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✅ udev 규칙 설정 완료"

# ── 7. ROS2 워크스페이스 빌드 ────────────────────────────────────────────
echo "[7/7] ROS2 워크스페이스 빌드..."

# launch/__init__.py 제거 (ROS2 launch 모듈 충돌 방지)
rm -f $WS/src/drone_sensors/launch/__init__.py

# 이전 빌드 잔여 파일 제거 (충돌 방지)
rm -rf $WS/build $WS/install $WS/log

cd $WS
colcon build --symlink-install
echo "✅ 빌드 완료"

# ── bashrc 설정 ───────────────────────────────────────────────────────────
echo "bashrc 설정 중..."

if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source $WS/install/setup.bash" ~/.bashrc; then
    echo "source $WS/install/setup.bash" >> ~/.bashrc
fi

if ! grep -q "alias start_drone" ~/.bashrc; then
    cat >> ~/.bashrc << ALIAS

# 드론 센서 편의 명령어
alias start_drone='pkill -f mavros_node 2>/dev/null; sleep 1; ros2 launch drone_sensors drone_sensor_launch.py'
alias stop_drone='pkill -f mavros_node 2>/dev/null; pkill -f drone_sensor_launch 2>/dev/null'
alias check_topics='ros2 topic list | grep -E "drone|mavros|respeaker|thl100|wcm6800"'
alias check_usb='ls -la /dev/serial/by-id/'
alias record_drone='$WS/scripts/record_data.sh'
alias analyze_drone='python3 $WS/scripts/analyze_bag.py'
ALIAS
fi

source ~/.bashrc 2>/dev/null || true
echo "✅ bashrc 설정 완료"

echo ""
echo "=========================================="
echo " 설치 완료!"
echo "=========================================="
echo ""
echo "사용 방법:"
echo "  ⚠️  로그아웃 후 재로그인 필요 (dialout 그룹 적용)"
echo ""
echo "  start_drone              — 전체 센서 실행"
echo "  stop_drone               — 전체 센서 종료"
echo "  check_topics             — 토픽 목록 확인"
echo "  check_usb                — USB 장치 확인"
echo "  record_drone 30          — 30초 데이터 녹화"
echo "  analyze_drone <bag경로>  — 데이터 분석 (CSV + 그래프)"
echo ""
echo "예시:"
echo "  record_drone 60"
echo "  analyze_drone ~/anomaly_data/anomaly_data_20260616_112216"
echo "=========================================="
