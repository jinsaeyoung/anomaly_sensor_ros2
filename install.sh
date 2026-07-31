#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 드론 센서 데이터 수집 환경 자동 설치 스크립트
# 사용법: bash install.sh
# ══════════════════════════════════════════════════════════════════════════════

set -e

ROS_DISTRO=humble
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo " 드론 센서 환경 설치 시작"
echo " 워크스페이스: $WS"
echo "=========================================="

# ── 1. ROS2 환경 확인 ─────────────────────────────────────────────────────
echo "[1/8] ROS2 환경 확인..."
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "ERROR: ROS2 $ROS_DISTRO 가 설치되어 있지 않습니다."
    echo "https://docs.ros.org/en/humble/Installation.html 를 참고하세요."
    exit 1
fi
source /opt/ros/$ROS_DISTRO/setup.bash
echo "✅ ROS2 $ROS_DISTRO 확인 완료"

# ── 2. 시스템 의존성 설치 ────────────────────────────────────────────────
echo "[2/8] 시스템 의존성 설치..."
sudo apt update -qq
sudo apt install -y \
    python3-pip \
    python3-pyaudio \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-diagnostic-msgs
echo "✅ 시스템 의존성 설치 완료"

# ── 3. pip 업그레이드 및 PATH 설정 ───────────────────────────────────────
echo "[3/8] pip 업그레이드..."
pip3 install --upgrade pip -q
export PATH=$HOME/.local/bin:$PATH
if ! grep -q 'local/bin' ~/.bashrc; then
    echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
fi
echo "✅ pip 업그레이드 완료"

# ── 4. Python 의존성 설치 ────────────────────────────────────────────────
echo "[4/8] Python 의존성 설치..."
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
echo "[5/8] GeographicLib 데이터 설치..."
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
echo "✅ GeographicLib 설치 완료"

# ── 6. udev 규칙 설정 ────────────────────────────────────────────────────
echo "[6/8] udev 규칙 설정..."
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/60-respeaker.rules > /dev/null
sudo usermod -aG dialout $USER
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✅ udev 규칙 설정 완료"

# ── 7. ROS2 패키지 구조 표준화 ───────────────────────────────────────────
echo "[7/8] ROS2 패키지 구조 표준화..."
if [ -f "$WS/fix_packaging.sh" ]; then
    ANOMALY_WS="$WS" bash "$WS/fix_packaging.sh" > /dev/null 2>&1 || true
    echo "✅ 패키지 구조 표준화 완료 (resource 마커, setup.py data_files)"
else
    # fix_packaging.sh 가 없을 때 최소 조치
    rm -f "$WS/src/drone_sensors/launch/__init__.py"
    echo "⚠️  fix_packaging.sh 없음 — 최소 조치만 수행"
fi

# ── 8. ROS2 워크스페이스 빌드 ────────────────────────────────────────────
echo "[8/8] ROS2 워크스페이스 빌드..."

# 이전 빌드 잔여 파일 제거 (충돌 방지)
rm -rf "$WS/build" "$WS/install" "$WS/log"
# 하위 패키지 내부에 잘못 생긴 빌드 산출물도 정리
find "$WS/src" -maxdepth 2 -type d \( -name build -o -name install -o -name log \) \
    -exec rm -rf {} + 2>/dev/null || true

cd "$WS"
colcon build --symlink-install
echo "✅ 빌드 완료"

# 스크립트 실행 권한 부여
chmod +x "$WS"/scripts/*.sh 2>/dev/null || true
chmod +x "$WS"/fix_packaging.sh 2>/dev/null || true

# ── bashrc 설정 ───────────────────────────────────────────────────────────
echo "bashrc 설정 중..."

# 예전 워크스페이스(ros2_ws) 잔재 제거
sed -i '/ros2_ws\/install\/setup.bash/d' ~/.bashrc 2>/dev/null || true

if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source $WS/install/setup.bash" ~/.bashrc; then
    echo "source $WS/install/setup.bash" >> ~/.bashrc
fi

# 기존 alias 제거 후 재등록 (재실행 시 중복/구버전 방지)
for a in start_drone stop_drone check_topics check_usb record_drone analyze_drone check_record onboard_log service_status setup_fc scan_baud; do
    sed -i "/^alias ${a}=/d" ~/.bashrc
done
sed -i '/^# 드론 센서 편의 명령어$/d' ~/.bashrc

cat >> ~/.bashrc << ALIAS

# 드론 센서 편의 명령어
alias start_drone='$WS/scripts/guard_service.sh && $WS/scripts/check_time_sync.sh; pkill -f mavros_node 2>/dev/null; sleep 1; ros2 launch drone_sensors drone_sensor_launch.py'
alias stop_drone='pkill -f mavros_node 2>/dev/null; pkill -f drone_sensor_launch 2>/dev/null'
alias check_topics='ros2 topic list | grep -E "mavros|respeaker|thl100|wcm6800"'
alias check_usb='ls -la /dev/serial/by-id/'
alias record_drone='$WS/scripts/record_data.sh'
alias service_status='bash $WS/scripts/install_service.sh status'
alias setup_fc='bash $WS/scripts/setup_fc_streams.sh'
alias scan_baud='bash $WS/scripts/scan_fcu_baud.sh'
alias analyze_drone='python3 $WS/scripts/analyze_bag.py'
alias check_record='bash $WS/scripts/check_record.sh'
alias onboard_log='tail -f \$HOME/anomaly_data/onboard.log'
ALIAS

source ~/.bashrc 2>/dev/null || true
echo "✅ bashrc 설정 완료"

echo ""
echo "=========================================="
echo " 설치 완료!"
echo "=========================================="
echo ""
echo "  ⚠️  로그아웃 후 재로그인 필요 (dialout 그룹 적용)"
echo ""
echo "  start_drone              — 시간 동기화 확인 후 전체 센서 실행"
echo "  stop_drone               — 전체 센서 종료"
echo "  check_topics             — 토픽 목록 확인"
echo "  check_usb                — USB 장치 확인"
echo "  record_drone 30          — 30초 데이터 녹화"
echo "  analyze_drone <bag경로>  — 데이터 분석 (CSV + 그래프)"
echo "  check_record             — 자동 녹화/서비스 상태 확인"
echo "  onboard_log              — 온보드 실행 로그 실시간 확인"
echo ""
echo "  service_status           — 부팅 자동실행 모드 확인"
echo "  setup_fc [0|1|2]         — FC 스트림(SR) 파라미터 설정 (기본 SR2/TELEM2)"
echo "  setup_fc check           — 현재 SR/SERIAL 파라미터 조회"
echo "  scan_baud <포트>         — FC baud rate 탐색"
echo ""
echo "  온보드 자동 실행:"
echo "    bash scripts/install_service.sh          # 등록만 (개발 모드)"
echo "    bash scripts/install_service.sh enable   # 부팅 자동실행 ON (운용)"
echo "    bash scripts/install_service.sh disable  # 개발 모드로 복귀"
echo ""
echo "  FC 포트 변경 시:"
echo "    ros2 launch drone_sensors drone_sensor_launch.py fcu_url:=/dev/ttyACM0:115200"
echo "=========================================="
