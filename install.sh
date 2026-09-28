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

# systemd 서비스와 동일한 DDS 도메인 사용 (미설정 시 토픽이 안 보임)
if ! grep -q 'ROS_DOMAIN_ID' ~/.bashrc; then
    echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
fi
export ROS_DOMAIN_ID=0
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

# CH340(FC 젠더)이 brltty 에 가로채이지 않도록 제외 규칙 추가
sudo tee /etc/udev/rules.d/85-anomaly-serial.rules > /dev/null << 'UDEVEOF'
# CH340 (FC USB-TTL 젠더) — brltty 가 점자 장치로 오인하지 않도록 제외
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{BRLTTY_BRAILLE_DRIVER}="", ENV{BRLTTY_NO_DRIVER}="1"
UDEVEOF
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
# 현재 alias + 이전 버전에서 쓰던 alias 를 모두 지우고 다시 등록합니다
for a in start_drone stop_drone monitor_drone fc_status onboard_log service_status \
         detect_serial onboard_env record_drone verify_bag bag_log analyze_drone extract_audio update_drone \
         check_topics check_usb check_record watch_fcu monitor_fast monitor_only monitor_sh \
         detect_fc scan_bags fix_devices setup_fc scan_baud; do
    sed -i "/^alias ${a}=/d" ~/.bashrc
done
sed -i '/^#   \(실행\|상태\|장치\|녹화·분석\) /d' ~/.bashrc
sed -i '/^# 드론 센서 편의 명령어$/d' ~/.bashrc

cat >> ~/.bashrc << ALIAS

# 드론 센서 편의 명령어
#   실행      start_drone / stop_drone
#   상태      monitor_drone / fc_status / onboard_log / service_status
#   장치      detect_serial / onboard_env
#   녹화·분석 record_drone / verify_bag / bag_log / analyze_drone / extract_audio
#   배포      update_drone
alias start_drone='$WS/scripts/guard_service.sh && $WS/scripts/check_time_sync.sh; pkill -f mavros_node 2>/dev/null; sleep 1; ros2 launch drone_sensors drone_sensor_launch.py'
alias stop_drone='pkill -INT -f drone_sensor_launch 2>/dev/null; sleep 5; pkill -f mavros_node 2>/dev/null; true'
alias monitor_drone='python3 $WS/scripts/monitor_node.py'
alias fc_status='bash $WS/scripts/watch_fcu.sh --once'
alias onboard_log='tail -f \$HOME/anomaly_data/onboard.log'
alias service_status='bash $WS/scripts/install_service.sh status'
alias detect_serial='python3 $WS/scripts/serial_autodetect.py'
alias onboard_env='bash $WS/scripts/setup_onboard_env.sh'
alias record_drone='$WS/scripts/record_data.sh'
alias verify_bag='python3 $WS/scripts/verify_bag.py'
alias bag_log='bash $WS/scripts/extract_bag_log.sh'
alias analyze_drone='python3 $WS/scripts/analyze_bag.py'
alias extract_audio='python3 $WS/scripts/extract_audio.py'
alias update_drone='bash $WS/scripts/update.sh'
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
echo "  [실행]"
echo "    start_drone              전체 실행 (수동, 자동녹화 없음)"
echo "    stop_drone               전체 종료"
echo "  [상태]"
echo "    monitor_drone            실시간 모니터 (--interval 1 / --no-log / --once)"
echo "    fc_status                FC 관리 상태 (포트·SYSID·재시작)"
echo "    onboard_log              서비스 실행 로그"
echo "    service_status           부팅 자동실행 여부"
echo "  [장치]"
echo "    detect_serial            FC/THL100/WCM6800 판별 (FC baud·SYSID 포함)"
echo "    onboard_env check        온보드 환경 점검"
echo "  [녹화·분석]"
echo "    record_drone 30          30초 수동 녹화"
echo "    verify_bag [bag|--all]   녹화 검증 (드론 판정 + 외부센서 표기)"
echo "    bag_log <bag>            해당 bag 전후 로그"
echo "    analyze_drone <bag>      CSV + 그래프"
echo "    extract_audio <bag>      마이크 PCM → WAV"
echo "  [배포]"
echo "    update_drone             GitHub 최신 반영 → 빌드 → 테스트 → 재시작"
echo ""
echo "  온보드(무인) 운용:"
echo "    bash scripts/setup_onboard_env.sh        # 최초 1회 (brltty 제거 등)"
echo "    bash scripts/install_service.sh          # 서비스 등록 (자동실행 OFF)"
echo "    bash scripts/install_service.sh enable   # 부팅 자동실행 ON"
echo "=========================================="
