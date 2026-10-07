#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 드론 센서 데이터 수집 환경 자동 설치 스크립트
# 사용법: bash install.sh
# ══════════════════════════════════════════════════════════════════════════════

set -e

ROS_DISTRO=humble
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# root 로 실행하면 빌드 결과·데이터 폴더가 root 소유가 되고 alias 가 root 의
# .bashrc 에 들어가, 이후 일반 사용자로 쓸 때 계속 권한 문제가 생깁니다.
if [ "$(id -u)" = "0" ]; then
    echo "❌ sudo 없이 실행하세요:  bash install.sh"
    echo "   필요한 단계에서만 sudo 비밀번호를 묻습니다."
    exit 1
fi

# 예전에 sudo 로 실행했던 흔적(root 소유 파일)이 있으면 빌드가 실패하므로 먼저 정리
if [ -n "$(find "$WS" -xdev ! -user "$(id -un)" -print -quit 2>/dev/null)" ]; then
    echo "  root 소유 파일 정리 (이전 sudo 실행 흔적)..."
    sudo chown -R "$(id -un):$(id -gn)" "$WS"
fi

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

# ── ROS2 apt 저장소 확인 ─────────────────────────────────────────────
# ROS2 는 설치돼 있어도 apt 저장소가 없거나(소스 설치 등) 예전 방식의 키가
# 만료되면 ros-humble-mavros 를 찾지 못합니다.
# 공식 문서의 현재 방식(ros2-apt-source 패키지)으로 저장소를 설정합니다.
ros_pkg_available() {
    apt-cache policy "ros-$ROS_DISTRO-mavros" 2>/dev/null | grep -q "Candidate: [0-9]"
}

APT_ETC="${APT_ETC:-/etc/apt}"

apt_update_verbose() {
    # 스크립트에서는 apt 대신 apt-get 을 씁니다 (apt 는 CLI 가 고정되지 않았다는 경고를 냄).
    # -qq 로 숨기면 키 만료·중복 등록 오류가 보이지 않으므로 오류·경고 줄만 보여줍니다.
    sudo apt-get update 2>&1 | grep -E "^(E|W|Err):|NO_PUBKEY|EXPKEYSIG|Conflicting values" \
        | sed 's/^/    /' || true
}

ros_source_entries() {
    # packages.ros.org/ros2 를 가리키는 모든 저장소 항목 (파일:줄)
    # (apt 는 .list/.sources 로 끝나는 파일만 읽으므로 비활성화한 파일은 제외)
    grep -rsn "packages.ros.org/ros2" "$APT_ETC/sources.list" "$APT_ETC/sources.list.d/" 2>/dev/null \
        | grep -v ':#' | grep -v 'disabled-by-anomaly' || true
}

disable_duplicate_ros_sources() {
    # 같은 ROS 저장소가 다른 키로 두 번 등록되면 apt 가
    # 'Conflicting values set for option Signed-By' 로 저장소 설정 전체를 읽지 못합니다.
    # 공식 설정(ros2.sources) 외의 항목은 지우지 않고 비활성화해 보관합니다.
    local f
    for f in "$APT_ETC"/sources.list.d/*; do
        [ -f "$f" ] || continue
        case "$(basename "$f")" in ros2.sources|*.disabled-by-anomaly) continue ;; esac
        if grep -qs "packages.ros.org/ros2" "$f"; then
            sudo mv "$f" "$f.disabled-by-anomaly"
            echo "  - 중복 ROS 저장소 비활성화: $(basename "$f") → $(basename "$f").disabled-by-anomaly"
        fi
    done
    if grep -qsE "^[[:space:]]*deb.*packages.ros.org/ros2" "$APT_ETC/sources.list"; then
        sudo sed -i.bak-anomaly -E 's|^([[:space:]]*deb.*packages.ros.org/ros2.*)$|# (anomaly install: 중복 비활성화) \1|' \
            "$APT_ETC/sources.list"
        echo "  - sources.list 의 중복 ROS 항목 주석 처리 (백업: sources.list.bak-anomaly)"
    fi
}

setup_ros_apt_source() {
    echo "  ROS2 apt 저장소를 설정합니다 (공식 방식: ros2-apt-source)"
    # 예전 방식의 목록·키가 남아 있으면 같은 저장소가 다른 키로 두 번 등록돼 충돌합니다
    sudo rm -f "$APT_ETC/sources.list.d/ros2.list" "$APT_ETC/sources.list.d/ros2-latest.list"
    sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
    disable_duplicate_ros_sources
    sudo apt-get install -y curl software-properties-common > /dev/null
    sudo add-apt-repository -y universe > /dev/null 2>&1 || true

    local ver codename
    ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    if [ -z "$ver" ]; then
        # GitHub API 호출 제한 등으로 실패하면 최신 릴리스 주소에서 태그를 얻습니다
        ver=$(curl -sI https://github.com/ros-infrastructure/ros-apt-source/releases/latest \
              | grep -i '^location:' | sed 's#.*/tag/##' | tr -d '\r')
    fi
    [ -z "$ver" ] && { echo "  ❌ ros-apt-source 버전을 확인할 수 없습니다 (인터넷 연결 확인)"; exit 1; }

    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb" \
        || { echo "  ❌ ros2-apt-source 다운로드 실패"; exit 1; }
    sudo dpkg -i /tmp/ros2-apt-source.deb > /dev/null
    echo "  ✓ ros2-apt-source ${ver} 설치"
    apt_update_verbose
}

diagnose_ros_apt() {
    # 저장소를 설정해도 못 찾을 때 원인을 바로 알 수 있게 진단을 출력합니다
    local arch foreign cpu
    arch=$(dpkg --print-architecture)
    foreign=$(dpkg --print-foreign-architectures | tr '\n' ' ')
    cpu=$(uname -m)
    echo ""
    echo "  ❌ 저장소를 설정했지만 ROS 패키지(ros-$ROS_DISTRO-rclpy)를 찾을 수 없습니다. 진단:"
    echo "     CPU: $cpu / 설치된 OS(dpkg): $arch / 추가 아키텍처: ${foreign:-없음}"
    echo "     등록된 ROS 저장소:"
    ros_source_entries | sed 's/^/       /'
    [ -z "$(ros_source_entries)" ] && echo "       (없음)"
    local errs
    errs=$(sudo apt-get update 2>&1 | grep -E "^(E|W|Err):" | grep -i "ros\|Signed-By\|Conflicting" | head -n 5)
    if [ -n "$errs" ]; then
        echo "     apt 오류:"
        echo "$errs" | sed 's/^/       /'
    fi
    if echo "$errs" | grep -q "Conflicting values"; then
        echo ""
        echo "     → 원인: 같은 ROS 저장소가 서로 다른 키로 두 번 등록돼 있습니다."
        echo "       위 '등록된 ROS 저장소' 중 ros2.sources 외의 항목을 지우고 다시 실행하세요."
        return
    fi
    if echo "$errs" | grep -qiE "Could not resolve|Failed to fetch|Temporary failure|Connection"; then
        echo ""
        echo "     → 원인: packages.ros.org 에 접속하지 못했습니다 (인터넷·프록시·방화벽 확인)."
        return
    fi
    case "$arch" in
        amd64|arm64) ;;
        *)
            echo ""
            echo "     → 원인: ${arch} 는 32비트 OS 입니다. ROS2 $ROS_DISTRO 는 amd64/arm64 패키지만 제공합니다."
            [ "$cpu" = "x86_64" ] && echo "       CPU 는 64비트이므로 64비트(amd64) Ubuntu 22.04 로 재설치하면 됩니다."
            [ "$cpu" = "aarch64" ] && echo "       CPU 는 64비트이므로 64비트(arm64) Ubuntu 22.04 로 재설치하면 됩니다."
            return ;;
    esac
    if ! ls /var/lib/apt/lists/ 2>/dev/null | grep -q "packages.ros.org.*binary-${arch}_Packages"; then
        echo "     → 원인: ROS 저장소의 ${arch} 패키지 목록을 받지 않았습니다."
        echo "       확인: grep -r Architectures /etc/apt/apt.conf.d/ /etc/apt/sources.list.d/"
        return
    fi
    if ! apt-cache policy "ros-$ROS_DISTRO-rclpy" 2>/dev/null | grep -q "Candidate: [0-9]"; then
        echo "     → 원인: ROS 패키지 목록이 손상된 것으로 보입니다. 다시 받으세요:"
        echo "       sudo rm -f /var/lib/apt/lists/packages.ros.org_* && sudo apt update"
        return
    fi
    echo "     → 다른 ROS 패키지는 있는데 mavros 만 없습니다. 저장소 상태를 확인하거나 소스 빌드가 필요합니다."
}

ros_repo_ok() {
    # mavros 대신 항상 있는 패키지로 저장소 동작 여부를 판단합니다
    apt-cache policy "ros-$ROS_DISTRO-rclpy" 2>/dev/null | grep -q "Candidate: [0-9]"
}

disable_duplicate_ros_sources_if_official() {
    # 공식 설정이 이미 있다면, 다른 이름으로 남은 중복 항목이 문제일 수 있습니다
    [ -f "$APT_ETC/sources.list.d/ros2.sources" ] && disable_duplicate_ros_sources
}

disable_duplicate_ros_sources_if_official
apt_update_verbose
if ! ros_repo_ok; then
    echo "  ⚠ ROS2 apt 저장소에서 패키지를 찾을 수 없습니다"
    setup_ros_apt_source
    if ! ros_repo_ok; then
        # 손상되거나 오래된 ROS 목록이 남아 있을 수 있어 한 번 지우고 다시 받습니다
        echo "  ROS 패키지 목록을 다시 받습니다..."
        sudo rm -f /var/lib/apt/lists/packages.ros.org_*
        apt_update_verbose
    fi
    if ! ros_repo_ok; then
        diagnose_ros_apt
        exit 1
    fi
    echo "  ✅ ROS2 apt 저장소 설정 완료"
fi

sudo apt install -y \
    python3-pip \
    python3-pyaudio \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-diagnostic-msgs

# ── mavros ────────────────────────────────────────────────────────────
# 2026-09 현재 Humble(Jammy) 저장소에서 mavros·mavros_extras·libmavconn 이
# 빠지고 mavros_msgs(2.15.1)만 남아 있습니다 (mavlink/mavros#2293).
# 이미 설치된 모듈은 영향이 없지만 새 모듈은 apt 로 설치할 수 없으므로,
# 저장소에 없으면 ROS 공식 스냅샷(snapshots.ros.org)에서 설치합니다.
MAVROS_PKGS="ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras ros-$ROS_DISTRO-mavros-msgs ros-$ROS_DISTRO-libmavconn"
MAVROS_SNAPSHOT="${MAVROS_SNAPSHOT:-2026-08-07}"     # mavros 가 마지막으로 있던 스냅샷

install_mavros_from_snapshot() {
    local list=/etc/apt/sources.list.d/ros2-mavros-snapshot.list
    echo "  mavros 를 ROS 스냅샷($MAVROS_SNAPSHOT)에서 설치합니다"
    echo "deb [trusted=yes] http://snapshots.ros.org/$ROS_DISTRO/$MAVROS_SNAPSHOT/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee "$list" > /dev/null
    sudo apt update -qq 2>/dev/null || true

    # 네 패키지를 모두 스냅샷의 같은 버전으로 맞춥니다.
    # mavros 만 옛 버전이고 mavros_msgs 가 최신이면 짝이 맞지 않아 실행 중 문제가 생길 수 있습니다.
    local args=() pkg ver
    for pkg in $MAVROS_PKGS; do
        ver=$(apt-cache madison "$pkg" 2>/dev/null | grep "snapshots.ros.org" | head -n1 | awk -F'|' '{gsub(/ /,"",$2); print $2}')
        if [ -z "$ver" ]; then
            echo "  ❌ 스냅샷에서 $pkg 를 찾을 수 없습니다 (스냅샷 날짜: $MAVROS_SNAPSHOT)"
            sudo rm -f "$list"; sudo apt update -qq 2>/dev/null || true
            echo "     다른 날짜로 시도: MAVROS_SNAPSHOT=2026-07-xx bash install.sh"
            exit 1
        fi
        args+=("$pkg=$ver")
    done
    sudo apt install -y --allow-downgrades "${args[@]}" || { sudo rm -f "$list"; exit 1; }

    # 이후 apt upgrade 가 mavros_msgs 만 올려 짝이 깨지지 않도록 고정합니다.
    # (저장소가 복구되면: sudo apt-mark unhold $MAVROS_PKGS && sudo apt install --only-upgrade $MAVROS_PKGS)
    sudo apt-mark hold $MAVROS_PKGS > /dev/null
    sudo rm -f "$list"
    sudo apt update -qq 2>/dev/null || true
    echo "  ✅ mavros 설치 및 버전 고정: ${args[*]}"
}

if dpkg -s "ros-$ROS_DISTRO-mavros" > /dev/null 2>&1; then
    echo "  mavros 이미 설치됨 ($(dpkg-query -W -f='${Version}' ros-$ROS_DISTRO-mavros))"
    if ! ros_pkg_available; then
        # 저장소에 mavros 가 없는 동안 apt upgrade 를 하면 mavros_msgs 만 올라가
        # 설치된 mavros 와 짝이 깨질 수 있으므로 현재 버전을 고정합니다.
        sudo apt-mark hold $MAVROS_PKGS > /dev/null 2>&1 || true
        echo "  저장소에 mavros 가 없어 현재 버전을 고정했습니다 (apt upgrade 로 짝이 깨지지 않도록)"
    fi
elif ros_pkg_available; then
    sudo apt install -y $MAVROS_PKGS
else
    echo "  ⚠ 저장소에 ros-$ROS_DISTRO-mavros 가 없습니다 (mavlink/mavros#2293)"
    install_mavros_from_snapshot
fi

# GeographicLib 데이터 — mavros 의 global_position 플러그인이 시작할 때 필요합니다
if [ ! -d /usr/share/GeographicLib/geoids ] && [ -x "/opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh" ]; then
    echo "  GeographicLib 데이터 설치..."
    sudo "/opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh" > /dev/null 2>&1 || true
fi
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
echo "[6/8] 장치 권한 설정..."
# udev 규칙·그룹·소유권을 한 곳(setup_permissions.sh)에서 처리합니다.
# USB 시리얼 장치에 직접 권한을 주므로 재로그인 없이 바로 쓸 수 있습니다.
bash "$WS/scripts/setup_permissions.sh"

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
         detect_serial onboard_env fix_permissions record_drone verify_bag bag_log analyze_drone extract_audio update_drone \
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
#   장치      detect_serial / onboard_env / fix_permissions
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
alias fix_permissions='bash $WS/scripts/setup_permissions.sh'
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
echo "  장치 권한은 바로 적용됐습니다 (재로그인 불필요). 새 명령은: source ~/.bashrc"
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
echo "    fix_permissions          장치 권한 설정·점검 (재로그인 불필요)"
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
