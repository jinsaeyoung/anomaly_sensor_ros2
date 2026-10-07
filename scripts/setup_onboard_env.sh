#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 온보드 환경 일괄 설정 스크립트
#
# 무인 운용에 필요한 시스템 레벨 설정을 한 번에 처리합니다.
#   1. brltty 제거        — CH340 젠더를 점자 장치로 오인해 가로채는 문제 해결
#   2. sudo 예외 정리     — 이전 버전의 비밀번호 생략 설정 제거 (더 이상 불필요)
#   3. ROS_DOMAIN_ID 고정 — 서비스와 셸의 DDS 도메인 불일치 방지
#   4. udev 규칙          — 시리얼/ReSpeaker 접근 권한
#   5. dialout 그룹       — 시리얼 포트 권한
#   6. 시각 보존          — 전원 차단 후 시계가 되돌아가지 않도록
#   6. 설정 검증          — 적용 결과 확인
#
# 사용법:
#   bash scripts/setup_onboard_env.sh          # 전체 적용
#   bash scripts/setup_onboard_env.sh check    # 현재 상태만 확인
# ══════════════════════════════════════════════════════════════════════════════

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"
RUN_USER="$(id -un)"
SERVICE_NAME="anomaly-sensor"
MODE="${1:-apply}"

ok()   { echo "  ✅ $*"; }
warn() { echo "  ⚠️  $*"; }
fail() { echo "  ❌ $*"; }
head() { echo ""; echo "=========================================="; echo " $*"; echo "=========================================="; }

# ══════════════════════════════════════════════════════════════════════════
# 상태 확인
# ══════════════════════════════════════════════════════════════════════════
do_check() {
    head "온보드 환경 상태 확인"

    echo ""
    echo "[1] brltty (CH340 충돌 원인)"
    if dpkg -l 2>/dev/null | grep -q "^ii  brltty "; then
        fail "brltty 설치됨 — CH340 젠더를 가로챌 수 있습니다"
    else
        ok "brltty 미설치"
    fi

    echo ""
    echo "[2] 불필요한 sudo 예외"
    if [ -f /etc/sudoers.d/anomaly-sensor ]; then
        warn "이전 버전의 비밀번호 생략 설정이 남아 있음 — 적용 시 제거됩니다"
    else
        ok "없음 (FC 복구는 관리 노드가 권한 없이 처리)"
    fi

    echo ""
    echo "[3] ROS_DOMAIN_ID"
    if grep -q "ROS_DOMAIN_ID" ~/.bashrc 2>/dev/null; then
        ok ".bashrc 에 등록됨 (현재 셸: [${ROS_DOMAIN_ID:-미설정}])"
    else
        warn "미등록 — 서비스 토픽이 셸에서 안 보일 수 있습니다"
    fi

    echo ""
    echo "[4] dialout 그룹"
    if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx dialout; then
        if groups | tr ' ' '\n' | grep -qx dialout; then
            ok "등록 및 현재 세션 적용됨"
        else
            warn "등록됐으나 현재 세션 미적용 — 재로그인 필요"
        fi
    else
        fail "미등록"
    fi

    echo ""
    echo "[5] udev 규칙"
    [ -f /etc/udev/rules.d/60-respeaker.rules ] && ok "ReSpeaker 규칙 있음" \
        || warn "ReSpeaker 규칙 없음"

    echo ""
    echo "[6] 연결된 시리얼 장치"
    if ls /dev/serial/by-id/* >/dev/null 2>&1; then
        for f in /dev/serial/by-id/*; do
            echo "     $(basename "$f")"
            echo "       → $(readlink -f "$f")"
        done
    else
        warn "시리얼 장치 없음"
    fi

    echo ""
    echo "[7] USB-UART 장치 인식"
    # 어떤 젠더든 쓸 수 있으므로 특정 칩을 요구하지 않습니다.
    # 다만 CH340 이 USB 로는 보이는데 시리얼 노드가 없으면 brltty 충돌입니다.
    n_tty=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l)
    if [ "$n_tty" -gt 0 ]; then
        ok "시리얼 장치 ${n_tty}개 (어떤 장치인지는 detect_serial 로 확인)"
    else
        warn "시리얼 장치 없음 — 젠더 연결 확인"
    fi
    if lsusb | grep -q "1a86:" && ! ls /dev/serial/by-id/ 2>/dev/null | grep -q "1a86"; then
        fail "CH340 은 USB 로 보이나 시리얼 노드 없음 → brltty 충돌 의심"
    fi

    echo ""
    echo "[8] 시각 보존"
    if [ -e /dev/rtc0 ] && sudo -n hwclock -r >/dev/null 2>&1; then
        ok "하드웨어 RTC 사용"
    elif dpkg -l 2>/dev/null | grep -q "^ii  fake-hwclock "; then
        ok "fake-hwclock 사용 (마지막 저장 시각부터 시작)"
    else
        warn "RTC·fake-hwclock 없음 — 전원 차단 후 시각이 과거로 돌아갈 수 있습니다"
    fi

    echo ""
    echo "[9] 로그 파일 권한"
    LOG_FILE="$HOME/anomaly_data/onboard.log"
    if [ -f "$LOG_FILE" ]; then
        owner=$(stat -c '%U' "$LOG_FILE")
        size_mb=$(du -m "$LOG_FILE" 2>/dev/null | cut -f1)
        if [ "$owner" = "$RUN_USER" ]; then
            ok "사용자 소유 (${size_mb}MB)"
        else
            fail "root 소유 — 로그 비우기 불가 (sudo chown $RUN_USER:$RUN_USER $LOG_FILE)"
        fi
    else
        warn "로그 파일 없음 (서비스 미실행)"
    fi

    echo ""
    echo "[10] systemd 서비스"
    if [ -f "/etc/systemd/system/${SERVICE_NAME}.service" ]; then
        echo -n "     실행:        "; systemctl is-active  "$SERVICE_NAME" 2>/dev/null || echo inactive
        echo -n "     부팅 자동실행: "; systemctl is-enabled "$SERVICE_NAME" 2>/dev/null || echo disabled
    else
        warn "서비스 미등록 — bash scripts/install_service.sh 실행 필요"
    fi

    echo ""
    echo "=========================================="
}

if [ "$MODE" = "check" ]; then
    do_check
    exit 0
fi

# ══════════════════════════════════════════════════════════════════════════
# 적용
# ══════════════════════════════════════════════════════════════════════════
head "온보드 환경 설정 시작"
echo " 사용자:       $RUN_USER"
echo " 워크스페이스: $WS"
echo "=========================================="

# ── 1. brltty 제거 ────────────────────────────────────────────────────
head "[1/8] brltty 제거 (CH340 젠더 충돌 해결)"
echo ""
echo " Ubuntu 기본 설치된 brltty(점자 단말기 데몬)가 CH340(1a86:7523)을"
echo " 점자 장치로 오인해 가로채면, ch341 드라이버가 바인딩되지 못해"
echo " /dev/ttyUSB* 노드가 생성되지 않습니다."
echo " (lsusb 에는 보이는데 /dev/ttyUSB* 가 생기지 않는 증상)"
echo ""

if dpkg -l 2>/dev/null | grep -q "^ii  brltty "; then
    sudo systemctl stop    brltty-udev.service 2>/dev/null || true
    sudo systemctl mask    brltty-udev.service 2>/dev/null || true
    sudo systemctl stop    brltty.service      2>/dev/null || true
    sudo systemctl disable brltty.service      2>/dev/null || true
    sudo apt remove -y brltty
    ok "brltty 제거 완료"
    NEED_REPLUG=1
else
    ok "brltty 미설치 (조치 불필요)"
    NEED_REPLUG=0
fi

# ── 2. 이전 버전 sudo 예외 제거 ───────────────────────────────────────────────────────────────────────────────────────
head "[2/8] 이전 버전 sudo 예외 제거"
echo ""
echo " 예전에는 watch_fcu 가 서비스를 재시작하도록 systemctl 을 비밀번호 없이 허용했습니다."
echo " 이제 FC 복구는 fcu_manager_node 가 권한 없이 처리하므로 필요 없습니다."
echo ""
if [ -f /etc/sudoers.d/anomaly-sensor ]; then
    sudo rm -f /etc/sudoers.d/anomaly-sensor
    ok "제거 완료: /etc/sudoers.d/anomaly-sensor"
else
    ok "해당 없음"
fi

# ── 3. ROS_DOMAIN_ID 고정 ─────────────────────────────────────────────
head "[3/8] ROS_DOMAIN_ID 고정"
echo ""
echo " systemd 서비스는 ROS_DOMAIN_ID=0 으로 실행됩니다."
echo " 셸에 값이 없거나 다르면 DDS 도메인이 달라져"
echo " 서비스가 정상 동작해도 monitor_drone 에 아무것도 안 보입니다."
echo ""

if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc 2>/dev/null; then
    echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
    ok ".bashrc 에 추가"
else
    ok "이미 등록됨"
fi
export ROS_DOMAIN_ID=0

# ── 4. udev 규칙 ──────────────────────────────────────────────────────
head "[4/8] 장치 권한 (udev·그룹·소유권)"
bash "$(dirname "${BASH_SOURCE[0]}")/setup_permissions.sh"

head "[5/8] (4단계에 통합됨)"
ok "dialout·audio 그룹은 4단계에서 처리"

head "[6/8] 로그 파일 권한"
echo ""
echo " systemd 의 append: 모드는 파일이 없으면 root 소유로 생성합니다."
echo " 그러면 사용자가 로그를 비울 수 없어 미리 소유권을 맞춰둡니다."
echo ""

mkdir -p "$HOME/anomaly_data"
LOG_FILE="$HOME/anomaly_data/onboard.log"
[ -f "$LOG_FILE" ] || touch "$LOG_FILE"
if [ "$(stat -c '%U' "$LOG_FILE")" != "$RUN_USER" ]; then
    sudo chown "$RUN_USER:$RUN_USER" "$LOG_FILE"
    ok "소유권 수정 완료"
else
    ok "이미 사용자 소유"
fi

# ── 7. 검증 ───────────────────────────────────────────────────────────
head "[7/8] 시각 보존 (RTC / fake-hwclock)"
echo ""
echo " 드론과 전원을 공유하면 매번 강제 종료됩니다."
echo " 배터리 달린 RTC 가 없으면 다음 부팅 때 시계가 과거로 돌아가고,"
echo " bag 이름과 타임스탬프가 뒤엉켜 녹화 순서를 알 수 없게 됩니다."
echo ""

if [ -e /dev/rtc0 ] && sudo hwclock -r >/dev/null 2>&1; then
    ok "하드웨어 RTC 있음 ($(sudo hwclock -r 2>/dev/null | head -1))"
else
    warn "하드웨어 RTC 없음 — fake-hwclock 으로 마지막 시각을 보존합니다"
    if dpkg -l 2>/dev/null | grep -q "^ii  fake-hwclock "; then
        ok "fake-hwclock 이미 설치됨"
    else
        sudo apt install -y fake-hwclock >/dev/null 2>&1 \
            && ok "fake-hwclock 설치 완료" \
            || fail "fake-hwclock 설치 실패 — 인터넷 연결 확인"
    fi
    # 종료 시점이 아니라 주기적으로 저장해야 강제 종료에도 남습니다
    if [ ! -f /etc/cron.hourly/fake-hwclock-save ]; then
        sudo tee /etc/cron.hourly/fake-hwclock-save > /dev/null << 'CRONEOF'
#!/bin/sh
# 강제 종료 대비 — 시각을 주기적으로 저장 (정상 종료 시에만 저장하면 소용없음)
/usr/sbin/fake-hwclock save
CRONEOF
        sudo chmod +x /etc/cron.hourly/fake-hwclock-save
        ok "시각 주기 저장 등록 (1시간 간격)"
    else
        ok "시각 주기 저장 이미 등록됨"
    fi
fi
echo ""
echo " 비행 전 인터넷에 한 번 연결하면 시각이 정확해집니다 (check_time_sync 자동 수행)."

head "[8/8] 적용 결과 확인"
do_check

# ── 안내 ──────────────────────────────────────────────────────────────
head "완료"
echo ""

if [ "${NEED_REPLUG:-0}" = "1" ]; then
    echo " ⚠️  brltty 를 제거했습니다."
    echo "     FC 젠더(CH340) USB 를 한 번 뽑았다 다시 꽂으세요."
    echo "     그 후 아래로 확인:"
    echo "       detect_serial      # FC / THL100 / WCM6800 판별 확인"
    echo ""
fi

if [ "${NEED_RELOGIN:-0}" = "1" ]; then
    echo " ⚠️  dialout 그룹 적용을 위해 재로그인이 필요합니다."
    echo "       exit  후 다시 접속"
    echo ""
fi

echo " 다음 단계:"
echo "   source ~/.bashrc"
echo "   bash scripts/install_service.sh          # 서비스 등록 (부팅 자동실행 OFF)"
echo "   sudo systemctl start $SERVICE_NAME"
echo "   sleep 40 && monitor_drone --once"
echo ""
echo "   문제 없으면 실기체 운용 전환:"
echo "   bash scripts/install_service.sh enable"
echo ""
echo " 상태 재확인:"
echo "   bash scripts/setup_onboard_env.sh check"
echo "=========================================="
