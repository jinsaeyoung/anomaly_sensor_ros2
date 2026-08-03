#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 온보드 환경 일괄 설정 스크립트
#
# 무인 운용에 필요한 시스템 레벨 설정을 한 번에 처리합니다.
#   1. brltty 제거        — CH340 젠더를 점자 장치로 오인해 가로채는 문제 해결
#   2. sudo NOPASSWD      — watch_fcu 자동 복구가 비밀번호 없이 동작하도록
#   3. ROS_DOMAIN_ID 고정 — 서비스와 셸의 DDS 도메인 불일치 방지
#   4. udev 규칙          — 시리얼/ReSpeaker 접근 권한
#   5. dialout 그룹       — 시리얼 포트 권한
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
    echo "[2] sudo NOPASSWD (자동 복구용)"
    if sudo -n systemctl status "$SERVICE_NAME" >/dev/null 2>&1; then
        ok "비밀번호 없이 systemctl 실행 가능"
    else
        warn "비밀번호 필요 — watch_fcu 자동 복구가 실패할 수 있습니다"
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
    echo "[7] USB 인식 (CH340 = FC 젠더)"
    if lsusb | grep -q "1a86:"; then
        if ls /dev/serial/by-id/ 2>/dev/null | grep -q "1a86"; then
            ok "CH340 인식 + 시리얼 노드 생성됨"
        else
            fail "CH340 은 보이나 시리얼 노드 없음 → brltty 충돌 의심"
        fi
    else
        warn "CH340 미인식 — 젠더 연결 확인 필요"
    fi

    echo ""
    echo "[8] systemd 서비스"
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
head "[1/6] brltty 제거 (CH340 젠더 충돌 해결)"
echo ""
echo " Ubuntu 기본 설치된 brltty(점자 단말기 데몬)가 CH340(1a86:7523)을"
echo " 점자 장치로 오인해 가로채면, ch341 드라이버가 바인딩되지 못해"
echo " /dev/ttyUSB* 노드가 생성되지 않습니다."
echo " (lsusb 에는 보이는데 check_usb 에는 안 나오는 증상)"
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

# ── 2. sudo NOPASSWD ──────────────────────────────────────────────────
head "[2/6] sudo NOPASSWD 설정 (자동 복구용)"
echo ""
echo " watch_fcu 가 FC 연결 실패 시 서비스를 자동 재시작하려면"
echo " 비밀번호 없이 systemctl 을 실행할 수 있어야 합니다."
echo " 지정한 3개 명령에만 적용되므로 전체 sudo 개방보다 안전합니다."
echo ""

SYSTEMCTL_BIN="$(command -v systemctl)"
SUDOERS_FILE="/etc/sudoers.d/anomaly-sensor"

sudo tee "$SUDOERS_FILE" > /dev/null << EOF
# anomaly_sensor_ros2 무인 운용용 — 서비스 제어만 비밀번호 없이 허용
$RUN_USER ALL=(ALL) NOPASSWD: $SYSTEMCTL_BIN restart $SERVICE_NAME
$RUN_USER ALL=(ALL) NOPASSWD: $SYSTEMCTL_BIN start $SERVICE_NAME
$RUN_USER ALL=(ALL) NOPASSWD: $SYSTEMCTL_BIN stop $SERVICE_NAME
$RUN_USER ALL=(ALL) NOPASSWD: $SYSTEMCTL_BIN status $SERVICE_NAME
EOF
sudo chmod 440 "$SUDOERS_FILE"

if sudo visudo -c -f "$SUDOERS_FILE" >/dev/null 2>&1; then
    ok "설정 완료: $SUDOERS_FILE"
else
    fail "문법 오류 — 파일을 제거합니다"
    sudo rm -f "$SUDOERS_FILE"
fi

# ── 3. ROS_DOMAIN_ID 고정 ─────────────────────────────────────────────
head "[3/6] ROS_DOMAIN_ID 고정"
echo ""
echo " systemd 서비스는 ROS_DOMAIN_ID=0 으로 실행됩니다."
echo " 셸에 값이 없거나 다르면 DDS 도메인이 달라져"
echo " 서비스가 정상 동작해도 check_topics 에 아무것도 안 보입니다."
echo ""

if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc 2>/dev/null; then
    echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
    ok ".bashrc 에 추가"
else
    ok "이미 등록됨"
fi
export ROS_DOMAIN_ID=0

# ── 4. udev 규칙 ──────────────────────────────────────────────────────
head "[4/6] udev 규칙 설정"

echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/60-respeaker.rules > /dev/null
ok "ReSpeaker (2886) 권한 규칙"

# CH340 이 brltty 에 잡히지 않도록 명시적으로 제외
sudo tee /etc/udev/rules.d/85-anomaly-serial.rules > /dev/null << 'EOF'
# CH340 (FC USB-TTL 젠더) — brltty 가 점자 장치로 오인하지 않도록 제외
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{BRLTTY_BRAILLE_DRIVER}="", ENV{BRLTTY_NO_DRIVER}="1"
EOF
ok "CH340 brltty 제외 규칙"

sudo udevadm control --reload-rules
sudo udevadm trigger
ok "udev 규칙 재적용"

# ── 5. dialout 그룹 ───────────────────────────────────────────────────
head "[5/6] dialout 그룹"

if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx dialout; then
    ok "이미 등록됨"
else
    sudo usermod -aG dialout "$RUN_USER"
    ok "등록 완료"
    NEED_RELOGIN=1
fi

if ! groups | tr ' ' '\n' | grep -qx dialout; then
    warn "현재 세션에는 미적용 — 재로그인 또는 'newgrp dialout' 필요"
    NEED_RELOGIN=1
fi

# ── 6. 검증 ───────────────────────────────────────────────────────────
head "[6/6] 적용 결과 확인"
do_check

# ── 안내 ──────────────────────────────────────────────────────────────
head "완료"
echo ""

if [ "${NEED_REPLUG:-0}" = "1" ]; then
    echo " ⚠️  brltty 를 제거했습니다."
    echo "     FC 젠더(CH340) USB 를 한 번 뽑았다 다시 꽂으세요."
    echo "     그 후 아래로 확인:"
    echo "       check_usb          # usb-1a86_USB_Serial 이 보여야 정상"
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
echo "   sleep 40 && check_record"
echo ""
echo "   문제 없으면 실기체 운용 전환:"
echo "   bash scripts/install_service.sh enable"
echo ""
echo " 상태 재확인:"
echo "   bash scripts/setup_onboard_env.sh check"
echo "=========================================="
