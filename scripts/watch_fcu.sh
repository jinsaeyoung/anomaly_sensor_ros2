#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# FC 연결 감시 및 자동 복구
#
# mavros 는 시리얼 장치가 사라졌다 다시 나타나면 재연결에 실패하는 경우가 있습니다.
# (FC 전원 인가 시 USB-TTL 젠더 재열거링, 케이블 접촉 불량 등)
#
# 이 스크립트는 /mavros/state 수신 여부를 주기적으로 확인하고,
# 장치는 존재하는데 연결이 끊긴 상태가 지속되면 서비스를 재시작합니다.
#
# 사용법:
#   bash scripts/watch_fcu.sh              # 포그라운드 감시
#   bash scripts/watch_fcu.sh --once       # 1회 점검만
#
# 환경변수:
#   FCU_URL          FC 연결 (기본: TELEM2 CH340 921600)
#   CHECK_INTERVAL   점검 주기 (기본 30초)
#   FAIL_THRESHOLD   연속 실패 허용 횟수 (기본 3회)
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:921600}"
FC_DEV="${FCU_URL%%:*}"
CHECK_INTERVAL="${CHECK_INTERVAL:-30}"
FAIL_THRESHOLD="${FAIL_THRESHOLD:-3}"
SERVICE_NAME="anomaly-sensor"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] [watch_fcu] $*"; }

# ── FC 연결 상태 점검 ─────────────────────────────────────────────────
# 반환: 0 = 정상,  1 = 장치 없음,  2 = 장치는 있으나 mavros 미연결
check_fcu() {
    if [ ! -e "$FC_DEV" ]; then
        return 1
    fi
    # /mavros/state 의 connected 필드 확인
    local out
    out=$(timeout 8 ros2 topic echo /mavros/state --once \
            --qos-reliability best_effort 2>/dev/null | grep -m1 "^connected:")
    if echo "$out" | grep -q "true"; then
        return 0
    fi
    return 2
}

do_check() {
    check_fcu
    local rc=$?
    case $rc in
        0) log "정상 — FC 연결됨" ;;
        1) log "장치 없음: $FC_DEV (FC 전원 확인 필요)" ;;
        2) log "장치는 있으나 mavros 미연결" ;;
    esac
    return $rc
}

# ── 1회 점검 모드 ─────────────────────────────────────────────────────
if [ "${1:-}" = "--once" ]; then
    do_check
    exit $?
fi

# ── 감시 루프 ─────────────────────────────────────────────────────────
log "FC 연결 감시 시작"
log "  장치:      $FC_DEV"
log "  점검 주기: ${CHECK_INTERVAL}초"
log "  복구 임계: 연속 ${FAIL_THRESHOLD}회 실패"

fail=0
while true; do
    sleep "$CHECK_INTERVAL"

    check_fcu
    rc=$?

    if [ $rc -eq 0 ]; then
        if [ $fail -gt 0 ]; then
            log "연결 복구됨"
        fi
        fail=0
        continue
    fi

    if [ $rc -eq 1 ]; then
        # 장치 자체가 없으면 재시작해도 소용없음 — 카운터만 유지
        log "장치 없음 — FC 전원 대기 중 (재시작 보류)"
        fail=0
        continue
    fi

    # rc=2 : 장치는 있는데 연결 안 됨 → 복구 대상
    fail=$((fail + 1))
    log "연결 실패 ${fail}/${FAIL_THRESHOLD} (장치는 존재)"

    if [ $fail -ge "$FAIL_THRESHOLD" ]; then
        # 녹화 중이면 재시작하지 않음 (데이터 손실 방지)
        rec=$(timeout 6 ros2 topic echo /auto_record/status --once 2>/dev/null \
                | grep -o '"recording": true' || true)
        if [ -n "$rec" ]; then
            log "녹화 중이므로 재시작 보류 — 비행 종료 후 조치하세요"
            fail=0
            continue
        fi

        log "서비스 재시작 시도"
        if sudo -n systemctl restart "$SERVICE_NAME" 2>/dev/null; then
            log "재시작 완료 — 40초 후 재점검"
            sleep 40
        else
            log "재시작 실패 (sudo 권한 없음)"
            log "  수동 실행: sudo systemctl restart $SERVICE_NAME"
        fi
        fail=0
    fi
done
