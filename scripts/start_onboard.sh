#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 온보드 자동 실행 스크립트 (systemd 에서 호출)
#
# 부팅 후 USB/시리얼 장치가 준비될 때까지 대기한 뒤
# 전체 센서 + 자동 녹화를 실행합니다.
# systemd 가 SIGINT 를 직접 전달할 수 있도록 마지막에 exec 를 사용합니다.
#
# 수동 실행:  bash scripts/start_onboard.sh
#
# 환경변수로 조정 가능:
#   ANOMALY_WS        워크스페이스 경로
#   ANOMALY_DATA      저장 경로
#   FCU_URL           FC 연결 (기본: TELEM2 CH340 젠더 921600)
#   WAIT_USB_SEC      장치 대기 최대 시간
#   FC_STABLE_SEC     FC 장치가 끊김 없이 유지되어야 하는 시간 (기본 6초)
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WAIT_USB_SEC="${WAIT_USB_SEC:-60}"
SAVE_DIR="${ANOMALY_DATA:-$HOME/anomaly_data}"

# FC 연결 — TELEM2 + USB-TTL 젠더(CH340) 기본
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:921600}"

mkdir -p "$SAVE_DIR"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

log "=========================================="
log " 온보드 데이터 수집 시작"
log " 워크스페이스: $WS"
log " 저장 경로:    $SAVE_DIR"
log " FC 연결:      $FCU_URL"
log "=========================================="

# ── ROS 환경 로드 ─────────────────────────────────────────────────────
# setup.bash 는 미정의 변수를 참조하므로 set -u 를 사용하지 않습니다.
if [ ! -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]; then
    log "ERROR: ROS2 $ROS_DISTRO_NAME 를 찾을 수 없습니다."
    exit 1
fi
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"

if [ ! -f "$WS/install/setup.bash" ]; then
    log "ERROR: $WS/install/setup.bash 없음 — 빌드가 필요합니다."
    exit 1
fi
source "$WS/install/setup.bash"

# ── FC 장치 대기 및 안정화 확인 ───────────────────────────────────────
# 부팅 직후에는 USB 열거링이 끝나지 않아 장치가 늦게 나타납니다.
# 또한 FC 에 전원이 인가되는 과정에서 TELEM 라인 전압이 흔들리면
# USB-TTL 젠더가 붙었다 떨어지기를 반복(재열거링)할 수 있습니다.
# 이때 mavros 가 먼저 포트를 열면 장치가 사라져 재연결에 실패하므로,
# STABLE_SEC 동안 연속으로 존재할 때만 안정된 것으로 판단합니다.
FC_DEV="${FCU_URL%%:*}"          # URL 에서 장치 경로만 추출
STABLE_SEC="${FC_STABLE_SEC:-6}" # 이 시간만큼 끊김 없이 유지되어야 함

log "FC 장치 대기 중: $FC_DEV"
log "  최대 대기 ${WAIT_USB_SEC}초 / 안정화 확인 ${STABLE_SEC}초"

waited=0
stable=0
while [ $waited -lt "$WAIT_USB_SEC" ]; do
    if [ -e "$FC_DEV" ]; then
        stable=$((stable + 1))
        if [ $stable -eq 1 ]; then
            log "  장치 감지 (${waited}초) — 안정화 확인 중"
        fi
        # 1초 간격으로 STABLE_SEC 회 연속 확인되면 완료
        if [ $stable -ge "$STABLE_SEC" ]; then
            log "FC 장치 안정화 완료 (${waited}초 경과)"
            break
        fi
    else
        if [ $stable -gt 0 ]; then
            log "  장치가 사라짐 — 재열거링 감지, 안정화 카운터 초기화"
        fi
        stable=0
    fi
    sleep 1
    waited=$((waited + 1))
done

if [ ! -e "$FC_DEV" ]; then
    log "경고: FC 장치를 찾지 못했습니다 — 센서만 동작합니다."
    log "      (FC 전원 인가 후 mavros 가 자동 재연결을 시도합니다)"
    log "      연결된 장치 목록:"
    ls -la /dev/serial/by-id/ 2>/dev/null | tail -n +4 | while read -r l; do
        log "        $l"
    done
elif [ $stable -lt "$STABLE_SEC" ]; then
    log "경고: 장치가 불안정합니다 (연속 유지 ${stable}/${STABLE_SEC}초)"
    log "      전원/케이블 접촉을 확인하세요. 일단 진행합니다."
fi

# 장치 노드 권한이 적용될 시간 확보 (udev 규칙 처리)
sleep 3

log "연결된 시리얼 장치:"
if ls /dev/serial/by-id/* >/dev/null 2>&1; then
    for f in /dev/serial/by-id/*; do
        log "  $(basename "$f") → $(readlink -f "$f")"
    done
else
    log "  (없음)"
fi

# ── 시간 동기화 확인 (실패해도 계속 진행) ─────────────────────────────
if [ -x "$WS/scripts/check_time_sync.sh" ]; then
    "$WS/scripts/check_time_sync.sh" || true
fi

# ── 이전 프로세스 정리 ────────────────────────────────────────────────
pkill -f mavros_node 2>/dev/null || true
sleep 2

# ── 실행 ──────────────────────────────────────────────────────────────
log "launch 실행 (자동 녹화 활성화)"

exec ros2 launch drone_sensors drone_sensor_launch.py \
    fcu_url:="$FCU_URL" \
    use_auto_record:=true \
    save_dir:="$SAVE_DIR" \
    post_disarm_sec:=10.0 \
    max_bag_duration:=300 \
    min_free_gb:=2.0
