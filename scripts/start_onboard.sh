#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 온보드 자동 실행 스크립트 (systemd 에서 호출)
#
# 부팅 후 USB 장치가 준비될 때까지 대기한 뒤 전체 센서 + 자동 녹화를 실행합니다.
# systemd 가 SIGINT 를 직접 전달할 수 있도록 마지막에 exec 를 사용합니다.
#
# 수동 실행도 가능:  bash scripts/start_onboard.sh
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WAIT_USB_SEC="${WAIT_USB_SEC:-60}"     # USB 장치 대기 최대 시간
SAVE_DIR="${ANOMALY_DATA:-$HOME/anomaly_data}"

mkdir -p "$SAVE_DIR"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

log "=========================================="
log " 온보드 데이터 수집 시작"
log " 워크스페이스: $WS"
log " 저장 경로:    $SAVE_DIR"
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

# ── FC 장치 대기 ──────────────────────────────────────────────────────
# 부팅 직후에는 USB 열거링이 끝나지 않아 장치가 늦게 나타날 수 있습니다.
log "FC 장치 대기 중 (최대 ${WAIT_USB_SEC}초)..."
waited=0
while [ $waited -lt "$WAIT_USB_SEC" ]; do
    if ls /dev/serial/by-id/*CubeOrange* >/dev/null 2>&1 \
       || ls /dev/ttyACM* >/dev/null 2>&1; then
        log "FC 장치 감지됨"
        break
    fi
    sleep 2
    waited=$((waited + 2))
done

if [ $waited -ge "$WAIT_USB_SEC" ]; then
    log "경고: FC 장치를 찾지 못했습니다. mavros 없이 센서만 동작할 수 있습니다."
fi

# 시리얼 센서도 잠시 더 대기
sleep 3

log "연결된 시리얼 장치:"
ls -la /dev/serial/by-id/ 2>/dev/null | tail -n +4 || log "  (없음)"

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
    use_auto_record:=true \
    save_dir:="$SAVE_DIR" \
    post_disarm_sec:=10.0 \
    max_bag_duration:=300 \
    min_free_gb:=2.0
