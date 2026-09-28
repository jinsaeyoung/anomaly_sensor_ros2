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
#   FCU_URL           FC 연결 강제 지정 (기본: 비움 = 자동 탐지)
#   TGT_SYSTEM        대상 기체 SYSID 강제 지정 (기본: 비움 = 자동 탐지)
#   EXPECT_SERIAL     부팅 시 기다릴 시리얼 장치 수 (기본 0 = 개수 무관)
#   WAIT_USB_SEC      장치 대기 최대 시간
#   FC_STABLE_SEC     FC 장치가 끊김 없이 유지되어야 하는 시간 (기본 6초)
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WAIT_USB_SEC="${WAIT_USB_SEC:-60}"
SAVE_DIR="${ANOMALY_DATA:-$HOME/anomaly_data}"

# FC 연결 — TELEM2 + USB-TTL 젠더(CH340) 기본
FCU_URL="${FCU_URL:-}"                 # 비우면 자동 탐지 (권장)
TGT_SYSTEM="${TGT_SYSTEM:-}"           # 비우면 HEARTBEAT 로 자동 탐지

mkdir -p "$SAVE_DIR"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

# ── 로그 로테이션 ─────────────────────────────────────────────────────
# onboard.log 는 서비스가 append 로 계속 기록하므로 장기 운용 시
# 수백 MB 까지 커져 grep 이 "바이너리 파일" 로 인식하기도 합니다.
# 시작 시 크기를 확인해 임계값을 넘으면 한 세대만 보관하고 새로 시작합니다.
LOG_FILE="$SAVE_DIR/onboard.log"
LOG_MAX_MB="${LOG_MAX_MB:-50}"
if [ -f "$LOG_FILE" ]; then
    size_mb=$(du -m "$LOG_FILE" 2>/dev/null | cut -f1)
    if [ -n "$size_mb" ] && [ "$size_mb" -ge "$LOG_MAX_MB" ]; then
        mv -f "$LOG_FILE" "${LOG_FILE}.1" 2>/dev/null || true
        : > "$LOG_FILE" 2>/dev/null || true
    fi
fi

log "=========================================="
log " 온보드 데이터 수집 시작"
log " 워크스페이스: $WS"
log " 저장 경로:    $SAVE_DIR"
log " FC 연결:      ${FCU_URL:-자동 탐지}  SYSID: ${TGT_SYSTEM:-자동 탐지}"
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

# ── 시리얼 장치 대기 및 안정화 확인 ───────────────────────────────────
# 부팅 직후에는 USB 열거가 끝나지 않아 장치가 늦게 나타나고,
# FC 전원 인가 중에는 USB-UART 젠더가 붙었다 떨어지기를 반복할 수 있습니다.
#
# FCU_URL 을 지정했으면 그 장치를, 지정하지 않았으면(기본)
# 연결된 시리얼 장치 목록 전체가 STABLE_SEC 동안 변하지 않을 때까지 기다립니다.
# 어떤 장치가 FC/THL100/WCM6800 인지는 launch 가 데이터로 판별합니다.
STABLE_SEC="${FC_STABLE_SEC:-6}"
EXPECT_SERIAL="${EXPECT_SERIAL:-0}"     # 기대 장치 수 (0 = 개수 무관)

serial_set() { ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort | tr '\n' ' '; }

if [ -n "$FCU_URL" ]; then
    FC_DEV="${FCU_URL%%:*}"
    log "FC 장치 대기 중: $FC_DEV (지정됨)"
else
    log "시리얼 장치 안정화 대기 (FC 는 launch 가 자동 판별)"
fi
log "  최대 대기 ${WAIT_USB_SEC}초 / 안정화 확인 ${STABLE_SEC}초"

waited=0
stable=0
prev=""
while [ $waited -lt "$WAIT_USB_SEC" ]; do
    if [ -n "$FCU_URL" ]; then
        [ -e "$FC_DEV" ] && cur="present" || cur=""
    else
        cur="$(serial_set)"
        if [ "$EXPECT_SERIAL" -gt 0 ]; then
            n=$(echo "$cur" | wc -w)
            [ "$n" -lt "$EXPECT_SERIAL" ] && cur=""
        fi
    fi

    if [ -n "$cur" ] && [ "$cur" = "$prev" ]; then
        stable=$((stable + 1))
        if [ $stable -ge "$STABLE_SEC" ]; then
            log "장치 안정화 완료 (${waited}초 경과)"
            break
        fi
    else
        [ $stable -gt 0 ] && log "  장치 목록 변동 감지 — 안정화 카운터 초기화"
        stable=0
    fi
    prev="$cur"
    sleep 1
    waited=$((waited + 1))
done

if [ $stable -lt "$STABLE_SEC" ]; then
    log "경고: 장치가 안정되지 않았습니다 — 일단 진행합니다."
    log "      각 노드가 실행 중 스스로 재탐색하므로 나중에 연결돼도 복구됩니다."
fi

# (장치 노드 권한은 udev 가 생성 시점에 적용합니다. 위 안정화 확인에서
#  이미 수 초 이상 유지된 장치이므로 별도 대기가 필요 없습니다.)

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
# 부팅 직후에는 대개 남은 프로세스가 없으므로, 정리했을 때만 기다립니다.
if pkill -f mavros_node 2>/dev/null; then
    sleep 2
fi

# ── 실행 ──────────────────────────────────────────────────────────────
log "launch 실행 (자동 녹화 활성화)"

LAUNCH_ARGS=(
    use_auto_record:=true
    save_dir:="$SAVE_DIR"
    post_disarm_sec:=10.0
    max_bag_duration:=3000
    min_free_gb:=2.0
)
# 지정한 경우에만 넘깁니다. 넘기지 않으면 launch 의 자동 탐지 결과를 씁니다.
[ -n "$FCU_URL" ]    && LAUNCH_ARGS+=( fcu_url:="$FCU_URL" )
[ -n "$TGT_SYSTEM" ] && LAUNCH_ARGS+=( tgt_system:="$TGT_SYSTEM" )

exec ros2 launch drone_sensors drone_sensor_launch.py "${LAUNCH_ARGS[@]}"
