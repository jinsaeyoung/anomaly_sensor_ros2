#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 실시간 모니터
#
# FC arm 상태와 자동 녹화 상태를 한 화면에서 실시간으로 확인합니다.
# arm → 녹화 시작, disarm → 녹화 종료 전환을 눈으로 추적할 수 있습니다.
#
# 사용법:
#   monitor_drone            # 기본 갱신 (6초)
#   monitor_drone 10         # 10초 갱신
#   monitor_drone --full     # 토픽 주기까지 포함 (갱신 느림)
#   monitor_drone --once     # 1회만 출력
#   monitor_drone --debug    # 도메인/노드 수 진단 정보 표시
#
# 환경변수:
#   MONITOR_TIMEOUT      토픽 조회 대기 (기본 5초)
#   MONITOR_REC_TIMEOUT  녹화 상태 조회 대기 (기본 8초, 발행 주기 5초)
#   MONITOR_HZ_TIMEOUT   주기 측정 최소 대기 (기본 8초)
#
# 참고: ros2 topic hz 는 average rate 출력에 메시지 2개가 필요하므로
#       1Hz 토픽은 최소 2초 + discovery 시간이 걸립니다.
#
# 종료: Ctrl+C
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"
SAVE_DIR="${ANOMALY_DATA:-$HOME/anomaly_data}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
set -u

FULL=0
ONCE=0
INTERVAL=2
DEBUG=0
for a in "$@"; do
    case "$a" in
        --full)  FULL=1 ;;
        --once)  ONCE=1 ;;
        --debug) DEBUG=1 ;;
        [0-9]*)  INTERVAL="$a" ;;
    esac
done

# ros2 topic echo 는 매 호출마다 노드를 새로 만들어 DDS discovery 를 거칩니다.
# 2초로는 부족해 응답을 놓치는 경우가 있어 5초를 기본으로 둡니다.
TMO="${MONITOR_TIMEOUT:-5}"

# ros2 topic hz 는 average rate 를 내려면 최소 2개 메시지가 필요합니다.
# 1Hz 토픽은 2초 + DDS discovery 시간이 들어 별도의 긴 타임아웃이 필요합니다.
HZ_TMO="${MONITOR_HZ_TIMEOUT:-8}"

# /auto_record/status 는 5초 주기로 발행되므로 그보다 길게 대기해야 합니다.
REC_TMO="${MONITOR_REC_TIMEOUT:-8}"

# 갱신 주기가 조회 소요 시간보다 짧으면 요청이 겹쳐 실패하므로 보정
#   기본 모드: FC 상태 + 녹화 상태 조회
#   --full   : 위 + 토픽 5개 주기 측정 (각 HZ_TMO 초)
# 조회는 모두 병렬 실행이므로 가장 오래 걸리는 것 기준으로 계산합니다.
BASE_TMO=$TMO
[ "$REC_TMO" -gt "$BASE_TMO" ] && BASE_TMO=$REC_TMO
if [ "$FULL" = "1" ]; then
    MIN_INTERVAL=$((BASE_TMO + HZ_TMO + 2))
else
    MIN_INTERVAL=$((BASE_TMO + 2))
fi
if [ "$INTERVAL" -lt "$MIN_INTERVAL" ]; then
    INTERVAL=$MIN_INTERVAL
fi

# ── ros2 daemon 상태 확인 및 자동 복구 ────────────────────────────────
# ros2 daemon 은 CLI 조회용 캐시 프로세스입니다.
# 죽어도 노드 간 통신과 녹화에는 영향이 없지만, node list / topic echo 가
# 빈 결과를 반환해 모니터 화면이 비어 보입니다.
# 서비스는 살아있는데 노드가 0개로 보이면 데몬 문제로 판단해 재시작합니다.
DAEMON_FIXED=0
ensure_daemon() {
    local n
    n=$(timeout "$TMO" ros2 node list 2>/dev/null | wc -l)
    if [ "$n" -gt 0 ]; then
        return 0
    fi
    # 서비스가 실행 중인데 노드가 0 → 데몬 이상
    if [ "$(systemctl is-active anomaly-sensor 2>/dev/null)" = "active" ]; then
        ros2 daemon stop  >/dev/null 2>&1
        ros2 daemon start >/dev/null 2>&1
        sleep 3
        DAEMON_FIXED=1
    fi
}

# ── 1회 렌더링 ────────────────────────────────────────────────────────
render() {
    echo "  $(date '+%Y-%m-%d %H:%M:%S')"
    if [ "$DAEMON_FIXED" = "1" ]; then
        echo "  (ros2 daemon 재시작됨 — 데이터 수집에는 영향 없음)"
        DAEMON_FIXED=0
    fi
    echo ""

    if [ "$DEBUG" = "1" ]; then
        echo "  [debug] ROS_DOMAIN_ID=[${ROS_DOMAIN_ID:-미설정}]  timeout=${TMO}s"
        echo "  [debug] 노드 수: $(timeout "$TMO" ros2 node list 2>/dev/null | wc -l)"
        echo ""
    fi

    # FC 상태와 녹화 상태를 병렬로 조회 (순차 시 두 배 시간 소요)
    local qdir
    qdir=$(mktemp -d /tmp/monitor_q.XXXXXX)
    (
        timeout "$TMO" ros2 topic echo /mavros/state --once \
            --qos-reliability best_effort 2>/dev/null > "$qdir/state"
    ) &
    (
        timeout "$REC_TMO" ros2 topic echo /auto_record/status --once 2>/dev/null \
            | grep -m1 "^data:" > "$qdir/rec"
    ) &
    wait

    # FC 상태
    echo "┌─ FC 상태 ─────────────────────────────────────────────"
    local state conn armd mode cs as
    state=$(cat "$qdir/state" 2>/dev/null)
    if [ -z "$state" ]; then
        echo "│  mavros 응답 없음"
        if ! timeout "$TMO" ros2 node list 2>/dev/null | grep -q mavros; then
            echo "│  → mavros 노드 자체가 없음 (서비스/launch 확인)"
        else
            echo "│  → 노드는 있으나 /mavros/state 미수신 (FC 연결 확인)"
        fi
    else
        conn=$(echo "$state" | grep -m1 "^connected:" | awk '{print $2}')
        armd=$(echo "$state" | grep -m1 "^armed:"     | awk '{print $2}')
        mode=$(echo "$state" | grep -m1 "^mode:"      | awk '{print $2}')
        if [ "$conn" = "true" ]; then cs="연결됨"; else cs="끊김"; fi
        if [ "$armd" = "true" ]; then as=">>> ARMED <<<"; else as="disarmed"; fi
        printf "│  연결: %-8s  상태: %-16s  모드: %s\n" "$cs" "$as" "$mode"
    fi
    echo "└───────────────────────────────────────────────────────"
    echo ""

    # 녹화 상태
    echo "┌─ 녹화 상태 ───────────────────────────────────────────"
    local rec recording bag elapsed freegb
    rec=$(cat "$qdir/rec" 2>/dev/null)
    if [ -z "$rec" ]; then
        if timeout "$TMO" ros2 node list 2>/dev/null | grep -q auto_record_node; then
            echo "│  노드는 실행 중이나 상태 수신 실패 (재시도 중)"
        else
            echo "│  auto_record_node 미실행"
            echo "│  (use_auto_record:=true 로 실행했는지 확인)"
        fi
    else
        recording=$(echo "$rec" | grep -o '"recording": *[a-z]*'  | awk '{print $2}')
        bag=$(      echo "$rec" | grep -o '"bag": *"[^"]*"'       | sed 's/.*: *"//; s/"//')
        elapsed=$(  echo "$rec" | grep -o '"elapsed_s": *[0-9.]*' | awk '{print $2}')
        freegb=$(   echo "$rec" | grep -o '"free_gb": *[0-9.]*'   | awk '{print $2}')

        if [ "$recording" = "true" ]; then
            printf "│  ●  녹화 중        경과: %s초\n" "$elapsed"
            printf "│     파일: %s\n" "$bag"
        else
            echo "│  ○  대기 중 (arm 하면 자동 시작)"
        fi
        printf "│     디스크 여유: %s GB\n" "$freegb"
    fi
    echo "└───────────────────────────────────────────────────────"
    echo ""

    # 토픽 주기 (--full)
    if [ "$FULL" = "1" ]; then
        echo "┌─ 토픽 주기 ───────────────────────────────────────────"
        # 토픽별 hz 측정은 각각 수 초가 걸리므로 병렬로 실행합니다.
        # (순차 실행 시 5개 × 8초 = 40초 소요)
        local item t want hz tmpdir
        tmpdir=$(mktemp -d /tmp/monitor_hz.XXXXXX)

        for item in "/mavros/imu/data:50" "/mavros/battery:10" \
                    "/thl100/data:1" "/wcm6800/data:10" "/respeaker/doa:31"; do
            t="${item%%:*}"
            (
                timeout "$HZ_TMO" ros2 topic hz "$t" 2>/dev/null \
                    | grep -m1 "average rate" | awk '{print $3}' \
                    > "$tmpdir/$(echo "$t" | tr '/' '_')"
            ) &
        done
        wait

        for item in "/mavros/imu/data:50" "/mavros/battery:10" \
                    "/thl100/data:1" "/wcm6800/data:10" "/respeaker/doa:31"; do
            t="${item%%:*}"
            want="${item##*:}"
            hz=$(cat "$tmpdir/$(echo "$t" | tr '/' '_')" 2>/dev/null)
            printf "│  %-24s %8s Hz  (기대 %s)\n" "$t" "${hz:---}" "$want"
        done
        rm -rf "$tmpdir"
        echo "└───────────────────────────────────────────────────────"
        echo ""
    fi

    rm -rf "$qdir"

    # 최근 녹화
    echo "┌─ 최근 녹화 3건 ───────────────────────────────────────"
    ls -lt "$SAVE_DIR" 2>/dev/null | grep "^d" | head -3 \
        | awk '{printf "│  %s %s %s  %s\n", $6, $7, $8, $9}'
    echo "└───────────────────────────────────────────────────────"
}

# ── 1회 출력 모드 ─────────────────────────────────────────────────────
if [ "$ONCE" = "1" ]; then
    trap 'rm -rf /tmp/monitor_q.* /tmp/monitor_hz.* 2>/dev/null' EXIT
    ensure_daemon
    render
    exit 0
fi

# ── 반복 갱신 ─────────────────────────────────────────────────────────
echo "실시간 모니터 시작 (갱신 ${INTERVAL}초, Ctrl+C 종료)"
sleep 1

trap 'echo ""; rm -rf /tmp/monitor_q.* /tmp/monitor_hz.* 2>/dev/null; echo "모니터 종료"; exit 0' INT

while true; do
    ensure_daemon
    out="$(render)"
    clear
    echo "$out"
    sleep "$INTERVAL"
done
