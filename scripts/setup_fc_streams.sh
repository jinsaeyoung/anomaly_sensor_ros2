#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# ArduPilot 데이터 스트림(SR) 파라미터 설정
#
# ArduPilot 은 기본적으로 일부 메시지만 전송합니다.
# mavros 로 IMU·포지션·진동 등을 받으려면 해당 포트의 SR 파라미터를 켜야 합니다.
# 값은 FC 에 영구 저장되므로 포트당 1회만 실행하면 됩니다.
#
# 중요: 포트마다 파라미터 접두어가 다릅니다.
#   USB 직결 (SERIAL0) → SR0_*
#   TELEM1   (SERIAL1) → SR1_*
#   TELEM2   (SERIAL2) → SR2_*      ← 기본값
#
# 사용법:
#   bash scripts/setup_fc_streams.sh          # SR2 (TELEM2)
#   bash scripts/setup_fc_streams.sh 0        # SR0 (USB)
#   bash scripts/setup_fc_streams.sh 2 10     # SR2, 10Hz
#   bash scripts/setup_fc_streams.sh check    # 현재 값 조회만
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u

PARAMS=(RAW_SENS EXT_STAT RC_CHAN RAW_CTRL POSITION EXTRA1 EXTRA2 EXTRA3)

# 각 스트림이 담당하는 토픽 (안내용)
declare -A DESC=(
  [RAW_SENS]="imu/data_raw, imu/mag"
  [EXT_STAT]="battery, sys_status, extended_state, gps"
  [RC_CHAN]="rc/in, rc/out"
  [RAW_CTRL]="제어 원시값"
  [POSITION]="local_position/*, global_position/*"
  [EXTRA1]="imu/data (ATTITUDE)"
  [EXTRA2]="vfr_hud"
  [EXTRA3]="vibration, wind, esc"
)

MODE="${1:-2}"
RATE="${2:-10}"

# ── mavros 연결 확인 ──────────────────────────────────────────────────
if ! timeout 5 ros2 service type /mavros/param/set >/dev/null 2>&1; then
    echo "ERROR: mavros 가 실행 중이 아닙니다."
    echo "       먼저 launch 를 실행한 뒤 다시 시도하세요."
    exit 1
fi

# ── 조회 모드 ─────────────────────────────────────────────────────────
if [ "$MODE" = "check" ]; then
    echo "=========================================="
    echo " 현재 SR 파라미터 / 시리얼 설정"
    echo "=========================================="
    for n in 0 1 2; do
        printf '\n[SERIAL%s]\n' "$n"
        for f in PROTOCOL BAUD OPTIONS; do
            v=$(timeout 5 ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet \
                 "{param_id: 'SERIAL${n}_${f}'}" 2>/dev/null \
                 | grep -oP 'integer=\K-?\d+' | head -1)
            printf '  SERIAL%s_%-9s = %s\n' "$n" "$f" "${v:-?}"
        done
        for p in "${PARAMS[@]}"; do
            v=$(timeout 5 ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet \
                 "{param_id: 'SR${n}_${p}'}" 2>/dev/null \
                 | grep -oP 'integer=\K-?\d+' | head -1)
            printf '  SR%s_%-10s = %s\n' "$n" "$p" "${v:-?}"
        done
    done
    echo ""
    echo "=========================================="
    exit 0
fi

# ── 설정 모드 ─────────────────────────────────────────────────────────
PORT_NUM="$MODE"
case "$PORT_NUM" in
    0) PORT_DESC="USB 직결 (SERIAL0)" ;;
    1) PORT_DESC="TELEM1 (SERIAL1)" ;;
    2) PORT_DESC="TELEM2 (SERIAL2)" ;;
    *) echo "ERROR: 포트 번호는 0, 1, 2 중 하나여야 합니다."; exit 1 ;;
esac

echo "=========================================="
echo " ArduPilot 스트림 파라미터 설정"
echo "=========================================="
echo " 대상 포트: $PORT_DESC → SR${PORT_NUM}_*"
echo " 설정 주기: ${RATE}Hz"
echo "=========================================="
echo ""

ok=0
fail=0
for p in "${PARAMS[@]}"; do
    name="SR${PORT_NUM}_${p}"
    printf '  %-16s (%s)\n' "$name" "${DESC[$p]}"
    if timeout 10 ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
        "{param_id: '$name', value: {type: 2, integer_value: $RATE}}" 2>/dev/null \
        | grep -q "success=True"; then
        echo "      → 설정 완료"
        ok=$((ok + 1))
    else
        echo "      → 실패"
        fail=$((fail + 1))
    fi
done

echo ""
echo "=========================================="
echo " 완료: 성공 $ok / 실패 $fail"
echo "=========================================="

if [ "$fail" -gt 0 ]; then
    echo ""
    echo "일부 실패 시 확인 사항:"
    echo "  - mavros 연결 상태 (ros2 topic echo /mavros/state --once --qos-reliability best_effort)"
    echo "  - 파라미터 목록 수신 완료 여부 (연결 직후 10~20초 소요)"
    echo "  - Mission Planner 등 GCS 동시 접속 여부"
fi

echo ""
echo "적용 확인:"
echo "  ros2 topic hz /mavros/imu/data"
echo "  ros2 topic hz /mavros/imu/data_raw"
echo "  bash scripts/setup_fc_streams.sh check"
