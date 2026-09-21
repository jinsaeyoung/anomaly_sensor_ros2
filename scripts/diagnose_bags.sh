#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 누락 bag 의 원인을 시간대별로 자동 분류
#
# scan_bags.py 로 찾은 문제 bag 들에 대해, onboard.log 에서
# 해당 시점의 노드 시작·USB 인식 상태를 추출해 사유를 판정합니다.
#
# 사용법:
#   bash scripts/diagnose_bags.sh                   # 전체 자동 분석
#   bash scripts/diagnose_bags.sh flight_2026...    # 특정 bag 만
# ══════════════════════════════════════════════════════════════════════════════

set -u

DATA="${ANOMALY_DATA:-$HOME/anomaly_data}"
LOG="${ANOMALY_LOG:-$DATA/onboard.log}"
WS="${ANOMALY_WS:-$HOME/anomaly_sensor_ros2}"
TARGET="${1:-}"

if [ ! -f "$LOG" ]; then
    echo "ERROR: 로그 없음: $LOG"
    exit 1
fi

echo "=========================================="
echo " 누락 bag 원인 분석"
echo " 로그: $LOG ($(wc -l < "$LOG") 줄)"
echo "=========================================="
echo ""

# ── 서비스/launch 시작 지점 목록 ──────────────────────────────────────
# 각 세션마다 ReSpeaker·시리얼 초기화 결과가 달라지므로
# 세션 경계를 찾아 그 안에서 상태를 판정합니다.
echo "── 세션(launch 시작) 목록 ────────────────"
grep -an "process started with pid" "$LOG" \
  | grep -a "mavros_node" \
  | awk -F: '{print $1}' > /tmp/_sessions.txt
echo "  세션 수: $(wc -l < /tmp/_sessions.txt)"
echo ""

# ── 세션별 센서 초기화 결과 ───────────────────────────────────────────
echo "── 세션별 초기화 결과 ────────────────────"
printf "%-8s %-22s %-10s %-10s %-10s\n" "줄번호" "시각(추정)" "ReSpeaker" "THL100" "WCM6800"
echo "------------------------------------------------------------------"

prev=1
while read -r line; do
    # 이 세션 구간 (다음 세션 전까지, 최대 400줄)
    end=$(( line + 400 ))

    seg=$(sed -n "${line},${end}p" "$LOG")

    # ReSpeaker 판정
    if echo "$seg" | grep -qa "오디오 장치 발견"; then
        mic="OK"
    elif echo "$seg" | grep -qa "ReSpeaker를 찾을 수 없"; then
        mic="실패"
    else
        mic="-"
    fi

    # THL100 판정
    if echo "$seg" | grep -qa "thl100_node.*시리얼 포트 연결"; then
        thl="OK"
    elif echo "$seg" | grep -qa "thl100_node.*시리얼 연결 실패"; then
        thl="실패"
    else
        thl="-"
    fi

    # WCM6800 판정
    if echo "$seg" | grep -qa "wcm6800_node.*시리얼 포트 연결"; then
        wcm="OK"
    elif echo "$seg" | grep -qa "wcm6800_node.*시리얼 연결 실패"; then
        wcm="실패"
    else
        wcm="-"
    fi

    # 이 세션에서 녹화된 bag 이름 (시각 추정용)
    bag=$(echo "$seg" | grep -aoE "flight_[0-9]{8}_[0-9]{6}" | head -1)
    [ -z "$bag" ] && bag="(녹화없음)"

    printf "%-8s %-22s %-10s %-10s %-10s\n" "$line" "$bag" "$mic" "$thl" "$wcm"
    prev=$line
done < /tmp/_sessions.txt

echo ""
echo "=========================================="
echo " 개별 bag 상세"
echo "=========================================="

# ── 특정 bag 지정 시 상세 출력 ────────────────────────────────────────
if [ -n "$TARGET" ]; then
    echo ""
    echo "── $TARGET ──────────────────────────────"
    ln=$(grep -an "녹화 시작.*$TARGET" "$LOG" | head -1 | cut -d: -f1)
    if [ -z "$ln" ]; then
        echo "  로그에 기록 없음 (로테이션 가능성)"
        exit 0
    fi

    from=$(( ln - 400 )); [ $from -lt 1 ] && from=1

    echo ""
    echo "  [초기화 이벤트]"
    sed -n "${from},${ln}p" "$LOG" \
      | grep -aE "process started|오디오 장치 발견|ReSpeaker를 찾을 수 없|시리얼 포트 연결|초기 버퍼" \
      | tail -12 | sed 's/^/    /'

    echo ""
    echo "  [녹화 직전 30초 경고]"
    sed -n "$(( ln - 60 )),${ln}p" "$LOG" \
      | grep -aE "수신 패킷 없음|진단" | tail -6 | sed 's/^/    /'
fi

echo ""
echo "=========================================="
echo " 판정 기준"
echo "=========================================="
echo "  ReSpeaker 실패 → 노드 즉시 종료 → 토픽 '없음'"
echo "  시리얼 실패    → 노드는 생존, 재시도 → 토픽 '0건'"
echo ""
echo " 특정 bag 상세:"
echo "   bash scripts/diagnose_bags.sh flight_20260827_154350"
