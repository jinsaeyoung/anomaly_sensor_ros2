#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 특정 bag 의 녹화 전후 로그를 추출
#
# scan_bags.py 로 문제 bag 을 찾은 뒤, 그 시점의 로그만 잘라내 확인합니다.
# 전체 로그가 수만 줄이라 직접 찾기 번거로운 것을 자동화합니다.
#
# 사용법:
#   bash scripts/extract_bag_log.sh flight_20260827_154350
#   bash scripts/extract_bag_log.sh flight_20260827_154350 200 100
#                                    (앞 200줄, 뒤 100줄)
# ══════════════════════════════════════════════════════════════════════════════

set -u

BAG_NAME="${1:-}"
BEFORE="${2:-120}"
AFTER="${3:-60}"
LOG="${ANOMALY_LOG:-$HOME/anomaly_data/onboard.log}"

if [ -z "$BAG_NAME" ]; then
    echo "사용법: $0 <bag이름> [앞줄수] [뒤줄수]"
    echo ""
    echo "최근 bag 목록:"
    ls -t "${ANOMALY_DATA:-$HOME/anomaly_data}" 2>/dev/null \
        | grep -E "^(flight|anomaly_data)_" | head -10 | sed 's/^/  /'
    exit 1
fi

if [ ! -f "$LOG" ]; then
    echo "ERROR: 로그 파일 없음: $LOG"
    exit 1
fi

echo "=========================================="
echo " bag: $BAG_NAME"
echo " 로그: $LOG"
echo "=========================================="

# 녹화 시작/종료 줄 번호 찾기
HITS=$(grep -an "$BAG_NAME" "$LOG" | head -20)
if [ -z "$HITS" ]; then
    echo "이 bag 에 대한 로그 기록이 없습니다."
    echo "로테이션되었을 수 있으니 ${LOG}.1 도 확인하세요:"
    echo "  ANOMALY_LOG=${LOG}.1 $0 $BAG_NAME"
    exit 1
fi

echo ""
echo "── 관련 로그 줄 ──────────────────────────"
echo "$HITS"
echo ""

START_LINE=$(echo "$HITS" | head -1 | cut -d: -f1)
END_LINE=$(echo "$HITS" | tail -1 | cut -d: -f1)

FROM=$(( START_LINE - BEFORE ))
[ $FROM -lt 1 ] && FROM=1
TO=$(( END_LINE + AFTER ))

echo "── 추출 구간: ${FROM} ~ ${TO} 줄 ──────────"
echo ""

OUT="/tmp/${BAG_NAME}_context.log"
sed -n "${FROM},${TO}p" "$LOG" > "$OUT"

# 반복되는 동일 메시지는 요약해서 보여줍니다
echo "── 요약 (중복 제거) ──────────────────────"
sed -n "${FROM},${TO}p" "$LOG" \
  | sed -E 's/\[[0-9]+\.[0-9]+\]//g' \
  | sed -E 's/[0-9]{10,}//g' \
  | sort | uniq -c | sort -rn | head -25

echo ""
echo "── 주요 이벤트 ───────────────────────────"
grep -aE "녹화 시작|녹화 종료|USB 연결|찾을 수 없|시리얼 포트 연결|진단|CON: Got HEARTBEAT" "$OUT" \
  | head -30

echo ""
echo "=========================================="
echo " 전체 구간 저장: $OUT"
echo " 줄 수: $(wc -l < "$OUT")"
echo "=========================================="
echo ""
echo " 다음 단계:"
echo "   less $OUT                    # 전체 보기"
echo "   grep -a respeaker $OUT       # 특정 항목만"
