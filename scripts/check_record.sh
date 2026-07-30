#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 자동 녹화 상태 확인 (온보드 운용 시 SSH 로 빠르게 점검)
#
# 사용법: bash scripts/check_record.sh
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"
SAVE_DIR="${ANOMALY_DATA:-$HOME/anomaly_data}"

source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

echo "=========================================="
echo " 서비스 상태"
echo "=========================================="
systemctl is-active anomaly-sensor 2>/dev/null || echo "미실행 또는 미등록"
systemctl is-enabled anomaly-sensor 2>/dev/null | sed 's/^/부팅 자동실행: /'

echo ""
echo "=========================================="
echo " 실행 중인 노드"
echo "=========================================="
timeout 5 ros2 node list 2>/dev/null | grep -E "mavros|thl100|wcm6800|respeaker|auto_record" \
  || echo "  (노드 없음)"

echo ""
echo "=========================================="
echo " 녹화 상태"
echo "=========================================="
timeout 6 ros2 topic echo /auto_record/status --once 2>/dev/null \
  || echo "  (auto_record_node 미실행)"

echo ""
echo "=========================================="
echo " FC arm 상태"
echo "=========================================="
timeout 6 ros2 topic echo /mavros/state --once --qos-reliability best_effort 2>/dev/null \
  | grep -E "armed|mode|connected" \
  || echo "  (mavros 미실행)"

echo ""
echo "=========================================="
echo " 디스크 / 저장 파일"
echo "=========================================="
df -h "$SAVE_DIR" | tail -1
echo ""
echo "최근 녹화 5건:"
ls -lt "$SAVE_DIR" 2>/dev/null | grep "^d" | head -5 | awk '{print "  "$6" "$7" "$8"  "$9}' \
  || echo "  (없음)"
echo "=========================================="
