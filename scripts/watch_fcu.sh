#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# FC 연결 상태 확인 (읽기 전용)
#
# FC 복구는 fcu_manager_node 가 자동으로 합니다.
#   연결이 끊기면 mavros 만 내리고 포트·baud·SYSID 를 재탐지해 다시 띄우며,
#   센서 노드와 녹화는 건드리지 않습니다.
#
# 이 스크립트는 그 상태를 보여주기만 합니다.
# (예전 버전은 서비스 전체를 재시작했는데, 관리 노드와 동시에 동작하면
#  센서·녹화까지 끊기므로 복구 기능을 제거했습니다)
#
# 사용법:
#   fc_status            # 1회 출력 (상시 확인은 monitor_drone)
# ══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

show() {
    echo "[$(date '+%H:%M:%S')] FC 관리 상태"
    local s
    # --field 옵션은 ROS2 버전에 따라 없을 수 있어 전체 출력에서 data 줄을 꺼냅니다
    s=$(timeout 6 ros2 topic echo /fcu_manager/status --once 2>/dev/null)
    if [ -z "$s" ]; then
        echo "  fcu_manager_node 응답 없음 — launch 실행 여부를 확인하세요"
        echo "  (FC 자체 확인: detect_serial)"
        return 1
    fi
    echo "$s" | python3 -c "
import sys, json
import yaml
text = sys.stdin.read()
try:
    raw = yaml.safe_load(text.split('---')[0])['data']
    d = json.loads(raw)
except Exception:
    print('  ' + text.strip()[:300]); sys.exit()
print(f\"  단계      : {d.get('phase')}  {d.get('note') or ''}\")
print(f\"  연결      : {'예' if d.get('connected') else '아니오'}\")
print(f\"  포트/baud : {d.get('port')} @ {d.get('baud')}\")
print(f\"  대상/자기 : {d.get('tgt')} / {d.get('self_id')}\")
print(f\"  재시작    : {d.get('restarts')}회\")
if len(d.get('autopilots') or []) > 1:
    print('  ⚠ 비행제어기 여러 대:', d['autopilots'])
if d.get('others'):
    print('  다른 구성원:', ', '.join(f'{s}.{c} {r}' for s, c, r in d['others']))
"
}

show
exit $?
