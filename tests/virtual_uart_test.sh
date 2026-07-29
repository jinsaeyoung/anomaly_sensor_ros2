#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 가상 UART 테스트 — 실제 센서 없이 노드 동작 검증
#
# socat으로 가상 시리얼 포트 쌍을 만들고, 한쪽에 모의 센서 데이터를 흘려보내
# ROS2 노드가 정상적으로 파싱/발행하는지 확인합니다.
#
# 사전 준비:  sudo apt install socat
# 사용법:     bash tests/virtual_uart_test.sh [thl100|wcm6800]
# ══════════════════════════════════════════════════════════════════════════════

set -u

SENSOR="${1:-thl100}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$SCRIPT_DIR")"

if ! command -v socat &> /dev/null; then
    echo "ERROR: socat이 필요합니다.  sudo apt install socat"
    exit 1
fi

# ROS setup.bash 는 미정의 변수를 참조하므로 이 구간만 set -u 해제
set +u
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u

PTY_A=/tmp/vuart_a
PTY_B=/tmp/vuart_b

echo "=========================================="
echo " 가상 UART 테스트: $SENSOR"
echo "=========================================="

# 가상 시리얼 포트 쌍 생성
socat -d -d pty,raw,echo=0,link=$PTY_A pty,raw,echo=0,link=$PTY_B 2>/dev/null &
SOCAT_PID=$!
sleep 1

if [ ! -e "$PTY_A" ]; then
    echo "ERROR: 가상 포트 생성 실패"
    kill $SOCAT_PID 2>/dev/null
    exit 1
fi
echo "가상 포트 생성됨: $PTY_A <-> $PTY_B"

cleanup() {
    echo ""
    echo "정리 중..."
    kill $NODE_PID 2>/dev/null
    kill $FEED_PID 2>/dev/null
    kill $SOCAT_PID 2>/dev/null
    rm -f $PTY_A $PTY_B
}
trap cleanup EXIT

# 노드 실행
if [ "$SENSOR" = "thl100" ]; then
    ros2 run thl100_sensor thl100_node --ros-args \
        -p port:=$PTY_A -p publish_rate_hz:=1.0 &
    NODE_PID=$!
    TOPIC=/thl100/data
else
    ros2 run wcm6800_sensor wcm6800_node --ros-args \
        -p port:=$PTY_A -p publish_rate_hz:=10.0 &
    NODE_PID=$!
    TOPIC=/wcm6800/data
fi
sleep 3

# 모의 센서 데이터 생성기
if [ "$SENSOR" = "thl100" ]; then
(
    seq_num=100
    while true; do
        printf "@T453,%d,28.5,38.3,236.5\r\n" $seq_num > $PTY_B
        seq_num=$(( (seq_num + 1) % 10000 ))
        sleep 1
    done
) &
else
(
    while true; do
        printf "+00450\r\n" > $PTY_B
        sleep 0.3
    done
) &
fi
FEED_PID=$!

echo ""
echo "모의 데이터 전송 시작 — 10초간 토픽 확인"
echo "------------------------------------------"
timeout 10 ros2 topic echo $TOPIC || true

echo ""
echo "------------------------------------------"
echo "발행 주기 확인 (5초)"
timeout 5 ros2 topic hz $TOPIC || true

echo ""
echo "=========================================="
echo " 테스트 완료"
echo "=========================================="
