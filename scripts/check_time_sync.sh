#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 시스템 시간 동기화 확인 스크립트
# start_drone 실행 전 호출되어 시간 정확성을 검증/보정합니다.
# ══════════════════════════════════════════════════════════════════════════════

echo "=========================================="
echo " 시간 동기화 상태 확인"
echo "=========================================="

SYNCED=$(timedatectl status | grep "System clock synchronized" | awk '{print $NF}')
CURRENT_TIME=$(date '+%Y-%m-%d %H:%M:%S')

echo "현재 시스템 시각: $CURRENT_TIME"
echo "NTP 동기화 상태:  $SYNCED"

if [ "$SYNCED" = "yes" ]; then
    echo "✅ 시간 동기화 정상 — 데이터 타임스탬프를 신뢰할 수 있습니다."
    exit 0
fi

echo "⚠️  시간이 NTP로 동기화되지 않았습니다."

# 인터넷 연결 확인 (1.1.1.1 핑 테스트, 1초 타임아웃)
if ping -c 1 -W 1 1.1.1.1 > /dev/null 2>&1; then
    echo "인터넷 연결 감지됨 — NTP 동기화를 시도합니다..."
    sudo timedatectl set-ntp true
    sudo systemctl restart systemd-timesyncd 2>/dev/null

    sleep 3
    SYNCED_RETRY=$(timedatectl status | grep "System clock synchronized" | awk '{print $NF}')

    if [ "$SYNCED_RETRY" = "yes" ]; then
        echo "✅ 동기화 완료 — 현재 시각: $(date '+%Y-%m-%d %H:%M:%S')"
        exit 0
    else
        echo "⚠️  동기화 시도했지만 아직 미완료 상태입니다. 잠시 후 다시 확인하세요."
        echo "    (timedatectl status 로 확인 가능)"
    fi
else
    echo "❌ 인터넷 연결이 없습니다."
    echo ""
    echo "    오프라인 환경에서는 시스템 시각이 마지막 부팅 시점 기준으로 부정확할 수 있습니다."
    echo "    수집되는 데이터의 '상대적 시간 간격'은 정확하지만,"
    echo "    '절대 시각(datetime)'은 실제와 다를 수 있다는 점을 감안하세요."
fi

echo "=========================================="
echo ""

# 이 스크립트는 경고만 출력하고 항상 0으로 종료 — start_drone 실행을 막지 않음
exit 0
