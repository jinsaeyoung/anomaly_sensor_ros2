#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 수동 실행 전 서비스 충돌 확인
#
# systemd 서비스가 이미 센서를 구동 중이면 수동 실행 시
# mavros 중복 실행과 시리얼 포트 충돌이 발생합니다.
# start_drone / record_drone 앞에서 호출되어 경고합니다.
#
# 반환: 0 = 계속 진행 가능,  1 = 서비스 실행 중 (중단 권장)
# ══════════════════════════════════════════════════════════════════════════════

SERVICE_NAME="anomaly-sensor"

if ! systemctl list-unit-files "${SERVICE_NAME}.service" >/dev/null 2>&1; then
    exit 0    # 서비스 미등록 → 문제 없음
fi

if [ "$(systemctl is-active "$SERVICE_NAME" 2>/dev/null)" != "active" ]; then
    exit 0    # 서비스 미실행 → 문제 없음
fi

echo ""
echo "=============================================================="
echo " ⚠️  '$SERVICE_NAME' 서비스가 이미 실행 중입니다"
echo "=============================================================="
echo " 지금 수동으로 실행하면 다음 문제가 발생합니다."
echo "   - mavros 중복 실행 → 노드 크래시"
echo "   - 시리얼 포트 점유 충돌 → 센서 연결 실패"
echo ""
echo " 먼저 서비스를 중지하세요:"
echo "   sudo systemctl stop $SERVICE_NAME"
echo ""
echo " 서비스가 수집 중인 데이터를 확인만 하려면:"
echo "   check_record"
echo "=============================================================="
echo ""
exit 1
