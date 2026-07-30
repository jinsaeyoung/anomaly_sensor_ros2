#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 부팅 자동 실행 systemd 서비스 설치 스크립트
#
# 사용법:
#   bash scripts/install_service.sh          — 설치 및 활성화
#   bash scripts/install_service.sh remove   — 제거
# ══════════════════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

SERVICE_NAME="anomaly-sensor"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
RUN_USER="$(id -un)"
RUN_HOME="$HOME"
SAVE_DIR="$RUN_HOME/anomaly_data"

# ── 제거 모드 ─────────────────────────────────────────────────────────
if [ "${1:-}" = "remove" ]; then
    echo "서비스 제거 중..."
    sudo systemctl stop    "$SERVICE_NAME" 2>/dev/null || true
    sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
    sudo rm -f "$SERVICE_FILE"
    sudo systemctl daemon-reload
    echo "✅ 제거 완료"
    exit 0
fi

echo "=========================================="
echo " systemd 서비스 설치"
echo "=========================================="
echo " 서비스명:     $SERVICE_NAME"
echo " 실행 사용자:  $RUN_USER"
echo " 워크스페이스: $WS"
echo " 저장 경로:    $SAVE_DIR"
echo "=========================================="

# ── 사전 확인 ─────────────────────────────────────────────────────────
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "ERROR: $WS/install/setup.bash 없음 — 먼저 bash install.sh 를 실행하세요."
    exit 1
fi
if [ ! -x "$WS/scripts/start_onboard.sh" ]; then
    chmod +x "$WS/scripts/start_onboard.sh" 2>/dev/null || {
        echo "ERROR: $WS/scripts/start_onboard.sh 없음"
        exit 1
    }
fi

mkdir -p "$SAVE_DIR"

# ── 서비스 파일 생성 ──────────────────────────────────────────────────
# KillSignal=SIGINT 가 핵심입니다.
#   rosbag 은 SIGINT 를 받아야 파일을 정상 마감합니다.
#   SIGTERM 으로 죽이면 bag 이 손상될 수 있습니다.
sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=Anomaly Sensor ROS2 Onboard Data Collection
Documentation=https://github.com/jinsaeyoung/anomaly_sensor_ros2
After=network-online.target multi-user.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
SupplementaryGroups=dialout
WorkingDirectory=$WS

Environment="HOME=$RUN_HOME"
Environment="ANOMALY_WS=$WS"
Environment="ANOMALY_DATA=$SAVE_DIR"
Environment="ROS_DOMAIN_ID=0"
Environment="PYTHONUNBUFFERED=1"

# 부팅 직후 USB 열거링 대기
ExecStartPre=/bin/sleep 15
ExecStart=/bin/bash $WS/scripts/start_onboard.sh

# rosbag 정상 마감을 위해 SIGINT 사용 (SIGTERM 은 bag 손상 위험)
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=40

Restart=on-failure
RestartSec=15

StandardOutput=append:$SAVE_DIR/onboard.log
StandardError=append:$SAVE_DIR/onboard.log

[Install]
WantedBy=multi-user.target
EOF

echo "✅ 서비스 파일 생성: $SERVICE_FILE"

# ── 등록 및 활성화 ────────────────────────────────────────────────────
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

echo "✅ 부팅 시 자동 실행 등록 완료"
echo ""
echo "=========================================="
echo " 사용 방법"
echo "=========================================="
echo "  sudo systemctl start   $SERVICE_NAME    # 지금 시작"
echo "  sudo systemctl stop    $SERVICE_NAME    # 중지 (bag 정상 마감)"
echo "  sudo systemctl status  $SERVICE_NAME    # 상태 확인"
echo "  sudo systemctl restart $SERVICE_NAME    # 재시작"
echo "  sudo systemctl disable $SERVICE_NAME    # 부팅 자동 실행 해제"
echo ""
echo "  journalctl -u $SERVICE_NAME -f          # 실시간 로그"
echo "  tail -f $SAVE_DIR/onboard.log           # 노드 출력 로그"
echo ""
echo "  bash scripts/install_service.sh remove  # 서비스 완전 제거"
echo "=========================================="
