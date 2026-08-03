#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 부팅 자동 실행 systemd 서비스 설치 스크립트
#
# 사용법:
#   bash scripts/install_service.sh           — 서비스 등록만 (개발용)
#                                               부팅 자동 실행 안 함, 수동 start 가능
#   bash scripts/install_service.sh enable    — 부팅 자동 실행 활성화 (운용용)
#   bash scripts/install_service.sh disable   — 부팅 자동 실행만 해제 (서비스 유지)
#   bash scripts/install_service.sh status    — 현재 상태 확인
#   bash scripts/install_service.sh remove    — 서비스 완전 제거
# ══════════════════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

SERVICE_NAME="anomaly-sensor"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
RUN_USER="$(id -un)"
RUN_HOME="$HOME"
SAVE_DIR="$RUN_HOME/anomaly_data"

# ── FC 연결 설정 ──────────────────────────────────────────────────────
# TELEM2 + USB-TTL 젠더(CH340) 기본. 환경변수로 덮어쓸 수 있습니다.
#   FCU_URL=/dev/ttyACM0:115200 bash scripts/install_service.sh
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:921600}"
WAIT_USB_SEC="${WAIT_USB_SEC:-60}"       # FC 장치 대기 최대 시간
FC_STABLE_SEC="${FC_STABLE_SEC:-6}"      # FC 장치 안정화 확인 시간

MODE="${1:-install}"

# ── 상태 확인 ─────────────────────────────────────────────────────────
if [ "$MODE" = "status" ]; then
    echo "=========================================="
    echo " 서비스 상태: $SERVICE_NAME"
    echo "=========================================="
    if [ ! -f "$SERVICE_FILE" ]; then
        echo "  등록 안 됨"
        exit 0
    fi
    echo -n "  현재 실행:      "; systemctl is-active  "$SERVICE_NAME" 2>/dev/null || echo "inactive"
    echo -n "  부팅 자동실행:  "; systemctl is-enabled "$SERVICE_NAME" 2>/dev/null || echo "disabled"
    exit 0
fi

# ── 제거 ──────────────────────────────────────────────────────────────
if [ "$MODE" = "remove" ]; then
    echo "서비스 제거 중..."
    sudo systemctl stop    "$SERVICE_NAME" 2>/dev/null || true
    sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
    sudo rm -f "$SERVICE_FILE"
    sudo systemctl daemon-reload
    echo "✅ 제거 완료"
    exit 0
fi

# ── 부팅 자동 실행 활성화 (운용 전환) ─────────────────────────────────
if [ "$MODE" = "enable" ]; then
    if [ ! -f "$SERVICE_FILE" ]; then
        echo "ERROR: 서비스가 등록되지 않았습니다. 먼저 인자 없이 실행하세요."
        exit 1
    fi
    sudo systemctl enable "$SERVICE_NAME"
    echo "✅ 부팅 자동 실행 활성화"
    echo "   이제 전원을 켜면 자동으로 데이터 수집이 시작됩니다."
    echo "   개발 작업 시에는 반드시 먼저 중지하세요:"
    echo "     sudo systemctl stop $SERVICE_NAME"
    exit 0
fi

# ── 부팅 자동 실행 해제 (개발 전환) ───────────────────────────────────
if [ "$MODE" = "disable" ]; then
    sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
    sudo systemctl stop    "$SERVICE_NAME" 2>/dev/null || true
    echo "✅ 부팅 자동 실행 해제 (서비스 파일은 유지)"
    echo "   필요할 때만 수동 실행: sudo systemctl start $SERVICE_NAME"
    exit 0
fi

echo "=========================================="
echo " systemd 서비스 설치"
echo "=========================================="
echo " 서비스명:     $SERVICE_NAME"
echo " 실행 사용자:  $RUN_USER"
echo " 워크스페이스: $WS"
echo " 저장 경로:    $SAVE_DIR"
echo " FC 연결:      $FCU_URL"
echo " FC 대기:      최대 ${WAIT_USB_SEC}초 / 안정화 ${FC_STABLE_SEC}초"
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

# FC 연결 설정
Environment="FCU_URL=$FCU_URL"
Environment="WAIT_USB_SEC=$WAIT_USB_SEC"
Environment="FC_STABLE_SEC=$FC_STABLE_SEC"

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

echo "✅ 서비스 등록 완료 (부팅 자동 실행은 아직 꺼져 있음)"
echo ""
echo "=========================================="
echo " 개발 중 (현재 상태)"
echo "=========================================="
echo "  부팅해도 자동 실행되지 않습니다."
echo "  필요할 때만 아래로 실행/중지하세요."
echo ""
echo "  sudo systemctl start   $SERVICE_NAME   # 지금 실행"
echo "  sudo systemctl stop    $SERVICE_NAME   # 중지 (bag 정상 마감)"
echo "  sudo systemctl status  $SERVICE_NAME   # 상태"
echo "  journalctl -u $SERVICE_NAME -f         # 실시간 로그"
echo ""
echo "=========================================="
echo " 실기체 운용 전환 (준비 완료 후)"
echo "=========================================="
echo "  bash scripts/install_service.sh enable   # 부팅 자동 실행 ON"
echo "  bash scripts/install_service.sh disable  # 다시 개발 모드로"
echo "  bash scripts/install_service.sh status   # 현재 모드 확인"
echo "  bash scripts/install_service.sh remove   # 완전 제거"
echo ""
echo "⚠️  주의: 서비스가 실행 중일 때 start_drone 을 함께 쓰면"
echo "    mavros 중복 실행과 시리얼 포트 충돌이 발생합니다."
echo "    수동 작업 전에 반드시 서비스를 중지하세요."
echo "=========================================="
