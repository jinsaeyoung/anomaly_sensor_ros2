#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 센서 데이터 rosbag 녹화 스크립트
#
# 사용법:
#   record_drone 30     — 30초 녹화
#   record_drone         — 무제한 (Ctrl+C로 종료)
#
# 환경변수:
#   ANOMALY_WS    워크스페이스 경로 (기본: 스크립트 위치의 상위 폴더)
#   ANOMALY_DATA  저장 경로       (기본: ~/anomaly_data)
# ══════════════════════════════════════════════════════════════════════════════

set -u

# 스크립트 위치 기준으로 워크스페이스 자동 탐지 (하드코딩 제거)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"

# ROS 환경 로드
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "경고: $WS/install/setup.bash 를 찾을 수 없습니다. 빌드가 필요할 수 있습니다."
fi

SAVE_DIR="${ANOMALY_DATA:-$HOME/anomaly_data}"
mkdir -p "$SAVE_DIR"
cd "$SAVE_DIR"

BAG_NAME="anomaly_data_$(date +%Y%m%d_%H%M%S)"
DURATION="${1:-}"

TOPICS=(
    # ── MAVROS 원본 토픽 ───────────────────────────────────────────────
    /mavros/state
    /mavros/imu/data
    /mavros/imu/data_raw
    /mavros/imu/mag
    /mavros/local_position/pose
    /mavros/local_position/velocity_local
    /mavros/global_position/raw/fix
    /mavros/global_position/raw/gps_vel
    /mavros/vfr_hud
    /mavros/battery
    /mavros/rc/out
    /mavros/vibration/raw/vibration
    /mavros/setpoint_raw/target_attitude
    /mavros/setpoint_raw/target_local

    # ── THL100 온습도/조도 ─────────────────────────────────────────────
    /thl100/data
    /thl100/raw

    # ── WCM6800 전류계 ────────────────────────────────────────────────
    /wcm6800/data
    /wcm6800/raw

    # ── ReSpeaker 마이크 ──────────────────────────────────────────────
    /respeaker/doa
    /respeaker/vad
    /respeaker/energy
    # /respeaker/audio  # 용량 큼 — 필요 시 주석 해제
)

echo "=========================================="
echo " rosbag 녹화 시작"
echo " 워크스페이스: $WS"
echo " 저장 경로:   $SAVE_DIR/$BAG_NAME"
echo " 토픽 수:     ${#TOPICS[@]}"
if [ -n "$DURATION" ]; then
    echo " 녹화 시간:   ${DURATION}초"
else
    echo " 녹화 시간:   무제한 (Ctrl+C로 종료)"
fi
echo "=========================================="

if [ -n "$DURATION" ]; then
    ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}" &
    BAG_PID=$!
    sleep "$DURATION"
    kill -SIGINT $BAG_PID 2>/dev/null
    wait $BAG_PID 2>/dev/null
else
    ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}"
fi

echo ""
echo "=========================================="
echo " 녹화 완료: $SAVE_DIR/$BAG_NAME"
echo "=========================================="
ros2 bag info "$BAG_NAME" 2>/dev/null || true
