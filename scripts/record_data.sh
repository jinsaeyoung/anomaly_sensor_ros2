#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 센서 데이터 rosbag 녹화 스크립트
#
# 사용법:
#   record_drone 30     — 30초 녹화
#   record_drone         — 무제한 (Ctrl+C로 종료)
# ══════════════════════════════════════════════════════════════════════════════

source /opt/ros/humble/setup.bash
source ~/anomaly_sensor_ros2/install/setup.bash

SAVE_DIR=~/anomaly_data
mkdir -p $SAVE_DIR
cd $SAVE_DIR

BAG_NAME="anomaly_data_$(date +%Y%m%d_%H%M%S)"
DURATION=$1

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
echo " 저장 경로: $SAVE_DIR/$BAG_NAME"
echo " 토픽 수: ${#TOPICS[@]}"
if [ -n "$DURATION" ]; then
    echo " 녹화 시간: ${DURATION}초"
else
    echo " 녹화 시간: 무제한 (Ctrl+C로 종료)"
fi
echo "=========================================="

if [ -n "$DURATION" ]; then
    ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}" &
    BAG_PID=$!
    sleep "$DURATION"
    kill -SIGINT $BAG_PID
    wait $BAG_PID 2>/dev/null
else
    ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}"
fi

echo ""
echo "=========================================="
echo " 녹화 완료: $SAVE_DIR/$BAG_NAME"
echo "=========================================="
ros2 bag info "$BAG_NAME" 2>/dev/null || true
