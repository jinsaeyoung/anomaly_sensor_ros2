#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 센서 데이터 rosbag 녹화 스크립트
# 사용법: ./record_data.sh [녹화시간(초), 생략시 무제한]
# ══════════════════════════════════════════════════════════════════════════════

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 저장 위치
SAVE_DIR=~/anomaly_data
mkdir -p $SAVE_DIR
cd $SAVE_DIR

# 파일명 (타임스탬프)
BAG_NAME="anomaly_data_$(date +%Y%m%d_%H%M%S)"

# 녹화할 토픽 목록
TOPICS=(
    # FC (drone_state 커스텀 토픽)
    /drone/local_ned/x /drone/local_ned/y /drone/local_ned/z
    /drone/local_ned_vel/vx /drone/local_ned_vel/vy /drone/local_ned_vel/vz
    /drone/accel/x /drone/accel/y /drone/accel/z
    /drone/attitude/roll /drone/attitude/pitch /drone/attitude/yaw
    /drone/attitude_rate/roll /drone/attitude_rate/pitch /drone/attitude_rate/yaw
    /drone/battery/voltage /drone/battery/current
    /drone/gps/latitude /drone/gps/longitude /drone/gps/altitude
    /drone/gyro/x /drone/gyro/y /drone/gyro/z
    /drone/mag/x /drone/mag/y /drone/mag/z
    /drone/rcout/c1 /drone/rcout/c2 /drone/rcout/c3 /drone/rcout/c4
    /drone/rcout/c5 /drone/rcout/c6 /drone/rcout/c7 /drone/rcout/c8

    # ReSpeaker
    /respeaker/doa
    /respeaker/vad
    /respeaker/energy
    # /respeaker/audio  # 용량 큼 - 필요시 주석 해제

    # THL100
    /thl100/temperature
    /thl100/humidity
    /thl100/light

    # WCM6800
    /wcm6800/current
    /wcm6800/current_type
)

echo "=========================================="
echo " rosbag 녹화 시작"
echo " 저장 경로: $SAVE_DIR/$BAG_NAME"
echo " 토픽 수: ${#TOPICS[@]}"
if [ -n "$1" ]; then
    echo " 녹화 시간: ${1}초"
else
    echo " 녹화 시간: 무제한 (Ctrl+C로 종료)"
fi
echo "=========================================="

if [ -n "$1" ]; then
    ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}" &
    BAG_PID=$!
    sleep "$1"
    kill -SIGINT $BAG_PID
    wait $BAG_PID 2>/dev/null
else
    ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}"
fi

echo ""
echo "=========================================="
echo " 녹화 완료: $SAVE_DIR/$BAG_NAME"
echo "=========================================="
ros2 bag info "$BAG_NAME"
