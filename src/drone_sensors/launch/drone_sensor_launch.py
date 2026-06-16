#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 센서 통합 Launch 파일
실행 노드:
  - mavros              (FC 연동)
  - respeaker_full_node (마이크 DoA/VAD/Audio/Energy)
  - thl100_node         (온습도/조도)
  - wcm6800_node        (전류계)
  - drone_state_node    (mavros 커스텀 토픽 재발행)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ══════════════════════════════════════════════════════════════════════
    # FC 연결 설정 — 연결 방식 변경 시 이 부분만 수정
    # ══════════════════════════════════════════════════════════════════════
    #
    # [현재] Micro 5pin USB 연결 (CubeOrange)
    FCU_URL = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_310041000551303139333430-if00:115200'
    #
    # [추후] TELEM2 UART 연결로 변경 시 아래 줄로 교체
    # FCU_URL = '/dev/serial/by-id/여기에_TELEM2_장치ID_입력:57600'
    #
    # [참고] 장치 ID 확인: ls -la /dev/serial/by-id/
    # [참고] baud rate — USB: 115200 / TELEM2: 57600
    # ══════════════════════════════════════════════════════════════════════

    # ── 파라미터 선언 ─────────────────────────────────────────────────────
    thl100_port_arg = DeclareLaunchArgument(
        'thl100_port',
        default_value='/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0',
        description='THL100 온습도계 시리얼 포트'
    )
    wcm6800_port_arg = DeclareLaunchArgument(
        'wcm6800_port',
        default_value='/dev/serial/by-id/usb-Diwell_Electronics_CP2102N_USB_to_UART_Bridge_Controller_067d08f3ea94eb1187b304608dca6574-if00-port0',
        description='WCM6800 전류계 시리얼 포트'
    )
    respeaker_update_rate_arg = DeclareLaunchArgument(
        'respeaker_update_rate',
        default_value='50.0',
        description='ReSpeaker DoA/VAD 폴링 Hz'
    )

    # ── mavros (IncludeLaunchDescription 방식 — 크래시 방지) ─────────────
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch',
                'apm.launch'
            )
        ),
        launch_arguments={
            'fcu_url':      FCU_URL,
            'fcu_protocol': 'v2.0',
            'gcs_url':      '',
    'log_output':   'log',    # ← screen 대신 log 파일로
        }.items()
    )

    # ── ReSpeaker 마이크 노드 ─────────────────────────────────────────────
    respeaker_node = Node(
        package='respeaker',
        executable='respeaker_full_node',
        name='respeaker_full_node',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('respeaker_update_rate'),
            'device_name': 'ReSpeaker',
        }]
    )

    # ── THL100 온습도/조도 노드 ───────────────────────────────────────────
    thl100_node = Node(
        package='thl100_sensor',
        executable='thl100_node',
        name='thl100_node',
        output='screen',
        parameters=[{
            'port':     LaunchConfiguration('thl100_port'),
            'baudrate': 9600,
        }]
    )

    # ── WCM6800 전류계 노드 ───────────────────────────────────────────────
    wcm6800_node = Node(
        package='wcm6800_sensor',
        executable='wcm6800_node',
        name='wcm6800_node',
        output='screen',
        parameters=[{
            'port':     LaunchConfiguration('wcm6800_port'),
            'baudrate': 9600,
        }]
    )

    # ── drone_state 커스텀 토픽 재발행 노드 ──────────────────────────────
    drone_state_node = Node(
        package='drone_state',
        executable='drone_state_node',
        name='drone_state_node',
        output='screen',
    )

    return LaunchDescription([
        # 파라미터 선언
        thl100_port_arg,
        wcm6800_port_arg,
        respeaker_update_rate_arg,

        # FC 연동 (mavros)
        mavros_launch,

        # 센서 노드
        respeaker_node,
        thl100_node,
        wcm6800_node,

        # 커스텀 토픽 재발행
        drone_state_node,
    ])
