#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 센서 통합 Launch 파일

실행 노드:
  - mavros              (FC 연동 — 원본 토픽 그대로 사용)
  - respeaker_full_node (마이크)
  - thl100_node         (온습도/조도, 기본 1Hz)
  - wcm6800_node        (전류계, 기본 10Hz)

실행 예시:
  ros2 launch drone_sensors drone_sensor_launch.py
  ros2 launch drone_sensors drone_sensor_launch.py fcu_url:=/dev/ttyACM0:115200
  ros2 launch drone_sensors drone_sensor_launch.py thl100_rate:=1.0 wcm6800_rate:=10.0
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# ══════════════════════════════════════════════════════════════════════════════
# 기본 장치 경로
#   장치 ID 확인: ls -la /dev/serial/by-id/
#   baud rate — USB: 115200 / TELEM2: 57600
#   재빌드 없이 launch 인자로 덮어쓸 수 있습니다.
# ══════════════════════════════════════════════════════════════════════════════
DEFAULT_FCU_URL = (
    '/dev/serial/by-id/'
    'usb-Hex_ProfiCNC_CubeOrange_310041000551303139333430-if00:115200'
)
DEFAULT_THL100_PORT = (
    '/dev/serial/by-id/'
    'usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0'
)
DEFAULT_WCM6800_PORT = (
    '/dev/serial/by-id/'
    'usb-Diwell_Electronics_CP2102N_USB_to_UART_Bridge_Controller_'
    '067d08f3ea94eb1187b304608dca6574-if00-port0'
)


def generate_launch_description():

    # ── Launch 인자 ───────────────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value=DEFAULT_FCU_URL,
        description='FC 연결 URL (예: /dev/ttyACM0:115200, /dev/ttyTHS1:57600)'
    )
    thl100_port_arg = DeclareLaunchArgument(
        'thl100_port',
        default_value=DEFAULT_THL100_PORT,
        description='THL100 온습도계 시리얼 포트'
    )
    wcm6800_port_arg = DeclareLaunchArgument(
        'wcm6800_port',
        default_value=DEFAULT_WCM6800_PORT,
        description='WCM6800 전류계 시리얼 포트'
    )
    respeaker_rate_arg = DeclareLaunchArgument(
        'respeaker_update_rate',
        default_value='50.0',
        description='ReSpeaker DoA/VAD 폴링 Hz'
    )
    thl100_rate_arg = DeclareLaunchArgument(
        'thl100_rate',
        default_value='1.0',
        description='THL100 발행 주기 Hz'
    )
    wcm6800_rate_arg = DeclareLaunchArgument(
        'wcm6800_rate',
        default_value='10.0',
        description='WCM6800 발행 주기 Hz'
    )

    # ── mavros (원본 토픽 그대로 사용) ────────────────────────────────
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch', 'apm.launch'
            )
        ),
        launch_arguments={
            'fcu_url':      LaunchConfiguration('fcu_url'),
            'fcu_protocol': 'v2.0',
            'gcs_url':      '',
            'log_output':   'log',
        }.items()
    )

    # ── ReSpeaker 마이크 ──────────────────────────────────────────────
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

    # ── THL100 온습도/조도 ────────────────────────────────────────────
    thl100_node = Node(
        package='thl100_sensor',
        executable='thl100_node',
        name='thl100_node',
        output='screen',
        parameters=[{
            'port':                LaunchConfiguration('thl100_port'),
            'baudrate':            9600,
            'publish_rate_hz':     LaunchConfiguration('thl100_rate'),
            'stale_timeout_sec':   5.0,
            'reconnect_delay_sec': 2.0,
            'drain_max_sec':       3.0,
        }]
    )

    # ── WCM6800 전류계 ────────────────────────────────────────────────
    wcm6800_node = Node(
        package='wcm6800_sensor',
        executable='wcm6800_node',
        name='wcm6800_node',
        output='screen',
        parameters=[{
            'port':                LaunchConfiguration('wcm6800_port'),
            'baudrate':            9600,
            'publish_rate_hz':     LaunchConfiguration('wcm6800_rate'),
            'stale_timeout_sec':   2.0,
            'reconnect_delay_sec': 2.0,
            'drain_max_sec':       3.0,
        }]
    )

    return LaunchDescription([
        fcu_url_arg,
        thl100_port_arg,
        wcm6800_port_arg,
        respeaker_rate_arg,
        thl100_rate_arg,
        wcm6800_rate_arg,

        mavros_launch,
        respeaker_node,
        thl100_node,
        wcm6800_node,
    ])
