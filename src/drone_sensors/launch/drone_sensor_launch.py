#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 센서 통합 Launch 파일

실행 노드:
  - fcu_manager_node    mavros 를 자식 프로세스로 띄우고 감시·복구
                        (포트·baud·SYSID 자동 탐지, USB 재삽입 대응)
  - respeaker_full_node 마이크 (장치 대기·재연결)
  - thl100_node         온습도/조도 (포트 자동 재탐색)
  - wcm6800_node        전류계     (포트 자동 재탐색)
  - sensor_health_node  센서 연결 상태 5초 주기 발행
  - auto_record_node    arm/disarm 자동 녹화 (use_auto_record:=true)

장치 구분:
  어떤 USB-UART 젠더를 어느 포트에 꽂아도 됩니다.
  각 포트의 수신 데이터 형식으로 FC / THL100 / WCM6800 을 판별하고,
  ReSpeaker 는 USB ID(2886:0018)로 찾습니다.

실행 예시:
  ros2 launch drone_sensors drone_sensor_launch.py use_auto_record:=true
  ros2 launch drone_sensors drone_sensor_launch.py tgt_system:=7          # 기체 지정
  ros2 launch drone_sensors drone_sensor_launch.py fcu_url:=/dev/ttyACM0:115200
"""

import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


# ══════════════════════════════════════════════════════════════════════════════
# 폴백 기본값
#   자동 탐지가 실패했을 때만 사용합니다. 평소에는 탐지 결과가 우선합니다.
#   각 노드는 실행 중에도 스스로 재탐색하므로 이 값이 틀려도 복구됩니다.
# ══════════════════════════════════════════════════════════════════════════════
DEFAULT_FCU_URL      = '/dev/ttyUSB0:921600'
DEFAULT_THL100_PORT  = '/dev/ttyUSB1'
DEFAULT_WCM6800_PORT = '/dev/ttyUSB2'


def _autodetect():
    """
    포트별 수신 데이터로 장치를 판별 (FC 는 baud·SYSID 까지)

    탐지 실패 시 빈 dict — 폴백 기본값을 쓰고, 노드들이 이후 재탐색합니다.
    """
    try:
        sys.path.insert(0, os.path.join(
            get_package_share_directory('drone_sensors'), 'scripts'))
        import serial_autodetect
        found = serial_autodetect.detect_devices(exclude_busy=True)
    except Exception as e:
        print(f'[launch] 자동 탐색 건너뜀 ({e}) — 기본값 사용')
        return {}

    print('[launch] 시리얼 자동 탐색 결과:')
    for k in ('fc', 'thl100', 'wcm6800'):
        print(f'[launch]   {k:8} → {found.get(k, "찾지 못함")}')
    fi = found.get('fc_info')
    if fi:
        print(f"[launch]   FC 상세 : {fi['baud']}bps  SYSID {fi['sysid']}.{fi['compid']}  "
              f"MAVLink {fi['protocol']}  mavros ID {fi['sysid']}.{fi['my_compid']}")
        if len(fi['autopilots']) > 1:
            print('[launch]   ⚠ 비행제어기 여러 대 감지 — 필요 시 tgt_system 지정')
        if fi['signed']:
            print('[launch]   ⚠ MAVLink 서명 사용 중 — mavros 요청이 거부될 수 있음')
    return found


def generate_launch_description():

    # ── 자동 탐지 ─────────────────────────────────────────────────────
    detected = {}
    if os.environ.get('ANOMALY_AUTODETECT', '1') != '0':
        detected = _autodetect()

    fi = detected.get('fc_info') or {}
    fcu_url_default = (f"{detected['fc']}:{fi['baud']}" if 'fc' in detected and fi
                       else DEFAULT_FCU_URL)
    tgt_default   = str(fi['sysid'])     if fi else 'auto'
    tcomp_default = str(fi['compid'])    if fi else 'auto'
    comp_default  = str(fi['my_compid']) if fi else 'auto'
    proto_default = fi.get('protocol', 'auto') if fi else 'auto'

    thl100_default  = detected.get('thl100',  DEFAULT_THL100_PORT)
    wcm6800_default = detected.get('wcm6800', DEFAULT_WCM6800_PORT)

    # ── 인자 ──────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('fcu_url', default_value=fcu_url_default,
            description='FC 연결 (탐지 결과 기본). 예: /dev/ttyACM0:115200'),
        DeclareLaunchArgument('tgt_system', default_value=tgt_default,
            description='대상 기체 SYSID. auto 면 HEARTBEAT 로 자동 탐지'),
        DeclareLaunchArgument('tgt_component', default_value=tcomp_default,
            description='대상 컴포넌트 ID (보통 1)'),
        DeclareLaunchArgument('mavros_component_id', default_value=comp_default,
            description='mavros 자신의 COMPID (기본 191, 이미 쓰이면 194~196)'),
        DeclareLaunchArgument('fcu_protocol', default_value=proto_default,
            description='MAVLink 버전 (auto | v1.0 | v2.0)'),
        DeclareLaunchArgument('fcu_managed', default_value='true',
            description='true: 관리 노드가 mavros 기동·복구 / false: 기존 방식'),
        DeclareLaunchArgument('fcu_rediscover', default_value='true',
            description='FC 연결이 끊기면 포트·baud·SYSID 재탐지'),
        DeclareLaunchArgument('thl100_port', default_value=thl100_default,
            description='THL100 시리얼 포트 (탐지 결과 기본)'),
        DeclareLaunchArgument('wcm6800_port', default_value=wcm6800_default,
            description='WCM6800 시리얼 포트 (탐지 결과 기본)'),
        DeclareLaunchArgument('respeaker_update_rate', default_value='50.0',
            description='ReSpeaker DoA/VAD 폴링 Hz'),
        DeclareLaunchArgument('thl100_rate', default_value='1.0',
            description='THL100 발행 주기 Hz'),
        DeclareLaunchArgument('wcm6800_rate', default_value='10.0',
            description='WCM6800 발행 주기 Hz'),
        DeclareLaunchArgument('use_auto_record', default_value='false',
            description='arm/disarm 연동 자동 녹화 (온보드 운용 시 true)'),
        DeclareLaunchArgument('save_dir', default_value=os.path.expanduser('~/anomaly_data'),
            description='rosbag 저장 경로'),
        DeclareLaunchArgument('post_disarm_sec', default_value='10.0',
            description='disarm 후 추가 녹화 시간(초)'),
        DeclareLaunchArgument('max_bag_duration', default_value='3000',
            description='bag 분할 주기(초). 0이면 분할하지 않음'),
        DeclareLaunchArgument('min_free_gb', default_value='2.0',
            description='녹화에 필요한 최소 디스크 여유(GB)'),
    ]

    # ── mavros 설정 파일 ──────────────────────────────────────────────
    # mavros 기본 pluginlist 는 vibration / altitude 를 denylist 로 막으므로
    # 패키지 내 커스텀 pluginlist 를 씁니다.
    mavros_share = get_package_share_directory('mavros')
    pluginlists_yaml = os.path.join(
        get_package_share_directory('drone_sensors'), 'config', 'apm_pluginlists.yaml')
    config_yaml = os.path.join(mavros_share, 'launch', 'apm_config.yaml')

    # ── FC 관리 노드 (기본) ───────────────────────────────────────────
    # mavros 는 시작 시 경로·SYSID 를 고정해 스스로 복구하지 못하므로
    # 관리 노드가 mavros 를 띄우고, 끊기면 재탐지해 다시 띄웁니다.
    fcu_manager = Node(
        package='drone_sensors',
        executable='fcu_manager_node',
        name='fcu_manager_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('fcu_managed')),
        parameters=[{
            'fcu_url':          LaunchConfiguration('fcu_url'),
            'tgt_system':       LaunchConfiguration('tgt_system'),
            'tgt_component':    LaunchConfiguration('tgt_component'),
            'component_id':     LaunchConfiguration('mavros_component_id'),
            'fcu_protocol':     LaunchConfiguration('fcu_protocol'),
            'rediscover':       LaunchConfiguration('fcu_rediscover'),
            'pluginlists_yaml': pluginlists_yaml,
            'config_yaml':      config_yaml,
        }]
    )

    # ── 기존 방식 (fcu_managed:=false) ────────────────────────────────
    # 재연결·SYSID 자동 전환이 되지 않습니다. 비교·디버깅용으로 남겨둡니다.
    def _or_default(name, default):
        return PythonExpression([
            "'", default, "' if '", LaunchConfiguration(name), "' == 'auto' else '",
            LaunchConfiguration(name), "'"])

    mavros_legacy = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(mavros_share, 'launch', 'node.launch')),
        condition=UnlessCondition(LaunchConfiguration('fcu_managed')),
        launch_arguments={
            'fcu_url':          LaunchConfiguration('fcu_url'),
            'gcs_url':          '',
            'tgt_system':       _or_default('tgt_system', '1'),
            'tgt_component':    _or_default('tgt_component', '1'),
            'pluginlists_yaml': pluginlists_yaml,
            'config_yaml':      config_yaml,
            'fcu_protocol':     _or_default('fcu_protocol', 'v2.0'),
            'respawn_mavros':   'false',
            'namespace':        'mavros',
        }.items()
    )

    # ── 센서 노드 ─────────────────────────────────────────────────────
    respeaker_node = Node(
        package='respeaker', executable='respeaker_full_node',
        name='respeaker_full_node', output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('respeaker_update_rate'),
            'device_name': 'ReSpeaker',
        }]
    )

    # autodetect=True: 지정 포트에서 정상 패킷이 끊기면 비어 있는 포트를
    # 다시 탐색해 자기 장치를 찾아갑니다 (젠더·포트가 바뀌어도 복구).
    thl100_node = Node(
        package='thl100_sensor', executable='thl100_node',
        name='thl100_node', output='screen',
        parameters=[{
            'port':                 LaunchConfiguration('thl100_port'),
            'baudrate':             9600,
            'publish_rate_hz':      LaunchConfiguration('thl100_rate'),
            'stale_timeout_sec':    5.0,
            'reconnect_delay_sec':  2.0,
            'drain_max_sec':        3.0,
            'autodetect':           True,
            'device_key':           'thl100',
            'identity_timeout_sec': 15.0,    # 1Hz 센서 — 15초간 정상 패킷 없으면 재탐색
        }]
    )

    wcm6800_node = Node(
        package='wcm6800_sensor', executable='wcm6800_node',
        name='wcm6800_node', output='screen',
        parameters=[{
            'port':                 LaunchConfiguration('wcm6800_port'),
            'baudrate':             9600,
            'publish_rate_hz':      LaunchConfiguration('wcm6800_rate'),
            'stale_timeout_sec':    2.0,
            'reconnect_delay_sec':  2.0,
            'drain_max_sec':        3.0,
            'autodetect':           True,
            'device_key':           'wcm6800',
            'identity_timeout_sec': 8.0,     # 3Hz 센서
        }]
    )

    sensor_health_node = Node(
        package='drone_sensors', executable='sensor_health_node',
        name='sensor_health_node', output='screen',
        parameters=[{'publish_rate_hz': 0.2}]
    )

    auto_record_node = Node(
        package='drone_sensors', executable='auto_record_node',
        name='auto_record_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_auto_record')),
        parameters=[{
            'save_dir':         LaunchConfiguration('save_dir'),
            'auto_on_arm':      True,
            'post_disarm_sec':  LaunchConfiguration('post_disarm_sec'),
            'max_bag_duration': LaunchConfiguration('max_bag_duration'),
            'min_free_gb':      LaunchConfiguration('min_free_gb'),
            'name_prefix':      'flight',
        }]
    )

    return LaunchDescription(args + [
        fcu_manager,
        mavros_legacy,
        respeaker_node,
        thl100_node,
        wcm6800_node,
        sensor_health_node,
        auto_record_node,
    ])
