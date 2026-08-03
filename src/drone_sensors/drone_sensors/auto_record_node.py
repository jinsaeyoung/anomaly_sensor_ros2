#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
자동 녹화 노드

/mavros/state 의 armed 상태를 감시하여 rosbag 녹화를 자동으로 시작/종료합니다.
온보드 헤드리스 운용을 전제로 하며, 디스크 여유 공간 확인과 bag 분할을 지원합니다.

동작:
  disarmed → armed : 녹화 시작
  armed → disarmed : post_disarm_sec 후 녹화 종료

발행 토픽:
  /auto_record/status  (String, JSON) — 현재 녹화 상태

수동 제어:
  ros2 topic pub --once /auto_record/command std_msgs/String "{data: start}"
  ros2 topic pub --once /auto_record/command std_msgs/String "{data: stop}"
"""

import os
import json
import shutil
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from mavros_msgs.msg import State


MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

DEFAULT_TOPICS = [
    # MAVROS: IMU / 자세 / 진동
    '/mavros/imu/data',
    '/mavros/imu/data_raw',
    '/mavros/imu/mag',
    '/mavros/vibration/raw/vibration',
    # MAVROS: RC 입력 / 모터 출력
    '/mavros/rc/in',
    '/mavros/rc/out',
    # MAVROS: 제어 목표값
    '/mavros/setpoint_raw/target_attitude',
    '/mavros/setpoint_raw/target_local',
    # MAVROS: 로컬 위치 / 속도 / 가속도
    '/mavros/local_position/pose',
    '/mavros/local_position/velocity_local',
    '/mavros/local_position/accel',
    # MAVROS: GPS / 고도
    '/mavros/global_position/raw/fix',
    '/mavros/global_position/raw/gps_vel',
    '/mavros/global_position/raw/satellites',
    '/mavros/global_position/global',
    '/mavros/global_position/rel_alt',
    '/mavros/gpsstatus/gps1/raw',
    '/mavros/altitude',
    # MAVROS: 전력 / ESC
    '/mavros/battery',
    '/mavros/battery2',
    '/mavros/esc_telemetry/telemetry',
    '/mavros/esc_status/status',
    # MAVROS: 기체 상태
    '/mavros/vfr_hud',
    '/mavros/state',
    '/mavros/extended_state',
    '/mavros/sys_status',
    '/mavros/statustext/recv',
    '/mavros/status_event',
    '/mavros/timesync_status',
    # MAVROS: 항법 / 환경
    '/mavros/nav_controller_output/output',
    '/mavros/wind_estimation',
    # 라벨 / 실험 메타데이터 (외부 발행)
    '/anomaly/label',
    '/test/metadata',
    # 시스템 진단
    '/diagnostics',
    # UART 센서
    '/thl100/data',
    '/thl100/raw',
    '/wcm6800/data',
    '/wcm6800/raw',
    # 마이크
    '/respeaker/doa',
    '/respeaker/vad',
    '/respeaker/energy',
]


class AutoRecordNode(Node):

    def __init__(self):
        super().__init__('auto_record_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('save_dir',          os.path.expanduser('~/anomaly_data'))
        self.declare_parameter('auto_on_arm',       True)
        self.declare_parameter('post_disarm_sec',   10.0)
        self.declare_parameter('min_free_gb',       2.0)
        self.declare_parameter('max_bag_duration',  300)     # 0이면 분할 안 함
        self.declare_parameter('topics',            DEFAULT_TOPICS)
        self.declare_parameter('name_prefix',       'flight')

        self.save_dir        = self.get_parameter('save_dir').value
        self.auto_on_arm     = self.get_parameter('auto_on_arm').value
        self.post_disarm_sec = self.get_parameter('post_disarm_sec').value
        self.min_free_gb     = self.get_parameter('min_free_gb').value
        self.max_bag_dur     = self.get_parameter('max_bag_duration').value
        self.topics          = list(self.get_parameter('topics').value)
        self.name_prefix     = self.get_parameter('name_prefix').value

        os.makedirs(self.save_dir, exist_ok=True)

        # ── 상태 ──────────────────────────────────────────────────────
        self._proc          = None    # rosbag record 프로세스
        self._bag_path      = None
        self._armed         = False
        self._stop_timer    = None
        self._start_time    = None

        # ── 통신 ──────────────────────────────────────────────────────
        self.pub_status = self.create_publisher(String, '/auto_record/status', 10)
        self.create_subscription(State,  '/mavros/state',           self._cb_state,   MAVROS_QOS)
        self.create_subscription(String, '/auto_record/command',    self._cb_command, 10)

        self.create_timer(5.0,  self._publish_status)
        self.create_timer(30.0, self._periodic_sync)

        mode = '자동(arm 연동)' if self.auto_on_arm else '수동'
        self.get_logger().info(
            f'AutoRecordNode 시작 [{mode}]\n'
            f'  저장 경로:      {self.save_dir}\n'
            f'  토픽 수:        {len(self.topics)}\n'
            f'  disarm 후 대기: {self.post_disarm_sec}초\n'
            f'  최소 여유 공간: {self.min_free_gb}GB\n'
            f'  bag 분할:       {self.max_bag_dur}초' +
            ('' if self.max_bag_dur else ' (분할 안 함)')
        )

    # ── 디스크 여유 확인 ──────────────────────────────────────────────
    def _free_gb(self):
        try:
            usage = shutil.disk_usage(self.save_dir)
            return usage.free / (1024 ** 3)
        except Exception:
            return -1.0

    # ── 녹화 시작 ─────────────────────────────────────────────────────
    def start_recording(self, reason=''):
        if self._proc is not None:
            self.get_logger().warn('이미 녹화 중입니다.')
            return False

        free = self._free_gb()
        if 0 <= free < self.min_free_gb:
            self.get_logger().error(
                f'디스크 여유 공간 부족: {free:.2f}GB < {self.min_free_gb}GB — 녹화 취소'
            )
            return False

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        name  = f'{self.name_prefix}_{stamp}'
        self._bag_path = os.path.join(self.save_dir, name)

        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path]
        if self.max_bag_dur and self.max_bag_dur > 0:
            cmd += ['--max-bag-duration', str(self.max_bag_dur)]
        cmd += self.topics

        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,   # 프로세스 그룹 분리 → SIGINT 전달용
            )
            self._start_time = self.get_clock().now()
            self.get_logger().info(
                f'녹화 시작 ({reason}) → {name}  [여유 {free:.1f}GB]'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'녹화 시작 실패: {e}')
            self._proc = None
            return False

    # ── 녹화 종료 ─────────────────────────────────────────────────────
    def stop_recording(self, reason=''):
        if self._proc is None:
            return False

        try:
            # SIGINT 로 안전 종료 (bag 파일 정상 마감)
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            self._proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('정상 종료 실패 — 강제 종료')
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f'종료 처리 중 오류: {e}')

        dur = 0.0
        if self._start_time is not None:
            dur = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        # 디스크 캐시 강제 기록 (전원 차단 시 데이터 손실 방지)
        try:
            subprocess.run(['sync'], timeout=10)
        except Exception:
            pass

        self.get_logger().info(
            f'녹화 종료 ({reason}) — {os.path.basename(self._bag_path or "")} '
            f'[{dur:.1f}초, 여유 {self._free_gb():.1f}GB]'
        )
        self._proc       = None
        self._start_time = None
        return True

    # ── armed 상태 감시 ───────────────────────────────────────────────
    def _cb_state(self, msg):
        if msg.armed == self._armed:
            return
        self._armed = msg.armed

        if not self.auto_on_arm:
            return

        if self._armed:
            # 예약된 종료가 있으면 취소 (재이륙)
            if self._stop_timer is not None:
                self._stop_timer.cancel()
                self._stop_timer = None
                self.get_logger().info('재arm 감지 — 종료 예약 취소, 녹화 계속')
                return
            self.start_recording('armed')
        else:
            if self._proc is None:
                return
            self.get_logger().info(
                f'disarm 감지 — {self.post_disarm_sec}초 후 녹화 종료'
            )
            self._stop_timer = self.create_timer(
                self.post_disarm_sec, self._delayed_stop
            )

    def _delayed_stop(self):
        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None
        self.stop_recording('disarmed')

    # ── 수동 명령 ─────────────────────────────────────────────────────
    def _cb_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.start_recording('수동 명령')
        elif cmd == 'stop':
            self.stop_recording('수동 명령')
        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd} (start | stop)')

    # ── 상태 발행 ─────────────────────────────────────────────────────
    def _publish_status(self):
        recording = self._proc is not None
        elapsed = 0.0
        if recording and self._start_time is not None:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        payload = {
            'recording': recording,
            'armed':     self._armed,
            'bag':       os.path.basename(self._bag_path) if recording else '',
            'elapsed_s': round(elapsed, 1),
            'free_gb':   round(self._free_gb(), 2),
        }
        m = String(); m.data = json.dumps(payload)
        self.pub_status.publish(m)

        # 녹화 중 디스크 부족 시 경고
        if recording and 0 <= payload['free_gb'] < self.min_free_gb:
            self.get_logger().error(
                f"디스크 여유 부족 ({payload['free_gb']}GB) — 녹화 중단",
                throttle_duration_sec=30.0
            )
            self.stop_recording('디스크 부족')

    # ── 주기적 디스크 flush (전원 차단 대비) ──────────────────────────
    def _periodic_sync(self):
        if self._proc is None:
            return
        try:
            subprocess.run(['sync'], timeout=10)
        except Exception:
            pass

    def destroy_node(self):
        self.stop_recording('노드 종료')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutoRecordNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[ERROR] {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
