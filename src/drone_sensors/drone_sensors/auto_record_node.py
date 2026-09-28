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
import sys
import threading
import time
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
    # MAVROS: 미션 / 지오펜스
    '/mavros/mission/waypoints',
    '/mavros/mission/reached',
    '/mavros/rallypoint/rallypoints',
    '/mavros/geofence/fences',
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
    '/respeaker/audio',   # 16kHz 6ch PCM 원본 (약 187 KB/s)
]


class AutoRecordNode(Node):

    def __init__(self):
        super().__init__('auto_record_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('save_dir',          os.path.expanduser('~/anomaly_data'))
        self.declare_parameter('auto_on_arm',       True)
        self.declare_parameter('post_disarm_sec',   10.0)
        self.declare_parameter('min_free_gb',       2.0)
        self.declare_parameter('max_bag_duration',  3000)    # 0이면 분할 안 함
        # FC 링크가 끊긴 채 이 시간이 지나면 녹화 종료 (추락·전원 차단 대비)
        # 그 전까지는 arm 상태를 알 수 없으므로 녹화를 유지합니다.
        self.declare_parameter('link_loss_stop_sec', 120.0)
        # rosbag 내부 캐시 크기(바이트).
        # 기본값(100MB)은 오디오 포함 시 5분치가 메모리에만 남아 있다가
        # 전원이 갑자기 끊기면 통째로 사라집니다. 작게 잡아 손실 구간을 줄입니다.
        # 전원은 레귤레이터로 안정 공급되고, 비행 중 차단은 상정하지 않습니다.
        # 다만 추락·전압 이상에 대비해 기본값(100MB, 약 5분치)보다는 작게 둡니다.
        # 8MB ≈ 25초분 (오디오 포함 약 340KB/s 기준)
        self.declare_parameter('max_cache_bytes', 8 * 1024 * 1024)
        self.declare_parameter('sync_interval_sec', 15.0)
        self.declare_parameter('topics',            DEFAULT_TOPICS)
        self.declare_parameter('name_prefix',       'flight')

        self.save_dir        = self.get_parameter('save_dir').value
        self.auto_on_arm     = self.get_parameter('auto_on_arm').value
        self.post_disarm_sec = self.get_parameter('post_disarm_sec').value
        self.min_free_gb     = self.get_parameter('min_free_gb').value
        self.max_bag_dur     = self.get_parameter('max_bag_duration').value
        self.link_loss_stop  = float(self.get_parameter('link_loss_stop_sec').value)
        self.max_cache_bytes = int(self.get_parameter('max_cache_bytes').value)
        self.sync_interval   = float(self.get_parameter('sync_interval_sec').value)
        self._rec_opts = self._supported_record_options()
        self._software = self._software_version()
        self.topics          = list(self.get_parameter('topics').value)
        self.name_prefix     = self.get_parameter('name_prefix').value

        os.makedirs(self.save_dir, exist_ok=True)

        # ── 상태 ──────────────────────────────────────────────────────
        self._proc          = None    # rosbag record 프로세스
        self._bag_path      = None
        self._health        = None    # 최근 센서 상태 (sensor_health_node)
        self._health_time   = None
        self._rec_log_fh    = None    # 비행별 녹화 로그 파일
        self._meta          = None    # 현재 녹화 메타데이터
        self._meta_path     = None
        self._events        = []      # 녹화 중 센서 상태 변화
        self._prev_groups   = {}
        self._fc_connected  = False
        self._last_state_mono = None
        self._link_lost_since = None
        self._safe_since    = None    # 녹화 마감 + 디스크 기록이 끝난 시각
        self._armed         = False
        self._stop_timer    = None
        self._start_time    = None

        # ── 통신 ──────────────────────────────────────────────────────
        self.pub_status = self.create_publisher(String, '/auto_record/status', 10)
        self.create_subscription(State,  '/mavros/state',           self._cb_state,   MAVROS_QOS)
        self.create_subscription(String, '/auto_record/command',    self._cb_command, 10)
        # 센서 연결 상태 — 녹화 시작 시 프리플라이트 체크에 사용
        self.create_subscription(String, '/sensor_health',          self._cb_health,  10)
        self.create_timer(5.0, self._link_watch)

        self.create_timer(5.0,  self._publish_status)
        self.create_timer(self.sync_interval, self._periodic_sync)

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

        # ── 프리플라이트 체크 ────────────────────────────────────
        # 녹화는 막지 않습니다. FC 데이터만이라도 확보하는 편이 낫고,
        # 센서가 늦게 붙는 경우도 있기 때문입니다.
        # 다만 상태를 명확히 남겨 사후에 바로 판별할 수 있게 합니다.
        ok, health, summary = self._preflight_check()
        if ok:
            self.get_logger().info(f'프리플라이트: {summary}')
        else:
            self.get_logger().warn(f'프리플라이트 경고 — {summary}')
            self.get_logger().warn('  센서 데이터가 누락된 채 녹화됩니다.')

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        name  = f'{self.name_prefix}_{stamp}'
        # 같은 초에 다시 녹화하면 이름이 겹치고, ros2 bag record 는
        # 이미 있는 폴더에 쓰지 못해 녹화가 실패합니다. 겹치면 번호를 붙입니다.
        n = 2
        while os.path.exists(os.path.join(self.save_dir, name)):
            name = f'{self.name_prefix}_{stamp}_{n}'
            n += 1
        self._bag_path = os.path.join(self.save_dir, name)

        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path]
        if self.max_bag_dur and self.max_bag_dur > 0 and '--max-bag-duration' in self._rec_opts:
            cmd += ['--max-bag-duration', str(self.max_bag_dur)]
        if self.max_cache_bytes > 0 and '--max-cache-size' in self._rec_opts:
            cmd += ['--max-cache-size', str(self.max_cache_bytes)]
        cmd += self.topics

        # 비행별 녹화 로그 — rosbag 의 구독·경고·오류 메시지를 bag 옆에 보관합니다.
        # 나중에 해당 비행의 데이터 문제를 추적할 때 이 파일만 보면 됩니다.
        log_path = os.path.join(self.save_dir, f'{name}_record.log')
        try:
            self._rec_log_fh = open(log_path, 'a', encoding='utf-8', buffering=1)
            self._rec_log_fh.write(
                f'# {datetime.now().isoformat(timespec="seconds")} 녹화 시작 ({reason})\n'
                f'# 프리플라이트: {summary}\n')
        except Exception as e:
            self.get_logger().warn(f'녹화 로그 파일 생성 실패: {e}')
            self._rec_log_fh = None

        try:
            out = self._rec_log_fh if self._rec_log_fh else subprocess.DEVNULL
            self._proc = subprocess.Popen(
                cmd,
                stdout=out,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,   # 프로세스 그룹 분리 → SIGINT 전달용
            )
            self._start_time = self.get_clock().now()
            self.get_logger().info(
                f'녹화 시작 ({reason}) → {name}  [여유 {free:.1f}GB]'
            )
            self._events = []
            self._prev_groups = dict(health or {})
            self._write_meta(name, reason, health, summary, ok)
            return True
        except Exception as e:
            self.get_logger().error(f'녹화 시작 실패: {e}')
            self._proc = None
            self._close_rec_log()
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

        self.get_logger().info(
            f'녹화 종료 ({reason}) — {os.path.basename(self._bag_path or "")} '
            f'[{dur:.1f}초, 여유 {self._free_gb():.1f}GB]'
        )

        # 종료 결과를 메타에 기록 — 비행 중 센서가 한 번이라도 이상했는지 바로 알 수 있게
        final = (self._health or {}).get('groups', {})
        self._update_meta(
            ended_at=datetime.now().isoformat(timespec='seconds'),
            duration_s=round(dur, 1),
            stop_reason=reason,
            rosbag_exit=self._proc.returncode if self._proc else None,
            sensor_status_end=final,
            sensor_events=self._events,
            sensors_ok_throughout=(not self._events and
                                   bool((self._meta or {}).get('preflight_ok'))),
        )
        if self._rec_log_fh:
            try:
                self._rec_log_fh.write(
                    f'# {datetime.now().isoformat(timespec="seconds")} 녹화 종료 '
                    f'({reason}, {dur:.1f}초, 센서 이벤트 {len(self._events)}건)\n')
            except Exception:
                pass
        self._close_rec_log()

        # 디스크 기록은 메타·로그까지 모두 쓴 뒤에 합니다.
        # 이 순서가 반대면 판정에 쓰는 _meta.json 과 _record.log 가
        # 아직 메모리에만 있는 상태로 전원이 내려갈 수 있습니다.
        try:
            subprocess.run(['sync'], timeout=10)
        except Exception:
            pass

        self._proc       = None
        self._start_time = None
        self._safe_since = time.monotonic()
        self.get_logger().info(
            '저장 완료 — 지금부터 전원을 내려도 안전합니다 '
            f'({os.path.basename(self._bag_path or "")})')

        # 저장이 끝난 뒤 별도 스레드에서 검증합니다. 도중에 전원을 내려도
        # 메타는 원자적으로 기록되므로 이전 내용이 깨지지 않습니다.
        if self._bag_path and self._meta_path:
            threading.Thread(target=self._verify_flight,
                             args=(self._bag_path, self._meta_path),
                             daemon=True).start()
        return True

    def _verify_flight(self, bag_path, meta_path):
        """
        착륙 직후 verify_bag 판정을 돌려 해당 비행의 메타에 기록합니다.

        검증 중에 다시 arm 하면 새 녹화의 메타가 '현재 메타' 가 되므로,
        반드시 넘겨받은 이 비행의 경로에만 씁니다.
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('drone_sensors'), 'scripts')
            if path not in sys.path:
                sys.path.insert(0, path)
            import verify_bag
            r = verify_bag.verify(bag_path, with_arm=False)
        except Exception as e:
            self.get_logger().warn(f'자동 검증 건너뜀 ({e}) — 필요하면 verify_bag 으로 확인하세요')
            return

        result = {
            'drone':    r['drone_level'],
            'reasons':  r['drone_reasons'][:5],
            'sensors':  r['sensor_state'],
            'unclosed': r['unclosed'],
            'checked_at': datetime.now().isoformat(timespec='seconds'),
        }
        try:
            meta = json.load(open(meta_path, encoding='utf-8'))
            meta['verify'] = result
            tmp = meta_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, meta_path)
        except Exception as e:
            self.get_logger().warn(f'검증 결과 기록 실패: {e}')
            return

        lacking = [g for g, s in result['sensors'].items() if s != '정상']
        text = (f"비행 검증 — 드론 {result['drone']}"
                + (f" ({', '.join(result['reasons'][:2])})" if result['reasons'] else '')
                + (f" / 외부센서 확인 필요: {', '.join(lacking)}" if lacking else ' / 외부센서 전부 정상'))
        (self.get_logger().info if result['drone'] == '정상' and not lacking
         else self.get_logger().warn)(text)

    def _close_rec_log(self):
        if self._rec_log_fh:
            try:
                self._rec_log_fh.flush()
                os.fsync(self._rec_log_fh.fileno())
            except Exception:
                pass
            try:
                self._rec_log_fh.close()
            except Exception:
                pass
        self._rec_log_fh = None

    # ── armed 상태 감시 ───────────────────────────────────────────────
    def _cb_state(self, msg):
        self._last_state_mono = time.monotonic()

        # FC 와 연결이 끊긴 동안 mavros 는 armed=False 를 발행합니다.
        # 이것을 disarm 으로 받아들이면 비행 중 링크가 잠깐만 끊겨도
        # 녹화가 종료되고 bag 이 쪼개지므로, 끊긴 동안의 arm 값은 무시합니다.
        if not msg.connected:
            if self._fc_connected:
                self._fc_connected = False
                self._on_link_change(False)
            return
        if not self._fc_connected:
            self._fc_connected = True
            self._on_link_change(True)

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

    def _on_link_change(self, up):
        if self._proc is None:
            return
        if up:
            self.get_logger().info('FC 연결 복구 — 녹화 계속')
        else:
            self.get_logger().warn(
                f'FC 연결 끊김 — arm 상태를 알 수 없어 녹화 유지 '
                f'({self.link_loss_stop:.0f}초 지속 시 종료)')
        elapsed = 0.0
        if self._start_time is not None:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        self._events.append({'t_s': round(elapsed, 1), 'sensor': 'FC_LINK',
                             'from': 'DOWN' if up else 'UP', 'to': 'UP' if up else 'DOWN'})
        self._update_meta(sensor_events=self._events)

    def _link_watch(self):
        """
        녹화 중 FC 링크 끊김이 오래 지속되면 녹화를 종료합니다.

        mavros 가 재기동 중이면 /mavros/state 자체가 오지 않으므로
        '끊김 상태' 와 '상태 미수신' 을 모두 끊김으로 봅니다.
        """
        if self._proc is None:
            self._link_lost_since = None
            return
        now = time.monotonic()
        stale = self._last_state_mono is None or now - self._last_state_mono > 5.0
        if self._fc_connected and not stale:
            self._link_lost_since = None
            return
        if self._link_lost_since is None:
            self._link_lost_since = now
            return
        lost = now - self._link_lost_since
        if lost >= self.link_loss_stop:
            self.get_logger().warn(f'FC 연결 끊김 {lost:.0f}초 지속 — 녹화 종료')
            if self._stop_timer is not None:
                self._stop_timer.cancel()
                self._stop_timer = None
            self.stop_recording('FC 연결 끊김 지속')
            self._armed = False
            self._link_lost_since = None

    def _delayed_stop(self):
        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None
        self.stop_recording('disarmed')

    # ── 코드 버전 ─────────────────────────────────────────────────────
    def _software_version(self):
        """
        이 데이터를 어느 커밋으로 수집했는지 기록합니다.
        여러 모듈을 운용하면 모듈마다 코드 버전이 다를 수 있어,
        나중에 데이터셋을 정리할 때 수집 조건을 구분하는 기준이 됩니다.
        """
        ws = os.environ.get('ANOMALY_WS', os.path.expanduser('~/anomaly_sensor_ros2'))
        info = {'commit': None, 'dirty': None}
        try:
            r = subprocess.run(['git', '-C', ws, 'rev-parse', '--short', 'HEAD'],
                               capture_output=True, timeout=5)
            if r.returncode == 0:
                info['commit'] = r.stdout.decode().strip()
                s = subprocess.run(['git', '-C', ws, 'status', '--porcelain', '--untracked-files=no'],
                                   capture_output=True, timeout=5)
                info['dirty'] = bool(s.stdout.strip())
        except Exception:
            pass
        return info

    # ── rosbag 옵션 지원 여부 확인 ────────────────────────────────────
    def _supported_record_options(self):
        """
        `ros2 bag record` 가 지원하는 옵션만 씁니다.

        없는 옵션을 넘기면 녹화 자체가 실패하므로, 시작할 때 한 번
        도움말에서 확인합니다. 확인에 실패하면 옵션을 붙이지 않습니다.
        """
        try:
            r = subprocess.run(['ros2', 'bag', 'record', '--help'],
                               capture_output=True, timeout=20)
            help_text = (r.stdout + r.stderr).decode('utf-8', 'ignore')
        except Exception as e:
            self.get_logger().warn(f'rosbag 옵션 확인 실패 ({e}) — 기본 옵션만 사용')
            return set()
        opts = {o for o in ('--max-bag-duration', '--max-cache-size') if o in help_text}
        missing = {'--max-bag-duration', '--max-cache-size'} - opts
        if missing:
            self.get_logger().warn(
                f'이 ROS2 버전이 지원하지 않는 옵션: {", ".join(sorted(missing))}')
        return opts

    # ── 센서 상태 수신 ────────────────────────────────────────────────
    def _cb_health(self, msg):
        try:
            self._health = json.loads(msg.data)
            self._health_time = self.get_clock().now().nanoseconds
        except Exception:
            return
        if self._proc is not None:
            self._track_health_change(self._health.get('groups', {}))

    def _track_health_change(self, groups):
        """
        녹화 중 센서 상태가 바뀌면 경고·기록합니다.

        서비스와 노드는 살아 있는데 데이터만 멈춘 경우를 잡기 위한 것입니다.
        (예: 비행 중 USB 접촉 불량으로 THL100 이 끊겼다 복구)
        변화는 onboard.log, 비행별 녹화 로그, 메타 JSON 에 모두 남습니다.
        """
        elapsed = 0.0
        if self._start_time is not None:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        changed = False
        for g, st in groups.items():
            prev = self._prev_groups.get(g)
            if prev == st:
                continue
            ev = {'t_s': round(elapsed, 1), 'sensor': g, 'from': prev, 'to': st}
            self._events.append(ev)
            changed = True
            text = f'녹화 중 센서 상태 변화 [{elapsed:.0f}초]: {g} {prev} → {st}'
            if st == 'OK':
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            if self._rec_log_fh:
                try:
                    self._rec_log_fh.write(f'# {text}\n')
                except Exception:
                    pass
        self._prev_groups = dict(groups)
        if changed:
            self._update_meta(sensor_events=self._events)

    # ── 프리플라이트 체크 ─────────────────────────────────────────────
    def _preflight_check(self):
        """
        녹화 시작 시 센서 상태를 확인하고 결과를 반환

        센서가 죽은 채로 이륙하면 비행이 끝난 뒤에야 데이터 누락을
        알게 되므로, arm 시점에 상태를 로그와 bag 메타데이터에 남깁니다.

        반환: (모두정상 여부, 상태 dict, 요약 문자열)
        """
        if self._health is None:
            return False, {}, 'sensor_health_node 미실행 — 센서 상태 확인 불가'

        # 상태가 너무 오래되었으면 신뢰할 수 없음
        if self._health_time is not None:
            age = (self.get_clock().now().nanoseconds - self._health_time) / 1e9
            if age > 15.0:
                return False, self._health.get('groups', {}), \
                       f'센서 상태 정보가 {age:.0f}초 전 것 — 신뢰 불가'

        groups = self._health.get('groups', {})
        bad = {g: s for g, s in groups.items() if s != 'OK'}

        if not bad:
            return True, groups, '전 센서 정상'

        desc = {
            'STALE':    '수신중단',
            'NO_DATA':  '데이터없음',
            'NO_TOPIC': '노드없음',
        }
        parts = [f'{g}({desc.get(s, s)})' for g, s in bad.items()]
        return False, groups, '이상: ' + ', '.join(parts)

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
            'sensors':   (self._health or {}).get('groups', {}),
            # 녹화가 없고 마지막 저장이 끝났으면 전원을 내려도 안전합니다
            'safe_power_off': (not recording) and self._safe_since is not None,
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

    # ── 녹화 메타데이터 기록 ──────────────────────────────────────────
    def _write_meta(self, name, reason, health, summary, ok):
        """
        bag 폴더 옆에 녹화 시점 정보를 남깁니다.

        나중에 데이터가 누락된 bag 을 발견했을 때
        '그때 센서가 살아있었는지' 를 로그를 뒤지지 않고 바로 알 수 있습니다.
        """
        meta = {
            'bag':            name,
            'started_at':     datetime.now().isoformat(timespec='seconds'),
            'trigger':        reason,
            'preflight_ok':   ok,
            'preflight':      summary,
            'sensor_status':  health,
            'free_gb':        round(self._free_gb(), 2),
            'topics':         len(self.topics),
            'record_log':     f'{name}_record.log',
            'software':       self._software,
        }
        self._meta = meta
        self._meta_path = os.path.join(self.save_dir, f'{name}_meta.json')
        self._flush_meta()

    def _update_meta(self, **kw):
        if self._meta is None:
            return
        self._meta.update(kw)
        self._flush_meta()

    def _flush_meta(self):
        try:
            tmp = self._meta_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(self._meta, f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())             # 내용을 디스크에 확정
            os.replace(tmp, self._meta_path)     # 쓰는 도중 전원이 끊겨도 깨지지 않게
            # 디렉터리 항목(파일 이름)도 확정해야 이름 바꾸기가 살아남습니다
            d = os.open(os.path.dirname(self._meta_path) or '.', os.O_RDONLY)
            try:
                os.fsync(d)
            finally:
                os.close(d)
        except Exception as e:
            self.get_logger().warn(f'메타데이터 기록 실패: {e}')

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
