#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FC(mavros) 관리 노드

mavros 를 launch 에서 직접 띄우지 않고, 이 노드가 자식 프로세스로 띄워 관리합니다.

왜 필요한가:
  mavros 는 시작할 때 fcu_url 과 tgt_system 을 한 번 정하면 바꾸지 않습니다.
    - USB 를 뽑았다 꽂으면 ttyUSB 번호가 바뀌어도 옛 경로만 재시도합니다.
    - FC 의 SYSID 가 1 이 아니면 HEARTBEAT 를 받아도 '연결됨'이 되지 않습니다.
  어느 쪽이든 mavros 스스로는 복구하지 못하므로, 바깥에서 새 설정을 찾아
  다시 띄워 주어야 합니다.

동작:
  1) 시작: launch 가 넘겨준 탐지 결과(포트·baud·SYSID)로 mavros 기동
           탐지 결과가 없으면 직접 탐지
  2) 감시: /mavros/state 로 연결 상태 확인
  3) 복구: 일정 시간 연결이 끊기면 mavros 만 종료하고
           비어 있는 포트를 재탐지해 새 설정으로 다시 기동
           (센서 노드·녹화 프로세스는 건드리지 않음)

FC 를 수정하지 않습니다:
  SYSID·baud 는 FC 가 보내는 HEARTBEAT 를 '듣기만' 해서 알아냅니다.
  탐지 과정에서 FC 로 아무것도 송신하지 않습니다.

mavros 의 자기 ID (MAVLink 컴패니언 규약):
  system_id    = 대상 기체 SYSID   (기체 1 이면 1, 7 이면 7)
  component_id = 191 (ONBOARD_COMPUTER), 이미 쓰이면 194/195/196
  → GCS 에는 '기체 N 의 컴패니언'으로 보이고, 별도 기체로 오인되지 않습니다.
  → 255(GCS 관례 번호)는 쓰지 않습니다. SYSID_MYGCS 와 겹치면
    GCS 페일세이프 판정에 영향을 줄 수 있기 때문입니다.

발행 토픽:
  /fcu_manager/status (String, JSON)
"""

import os
import sys
import json
import time
import signal
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from mavros_msgs.msg import State
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory, get_package_prefix


MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _load_autodetect():
    """drone_sensors 패키지에 설치된 serial_autodetect 모듈을 불러옵니다."""
    path = os.path.join(get_package_share_directory('drone_sensors'), 'scripts')
    if path not in sys.path:
        sys.path.insert(0, path)
    import serial_autodetect
    return serial_autodetect


def _as_int(v, default=0):
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return default


class FcuManagerNode(Node):

    def __init__(self):
        super().__init__('fcu_manager_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        # launch 의 탐지 결과가 그대로 들어옵니다. 비어 있거나 0 이면 자동.
        p = self.declare_parameter
        # launch 는 '7' 같은 값을 정수로, 'auto' 는 문자열로 넘깁니다.
        # Humble 은 선언 타입과 다르면 노드가 죽으므로 동적 타입으로 받습니다.
        dyn = ParameterDescriptor(dynamic_typing=True)
        p('fcu_url',          '')                       # '/dev/xxx:921600' 또는 ''
        p('tgt_system',       'auto', dyn)              # 'auto' 또는 1~254
        p('tgt_component',    'auto', dyn)
        p('system_id',        'auto', dyn)              # mavros 자신의 SYSID
        p('component_id',     'auto', dyn)              # mavros 자신의 COMPID
        p('fcu_protocol',     'auto', dyn)              # 'auto' | 'v1.0' | 'v2.0'
        p('pluginlists_yaml', '')
        p('config_yaml',      '')
        p('check_interval',   3.0)      # 연결 점검 주기(초)
        p('startup_grace',    25.0)     # 기동 후 판정 유예 (파라미터 수신 시간)
        p('disconnect_sec',   12.0)     # 이 시간 이상 끊기면 복구
        p('rediscover',       True, dyn)  # 끊기면 포트를 다시 탐지할지
        p('recover_while_armed', True)  # 비행 중에도 복구할지

        g = lambda k: self.get_parameter(k).value
        self._user_url    = str(g('fcu_url') or '')
        self._user_tgt    = str(g('tgt_system'))
        self._user_tcomp  = str(g('tgt_component'))
        self._user_sys    = str(g('system_id'))
        self._user_comp   = str(g('component_id'))
        self._user_proto  = str(g('fcu_protocol'))
        self.check_interval = float(g('check_interval'))
        self.startup_grace  = float(g('startup_grace'))
        self.disconnect_sec = float(g('disconnect_sec'))
        self.rediscover     = str(g('rediscover')).lower() not in ('false', '0', 'no')
        self.recover_armed  = bool(g('recover_while_armed'))

        share = get_package_share_directory('drone_sensors')
        mshare = get_package_share_directory('mavros')
        self.pluginlists = g('pluginlists_yaml') or os.path.join(share, 'config', 'apm_pluginlists.yaml')
        self.config_yaml = g('config_yaml') or os.path.join(mshare, 'launch', 'apm_config.yaml')
        self.mavros_exe  = os.path.join(get_package_prefix('mavros'), 'lib', 'mavros', 'mavros_node')

        try:
            self.sa = _load_autodetect()
        except Exception as e:
            self.sa = None
            self.get_logger().warn(f'serial_autodetect 로드 실패 ({e}) — 재탐지 불가')

        # ── 상태 ──────────────────────────────────────────────────────
        self._proc      = None
        self._cfg       = None     # 현재 mavros 설정
        self._spawned   = 0.0
        self._last_ok   = None     # 마지막으로 connected=True 를 본 시각
        self._state_t   = None
        self._connected = False
        self._armed     = False
        self._restarts  = 0
        self._phase     = 'INIT'
        self._note      = ''
        self._link      = {}       # 링크 구성원 (탐지 결과)
        self._lock      = threading.Lock()
        self._stop      = threading.Event()

        self.pub = self.create_publisher(String, '/fcu_manager/status', 10)
        self.create_subscription(State, '/mavros/state', self._cb_state, MAVROS_QOS)
        self.create_timer(2.0, self._publish)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    # ══════════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════════
    def _cb_state(self, msg):
        now = time.monotonic()
        with self._lock:
            self._state_t = now
            self._connected = bool(msg.connected)
            self._armed = bool(msg.armed)
            if msg.connected:
                self._last_ok = now

    # ══════════════════════════════════════════════════════════════════
    # 설정 결정
    # ══════════════════════════════════════════════════════════════════
    def _initial_cfg(self):
        """launch 가 넘겨준 값으로 첫 설정 구성"""
        url = self._user_url
        if not url or ':' not in url:
            return None
        port, baud = url.rsplit(':', 1)
        tgt = _as_int(self._user_tgt, 0)
        if not tgt:
            return None          # SYSID 를 모르면 탐지부터 합니다
        return self._build_cfg(port, _as_int(baud, 921600), tgt,
                               _as_int(self._user_tcomp, 1) or 1,
                               self._user_proto if self._user_proto.startswith('v') else 'v2.0',
                               my_comp=None, source='launch')

    def _build_cfg(self, port, baud, sysid, compid, proto, my_comp=None, source=''):
        my_sys = _as_int(self._user_sys, 0) or sysid
        my_c   = _as_int(self._user_comp, 0) or my_comp or 191
        if my_sys == 255:
            self.get_logger().warn(
                'system_id 255 는 GCS 관례 번호라 사용하지 않습니다 — 대상 SYSID 로 대체')
            my_sys = sysid
        return {
            'port': port, 'baud': baud,
            'tgt_system': sysid, 'tgt_component': compid,
            'system_id': my_sys, 'component_id': my_c,
            'protocol': proto, 'source': source,
        }

    def _detect_cfg(self):
        """비어 있는 포트에서 FC 를 탐지해 설정 구성"""
        if self.sa is None:
            return None
        self._phase = 'SEARCHING'
        self._note = 'FC 탐색 중'
        try:
            info = self.sa.find_fc(exclude_busy=True)
        except Exception as e:
            self.get_logger().warn(f'FC 탐지 오류: {e}')
            return None
        if not info:
            return None

        # 사용자가 SYSID 를 고정했다면 그 기체를 우선합니다.
        forced = _as_int(self._user_tgt, 0)
        sysid, compid = info['sysid'], info['compid']
        if forced and forced != sysid:
            seen = [s for s, _, _ in info['autopilots']]
            if forced in seen:
                sysid, compid = forced, 1
            else:
                self.get_logger().warn(
                    f'지정한 tgt_system={forced} 는 링크에 없습니다 '
                    f'(보이는 기체: {seen}) — 탐지된 {sysid} 사용')

        proto = self._user_proto if self._user_proto.startswith('v') else info['protocol']
        self._link = info
        self._report_link(info, sysid)
        return self._build_cfg(info['port'], info['baud'], sysid, compid, proto,
                               my_comp=info.get('my_compid'), source='detect')

    def _report_link(self, info, sysid):
        log = self.get_logger()
        log.info(f"FC 탐지: {info['port']} @ {info['baud']}bps  "
                 f"SYSID {sysid}.{info['compid']}  MAVLink {info['protocol']}")
        if len(info['autopilots']) > 1:
            log.warn('비행제어기가 여러 대 보입니다: '
                     + ', '.join(f'{s}.{c}({n}회)' for s, c, n in info['autopilots'])
                     + ' — 다른 기체라면 tgt_system 을 직접 지정하세요')
        if info['others']:
            log.info('링크의 다른 구성원(대상 제외): '
                     + ', '.join(f'{s}.{c} {r}' for s, c, r in info['others']))
        if info['signed']:
            log.warn('MAVLink 서명(signing) 사용 중입니다. 수신 데이터는 기록되지만 '
                     'mavros 의 요청(파라미터 조회 등)은 FC 가 거부할 수 있습니다.')

    # ══════════════════════════════════════════════════════════════════
    # mavros 프로세스
    # ══════════════════════════════════════════════════════════════════
    def _spawn(self, cfg):
        cmd = [
            self.mavros_exe, '--ros-args',
            '-r', '__ns:=/mavros',
            '--params-file', self.pluginlists,
            '--params-file', self.config_yaml,
            # 아래 값이 yaml 보다 뒤에 와야 우선 적용됩니다.
            '-p', f"fcu_url:={cfg['port']}:{cfg['baud']}",
            '-p', f"tgt_system:={cfg['tgt_system']}",
            '-p', f"tgt_component:={cfg['tgt_component']}",
            '-p', f"system_id:={cfg['system_id']}",
            '-p', f"component_id:={cfg['component_id']}",
            '-p', f"fcu_protocol:={cfg['protocol']}",
        ]
        self.get_logger().info(
            f"mavros 기동: {cfg['port']}:{cfg['baud']}  "
            f"대상 {cfg['tgt_system']}.{cfg['tgt_component']}  "
            f"자기 ID {cfg['system_id']}.{cfg['component_id']}  ({cfg['source']})")
        try:
            # 새 세션으로 띄워 종료 시 자식까지 한 번에 정리합니다.
            self._proc = subprocess.Popen(cmd, start_new_session=True)
        except Exception as e:
            self.get_logger().error(f'mavros 실행 실패: {e}')
            self._proc = None
            return False
        self._cfg = cfg
        self._spawned = time.monotonic()
        with self._lock:
            self._last_ok = None
            self._connected = False
        self._phase = 'STARTING'
        self._note = 'HEARTBEAT 대기'
        return True

    def _kill(self, reason=''):
        proc, self._proc = self._proc, None
        if proc is None or proc.poll() is not None:
            return
        self.get_logger().warn(f'mavros 종료 ({reason})')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5)
            except Exception:
                pass
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════
    # 관리 루프 (블로킹 탐지를 ROS 콜백과 분리)
    # ══════════════════════════════════════════════════════════════════
    def _run(self):
        backoff = 3.0
        cfg = self._initial_cfg()

        while not self._stop.is_set():
            # ── mavros 가 없으면 설정을 정하고 기동 ──────────────────
            if self._proc is None:
                if cfg is None:
                    cfg = self._detect_cfg()
                if cfg is None:
                    self._phase = 'NO_DEVICE'
                    self._note = f'FC 를 찾지 못함 — {backoff:.0f}초 후 재탐색'
                    self.get_logger().warn(self._note, throttle_duration_sec=30.0)
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2, 30.0)
                    continue
                backoff = 3.0
                self._spawn(cfg)
                cfg = None
                self._stop.wait(self.check_interval)
                continue

            # ── 프로세스가 스스로 죽었으면 재기동 ────────────────────
            if self._proc.poll() is not None:
                self.get_logger().warn(f'mavros 가 종료됨 (코드 {self._proc.returncode}) — 재기동')
                self._proc = None
                self._restarts += 1
                cfg = self._cfg if not self.rediscover else None
                self._stop.wait(2.0)
                continue

            now = time.monotonic()
            with self._lock:
                last_ok, connected, armed = self._last_ok, self._connected, self._armed

            # ── 기동 직후는 유예 ────────────────────────────────────
            if now - self._spawned < self.startup_grace and not connected:
                self._stop.wait(self.check_interval)
                continue

            if connected:
                self._phase = 'CONNECTED'
                self._note = ''
                self._stop.wait(self.check_interval)
                continue

            # ── 끊김 판정 ────────────────────────────────────────────
            since = now - (last_ok if last_ok else self._spawned)
            self._phase = 'DISCONNECTED'
            self._note = f'연결 끊김 {since:.0f}초'
            if since < self.disconnect_sec:
                self._stop.wait(self.check_interval)
                continue

            if armed and not self.recover_armed:
                self._note = '비행 중 — 복구 보류(recover_while_armed=false)'
                self._stop.wait(self.check_interval)
                continue

            # ── 복구: mavros 만 내리고 재탐지 ────────────────────────
            port_exists = os.path.exists(self._cfg['port']) if self._cfg else False
            self._phase = 'RECOVERING'
            self._note = ('장치 번호 변경/분리 의심' if not port_exists
                          else 'HEARTBEAT 없음 — baud/SYSID 재확인')
            self.get_logger().warn(f'FC 복구 시작: {self._note}')
            self._kill('재연결')
            self._restarts += 1
            cfg = None if self.rediscover else self._cfg
            self._stop.wait(1.0)

    # ══════════════════════════════════════════════════════════════════
    # 상태 발행
    # ══════════════════════════════════════════════════════════════════
    def _publish(self):
        c = self._cfg or {}
        with self._lock:
            connected = self._connected
            st_age = (time.monotonic() - self._state_t) if self._state_t else None
        payload = {
            'phase':        self._phase,
            'note':         self._note,
            'connected':    connected,
            'port':         c.get('port'),
            'baud':         c.get('baud'),
            'tgt':          f"{c.get('tgt_system')}.{c.get('tgt_component')}" if c else None,
            'self_id':      f"{c.get('system_id')}.{c.get('component_id')}" if c else None,
            'protocol':     c.get('protocol'),
            'restarts':     self._restarts,
            'state_age_s':  round(st_age, 1) if st_age is not None else None,
            'autopilots':   self._link.get('autopilots', []),
            'others':       self._link.get('others', []),
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(m)

    def destroy_node(self):
        self._stop.set()
        self._kill('노드 종료')
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FcuManagerNode()
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
