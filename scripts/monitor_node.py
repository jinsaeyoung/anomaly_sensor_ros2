#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
실시간 모니터 (경량 상주 노드)

설계 원칙:
  1. 고주기 토픽을 직접 구독하지 않습니다.
     50Hz IMU 를 구독하면 초당 50번 콜백이 돌아 rosbag 과 CPU 를 경쟁합니다.
     Hz 계산은 sensor_health_node 가 한 번만 수행하고,
     모니터는 그 결과(5초 주기 요약)를 받아 씁니다.

  2. 구독 대상은 저주기 3종뿐입니다.
     /mavros/state          1Hz
     /auto_record/status    0.2Hz
     /sensor_health         0.2Hz
     /fcu_manager/status    0.5Hz  (포트·baud·SYSID·복구 상태)
     → 초당 콜백 3회 미만, 부하 무시 가능

  3. 화면 갱신 주기는 표시 속도만 결정합니다.
     캐시를 읽어 문자열을 만드는 비용이 대부분이라
     1초로 둬도 CPU 0.3% 수준입니다.

주기 선택 기준:
  1초  arm 테스트·비행 직전 점검 (전환 순간 포착)
  3초  비행 중 상시 관찰 (기본값, 읽기 편하고 부하 낮음)
  5초  장시간 방치·원격 SSH (대역폭 절감)

사용법:
  monitor_drone                # 3초 갱신 + 로그 (기본)
  monitor_drone --rate 1       # 1초 갱신
  monitor_drone --interval 5   # 5초 갱신
  monitor_drone --no-log       # 로그 제외
  monitor_drone --once         # 1회 출력
  monitor_drone --plain        # 색상 없이 (파일 저장용)
"""

import os
import re
import sys
import json
import time
import threading

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

W = 56   # 박스 너비


class C:
    """ANSI 색상 — --plain 이면 전부 빈 문자열"""
    def __init__(self, enabled=True):
        self.dim    = '\033[2m'  if enabled else ''
        self.red    = '\033[31m' if enabled else ''
        self.green  = '\033[32m' if enabled else ''
        self.yellow = '\033[33m' if enabled else ''
        self.bold   = '\033[1m'  if enabled else ''
        self.reset  = '\033[0m'  if enabled else ''


class MonitorNode(Node):

    def __init__(self, interval=3.0, show_log=True, log_lines=8,
                 once=False, color=True):
        super().__init__('monitor_node')

        self.interval  = interval
        self.show_log  = show_log
        self.log_lines = log_lines
        self.once      = once
        self.c         = C(color)

        self.save_dir = os.environ.get(
            'ANOMALY_DATA', os.path.expanduser('~/anomaly_data'))
        self.log_path = os.environ.get(
            'ANOMALY_LOG', os.path.join(self.save_dir, 'onboard.log'))

        # ── 캐시 ──────────────────────────────────────────────────────
        self._lock     = threading.Lock()
        self._state    = None
        self._state_t  = None
        self._record   = None
        self._health   = None
        self._health_t = None
        self._fcm      = None      # /fcu_manager/status

        # ── 구독: 저주기 3종만 ────────────────────────────────────────
        self.create_subscription(State,  '/mavros/state',
                                 self._cb_state,  MAVROS_QOS)
        self.create_subscription(String, '/auto_record/status',
                                 self._cb_record, 10)
        self.create_subscription(String, '/sensor_health',
                                 self._cb_health, 10)
        self.create_subscription(String, '/fcu_manager/status',
                                 self._cb_fcm, 10)

        # 로그는 화면보다 느리게 갱신 (파일 I/O 절감)
        self._log_cache    = []
        self._log_time     = 0.0
        self._log_interval = max(3.0, interval)

        self._start = time.monotonic()
        self.create_timer(interval, self._render)

    # ══════════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════════
    def _cb_state(self, msg):
        with self._lock:
            self._state = msg
            self._state_t = time.monotonic()

    def _cb_record(self, msg):
        with self._lock:
            try:
                self._record = json.loads(msg.data)
            except Exception:
                pass

    def _cb_fcm(self, msg):
        with self._lock:
            try:
                self._fcm = json.loads(msg.data)
            except Exception:
                pass

    def _cb_health(self, msg):
        with self._lock:
            try:
                self._health = json.loads(msg.data)
                self._health_t = time.monotonic()
            except Exception:
                pass

    # ══════════════════════════════════════════════════════════════════
    # 화면
    # ══════════════════════════════════════════════════════════════════
    def _box_top(self, title):
        return f"┌─ {title} " + "─" * max(0, W - len(title) - 4)

    def _box_bottom(self):
        return "└" + "─" * (W - 1)

    def _render(self):
        c = self.c
        out = []

        now = time.strftime('%Y-%m-%d %H:%M:%S')
        up  = time.monotonic() - self._start
        out.append(f"  {c.dim}{now}   갱신 {self.interval:.0f}초   "
                   f"실행 {up/60:.0f}분{c.reset}")
        out.append('')

        out += self._section_fc()
        out.append('')
        out += self._section_record()
        out.append('')
        out += self._section_sensors()

        if self.show_log:
            out.append('')
            out += self._section_log()

        text = '\n'.join(out)

        if self.once:
            print(text)
            rclpy.shutdown()
        else:
            # 화면 지우고 상단부터 다시 그림 (스크롤 없이 갱신)
            sys.stdout.write('\033[H\033[J' + text + '\n')
            sys.stdout.flush()

    def _section_fc(self):
        c = self.c
        o = [self._box_top('FC 상태')]
        with self._lock:
            st, st_t = self._state, self._state_t

        if st is None:
            o.append(f"│  {c.red}mavros 응답 없음{c.reset}")
            o.append(f"│  {c.dim}launch 실행 여부와 FC 연결을 확인하세요 (detect_fc){c.reset}")
        else:
            age = time.monotonic() - st_t
            if age > 5.0:
                o.append(f"│  {c.yellow}수신 중단 ({age:.0f}초 전 마지막){c.reset}")
            else:
                conn = f"{c.green}연결됨{c.reset}" if st.connected else f"{c.red}끊김{c.reset}"
                arm  = (f"{c.yellow}{c.bold}>>> ARMED <<<{c.reset}" if st.armed
                        else f"{c.dim}disarmed{c.reset}")
                o.append(f"│  연결: {conn}   상태: {arm}   모드: {st.mode}")

        # 관리 노드 정보 — 어느 포트·baud·SYSID 로 붙었는지
        with self._lock:
            fm = self._fcm
        if fm:
            phase_txt = {
                'CONNECTED':    f"{c.green}연결 유지{c.reset}",
                'STARTING':     f"{c.yellow}기동 중{c.reset}",
                'SEARCHING':    f"{c.yellow}FC 탐색 중{c.reset}",
                'DISCONNECTED': f"{c.yellow}끊김 감지{c.reset}",
                'RECOVERING':   f"{c.red}복구 중{c.reset}",
                'NO_DEVICE':    f"{c.red}FC 없음{c.reset}",
            }.get(fm.get('phase'), fm.get('phase', '?'))
            port = os.path.basename(fm.get('port') or '-')
            o.append(f"│  {c.dim}포트 {port} @ {fm.get('baud') or '-'}  "
                     f"대상 {fm.get('tgt') or '-'}  자기ID {fm.get('self_id') or '-'}{c.reset}")
            o.append(f"│  관리: {phase_txt}   재시작 {fm.get('restarts', 0)}회"
                     + (f"   {c.dim}{fm.get('note')}{c.reset}" if fm.get('note') else ''))
            if len(fm.get('autopilots') or []) > 1:
                o.append(f"│  {c.yellow}⚠ 비행제어기 여러 대 — tgt_system 지정 권장{c.reset}")
        o.append(self._box_bottom())
        return o

    def _section_record(self):
        c = self.c
        o = [self._box_top('녹화 상태')]
        with self._lock:
            rec = self._record

        if rec is None:
            o.append(f"│  {c.dim}auto_record_node 미실행{c.reset}")
            o.append(f"│  {c.dim}(use_auto_record:=true 로 실행했는지 확인){c.reset}")
        elif rec.get('recording'):
            o.append(f"│  {c.red}●{c.reset}  녹화 중        "
                     f"경과: {rec.get('elapsed_s', 0):.0f}초")
            o.append(f"│     파일: {rec.get('bag', '')}")
            o.append(f"│     디스크 여유: {rec.get('free_gb', 0):.1f} GB")
        else:
            o.append(f"│  ○  대기 중 (arm 하면 자동 시작)")
            o.append(f"│     디스크 여유: {rec.get('free_gb', 0):.1f} GB")
        o.append(self._box_bottom())
        return o

    def _section_sensors(self):
        c = self.c
        o = [self._box_top('센서 연결')]
        with self._lock:
            hl, hl_t = self._health, self._health_t

        marks = {
            'OK':       (f"{c.green}●{c.reset}", '정상'),
            'STALE':    (f"{c.yellow}▲{c.reset}", '수신중단'),
            'NO_DATA':  (f"{c.yellow}○{c.reset}", '데이터없음'),
            'NO_TOPIC': (f"{c.red}✕{c.reset}",   '노드없음'),
        }

        if hl is None:
            o.append(f"│  {c.dim}sensor_health_node 미실행{c.reset}")
            o.append(f"│  {c.dim}(launch 에 포함되었는지 확인){c.reset}")
        else:
            age = time.monotonic() - hl_t if hl_t else 999
            if age > 20:
                o.append(f"│  {c.yellow}상태 정보가 {age:.0f}초 전 것 — 노드 확인 필요{c.reset}")

            groups = hl.get('groups', {})
            hz     = hl.get('hz', {})
            for name in ('FC', 'ReSpeaker', 'THL100', 'WCM6800'):
                st = groups.get(name, '?')
                sym, desc = marks.get(st, ('?', st))
                h = hz.get(name)
                hz_s = f"{h:6.1f}Hz" if isinstance(h, (int, float)) else "     --"
                o.append(f"│  {sym} {name:10} {desc:12} {c.dim}{hz_s}{c.reset}")

            if not hl.get('all_ok', True):
                o.append("│")
                o.append(f"│  {c.yellow}⚠ 일부 센서 이상 — 비행 전 확인 필요{c.reset}")
        o.append(self._box_bottom())
        return o

    def _section_log(self):
        o = [self._box_top('최근 로그')]
        for line in self._tail_log():
            o.append(f"│ {line}")
        o.append(self._box_bottom())
        return o

    def _tail_log(self):
        """
        반복되는 재시도 경고를 걸러내고 의미 있는 이벤트만 표시

        화면 갱신마다 파일을 읽을 필요는 없으므로
        _log_interval 간격으로만 다시 읽고 나머지는 캐시를 재사용합니다.
        """
        now = time.monotonic()
        if now - self._log_time < self._log_interval and self._log_cache:
            return self._log_cache
        self._log_time = now

        if not os.path.isfile(self.log_path):
            self._log_cache = [f"로그 없음: {os.path.basename(self.log_path)}"]
            return self._log_cache

        skip = ('시리얼 연결 실패', 'USB 읽기 실패', '오디오 읽기 실패',
                'reconnect failed', 'No GPS fix')
        keep = ('녹화', '프리플라이트', '연결', '진단', 'ERROR',
                'HEARTBEAT', '찾을 수 없', '미연결', '이상')

        try:
            with open(self.log_path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 65536))
                raw = f.read().decode('utf-8', errors='ignore')
        except Exception as e:
            self._log_cache = [f"읽기 실패: {e}"]
            return self._log_cache

        picked = []
        for line in raw.splitlines():
            if any(s in line for s in skip):
                continue
            if not any(k in line for k in keep):
                continue
            line = re.sub(r'\[\d+\.\d+\]', '', line)
            line = re.sub(r'\[INFO\]|\[WARN\]', '', line)
            line = re.sub(r'\s+', ' ', line).strip()
            picked.append(line[:W - 2])

        self._log_cache = picked[-self.log_lines:] or ["(표시할 이벤트 없음)"]
        return self._log_cache


def main():
    args = sys.argv[1:]
    interval  = 3.0
    show_log  = True
    log_lines = 8
    once      = False
    color     = True

    i = 0
    while i < len(args):
        a = args[i]
        if a in ('--rate', '--interval') and i + 1 < len(args):
            v = float(args[i + 1])
            # --rate 는 Hz, --interval 은 초
            interval = (1.0 / v) if a == '--rate' else v
            i += 1
        elif a == '--no-log':
            show_log = False
        elif a == '--log' and i + 1 < len(args) and args[i + 1].isdigit():
            log_lines = int(args[i + 1]); i += 1
        elif a == '--once':
            once = True
        elif a == '--plain':
            color = False
        elif a in ('-h', '--help'):
            print(__doc__); return
        elif a.replace('.', '').isdigit():
            interval = float(a)
        i += 1

    # 너무 짧으면 터미널 출력만 늘어나므로 하한을 둡니다
    interval = max(0.5, interval)

    rclpy.init()
    node = None
    try:
        node = MonitorNode(interval=interval, show_log=show_log,
                           log_lines=log_lines, once=once, color=color)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n모니터 종료')
    except Exception as e:
        print(f'[ERROR] {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
