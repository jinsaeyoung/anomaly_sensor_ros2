#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
실시간 모니터 (경량 상주 노드)

화면 구성 (위에서부터):
  1. FC 상태      연결·arm·모드 + 관리 노드 정보(포트·baud·SYSID·복구 횟수)
  2. 녹화 상태    녹화 중 여부·경과·파일·디스크 여유
  3. 센서 연결    FC / ReSpeaker / THL100 / WCM6800 상태와 실측 Hz
  4. 최근 로그    의미 있는 이벤트 6줄 (최근 10분 이내만)
  5. 최근 녹화    최근 3건 — 시각·길이·크기·프리플라이트 결과

설계:
  - 고주기 토픽을 직접 구독하지 않습니다. (50Hz IMU 를 구독하면 rosbag 과 경쟁)
    Hz 는 sensor_health_node 가 계산한 값을 받아 씁니다.
  - 구독은 저주기 4종뿐 (/mavros/state, /auto_record/status,
    /sensor_health, /fcu_manager/status) → 부하 무시 가능
  - 로그·녹화 목록은 파일을 읽으므로 화면보다 느리게 갱신합니다.

로그 출처:
  서비스로 실행 중이면 ~/anomaly_data/onboard.log 를 읽고,
  수동 실행(ros2 launch)이라 onboard.log 가 오래됐으면
  각 노드가 남기는 ~/.ros/log/*.log 에서 읽습니다.
  어느 쪽이든 최근 10분 이내 줄만 보여 옛 로그가 섞이지 않습니다.

사용법:
  monitor_drone                # 3초 갱신 (기본)
  monitor_drone --interval 1   # 1초 갱신 (arm 테스트)
  monitor_drone --interval 5   # 5초 갱신 (장시간 방치)
  monitor_drone --log 10       # 로그 10줄
  monitor_drone --no-log       # 로그 섹션 제외
  monitor_drone --once         # 1회 출력 (데이터 수신 후)
  monitor_drone --plain        # 색상 없이
"""

import os
import re
import sys
import json
import glob
import time
import shutil
import threading
import unicodedata

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

W = 60                      # 박스 너비
LOG_WINDOW_SEC = 600        # 최근 로그로 인정하는 범위 (10분)
LOG_REFRESH_SEC = 3.0       # 로그 재읽기 간격
REC_REFRESH_SEC = 10.0      # 녹화 목록 재읽기 간격


def _dw(s):
    """화면 표시 폭 (한글 등 전각 문자는 2칸)"""
    return sum(2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1 for ch in s)


def _trunc(s, width):
    """표시 폭 기준으로 자르기"""
    out, w = '', 0
    for ch in s:
        cw = 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1
        if w + cw > width:
            return out + '…'
        out, w = out + ch, w + cw
    return out


_ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


def _fit(line, cols):
    """색상 코드는 폭에서 빼고, 터미널 폭을 넘으면 자릅니다 (줄바꿈 방지)"""
    out, w, i = '', 0, 0
    while i < len(line):
        m = _ANSI.match(line, i)
        if m:
            out += m.group(0); i = m.end(); continue
        ch = line[i]
        cw = 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1
        if w + cw > cols - 1:
            return out + '…\x1b[0m'
        out += ch; w += cw; i += 1
    return out


class C:
    """ANSI 색상 — --plain 이면 전부 빈 문자열"""
    def __init__(self, enabled=True):
        on = enabled and sys.stdout.isatty()
        self.dim    = '\033[2m'  if on else ''
        self.red    = '\033[31m' if on else ''
        self.green  = '\033[32m' if on else ''
        self.yellow = '\033[33m' if on else ''
        self.bold   = '\033[1m'  if on else ''
        self.reset  = '\033[0m'  if on else ''


class MonitorNode(Node):

    def __init__(self, interval=3.0, show_log=True, log_lines=6,
                 once=False, color=True):
        super().__init__('monitor_node')

        self.interval  = interval
        self.show_log  = show_log
        self.log_lines = log_lines
        self.once      = once
        self.c         = C(color)

        self.save_dir = os.environ.get(
            'ANOMALY_DATA', os.path.expanduser('~/anomaly_data'))
        self.onboard_log = os.environ.get(
            'ANOMALY_LOG', os.path.join(self.save_dir, 'onboard.log'))
        self.ros_log_dir = os.environ.get(
            'ROS_LOG_DIR', os.path.expanduser('~/.ros/log'))

        # ── 캐시 ──────────────────────────────────────────────────────
        self._lock     = threading.Lock()
        self._state    = None
        self._state_t  = None
        self._record   = None
        self._health   = None
        self._health_t = None
        self._fcm      = None

        self._log_cache, self._log_time, self._log_src = [], 0.0, ''
        self._rec_cache, self._rec_time = [], 0.0

        # ── 구독: 저주기 4종 ──────────────────────────────────────────
        self.create_subscription(State,  '/mavros/state',       self._cb_state,  MAVROS_QOS)
        self.create_subscription(String, '/auto_record/status', self._cb_record, 10)
        self.create_subscription(String, '/sensor_health',      self._cb_health, 10)
        self.create_subscription(String, '/fcu_manager/status', self._cb_fcm,    10)

        self._start = time.monotonic()
        # --once 는 데이터가 모일 때까지 짧은 주기로 확인 후 한 번만 출력
        self.create_timer(0.5 if once else interval, self._tick)

    # ══════════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════════
    def _cb_state(self, msg):
        with self._lock:
            self._state, self._state_t = msg, time.monotonic()

    def _json_cb(attr):
        def cb(self, msg):
            try:
                data = json.loads(msg.data)
            except Exception:
                return
            with self._lock:
                setattr(self, attr, data)
                if attr == '_health':
                    self._health_t = time.monotonic()
        return cb

    _cb_record = _json_cb('_record')
    _cb_health = _json_cb('_health')
    _cb_fcm    = _json_cb('_fcm')

    # ══════════════════════════════════════════════════════════════════
    # 출력
    # ══════════════════════════════════════════════════════════════════
    def _tick(self):
        if self.once:
            # 상태 토픽이 대부분 도착했거나 7초가 지나면 출력
            with self._lock:
                ready = sum(x is not None for x in
                            (self._state, self._record, self._health, self._fcm))
            if ready < 3 and time.monotonic() - self._start < 7.0:
                return
            print(self._render())
            raise SystemExit
        # 터미널 크기에 맞춰 자릅니다. 넘치면 윗줄이 스크롤백에 쌓여
        # 같은 내용이 반복되는 것처럼 보이기 때문입니다.
        cols, rows = shutil.get_terminal_size((100, 50))
        lines = [_fit(l, cols) for l in self._render().split('\n')]
        if len(lines) > rows - 1:
            lines = lines[:rows - 2] + [f"{self.c.dim}… 창을 키우면 전체가 보입니다 "
                                        f"({len(lines)}줄 필요 / 현재 {rows}줄){self.c.reset}"]
        sys.stdout.write('\033[H\033[J' + '\n'.join(lines))
        sys.stdout.flush()

    def _render(self):
        c = self.c
        out = []
        now = time.strftime('%Y-%m-%d %H:%M:%S')
        up = (time.monotonic() - self._start) / 60
        out.append(f"  {c.dim}{now}   갱신 {self.interval:g}초   실행 {up:.0f}분{c.reset}")
        out.append('')
        out += self._section_fc();      out.append('')
        out += self._section_record();  out.append('')
        out += self._section_sensors()
        if self.show_log:
            out.append('')
            out += self._section_log()
        out.append('')
        out += self._section_recent()
        return '\n'.join(out)

    def _top(self, title):
        return f"┌─ {title} " + "─" * max(0, W - _dw(title) - 4)

    def _bottom(self):
        return "└" + "─" * (W - 1)

    # ── 1. FC 상태 ────────────────────────────────────────────────────
    def _section_fc(self):
        c = self.c
        o = [self._top('FC 상태')]
        with self._lock:
            st, st_t, fm = self._state, self._state_t, self._fcm

        if st is None:
            o.append(f"│  {c.red}mavros 응답 없음{c.reset}")
            if fm is None:
                o.append(f"│  {c.dim}launch 가 실행 중인지 확인하세요 (detect_serial 로 FC 확인){c.reset}")
        else:
            age = time.monotonic() - st_t
            if age > 5.0:
                o.append(f"│  {c.yellow}수신 중단 ({age:.0f}초 전 마지막){c.reset}")
            else:
                conn = f"{c.green}연결됨{c.reset}" if st.connected else f"{c.red}끊김{c.reset}"
                arm = (f"{c.yellow}{c.bold}>>> ARMED <<<{c.reset}" if st.armed
                       else f"{c.dim}disarmed{c.reset}")
                o.append(f"│  연결: {conn}   상태: {arm}   모드: {st.mode}")

        if fm:
            phase = {
                'CONNECTED':    f"{c.green}연결 유지{c.reset}",
                'STARTING':     f"{c.yellow}기동 중{c.reset}",
                'SEARCHING':    f"{c.yellow}FC 탐색 중{c.reset}",
                'DISCONNECTED': f"{c.yellow}끊김 감지{c.reset}",
                'RECOVERING':   f"{c.red}복구 중{c.reset}",
                'NO_DEVICE':    f"{c.red}FC 없음{c.reset}",
            }.get(fm.get('phase'), fm.get('phase', '?'))
            # by-id 이름은 너무 길어 줄이 넘치므로 실제 장치명(ttyUSB0 등)으로 표시
            port = os.path.basename(os.path.realpath(fm['port'])) if fm.get('port') else '-'
            o.append(f"│  {c.dim}포트 {port} @ {fm.get('baud') or '-'}   "
                     f"대상 {fm.get('tgt') or '-'}   자기ID {fm.get('self_id') or '-'}{c.reset}")
            note = f"   {c.dim}{fm['note']}{c.reset}" if fm.get('note') else ''
            o.append(f"│  관리: {phase}   재시작 {fm.get('restarts', 0)}회{note}")
            if fm.get('restarts') and fm.get('last_restart'):
                o.append(f"│  {c.dim}마지막 재시작 사유: {fm['last_restart']}{c.reset}")
            if len(fm.get('autopilots') or []) > 1:
                o.append(f"│  {c.yellow}⚠ 비행제어기 여러 대 — tgt_system 지정 권장{c.reset}")
        o.append(self._bottom())
        return o

    # ── 2. 녹화 상태 ──────────────────────────────────────────────────
    def _section_record(self):
        c = self.c
        o = [self._top('녹화 상태')]
        with self._lock:
            rec = self._record
        if rec is None:
            o.append(f"│  {c.dim}auto_record_node 미실행 (use_auto_record:=true 확인){c.reset}")
        elif rec.get('recording'):
            o.append(f"│  {c.red}●{c.reset}  녹화 중        경과: {rec.get('elapsed_s', 0):.0f}초")
            o.append(f"│     파일: {rec.get('bag', '')}")
            o.append(f"│     디스크 여유: {rec.get('free_gb', 0):.1f} GB")
        else:
            o.append("│  ○  대기 중 (arm 하면 자동 시작)")
            o.append(f"│     디스크 여유: {rec.get('free_gb', 0):.1f} GB")

        # 전원을 내려도 되는 시점 — disarm 후에도 몇 초간은 녹화가 이어지므로
        # 그 사이에 전원을 끊으면 마지막 bag 이 마감되지 못합니다.
        if rec.get('recording'):
            o.append(f"│  {c.red}⚠ 녹화 중 — 전원을 내리지 마세요{c.reset}")
        elif rec.get('safe_power_off'):
            o.append(f"│  {c.green}저장 완료 — 전원 차단 가능{c.reset}")
        o.append(self._bottom())
        return o

    # ── 3. 센서 연결 ──────────────────────────────────────────────────
    def _section_sensors(self):
        c = self.c
        o = [self._top('센서 연결')]
        with self._lock:
            hl, hl_t = self._health, self._health_t

        marks = {
            'OK':       (f"{c.green}●{c.reset}",  '정상'),
            'STALE':    (f"{c.yellow}▲{c.reset}", '수신중단'),
            'NO_DATA':  (f"{c.yellow}○{c.reset}", '데이터없음'),
            'NO_TOPIC': (f"{c.red}✕{c.reset}",    '노드없음'),
        }
        if hl is None:
            o.append(f"│  {c.dim}sensor_health_node 미실행 (launch 에 포함되었는지 확인){c.reset}")
        else:
            age = time.monotonic() - hl_t if hl_t else 999
            if age > 20:
                o.append(f"│  {c.yellow}상태 정보가 {age:.0f}초 전 것 — 노드 확인 필요{c.reset}")
            groups, hz = hl.get('groups', {}), hl.get('hz', {})
            for name in ('FC', 'ReSpeaker', 'THL100', 'WCM6800'):
                sym, desc = marks.get(groups.get(name, '?'), ('?', groups.get(name, '?')))
                h = hz.get(name)
                hz_s = f"{h:6.1f}Hz" if isinstance(h, (int, float)) else "     --"
                o.append(f"│  {sym} {name:10} {desc:10} {c.dim}{hz_s}{c.reset}")
            if not hl.get('all_ok', True):
                o.append("│")
                o.append(f"│  {c.yellow}⚠ 일부 센서 이상 — 비행 전 확인 필요{c.reset}")
        o.append(self._bottom())
        return o

    # ── 4. 최근 로그 ──────────────────────────────────────────────────
    def _section_log(self):
        lines = self._tail_log()
        title = '최근 로그' + (f' ({self._log_src})' if self._log_src else '')
        o = [self._top(title)]
        for line in lines:
            o.append(f"│ {line}")
        o.append(self._bottom())
        return o

    SKIP = ('시리얼 연결 실패', 'USB 읽기 실패', '오디오 읽기 실패',
            'reconnect failed', 'No GPS fix', 'RTT too high', 'Time jump')
    KEEP = ('녹화', '프리플라이트', '연결', '진단', 'ERROR', 'WARN',
            'HEARTBEAT', '찾을 수 없', '미연결', '이상', '탐지', '복구', '재탐색', '기동')
    STAMP = re.compile(r'\[(\d{10})\.\d+\]')

    def _log_sources(self):
        """
        읽을 로그 파일 목록

        onboard.log 가 최근(2분 이내)에 갱신됐으면 서비스 실행 중이므로 그것만 씁니다.
        아니면 수동 실행으로 보고 최근 갱신된 노드 로그(~/.ros/log/*.log)를 씁니다.
        """
        now = time.time()
        try:
            if now - os.path.getmtime(self.onboard_log) < 120:
                return [self.onboard_log], 'onboard.log'
        except OSError:
            pass
        files = [f for f in glob.glob(os.path.join(self.ros_log_dir, '*.log'))
                 if now - os.path.getmtime(f) < LOG_WINDOW_SEC]
        files.sort(key=os.path.getmtime, reverse=True)
        if files:
            return files[:12], '노드 로그'
        try:
            os.path.getmtime(self.onboard_log)
            return [self.onboard_log], 'onboard.log'
        except OSError:
            return [], ''

    def _tail_log(self):
        now = time.monotonic()
        if now - self._log_time < LOG_REFRESH_SEC and self._log_cache:
            return self._log_cache
        self._log_time = now

        files, self._log_src = self._log_sources()
        if not files:
            self._log_cache = ['(로그 없음 — launch 실행 후 표시됩니다)']
            return self._log_cache

        cutoff = time.time() - LOG_WINDOW_SEC
        picked = []
        for path in files:
            try:
                with open(path, 'rb') as f:
                    f.seek(0, os.SEEK_END)
                    f.seek(max(0, f.tell() - 65536))
                    raw = f.read().decode('utf-8', errors='ignore')
            except OSError:
                continue
            for line in raw.splitlines():
                if any(s in line for s in self.SKIP):
                    continue
                if not any(k in line for k in self.KEEP):
                    continue
                m = self.STAMP.search(line)
                if not m:
                    continue
                ts = int(m.group(1))
                if ts < cutoff:
                    continue            # 오래된 로그는 표시하지 않음
                text = self.STAMP.sub('', line)
                text = re.sub(r'\[(INFO|WARN|ERROR)\]', lambda x: '!' if x.group(1) != 'INFO' else '', text)
                text = re.sub(r'\[[a-z0-9_]+-\d+\]', '', text)      # [node-3] 접두어 제거
                text = re.sub(r'\s+', ' ', text).strip()
                picked.append((ts, f"{time.strftime('%H:%M:%S', time.localtime(ts))} {text}"))

        picked.sort(key=lambda x: x[0])
        # 같은 내용 연속 중복 제거
        uniq, last = [], None
        for _, s in picked:
            body = s[9:]
            if body != last:
                uniq.append(_trunc(s, W - 4))
            last = body
        self._log_cache = uniq[-self.log_lines:] or ['(최근 10분간 표시할 이벤트 없음)']
        return self._log_cache

    # ── 5. 최근 녹화 ──────────────────────────────────────────────────
    def _section_recent(self):
        c = self.c
        o = [self._top('최근 녹화 3건')]
        rows = self._recent_bags()
        if not rows:
            o.append(f"│  {c.dim}(녹화 없음){c.reset}")
        for r in rows:
            pf = ''
            v = r.get('verify')
            if v:
                # 착륙 후 자동 검증 결과가 있으면 그것을 우선 표시
                lacking = [g for g, s in (v.get('sensors') or {}).items() if s != '정상']
                col = c.green if v.get('drone') == '정상' and not lacking else c.yellow
                if v.get('drone') == '불량':
                    col = c.red
                pf = f"{col}드론 {v.get('drone')}" + (f" · {','.join(lacking)} 확인" if lacking else '') + c.reset
            elif r['preflight'] is True:
                pf = f"{c.green}프리플라이트 OK{c.reset}"
            elif r['preflight'] is False:
                pf = f"{c.yellow}프리플라이트 경고{c.reset}"
            o.append(f"│  {r['time']}  {r['name']:26} {r['size']:>8}  {pf}")
        o.append(self._bottom())
        return o

    def _recent_bags(self):
        now = time.monotonic()
        if now - self._rec_time < REC_REFRESH_SEC and self._rec_time:
            return self._rec_cache
        self._rec_time = now

        dirs = [d for d in glob.glob(os.path.join(self.save_dir, '*'))
                if os.path.isdir(d)
                and os.path.basename(d).startswith(('flight_', 'anomaly_data_'))]
        dirs.sort(key=os.path.getmtime, reverse=True)

        rows = []
        for d in dirs[:3]:
            name = os.path.basename(d)
            try:
                size = sum(os.path.getsize(f) for f in glob.glob(os.path.join(d, '*')))
                size_s = f"{size / 1024 / 1024:.1f}MB" if size < 1024 ** 3 else f"{size / 1024 ** 3:.2f}GB"
            except OSError:
                size_s = '-'
            preflight, verify = None, None
            meta = os.path.join(self.save_dir, f'{name}_meta.json')
            if os.path.exists(meta):
                try:
                    with open(meta, encoding='utf-8') as fh:
                        md = json.load(fh)
                    preflight = bool(md.get('preflight_ok'))
                    verify = md.get('verify')          # 착륙 후 자동 검증 결과
                except Exception:
                    pass
            rows.append({
                'name': name, 'size': size_s, 'preflight': preflight, 'verify': verify,
                'time': time.strftime('%m-%d %H:%M', time.localtime(os.path.getmtime(d))),
            })
        self._rec_cache = rows
        return rows


def main():
    args = sys.argv[1:]
    interval, show_log, log_lines, once, color = 3.0, True, 6, False, True

    i = 0
    while i < len(args):
        a = args[i]
        if a in ('--rate', '--interval') and i + 1 < len(args):
            v = float(args[i + 1])
            interval = (1.0 / v) if a == '--rate' else v
            i += 1
        elif a == '--no-log':
            show_log = False
        elif a == '--log':
            if i + 1 < len(args) and args[i + 1].isdigit():
                log_lines = int(args[i + 1]); i += 1
        elif a == '--once':
            once = True
        elif a == '--plain':
            color = False
        elif a in ('-h', '--help'):
            print(__doc__); return
        elif a.replace('.', '', 1).isdigit():
            interval = float(a)
        i += 1
    interval = max(0.5, interval)

    # top/htop 처럼 별도 화면을 씁니다. 종료하면 원래 터미널 내용이 그대로 돌아오고
    # 갱신 화면이 스크롤백에 쌓이지 않습니다.
    alt = not once and sys.stdout.isatty()
    if alt:
        sys.stdout.write('\033[?1049h\033[?25l'); sys.stdout.flush()

    rclpy.init()
    node = None
    try:
        node = MonitorNode(interval=interval, show_log=show_log,
                           log_lines=log_lines, once=once, color=color)
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:
        print(f'[ERROR] {e}')
    finally:
        if alt:
            sys.stdout.write('\033[?25h\033[?1049l'); sys.stdout.flush()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
