#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
시리얼 장치 자동 탐색

어떤 USB-UART 젠더를 어느 USB 포트에 꽂아도 장치를 구분하기 위한 모듈입니다.
VID:PID 나 by-id 경로에 의존하지 않고, 각 포트를 실제로 열어
**수신 데이터의 형식**으로 장치를 판별합니다.

  THL100   9600bps   '@ID,seq,temp,humi,light'
  WCM6800  9600bps   '[~+-]NNNNN' (6바이트 고정)
  FC       자동       MAVLink 프레임 (CRC 검증) + HEARTBEAT 해석

FC 는 baud 와 SYSID 까지 함께 알아냅니다.
  - baud  : ArduPilot SERIALn_BAUD 후보를 차례로 시도해 CRC 가 맞는 속도를 채택
  - SYSID : HEARTBEAT 를 **수신만** 해서 읽습니다. 아무것도 송신하지 않으므로
            FC·GCS·페이로드 동작에 영향을 주지 않습니다.
  - 같은 링크에 GCS, 짐벌, 카메라, 다른 컴패니언이 섞여 있어도
    HEARTBEAT 의 type/autopilot 필드로 비행제어기만 골라냅니다.

다른 프로세스가 이미 열고 있는 포트는 건너뜁니다.
(실행 중인 노드의 포트를 열면 데이터를 빼앗아 양쪽이 모두 깨집니다)

사용법 (CLI):
    python3 serial_autodetect.py              # 전체 스캔
    python3 serial_autodetect.py --json       # JSON 출력
    python3 serial_autodetect.py --device fc  # 특정 장치 경로만
    python3 serial_autodetect.py --fc         # FC 상세 (baud/SYSID/링크 구성원)
"""

import glob
import os
import re
import sys
import time

try:
    import serial
except ImportError:
    serial = None


# ══════════════════════════════════════════════════════════════════════════════
# MAVLink 파서 (CRC 검증)
# ══════════════════════════════════════════════════════════════════════════════
# 메시지별 CRC_EXTRA — pymavlink ardupilotmega 방언에서 추출한 값입니다.
# 여기에 없는 메시지는 CRC 를 검증할 수 없어 판정에 쓰지 않습니다.
# (HEARTBEAT 만 있어도 탐지는 되며, 나머지는 baud 판정을 빠르게 하는 용도)
CRC_EXTRA = {
    0: 50, 1: 124, 2: 137, 22: 220, 24: 24, 27: 144, 29: 115, 30: 39,
    32: 185, 33: 104, 35: 244, 36: 222, 42: 28, 62: 183, 65: 118, 73: 38,
    74: 20, 77: 143, 83: 22, 87: 150, 109: 185, 111: 34, 116: 76, 125: 203,
    129: 46, 136: 1, 147: 154, 148: 178, 152: 208, 163: 127, 164: 154,
    165: 21, 168: 1, 173: 83, 178: 47, 193: 71, 241: 90, 242: 104, 245: 130,
    253: 83, 11030: 144,
}

# HEARTBEAT.autopilot == MAV_AUTOPILOT_INVALID(8) 이면 비행제어기가 아닙니다.
MAV_AUTOPILOT_INVALID = 8

# HEARTBEAT.type 중 '기체'가 아닌 것들 — 이들의 SYSID 는 대상으로 삼지 않습니다.
NON_VEHICLE_TYPES = {
    6:  'GCS',
    18: 'ONBOARD_CONTROLLER',
    26: 'GIMBAL',
    27: 'ADSB',
    30: 'CAMERA',
    31: 'CHARGING_STATION',
    32: 'FLARM',
    33: 'SERVO',
    34: 'ODID',
    36: 'BATTERY',
    37: 'PARACHUTE',
    38: 'LOG',
    39: 'OSD',
    40: 'IMU',
    41: 'GPS',
    42: 'WINCH',
}

# 컴패니언 컴퓨터용 컴포넌트 ID (MAV_COMP_ID_ONBOARD_COMPUTER, 2, 3, 4)
COMPANION_COMPIDS = (191, 194, 195, 196)

# ArduPilot SERIALn_BAUD 에서 흔히 쓰는 값 — 앞쪽부터 시도합니다.
FC_BAUDS = (921600, 115200, 57600, 460800, 230400, 500000)


def _crc_accumulate(data, crc=0xFFFF):
    """MAVLink X.25 CRC"""
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def parse_mavlink(data):
    """
    바이트 스트림에서 CRC 가 맞는 MAVLink 프레임만 추출

    CRC 를 확인하지 않으면 잘못된 baud 로 읽은 잡음에서도
    0xFD/0xFE 가 우연히 나와 엉뚱한 SYSID 가 잡힙니다.
    (예: 'detected remote address 191.239' 같은 로그)

    반환: [{'ver', 'sysid', 'compid', 'msgid', 'payload', 'signed'}, ...]
    """
    frames = []
    i = 0
    n = len(data)
    while i < n - 8:
        stx = data[i]

        if stx == 0xFD and i + 12 <= n:
            plen = data[i + 1]
            incompat = data[i + 2]
            sig = 13 if (incompat & 0x01) else 0
            end = i + 10 + plen + 2 + sig
            if end <= n:
                msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
                extra = CRC_EXTRA.get(msgid)
                if extra is not None:
                    crc = _crc_accumulate(data[i + 1:i + 10 + plen])
                    crc = _crc_accumulate([extra], crc)
                    got = data[i + 10 + plen] | (data[i + 11 + plen] << 8)
                    if crc == got:
                        frames.append({
                            'ver': 2, 'sysid': data[i + 5], 'compid': data[i + 6],
                            'msgid': msgid, 'payload': bytes(data[i + 10:i + 10 + plen]),
                            'signed': bool(sig),
                        })
                        i = end
                        continue

        elif stx == 0xFE:
            plen = data[i + 1]
            end = i + 6 + plen + 2
            if end <= n:
                msgid = data[i + 5]
                extra = CRC_EXTRA.get(msgid)
                if extra is not None:
                    crc = _crc_accumulate(data[i + 1:i + 6 + plen])
                    crc = _crc_accumulate([extra], crc)
                    got = data[i + 6 + plen] | (data[i + 7 + plen] << 8)
                    if crc == got:
                        frames.append({
                            'ver': 1, 'sysid': data[i + 3], 'compid': data[i + 4],
                            'msgid': msgid, 'payload': bytes(data[i + 6:i + 6 + plen]),
                            'signed': False,
                        })
                        i = end
                        continue
        i += 1
    return frames


def decode_heartbeat(payload):
    """
    HEARTBEAT 페이로드 해석
      custom_mode(u32) type(u8) autopilot(u8) base_mode(u8) system_status(u8) ver(u8)
    MAVLink2 는 뒤쪽 0 바이트를 잘라 보내므로 9바이트로 채워서 읽습니다.
    """
    p = payload + b'\x00' * (9 - len(payload))
    return {'type': p[4], 'autopilot': p[5], 'base_mode': p[6], 'status': p[7]}


def summarize_link(frames):
    """
    링크에 누가 있는지 정리하고 대상 비행제어기를 고릅니다.

    반환: {
      'target':     {'sysid','compid','count','type','autopilot'} | None,
      'autopilots': [...],  비행제어기 후보 (여러 개면 다중 기체 링크)
      'others':     [...],  GCS·짐벌·카메라·컴패니언 등
      'protocol':   'v2.0' | 'v1.0',
      'signed':     bool,
      'valid':      CRC 통과 프레임 수,
    }
    """
    hb = {}
    for f in frames:
        if f['msgid'] != 0:
            continue
        key = (f['sysid'], f['compid'])
        info = hb.setdefault(key, {'sysid': f['sysid'], 'compid': f['compid'],
                                   'count': 0, **decode_heartbeat(f['payload'])})
        info['count'] += 1

    autopilots, others = [], []
    for info in hb.values():
        is_vehicle = (info['autopilot'] != MAV_AUTOPILOT_INVALID
                      and info['type'] not in NON_VEHICLE_TYPES
                      and info['sysid'] not in (0, 255))
        if is_vehicle:
            autopilots.append(info)
        else:
            info['role'] = NON_VEHICLE_TYPES.get(info['type'], f'type{info["type"]}')
            others.append(info)

    # 같은 기체에 컴포넌트가 여러 개면 compid=1(AUTOPILOT1) 을 우선합니다.
    autopilots.sort(key=lambda x: (x['compid'] != 1, -x['count']))
    target = autopilots[0] if autopilots else None

    v2 = sum(1 for f in frames if f['ver'] == 2)
    v1 = len(frames) - v2
    return {
        'target':     target,
        'autopilots': autopilots,
        'others':     others,
        'protocol':   'v2.0' if v2 >= v1 else 'v1.0',
        'signed':     any(f['signed'] for f in frames),
        'valid':      len(frames),
    }


def choose_companion_compid(link, sysid):
    """
    mavros 가 사용할 컴포넌트 ID 선택

    같은 기체(SYSID)에 이미 다른 컴패니언(예: 191)이 있으면
    겹치지 않는 번호(194, 195, 196)를 고릅니다.
    """
    used = {o['compid'] for o in link.get('others', []) if o['sysid'] == sysid}
    for c in COMPANION_COMPIDS:
        if c not in used:
            return c
    return COMPANION_COMPIDS[0]


# ══════════════════════════════════════════════════════════════════════════════
# 9600bps 센서 시그니처
# ══════════════════════════════════════════════════════════════════════════════
def _match_thl100(data: bytes) -> int:
    """@sensorID,seq,temp,humi,light 형식"""
    text = data.decode('ascii', errors='ignore')
    return len(re.findall(r'@[A-Za-z0-9]+,\d+,[\d.\-]*,[\d.\-]*,[\d.\-]*', text))


def _match_wcm6800(data: bytes) -> int:
    """[~+-]NNNNN 형식 (6바이트 고정)"""
    text = data.decode('ascii', errors='ignore')
    hits = 0
    for line in text.replace('\r', '\n').split('\n'):
        line = line.strip()
        if len(line) == 6 and line[0] in '~+-' and line[1:].isdigit():
            hits += 1
    return hits


def _match_mavlink(data: bytes) -> int:
    """CRC 가 맞는 MAVLink 프레임 수 (하위 호환용)"""
    return len(parse_mavlink(data))


DEVICE_SIGNATURES = {
    'thl100': {
        'name': 'OSTSen-THL100 (온습도/조도)', 'baud': 9600,
        'probe_sec': 3.0, 'min_hits': 1, 'matcher': _match_thl100,
    },
    'wcm6800': {
        'name': 'Winson WCM6800 (전류계)', 'baud': 9600,
        'probe_sec': 2.5, 'min_hits': 2, 'matcher': _match_wcm6800,
    },
    'fc': {
        'name': 'ArduPilot FC (MAVLink)', 'baud': 921600,
        'probe_sec': 2.5, 'min_hits': 1, 'matcher': _match_mavlink,
    },
}
PROBE_ORDER = ['wcm6800', 'thl100', 'fc']

# 포트를 닫은 뒤 다시 열기까지의 안정화 시간
PORT_SETTLE_SEC = 0.3


# ══════════════════════════════════════════════════════════════════════════════
# 포트 목록 / 사용 중 포트
# ══════════════════════════════════════════════════════════════════════════════
def list_serial_ports():
    """
    후보 시리얼 포트 — by-id 경로 우선, 없으면 ttyUSB/ttyACM 로 보완

    같은 실제 장치를 가리키는 경로는 하나만 남깁니다.
    (PL2303 처럼 시리얼 번호가 없는 젠더 두 개는 by-id 가 하나로 겹치므로
     나머지 하나는 /dev/ttyUSBn 으로 포함됩니다)
    """
    ports, seen = [], set()
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        real = os.path.realpath(p)
        if real not in seen:
            ports.append(p)
            seen.add(real)
    for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
        for p in sorted(glob.glob(pattern)):
            real = os.path.realpath(p)
            if real not in seen:
                ports.append(p)
                seen.add(real)
    return ports


# ── 포트 선점 표시 ────────────────────────────────────────────────────
# mavros 가 포트를 열기 직전의 짧은 틈에 다른 노드가 먼저 열면 데이터를 나눠 갖게 됩니다.
# /proc 확인만으로는 '아직 안 열린' 포트를 알 수 없으므로,
# 관리 노드가 쓰기로 정한 포트를 파일로 표시해 다른 노드가 피하게 합니다.
# 표시한 프로세스가 죽으면 표시는 자동으로 무시됩니다.
CLAIM_DIR = '/tmp/anomaly_sensor_claims'


def claim_port(key, port):
    try:
        os.makedirs(CLAIM_DIR, exist_ok=True)
        with open(os.path.join(CLAIM_DIR, key), 'w') as f:
            f.write(f'{os.getpid()} {os.path.realpath(port)}\n')
    except OSError:
        pass


def release_port(key):
    try:
        os.remove(os.path.join(CLAIM_DIR, key))
    except OSError:
        pass


def claimed_ports(exclude_key=None):
    ports = set()
    for path in glob.glob(os.path.join(CLAIM_DIR, '*')):
        if os.path.basename(path) == exclude_key:
            continue
        try:
            pid, real = open(path).read().split()
            os.kill(int(pid), 0)          # 표시한 프로세스가 살아 있을 때만 유효
            ports.add(real)
        except (OSError, ValueError):
            continue
    return ports


def ports_in_use(exclude_key=None):
    """
    다른 프로세스가 열고 있는 tty 장치 (실제 경로 집합)

    실행 중인 노드(mavros, thl100 등)의 포트를 탐색하면 데이터를 빼앗아
    양쪽이 모두 깨지므로, 재탐색 시에는 이 목록을 제외합니다.
    같은 사용자로 실행된 프로세스만 확인할 수 있습니다.
    """
    used = set()
    me = str(os.getpid())
    for fd_dir in glob.glob('/proc/[0-9]*/fd'):
        if fd_dir.split('/')[2] == me:
            continue
        try:
            for fd in os.listdir(fd_dir):
                try:
                    target = os.readlink(os.path.join(fd_dir, fd))
                except OSError:
                    continue
                if target.startswith('/dev/tty'):
                    used.add(os.path.realpath(target))
        except OSError:
            continue
    return used | claimed_ports(exclude_key)


def _open(port, baud):
    """탐색용 포트 열기 — exclusive 로 열어 다른 노드와 동시 점유를 막습니다."""
    kwargs = dict(port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  timeout=0.2)
    try:
        return serial.Serial(exclusive=True, **kwargs)
    except TypeError:
        return serial.Serial(**kwargs)


def _read_for(ser, seconds, stop=None):
    """seconds 동안 수신. stop(buf) 가 True 면 조기 종료"""
    buf = b''
    time.sleep(0.15)                 # 이전 baud 잔여 데이터 정리
    ser.reset_input_buffer()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if stop and stop(buf):
                break
        else:
            time.sleep(0.02)
    return buf


# ══════════════════════════════════════════════════════════════════════════════
# FC 탐지 (baud + SYSID)
# ══════════════════════════════════════════════════════════════════════════════
def probe_fc(port, bauds=FC_BAUDS, verbose=False):
    """
    포트가 FC 인지 확인하고 baud·SYSID·링크 구성원을 알아냅니다.

    송신은 전혀 하지 않습니다. HEARTBEAT 는 FC 가 1초마다 스스로 보내므로
    듣기만 해도 SYSID 를 알 수 있습니다.

    반환: {'port','baud','sysid','compid','protocol','signed',
           'my_compid','autopilots','others'} | None
    """
    if serial is None:
        raise RuntimeError('pyserial 미설치 — pip3 install pyserial')

    for idx, baud in enumerate(bauds):
        try:
            ser = _open(port, baud)
        except Exception as e:
            if verbose:
                print(f'      {baud:>7}: 열기 실패 ({e})')
            return None

        try:
            # HEARTBEAT 는 1Hz 이므로 최대 2.5초 기다립니다.
            # 비행제어기 HEARTBEAT 를 받으면 즉시 종료합니다.
            # 조기 종료 판정은 버퍼 전체를 다시 파싱하므로 자주 하면 비쌉니다.
            # (921600bps 에서 2.5초면 200KB 이상 — 매번 파싱하면 탐지가 느려짐)
            # 0.3초 간격으로만 확인합니다.
            last = [time.monotonic()]

            def got_autopilot(b):
                now = time.monotonic()
                if now - last[0] < 0.3:
                    return False
                last[0] = now
                return summarize_link(parse_mavlink(b))['target'] is not None

            buf = _read_for(ser, 2.5, stop=got_autopilot)
        except Exception:
            buf = b''
        finally:
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(PORT_SETTLE_SEC)

        # 어떤 baud 로 읽든 전기 신호가 있으면 바이트는 들어옵니다.
        # 첫 시도에서 0바이트면 이 포트는 송신 자체가 없으므로 더 볼 필요가 없습니다.
        if idx == 0 and len(buf) == 0:
            if verbose:
                print(f'      {baud:>7}: 수신 없음 — FC 아님')
            return None

        link = summarize_link(parse_mavlink(buf))
        if verbose:
            t = link['target']
            desc = f"SYSID {t['sysid']}.{t['compid']}" if t else '비행제어기 HEARTBEAT 없음'
            print(f'      {baud:>7}: {len(buf):5d}B  CRC OK {link["valid"]:3d}  {desc}')

        if link['target']:
            t = link['target']
            return {
                'port':       port,
                'baud':       baud,
                'sysid':      t['sysid'],
                'compid':     t['compid'],
                'protocol':   link['protocol'],
                'signed':     link['signed'],
                'my_compid':  choose_companion_compid(link, t['sysid']),
                'autopilots': [(a['sysid'], a['compid'], a['count']) for a in link['autopilots']],
                'others':     [(o['sysid'], o['compid'], o['role']) for o in link['others']],
            }
    return None


def probe_port(port, device_key, verbose=False):
    """단일 장치 확인 — 반환: (일치 여부, 일치 횟수, 수신 바이트 수)"""
    if device_key == 'fc':
        info = probe_fc(port, verbose=verbose)
        return (info is not None), (1 if info else 0), 0

    sig = DEVICE_SIGNATURES[device_key]
    try:
        ser = _open(port, sig['baud'])
    except Exception:
        return False, 0, 0
    try:
        buf = _read_for(ser, sig['probe_sec'],
                        stop=lambda b: sig['matcher'](b) >= sig['min_hits'])
    except Exception:
        buf = b''
    finally:
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(PORT_SETTLE_SEC)
    hits = sig['matcher'](buf)
    return hits >= sig['min_hits'], hits, len(buf)


def _probe_9600(port):
    """9600 장치 두 종류를 한 번의 수신으로 함께 판별"""
    try:
        ser = _open(port, 9600)
    except Exception:
        return None, 0
    try:
        def done(b):
            return (_match_thl100(b) >= DEVICE_SIGNATURES['thl100']['min_hits'] or
                    _match_wcm6800(b) >= DEVICE_SIGNATURES['wcm6800']['min_hits'])
        buf = _read_for(ser, 3.0, stop=done)
    except Exception:
        buf = b''
    finally:
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(PORT_SETTLE_SEC)

    thl, wcm = _match_thl100(buf), _match_wcm6800(buf)
    if thl >= 1 and thl >= wcm:
        return 'thl100', len(buf)
    if wcm >= 2:
        return 'wcm6800', len(buf)
    return None, len(buf)


# ══════════════════════════════════════════════════════════════════════════════
# 전체 탐색
# ══════════════════════════════════════════════════════════════════════════════
def detect_devices(ports=None, exclude_busy=True, want=None, verbose=False):
    """
    전체 포트를 스캔해 장치를 매칭

    want         : 찾을 장치 목록 (기본 전체). 재탐색 시 필요한 것만 지정
    exclude_busy : 다른 프로세스가 열고 있는 포트는 건너뜀

    반환: {'fc': port, 'thl100': port, 'wcm6800': port, 'fc_info': {...}}
    """
    want = set(want or PROBE_ORDER)
    if ports is None:
        ports = list_serial_ports()

    busy = ports_in_use() if exclude_busy else set()
    result = {}

    if verbose:
        print(f'후보 포트 {len(ports)}개' + (f' (사용 중 {len(busy)}개 제외)' if busy else ''))
        for p in ports:
            mark = '  [사용 중 — 건너뜀]' if os.path.realpath(p) in busy else ''
            print(f'  {p}\n    → {os.path.realpath(p)}{mark}')
        print()

    for port in ports:
        if os.path.realpath(port) in busy:
            continue
        if not want - set(result):
            break
        if verbose:
            print(f'[{os.path.basename(port)}]')

        # 1) 9600 센서 — 빠르게 걸러냅니다
        if want & {'thl100', 'wcm6800'} - set(result):
            key, nbytes = _probe_9600(port)
            if key and key in want and key not in result:
                result[key] = port
                if verbose:
                    print(f'    9600: {nbytes}B → {DEVICE_SIGNATURES[key]["name"]} 확정\n')
                continue
            if verbose:
                print(f'    9600: {nbytes}B → 불일치')

        # 2) FC — baud 와 SYSID 까지
        if 'fc' in want and 'fc' not in result:
            if verbose:
                print('    FC 탐색:')
            info = probe_fc(port, verbose=verbose)
            if info:
                result['fc'] = port
                result['fc_info'] = info
                if verbose:
                    print(f'    → FC 확정 (baud {info["baud"]}, SYSID {info["sysid"]})\n')
                continue
        if verbose:
            print('    → 미확정\n')

    return result


def find_port(device_key, fallback=None, exclude_busy=True, verbose=False):
    """단일 장치 재탐색 — 사용 중인 포트는 건너뜁니다."""
    found = detect_devices(exclude_busy=exclude_busy, want=[device_key], verbose=verbose)
    return found.get(device_key, fallback)


def find_fc(exclude_busy=True, verbose=False):
    """FC 재탐색 — fc_info dict 또는 None"""
    found = detect_devices(exclude_busy=exclude_busy, want=['fc'], verbose=verbose)
    return found.get('fc_info')


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════
def _print_fc(info):
    print(f"  포트      : {info['port']}")
    print(f"  baud      : {info['baud']}")
    print(f"  대상 기체 : SYSID {info['sysid']}  COMPID {info['compid']}")
    print(f"  프로토콜  : MAVLink {info['protocol']}"
          + ('  (서명 사용 — 아래 주의 참고)' if info['signed'] else ''))
    print(f"  mavros ID : {info['sysid']}.{info['my_compid']}  (컴패니언 규약)")
    if len(info['autopilots']) > 1:
        print('  ⚠ 비행제어기가 여러 대 보입니다 — tgt_system 을 직접 지정하세요')
        for s, c, n in info['autopilots']:
            print(f'      SYSID {s}.{c}  HEARTBEAT {n}회')
    if info['others']:
        print('  링크의 다른 구성원 (대상에서 제외됨):')
        for s, c, role in info['others']:
            print(f'      {s}.{c}  {role}')


def main():
    args = sys.argv[1:]
    if '--help' in args or '-h' in args:
        print(__doc__)
        return

    if '--device' in args:
        key = args[args.index('--device') + 1]
        port = find_port(key, exclude_busy='--all' not in args)
        if port:
            print(port)
            sys.exit(0)
        sys.exit(1)

    if '--fc' in args:
        info = find_fc(exclude_busy='--all' not in args, verbose=True)
        print('=' * 66)
        if info:
            _print_fc(info)
        else:
            print('  FC 를 찾지 못했습니다.')
        print('=' * 66)
        sys.exit(0 if info else 1)

    as_json = '--json' in args
    found = detect_devices(exclude_busy='--all' not in args, verbose=not as_json)

    if as_json:
        import json
        print(json.dumps(found, indent=2, ensure_ascii=False))
        return

    print('=' * 66)
    print(' 탐색 결과')
    print('=' * 66)
    for key in ('fc', 'thl100', 'wcm6800'):
        name = DEVICE_SIGNATURES[key]['name']
        if key in found:
            print(f'  ✅ {name}\n      {found[key]} → {os.path.realpath(found[key])}')
        else:
            print(f'  ❌ {name} — 찾지 못함')
    if 'fc_info' in found:
        print()
        _print_fc(found['fc_info'])
    print('=' * 66)


if __name__ == '__main__':
    main()
