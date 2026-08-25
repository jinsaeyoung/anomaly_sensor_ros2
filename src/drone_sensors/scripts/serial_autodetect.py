#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
시리얼 장치 자동 탐색

USB-TTL 젠더는 같은 모델을 여러 개 쓰면 VID:PID 로 구분할 수 없고,
by-id 는 젠더 개체를 교체하면 경로가 바뀝니다.
이 모듈은 각 포트를 실제로 열어 **데이터 시그니처**로 장치를 판별합니다.

판별 기준:
  THL100   9600bps   '@' 로 시작, 콤마 5필드   예: @T453,1234,25.1,43.5,36.6
  WCM6800  9600bps   '~'/'+'/'-' + 5자리 숫자   예: +01230
  FC       921600bps MAVLink 프레임 (0xFD/0xFE)

사용법 (모듈):
    from serial_autodetect import detect_device, find_port
    port = find_port('thl100')

사용법 (CLI):
    python3 serial_autodetect.py              # 전체 포트 스캔
    python3 serial_autodetect.py --json       # JSON 출력
    python3 serial_autodetect.py --device fc  # 특정 장치 경로만 출력
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
# 장치 시그니처 정의
#   baud       : 통신 속도
#   probe_sec  : 판별에 사용할 최대 수신 시간
#   min_hits   : 이 횟수 이상 패턴이 맞아야 확정 (오탐 방지)
#   matcher    : 수신 바이트 → 일치 횟수
# ══════════════════════════════════════════════════════════════════════════════

def _match_thl100(data: bytes) -> int:
    """@sensorID,seq,temp,humi,light\\r\\n 형식"""
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
    """
    MAVLink 프레임 시작 바이트 뒤 페이로드 길이가 유효한지 확인
      v2: 0xFD, v1: 0xFE
    단순히 0xFD 개수만 세면 잡음도 잡히므로 프레임 구조를 확인합니다.
    """
    hits = 0
    i = 0
    n = len(data)
    while i < n - 3:
        b = data[i]
        if b == 0xFD:            # MAVLink v2: STX, LEN, INCOMPAT, COMPAT, SEQ, SYSID, COMPID...
            plen = data[i + 1]
            if plen <= 253 and i + 12 + plen <= n:
                hits += 1
                i += 12 + plen
                continue
        elif b == 0xFE:          # MAVLink v1: STX, LEN, SEQ, SYSID, COMPID, MSGID...
            plen = data[i + 1]
            if plen <= 255 and i + 8 + plen <= n:
                hits += 1
                i += 8 + plen
                continue
        i += 1
    return hits


DEVICE_SIGNATURES = {
    'thl100': {
        'name':      'OSTSen-THL100 (온습도/조도)',
        'baud':      9600,
        'probe_sec': 3.0,        # 1Hz 이므로 최소 2~3초 필요
        'min_hits':  1,
        'matcher':   _match_thl100,
    },
    'wcm6800': {
        'name':      'Winson WCM6800 (전류계)',
        'baud':      9600,
        'probe_sec': 2.5,        # 3Hz — 2패킷에 약 0.7초, 여유 포함
        'min_hits':  2,
        'matcher':   _match_wcm6800,
    },
    'fc': {
        'name':      'ArduPilot FC (MAVLink)',
        'baud':      921600,
        'probe_sec': 2.0,        # 고주기지만 링크 상태에 따라 여유 확보
        'min_hits':  3,
        'matcher':   _match_mavlink,
    },
}

# 탐색 우선순위
#   9600 장치를 먼저 확정합니다.
#   FC(921600) 를 먼저 돌리면 모든 포트를 고속으로 열었다 닫게 되어
#   직후 9600 재접속 시 데이터를 놓치기 쉽습니다.
#   저속 장치가 확정되면 그 포트는 후보에서 빠져 FC 탐색도 빨라집니다.
PROBE_ORDER = ['wcm6800', 'thl100', 'fc']


def list_serial_ports():
    """
    후보 시리얼 포트 목록

    by-id 경로를 우선 사용합니다(포트 번호가 바뀌어도 안정적).
    by-id 가 없는 장치는 /dev/ttyUSB*, /dev/ttyACM* 로 보완합니다.
    """
    ports = []
    seen_real = set()

    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        real = os.path.realpath(p)
        if real not in seen_real:
            ports.append(p)
            seen_real.add(real)

    for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
        for p in sorted(glob.glob(pattern)):
            real = os.path.realpath(p)
            if real not in seen_real:
                ports.append(p)
                seen_real.add(real)

    return ports


# 포트를 닫은 뒤 다시 열기까지 필요한 안정화 시간(초)
# 다른 baud 로 열었다 닫으면 드라이버가 정리될 시간이 필요합니다.
# 이 시간이 없으면 직후 재접속 시 초기 데이터를 놓칩니다.
PORT_SETTLE_SEC = 0.3


def probe_port(port, device_key, verbose=False):
    """
    지정한 포트가 해당 장치인지 확인

    반환: (일치 여부, 일치 횟수, 수신 바이트 수)
    """
    if serial is None:
        raise RuntimeError('pyserial 미설치 — pip3 install pyserial')

    sig = DEVICE_SIGNATURES[device_key]
    buf = b''

    try:
        ser = serial.Serial(
            port=port,
            baudrate=sig['baud'],
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        )
    except Exception as e:
        if verbose:
            print(f'    열기 실패: {e}')
        return False, 0, 0

    try:
        # 포트를 연 직후에는 이전 baud 로 수신된 잔여 바이트가 남아 있어
        # 잘못된 패턴으로 읽힐 수 있습니다. 잠시 대기 후 버퍼를 비웁니다.
        time.sleep(0.15)
        ser.reset_input_buffer()

        deadline = time.monotonic() + sig['probe_sec']
        while time.monotonic() < deadline:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                # 충분히 모였으면 조기 판정
                if sig['matcher'](buf) >= sig['min_hits']:
                    break
            else:
                time.sleep(0.02)
    except Exception as e:
        if verbose:
            print(f'    읽기 오류: {e}')
    finally:
        try:
            ser.close()
        except Exception:
            pass
        # 다음 probe 가 같은 포트를 다른 baud 로 열 수 있으므로 안정화 대기
        time.sleep(PORT_SETTLE_SEC)

    hits = sig['matcher'](buf)
    return hits >= sig['min_hits'], hits, len(buf)


def probe_port_all(port, verbose=False):
    """
    한 포트에서 baud 별로 한 번씩만 열어 모든 장치 시그니처를 동시 검사

    포트 × 장치 조합마다 열고 닫으면 시간이 오래 걸리고,
    잦은 open/close 가 USB 재열거링을 유발할 수 있습니다.
    같은 baud 를 쓰는 장치는 한 번의 수신으로 함께 판별합니다.

    반환: {device_key: (hits, nbytes)}
    """
    # baud 별로 어떤 장치를 검사할지 묶기
    by_baud = {}
    for key in PROBE_ORDER:
        sig = DEVICE_SIGNATURES[key]
        by_baud.setdefault(sig['baud'], []).append(key)

    out = {}
    for baud, keys in sorted(by_baud.items()):
        probe_sec = max(DEVICE_SIGNATURES[k]['probe_sec'] for k in keys)
        min_needed = {k: DEVICE_SIGNATURES[k]['min_hits'] for k in keys}
        buf = b''

        try:
            ser = serial.Serial(
                port=port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.2,
            )
        except Exception as e:
            if verbose:
                print(f'      {baud}bps 열기 실패: {e}')
            for k in keys:
                out[k] = (0, 0)
            continue

        try:
            time.sleep(0.15)          # 이전 baud 잔여 데이터 정리
            ser.reset_input_buffer()
            deadline = time.monotonic() + probe_sec
            while time.monotonic() < deadline:
                n = ser.in_waiting
                if n:
                    buf += ser.read(n)
                    # 이 baud 의 장치 중 하나라도 확정되면 조기 종료
                    if any(DEVICE_SIGNATURES[k]['matcher'](buf) >= min_needed[k]
                           for k in keys):
                        break
                else:
                    time.sleep(0.02)
        except Exception:
            pass
        finally:
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(PORT_SETTLE_SEC)

        for k in keys:
            out[k] = (DEVICE_SIGNATURES[k]['matcher'](buf), len(buf))

    return out


def detect_devices(ports=None, verbose=False):
    """
    전체 포트를 스캔해 장치를 매칭

    포트마다 한 번씩만 순회하며 baud 별로 열어 모든 장치를 동시 판별합니다.
    (장치×포트 조합마다 여닫으면 느리고 USB 부하가 큼)

    반환: {'thl100': '/dev/...', 'wcm6800': ..., 'fc': ...}
          찾지 못한 항목은 키가 없습니다.
    """
    if ports is None:
        ports = list_serial_ports()

    result = {}
    used = set()

    if verbose:
        print(f'후보 포트 {len(ports)}개')
        for p in ports:
            print(f'  {p}')
            print(f'    → {os.path.realpath(p)}')
        print()

    # ── 1차: 포트별 1회 순회로 판별 ────────────────────────────────
    for port in ports:
        if os.path.realpath(port) in used:
            continue
        if verbose:
            print(f'[{os.path.basename(port)}]')

        hits_map = probe_port_all(port, verbose=verbose)

        # 가장 확실한 매칭 선택 (min_hits 대비 초과분이 큰 것)
        best, best_score = None, 0
        for key, (hits, nbytes) in hits_map.items():
            need = DEVICE_SIGNATURES[key]['min_hits']
            if verbose and nbytes:
                mark = ' ✓' if hits >= need else ''
                print(f'    {key:8} {nbytes:5d}bytes 일치 {hits:3d}/{need}{mark}')
            if hits >= need and key not in result and hits > best_score:
                best, best_score = key, hits

        if best:
            result[best] = port
            used.add(os.path.realpath(port))
            if verbose:
                print(f'    → {DEVICE_SIGNATURES[best]["name"]} 확정')
        elif verbose:
            print('    → 미확정')
        if verbose:
            print()

    # ── 2차: 미확정 장치만 남은 포트에서 재시도 ────────────────────
    missing = [k for k in PROBE_ORDER if k not in result]
    if missing:
        if verbose:
            print(f'--- 재시도 (미확정: {", ".join(missing)}) ---\n')
        for key in missing:
            sig = DEVICE_SIGNATURES[key]
            for port in ports:
                if os.path.realpath(port) in used:
                    continue
                ok, hits, nbytes = probe_port(port, key, verbose=False)
                if verbose:
                    print(f'  {key:8} @ {os.path.basename(port)} '
                          f'{nbytes}bytes 일치 {hits}회 {"→ 확정" if ok else ""}')
                if ok:
                    result[key] = port
                    used.add(os.path.realpath(port))
                    break
        if verbose:
            print()

    return result


def find_port(device_key, fallback=None, verbose=False):
    """
    단일 장치의 포트를 찾습니다.
    실패하면 fallback 을 반환합니다(기존 고정 경로 등).
    """
    if device_key not in DEVICE_SIGNATURES:
        raise ValueError(f'알 수 없는 장치: {device_key}')

    for port in list_serial_ports():
        ok, _, _ = probe_port(port, device_key, verbose=verbose)
        if ok:
            return port
    return fallback


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════

def main():
    args = sys.argv[1:]

    if '--help' in args or '-h' in args:
        print(__doc__)
        return

    as_json = '--json' in args

    single = None
    if '--device' in args:
        single = args[args.index('--device') + 1]

    if single:
        port = find_port(single)
        if port:
            print(port)
            sys.exit(0)
        else:
            sys.exit(1)

    if not as_json:
        print('=' * 66)
        print(' 시리얼 장치 자동 탐색')
        print('=' * 66)
        print()

    found = detect_devices(verbose=not as_json)

    if as_json:
        import json
        print(json.dumps(found, indent=2))
        return

    print('=' * 66)
    print(' 탐색 결과')
    print('=' * 66)
    for key in PROBE_ORDER:
        name = DEVICE_SIGNATURES[key]['name']
        if key in found:
            print(f'  ✅ {name}')
            print(f'      {found[key]}')
            print(f'      → {os.path.realpath(found[key])}')
        else:
            print(f'  ❌ {name} — 찾지 못함')
    print()

    if len(found) < len(PROBE_ORDER):
        print(' 찾지 못한 장치가 있습니다. 확인 사항:')
        print('   - 장치 전원 및 USB 연결')
        print('   - FC 는 TELEM 포트 설정(SERIALn_PROTOCOL=2) 필요')
        print('   - baud 가 다르면 DEVICE_SIGNATURES 수정')
        print()

    print(' launch 실행 예시:')
    parts = ['ros2 launch drone_sensors drone_sensor_launch.py']
    if 'fc' in found:
        parts.append(f'  fcu_url:={found["fc"]}:921600')
    if 'thl100' in found:
        parts.append(f'  thl100_port:={found["thl100"]}')
    if 'wcm6800' in found:
        parts.append(f'  wcm6800_port:={found["wcm6800"]}')
    print(' \\\n'.join(parts))
    print('=' * 66)


if __name__ == '__main__':
    main()
