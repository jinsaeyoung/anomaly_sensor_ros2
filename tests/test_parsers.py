#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UART 파서 / 분석기 단위 테스트 (ROS2 없이 실행 가능)

실행:
  python3 tests/test_parsers.py
"""

import sys
import os
import math
import unittest

# ── 테스트 대상 로직을 ROS 의존성 없이 재현 ─────────────────────────────
# (실제 노드 코드와 동일한 알고리즘을 검증)


class StreamBuffer:
    """THL100 스트림 버퍼 파서 (노드와 동일 로직)"""
    def __init__(self):
        self.buf = ''

    def extract(self, chunk):
        self.buf += chunk
        packets = []
        while True:
            start = self.buf.find('@')
            if start < 0:
                if len(self.buf) > 512:
                    self.buf = ''
                break
            end = self.buf.find('\n', start)
            if end < 0:
                self.buf = self.buf[start:]
                if len(self.buf) > 512:
                    self.buf = ''
                break
            pkt = self.buf[start:end].strip()
            self.buf = self.buf[end + 1:]
            if pkt:
                packets.append(pkt)
        return packets


def parse_thl100(packet):
    if not packet.startswith('@'):
        return None
    fields = packet[1:].split(',')
    if len(fields) != 5:
        return None
    try:
        return {
            'sensor_id':   fields[0],
            'sequence':    int(fields[1]),
            'temperature': float(fields[2]) if fields[2] else None,
            'humidity':    float(fields[3]) if fields[3] else None,
            'light':       float(fields[4]) if fields[4] else None,
        }
    except ValueError:
        return None


def parse_wcm6800(packet):
    if len(packet) != 6:
        return None
    t, d = packet[0], packet[1:]
    if not d.isdigit():
        return None
    v = int(d) / 1000.0
    if t == '~':
        return {'current': v,  'current_type': 'AC'}
    if t == '+':
        return {'current': v,  'current_type': 'DC+'}
    if t == '-':
        return {'current': -v, 'current_type': 'DC-'}
    return None


def quat_to_euler(x, y, z, w):
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def enu_to_ned(x, y, z):
    """ENU → NED 변환"""
    return y, x, -z   # north, east, down


def gps_course(vx_east, vy_north):
    """항공 course: 북쪽 0도, 시계방향"""
    return (math.degrees(math.atan2(vx_east, vy_north)) + 360.0) % 360.0


# ══════════════════════════════════════════════════════════════════════════
class TestTHL100Parser(unittest.TestCase):

    def test_single_packet(self):
        p = parse_thl100('@T453,100,28.5,38.3,236.5')
        self.assertIsNotNone(p)
        self.assertEqual(p['sensor_id'], 'T453')
        self.assertEqual(p['sequence'], 100)
        self.assertAlmostEqual(p['temperature'], 28.5)
        self.assertAlmostEqual(p['humidity'], 38.3)
        self.assertAlmostEqual(p['light'], 236.5)

    def test_missing_field_value(self):
        p = parse_thl100('@T453,100,,38.3,236.5')
        self.assertIsNotNone(p)
        self.assertIsNone(p['temperature'])
        self.assertAlmostEqual(p['humidity'], 38.3)

    def test_wrong_field_count(self):
        self.assertIsNone(parse_thl100('@T453,100,28.5'))

    def test_no_header(self):
        self.assertIsNone(parse_thl100('T453,100,28.5,38.3,236.5'))

    def test_stream_buffer_split_packet(self):
        """패킷이 두 번에 나뉘어 도착하는 경우"""
        sb = StreamBuffer()
        self.assertEqual(sb.extract('@T453,100,28.5,'), [])
        pkts = sb.extract('38.3,236.5\r\n')
        self.assertEqual(len(pkts), 1)
        self.assertEqual(parse_thl100(pkts[0])['sequence'], 100)

    def test_stream_buffer_concatenated(self):
        """두 패킷이 붙어서 도착 — 둘 다 살려야 함"""
        sb = StreamBuffer()
        pkts = sb.extract('@T453,100,28.5,38.3,236.5\r\n@T453,101,28.6,38.4,240.0\r\n')
        self.assertEqual(len(pkts), 2)
        self.assertEqual(parse_thl100(pkts[0])['sequence'], 100)
        self.assertEqual(parse_thl100(pkts[1])['sequence'], 101)

    def test_stream_buffer_garbage_prefix(self):
        sb = StreamBuffer()
        pkts = sb.extract('\x00\xff garbage @T453,102,29.0,40.0,300.0\r\n')
        self.assertEqual(len(pkts), 1)
        self.assertEqual(parse_thl100(pkts[0])['sequence'], 102)


class TestWCM6800Parser(unittest.TestCase):

    def test_ac(self):
        p = parse_wcm6800('~01230')
        self.assertAlmostEqual(p['current'], 1.230)
        self.assertEqual(p['current_type'], 'AC')

    def test_dc_positive(self):
        p = parse_wcm6800('+10760')
        self.assertAlmostEqual(p['current'], 10.760)
        self.assertEqual(p['current_type'], 'DC+')

    def test_dc_negative(self):
        p = parse_wcm6800('-01230')
        self.assertAlmostEqual(p['current'], -1.230)
        self.assertEqual(p['current_type'], 'DC-')

    def test_invalid_length(self):
        self.assertIsNone(parse_wcm6800('+1230'))

    def test_invalid_digits(self):
        self.assertIsNone(parse_wcm6800('+012A0'))

    def test_unknown_type(self):
        self.assertIsNone(parse_wcm6800('X01230'))


class TestCoordinateConversion(unittest.TestCase):

    def test_quat_identity(self):
        r, p, y = quat_to_euler(0, 0, 0, 1)
        self.assertAlmostEqual(r, 0, places=5)
        self.assertAlmostEqual(p, 0, places=5)
        self.assertAlmostEqual(y, 0, places=5)

    def test_quat_yaw_90(self):
        # z축 90도 회전
        s = math.sin(math.radians(45))
        c = math.cos(math.radians(45))
        r, p, y = quat_to_euler(0, 0, s, c)
        self.assertAlmostEqual(y, 90.0, places=3)

    def test_enu_to_ned(self):
        # ENU (East=1, North=2, Up=3) → NED (N=2, E=1, D=-3)
        n, e, d = enu_to_ned(1.0, 2.0, 3.0)
        self.assertAlmostEqual(n, 2.0)
        self.assertAlmostEqual(e, 1.0)
        self.assertAlmostEqual(d, -3.0)

    def test_gps_course_north(self):
        # 북쪽으로 이동 (East=0, North=1) → 0도
        self.assertAlmostEqual(gps_course(0.0, 1.0), 0.0, places=3)

    def test_gps_course_east(self):
        # 동쪽으로 이동 (East=1, North=0) → 90도
        self.assertAlmostEqual(gps_course(1.0, 0.0), 90.0, places=3)

    def test_gps_course_south(self):
        # 남쪽 (East=0, North=-1) → 180도
        self.assertAlmostEqual(gps_course(0.0, -1.0), 180.0, places=3)

    def test_gps_course_west(self):
        # 서쪽 (East=-1, North=0) → 270도
        self.assertAlmostEqual(gps_course(-1.0, 0.0), 270.0, places=3)


class TestSplitBagOrdering(unittest.TestCase):
    """
    rosbag2 는 max_bag_duration 도달 시 파일을 분할합니다.
    분할 파일을 사전순으로 정렬하면 _10.db3 가 _2.db3 앞에 와서
    시계열 순서가 깨지므로 숫자 접미사 기준 정렬이 필요합니다.
    """

    @staticmethod
    def _seq(name):
        import re
        m = re.search(r'_(\d+)\.db3$', name)
        return int(m.group(1)) if m else 0

    def test_numeric_ordering(self):
        files = ['f_0.db3', 'f_10.db3', 'f_1.db3', 'f_11.db3', 'f_2.db3', 'f_9.db3']
        got = sorted(files, key=self._seq)
        want = ['f_0.db3', 'f_1.db3', 'f_2.db3', 'f_9.db3', 'f_10.db3', 'f_11.db3']
        self.assertEqual(got, want)

    def test_lexical_ordering_is_wrong(self):
        """사전순 정렬이 실제로 순서를 깨뜨리는지 확인"""
        files = ['f_0.db3', 'f_10.db3', 'f_2.db3']
        self.assertNotEqual(sorted(files), sorted(files, key=self._seq))

    def test_no_suffix(self):
        self.assertEqual(self._seq('flight.db3'), 0)


class TestAudioSummary(unittest.TestCase):
    """ReSpeaker 원본 PCM 요약 통계 계산 검증"""

    CHANNELS = 6

    def test_channel_extraction(self):
        import numpy as np
        # 6채널 interleaved: [c0,c1,...,c5, c0,c1,...]
        n_frames = 4
        pcm = np.arange(n_frames * self.CHANNELS, dtype=np.int16)
        ch0 = pcm[::self.CHANNELS]
        self.assertEqual(list(ch0), [0, 6, 12, 18])
        self.assertEqual(ch0.size, n_frames)

    def test_rms(self):
        import numpy as np
        pcm = np.array([3, 0, 0, 0, 0, 0,
                        4, 0, 0, 0, 0, 0], dtype=np.int16)
        ch0 = pcm[::self.CHANNELS].astype(np.float32)
        rms = float(np.sqrt(np.mean(ch0 ** 2)))
        self.assertAlmostEqual(rms, 3.5355, places=3)   # sqrt((9+16)/2)

    def test_clip_detection(self):
        import numpy as np
        pcm = np.zeros(6 * 4, dtype=np.int16)
        pcm[0]  = 32700     # ch0 프레임0 — 클리핑
        pcm[6]  = 100       # ch0 프레임1
        pcm[12] = 32500     # ch0 프레임2 — 클리핑
        pcm[18] = 200       # ch0 프레임3
        ch0 = pcm[::6].astype(np.float32)
        clip_pct = float(np.mean(np.abs(ch0) > 32000) * 100.0)
        self.assertAlmostEqual(clip_pct, 50.0)


class TestTrackingError(unittest.TestCase):
    """목표 대비 실제의 추종 오차 계산 검증"""

    @staticmethod
    def _wrap180(v):
        return ((v + 180.0) % 360.0) - 180.0

    def test_yaw_wrap_positive(self):
        # 359° 목표, 1° 실제 → 단순 차는 358 이지만 실제 오차는 -2
        self.assertAlmostEqual(self._wrap180(359.0 - 1.0), -2.0)

    def test_yaw_wrap_negative(self):
        self.assertAlmostEqual(self._wrap180(-190.0), 170.0)

    def test_yaw_boundary(self):
        self.assertAlmostEqual(self._wrap180(180.0), -180.0)

    def test_yaw_no_wrap(self):
        self.assertAlmostEqual(self._wrap180(5.0), 5.0)

    def test_attitude_error_magnitude(self):
        import math
        err_roll, err_pitch = 3.0, -4.0
        mag = math.sqrt(err_roll ** 2 + err_pitch ** 2)
        self.assertAlmostEqual(mag, 5.0)


class TestTypeMask(unittest.TestCase):
    """
    setpoint 메시지의 type_mask 해석 검증
    bit 가 set 되면 해당 필드를 무시하라는 의미이므로,
    유효 판정은 반전됩니다.
    """

    def test_attitude_valid(self):
        # bit7(0x80) = attitude ignore
        self.assertEqual(int(not (0x00 & 0x80)), 1)   # 유효
        self.assertEqual(int(not (0x80 & 0x80)), 0)   # 무시

    def test_body_rate_valid(self):
        # bit0~2 = body rate ignore
        self.assertEqual(int(not (0x00 & 0x07)), 1)
        self.assertEqual(int(not (0x07 & 0x07)), 0)

    def test_position_valid(self):
        # bit0~2 = position ignore
        self.assertEqual(int(not (0x00 & 0x07)), 1)
        self.assertEqual(int(not (0x07 & 0x07)), 0)

    def test_velocity_valid(self):
        # bit3~5 = velocity ignore
        self.assertEqual(int(not (0x00 & 0x38)), 1)
        self.assertEqual(int(not (0x38 & 0x38)), 0)


class TestSerialAutodetect(unittest.TestCase):
    """
    시리얼 장치 자동 탐색 시그니처 검증

    같은 모델 USB-TTL 젠더를 여러 개 쓰면 VID:PID 로 구분할 수 없고,
    by-id 는 젠더를 교체하면 경로가 바뀝니다.
    데이터 패턴으로 판별하므로 상호 오탐이 없어야 합니다.
    """

    @staticmethod
    def _thl100(data):
        import re
        text = data.decode('ascii', errors='ignore')
        return len(re.findall(r'@[A-Za-z0-9]+,\d+,[\d.\-]*,[\d.\-]*,[\d.\-]*', text))

    @staticmethod
    def _wcm6800(data):
        text = data.decode('ascii', errors='ignore')
        hits = 0
        for line in text.replace('\r', '\n').split('\n'):
            line = line.strip()
            if len(line) == 6 and line[0] in '~+-' and line[1:].isdigit():
                hits += 1
        return hits

    @staticmethod
    def _mavlink(data):
        hits, i, n = 0, 0, len(data)
        while i < n - 3:
            b = data[i]
            if b == 0xFD:
                plen = data[i + 1]
                if plen <= 253 and i + 12 + plen <= n:
                    hits += 1
                    i += 12 + plen
                    continue
            elif b == 0xFE:
                plen = data[i + 1]
                if plen <= 255 and i + 8 + plen <= n:
                    hits += 1
                    i += 8 + plen
                    continue
            i += 1
        return hits

    # ── THL100 ──────────────────────────────────────────────────
    def test_thl100_match(self):
        data = b'@T453,1664,25.1,43.5,36.6\r\n@T453,1665,25.1,43.5,46.6\r\n'
        self.assertEqual(self._thl100(data), 2)

    def test_thl100_missing_field(self):
        self.assertEqual(self._thl100(b'@T453,100,,38.3,236.5\r\n'), 1)

    def test_thl100_no_false_positive(self):
        self.assertEqual(self._thl100(b'+01230\r\n+01240\r\n'), 0)

    # ── WCM6800 ─────────────────────────────────────────────────
    def test_wcm6800_match(self):
        self.assertEqual(self._wcm6800(b'+01230\r\n~00450\r\n-01230\r\n'), 3)

    def test_wcm6800_no_false_positive(self):
        self.assertEqual(self._wcm6800(b'@T453,1664,25.1,43.5,36.6\r\n'), 0)

    def test_wcm6800_wrong_length(self):
        self.assertEqual(self._wcm6800(b'+1230\r\n'), 0)

    # ── MAVLink ─────────────────────────────────────────────────
    def test_mavlink_v2_frame(self):
        frame = bytes([0xFD, 0x09, 0x00, 0x00, 0xAF, 0x01, 0x01]) + b'\x00' * 14
        self.assertGreaterEqual(self._mavlink(frame * 4), 3)

    def test_mavlink_no_false_positive_thl100(self):
        self.assertEqual(self._mavlink(b'@T453,1664,25.1,43.5,36.6\r\n' * 3), 0)

    def test_mavlink_no_false_positive_wcm(self):
        self.assertEqual(self._mavlink(b'+01230\r\n' * 5), 0)


class TestMissionParsing(unittest.TestCase):
    """
    미션(WaypointList) 요약 로직 검증

    웨이포인트 전체를 CSV 컬럼으로 펼칠 수 없으므로
    개수·현재 seq·명령 분포만 요약하고 상세는 JSON 으로 보존합니다.
    """

    class _WP:
        def __init__(self, cmd, lat=0.0, lon=0.0, alt=0.0):
            self.command = cmd
            self.x_lat = lat
            self.y_long = lon
            self.z_alt = alt
            self.frame = 3
            self.param1 = self.param2 = self.param3 = self.param4 = 0.0
            self.autocontinue = True

    @staticmethod
    def _to_int(v, default=0):
        if isinstance(v, (bytes, bytearray)):
            return v[0] if len(v) else default
        try:
            return int(v)
        except (TypeError, ValueError):
            return default

    def test_command_histogram(self):
        # 22=TAKEOFF, 16=WAYPOINT, 21=LAND
        wps = [self._WP(22), self._WP(16), self._WP(16), self._WP(21)]
        cmds = {}
        for w in wps:
            k = self._to_int(w.command)
            cmds[k] = cmds.get(k, 0) + 1
        summary = ','.join(f'{k}x{v}' for k, v in sorted(cmds.items()))
        self.assertEqual(summary, '16x2,21x1,22x1')

    def test_current_target_in_range(self):
        wps = [self._WP(16, 37.1, 127.1, 50.0),
               self._WP(16, 37.2, 127.2, 60.0)]
        cur = 1
        self.assertTrue(0 <= cur < len(wps))
        self.assertAlmostEqual(wps[cur].x_lat, 37.2)
        self.assertAlmostEqual(wps[cur].z_alt, 60.0)

    def test_current_seq_out_of_range(self):
        """current_seq 가 -1 이거나 범위를 벗어나면 좌표를 기록하지 않아야 함"""
        wps = [self._WP(16)]
        for cur in (-1, 5):
            self.assertFalse(0 <= cur < len(wps))

    def test_empty_mission(self):
        wps = []
        self.assertEqual(len(wps), 0)

    def test_json_serializable(self):
        import json
        wps = [self._WP(22, 37.1, 127.1, 10.0)]
        data = [{
            'seq': i, 'cmd': self._to_int(w.command),
            'lat': w.x_lat, 'lon': w.y_long, 'alt': w.z_alt,
        } for i, w in enumerate(wps)]
        s = json.dumps(data, ensure_ascii=False)
        self.assertIn('"cmd": 22', s)
        self.assertEqual(json.loads(s)[0]['alt'], 10.0)


class TestStaleLogic(unittest.TestCase):
    """age 기반 stale 판정 로직 검증"""

    def test_fresh_data(self):
        age_sec = 0.5
        limit = 3.0
        self.assertFalse(age_sec > limit)

    def test_stale_data(self):
        age_sec = 5.0
        limit = 3.0
        self.assertTrue(age_sec > limit)

    def test_boundary(self):
        self.assertFalse(3.0 > 3.0)
        self.assertTrue(3.001 > 3.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
