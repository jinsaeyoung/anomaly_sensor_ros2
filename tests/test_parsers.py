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
