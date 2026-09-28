#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
단위 테스트 — 제품 코드를 직접 불러와 검사합니다 (ROS2 없이도 실행 가능)

  python3 tests/test_parsers.py

ROS2 가 설치된 장비에서는 실제 rclpy 등을 쓰고, 없는 환경에서는
노드 클래스 정의에 필요한 최소한의 가짜 모듈만 채웁니다.
테스트가 로직을 복사해 두면 제품 코드가 바뀌어도 알아채지 못하므로,
모든 검사는 저장소의 실제 파일을 대상으로 합니다.
"""

import importlib.util
import math
import os
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


# ══════════════════════════════════════════════════════════════════════════════
# 제품 코드 로더
# ══════════════════════════════════════════════════════════════════════════════
def _stub_if_missing():
    """ROS 모듈이 없을 때만 클래스 정의에 필요한 최소 가짜 모듈을 채웁니다."""
    def need(name):
        try:
            __import__(name)
            return False
        except Exception:
            return True

    def mod(name):
        m = sys.modules.get(name) or types.ModuleType(name)
        sys.modules[name] = m
        return m

    if need('rclpy'):
        mod('rclpy')
        mod('rclpy.node').Node = type('Node', (), {})
        q = mod('rclpy.qos')
        q.QoSProfile = lambda **k: None
        for a in ('ReliabilityPolicy', 'DurabilityPolicy', 'HistoryPolicy'):
            setattr(q, a, types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=0, VOLATILE=0,
                                                TRANSIENT_LOCAL=0, KEEP_LAST=0))
        mod('rclpy.serialization').deserialize_message = lambda *a: None
    if need('serial'):
        s = mod('serial')
        s.Serial = object
        s.SerialException = type('SerialException', (Exception,), {})
        s.EIGHTBITS, s.PARITY_NONE, s.STOPBITS_ONE = 8, 'N', 1
    if need('std_msgs.msg'):
        m = mod('std_msgs.msg'); mod('std_msgs')
        for n in ('String', 'Float32', 'Int32', 'Bool', 'UInt8MultiArray', 'Header'):
            setattr(m, n, type(n, (), {}))
    if need('rosidl_runtime_py.utilities'):
        mod('rosidl_runtime_py'); mod('rosidl_runtime_py.utilities').get_message = lambda t: None
    if need('ament_index_python.packages'):
        mod('ament_index_python')
        p = mod('ament_index_python.packages')
        p.get_package_share_directory = lambda n: '/nonexistent'


def _load(name, *relpaths):
    """저장소 구조(scripts/, src/...)와 평면 배치 모두에서 파일을 찾아 불러옵니다."""
    for rel in relpaths + (os.path.basename(relpaths[0]),):
        for base in (ROOT, HERE):
            path = os.path.join(base, rel)
            if os.path.exists(path):
                _stub_if_missing()
                spec = importlib.util.spec_from_file_location(name, path)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                return module
    raise unittest.SkipTest(f'{relpaths[0]} 를 찾을 수 없음')


_cache = {}


def prod(key):
    paths = {
        'thl':  ('src/thl100_sensor/thl100_sensor/thl100_uart_node.py',),
        'wcm':  ('src/wcm6800_sensor/wcm6800_sensor/wcm6800_uart_node.py',),
        'ab':   ('scripts/analyze_bag.py',),
        'sa':   ('scripts/serial_autodetect.py',),
        'vb':   ('scripts/verify_bag.py',),
    }
    if key not in _cache:
        _cache[key] = _load(f'prod_{key}', *paths[key])
    return _cache[key]


def _w(path):
    """yaml.dump 대상 — 닫히지 않은 파일 경고 방지용 래퍼"""
    class _F:
        def __init__(self): self.buf = []
        def write(self, s): self.buf.append(s)
        def __del__(self):
            with open(path, 'w') as f:
                f.write(''.join(self.buf))
    return _F()


NS = types.SimpleNamespace
V3 = lambda x=0.0, y=0.0, z=0.0: NS(x=x, y=y, z=z)


# ══════════════════════════════════════════════════════════════════════════════
# UART 파서 (노드 메서드 직접 호출)
# ══════════════════════════════════════════════════════════════════════════════
class TestTHL100Parser(unittest.TestCase):

    def setUp(self):
        self.node = object.__new__(prod('thl').THL100Node)   # __init__ 없이 메서드만 사용
        self.node._buffer = ''

    def test_single_packet(self):
        p = self.node._parse('@T453,100,28.5,38.3,236.5')
        self.assertEqual((p['sensor_id'], p['sequence']), ('T453', 100))
        self.assertAlmostEqual(p['temperature'], 28.5)
        self.assertAlmostEqual(p['humidity'], 38.3)
        self.assertAlmostEqual(p['light'], 236.5)

    def test_missing_field_value(self):
        p = self.node._parse('@T453,100,,38.3,236.5')
        self.assertIsNone(p['temperature'])
        self.assertAlmostEqual(p['humidity'], 38.3)

    def test_wrong_field_count(self):
        self.assertIsNone(self.node._parse('@T453,100,28.5'))

    def test_no_header(self):
        self.assertIsNone(self.node._parse('T453,100,28.5,38.3,236.5'))

    def test_split_packet(self):
        """패킷이 두 번에 나뉘어 도착"""
        self.assertEqual(self.node._extract_packets('@T453,100,28.5,'), [])
        pkts = self.node._extract_packets('38.3,236.5\r\n')
        self.assertEqual(self.node._parse(pkts[0])['sequence'], 100)

    def test_concatenated(self):
        """두 패킷이 붙어서 도착 — 둘 다 살려야 함"""
        pkts = self.node._extract_packets('@T453,100,28.5,38.3,236.5\r\n@T453,101,28.6,38.4,240.0\r\n')
        self.assertEqual([self.node._parse(p)['sequence'] for p in pkts], [100, 101])

    def test_garbage_prefix(self):
        pkts = self.node._extract_packets('\x00\xff garbage @T453,102,29.0,40.0,300.0\r\n')
        self.assertEqual(self.node._parse(pkts[0])['sequence'], 102)


class TestWCM6800Parser(unittest.TestCase):

    def setUp(self):
        self.node = object.__new__(prod('wcm').WCM6800Node)
        self.node._buffer = ''

    def _p(self, s):
        return self.node._parse(s)

    def test_ac(self):
        p = self._p('~01230')
        self.assertAlmostEqual(p['current'], 1.230)
        self.assertEqual(p['current_type'], 'AC')

    def test_dc_positive(self):
        p = self._p('+10760')
        self.assertAlmostEqual(p['current'], 10.760)
        self.assertEqual(p['current_type'], 'DC+')

    def test_dc_negative(self):
        p = self._p('-01230')
        self.assertAlmostEqual(p['current'], -1.230)
        self.assertEqual(p['current_type'], 'DC-')

    def test_invalid_length(self):
        self.assertIsNone(self._p('+1230'))

    def test_invalid_digits(self):
        self.assertIsNone(self._p('+012A0'))

    def test_unknown_type(self):
        self.assertIsNone(self._p('X01230'))


# ══════════════════════════════════════════════════════════════════════════════
# 분석기 (analyze_bag.parse_msg 등 직접 호출)
# ══════════════════════════════════════════════════════════════════════════════
class TestCoordinateConversion(unittest.TestCase):

    def setUp(self):
        self.ab = prod('ab')

    def test_quat_identity(self):
        for v in self.ab.quat_to_euler(0, 0, 0, 1):
            self.assertAlmostEqual(v, 0, places=5)

    def test_quat_yaw_90(self):
        s, c = math.sin(math.radians(45)), math.cos(math.radians(45))
        self.assertAlmostEqual(self.ab.quat_to_euler(0, 0, s, c)[2], 90.0, places=3)

    def test_enu_to_ned(self):
        """ENU (E=1, N=2, U=3) → NED (N=2, E=1, D=-3)"""
        row = self.ab.parse_msg('/mavros/local_position/pose', NS(pose=NS(position=V3(1.0, 2.0, 3.0))))
        self.assertEqual((row['LocalNED_N'], row['LocalNED_E'], row['LocalNED_D']), (2.0, 1.0, -3.0))

    def _course(self, east, north):
        row = self.ab.parse_msg('/mavros/global_position/raw/gps_vel', NS(twist=NS(linear=V3(east, north))))
        return row['GPS_CourseAngle']

    def test_course_north(self):
        self.assertAlmostEqual(self._course(0.0, 1.0), 0.0, places=3)

    def test_course_east(self):
        self.assertAlmostEqual(self._course(1.0, 0.0), 90.0, places=3)

    def test_course_south(self):
        self.assertAlmostEqual(self._course(0.0, -1.0), 180.0, places=3)

    def test_course_west(self):
        self.assertAlmostEqual(self._course(-1.0, 0.0), 270.0, places=3)


class TestSplitBagOrdering(unittest.TestCase):
    """분할 bag 은 _10 이 _2 보다 앞서지 않도록 숫자 순서로 읽어야 함"""

    def setUp(self):
        self.ab = prod('ab')
        self.dir = tempfile.mkdtemp()

    def _touch(self, *names):
        for n in names:
            open(os.path.join(self.dir, n), 'w').close()

    def test_numeric_ordering(self):
        self._touch('f_0.db3', 'f_10.db3', 'f_1.db3', 'f_11.db3', 'f_2.db3', 'f_9.db3')
        got = [os.path.basename(p) for p in self.ab.list_db_files(self.dir)]
        self.assertEqual(got, ['f_0.db3', 'f_1.db3', 'f_2.db3', 'f_9.db3', 'f_10.db3', 'f_11.db3'])

    def test_metadata_order(self):
        import yaml
        self._touch('a_0.db3', 'a_1.db3')
        yaml.dump({'rosbag2_bagfile_information': {'relative_file_paths': ['a_1.db3', 'a_0.db3']}},
                  _w(os.path.join(self.dir, 'metadata.yaml')))
        got = [os.path.basename(p) for p in self.ab.list_db_files(self.dir)]
        self.assertEqual(got, ['a_1.db3', 'a_0.db3'])

    def test_missing_file_skipped(self):
        import yaml
        self._touch('b_0.db3')
        yaml.dump({'rosbag2_bagfile_information': {'relative_file_paths': ['b_0.db3', 'b_1.db3']}},
                  _w(os.path.join(self.dir, 'metadata.yaml')))
        got = [os.path.basename(p) for p in self.ab.list_db_files(self.dir)]
        self.assertEqual(got, ['b_0.db3'])


class TestAudioSummary(unittest.TestCase):
    """ReSpeaker 6ch PCM → ch0 요약 통계"""

    def setUp(self):
        self.ab = prod('ab')

    def _row(self, samples):
        import numpy as np
        return self.ab.parse_msg('/respeaker/audio', NS(data=np.array(samples, dtype=np.int16).tobytes()))

    def test_rms_uses_ch0_only(self):
        ch = self.ab.MIC_CHANNELS
        pcm = [0] * (ch * 2)
        pcm[0], pcm[ch] = 3, 4            # ch0 만 값, 나머지 채널 0
        pcm[1] = 30000                    # ch1 값은 무시돼야 함
        self.assertAlmostEqual(self._row(pcm)['MICRaw_RMS'], math.sqrt((9 + 16) / 2), places=3)

    def test_clip_percent(self):
        ch = self.ab.MIC_CHANNELS
        pcm = [0] * (ch * 4)
        pcm[0], pcm[ch * 2] = 32700, 32500   # ch0 네 샘플 중 두 개 클리핑
        self.assertAlmostEqual(self._row(pcm)['MICRaw_ClipPct'], 50.0)

    def test_sample_count(self):
        ch = self.ab.MIC_CHANNELS
        self.assertEqual(self._row([1] * (ch * 5))['MICRaw_Samples'], 5)


class TestTrackingError(unittest.TestCase):

    def setUp(self):
        self.ab = prod('ab')

    def _wrap(self, v):
        import pandas as pd
        return float(self.ab._wrap180(pd.Series([v]))[0])

    def test_yaw_wrap(self):
        self.assertAlmostEqual(self._wrap(359.0 - 1.0), -2.0)
        self.assertAlmostEqual(self._wrap(-190.0), 170.0)
        self.assertAlmostEqual(self._wrap(180.0), -180.0)
        self.assertAlmostEqual(self._wrap(5.0), 5.0)

    def test_errors_and_magnitude(self):
        import pandas as pd
        df = pd.DataFrame({'ATT_DesRoll': [10.0], 'ATT_Roll': [7.0],
                           'ATT_DesPitch': [-5.0], 'ATT_Pitch': [-1.0],
                           'ATT_DesYaw': [359.0], 'ATT_Yaw': [1.0], 'Nav_Roll': [12.0]})
        r = self.ab.add_tracking_errors(df)
        self.assertAlmostEqual(r['Err_Roll'][0], 3.0)
        self.assertAlmostEqual(r['Err_Pitch'][0], -4.0)
        self.assertAlmostEqual(r['Err_Yaw'][0], -2.0)
        self.assertAlmostEqual(r['ErrNav_Roll'][0], 5.0)
        self.assertAlmostEqual(r['Err_AttMag'][0], 5.0)

    def test_no_target_no_error_column(self):
        import pandas as pd
        self.assertNotIn('Err_Roll', self.ab.add_tracking_errors(pd.DataFrame({'ATT_Roll': [1.0]})).columns)


class TestTypeMask(unittest.TestCase):
    """setpoint type_mask 의 비트가 set 이면 해당 필드는 무시(유효하지 않음)"""

    def setUp(self):
        self.ab = prod('ab')

    def _att(self, mask):
        msg = NS(orientation=NS(x=0, y=0, z=0, w=1), body_rate=V3(), type_mask=mask, thrust=0.5)
        return self.ab.parse_msg('/mavros/setpoint_raw/target_attitude', msg)

    def test_attitude_valid(self):
        self.assertEqual(self._att(0x00)['ATT_Des_AttValid'], 1)
        self.assertEqual(self._att(0x80)['ATT_Des_AttValid'], 0)

    def test_body_rate_valid(self):
        self.assertEqual(self._att(0x00)['ATT_Des_RateValid'], 1)
        self.assertEqual(self._att(0x07)['ATT_Des_RateValid'], 0)

    def test_local_position_velocity(self):
        def row(mask):
            return self.ab.parse_msg('/mavros/setpoint_raw/target_local',
                                     NS(position=V3(), velocity=V3(), type_mask=mask))
        self.assertEqual((row(0x00)['Des_PosValid'], row(0x00)['Des_VelValid']), (1, 1))
        self.assertEqual(row(0x07)['Des_PosValid'], 0)
        self.assertEqual(row(0x38)['Des_VelValid'], 0)

    def test_byte_type_mask(self):
        """rclpy 가 bytes 로 넘겨도 처리돼야 함"""
        self.assertEqual(self._att(b'\x80')['ATT_Des_AttValid'], 0)


class TestMissionParsing(unittest.TestCase):

    def setUp(self):
        self.ab = prod('ab')

    @staticmethod
    def _wp(cmd, lat=0.0, lon=0.0, alt=0.0):
        return NS(command=cmd, x_lat=lat, y_long=lon, z_alt=alt, frame=3,
                  param1=0.0, param2=0.0, param3=0.0, param4=0.0, autocontinue=True)

    def _row(self, wps, cur):
        return self.ab.parse_msg('/mavros/mission/waypoints', NS(waypoints=wps, current_seq=cur))

    def test_summary(self):
        wps = [self._wp(22), self._wp(16, 37.1, 127.1, 50.0), self._wp(16), self._wp(21)]
        r = self._row(wps, 1)
        self.assertEqual(r['Mission_Count'], 4)
        self.assertEqual(r['Mission_Cmds'], '16x2,21x1,22x1')
        self.assertAlmostEqual(r['Mission_TgtLat'], 37.1)
        self.assertEqual(r['Mission_TgtCmd'], 16)

    def test_current_out_of_range(self):
        r = self._row([self._wp(16)], 5)
        self.assertNotIn('Mission_TgtLat', r)

    def test_json_route(self):
        import json
        r = self._row([self._wp(22, alt=10.0)], 0)
        self.assertEqual(json.loads(r['Mission_Waypoints'])[0]['alt'], 10.0)

    def test_empty(self):
        self.assertEqual(self._row([], -1)['Mission_Count'], 0)


class TestStaleLimits(unittest.TestCase):
    """merged CSV 의 허용 age 는 컬럼 접두어로 결정"""

    def setUp(self):
        self.ab = prod('ab')

    def test_mission_is_latched(self):
        self.assertGreaterEqual(self.ab._stale_limit_for('Mission_Count'), 3600)

    def test_imu_is_tight(self):
        self.assertLessEqual(self.ab._stale_limit_for('IMU_AccX'), 1.0)

    def test_unknown_uses_default(self):
        self.assertEqual(self.ab._stale_limit_for('Unknown_Col'), self.ab.DEFAULT_STALE)


class TestSensorSignatures(unittest.TestCase):
    """자동 탐지 시그니처 — 서로 오탐이 없어야 함"""

    def setUp(self):
        self.sa = prod('sa')

    THL = b'@T453,1664,25.1,43.5,36.6\r\n@T453,1665,25.1,43.5,46.6\r\n'
    WCM = b'+01230\r\n~00450\r\n-01230\r\n'

    def test_thl100(self):
        self.assertEqual(self.sa._match_thl100(self.THL), 2)
        self.assertEqual(self.sa._match_thl100(b'@T453,100,,38.3,236.5\r\n'), 1)

    def test_wcm6800(self):
        self.assertEqual(self.sa._match_wcm6800(self.WCM), 3)
        self.assertEqual(self.sa._match_wcm6800(b'+1230\r\n'), 0)

    def test_no_cross_match(self):
        self.assertEqual(self.sa._match_thl100(self.WCM), 0)
        self.assertEqual(self.sa._match_wcm6800(self.THL), 0)
        self.assertEqual(self.sa._match_mavlink(self.THL * 20 + self.WCM * 20), 0)


class TestFcSysidDetection(unittest.TestCase):
    """
    FC 의 SYSID 를 HEARTBEAT 로 자동 탐지하는 로직 검증

    아래 바이트는 pymavlink 로 생성한 실제 MAVLink 프레임입니다.
    CRC 를 확인하므로 잡음이나 잘못된 baud 에서는 SYSID 가 잡히지 않고,
    같은 링크의 GCS·짐벌·컴패니언은 대상에서 제외돼야 합니다.
    """

    FC_SYS7      = bytes.fromhex('fd090000000701000000000000000203000403a76f')
    GCS_255      = bytes.fromhex('fd09000000ffbe0000000000000006080004033d48')
    GIMBAL_7     = bytes.fromhex('fd09000000079a000000000000001a08000403f777')
    COMPANION_7  = bytes.fromhex('fd0900000007bf000000000000001208000403437e')
    FC_SYS200_V1 = bytes.fromhex('fe0900c80100000000000e0300040312a8')

    @classmethod
    def setUpClass(cls):
        import importlib.util, os
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (os.path.join(here, '..', 'scripts', 'serial_autodetect.py'),
                     os.path.join(here, '..', 'serial_autodetect.py')):
            if os.path.exists(cand):
                spec = importlib.util.spec_from_file_location('sa', cand)
                cls.sa = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(cls.sa)
                return
        raise unittest.SkipTest('serial_autodetect.py 없음')

    def _target(self, data):
        return self.sa.summarize_link(self.sa.parse_mavlink(data))

    def test_sysid_7(self):
        t = self._target(self.FC_SYS7)['target']
        self.assertEqual((t['sysid'], t['compid']), (7, 1))

    def test_mavlink1_sysid_200(self):
        link = self._target(self.FC_SYS200_V1)
        self.assertEqual(link['target']['sysid'], 200)
        self.assertEqual(link['protocol'], 'v1.0')

    def test_excludes_gcs_gimbal_companion(self):
        data = self.GCS_255 + self.GIMBAL_7 + self.COMPANION_7 + self.FC_SYS7
        link = self._target(data)
        self.assertEqual(link['target']['sysid'], 7)
        self.assertEqual(link['target']['compid'], 1)
        self.assertEqual(len(link['others']), 3)

    def test_only_non_vehicles_gives_no_target(self):
        """GCS·짐벌만 보이면 대상이 없어야 함 (잘못 붙지 않도록)"""
        link = self._target(self.GCS_255 + self.GIMBAL_7)
        self.assertIsNone(link['target'])

    def test_companion_compid_avoids_collision(self):
        """기체 7 에 이미 컴패니언 191 이 있으면 mavros 는 194 를 써야 함"""
        link = self._target(self.COMPANION_7 + self.FC_SYS7)
        self.assertEqual(self.sa.choose_companion_compid(link, 7), 194)

    def test_companion_compid_default(self):
        link = self._target(self.FC_SYS7)
        self.assertEqual(self.sa.choose_companion_compid(link, 7), 191)

    def test_corrupted_crc_rejected(self):
        bad = bytearray(self.FC_SYS7)
        bad[-1] ^= 0xFF
        self.assertIsNone(self._target(bytes(bad))['target'])

    def test_noise_no_false_sysid(self):
        import random
        random.seed(7)
        noise = bytes(random.randrange(256) for _ in range(20000))
        # 과거 로그에 찍혔던 형태의 가짜 헤더(191.239)
        noise += bytes([0xFD, 9, 0, 0, 0xAF, 191, 239, 0, 0, 0]) * 20
        self.assertIsNone(self._target(noise)['target'])

    def test_frame_inside_noise(self):
        import random
        random.seed(8)
        noise = bytes(random.randrange(256) for _ in range(2000))
        t = self._target(noise + self.FC_SYS7 + noise)['target']
        self.assertEqual(t['sysid'], 7)

    def test_sensor_data_not_mavlink(self):
        self.assertEqual(self.sa._match_mavlink(b'@T453,1,25.1,43.5,36.6\r\n' * 50), 0)
        self.assertEqual(self.sa._match_mavlink(b'+01230\r\n' * 100), 0)


class TestPortClaim(unittest.TestCase):
    """
    FC 포트 선점 표시

    mavros 가 포트를 열기 직전의 틈에 THL100 노드 등이 먼저 열면
    MAVLink 데이터를 나눠 가져 첫 연결이 실패합니다.
    관리 노드가 표시한 포트는 다른 노드가 피해야 하고,
    표시한 프로세스가 죽으면 표시는 무시돼야 합니다.
    """

    @classmethod
    def setUpClass(cls):
        import importlib.util, os, tempfile
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (os.path.join(here, '..', 'scripts', 'serial_autodetect.py'),
                     os.path.join(here, '..', 'serial_autodetect.py')):
            if os.path.exists(cand):
                spec = importlib.util.spec_from_file_location('sa_claim', cand)
                cls.sa = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(cls.sa)
                break
        else:
            raise unittest.SkipTest('serial_autodetect.py 없음')
        cls.sa.CLAIM_DIR = tempfile.mkdtemp()
        cls.dev = os.path.join(tempfile.mkdtemp(), 'ttyUSB9')
        open(cls.dev, 'w').close()

    def tearDown(self):
        self.sa.release_port('fc')

    def test_claim_visible_to_others(self):
        import os
        self.sa.claim_port('fc', self.dev)
        self.assertIn(os.path.realpath(self.dev), self.sa.claimed_ports(exclude_key='thl100'))

    def test_own_claim_excluded(self):
        import os
        self.sa.claim_port('fc', self.dev)
        self.assertNotIn(os.path.realpath(self.dev), self.sa.claimed_ports(exclude_key='fc'))

    def test_release(self):
        import os
        self.sa.claim_port('fc', self.dev)
        self.sa.release_port('fc')
        self.assertNotIn(os.path.realpath(self.dev), self.sa.claimed_ports())

    def test_dead_owner_ignored(self):
        import os, subprocess
        p = subprocess.Popen(['true']); p.wait()
        with open(os.path.join(self.sa.CLAIM_DIR, 'fc'), 'w') as f:
            f.write(f'{p.pid} {os.path.realpath(self.dev)}\n')
        self.assertNotIn(os.path.realpath(self.dev), self.sa.claimed_ports())


class TestPackagingConsistency(unittest.TestCase):
    """
    패키지 안의 사본이 원본(scripts/)과 같은지

    노드는 설치된 사본을 import 합니다. 원본만 고치고 fix_packaging.sh 를
    다시 돌리지 않은 채 커밋하면 실행 시 옛 코드가 쓰이므로 여기서 잡습니다.
    """

    def test_packaged_copies_match(self):
        import filecmp
        checked = 0
        for mod in ('serial_autodetect.py', 'verify_bag.py'):
            src = os.path.join(ROOT, 'scripts', mod)
            pkg = os.path.join(ROOT, 'src', 'drone_sensors', 'scripts', mod)
            if os.path.exists(src) and os.path.exists(pkg):
                self.assertTrue(filecmp.cmp(src, pkg, shallow=False),
                                f'{mod} 사본이 원본과 다름 — bash fix_packaging.sh 후 커밋하세요')
                checked += 1
        if not checked:
            self.skipTest('저장소 구조가 아님')

    def test_verify_bag_importable_from_thread(self):
        """녹화 노드가 착륙 후 별도 스레드에서 verify_bag 을 불러올 수 있어야 함"""
        import threading
        err = []

        def load():
            try:
                _load('vb_thread', 'scripts/verify_bag.py')
            except unittest.SkipTest:
                pass
            except Exception as e:
                err.append(e)
        th = threading.Thread(target=load)
        th.start(); th.join()
        self.assertEqual(err, [], f'스레드에서 import 실패: {err}')


class TestVerifyBag(unittest.TestCase):
    """녹화 검증: 드론 판정과 외부센서 표기 규칙"""

    @classmethod
    def setUpClass(cls):
        import importlib.util, os
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (os.path.join(here, '..', 'scripts', 'verify_bag.py'),
                     os.path.join(here, '..', 'verify_bag.py')):
            if os.path.exists(cand):
                spec = importlib.util.spec_from_file_location('vb', cand)
                cls.vb = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(cls.vb)
                return
        raise unittest.SkipTest('verify_bag.py 없음')

    T0 = 1_000_000_000_000

    def _stamps(self, hz, dur, holes=()):
        out, t = [], 0.0
        while t < dur:
            if not any(a <= t < b for a, b in holes):
                out.append(int(self.T0 + t * 1e9))
            t += 1.0 / hz
        return out

    def _stats(self, hz, dur=60, holes=(), gap=1.0, exp=None):
        t1 = int(self.T0 + dur * 1e9)
        return self.vb.topic_stats(self._stamps(hz, dur, holes), self.T0, t1, exp or hz, gap)

    def test_normal_rate(self):
        s = self._stats(50)
        self.assertTrue(s['rate_ok'])
        self.assertEqual(s['gaps'], [])

    def test_gap_detected(self):
        s = self._stats(50, holes=[(20, 23)])
        self.assertEqual(len(s['gaps']), 1)
        self.assertAlmostEqual(s['max_gap'], 3.0, delta=0.1)

    def test_late_start_counts_as_gap(self):
        """센서가 녹화 시작 10초 뒤에 붙으면 공백으로 잡혀야 함"""
        s = self._stats(1, holes=[(0, 10)], gap=5.0)
        self.assertTrue(s['gaps'] and s['gaps'][0][0] == 0.0)

    def test_low_rate(self):
        self.assertFalse(self._stats(20, exp=50)['rate_ok'])

    def _drone(self, **override):
        d = {t: self._stats(c['hz'], gap=c['gap']) for t, c in self.vb.DRONE_TOPICS.items()}
        d.update(override)
        return d

    def test_drone_normal(self):
        self.assertEqual(self.vb.judge_drone(self._drone())[0], '정상')

    def test_drone_missing_is_bad(self):
        empty = self.vb.topic_stats([], self.T0, int(self.T0 + 60e9), 50, 1.0)
        self.assertEqual(self.vb.judge_drone(self._drone(**{'/mavros/imu/data': empty}))[0], '불량')

    def test_drone_long_gap_is_bad(self):
        g = self._stats(50, holes=[(10, 20)])
        self.assertEqual(self.vb.judge_drone(self._drone(**{'/mavros/imu/data': g}))[0], '불량')

    def test_drone_short_gap_is_warn(self):
        g = self._stats(50, holes=[(10, 12)])
        self.assertEqual(self.vb.judge_drone(self._drone(**{'/mavros/imu/data': g}))[0], '주의')

    def test_result_reuse(self):
        """같은 bag·같은 기준이면 재사용, bag 이 바뀌거나 --recheck 면 다시 검증"""
        d = tempfile.mkdtemp()
        bag = os.path.join(d, 'flight_x'); os.makedirs(bag)
        with open(os.path.join(bag, 'flight_x_0.db3'), 'w') as f:
            f.write('a')
        calls = []
        orig = self.vb.verify
        self.vb.verify = lambda b, with_arm=True: calls.append(b) or {
            'name': 'flight_x', 'duration': 1.0, 'drone_level': '정상',
            'drone_reasons': [], 'sensor_state': {}, 'unclosed': False}
        try:
            cache = {}
            self.assertFalse(self.vb.verify_summary(bag, cache)[1])      # 처음: 검증
            self.assertTrue(self.vb.verify_summary(bag, cache)[1])       # 두 번째: 재사용
            self.assertFalse(self.vb.verify_summary(bag, cache, recheck=True)[1])
            with open(os.path.join(bag, 'flight_x_0.db3'), 'a') as f:  # bag 변경
                f.write('bbbb')
            self.assertFalse(self.vb.verify_summary(bag, cache)[1])
            cache['flight_x']['criteria'] = 'old'                          # 기준 변경
            self.assertFalse(self.vb.verify_summary(bag, cache)[1])
            self.assertEqual(len(calls), 4)
        finally:
            self.vb.verify = orig

    def test_sensor_states(self):
        empty = self.vb.topic_stats([], self.T0, int(self.T0 + 60e9), 1, 5.0)
        self.assertEqual(self.vb.judge_sensor(empty), '없음')
        self.assertEqual(self.vb.judge_sensor(self._stats(1, holes=[(20, 40)], gap=5.0)), '부분')
        self.assertEqual(self.vb.judge_sensor(self._stats(1, gap=5.0)), '정상')


if __name__ == '__main__':
    unittest.main(verbosity=2)
