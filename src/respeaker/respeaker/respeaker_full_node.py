#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ReSpeaker Mic Array v3.0 (XVF3000) - ROS2 Full Node

퍼블리시 토픽:
  /respeaker/doa    - 소리 방향 각도 (Int32, 0~359도)
  /respeaker/vad    - 음성 감지 여부 (Bool)
  /respeaker/audio  - 마이크 raw 오디오 (UInt8MultiArray, PCM 16bit)
  /respeaker/energy - 음성 에너지 레벨 (Float32, RMS)

설계:
  장치를 찾지 못해도 노드는 종료하지 않고 재연결을 시도합니다.
  (이전에는 __init__ 에서 예외를 던져 프로세스가 죽었고,
   그 결과 rosbag 이 토픽을 구독조차 못 해 데이터가 통째로 누락됐습니다)

  퍼블리셔는 장치 상태와 무관하게 먼저 생성하므로,
  장치가 늦게 연결되어도 같은 토픽으로 데이터가 흐릅니다.
"""

import threading
import time

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32, UInt8MultiArray

from respeaker.firmware.tuning import find as tuning_find


class RespeakerFullNode(Node):
    RATE   = 16000
    CHUNK  = 1024
    FORMAT = pyaudio.paInt16

    def __init__(self):
        super().__init__('respeaker_full_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('update_rate',         50.0)
        self.declare_parameter('device_name',         'ReSpeaker')
        self.declare_parameter('reconnect_delay_sec', 3.0)
        self.declare_parameter('max_read_fail',       20)

        self.update_rate     = self.get_parameter('update_rate').value
        self.device_name     = self.get_parameter('device_name').value
        self.reconnect_delay = self.get_parameter('reconnect_delay_sec').value
        self.max_read_fail   = self.get_parameter('max_read_fail').value

        # ── 퍼블리셔 (장치 상태와 무관하게 먼저 생성) ─────────────────
        # 이렇게 해야 장치가 없어도 토픽이 존재하고,
        # rosbag 이 구독할 수 있으며, 나중에 연결되면 바로 데이터가 흐릅니다.
        self.pub_doa    = self.create_publisher(Int32,           '/respeaker/doa',    10)
        self.pub_vad    = self.create_publisher(Bool,            '/respeaker/vad',    10)
        self.pub_audio  = self.create_publisher(UInt8MultiArray, '/respeaker/audio',  10)
        self.pub_energy = self.create_publisher(Float32,         '/respeaker/energy', 10)

        # ── 상태 ──────────────────────────────────────────────────────
        self.tuning    = None
        self.pa        = None
        self.stream    = None
        self._channels = 1
        self._connected = False
        self._lock = threading.Lock()

        self._stat = {'usb_ok': 0, 'usb_fail': 0,
                      'audio_ok': 0, 'audio_fail': 0, 'reconnect': 0}
        self._stat_prev = dict(self._stat)
        self._diag_period_sec = 30.0
        self._read_fail = 0

        # ── 연결 스레드 (블로킹 작업을 타이머와 분리) ─────────────────
        # PyAudio 열기·USB 탐색은 수 초가 걸릴 수 있어
        # 타이머 콜백에서 하면 다른 콜백이 밀립니다.
        self._stop_event = threading.Event()
        self._conn_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._conn_thread.start()

        # ── 타이머 ────────────────────────────────────────────────────
        self.create_timer(1.0 / self.update_rate,     self._usb_callback)
        self.create_timer(self.CHUNK / self.RATE,     self._audio_callback)
        self.create_timer(self._diag_period_sec,      self._diag_timer)

        self.get_logger().info(
            'RespeakerFullNode 시작 (장치 연결 대기 중)\n'
            '  /respeaker/doa    (Int32)           소리 방향 0~359도\n'
            '  /respeaker/vad    (Bool)             음성 감지 여부\n'
            '  /respeaker/audio  (UInt8MultiArray)  PCM 16bit raw\n'
            '  /respeaker/energy (Float32)          RMS 에너지'
        )

    # ══════════════════════════════════════════════════════════════════
    # 연결 관리
    # ══════════════════════════════════════════════════════════════════
    def _connect_loop(self):
        """장치가 연결될 때까지, 그리고 끊길 때마다 재연결을 시도"""
        first = True
        while not self._stop_event.is_set():
            if self._connected:
                self._stop_event.wait(1.0)
                continue

            if self._try_connect(quiet=not first):
                first = True
            else:
                first = False
                self._stop_event.wait(self.reconnect_delay)

    def _try_connect(self, quiet=False):
        """
        USB(tuning) + 오디오 스트림을 연결

        quiet=True 면 실패 로그를 억제합니다.
        장치가 없는 동안 같은 경고가 수백 줄 쌓이는 것을 막기 위함입니다.
        """
        # 1) USB 인터페이스
        try:
            tuning = tuning_find()
        except Exception as e:
            if not quiet:
                self.get_logger().warn(f'USB 탐색 오류: {e}')
            return False

        if tuning is None:
            if not quiet:
                self.get_logger().warn(
                    'ReSpeaker를 찾을 수 없습니다 — USB 연결을 확인하세요. '
                    f'{self.reconnect_delay:.0f}초마다 재시도합니다.'
                )
            return False

        # 2) 오디오 장치
        pa = None
        stream = None
        try:
            pa = pyaudio.PyAudio()
            idx, ch = self._find_device(pa, quiet=quiet)
            if idx is None:
                raise RuntimeError('오디오 입력 장치를 찾지 못함')

            stream = pa.open(
                rate=self.RATE, channels=ch, format=self.FORMAT,
                input=True, input_device_index=idx,
                frames_per_buffer=self.CHUNK,
            )
        except Exception as e:
            if not quiet:
                self.get_logger().warn(f'오디오 스트림 열기 실패: {e}')
            self._safe_close(tuning, stream, pa)
            return False

        # 3) 연결 확정
        with self._lock:
            self.tuning    = tuning
            self.pa        = pa
            self.stream    = stream
            self._channels = ch
            self._connected = True
            self._read_fail = 0

        self._stat['reconnect'] += 1
        self.get_logger().info(
            f'ReSpeaker 연결 성공 (ch={ch}, {self.RATE}Hz) '
            f'[{self._stat["reconnect"]}회째]'
        )
        return True

    def _find_device(self, pa, quiet=False):
        for i in range(pa.get_device_count()):
            try:
                info = pa.get_device_info_by_index(i)
            except Exception:
                continue
            if self.device_name.lower() in str(info.get('name', '')).lower() \
               and info.get('maxInputChannels', 0) > 0:
                ch = min(int(info['maxInputChannels']), 6)
                self.get_logger().info(
                    f'오디오 장치 발견: [{i}] {info["name"]} (ch={ch})'
                )
                return i, ch

        if not quiet:
            self.get_logger().warn(
                f'"{self.device_name}" 오디오 장치를 찾지 못했습니다.'
            )
        return None, 1

    def _disconnect(self, reason=''):
        """연결을 해제하고 재연결 대상으로 되돌립니다"""
        with self._lock:
            if not self._connected:
                return
            self._connected = False
            tuning, stream, pa = self.tuning, self.stream, self.pa
            self.tuning = self.stream = self.pa = None

        self._safe_close(tuning, stream, pa)
        self.get_logger().warn(f'ReSpeaker 연결 해제 ({reason}) — 재연결 시도')

    @staticmethod
    def _safe_close(tuning, stream, pa):
        for obj, fn in ((tuning, 'close'),):
            if obj is not None:
                try:
                    getattr(obj, fn)()
                except Exception:
                    pass
        if stream is not None:
            try:
                stream.stop_stream()
                stream.close()
            except Exception:
                pass
        if pa is not None:
            try:
                pa.terminate()
            except Exception:
                pass

    # ══════════════════════════════════════════════════════════════════
    # 데이터 수집
    # ══════════════════════════════════════════════════════════════════
    def _usb_callback(self):
        if not self._connected:
            return

        with self._lock:
            tuning = self.tuning
        if tuning is None:
            return

        try:
            m = Int32(); m.data = int(tuning.direction)
            self.pub_doa.publish(m)

            b = Bool(); b.data = bool(tuning.is_voice())
            self.pub_vad.publish(b)

            self._stat['usb_ok'] += 1
            self._read_fail = 0

        except Exception as e:
            self._stat['usb_fail'] += 1
            self._read_fail += 1
            self.get_logger().warn(
                f'USB 읽기 실패: {e}', throttle_duration_sec=5.0
            )
            # 연속 실패가 누적되면 장치가 빠진 것으로 판단
            if self._read_fail >= self.max_read_fail:
                self._disconnect('USB 읽기 연속 실패')

    def _audio_callback(self):
        if not self._connected:
            return

        with self._lock:
            stream = self.stream
            ch = self._channels
        if stream is None:
            return

        try:
            if stream.get_read_available() < self.CHUNK:
                return

            raw = stream.read(self.CHUNK, exception_on_overflow=False)

            m = UInt8MultiArray(); m.data = list(raw)
            self.pub_audio.publish(m)

            samples = np.frombuffer(raw, dtype=np.int16)
            if ch > 1:
                samples = samples[::ch]
            rms = float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))
            e = Float32(); e.data = rms
            self.pub_energy.publish(e)

            self._stat['audio_ok'] += 1
            self._read_fail = 0

        except Exception as e:
            self._stat['audio_fail'] += 1
            self._read_fail += 1
            self.get_logger().warn(
                f'오디오 읽기 실패: {e}', throttle_duration_sec=5.0
            )
            if self._read_fail >= self.max_read_fail:
                self._disconnect('오디오 읽기 연속 실패')

    # ══════════════════════════════════════════════════════════════════
    # 진단
    # ══════════════════════════════════════════════════════════════════
    def _diag_timer(self):
        s, p = self._stat, self._stat_prev
        period = self._diag_period_sec

        d_usb   = s['usb_ok']     - p['usb_ok']
        d_audio = s['audio_ok']   - p['audio_ok']
        d_uf    = s['usb_fail']   - p['usb_fail']
        d_af    = s['audio_fail'] - p['audio_fail']
        self._stat_prev = dict(s)

        if not self._connected:
            self.get_logger().warn(
                f'ReSpeaker 미연결 — 장치 대기 중 '
                f'(재연결 {self.reconnect_delay:.0f}초 주기)'
            )
            return

        usb_hz   = d_usb / period
        audio_hz = d_audio / period
        self.get_logger().info(
            f'ReSpeaker 진단 [{period:.0f}s] '
            f'doa/vad={d_usb} ({usb_hz:.1f}Hz) '
            f'audio={d_audio} ({audio_hz:.1f}Hz) '
            f'fail(usb={d_uf}, audio={d_af}) '
            f'| 누적 reconnect={s["reconnect"]}'
        )

    def destroy_node(self):
        self._stop_event.set()
        if self._conn_thread.is_alive():
            self._conn_thread.join(timeout=2.0)
        with self._lock:
            tuning, stream, pa = self.tuning, self.stream, self.pa
            self.tuning = self.stream = self.pa = None
            self._connected = False
        self._safe_close(tuning, stream, pa)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RespeakerFullNode()
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
