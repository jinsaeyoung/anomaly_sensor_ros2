#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Winson WCM6800 전류계 UART ROS2 노드
패킷 형식 (8바이트 고정): [type][d1~d5]\r\n
  type: '~'=AC, '+'=DC+, '-'=DC-

발행 토픽:
  /wcm6800/data          - 전류 묶음 (JSON, 센서 수신 시각 포함)
  /wcm6800/raw           - 원시 패킷 (수신 즉시 발행)
  /wcm6800/current       - Float32 (하위 호환)
  /wcm6800/current_type  - String  (하위 호환)

특징:
  - UART 수신 스레드와 발행 타이머 분리
  - USB 분리 시 자동 재연결
  - 스트림 버퍼 기반 패킷 파서
  - 센서 수신 시각(ROS clock) 기록
"""

import threading
import time
import json
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class WCM6800Node(Node):

    def __init__(self):
        super().__init__('wcm6800_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('port',                '/dev/ttyUSB0')
        self.declare_parameter('baudrate',            9600)
        self.declare_parameter('publish_rate_hz',     10.0)
        self.declare_parameter('stale_timeout_sec',   2.0)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('drain_max_sec',       3.0)

        self.port            = self.get_parameter('port').value
        self.baudrate        = self.get_parameter('baudrate').value
        publish_rate_hz      = self.get_parameter('publish_rate_hz').value
        self.stale_sec       = self.get_parameter('stale_timeout_sec').value
        self.reconnect_delay = self.get_parameter('reconnect_delay_sec').value
        self.drain_max_sec   = self.get_parameter('drain_max_sec').value

        # ── 퍼블리셔 ──────────────────────────────────────────────────
        self.pub_data         = self.create_publisher(String,  '/wcm6800/data',         10)
        self.pub_raw          = self.create_publisher(String,  '/wcm6800/raw',          10)
        self.pub_current      = self.create_publisher(Float32, '/wcm6800/current',      10)
        self.pub_current_type = self.create_publisher(String,  '/wcm6800/current_type', 10)

        # ── 상태 ──────────────────────────────────────────────────────
        self._latest         = None
        self._latest_rx_ns   = None   # ROS clock (ns) — 타임스탬프 기록용
        self._latest_rx_mono = None   # monotonic (s) — 경과시간 측정용
        self._lock         = threading.Lock()
        self._buffer       = ''

        self._stat            = {'rx': 0, 'parse_ok': 0, 'parse_fail': 0, 'reconnect': 0}
        self._stat_prev       = dict(self._stat)
        self._diag_period_sec = 30.0

        self.ser         = None
        self._stop_event = threading.Event()

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / publish_rate_hz, self._publish_timer)
        self.create_timer(30.0, self._diag_timer)

        self.get_logger().info(
            f'WCM6800Node 시작 (port={self.port}, {publish_rate_hz}Hz)\n'
            f'  /wcm6800/data          — {publish_rate_hz}Hz (JSON, 수신 시각 포함)\n'
            f'  /wcm6800/raw           — 수신 즉시\n'
            f'  /wcm6800/current, /current_type — {publish_rate_hz}Hz'
        )

    def _connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2
            )
            # ── 연결 직후 누적된 오래된 데이터 폐기 ──────────────────
            # PC 부팅 후 아무도 읽지 않은 동안 tty/드라이버 버퍼에 쌓인
            # 과거 패킷이 한꺼번에 유입되는 것을 방지합니다.
            # 종료 조건:
            #   - idle_gap_sec 동안 새 데이터가 없으면 = 밀린 것 모두 비움 → 종료
            #   - 또는 drain_max_sec 초과 시 무조건 종료 (무한 대기 방지)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            dropped   = 0
            idle_gap  = 0.15                                   # 이만큼 조용하면 완료로 판단
            deadline  = time.monotonic() + self.drain_max_sec  # 상한
            idle_from = None

            while time.monotonic() < deadline:
                n = self.ser.in_waiting
                if n:
                    dropped += len(self.ser.read(n))
                    idle_from = None
                else:
                    now = time.monotonic()
                    if idle_from is None:
                        idle_from = now
                    elif now - idle_from >= idle_gap:
                        break
                    time.sleep(0.01)

            self.ser.reset_input_buffer()
            self._buffer = ''

            self.get_logger().info(
                f'시리얼 포트 연결: {self.port} @ {self.baudrate}bps '
                f'(초기 버퍼 {dropped} bytes 폐기)'
            )
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f'시리얼 연결 실패: {e}')
            self.ser = None
            return False

    def _extract_packets(self, chunk):
        """
        스트림 버퍼에서 완전한 패킷 추출
        패킷: [type][5자리숫자]\r\n  (총 6문자 + 개행)
        """
        self._buffer += chunk
        packets = []

        while True:
            end = self._buffer.find('\n')
            if end < 0:
                if len(self._buffer) > 256:
                    self._buffer = ''
                break
            line = self._buffer[:end].strip()
            self._buffer = self._buffer[end + 1:]
            if line:
                packets.append(line)

        return packets

    def _parse(self, packet):
        """단일 패킷 파싱 → dict, 실패 시 None"""
        if len(packet) != 6:
            return None

        type_char = packet[0]
        digits    = packet[1:]

        if not digits.isdigit():
            return None

        value = int(digits) / 1000.0

        if type_char == '~':
            return {'current': value,  'current_type': 'AC',  'valid': True}
        if type_char == '+':
            return {'current': value,  'current_type': 'DC+', 'valid': True}
        if type_char == '-':
            return {'current': -value, 'current_type': 'DC-', 'valid': True}
        return None

    def _read_loop(self):
        while not self._stop_event.is_set():
            if self.ser is None or not self.ser.is_open:
                if not self._connect():
                    self._stop_event.wait(self.reconnect_delay)
                    continue
                else:
                    self._stat['reconnect'] += 1
                    self._buffer = ''

            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue

                text = chunk.decode('ascii', errors='ignore')
                for pkt in self._extract_packets(text):
                    self._stat['rx'] += 1

                    raw_msg = String(); raw_msg.data = pkt
                    self.pub_raw.publish(raw_msg)

                    parsed = self._parse(pkt)
                    if parsed is None:
                        self._stat['parse_fail'] += 1
                        continue
                    self._stat['parse_ok'] += 1

                    now_ns   = self.get_clock().now().nanoseconds
                    now_mono = time.monotonic()
                    with self._lock:
                        self._latest         = parsed
                        self._latest_rx_ns   = now_ns
                        self._latest_rx_mono = now_mono

            except (serial.SerialException, OSError) as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f'시리얼 오류 — 재연결 시도: {e}')
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self._stop_event.wait(self.reconnect_delay)
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f'수신 처리 오류: {e}')

    def _publish_timer(self):
        with self._lock:
            data    = self._latest
            rx_ns   = self._latest_rx_ns
            rx_mono = self._latest_rx_mono

        if data is None:
            return

        # 경과시간은 monotonic 기준
        age_sec = time.monotonic() - rx_mono

        if age_sec > self.stale_sec:
            self.get_logger().warn(
                f'WCM6800 데이터 stale ({age_sec:.1f}s > {self.stale_sec}s) — 발행 보류',
                throttle_duration_sec=10.0
            )
            return

        payload = {
            'stamp_sec':    rx_ns // 1_000_000_000,
            'stamp_nsec':   rx_ns %  1_000_000_000,
            'age_ms':       round(age_sec * 1000, 1),
            'current':      data['current'],
            'current_type': data['current_type'],
            'valid':        data['valid'],
        }
        m = String(); m.data = json.dumps(payload)
        self.pub_data.publish(m)

        f = Float32(); f.data = float(data['current'])
        self.pub_current.publish(f)

        t = String(); t.data = data['current_type']
        self.pub_current_type.publish(t)

    def _diag_timer(self):
        s = self._stat
        p = self._stat_prev
        period = self._diag_period_sec

        d_rx   = s['rx']         - p['rx']
        d_ok   = s['parse_ok']   - p['parse_ok']
        d_fail = s['parse_fail'] - p['parse_fail']
        self._stat_prev = dict(s)

        if d_rx == 0:
            self.get_logger().warn(f'WCM6800: 최근 {period:.0f}초간 수신 패킷 없음')
            return

        hz = d_rx / period
        self.get_logger().info(
            f"WCM6800 진단 [{period:.0f}s] rx={d_rx} ({hz:.2f}Hz) "
            f"ok={d_ok} fail={d_fail} "
            f"| 누적 rx={s['rx']} reconnect={s['reconnect']}"
        )

    def destroy_node(self):
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WCM6800Node()
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
