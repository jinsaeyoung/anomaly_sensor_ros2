#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Winson WCM6800 전류계 UART ROS2 노드
패킷 형식 (8바이트 고정): [type][d1~d5]\r\n

발행 토픽:
  /wcm6800/data  - 전류 묶음 (Header 포함, publish_rate_hz 주기)
  /wcm6800/raw   - 원시 패킷 (수신 즉시 발행)
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

        # 파라미터
        self.declare_parameter('port',              '/dev/ttyUSB0')
        self.declare_parameter('baudrate',          9600)
        self.declare_parameter('publish_rate_hz',   10.0)
        self.declare_parameter('stale_timeout_sec', 1.0)

        port             = self.get_parameter('port').value
        baudrate         = self.get_parameter('baudrate').value
        publish_rate_hz  = self.get_parameter('publish_rate_hz').value
        self.stale_sec   = self.get_parameter('stale_timeout_sec').value

        # 퍼블리셔
        self.pub_data         = self.create_publisher(String,  '/wcm6800/data',         10)
        self.pub_raw          = self.create_publisher(String,  '/wcm6800/raw',           10)
        self.pub_current      = self.create_publisher(Float32, '/wcm6800/current',       10)
        self.pub_current_type = self.create_publisher(String,  '/wcm6800/current_type',  10)

        # 최신 데이터 저장소
        self._latest = None
        self._latest_time = None
        self._lock = threading.Lock()

        # 시리얼 포트
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2.0
            )
            self.get_logger().info(f'시리얼 포트 연결 성공: {port} @ {baudrate}bps')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 포트 열기 실패: {e}')
            raise

        # UART 수신 스레드
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        # 발행 타이머
        self.create_timer(1.0 / publish_rate_hz, self._publish_timer)

        self.get_logger().info(
            f'WCM6800Node 시작\n'
            f'  /wcm6800/data         — {publish_rate_hz}Hz (묶음, Header 포함)\n'
            f'  /wcm6800/raw          — 수신 즉시\n'
            f'  /wcm6800/current      — {publish_rate_hz}Hz\n'
            f'  /wcm6800/current_type — {publish_rate_hz}Hz'
        )

    def _parse(self, line):
        """8바이트 패킷 파싱 → dict 반환"""
        line = line.strip()
        if len(line) != 6:
            return None

        type_char = line[0]
        digits    = line[1:]

        if not digits.isdigit():
            return None

        value = int(digits) / 1000.0

        if type_char == '~':
            return {'current': value,  'current_type': 'AC',  'valid': True}
        elif type_char == '+':
            return {'current': value,  'current_type': 'DC+', 'valid': True}
        elif type_char == '-':
            return {'current': -value, 'current_type': 'DC-', 'valid': True}
        return None

    def _read_loop(self):
        """UART 수신 전용 스레드"""
        while not self._stop_event.is_set():
            try:
                raw_line = self.ser.readline().decode('ascii', errors='ignore')

                if not raw_line.strip():
                    continue

                # raw 즉시 발행
                raw_msg = String()
                raw_msg.data = raw_line.strip()
                self.pub_raw.publish(raw_msg)

                # 파싱 후 최신값 저장
                parsed = self._parse(raw_line)
                if parsed:
                    with self._lock:
                        self._latest = parsed
                        self._latest_time = time.time()

            except serial.SerialException as e:
                if not self._stop_event.is_set():
                    self.get_logger().error(f'시리얼 읽기 오류: {e}')
                break
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f'수신 오류: {e}')

    def _publish_timer(self):
        """publish_rate_hz 주기로 최신값 발행"""
        with self._lock:
            data = self._latest
            recv_time = self._latest_time

        if data is None:
            return

        if time.time() - recv_time > self.stale_sec:
            self.get_logger().warn(
                f'WCM6800 데이터가 {self.stale_sec}초 이상 갱신되지 않았습니다.'
            )
            return

        now = self.get_clock().now()

        # /wcm6800/data — JSON 묶음 발행
        payload = {
            'stamp_sec':    now.nanoseconds // 1_000_000_000,
            'stamp_nsec':   now.nanoseconds %  1_000_000_000,
            'current':      data['current'],
            'current_type': data['current_type'],
            'valid':        data['valid'],
        }
        data_msg = String()
        data_msg.data = json.dumps(payload)
        self.pub_data.publish(data_msg)

        # 개별 토픽 병행 발행
        m = Float32(); m.data = float(data['current'])
        self.pub_current.publish(m)

        t = String(); t.data = data['current_type']
        self.pub_current_type.publish(t)

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
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
