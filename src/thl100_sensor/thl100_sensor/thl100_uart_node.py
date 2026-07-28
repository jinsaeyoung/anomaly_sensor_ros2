#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OSTSen-THL100 UART ROS2 노드
패킷 형식: @sensorID,Sequence,Temperature,Humidity,Light\r\n

발행 토픽:
  /thl100/data  - 온도/습도/조도 묶음 (Header 포함, publish_rate_hz 주기)
  /thl100/raw   - 원시 패킷 문자열 (수신 즉시 발행)
"""

import threading
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Temperature
from builtin_interfaces.msg import Time

# 커스텀 메시지 대신 std_msgs 조합으로 구현
# /thl100/data 는 String JSON 형태로 발행 (Header timestamp 포함)
import json


class THL100Node(Node):

    def __init__(self):
        super().__init__('thl100_node')

        # 파라미터
        self.declare_parameter('port',              '/dev/ttyUSB0')
        self.declare_parameter('baudrate',          9600)
        self.declare_parameter('publish_rate_hz',   1.0)
        self.declare_parameter('stale_timeout_sec', 3.0)

        port             = self.get_parameter('port').value
        baudrate         = self.get_parameter('baudrate').value
        publish_rate_hz  = self.get_parameter('publish_rate_hz').value
        self.stale_sec   = self.get_parameter('stale_timeout_sec').value

        # 퍼블리셔
        self.pub_data = self.create_publisher(String, '/thl100/data', 10)
        self.pub_raw  = self.create_publisher(String, '/thl100/raw',  10)

        # 개별 Float32 토픽도 병행 발행 (하위 호환)
        from std_msgs.msg import Float32
        self.pub_temp  = self.create_publisher(Float32, '/thl100/temperature', 10)
        self.pub_humi  = self.create_publisher(Float32, '/thl100/humidity',    10)
        self.pub_light = self.create_publisher(Float32, '/thl100/light',       10)

        # 최신 데이터 저장소
        self._latest = None          # dict: temperature, humidity, light, sensor_id, sequence
        self._latest_time = None     # 마지막 수신 시각
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

        # UART 수신 스레드 (계속 수신)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        # 발행 타이머 (publish_rate_hz 주기)
        self.create_timer(1.0 / publish_rate_hz, self._publish_timer)

        self.get_logger().info(
            f'THL100Node 시작\n'
            f'  /thl100/data        — {publish_rate_hz}Hz (묶음, Header 포함)\n'
            f'  /thl100/raw         — 수신 즉시\n'
            f'  /thl100/temperature — {publish_rate_hz}Hz\n'
            f'  /thl100/humidity    — {publish_rate_hz}Hz\n'
            f'  /thl100/light       — {publish_rate_hz}Hz'
        )

    def _parse(self, line):
        """패킷 파싱 → dict 반환, 실패 시 None"""
        # 여러 패킷이 붙어서 올 경우 마지막 완전한 패킷만 사용
        parts = line.split('@')
        if len(parts) < 2:
            return None
        line = '@' + parts[-1].strip()

        if not line.startswith('@'):
            return None

        fields = line[1:].split(',')
        if len(fields) != 5:
            return None

        try:
            return {
                'sensor_id':   fields[0],
                'sequence':    int(fields[1]),
                'temperature': float(fields[2]) if fields[2] else None,
                'humidity':    float(fields[3]) if fields[3] else None,
                'light':       float(fields[4]) if fields[4] else None,
                'valid':       True,
            }
        except ValueError:
            return None

    def _read_loop(self):
        """UART 수신 전용 스레드 — 수신 즉시 raw 발행, 최신값 저장"""
        while not self._stop_event.is_set():
            try:
                raw_line = self.ser.readline().decode('ascii', errors='ignore')

                if not raw_line.strip():
                    continue

                # raw 토픽: 수신 즉시 발행
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

        # stale 체크
        if time.time() - recv_time > self.stale_sec:
            self.get_logger().warn(
                f'THL100 데이터가 {self.stale_sec}초 이상 갱신되지 않았습니다.'
            )
            return

        now = self.get_clock().now()

        # /thl100/data — JSON 형태로 묶음 발행 (Header timestamp 포함)
        payload = {
            'stamp_sec':   now.nanoseconds // 1_000_000_000,
            'stamp_nsec':  now.nanoseconds %  1_000_000_000,
            'sensor_id':   data['sensor_id'],
            'sequence':    data['sequence'],
            'temperature': data['temperature'],
            'humidity':    data['humidity'],
            'light':       data['light'],
            'valid':       data['valid'],
        }
        data_msg = String()
        data_msg.data = json.dumps(payload)
        self.pub_data.publish(data_msg)

        # 개별 Float32 토픽 병행 발행
        from std_msgs.msg import Float32
        if data['temperature'] is not None:
            m = Float32(); m.data = data['temperature']
            self.pub_temp.publish(m)
        if data['humidity'] is not None:
            m = Float32(); m.data = data['humidity']
            self.pub_humi.publish(m)
        if data['light'] is not None:
            m = Float32(); m.data = data['light']
            self.pub_light.publish(m)

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = THL100Node()
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
