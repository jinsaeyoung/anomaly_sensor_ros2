#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OSTSen-THL100 UART ROS2 노드
패킷 형식: @sensorID,Sequence,Temperature,Humidity,Light\r\n
예시:       @T010,100,28.5,38.3,236.5\r\n

퍼블리시 토픽:
  /thl100/temperature  - 온도 (Float32, °C)
  /thl100/humidity     - 습도 (Float32, %RH)
  /thl100/light        - 조도 (Float32, Lux)
  /thl100/raw          - 원시 패킷 문자열 (String)
"""

import serial
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class THL100Node(Node):

    def __init__(self):
        super().__init__('thl100_node')

        # 파라미터 선언
        self.declare_parameter('port',     '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # 퍼블리셔
        self.pub_temp  = self.create_publisher(Float32, '/thl100/temperature', 10)
        self.pub_humi  = self.create_publisher(Float32, '/thl100/humidity',    10)
        self.pub_light = self.create_publisher(Float32, '/thl100/light',       10)
        self.pub_raw   = self.create_publisher(String,  '/thl100/raw',         10)

        # 시리얼 포트 오픈
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2.0
            )
            self.get_logger().info(f'시리얼 포트 연결 성공: {port} @ {baudrate}bps')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 포트 열기 실패: {e}')
            raise

        # UART 수신 스레드 시작
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            'THL100Node 시작\n'
            '  /thl100/temperature  (Float32)  온도 °C\n'
            '  /thl100/humidity     (Float32)  습도 %RH\n'
            '  /thl100/light        (Float32)  조도 Lux\n'
            '  /thl100/raw          (String)   원시 패킷'
        )

    def _read_loop(self):
        """UART 수신 전용 스레드 — 패킷 파싱 후 퍼블리시"""
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()

                if not line:
                    continue

                # 원시 패킷 퍼블리시
                raw_msg      = String()
                raw_msg.data = line
                self.pub_raw.publish(raw_msg)

                # 패킷 검증: '@'로 시작해야 함
                if not line.startswith('@'):
                    self.get_logger().debug(f'헤더 없음, 무시: {line}')
                    continue

                # '@' 제거 후 파싱
                # 형식: sensorID,Sequence,Temperature,Humidity,Light
                body   = line[1:]
                fields = body.split(',')

                if len(fields) != 5:
                    self.get_logger().warn(f'필드 수 오류 ({len(fields)}개): {line}')
                    continue

                sensor_id = fields[0]   # 예: T010
                sequence  = fields[1]   # 예: 100
                temp_str  = fields[2]   # 예: 28.5
                humi_str  = fields[3]   # 예: 38.3
                light_str = fields[4]   # 예: 236.5

                # 값이 없는 경우(',' 만 있는 경우) 처리
                if temp_str:
                    temp_msg      = Float32()
                    temp_msg.data = float(temp_str)
                    self.pub_temp.publish(temp_msg)

                if humi_str:
                    humi_msg      = Float32()
                    humi_msg.data = float(humi_str)
                    self.pub_humi.publish(humi_msg)

                if light_str:
                    light_msg      = Float32()
                    light_msg.data = float(light_str)
                    self.pub_light.publish(light_msg)

                self.get_logger().debug(
                    f'[{sensor_id}] seq={sequence} '
                    f'temp={temp_str}°C humi={humi_str}%RH light={light_str}Lux'
                )

            except ValueError as e:
                self.get_logger().warn(f'값 변환 실패: {e} / 패킷: {line}')
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 읽기 오류: {e}')
                break
            except Exception as e:
                self.get_logger().warn(f'파싱 오류: {e}')

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
