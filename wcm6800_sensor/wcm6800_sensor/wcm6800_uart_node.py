#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Winson WCM6800 전류계 UART ROS2 노드
패킷 형식 (8바이트 고정, Continuous Mode):
  [type][d1][d2][d3][d4][d5]\r\n
  type: '~' = AC, '+' = DC+, '-' = DC-
  d1~d5: 5자리 ASCII 숫자 (소수점 3자리, 예: 01230 → 1.230A)

퍼블리시 토픽:
  /wcm6800/current      - 전류값 (Float32, A) — DC는 부호 포함
  /wcm6800/current_type - 측정 타입 (String: "AC" / "DC+" / "DC-")
  /wcm6800/raw          - 원시 패킷 (String)
"""

import serial
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class WCM6800Node(Node):

    def __init__(self):
        super().__init__('wcm6800_node')

        # 파라미터
        self.declare_parameter('port',     '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # 퍼블리셔
        self.pub_current = self.create_publisher(Float32, '/wcm6800/current',      10)
        self.pub_type    = self.create_publisher(String,  '/wcm6800/current_type', 10)
        self.pub_raw     = self.create_publisher(String,  '/wcm6800/raw',          10)

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

        # 수신 스레드 시작
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            'WCM6800Node 시작\n'
            '  /wcm6800/current       (Float32)  전류 A (DC는 부호 포함)\n'
            '  /wcm6800/current_type  (String)   AC / DC+ / DC-\n'
            '  /wcm6800/raw           (String)   원시 패킷'
        )

    def _parse_packet(self, line):
        """
        패킷 파싱
        입력: '~01230' 또는 '+01230' 또는 '-01230' (strip 후 6자)
        출력: (current_float, type_string)

        5자리 숫자 → 소수점 3자리
        예) '01230' → 1.230A
            '10760' → 10.760A
        """
        if len(line) != 6:
            raise ValueError(f'길이 오류: {len(line)} (예상: 6)')

        type_char = line[0]
        digits    = line[1:]  # 5자리

        if not digits.isdigit():
            raise ValueError(f'숫자 오류: {digits}')

        # 소수점 3자리 (예: 01230 → 1.230)
        value = int(digits) / 1000.0

        if type_char == '~':
            return value, 'AC'
        elif type_char == '+':
            return value, 'DC+'
        elif type_char == '-':
            return -value, 'DC-'
        else:
            raise ValueError(f'알 수 없는 타입: {type_char}')

    def _read_loop(self):
        """UART 수신 전용 스레드"""
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()

                if not line:
                    continue

                # 원시 패킷 퍼블리시
                raw_msg      = String()
                raw_msg.data = line
                self.pub_raw.publish(raw_msg)

                # 파싱
                current, current_type = self._parse_packet(line)

                # 전류 퍼블리시
                current_msg      = Float32()
                current_msg.data = float(current)
                self.pub_current.publish(current_msg)

                # 타입 퍼블리시
                type_msg      = String()
                type_msg.data = current_type
                self.pub_type.publish(type_msg)

                self.get_logger().debug(
                    f'[{current_type}] {current:.3f} A'
                )

            except ValueError as e:
                self.get_logger().warn(f'파싱 실패: {e} / 패킷: "{line}"')
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 읽기 오류: {e}')
                break
            except Exception as e:
                self.get_logger().warn(f'오류: {e}')

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
