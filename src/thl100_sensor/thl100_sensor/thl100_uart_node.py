#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OSTSen-THL100 UART ROS2 노드
패킷 형식: @sensorID,Sequence,Temperature,Humidity,Light\r\n

발행 토픽:
  /thl100/data  - 온도/습도/조도 묶음 (JSON, 센서 수신 시각 포함)
  /thl100/raw   - 원시 패킷 문자열 (수신 즉시 발행)
  /thl100/temperature, /humidity, /light  - 개별 Float32 (하위 호환)

특징:
  - UART 수신 스레드와 발행 타이머 분리
  - USB 분리 시 자동 재연결
  - 스트림 버퍼 기반 패킷 파서 (붙은 패킷 모두 처리)
  - 수신 시각 이중 기록: ROS clock(타임스탬프) + monotonic(경과시간 측정)
  - 동일 sequence 중복 발행 방지 옵션
"""

import os
import sys
import threading
import time
import json
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class THL100Node(Node):

    def __init__(self):
        super().__init__('thl100_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('port',              '/dev/ttyUSB0')
        self.declare_parameter('baudrate',          9600)
        self.declare_parameter('publish_rate_hz',   1.0)
        self.declare_parameter('stale_timeout_sec', 5.0)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('skip_duplicate',    False)
        self.declare_parameter('drain_max_sec',     3.0)
        # 자동 재탐색 — 젠더·포트가 바뀌어도 자기 장치를 찾아갑니다
        self.declare_parameter('autodetect',           True)
        self.declare_parameter('device_key',           'thl100')
        self.declare_parameter('identity_timeout_sec', 15.0)
        self.declare_parameter('rediscover_interval',  10.0)

        self.port            = self.get_parameter('port').value
        self.baudrate        = self.get_parameter('baudrate').value
        publish_rate_hz      = self.get_parameter('publish_rate_hz').value
        self.stale_sec       = self.get_parameter('stale_timeout_sec').value
        self.reconnect_delay = self.get_parameter('reconnect_delay_sec').value
        self.skip_duplicate  = self.get_parameter('skip_duplicate').value
        self.drain_max_sec   = self.get_parameter('drain_max_sec').value
        self.autodetect          = bool(self.get_parameter('autodetect').value)
        self.device_key          = self.get_parameter('device_key').value
        self.identity_timeout    = float(self.get_parameter('identity_timeout_sec').value)
        self.rediscover_interval = float(self.get_parameter('rediscover_interval').value)
        self._sa = None
        self._sa_failed = False
        self._last_rediscover = 0.0
        self._last_valid_mono = time.monotonic()
        self._connect_fail = 0

        # ── 퍼블리셔 ──────────────────────────────────────────────────
        self.pub_data  = self.create_publisher(String,  '/thl100/data',        10)
        self.pub_raw   = self.create_publisher(String,  '/thl100/raw',         10)
        self.pub_temp  = self.create_publisher(Float32, '/thl100/temperature', 10)
        self.pub_humi  = self.create_publisher(Float32, '/thl100/humidity',    10)
        self.pub_light = self.create_publisher(Float32, '/thl100/light',       10)

        # ── 상태 ──────────────────────────────────────────────────────
        self._latest        = None   # 파싱된 최신 데이터
        self._latest_rx_ns  = None   # 수신 시각 (ROS clock, ns) — 타임스탬프 기록용
        self._latest_rx_mono = None  # 수신 시각 (monotonic, s) — 경과시간 측정용
        self._last_pub_seq  = None   # 마지막 발행 sequence
        self._lock          = threading.Lock()
        self._buffer        = ''     # 스트림 버퍼

        # 진단 카운터
        self._stat = {'rx': 0, 'parse_ok': 0, 'parse_fail': 0, 'transient': 0,
                      'reconnect': 0, 'seq_gap': 0}
        self._stat_prev       = dict(self._stat)
        self._diag_period_sec = 30.0
        self._last_seq        = None

        self.ser         = None
        self._stop_event = threading.Event()

        # ── 수신 스레드 시작 (연결도 스레드 내에서 처리 → 재연결 가능) ──
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        # ── 발행 타이머 ───────────────────────────────────────────────
        self.create_timer(1.0 / publish_rate_hz, self._publish_timer)

        # ── 진단 타이머 (30초마다) ────────────────────────────────────
        self.create_timer(30.0, self._diag_timer)

        self.get_logger().info(
            f'THL100Node 시작 (port={self.port}, {publish_rate_hz}Hz)\n'
            f'  /thl100/data        — {publish_rate_hz}Hz (JSON, 수신 시각 포함)\n'
            f'  /thl100/raw         — 수신 즉시\n'
            f'  /thl100/temperature, /humidity, /light — {publish_rate_hz}Hz'
        )

    # ── 시리얼 연결/재연결 ────────────────────────────────────────────
    def _connect(self):
        # 다른 프로세스(mavros 등)가 이미 연 포트는 열지 않습니다.
        # mavros 는 파일 잠금을 쓰지 않아 exclusive 로도 막히지 않으므로,
        # 열어 버리면 그쪽 데이터를 빼앗아 양쪽이 모두 깨집니다.
        if self.autodetect:
            sa = self._load_autodetect()
            if sa is not None:
                try:
                    if os.path.realpath(self.port) in sa.ports_in_use():
                        self.get_logger().warn(
                            f'{self.port} 는 다른 프로세스가 사용 중 — 열지 않고 재탐색',
                            throttle_duration_sec=30.0)
                        return False
                except Exception:
                    pass
        try:
            # exclusive=True: 다른 노드·탐색 프로세스가 같은 포트를 동시에 열지 못하게 함
            self.ser = serial.Serial(
                exclusive=True,
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

    # ── 스트림 버퍼 파서 ──────────────────────────────────────────────
    # ══════════════════════════════════════════════════════════════════
    # 포트 재탐색
    #   USB 재삽입으로 ttyUSB 번호가 바뀌거나, 젠더를 다른 포트로 옮기면
    #   지정 포트가 사라지거나 다른 장치의 포트가 됩니다.
    #   이 경우 정상 패킷이 끊기므로, 비어 있는 포트를 다시 탐색해
    #   자기 장치를 찾아갑니다. (다른 노드가 쓰는 포트는 건드리지 않음)
    # ══════════════════════════════════════════════════════════════════
    def _load_autodetect(self):
        if self._sa is not None or self._sa_failed:
            return self._sa
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('drone_sensors'), 'scripts')
            if path not in sys.path:
                sys.path.insert(0, path)
            import serial_autodetect
            self._sa = serial_autodetect
        except Exception as e:
            self._sa_failed = True
            self.get_logger().warn(f'자동 재탐색 비활성 (serial_autodetect 로드 실패: {e})')
        return self._sa

    def _rediscover(self, reason):
        """비어 있는 포트에서 자기 장치를 찾아 self.port 를 갱신"""
        if not self.autodetect:
            return False
        now = time.monotonic()
        if now - self._last_rediscover < self.rediscover_interval:
            return False
        self._last_rediscover = now

        sa = self._load_autodetect()
        if sa is None:
            return False

        self.get_logger().warn(f'포트 재탐색 ({reason}) — 현재 {self.port}',
                               throttle_duration_sec=30.0)
        try:
            found = sa.find_port(self.device_key, exclude_busy=True)
        except Exception as e:
            self.get_logger().warn(f'재탐색 오류: {e}')
            return False

        if found and found != self.port:
            self.get_logger().info(f'장치를 새 포트에서 발견: {self.port} → {found}')
            self.port = found
            self._stat['rediscover'] = self._stat.get('rediscover', 0) + 1
            return True
        return False

    def _extract_packets(self, chunk):
        """
        버퍼에 누적 후 완전한 패킷만 추출
        패킷: '@' ... '\r\n'
        붙어서 들어온 여러 패킷도 모두 반환
        """
        self._buffer += chunk
        packets = []

        while True:
            start = self._buffer.find('@')
            if start < 0:
                # '@'가 없으면 버퍼 비움 (쓰레기 데이터)
                if len(self._buffer) > 512:
                    self._buffer = ''
                break

            end = self._buffer.find('\n', start)
            if end < 0:
                # 아직 패킷이 완성되지 않음 — 앞쪽 쓰레기만 제거
                self._buffer = self._buffer[start:]
                if len(self._buffer) > 512:
                    self._buffer = ''
                break

            packet = self._buffer[start:end].strip()
            self._buffer = self._buffer[end + 1:]
            if packet:
                packets.append(packet)

        return packets

    def _parse(self, packet):
        """단일 패킷 파싱 → dict, 실패 시 None"""
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
                'valid':       True,
            }
        except ValueError:
            return None

    # ── 수신 스레드 ───────────────────────────────────────────────────
    def _read_loop(self):
        while not self._stop_event.is_set():
            # 미연결 상태면 재연결 시도
            if self.ser is None or not self.ser.is_open:
                if not self._connect():
                    self._connect_fail += 1
                    # 포트가 사라졌거나 연속 실패하면 다른 포트에서 찾아봅니다
                    if not os.path.exists(self.port) or self._connect_fail >= 3:
                        if self._rediscover('포트 없음/연결 실패'):
                            self._connect_fail = 0
                            continue
                    self._stop_event.wait(self.reconnect_delay)
                    continue
                else:
                    self._connect_fail = 0
                    self._last_valid_mono = time.monotonic()
                    self._stat['reconnect'] += 1
                    self._buffer = ''
                    self._last_seq = None   # 재연결 후 첫 패킷은 gap 계산 제외

            try:
                # timeout 안에 데이터가 없으면 b'' 를 반환합니다(정상).
                # in_waiting 이 0 일 때 read(1) 로 블로킹하면
                # pyserial 이 "readiness to read but returned no data" 예외를
                # 던지는 경우가 있어, 대기 후 남은 바이트를 일괄로 읽습니다.
                # 연결은 되어 있는데 정상 패킷이 끊긴 경우:
                #   센서 전원이 빠졌거나, 재삽입 후 이 번호가 다른 장치에 할당된 것입니다.
                #   후자라면 계속 붙잡고 있으면 그 장치(예: FC)의 데이터를 빼앗으므로
                #   포트를 놓고 자기 장치를 다시 찾습니다.
                if (self.autodetect and
                        time.monotonic() - self._last_valid_mono > self.identity_timeout):
                    self.get_logger().warn(
                        f'{self.identity_timeout:.0f}초간 정상 패킷 없음 — 포트를 놓고 재탐색',
                        throttle_duration_sec=30.0)
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    self._last_valid_mono = time.monotonic()
                    self._last_rediscover = 0.0      # 즉시 재탐색 허용
                    self._rediscover('정상 패킷 없음')
                    continue

                n = self.ser.in_waiting
                if n:
                    chunk = self.ser.read(n)
                else:
                    # 짧게 쉬고 다시 확인 (CPU 점유 방지)
                    self._stop_event.wait(0.02)
                    continue
                if not chunk:
                    continue

                text = chunk.decode('ascii', errors='ignore')
                packets = self._extract_packets(text)

                for pkt in packets:
                    self._stat['rx'] += 1

                    # raw 즉시 발행
                    raw_msg = String()
                    raw_msg.data = pkt
                    self.pub_raw.publish(raw_msg)

                    parsed = self._parse(pkt)
                    if parsed is None:
                        self._stat['parse_fail'] += 1
                        continue

                    self._stat['parse_ok'] += 1
                    self._last_valid_mono = time.monotonic()

                    # sequence 누락 검사
                    seq = parsed['sequence']
                    if self._last_seq is not None:
                        expected = (self._last_seq + 1) % 10000
                        if seq != expected:
                            self._stat['seq_gap'] += 1
                    self._last_seq = seq

                    # 수신 시각 저장
                    #   ROS clock  → 발행 타임스탬프 (ROS 시간 체계 일관성)
                    #   monotonic  → 경과시간 측정 (NTP 시각 점프 영향 없음)
                    now_ns   = self.get_clock().now().nanoseconds
                    now_mono = time.monotonic()
                    with self._lock:
                        self._latest         = parsed
                        self._latest_rx_ns   = now_ns
                        self._latest_rx_mono = now_mono

            except (serial.SerialException, OSError) as e:
                msg = str(e)
                # 아래는 데이터가 잠시 없을 때도 발생하는 일시적 오류입니다.
                # 장치가 실제로 빠진 것이 아니므로 포트를 닫지 않고 재시도합니다.
                # (닫았다 여는 동작 자체가 USB 재열거링을 유발할 수 있음)
                transient = (
                    'readiness to read' in msg
                    or 'returned no data' in msg
                    or 'Interrupted system call' in msg
                )
                if transient and self.ser is not None and self.ser.is_open:
                    self._stat['transient'] = self._stat.get('transient', 0) + 1
                    self._stop_event.wait(0.05)
                    continue

                if not self._stop_event.is_set():
                    self.get_logger().warn(
                        f'시리얼 오류 — 재연결 시도: {e}',
                        throttle_duration_sec=5.0
                    )
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

    # ── 발행 타이머 ───────────────────────────────────────────────────
    def _publish_timer(self):
        with self._lock:
            data    = self._latest
            rx_ns   = self._latest_rx_ns
            rx_mono = self._latest_rx_mono

        if data is None:
            return

        # 경과시간은 monotonic 기준 (NTP 시각 변경에 영향받지 않음)
        age_sec = time.monotonic() - rx_mono

        if age_sec > self.stale_sec:
            self.get_logger().warn(
                f'THL100 데이터 stale ({age_sec:.1f}s > {self.stale_sec}s) — 발행 보류',
                throttle_duration_sec=10.0
            )
            return

        # 중복 발행 방지 (옵션)
        if self.skip_duplicate and data['sequence'] == self._last_pub_seq:
            return
        self._last_pub_seq = data['sequence']

        # /thl100/data — 센서 수신 시각(rx_ns) 기록
        payload = {
            'stamp_sec':   rx_ns // 1_000_000_000,
            'stamp_nsec':  rx_ns %  1_000_000_000,
            'age_ms':      round(age_sec * 1000, 1),
            'sensor_id':   data['sensor_id'],
            'sequence':    data['sequence'],
            'temperature': data['temperature'],
            'humidity':    data['humidity'],
            'light':       data['light'],
            'valid':       data['valid'],
        }
        m = String(); m.data = json.dumps(payload)
        self.pub_data.publish(m)

        # 개별 Float32
        if data['temperature'] is not None:
            f = Float32(); f.data = float(data['temperature']); self.pub_temp.publish(f)
        if data['humidity'] is not None:
            f = Float32(); f.data = float(data['humidity']);    self.pub_humi.publish(f)
        if data['light'] is not None:
            f = Float32(); f.data = float(data['light']);       self.pub_light.publish(f)

    # ── 진단 로그 (직전 구간 기준) ────────────────────────────────────
    def _diag_timer(self):
        s = self._stat
        p = self._stat_prev
        period = self._diag_period_sec

        d_tr   = s.get('transient', 0) - p.get('transient', 0)
        d_rx   = s['rx']         - p['rx']
        d_ok   = s['parse_ok']   - p['parse_ok']
        d_fail = s['parse_fail'] - p['parse_fail']
        d_gap  = s['seq_gap']    - p['seq_gap']
        self._stat_prev = dict(s)

        if d_rx == 0:
            self.get_logger().warn(f'THL100: 최근 {period:.0f}초간 수신 패킷 없음')
            return

        hz = d_rx / period
        self.get_logger().info(
            f"THL100 진단 [{period:.0f}s] rx={d_rx} ({hz:.2f}Hz) "
            f"ok={d_ok} fail={d_fail} seq_gap={d_gap} "
            f"| 누적 rx={s['rx']} reconnect={s['reconnect']} "
            f"transient={d_tr}"
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
