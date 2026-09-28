#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
센서 연결 상태 감시 노드

각 센서 토픽의 최근 수신 시각을 추적해 연결 상태를 집계합니다.
5초 주기로 /sensor_health 를 발행하며, monitor_drone 과
auto_record_node(프리플라이트 체크)가 이를 구독합니다.

기존 방식(토픽 hz 를 매번 새로 측정)은 호출마다 노드를 만들어
DDS discovery 를 거치므로 느리고 부정확했습니다.
상주 구독 노드가 수신 시각만 기록하면 즉시 조회할 수 있습니다.

발행 토픽:
  /sensor_health  (String, JSON)

판정:
  OK        최근 수신 있음 (timeout 이내)
  STALE     토픽은 있으나 timeout 초과
  NO_DATA   노드 실행 후 한 번도 수신 못 함
  NO_TOPIC  발행자 자체가 없음 (노드 사망)
"""

import json
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32, Float32


BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# 감시 대상: 그룹 → [(토픽, 메시지타입, 허용 무수신 시간(초))]
# 허용 시간은 발행 주기의 약 5배로 잡아 일시적 지연에 흔들리지 않게 합니다.
WATCH = {
    'THL100': [
        ('/thl100/data', String, 5.0),      # 1Hz
    ],
    'WCM6800': [
        ('/wcm6800/data', String, 3.0),     # 10Hz
    ],
    'ReSpeaker': [
        ('/respeaker/doa',    Int32,   3.0),   # 25Hz — USB(tuning) 경로
        # energy 는 오디오 스트림과 같은 콜백에서 발행되므로,
        # 12KB 짜리 /respeaker/audio 를 개수만 세려고 받지 않아도
        # 4바이트 짜리 이 토픽으로 오디오 경로 생존을 확인할 수 있습니다.
        # (audio 자체의 기록 여부는 verify_bag 이 bag 에서 확인)
        ('/respeaker/energy', Float32, 3.0),   # 15.6Hz — 오디오 경로
    ],
    'FC': [
        ('/mavros/imu/data', None, 2.0),    # 50Hz — 타입은 동적 로드
    ],
}


class SensorHealthNode(Node):

    def __init__(self):
        super().__init__('sensor_health_node')

        self.declare_parameter('publish_rate_hz', 0.2)   # 5초 주기
        rate = self.get_parameter('publish_rate_hz').value

        self.pub = self.create_publisher(String, '/sensor_health', 10)

        # 토픽별 마지막 수신 시각 (monotonic)
        self._last = {}
        self._count = {}
        # 최근 수신 시각 윈도우 — Hz 계산용
        # 모니터가 직접 고주기 토픽을 구독하면 rosbag 과 경쟁하므로
        # 여기서 한 번만 계산해 /sensor_health 에 실어 보냅니다.
        self._win = {}
        self._start = time.monotonic()
        self._ready_logged = False

        for group, items in WATCH.items():
            for topic, msg_type, _ in items:
                self._last[topic] = None
                self._count[topic] = 0
                self._win[topic] = deque(maxlen=50)

                if msg_type is None:
                    # mavros IMU — 타입을 런타임에 가져옵니다
                    try:
                        from sensor_msgs.msg import Imu
                        msg_type = Imu
                    except ImportError:
                        continue

                qos = BEST_EFFORT if topic.startswith('/mavros') else 10
                self.create_subscription(
                    msg_type, topic,
                    self._make_cb(topic), qos
                )

        self.create_timer(1.0 / rate, self._publish_health)

        self.get_logger().info(
            f'SensorHealthNode 시작 — {len(self._last)}개 토픽 감시, '
            f'{1.0/rate:.0f}초 주기 발행'
        )

    def _make_cb(self, topic):
        def cb(msg):
            now = time.monotonic()
            self._last[topic] = now
            self._count[topic] += 1
            self._win[topic].append(now)
        return cb

    def _hz_of(self, topic):
        """최근 수신 간격으로 실제 주기를 추정"""
        w = self._win.get(topic)
        if not w or len(w) < 2:
            return None
        span = w[-1] - w[0]
        if span <= 0:
            return None
        # 마지막 수신이 오래됐으면 중단 상태
        if time.monotonic() - w[-1] > 3.0:
            return 0.0
        return round((len(w) - 1) / span, 1)

    def _topic_has_publisher(self, topic):
        """발행자 존재 여부 — 노드가 죽었는지 판별"""
        try:
            return self.count_publishers(topic) > 0
        except Exception:
            return False

    def _status_of(self, topic, timeout):
        now = time.monotonic()
        last = self._last[topic]

        if last is not None:
            age = now - last
            if age <= timeout:
                return 'OK', round(age, 1)
            return 'STALE', round(age, 1)

        # 한 번도 못 받음 — 발행자 유무로 구분
        if self._topic_has_publisher(topic):
            return 'NO_DATA', None
        return 'NO_TOPIC', None

    def _publish_health(self):
        groups = {}
        all_ok = True

        for group, items in WATCH.items():
            topics = {}
            worst = 'OK'
            for topic, _, timeout in items:
                st, age = self._status_of(topic, timeout)
                topics[topic] = {
                    'status': st,
                    'age_s': age,
                    'count': self._count[topic],
                    'hz': self._hz_of(topic),
                }
                # 심각도: NO_TOPIC > NO_DATA > STALE > OK
                order = {'OK': 0, 'STALE': 1, 'NO_DATA': 2, 'NO_TOPIC': 3}
                if order[st] > order[worst]:
                    worst = st

            groups[group] = {'status': worst, 'topics': topics}
            if worst != 'OK':
                all_ok = False

        # 그룹 대표 Hz — 첫 번째 토픽 기준
        hz_summary = {}
        for g, items in WATCH.items():
            first = items[0][0]
            hz_summary[g] = self._hz_of(first)

        payload = {
            'all_ok': all_ok,
            'uptime_s': round(time.monotonic() - self._start, 1),
            'groups': {g: v['status'] for g, v in groups.items()},
            'hz': hz_summary,
            'detail': groups,
        }

        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(m)

        # 전원 인가 후 언제부터 수집 준비가 됐는지 한 번 남깁니다.
        # 드론과 전원을 공유하면 부팅 중에 이륙할 수 있어, 비행 후
        # '그때 준비가 됐었는지' 를 판단하는 기준이 됩니다.
        if all_ok and not self._ready_logged:
            self._ready_logged = True
            self.get_logger().info(
                f'수집 준비 완료 — 전 센서 정상 (노드 시작 후 {payload["uptime_s"]:.0f}초)')

        # 문제가 있으면 주기적으로 로그에도 남깁니다
        if not all_ok:
            bad = [f'{g}={v}' for g, v in payload['groups'].items() if v != 'OK']
            self.get_logger().warn(
                '센서 상태 이상: ' + ', '.join(bad),
                throttle_duration_sec=30.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorHealthNode()
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
