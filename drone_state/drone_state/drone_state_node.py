#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 상태 커스텀 토픽 발행 노드
mavros 토픽을 구독해서 필요한 변수만 추출 후 재발행
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float64

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, BatteryState
from mavros_msgs.msg import RCOut, AttitudeTarget


def quat_to_euler(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# mavros QoS — BEST_EFFORT + VOLATILE
MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class DroneStateNode(Node):

    def __init__(self):
        super().__init__('drone_state_node')

        def pub32(topic):
            return self.create_publisher(Float32, topic, 10)
        def pub64(topic):
            return self.create_publisher(Float64, topic, 10)

        # 1~3: 로컬좌표
        self.p_ned_x  = pub32('/drone/local_ned/x')
        self.p_ned_y  = pub32('/drone/local_ned/y')
        self.p_ned_z  = pub32('/drone/local_ned/z')
        # 4~6: 로컬좌표 목표
        self.p_ned_tx = pub32('/drone/local_ned_target/x')
        self.p_ned_ty = pub32('/drone/local_ned_target/y')
        self.p_ned_tz = pub32('/drone/local_ned_target/z')
        # 7~9: 로컬 선속도
        self.p_vel_x  = pub32('/drone/local_ned_vel/vx')
        self.p_vel_y  = pub32('/drone/local_ned_vel/vy')
        self.p_vel_z  = pub32('/drone/local_ned_vel/vz')
        # 10~12: 선속도 목표
        self.p_vel_tx = pub32('/drone/local_ned_vel_target/vx')
        self.p_vel_ty = pub32('/drone/local_ned_vel_target/vy')
        self.p_vel_tz = pub32('/drone/local_ned_vel_target/vz')
        # 13~15: 가속도
        self.p_acc_x  = pub32('/drone/accel/x')
        self.p_acc_y  = pub32('/drone/accel/y')
        self.p_acc_z  = pub32('/drone/accel/z')
        # 16~18: 자세각
        self.p_roll   = pub32('/drone/attitude/roll')
        self.p_pitch  = pub32('/drone/attitude/pitch')
        self.p_yaw    = pub32('/drone/attitude/yaw')
        # 19~21: 자세 목표
        self.p_roll_t  = pub32('/drone/attitude_target/roll')
        self.p_pitch_t = pub32('/drone/attitude_target/pitch')
        self.p_yaw_t   = pub32('/drone/attitude_target/yaw')
        # 22~24: 각속도
        self.p_rr     = pub32('/drone/attitude_rate/roll')
        self.p_pr     = pub32('/drone/attitude_rate/pitch')
        self.p_yr     = pub32('/drone/attitude_rate/yaw')
        # 25~27: 각속도 목표
        self.p_rr_t   = pub32('/drone/attitude_rate_target/roll')
        self.p_pr_t   = pub32('/drone/attitude_rate_target/pitch')
        self.p_yr_t   = pub32('/drone/attitude_rate_target/yaw')
        # 28~29: 배터리
        self.p_batt_v = pub32('/drone/battery/voltage')
        self.p_batt_i = pub32('/drone/battery/current')
        # 30~34: GPS
        self.p_lat    = pub64('/drone/gps/latitude')
        self.p_lon    = pub64('/drone/gps/longitude')
        self.p_alt    = pub32('/drone/gps/altitude')
        self.p_spd    = pub32('/drone/gps/ground_speed')
        self.p_crs    = pub32('/drone/gps/course_angle')
        # 35~37: Gyro
        self.p_gx     = pub32('/drone/gyro/x')
        self.p_gy     = pub32('/drone/gyro/y')
        self.p_gz     = pub32('/drone/gyro/z')
        # 38~40: 지자기
        self.p_mx     = pub32('/drone/mag/x')
        self.p_my     = pub32('/drone/mag/y')
        self.p_mz     = pub32('/drone/mag/z')
        # 41~48: 모터 PWM
        self.p_pwm    = [pub32(f'/drone/rcout/c{i}') for i in range(1, 9)]
        # 57~58: idle/max PWM
        self.p_idle_pwm = pub32('/drone/rcout/idle_pwm')
        self.p_max_pwm  = pub32('/drone/rcout/max_pwm')

        # ── 구독자 (MAVROS_QOS 적용) ─────────────────────────────────────
        self.create_subscription(PoseStamped,    '/mavros/local_position/pose',           self._cb_pose,         MAVROS_QOS)
        self.create_subscription(TwistStamped,   '/mavros/local_position/velocity_local', self._cb_vel,          MAVROS_QOS)
        self.create_subscription(PoseStamped,    '/mavros/setpoint_position/local',       self._cb_setpoint_pos, MAVROS_QOS)
        self.create_subscription(TwistStamped,   '/mavros/setpoint_velocity/cmd_vel',     self._cb_setpoint_vel, MAVROS_QOS)
        self.create_subscription(Imu,            '/mavros/imu/data',                      self._cb_imu,          MAVROS_QOS)
        self.create_subscription(Imu,            '/mavros/imu/data_raw',                  self._cb_imu_raw,      MAVROS_QOS)
        self.create_subscription(AttitudeTarget, '/mavros/setpoint_raw/target_attitude',  self._cb_att_target,   MAVROS_QOS)
        self.create_subscription(BatteryState,   '/mavros/battery',                       self._cb_battery,      MAVROS_QOS)
        self.create_subscription(NavSatFix,      '/mavros/global_position/raw/fix',       self._cb_gps,          MAVROS_QOS)
        self.create_subscription(MagneticField,  '/mavros/imu/mag',                       self._cb_mag,          MAVROS_QOS)
        self.create_subscription(RCOut,          '/mavros/rc/out',                        self._cb_rcout,        MAVROS_QOS)

        self.get_logger().info('DroneStateNode 시작 — mavros QoS(BEST_EFFORT) 적용')

    # ── 헬퍼 ─────────────────────────────────────────────────────────────

    def _pub(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def _pub64(self, publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    # ── 콜백 ─────────────────────────────────────────────────────────────

    def _cb_pose(self, msg):
        p = msg.pose.position
        self._pub(self.p_ned_x, p.x)
        self._pub(self.p_ned_y, p.y)
        self._pub(self.p_ned_z, p.z)
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self._pub(self.p_roll,  math.degrees(roll))
        self._pub(self.p_pitch, math.degrees(pitch))
        self._pub(self.p_yaw,   math.degrees(yaw))

    def _cb_vel(self, msg):
        v = msg.twist.linear
        self._pub(self.p_vel_x, v.x)
        self._pub(self.p_vel_y, v.y)
        self._pub(self.p_vel_z, v.z)

    def _cb_setpoint_pos(self, msg):
        p = msg.pose.position
        self._pub(self.p_ned_tx, p.x)
        self._pub(self.p_ned_ty, p.y)
        self._pub(self.p_ned_tz, p.z)

    def _cb_setpoint_vel(self, msg):
        v = msg.twist.linear
        self._pub(self.p_vel_tx, v.x)
        self._pub(self.p_vel_ty, v.y)
        self._pub(self.p_vel_tz, v.z)

    def _cb_imu(self, msg):
        a = msg.linear_acceleration
        self._pub(self.p_acc_x, a.x)
        self._pub(self.p_acc_y, a.y)
        self._pub(self.p_acc_z, a.z)
        g = msg.angular_velocity
        self._pub(self.p_rr, math.degrees(g.x))
        self._pub(self.p_pr, math.degrees(g.y))
        self._pub(self.p_yr, math.degrees(g.z))

    def _cb_imu_raw(self, msg):
        g = msg.angular_velocity
        self._pub(self.p_gx, math.degrees(g.x))
        self._pub(self.p_gy, math.degrees(g.y))
        self._pub(self.p_gz, math.degrees(g.z))

    def _cb_att_target(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self._pub(self.p_roll_t,  math.degrees(roll))
        self._pub(self.p_pitch_t, math.degrees(pitch))
        self._pub(self.p_yaw_t,   math.degrees(yaw))
        r = msg.body_rate
        self._pub(self.p_rr_t, math.degrees(r.x))
        self._pub(self.p_pr_t, math.degrees(r.y))
        self._pub(self.p_yr_t, math.degrees(r.z))

    def _cb_battery(self, msg):
        self._pub(self.p_batt_v, msg.voltage)
        self._pub(self.p_batt_i, msg.current)

    def _cb_gps(self, msg):
        self._pub64(self.p_lat, msg.latitude)
        self._pub64(self.p_lon, msg.longitude)
        self._pub(self.p_alt,   msg.altitude)
        self._pub(self.p_spd,   0.0)
        self._pub(self.p_crs,   0.0)

    def _cb_mag(self, msg):
        m = msg.magnetic_field
        self._pub(self.p_mx, m.x)
        self._pub(self.p_my, m.y)
        self._pub(self.p_mz, m.z)

    def _cb_rcout(self, msg):
        for i, pub in enumerate(self.p_pwm):
            if i < len(msg.channels):
                self._pub(pub, msg.channels[i])
        if msg.channels:
            self._pub(self.p_idle_pwm, min(msg.channels))
            self._pub(self.p_max_pwm,  max(msg.channels))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DroneStateNode()
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
