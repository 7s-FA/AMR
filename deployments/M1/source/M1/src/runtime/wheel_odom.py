#!/usr/bin/env python3
# ========================================================================
# 역할: 버거3(STM32F411) odom. /burger1/joint_states(바퀴 각도) + /burger1/imu(방향) → /burger1/odom + TF(odom→base_footprint).
#   (2026-10-15) 버거1·2(OpenCR, use_imu) 와 같이 회전은 IMU 방향 변화로 계산한다. 이동 거리는 바퀴.
#   IMU 는 짝맞춤 없이 가장 최신 값을 쓴다. IMU 가 IMU_FRESH_S 넘게 안 오면 그 구간만 바퀴 회전으로 대신한다.
#   브리지 stm32_odometry 는 IMU 와 짝을 맞출 때만 계산해서 3Hz 밖에 안 나와(우리 코드 기준 0.3초) 대신 쓴다.
#   계산식은 ROBOTIS turtlebot3 odometry 와 같다: 이동 = r·(우+좌)/2, 회전 = IMU 방향 변화 (IMU 없으면 r·(우−좌)/분리거리).
#   추가한 보호: 첫 값, 0.5초 넘는 끊김, 시간 역행, 바퀴 각도 급변(토크 껐다 켤 때 STM32 가 각도를 0 으로 다시 시작)
#   이면 위치를 움직이지 않고 기준값만 다시 잡는다 → odom 이 튀지 않는다.
# 실행: base.launch.py 가 /burger1 네임스페이스, /tf→/burger1/tf 재매핑으로 띄운다. 모터 명령 없음.
# ========================================================================
import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
import os
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.033      # 브리지 bringup.launch.py 와 같은 값 (TurtleBot3 Burger)
WHEEL_SEPARATION = 0.160
MAX_GAP_S = 0.5           # 이보다 오래 끊기면 기준 다시 잡기 (브리지 IMU 경로와 같은 기준)
IMU_FRESH_S = 0.2         # 이보다 오래된 IMU 는 쓰지 않음 (IMU 약 20Hz)
POSE_FILE = os.path.join(os.environ.get('XDG_RUNTIME_DIR', '/tmp'), 'f411_wheel_odom_pose.json')
POSE_KEEP_S = 300.0       # 재시작 때 이 시간 안에 저장된 마지막 위치에서 이어서 시작 (AMCL 위치 유지)
USE_IMU = os.environ.get('F411_ODOM_IMU', '1') != '0'   # 0 이면 예전처럼 바퀴만
MAX_WHEEL_MPS = 0.3       # 이보다 빠른 바퀴 이동이 계산되면 각도 초기화로 보고 기준 다시 잡기 (실제 최고 약 0.1m/s)


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        ns = self.get_namespace().strip('/')
        prefix = ns + '/' if ns else ''
        self.frame, self.child = prefix + 'odom', prefix + 'base_footprint'
        self.left_name, self.right_name = prefix + 'wheel_left_joint', prefix + 'wheel_right_joint'
        self.x = self.y = self.th = 0.0
        self.saved_at = 0.0
        try:   # 본체 재시작 시 odom 이 0 으로 튀면 AMCL 위치가 틀어지므로 마지막 위치에서 이어간다 (2026-10-15)
            with open(POSE_FILE) as f:
                saved = json.load(f)
            if time.time() - saved['wall'] <= POSE_KEEP_S and saved.get('frame') == prefix + 'odom':
                self.x, self.y, self.th = saved['x'], saved['y'], saved['th']
        except (OSError, ValueError, KeyError):
            pass
        self.last = None            # (stamp_s, left_rad, right_rad)
        self.rebase = 0
        self.imu_yaw = None         # (yaw_rad, monotonic)
        self.last_imu_yaw = None    # 이전 odom 계산에 쓴 IMU 방향
        self.imu_steps = self.wheel_steps = 0
        self.pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.tf = TransformBroadcaster(self)
        self.create_subscription(JointState, 'joint_states', self.on_joint,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        if USE_IMU:
            self.create_subscription(Imu, 'imu', self.on_imu, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(30.0, self.report)
        self.get_logger().info(f'wheel_odom: {self.frame} → {self.child} (회전: {"IMU, 없으면 바퀴" if USE_IMU else "바퀴만"}), '
                               f'시작 위치 x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.th):.1f}°')

    def on_imu(self, msg):
        q = msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if math.isfinite(yaw):
            self.imu_yaw = (yaw, time.monotonic())

    def current_imu_yaw(self):
        if not USE_IMU or self.imu_yaw is None or time.monotonic() - self.imu_yaw[1] > IMU_FRESH_S:
            return None
        return self.imu_yaw[0]

    def report(self):
        total = self.imu_steps + self.wheel_steps
        if total:
            self.get_logger().info(f'30초 회전 계산: IMU {self.imu_steps} / 바퀴 대체 {self.wheel_steps}')
        self.imu_steps = self.wheel_steps = 0

    def on_joint(self, msg):
        try:
            names = list(msg.name)
            left = msg.position[names.index(self.left_name)]
            right = msg.position[names.index(self.right_name)]
        except (ValueError, IndexError):
            return
        if not (math.isfinite(left) and math.isfinite(right)):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = w = 0.0
        if self.last is not None:
            dt = stamp - self.last[0]
            dl, dr = left - self.last[1], right - self.last[2]
            limit = MAX_WHEEL_MPS * max(dt, 0.0) / WHEEL_RADIUS + 0.05
            if 0.0 < dt <= MAX_GAP_S and abs(dl) <= limit and abs(dr) <= limit:
                ds = WHEEL_RADIUS * (dr + dl) / 2.0
                dth = WHEEL_RADIUS * (dr - dl) / WHEEL_SEPARATION
                yaw = self.current_imu_yaw()
                if yaw is not None and self.last_imu_yaw is not None:
                    dth = math.atan2(math.sin(yaw - self.last_imu_yaw), math.cos(yaw - self.last_imu_yaw))
                    self.imu_steps += 1
                else:
                    self.wheel_steps += 1
                self.x += ds * math.cos(self.th + dth / 2.0)
                self.y += ds * math.sin(self.th + dth / 2.0)
                self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
                v, w = ds / dt, dth / dt
            elif dt <= 0.0 and self.last is not None and stamp == self.last[0]:
                return                      # 같은 시각 중복 메시지
            else:
                self.rebase += 1
                self.get_logger().warn(f'기준 다시 잡음 #{self.rebase}: dt={dt:.3f}s dl={dl:.3f} dr={dr:.3f}rad '
                                       '(끊김 또는 토크 전환으로 바퀴 각도 초기화)')
        self.last = (stamp, left, right)
        self.last_imu_yaw = self.current_imu_yaw()   # 다음 계산의 IMU 기준 (없으면 다음 한 번은 바퀴)
        self.publish(msg.header.stamp, v, w)

    def publish(self, stamp, v, w):
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.frame, self.child
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        for i, c in ((0, 1e-3), (7, 1e-3), (35, 1e-2)):
            odom.pose.covariance[i] = c
            odom.twist.covariance[i] = c
        self.pub.publish(odom)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.frame, self.child
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)
        if time.monotonic() - self.saved_at >= 0.5:
            self.saved_at = time.monotonic()
            try:
                tmp = POSE_FILE + '.tmp'
                with open(tmp, 'w') as f:
                    json.dump({'x': self.x, 'y': self.y, 'th': self.th, 'wall': time.time(), 'frame': self.frame}, f)
                os.replace(tmp, POSE_FILE)
            except OSError:
                pass


def main():
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
