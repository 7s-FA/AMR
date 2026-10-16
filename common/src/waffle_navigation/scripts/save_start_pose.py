#!/usr/bin/env python3
# ========================================================================
# 역할: 지금 멈춰 있는 위치(AMCL)를 Nav2 설정 파일의 AMCL 초기 위치로 저장하는 도구 (다음 부팅부터 그 자리에서 시작).
# 실행: 수동. save_start_pose.sh [--preview]. 설정 파일의 다른 값·주석은 그대로 둔다.
# ========================================================================
"""Register a stationary, localized Burger pose as AMCL's persistent start pose.

Run once at the real starting place after aligning RViz with the robot.
This command subscribes to localization/odometry only; it never commands motion.
"""

import argparse
import copy
import math
from pathlib import Path
import re
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


# 로봇 접두어 좌표계 이름.
def frame_name(reader, name):
    return getattr(reader, 'frame_prefix', '') + name


# 설정 텍스트에서 AMCL 초기 위치 숫자만 바꾼 새 텍스트를 만든다.
def updated_config(original, pose):
    """Change only AMCL initialization scalars, preserving all tuning and comments."""
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise ValueError('시작 위치는 유한한 x, y, yaw 값이어야 합니다.')
    before = yaml.safe_load(original)
    expected = copy.deepcopy(before)
    params = expected['amcl']['ros__parameters']
    params['set_initial_pose'] = True
    params['initial_pose'] = dict(x=float(pose[0]), y=float(pose[1]), z=0.0, yaw=float(pose[2]))
    match = re.search(r'^amcl:\s*\n(?:(?:[ \t].*|)\n)*', original, flags=re.MULTILINE)
    if not match:
        raise ValueError('AMCL 설정 블록을 찾지 못했습니다.')
    block = match.group()
    # 키 하나의 값만 정확히 한 번 교체.
    def replace_one(pattern, replacement):
        nonlocal block
        block, count = re.subn(pattern, replacement, block, flags=re.MULTILINE)
        if count != 1:
            raise ValueError('AMCL 초기 위치 설정 형식을 확인하세요. 파일을 변경하지 않았습니다.')
    replace_one(r'^(    set_initial_pose:)\s*[^\n]*$', r'\1 true')
    # These exact indent levels belong to the existing initial_pose mapping.
    for key, value in params['initial_pose'].items():
        replace_one(rf'^(      {key}:)\s*[^\n]*$', rf'\g<1> {value!r}')
    updated = original[:match.start()] + block + original[match.end():]
    if yaml.safe_load(updated) != expected:
        raise ValueError('초기 위치 이외의 변경이 감지되어 저장을 중단했습니다.')
    return updated


# 백업 후 설정 파일 저장.
def save_config(path, pose):
    path = Path(path).expanduser().resolve(strict=True)
    original = path.read_text(encoding='utf-8')
    updated = updated_config(original, pose)
    backup = path.with_name(path.name + '.before_start_pose')
    # Preserve the first pre-registration configuration as a recovery copy.
    if not backup.exists():
        backup.write_text(original, encoding='utf-8')
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=path.name + '.', delete=False) as stream:
            temp_path = Path(stream.name)
            stream.write(updated)
        temp_path.chmod(path.stat().st_mode & 0o777)
        temp_path.replace(path)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
    return path


# AMCL 위치와 odom 으로 '멈춰 있는 최신 위치'를 읽는 노드.
class StartPoseReader(Node):
    # 구독 생성.
    def __init__(self, namespace=''):
        super().__init__('burger_start_pose_reader', namespace=namespace)
        prefix = self.get_namespace().strip('/')
        self.frame_prefix = prefix + '/' if prefix else ''
        self.localized = False
        self.stationary_since = None
        self.odom_received = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)

    # AMCL 위치 저장.
    def on_amcl(self, msg):
        self.localized = msg.header.frame_id == frame_name(self, 'map')

    # 메시지가 최신인지.
    def fresh(self, stamp):
        age = (self.get_clock().now().nanoseconds-rclpy.time.Time.from_msg(stamp).nanoseconds)/1e9
        return -0.1 <= age <= 0.5

    # odom 으로 정지 상태 판단.
    def on_odom(self, msg):
        now = time.monotonic()
        v = msg.twist.twist
        stopped = (all(math.isfinite(value) for value in (v.linear.x, v.linear.y, v.angular.z))
                   and math.hypot(v.linear.x, v.linear.y) < 0.01 and abs(v.angular.z) < 0.03
                   and self.fresh(msg.header.stamp))
        if (not stopped or self.odom_received is None or now-self.odom_received > 0.3):
            self.stationary_since = now if stopped else None
        elif self.stationary_since is None:
            self.stationary_since = now
        self.odom_received = now

    # 정지한 최신 위치를 제한 시간 안에 읽는다.
    def read_pose(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if (not self.localized or self.stationary_since is None
                    or now-self.stationary_since < 0.75 or now-self.odom_received > 0.3):
                continue
            try:
                tf = self.buffer.lookup_transform(frame_name(self, 'map'), frame_name(self, 'base_footprint'), rclpy.time.Time())
            except TransformException:
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            if (not self.fresh(tf.header.stamp)
                    or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0) > 0.01):
                continue
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            return p.x, p.y, yaw
        raise RuntimeError('정지 상태의 최신 위치를 확인하지 못했습니다. Nav2/bringup과 '
                           'RViz 초기 위치를 확인하세요. 설정 파일은 변경하지 않았습니다.')


# 위치 읽기 → (미리보기 또는) 설정 저장.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='', help='Robot namespace and TF frame prefix')
    parser.add_argument('--params-file', default=str(Path(get_package_share_directory(
        'waffle_navigation')) / 'config/nav2_burger_params.yaml'))
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--preview', action='store_true', help='현재 시작 위치 후보만 출력하고 저장하지 않음')
    args = parser.parse_args(remove_ros_args()[1:])
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout은 0보다 큰 유한한 숫자여야 합니다.')
    rclpy.init()
    reader = StartPoseReader(namespace=args.namespace)
    try:
        print('실제 시작 자리의 정지 위치를 읽습니다. RViz 위치/방향이 맞아야 합니다.', flush=True)
        pose = reader.read_pose(args.timeout)
        print(f'x={pose[0]:.9f}, y={pose[1]:.9f}, yaw={pose[2]:.9f} rad '
              f'({math.degrees(pose[2]):.3f} deg)', flush=True)
        if not args.preview:
            path = save_config(args.params_file, pose)
            print(f'저장 완료: {path}\n다음 Nav2 시작부터 이 위치로 자동 초기화합니다. '
                  '현재 AMCL 위치는 변경하지 않았습니다.')
        return 0
    except (RuntimeError, ValueError, KeyError, OSError, yaml.YAMLError) as exc:
        print(f'시작 위치 등록 실패: {exc}')
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        reader.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
