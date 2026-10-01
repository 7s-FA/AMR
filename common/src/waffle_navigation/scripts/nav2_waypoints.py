#!/usr/bin/env python3
"""Visit map waypoints with a forward-only or reverse-only controller per leg."""

import argparse
import importlib.util
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener
from sensor_msgs.msg import LaserScan
import tf_transformations
import yaml


CONTROLLERS = {'forward': 'FollowPositionForward', 'reverse': 'FollowPositionReverse'}
# Legacy fallback used only after a completed intermediate Spin; the Burger1
# route overrides this in waypoints.yaml. Pre-turn and final gates remain strict.
INTERMEDIATE_XY_TOLERANCE = 0.10
REAR_SCAN_RETRY_SECONDS = 0.5
TERMINAL_APPROACH_DISTANCE = 0.10
TERMINAL_MAX_DISTANCE = 0.15  # Reject a localization shift during the stop/turn.
TERMINAL_SPEED = 0.03
TERMINAL_HANDOFF = object()
INTERMEDIATE_HANDOFF = object()


class RearScanCoverageError(RuntimeError):
    """A temporary gap in lidar coverage, distinct from a detected obstacle."""


class ApproachOvershootGuard:
    """Abort a translation that recedes after approaching the map goal.

    This is a stop condition, never an alternative arrival/success condition.
    Make a new instance for every goal attempt; do not apply it to Spin/backup.
    """
    approach_distance = 0.10
    recession_distance = 0.03
    hold_seconds = 0.30

    def __init__(self):
        self.best_distance = math.inf
        self.receding_since = None

    def update(self, distance, now):
        if not math.isfinite(distance) or distance < 0 or not math.isfinite(now):
            raise ValueError('목표 지나침 감시에 유효하지 않은 거리/시간입니다.')
        if distance <= self.approach_distance:
            self.best_distance = min(self.best_distance, distance)
        if distance >= self.best_distance + self.recession_distance:
            if self.receding_since is None:
                self.receding_since = now
            return now - self.receding_since >= self.hold_seconds
        self.receding_since = None
        return False


def planned_path_distance(points, pose):
    """Distance to the polyline, including segment interiors and endpoints."""
    if not points:
        return None
    best = math.hypot(pose[0]-points[0][0], pose[1]-points[0][1])
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length_squared = dx*dx + dy*dy
        t = (max(0., min(1., ((pose[0]-a[0])*dx + (pose[1]-a[1])*dy)
                             / length_squared)) if length_squared else 0.)
        best = min(best, math.hypot(pose[0]-a[0]-t*dx, pose[1]-a[1]-t*dy))
    return best


class PlannedPathGuard:
    """Nav2 routes can bend; a start-to-goal axis is not their path.

    Only a fresh plan for this goal can establish a path deviation. A brief
    localization/replan transition must not cancel an otherwise valid route.
    Missing plans do not imply arrival and never weaken Nav2's own checks.
    """
    deviation_distance = .15
    hold_seconds = 1.0

    def __init__(self):
        self.deviating_since = None

    def update(self, offset, now):
        if offset is None:
            self.deviating_since = None
            return False
        if not math.isfinite(offset) or offset < 0 or not math.isfinite(now):
            raise ValueError('경로 이탈 감시에 유효하지 않은 거리/시간입니다.')
        if offset > self.deviation_distance:
            if self.deviating_since is None:
                self.deviating_since = now
            return now-self.deviating_since >= self.hold_seconds
        self.deviating_since = None
        return False


def missed_target_axis(start, waypoint, pose, distance):
    """Stop when the main travel coordinate is nearly passed off-path.

    Distance-only overshoot detection cannot arm if the robot runs alongside
    the goal at a nearly constant lateral offset (the observed Burger1 fault).
    """
    dx = waypoint['x'] - start[0]
    dy = waypoint['y'] - start[1]
    if math.hypot(dx, dy) < 0.25:
        return False
    if abs(dx) >= abs(dy):
        axis_total, axis_done = dx, pose[0] - start[0]
    else:
        axis_total, axis_done = dy, pose[1] - start[1]
    return (axis_total * axis_done >= 0.85 * axis_total * axis_total
            and distance > 0.15)


def frame_name(nav, name):
    return getattr(nav, 'frame_prefix', '') + name


def action_in_namespace(topic, namespace):
    return ('/' + topic.lstrip('/')).startswith(namespace.rstrip('/') + '/')


def spin_for(nav, duration):
    """Drain TF/odom/scan callbacks between commands instead of only one callback."""
    end = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=max(0.0, end-time.monotonic()))


def planar_pose(x, y, q):
    if (not all(math.isfinite(v) for v in (x, y, q.x, q.y, q.z, q.w))
            or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01):
        raise RuntimeError('유효하지 않은 출차 위치/방향입니다.')
    return x, y, math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def backup_progress(start, pose):
    dx, dy = pose[0]-start[0], pose[1]-start[1]
    progress = -(dx*math.cos(start[2]) + dy*math.sin(start[2]))
    lateral = -dx*math.sin(start[2]) + dy*math.cos(start[2])
    yaw_error = math.atan2(math.sin(pose[2]-start[2]), math.cos(pose[2]-start[2]))
    if progress < -0.01 or abs(lateral) > 0.025 or abs(yaw_error) > math.radians(10):
        raise RuntimeError('직선 후진 방향/횡방향 오차가 허용 범위를 벗어났습니다.')
    return progress


def check_rear_scan(scan, tx, ty, yaw):
    """Require observed rear sectors and guard Burger's rear with a 5+ cm margin."""
    if (not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_increment,
                                         scan.range_min, scan.range_max))
            or scan.angle_increment <= 0 or scan.range_min < 0
            or scan.range_max <= max(0.3, scan.range_min) or not scan.ranges):
        raise RuntimeError('출차 scan 형식이 올바르지 않습니다.')
    # Fifteen 10-degree bins cover +/-75 degrees around the rear direction.
    observed = set()
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index*scan.angle_increment + yaw
        rear_angle = math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi))
        valid = (scan.range_min <= value <= scan.range_max and value > 0)
        if abs(rear_angle) <= math.radians(75) and (valid or value == math.inf):
            observed.add(min(14, int((math.degrees(rear_angle)+75)/10)))
        if valid:
            x, y = tx + value*math.cos(angle), ty + value*math.sin(angle)
            if -0.19 <= x <= 0.0 and abs(y) <= 0.14:
                raise RuntimeError('후방 정지 영역에서 장애물이 감지되었습니다.')
    if len(observed) != 15:
        raise RearScanCoverageError('후방 라이다 관측이 부족합니다.')


class WaypointNavigator(BasicNavigator):
    def __init__(self, namespace=''):
        super().__init__(namespace=namespace)
        prefix = self.get_namespace().strip('/')
        self.frame_prefix = prefix + '/' if prefix else ''
        self.odom_received_at = None
        self.odom_stopped = False
        self.odom_message = None
        self.scan_message = None
        self.scan_received_at = None
        self.plan_message = None
        self.plan_received_at = None
        self.plan_points = ()
        self.plan_subscription = self.create_subscription(
            NavPath, 'plan', self._on_plan, qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.xy_tolerance = 0.05
        self.yaw_tolerance = 0.25
        self.arrival_tuning = {}
        self.intermediate_xy_tolerance = INTERMEDIATE_XY_TOLERANCE

    def _waitForNodeToActivate(self, node_name):
        """Bound state requests: a lost DDS response must not hang forever."""
        from lifecycle_msgs.srv import GetState
        client = self.create_client(GetState, f'{node_name}/get_state')
        deadline = time.monotonic() + 90.0
        startup_attempted = False
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{node_name}: 상태 서비스 연결 대기')
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                if not future.done():
                    client.remove_pending_request(future)
                    future.cancel()
                    self.info(f'{node_name}: 상태 응답 지연, 다시 조회합니다')
                    continue
                response = future.result()
                if response is not None and response.current_state.label == 'active':
                    return
                if node_name == 'bt_navigator' and not startup_attempted:
                    startup_attempted = True
                    from nav2_msgs.srv import ManageLifecycleNodes
                    manager = self.create_client(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
                    try:
                        if manager.wait_for_service(timeout_sec=3.0):
                            self.info('초기 위치 확인 후 주행 서버 활성화를 다시 요청합니다.')
                            request = ManageLifecycleNodes.Request(command=0)
                            pending = manager.call_async(request)
                            rclpy.spin_until_future_complete(self, pending, timeout_sec=15.0)
                            if not pending.done():
                                manager.remove_pending_request(pending)
                                pending.cancel()
                            elif pending.result() is not None and not pending.result().success:
                                raise RuntimeError('Nav2 활성화 실패. 주행 서버 로그를 확인하세요.')
                    finally:
                        self.destroy_client(manager)
                self.info(f'{node_name}: 활성화 대기')
                spin_for(self, 0.5)
            raise RuntimeError(f'{node_name}: 90초 내 활성화 확인 실패. Nav2 Startup 상태를 확인하세요.')
        finally:
            self.destroy_client(client)

    def _waitForInitialPose(self):
        """Use RViz/AMCL localization; never publish BasicNavigator's default origin."""
        if not self.initial_pose_received:
            self.info('AMCL 초기 위치를 기다립니다. 저장된 시작 위치는 Nav2가 자동 적용합니다. '
                      '시작 위치를 등록하지 않았다면 RViz의 2D Pose Estimate로 지정하세요.')
        while rclpy.ok() and not self.initial_pose_received:
            rclpy.spin_once(self, timeout_sec=1.0)
        if not self.initial_pose_received:
            raise RuntimeError('AMCL 위치를 받기 전에 ROS가 종료되었습니다.')

    def _on_odom(self, msg):
        self.odom_message = msg
        velocity = msg.twist.twist
        self.odom_received_at = time.monotonic()
        self.odom_stopped = (
            math.hypot(velocity.linear.x, velocity.linear.y) < 0.01
            and abs(velocity.angular.z) < 0.03)

    def _on_scan(self, msg):
        self.scan_message = msg
        self.scan_received_at = time.monotonic()

    def _on_plan(self, msg):
        self.plan_message = msg
        self.plan_received_at = time.monotonic()
        points = tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses)
        valid = (msg.header.frame_id == frame_name(self, 'map')
                 and all(p.header.frame_id in ('', msg.header.frame_id) for p in msg.poses)
                 and all(math.isfinite(v) for point in points for v in point))
        self.plan_points = points if valid else ()

    def current_plan_offset(self, waypoint, pose, goal_started):
        msg, received = self.plan_message, self.plan_received_at
        points = self.plan_points
        # A previous goal's latched/stale plan is not evidence about this route.
        if (not points or msg is None or received is None or received < goal_started
                or time.monotonic()-received > 2.5
                or not self._fresh_stamp(msg.header.stamp, 2.5)
                or math.hypot(points[-1][0]-waypoint['x'],
                              points[-1][1]-waypoint['y']) > .05):
            return None
        return planned_path_distance(points, pose)

    def _departure_pose(self):
        msg = self.odom_message
        if (msg is None or self.odom_received_at is None
                or time.monotonic() - self.odom_received_at > 0.3
                or not self._fresh_stamp(msg.header.stamp, 0.3)):
            raise RuntimeError('출차 중 최신 odom이 없습니다.')
        if (msg.header.frame_id != frame_name(self, 'odom')
                or msg.child_frame_id not in (frame_name(self, 'base_link'), frame_name(self, 'base_footprint'))):
            raise RuntimeError('출차 odom 좌표계가 예상과 다릅니다.')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return planar_pose(p.x, p.y, q)

    def _fresh_stamp(self, stamp, limit):
        age = (self.get_clock().now().nanoseconds
               - rclpy.time.Time.from_msg(stamp).nanoseconds) / 1e9
        return -0.1 <= age <= limit

    def _check_departure_scan(self):
        msg = self.scan_message
        if (msg is None or self.scan_received_at is None
                or time.monotonic() - self.scan_received_at > 0.5
                or not self._fresh_stamp(msg.header.stamp, 0.5)):
            raise RuntimeError('출차 중 최신 scan이 없습니다.')
        try:
            tf = self.tf_buffer.lookup_transform(
                frame_name(self, 'base_footprint'), msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
        except TransformException as exc:
            raise RuntimeError('출차 scan의 차체 좌표 변환이 없습니다.') from exc
        t, q = tf.transform.translation, tf.transform.rotation
        x, y, yaw = planar_pose(t.x, t.y, q)
        check_rear_scan(msg, x, y, yaw)

    def _verify_departure_pipeline(self):
        # Send through the existing smoother and collision monitor, never directly
        # to the motor topic. Require the project's stamped-velocity wiring.
        for node in ('velocity_smoother', 'collision_monitor'):
            client = self.create_client(GetState, f'{node}/get_state')
            try:
                if not client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f'{node} 상태 서비스를 찾을 수 없습니다.')
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                if not future.done() or future.result() is None or future.result().current_state.id != 3:
                    raise RuntimeError(f'{node}가 활성화되어 있지 않습니다.')
            finally:
                self.destroy_client(client)
        self._wait_for_velocity_connections()

    def _wait_for_velocity_connections(self, timeout=5.0):
        # DDS graph discovery can lag behind lifecycle service discovery.
        # Never send movement commands until both expected subscribers appear.
        namespace = self.get_namespace().rstrip('/')
        deadline = time.monotonic() + timeout
        missing = []
        while True:
            missing = []
            for topic, consumer in (('cmd_vel_nav', 'velocity_smoother'),
                                    ('cmd_vel_smoothed', 'collision_monitor')):
                full_topic = f'{namespace}/{topic}'
                endpoints = self.get_subscriptions_info_by_topic(full_topic)
                if not any(info.node_name == consumer
                           and info.node_namespace.rstrip('/') == namespace
                           and info.topic_type == 'geometry_msgs/msg/TwistStamped'
                           for info in endpoints):
                    missing.append(f'{full_topic} → {consumer}')
            if not missing:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not rclpy.ok():
                raise RuntimeError('속도 명령 연결 확인 시간 초과: ' + ', '.join(missing))
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def pre_backup(self, distance, speed, deadline):
        """Short odometry-measured departure before any waypoint goal is sent."""
        self._verify_departure_pipeline()
        active = {}
        status_subs = []
        publisher = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            for topic, types in self.get_topic_names_and_types():
                namespace = getattr(self, 'get_namespace', lambda: '/')()
                if (action_in_namespace(topic, namespace)
                        and topic.endswith('/_action/status') and 'action_msgs/msg/GoalStatusArray' in types):
                    def receive(msg, name=topic):
                        active[name] = any(item.status in (1, 2, 3) for item in msg.status_list)
                    status_subs.append(self.create_subscription(GoalStatusArray, topic, receive, qos))
            # Gather action status and fresh stationary odometry before publishing.
            if not self.wait_until_stopped(timeout=min(5.0, max(0.0, deadline-time.monotonic()))):
                raise RuntimeError('출차 전 정지를 확인하지 못했습니다.')
            if any(active.values()):
                raise RuntimeError('다른 ROS 액션이 실행 중입니다. 출차를 시작하지 않습니다.')
            coverage_retry_until = time.monotonic() + REAR_SCAN_RETRY_SECONDS
            while True:
                try:
                    self._check_departure_scan()
                    break
                except RearScanCoverageError as exc:
                    if time.monotonic() >= coverage_retry_until:
                        raise RuntimeError(f'{exc} 후진을 시작하지 않습니다.') from exc
                    spin_for(self, 0.05)
            start = self._departure_pose()
            publisher = self.create_publisher(TwistStamped, 'cmd_vel_nav', 1)
            end = min(deadline, time.monotonic() + distance / speed * 2.0 + 5.0)
            best, last_progress = 0.0, time.monotonic()
            previous = start
            coverage_gap_since = None
            print(f'[출차] {distance:.2f} m 후진, 최대 {speed:.2f} m/s', flush=True)
            while rclpy.ok():
                spin_for(self, 0.05)
                now = time.monotonic()
                if now >= end:
                    raise RuntimeError('출차 제한 시간 초과: 웨이포인트를 시작하지 않습니다.')
                if any(active.values()):
                    raise RuntimeError('출차 중 다른 액션이 시작되었습니다.')
                try:
                    self._check_departure_scan()
                    coverage_gap_since = None
                except RearScanCoverageError as exc:
                    # Never keep commanding reverse while a sector is unknown.
                    stop = TwistStamped()
                    stop.header.stamp = self.get_clock().now().to_msg()
                    stop.header.frame_id = frame_name(self, 'base_footprint')
                    publisher.publish(stop)
                    if coverage_gap_since is None:
                        coverage_gap_since = now
                    if now - coverage_gap_since >= REAR_SCAN_RETRY_SECONDS:
                        raise RuntimeError(f'{exc} 0.5초간 회복되지 않아 출차를 중단합니다.') from exc
                    continue
                pose = self._departure_pose()
                if math.hypot(pose[0]-previous[0], pose[1]-previous[1]) > 0.03:
                    raise RuntimeError('출차 중 odom 위치가 갑자기 변했습니다.')
                previous = pose
                progress = backup_progress(start, pose)
                if progress >= distance:
                    print(f'[출차] odom 후진 거리 {progress:.3f} m, 정지', flush=True)
                    break
                if progress > best + 0.005:
                    best, last_progress = progress, now
                if now - last_progress > 3.0:
                    raise RuntimeError('출차 중 이동이 없습니다. 장애물 감시 상태를 확인하세요.')
                command = TwistStamped()
                command.header.stamp = self.get_clock().now().to_msg()
                command.header.frame_id = frame_name(self, 'base_footprint')
                command.twist.linear.x = -min(speed, max(0.015, (distance-progress)*0.8))
                publisher.publish(command)
            else:
                raise RuntimeError('출차 중 ROS가 종료되었습니다.')
        finally:
            if publisher is not None:
                # Flush repeated fresh stop commands on success, failure and Ctrl+C.
                try:
                    for _ in range(10):
                        command = TwistStamped()
                        command.header.stamp = self.get_clock().now().to_msg()
                        command.header.frame_id = frame_name(self, 'base_footprint')
                        publisher.publish(command)
                        time.sleep(0.05)
                finally:
                    self.destroy_publisher(publisher)
            for subscription in status_subs:
                self.destroy_subscription(subscription)
        if not self.wait_until_stopped(timeout=min(5.0, max(0.0, deadline-time.monotonic()))):
            raise RuntimeError('출차 후 정지를 확인하지 못했습니다. 웨이포인트를 시작하지 않습니다.')

    def pre_backup_timed(self, distance, speed, deadline):
        """Open-loop departure: command speed for distance/speed seconds.

        This routine intentionally reads neither scan nor odom. The downstream
        velocity smoother and collision monitor still receive the commands.
        """
        self._verify_departure_pipeline()
        duration = distance / speed
        if time.monotonic() + duration + 1.0 >= deadline:
            raise RuntimeError('시간제어 후진에 필요한 시간이 부족합니다.')
        publisher = self.create_publisher(TwistStamped, 'cmd_vel_nav', 1)
        try:
            print(f'[출차] 시간제어 후진: {-speed:.3f} m/s × {duration:.2f} s '
                  f'(계산상 {distance:.3f} m, 실측 아님)', flush=True)
            end = time.monotonic() + duration
            while rclpy.ok() and time.monotonic() < end:
                command = TwistStamped()
                command.header.stamp = self.get_clock().now().to_msg()
                command.header.frame_id = frame_name(self, 'base_footprint')
                command.twist.linear.x = -speed
                publisher.publish(command)
                spin_for(self, min(0.05, max(0.0, end - time.monotonic())))
            if not rclpy.ok():
                raise RuntimeError('시간제어 후진 중 ROS가 종료되었습니다.')
        finally:
            # Explicit stop on completion, failure, and Ctrl+C.
            for _ in range(12):
                command = TwistStamped()
                command.header.stamp = self.get_clock().now().to_msg()
                command.header.frame_id = frame_name(self, 'base_footprint')
                publisher.publish(command)
                time.sleep(0.05)
            self.destroy_publisher(publisher)

    def wait_until_stopped(self, timeout=5.0):
        """Require fresh stationary odometry for 0.5 s before every new goal."""
        started = time.monotonic()
        stationary_since = None
        while time.monotonic() - started < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            fresh = (self.odom_received_at is not None
                     and self.odom_received_at >= started
                     and now - self.odom_received_at < 0.3)
            if fresh and self.odom_stopped:
                if stationary_since is None:
                    stationary_since = now
                if now - stationary_since >= 0.5:
                    return True
            else:
                stationary_since = None
        return False

    def verify_controllers(self, modes):
        """Reject old/stock configurations before sending any motion goal."""
        client = self.create_client(GetParameters, 'controller_server/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('controller_server 파라미터 서비스를 찾을 수 없습니다.')
            names = ['controller_plugins', 'goal_checker_plugins',
                     'position_goal_checker.plugin', 'position_goal_checker.xy_goal_tolerance',
                     'goal_checker.yaw_goal_tolerance']
            for mode in sorted(modes):
                controller = CONTROLLERS[mode]
                names.extend([f'{controller}.min_vel_x', f'{controller}.max_vel_x'])
            check_position_approach = (getattr(self, 'frame_prefix', '') in ('burger1/', 'burger2/')
                                      and getattr(self, 'nav2_position_then_yaw', False))
            if check_position_approach:
                names.extend(['FollowPositionForward.critics',
                              'FollowPositionForward.waffle_navigation::PositionApproach.scale'])
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                raise RuntimeError('컨트롤러 설정 확인 시간이 초과되었습니다.')
            values = future.result().values
            if len(values) != len(names):
                raise RuntimeError('컨트롤러 파라미터 응답이 올바르지 않습니다.')
            if check_position_approach and (
                    'waffle_navigation::PositionApproach' not in values[-2].string_array_value
                    or not math.isclose(values[-1].double_value, 1000., abs_tol=1e-9)):
                raise RuntimeError('좌표 접근 조향/저속 보호 설정이 없습니다. Nav2를 다시 준비하세요.')
            plugins = values[0].string_array_value
            if ('position_goal_checker' not in values[1].string_array_value
                    or values[2].string_value != 'nav2_controller::PositionGoalChecker'
                    or any(value.type != 3 or not math.isfinite(value.double_value)
                           or value.double_value <= 0 for value in values[3:5])):
                raise RuntimeError('위치 도달/방향 정렬 분리 설정이 없습니다. '
                                   '프로젝트 설정으로 Nav2를 다시 실행하세요.')
            self.xy_tolerance, self.yaw_tolerance = [value.double_value for value in values[3:5]]
            if self.arrival_tuning and (
                    not math.isclose(self.xy_tolerance, self.arrival_tuning['xy_goal_tolerance'], abs_tol=1e-9)
                    or not math.isclose(self.yaw_tolerance, self.arrival_tuning['yaw_goal_tolerance'], abs_tol=1e-9)):
                raise RuntimeError('실행 중 Nav2 도착 공차가 waypoints.yaml 설정과 다릅니다. Nav2를 다시 실행하세요.')
            for index, mode in enumerate(sorted(modes)):
                low, high = values[5 + 2 * index:7 + 2 * index]
                bounds = (low.double_value, high.double_value)
                valid = (low.type == 3 and high.type == 3
                         and all(math.isfinite(value) for value in bounds)
                         and ((mode == 'forward' and 0.0 <= bounds[0] < bounds[1])
                              or (mode == 'reverse' and bounds[0] < bounds[1] <= 0.0)))
                if CONTROLLERS[mode] not in plugins or not valid:
                    raise RuntimeError(
                        f'{mode} 컨트롤러 설정이 없습니다. 프로젝트의 '
                        'nav2_burger_params.yaml로 Nav2를 다시 실행하세요.')
        finally:
            self.destroy_client(client)

    def align_for_travel(self, waypoint, deadline, index):
        # Translation has a real minimum wheel-driving speed. Turn while stopped
        # first, rather than forcing a forward/reverse arc around a nearby goal.
        for attempt in range(3):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            distance, angle = travel_heading_error(
                waypoint, self.current_map_pose(timeout=min(2.0, remaining)))
            if distance <= self.xy_tolerance or abs(angle) <= math.radians(8):
                return True
            if attempt == 2:
                self.error('출발 방향 정렬이 수렴하지 않아 이동을 중단합니다.')
                return False
            self.info(f'이동 전 정지 상태에서 진행 방향 정렬: {math.degrees(angle):.1f}°')
            if not self.spin(spin_dist=angle, time_allowance=int(min(30, max(1, remaining)))):
                return False
            if not wait_for_task(self, deadline, index):
                return False
            if not self.wait_until_stopped(timeout=min(5.0, max(0.0, deadline-time.monotonic()))):
                return False
        return False

    def verify_precision_controller(self):
        client = self.create_client(GetParameters, 'controller_server/get_parameters')
        names = ['FollowPositionForward.critics', 'precision_goal_checker.plugin',
                 'precision_goal_checker.stateful', 'precision_goal_checker.xy_goal_tolerance',
                 'precision_goal_checker.yaw_goal_tolerance',
                 'FollowPositionForward.waffle_navigation::PrecisionPose.scale']
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('정밀 접근 설정 확인 실패')
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(self, future, timeout_sec=3)
            if not future.done() or future.result() is None:
                raise RuntimeError('정밀 접근 설정 응답 없음')
            v = future.result().values
            if (len(v) != 6 or 'waffle_navigation::PrecisionPose' not in v[0].string_array_value
                    or v[1].string_value != 'nav2_controller::SimpleGoalChecker'
                    or v[2].type != 1 or v[2].bool_value
                    or not math.isclose(v[3].double_value, .02, abs_tol=1e-9)
                    or not math.isclose(v[4].double_value, math.radians(3), abs_tol=1e-9)
                    or not math.isclose(v[5].double_value, 1000., abs_tol=1e-9)):
                raise RuntimeError('5cm 정밀 접근 컨트롤러/엄격한 XY·yaw 판정 설정 불일치')
        finally:
            self.destroy_client(client)

    def cancel_guarded_translation(self, for_handoff=False):
        """Request cancellation and independently check stop, with bounded waits."""
        canceled = False
        try:
            pending = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, pending, timeout_sec=3.0)
            if pending.done() and pending.result() is not None:
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    if self.isTaskComplete():
                        canceled = True
                        break
            if not canceled:
                self.error('주행 취소 완료를 확인하지 못했습니다. 수동 정지가 필요합니다.')
        except Exception as exc:
            self.error(f'주행 취소 요청 실패: {exc}. 수동 정지가 필요합니다.')
        stopped = self.wait_until_stopped(timeout=3.0)
        if not stopped:
            self.error('최신 odom에서 정지를 확인하지 못했습니다. 수동 정지가 필요합니다.')
        if canceled and stopped:
            if for_handoff:
                if self.getResult() not in (TaskResult.CANCELED, TaskResult.SUCCEEDED):
                    self.error('접근 전환 중 기존 주행 실패: 다음 동작을 시작하지 않습니다.')
                    return False
                self.info('접근 전환: 기존 주행 종료 및 정지 확인.')
            else:
                self.info('목표 접근 보호: 주행 종료 및 정지 확인. 자동 재출발하지 않습니다.')
        return canceled and stopped

    def verify_terminal_behavior(self):
        """Check the actual behavior pipeline before any route motion."""
        self._verify_departure_pipeline()
        client = self.create_client(GetParameters, 'behavior_server/get_parameters')
        try:
            names = ['behavior_plugins', 'drive_on_heading.plugin', 'local_frame',
                     'robot_base_frame', 'simulate_ahead_time']
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('마지막 접근 behavior_server 설정을 확인할 수 없습니다.')
            pending = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(self, pending, timeout_sec=3.0)
            if not pending.done() or pending.result() is None:
                raise RuntimeError('마지막 접근 behavior_server 설정 응답 시간 초과.')
            v = pending.result().values
            if (len(v) != len(names) or 'drive_on_heading' not in v[0].string_array_value
                    or v[1].string_value != 'nav2_behaviors::DriveOnHeading'
                    or v[2].string_value != frame_name(self, 'map')
                    or v[3].string_value not in (frame_name(self, 'base_footprint'),
                                                frame_name(self, 'base_link'))
                    or v[4].type != 3 or not math.isfinite(v[4].double_value)
                    or v[4].double_value < 2.0):
                raise RuntimeError('마지막 접근의 충돌 검사/좌표계 설정이 예상과 다릅니다.')
            for action in (self.spin_client, self.drive_on_heading_client):
                if not action.wait_for_server(timeout_sec=3.0):
                    raise RuntimeError('마지막 접근 Spin/DriveOnHeading 서버가 없습니다.')
        finally:
            self.destroy_client(client)

    def current_map_pose(self, timeout=2.0):
        """Read a fresh map pose; never substitute origin or stale localization."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.tf_buffer.lookup_transform(frame_name(self, 'map'), frame_name(self, 'base_footprint'), rclpy.time.Time())
            except TransformException:
                continue
            age = (self.get_clock().now().nanoseconds
                   - rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
            if not -0.1 <= age <= 0.5:
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            values = (p.x, p.y, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(value) for value in values):
                continue
            if abs(sum(value * value for value in values[2:]) - 1.0) > 0.01:
                continue
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            return p.x, p.y, yaw
        raise RuntimeError('최신 map → base_footprint 위치를 확인하지 못했습니다.')


def get_quaternion_from_yaw(yaw_degrees):
    yaw_radians = math.radians(yaw_degrees)
    return tf_transformations.quaternion_from_euler(0, 0, yaw_radians)


def load_waypoints(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError('waypoints 목록이 있는 YAML 파일이 필요합니다.')
    if not data['waypoints']:
        raise ValueError('waypoints가 비어 있습니다. 실제 지도 좌표를 먼저 입력하세요.')
    for index, waypoint in enumerate(data['waypoints'], start=1):
        if (not isinstance(waypoint, dict)
                or not {'x', 'y', 'yaw'} <= set(waypoint)
                or set(waypoint) - {'x', 'y', 'yaw', 'mode'}):
            raise ValueError(f'{index}번 좌표는 x, y, yaw와 선택 항목 mode만 허용합니다.')
        mode = waypoint.get('mode', 'forward')
        if not isinstance(mode, str) or mode not in CONTROLLERS:
            raise ValueError(f'{index}번 mode는 forward 또는 reverse여야 합니다.')
        for key in ('x', 'y', 'yaw'):
            value = waypoint[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{index}번 좌표는 유한한 숫자여야 합니다.')
        waypoint['mode'] = mode
    return data['waypoints']


def travel_heading_error(waypoint, pose):
    """Heading for travel, independent of the requested final parking yaw."""
    dx, dy = waypoint['x']-pose[0], waypoint['y']-pose[1]
    desired = math.atan2(dy, dx)
    if waypoint.get('mode', 'forward') == 'reverse':
        desired += math.pi
    return math.hypot(dx, dy), math.atan2(math.sin(desired-pose[2]), math.cos(desired-pose[2]))


def make_goal_pose_list(nav, waypoints):
    goal_pose_list = []
    for waypoint in waypoints:
        goal_yaw = waypoint['yaw']
        q = get_quaternion_from_yaw(goal_yaw)

        # 강의의 PoseStamped 생성 및 append 순서를 유지합니다.
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_name(nav, 'map')
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(waypoint['x'])
        goal_pose.pose.position.y = float(waypoint['y'])
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        goal_pose_list.append(goal_pose)
    return goal_pose_list


def cancel_and_wait(nav):
    nav.cancelTask()
    while not nav.isTaskComplete():
        time.sleep(0.05)


def report_failure(nav, index):
    result = nav.getResult()
    print(f'Waypoint {index}: {result.name}. 경로를 중단합니다.')
    future = getattr(nav, 'result_future', None)
    response = future.result() if future is not None and future.done() else None
    if response is not None:
        details = response.result
        code = getattr(details, 'error_code', None)
        message = getattr(details, 'error_msg', '')
        print(f'Nav2 error_code={code}, error_msg={message or "(메시지 없음)"}')
    print('상세 원인은 Nav2 실행 터미널의 planner_server/controller_server 로그를 확인하세요.')


def wait_for_task(nav, deadline, index, waypoint=None, terminal_handoff=False,
                  intermediate_handoff=False, planned_route=False):
    guard = ApproachOvershootGuard() if waypoint is not None else None
    start_pose = None
    path_guard = PlannedPathGuard() if planned_route and guard is not None else None
    goal_started = time.monotonic()
    next_progress_log = goal_started
    while not nav.isTaskComplete():
        if time.monotonic() >= deadline:
            print('전체 경로 제한 시간 초과: 현재 동작 취소', flush=True)
            if waypoint is not None:
                nav.cancel_guarded_translation()
            else:
                cancel_and_wait(nav)
            return False
        if guard is not None:
            try:
                pose = nav.current_map_pose(timeout=0.15)
                distance = math.hypot(pose[0] - waypoint['x'], pose[1] - waypoint['y'])
                overshot = guard.update(distance, time.monotonic())
            except (RuntimeError, ValueError) as exc:
                nav.error(f'Waypoint {index}: 목표 접근 감시 위치 확인 실패: {exc}')
                nav.cancel_guarded_translation()
                return False
            if overshot:
                nav.error(f'Waypoint {index}: 목표 지나침 감지 — 최단 거리 '
                          f'{guard.best_distance:.3f} m → 현재 {distance:.3f} m. '
                          '주행을 취소하고 경로를 중단합니다.')
                nav.cancel_guarded_translation()
                return False
            if start_pose is None:
                start_pose = pose
            if path_guard is not None:
                now = time.monotonic()
                offset = nav.current_plan_offset(waypoint, pose, goal_started)
                if path_guard.update(offset, now):
                    nav.error(f'Waypoint {index}: 실제 계획 경로에서 {offset:.3f} m '
                              '이탈한 상태가 1초 지속되어 주행을 중단합니다.')
                    nav.cancel_guarded_translation()
                    return False
                if now >= next_progress_log:
                    path_text = f'{offset:.3f} m' if offset is not None else '계획 갱신 대기'
                    nav.info(f'Waypoint {index}: 이동 위치 x={pose[0]:.3f}, '
                             f'y={pose[1]:.3f}, yaw={math.degrees(pose[2]):.1f}°, '
                             f'목표 거리 {distance:.3f} m, 계획 경로 오차 {path_text}')
                    next_progress_log = now+2.0
            elif missed_target_axis(start_pose, waypoint, pose, distance):
                nav.error(f'Waypoint {index}: 직선 이동 축을 벗어났으며 '
                          f'목표 거리 {distance:.3f} m입니다. 주행을 중단합니다.')
                nav.cancel_guarded_translation()
                return False
            # A pass-through point needs a confirmed stop, not Nav2 chasing the
            # final 2 cm goal until it crosses the point and triggers recovery.
            if intermediate_handoff and distance <= 0.04:
                if not nav.cancel_guarded_translation(for_handoff=True):
                    return False
                return INTERMEDIATE_HANDOFF
            # This is a planned stage transition, not recovery after a fault.
            # The overshoot/fresh-pose checks above always take precedence.
            if terminal_handoff and distance <= getattr(
                    nav, 'terminal_approach_distance', TERMINAL_APPROACH_DISTANCE):
                if not nav.cancel_guarded_translation(for_handoff=True):
                    return False
                return TERMINAL_HANDOFF
        time.sleep(0.05)
    if nav.getResult() != TaskResult.SUCCEEDED:
        report_failure(nav, index)
        return False
    return True


def terminal_heading_tolerance(distance, xy_tolerance):
    # Reserve half the XY budget for lateral error. At the observed 6.9 cm
    # handoff, 6.4 degrees projects to <8 mm; another Spin shifted localization.
    # The geometric bound tightens automatically for longer approaches.
    return min(math.radians(7.0), math.asin(min(1.0, xy_tolerance * 0.5 / distance)))


def stable_map_pose(nav, deadline):
    """Require a stationary, fresh map pose for 0.5 s, bounded to four seconds."""
    end = min(deadline, time.monotonic() + 4.0)
    remaining = end - time.monotonic()
    if remaining <= 0 or not nav.wait_until_stopped(timeout=min(2.0, remaining)):
        raise RuntimeError('최종 접근: 정지 상태를 확인하지 못했습니다.')
    samples = []
    while time.monotonic() < end:
        pose = nav.current_map_pose(timeout=min(.5, end-time.monotonic()))
        now = time.monotonic()
        samples.append((now, pose))
        samples = [sample for sample in samples if now-sample[0] <= .65]
        if samples and now-samples[0][0] >= .5:
            if all(math.hypot(p[0]-q[0], p[1]-q[1]) <= .008
                   and abs(math.atan2(math.sin(p[2]-q[2]),
                                      math.cos(p[2]-q[2]))) <= math.radians(2)
                   for _, p in samples for _, q in samples):
                nav.info(f'정지 위치 안정 확인: map=({pose[0]:.4f}, {pose[1]:.4f}, '
                         f'{math.degrees(pose[2]):.2f}°)')
                return pose
        spin_for(nav, .05)
    raise RuntimeError('최종 접근: 지도 위치가 안정되지 않았습니다 (0.5초/8mm/2°).')


def final_approach_geometry(waypoint, pose):
    dx, dy = waypoint['x']-pose[0], waypoint['y']-pose[1]
    along = dx*math.cos(pose[2]) + dy*math.sin(pose[2])
    lateral = -dx*math.sin(pose[2]) + dy*math.cos(pose[2])
    yaw_error = math.radians(waypoint['yaw'])-pose[2]
    return along, lateral, math.atan2(math.sin(yaw_error), math.cos(yaw_error))


def staged_final_approach(nav, waypoint, deadline, index, tree):
    """Align away from the final point, then approach forward without a final spin."""
    if waypoint.get('mode', 'forward') != 'forward':
        raise RuntimeError('최종 진입점 접근은 전진 경로만 지원합니다.')
    yaw = math.radians(waypoint['yaw'])
    stage = dict(waypoint, x=waypoint['x']-nav.final_staging_distance*math.cos(yaw),
                 y=waypoint['y']-nav.final_staging_distance*math.sin(yaw))
    nav.info(f'Waypoint {index}: 최종 진입점 x={stage["x"]:.3f}, y={stage["y"]:.3f}; '
             f'목표 앞 {nav.final_staging_distance:.2f} m에서 도착 방향 정렬')
    if not nav.align_for_travel(stage, deadline, index):
        return False
    pose = make_goal_pose_list(nav, [stage])[0]
    if not nav.goToPose(pose, behavior_tree=str(tree)):
        return False
    if not wait_for_task(nav, deadline, index, waypoint=stage, planned_route=True):
        return False
    current = stable_map_pose(nav, deadline)
    yaw_tolerance = getattr(nav, 'final_yaw_tolerance', None) or nav.yaw_tolerance
    # Tighten heading before the straight approach; never chase the bearing of
    # a goal only 2-3 cm away, which caused the recorded -54/-14/+79 deg spins.
    for correction in range(2):
        if math.hypot(current[0]-stage['x'], current[1]-stage['y']) > .05:
            nav.error('최종 진입점에서 5cm 이상 벗어났습니다. 접근을 중단합니다.')
            return False
        _, _, error = final_approach_geometry(waypoint, current)
        if abs(error) <= min(yaw_tolerance, math.radians(1.5)):
            break
        remaining = deadline-time.monotonic()
        if remaining < 2 or not nav.spin(spin_dist=error,
                                         time_allowance=int(min(30, remaining))):
            return False
        if not wait_for_task(nav, deadline, index):
            return False
        current = stable_map_pose(nav, deadline)
    along, lateral, error = final_approach_geometry(waypoint, current)
    nav.info(f'최종 전진 진입: 전방 {along:.3f} m, 횡오차 {lateral:.3f} m, '
             f'방향 오차 {math.degrees(error):.1f}°')
    if (abs(error) > yaw_tolerance or abs(lateral) > nav.xy_tolerance*.6
            or not .03 <= along <= nav.final_staging_distance+.05):
        nav.error('최종 전진 진입 조건 불충족: 가까운 목표를 향해 재회전하지 않고 중단합니다.')
        return False
    travel = along - nav.xy_tolerance*.3
    allowance = min(15, int(deadline-time.monotonic()))
    if allowance < travel/TERMINAL_SPEED+2:
        return False
    nav.info(f'최종 방향 유지 전진 {travel:.3f} m, {TERMINAL_SPEED:.3f} m/s (충돌 검사 활성)')
    if not nav.driveOnHeading(dist=travel, speed=TERMINAL_SPEED, time_allowance=allowance):
        return False
    if not wait_for_task(nav, min(deadline, time.monotonic()+allowance), index, waypoint=waypoint):
        return False
    current = stable_map_pose(nav, deadline)
    distance, error = pose_errors(waypoint, current)
    nav.info(f'최종 정지 검증: x={current[0]:.4f}, y={current[1]:.4f}, '
             f'yaw={math.degrees(current[2]):.2f}°, 위치 오차 {distance:.4f} m '
             f'(허용 {nav.xy_tolerance:.3f}), 방향 오차 {math.degrees(error):.2f}° '
             f'(허용 {math.degrees(yaw_tolerance):.1f}°)')
    if distance > nav.xy_tolerance or abs(error) > yaw_tolerance:
        nav.error('최종 정지 검증 실패: 재회전/재출발 없이 중단합니다.')
        return False
    return True


def terminal_approach(nav, waypoint, deadline, index):
    """One collision-checked straight approach after a confirmed stop.

    No planner retry or automatic recovery on failure. Re-read the map pose
    after each Spin; the displacement action is not itself an arrival verdict.
    """
    for attempt in range(3):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not nav.wait_until_stopped(timeout=min(5.0, remaining)):
            return False
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        current_pose = nav.current_map_pose(timeout=min(2.0, remaining))
        distance, angle = travel_heading_error(waypoint, current_pose)
        nav.info(f'Waypoint {index}: 마지막 접근 지도 위치 '
                 f'x={current_pose[0]:.3f}, y={current_pose[1]:.3f}, '
                 f'yaw={math.degrees(current_pose[2]):.1f}°, '
                 f'남은 거리 {distance:.3f} m, 목표 방향 {math.degrees(angle):.1f}°')
        if distance > TERMINAL_MAX_DISTANCE:
            nav.error('마지막 접근 전 위치가 15cm 범위를 벗어났습니다. 경로를 중단합니다.')
            return False
        if distance <= nav.xy_tolerance:
            return True
        if abs(angle) <= terminal_heading_tolerance(distance, nav.xy_tolerance):
            break
        if attempt == 2:
            nav.error('마지막 접근 방향 정렬이 수렴하지 않았습니다. 경로를 중단합니다.')
            return False
        remaining = deadline - time.monotonic()
        if remaining < 1.0:
            return False
        nav.info(f'Waypoint {index}: 마지막 접근 전 제자리 정렬 {math.degrees(angle):.1f}°')
        if not nav.spin(spin_dist=angle, time_allowance=int(min(30.0, remaining))):
            return False
        if not wait_for_task(nav, deadline, index):
            return False
    # Aim half a tolerance short of the center, leaving room for stopping.
    # DriveOnHeading requires both distance AND speed to have the same sign.
    travel = distance - nav.xy_tolerance * 0.5
    sign = -1.0 if waypoint.get('mode', 'forward') == 'reverse' else 1.0
    allowance = min(12, int(deadline - time.monotonic()))
    if allowance < travel / TERMINAL_SPEED + 2.0:
        nav.error('마지막 접근에 필요한 시간이 부족합니다.')
        return False
    nav.info(f'Waypoint {index}: 마지막 직선 접근 {sign*travel:.3f} m, '
             f'{sign*TERMINAL_SPEED:.3f} m/s (충돌 검사 활성)')
    if not nav.driveOnHeading(dist=sign*travel, speed=sign*TERMINAL_SPEED,
                              time_allowance=allowance):
        return False
    return wait_for_task(nav, min(deadline, time.monotonic()+allowance), index,
                         waypoint=waypoint)


def pose_errors(waypoint, pose):
    x, y, yaw = pose
    distance = math.hypot(x - waypoint['x'], y - waypoint['y'])
    delta = math.radians(waypoint['yaw']) - yaw
    return distance, math.atan2(math.sin(delta), math.cos(delta))


def finish_waypoint(nav, waypoint, deadline, index, is_final=True,
                    require_yaw=True):
    nav.position_retry_requested = False
    nav.position_retry_before_yaw = False
    yaw_tolerance = getattr(nav, 'final_yaw_tolerance', None) or nav.yaw_tolerance
    # Recheck the latest map pose after the robot has stopped.
    # Always reach the requested translation tolerance before turning. Only a
    # completed intermediate Spin may use the post-turn transition allowance.
    # At centimeter distances, chasing AMCL shifts can reverse the target bearing
    # and trigger repeated large turns without improving physical positioning.
    xy_tolerance = (nav.intermediate_xy_tolerance
                    if not is_final and not require_yaw else nav.xy_tolerance)
    remaining = deadline - time.monotonic()
    if remaining <= 0 or not nav.wait_until_stopped(timeout=min(5.0, remaining)):
        print('위치 도달 후 정지를 확인하지 못했습니다. 방향 정렬을 시작하지 않습니다.')
        return False
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return False
    distance, angle = pose_errors(waypoint, nav.current_map_pose(timeout=min(2.0, remaining)))
    nav.info(f'Waypoint {index}: 위치 오차 {distance:.3f} m '
             f'(정지 후 허용 {xy_tolerance:.3f} m), 방향 오차 {math.degrees(angle):.1f}°')
    if distance > xy_tolerance:
        nav.position_retry_requested = distance <= 0.10
        nav.position_retry_before_yaw = distance <= 0.08
        nav.error('정지 후 위치 허용오차 초과: 제한된 위치 재보정 가능 여부를 확인합니다.')
        return False
    if not require_yaw:
        nav.info(f'Waypoint {index}: 중간 지점 도착; 다음 구간 시작 전에 진행 방향을 맞춥니다.')
        return True
    corrections = 2 if getattr(nav, 'final_yaw_tolerance', None) else 1
    spun = False
    for correction in range(corrections):
        if abs(angle) <= yaw_tolerance:
            break
        if distance > xy_tolerance:
            nav.error('방향 보정 전 위치 허용오차를 벗어났습니다.')
            return False
        remaining = deadline - time.monotonic()
        if remaining < 1.0:
            print('방향 정렬에 필요한 시간이 남아 있지 않습니다.')
            return False
        print(f'Waypoint {index}: 제자리 방향 정렬 {math.degrees(angle):.1f}° '
              f'({correction + 1}/{corrections})', flush=True)
        # Spin uses Nav2's collision checker and publishes zero linear velocity.
        if not nav.spin(spin_dist=angle, time_allowance=int(min(30.0, remaining))):
            print('방향 정렬 요청이 거절되었습니다.')
            return False
        if not wait_for_task(nav, deadline, index):
            return False
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not nav.wait_until_stopped(timeout=min(5.0, remaining)):
            print('방향 정렬 후 정지를 확인하지 못했습니다.')
            return False
        spun = True
        if not is_final:
            xy_tolerance = nav.intermediate_xy_tolerance
        elif getattr(nav, "final_post_turn_xy_tolerance", None) is not None:
            xy_tolerance = max(nav.xy_tolerance, nav.final_post_turn_xy_tolerance)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        distance, angle = pose_errors(waypoint, nav.current_map_pose(timeout=min(2.0, remaining)))
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return False
    if not spun:
        distance, angle = pose_errors(waypoint, nav.current_map_pose(timeout=min(2.0, remaining)))
    nav.info(f'Waypoint {index}: 최종 위치 오차 {distance:.3f} m '
             f'(허용 {xy_tolerance:.3f} m), 방향 오차 {math.degrees(angle):.1f}° '
             f'(허용 {math.degrees(yaw_tolerance):.1f}°)')
    if distance > xy_tolerance or abs(angle) > yaw_tolerance:
        exceeded = []
        if distance > xy_tolerance:
            exceeded.append('위치')
        if abs(angle) > yaw_tolerance:
            exceeded.append('방향')
        nav.position_retry_requested = xy_tolerance < distance <= 0.10
        nav.error(f'최종 {"/".join(exceeded)} 허용오차 초과: '
                  + ('제한된 위치 재보정을 검토합니다.' if nav.position_retry_requested
                     else '경로를 중단합니다.'))
        return False
    if distance > nav.xy_tolerance:
        nav.info('회전 후 위치 허용오차 내에서 도착을 확인했습니다.' if is_final else
                 '중간 지점의 회전 후 전환 허용오차 내입니다. 다음 웨이포인트로 진행합니다.')
    return True


def run_waypoints(nav, waypoints, timeout, tree_dir=None,
                  pre_backup_distance=0.0, pre_backup_speed=0.05,
                  pre_turn_angle_deg=0.0, pre_backup_open_loop=False,
                  continuous_intermediate=False):
    if tree_dir is None:
        tree_dir = Path(get_package_share_directory('waffle_navigation')) / 'behavior_trees'
    trees = {mode: Path(tree_dir) / f'navigate_{mode}.xml'
             for mode in {point.get('mode', 'forward') for point in waypoints}}
    precision = getattr(nav, 'nav2_precision_pose', False)
    if precision:
        if set(trees) != {'forward'}:
            raise RuntimeError('정밀 접근은 전진 경로 전용입니다.')
        trees['forward'] = Path(tree_dir) / 'navigate_precision_forward.xml'
    for tree in trees.values():
        if not tree.is_file():
            raise RuntimeError(f'주행 동작 파일이 없습니다. 패키지를 빌드하세요: {tree}')
    print('[Nav2] 초기 위치 확인: 저장된 시작 위치 자동 적용 / 미등록 시 RViz에서 지정', flush=True)
    nav.waitUntilNav2Active()
    nav.verify_controllers(set(trees))
    nav.verify_terminal_behavior()
    if precision:
        nav.verify_precision_controller()
    if not precision:
        nav.info(f'도착 판정: Nav2 이동 {nav.xy_tolerance:.3f} m, '
                 f'회전 전 정지 {nav.xy_tolerance:.3f} m, '
                 f'중간 지점 회전 후 전환 {nav.intermediate_xy_tolerance:.3f} m, '
                 f'마지막 지점 {nav.xy_tolerance:.3f} m, '
                 f'방향 {math.degrees(getattr(nav, "final_yaw_tolerance", None) or nav.yaw_tolerance):.1f}°')
    if precision:
        nav.info('각 지점: Nav2 주행 → 5cm 이내 감속·XY/yaw 동시 평가 → 저속 정렬. '
                 '모든 지점에서 XY 2cm 및 yaw 3° 검증; 별도 도착 Spin 없음.')
    elif getattr(nav, 'nav2_position_then_yaw', False):
        nav.info('각 지점: Nav2 전진 피드백 주행 → 좌표 도착·정지 → 목표 yaw 회전. '
                 '목표까지 계획 경로 기준 접근; 최종 좌표/방향 검증 유지.')
    elif getattr(nav, 'final_staging_distance', 0.0):
        nav.info(f'최종 진입점 {nav.final_staging_distance:.2f} m: 방향 먼저 정렬, '
                 f'{TERMINAL_SPEED:.2f} m/s 전진 후 정지 검증, 도착 후 회전 없음.')
    else:
        nav.info(f'마지막 접근: 목표 {getattr(nav, "terminal_approach_distance", TERMINAL_APPROACH_DISTANCE):.2f} m 이내에서 '
                 f'정지·방향 정렬 후 {TERMINAL_SPEED:.2f} m/s 직선 접근. 실패 시 재출발 없음.')
    goal_pose_list = make_goal_pose_list(nav, waypoints)
    nav_start = time.monotonic()
    deadline = nav_start + timeout
    if pre_backup_distance > 0:
        if pre_backup_open_loop:
            nav.pre_backup_timed(pre_backup_distance, pre_backup_speed, deadline)
        else:
            nav.pre_backup(pre_backup_distance, pre_backup_speed, deadline)
    if pre_turn_angle_deg:
        remaining = deadline - time.monotonic()
        if remaining < 1.0 or not nav.wait_until_stopped(timeout=min(5.0, remaining)):
            raise RuntimeError('180도 회전 전 정지 상태를 확인하지 못했습니다.')
        print(f'[출차] 제자리 회전 {pre_turn_angle_deg:g}°', flush=True)
        remaining = deadline - time.monotonic()
        if remaining < 1.0 or not nav.spin(
                spin_dist=math.radians(pre_turn_angle_deg),
                time_allowance=int(min(60.0, remaining))):
            raise RuntimeError('출차 후 제자리 회전 요청이 거절되었습니다.')
        if not wait_for_task(nav, deadline, '출차 회전'):
            raise RuntimeError('출차 후 제자리 회전을 완료하지 못했습니다.')
        if not nav.wait_until_stopped(timeout=min(5.0, max(0.0, deadline-time.monotonic()))):
            raise RuntimeError('180도 회전 후 정지 상태를 확인하지 못했습니다.')
    for index, (waypoint, pose) in enumerate(zip(waypoints, goal_pose_list), start=1):
        remaining = timeout - (time.monotonic() - nav_start)
        if remaining <= 0:
            print(f'{timeout:g}초 초과: 다음 지점을 시작하지 않습니다.')
            return 1
        if not nav.wait_until_stopped(timeout=min(5.0, remaining)):
            print('최신 /odom에서 정지 상태를 확인하지 못했습니다. 주행을 중단합니다.')
            return 1
        if time.monotonic() - nav_start >= timeout:
            print(f'{timeout:g}초 초과: 다음 지점을 시작하지 않습니다.')
            return 1
        mode = waypoint.get('mode', 'forward')
        if precision:
            pose.header.stamp = nav.get_clock().now().to_msg()
            nav.info(f'Waypoint {index}/{len(waypoints)}: Nav2 5cm 정밀 접근 시작')
            if not nav.goToPose(pose, behavior_tree=str(trees[mode])):
                return 1
            if not wait_for_task(nav, deadline, index, waypoint=waypoint, planned_route=True):
                return 1
            settled = stable_map_pose(nav, deadline)
            distance = math.hypot(settled[0]-waypoint['x'], settled[1]-waypoint['y'])
            yaw = math.atan2(math.sin(math.radians(waypoint['yaw'])-settled[2]),
                             math.cos(math.radians(waypoint['yaw'])-settled[2]))
            nav.info(f'정밀 도착 검증: XY {distance:.3f} m, yaw {math.degrees(yaw):.2f}°')
            if distance > .02 or abs(yaw) > math.radians(3):
                nav.error('정지 후 위치/방향 허용오차 초과: 재출발 없이 중단합니다.')
                return 1
            continue
        if getattr(nav, 'nav2_position_then_yaw', False):
            # At most one explicit, collision-checked correction after position
            # success/stop, before any final yaw Spin. Never retry a failed action
            # or redispatch after yaw alignment or excessive localization error.
            retries = getattr(nav, 'position_arrival_retries', 0)
            for attempt in range(retries + 1):
                if time.monotonic() >= deadline:
                    return 1
                # A forward-only DWB can choose a stationary trajectory when the
                # next waypoint lies behind the robot. Align using collision-
                # checked Spin only for large heading changes, never blind yaw.
                if getattr(nav, 'align_large_heading_before_navigation', False):
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        return 1
                    distance, heading = travel_heading_error(
                        waypoint, nav.current_map_pose(timeout=min(2.0, remaining)))
                    if distance > nav.xy_tolerance and abs(heading) > math.radians(45):
                        if not nav.wait_until_stopped(timeout=min(5.0, remaining)):
                            nav.error('구간 시작 방향 정렬 전 정지를 확인하지 못했습니다.')
                            return 1
                        nav.info(f'구간 시작 방향 차이 {math.degrees(heading):.1f}°: '
                                 '정지 상태에서 방향 정렬 후 전진합니다.')
                        if not nav.align_for_travel(waypoint, deadline, index):
                            nav.error('구간 시작 방향 정렬 실패: 이동을 시작하지 않습니다.')
                            return 1
                pose.header.stamp = nav.get_clock().now().to_msg()
                nav.info(f'Waypoint {index}/{len(waypoints)}: Nav2 좌표 이동 시작 '
                         f'({attempt + 1}/{retries + 1})')
                if not nav.goToPose(pose, behavior_tree=str(trees[mode])):
                    return 1
                if not wait_for_task(nav, deadline, index, waypoint=waypoint, planned_route=True):
                    return 1
                if finish_waypoint(nav, waypoint, deadline, index,
                                   is_final=index == len(waypoints), require_yaw=True):
                    break
                if (attempt >= retries or
                        not getattr(nav, 'position_retry_before_yaw', False)):
                    nav.error('좌표 도착/목표 방향 검증 실패: 다음 지점으로 진행하지 않습니다.')
                    return 1
                if not nav.wait_until_stopped(timeout=min(5.0, max(0.0, deadline-time.monotonic()))):
                    nav.error('위치 재보정 전 정지를 확인하지 못했습니다.')
                    return 1
                nav.info('정지 후 위치 오차 8cm 이내: 회전 전에 Nav2로 한 번 재접근합니다.')
            continue
        if index == len(waypoints) and getattr(nav, 'final_staging_distance', 0.0) > 0:
            if not staged_final_approach(nav, waypoint, deadline, index, trees[mode]):
                return 1
            continue
        # At most two additional Nav2 attempts. Never relax the tolerance,
        # bypass collision checking, or change the requested travel direction.
        for attempt in range(3):
            if time.monotonic() >= deadline:
                return 1
            if continuous_intermediate and index > 1:
                # This route has a near-right-angle corner. A forward-only
                # controller should start the next leg facing that leg, rather
                # than steering a wide arc away from the intended corridor.
                if not nav.align_for_travel(waypoint, deadline, index):
                    return 1
                previous = waypoints[index - 2]
                current = nav.current_map_pose(timeout=2.0)
                corner_error = math.hypot(current[0] - previous['x'],
                                          current[1] - previous['y'])
                if corner_error > 0.10:
                    nav.error(f'Waypoint {index}: 회전 후 {index - 1}번 지점에서 '
                              f'{corner_error:.3f} m 벗어나 주행을 중단합니다.')
                    return 1
            elif not nav.align_for_travel(waypoint, deadline, index):
                return 1
            pose.header.stamp = nav.get_clock().now().to_msg()
            print(f'Waypoint {index}/{len(waypoints)}: {mode} 위치 이동 (시도 {attempt + 1}/3)', flush=True)
            if not nav.goToPose(pose, behavior_tree=str(trees[mode])):
                print('Waypoint goal was rejected!')
                return 1
            outcome = wait_for_task(nav, deadline, index, waypoint=waypoint, planned_route=True,
                                    terminal_handoff=not continuous_intermediate
                                    or index == len(waypoints),
                                    intermediate_handoff=continuous_intermediate
                                    and index < len(waypoints))
            if not outcome:
                return 1
            used_terminal = outcome is TERMINAL_HANDOFF
            used_intermediate = outcome is INTERMEDIATE_HANDOFF
            if used_terminal and not terminal_approach(nav, waypoint, deadline, index):
                nav.error('마지막 접근 실패: 재출발 없이 경로를 중단합니다.')
                return 1
            if finish_waypoint(nav, waypoint, deadline, index,
                               is_final=index == len(waypoints),
                               require_yaw=not continuous_intermediate
                               or index == len(waypoints)):
                break
            if used_terminal:
                nav.error('마지막 접근 후 도착 검증 실패: 재출발 없이 경로를 중단합니다.')
                return 1
            if used_intermediate:
                nav.error('중간 지점 정지 후 허용오차 초과: 자동으로 되돌아가지 않고 중단합니다.')
                return 1
            if not getattr(nav, 'position_retry_requested', False) or attempt == 2:
                nav.error('위치 재보정 불가 또는 최대 횟수 도달: 경로를 중단합니다.')
                return 1
            if not nav.wait_until_stopped(timeout=min(5.0, max(0.0, deadline-time.monotonic()))):
                return 1
            nav.info('최신 지도 위치 기준으로 같은 목표에 다시 접근합니다.')
    print('Goal succeeded!')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='', help='Robot namespace and TF frame prefix')
    parser.add_argument('--waypoints', default=str(Path(
        get_package_share_directory('waffle_navigation')) / 'config/waypoints.yaml'))
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='전체 경로 제한 시간(초), 기본 300')
    parser.add_argument('--dry-run', action='store_true', help='좌표만 검증하고 출력')
    parser.add_argument('--pre-backup-distance', type=float, default=0.10,
                        help='첫 웨이포인트 전 직선 후진 거리(m), 기본 0.10; 0이면 생략')
    parser.add_argument('--pre-backup-speed', type=float, default=0.05,
                        help='출차 후진 최대 속도(m/s), 기본 0.05')
    parser.add_argument('--pre-backup-open-loop', action='store_true',
                        help='출차 후진을 scan/odom 없이 속도×시간 명령으로 실행(거리 실측 없음)')
    parser.add_argument('--pre-turn-angle-deg', type=float, default=0.0,
                        help='출차 후 Nav2 Spin 제자리 회전 각도(도), 기본 0')
    parser.add_argument('--final-post-turn-xy-tolerance', type=float,
                        help='최종 지점 회전 완료 후 위치 허용반경(m); 이동·회전 전 기준은 유지')
    parser.add_argument('--align-large-heading-before-navigation', action='store_true',
                        help='다음 좌표 진행 방향 차이가 45도 초과이면 정지 회전 후 전진')
    parser.add_argument('--position-arrival-retries', type=int, choices=(0, 1), default=0,
                        help='좌표 도착 후 정지 오차 8cm 이내일 때 회전 전 Nav2 재접근 횟수 (0 또는 1)')
    parser.add_argument('--final-yaw-tolerance-deg', type=float,
                        help='최종 방향 허용오차(도); 생략 시 Nav2 설정 사용')
    parser.add_argument('--terminal-approach-distance', type=float,
                        default=TERMINAL_APPROACH_DISTANCE,
                        help='Nav2에서 마지막 직선 접근으로 전환하는 거리(m), 기본 0.10')
    parser.add_argument('--continuous-intermediate', action='store_true',
                        help='중간 지점 정지 후 다음 구간의 전진/후진 방향에 맞춰 회전')
    parser.add_argument('--nav2-position-then-yaw', action='store_true',
                        help='각 좌표까지 Nav2 주행 후 정지하고 목표 yaw 회전; 이동 전 회전/별도 접근 없음')
    parser.add_argument('--nav2-precision-pose', action='store_true',
                        help='5cm 이내 Nav2 저속 XY/yaw 동시 보정; 모든 지점 2cm/3도 검증')
    parser.add_argument('--final-staging-distance', type=float, default=0.0,
                        help='최종 방향을 먼저 맞출 진입점 거리(m), 0이면 기존 접근; 전진 전용')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.final_post_turn_xy_tolerance is not None and (
            not args.nav2_position_then_yaw or
            not math.isfinite(args.final_post_turn_xy_tolerance) or
            not 0 < args.final_post_turn_xy_tolerance <= .05):
        parser.error('--final-post-turn-xy-tolerance는 좌표 도착 후 정렬 모드에서 0 초과 0.05 m 이하로 지정하세요.')
    if args.nav2_precision_pose and (args.nav2_position_then_yaw or args.continuous_intermediate or args.final_staging_distance):
        parser.error('정밀 접근은 다른 접근/정렬 모드와 함께 사용할 수 없습니다.')
    if args.nav2_position_then_yaw and (args.continuous_intermediate or args.final_staging_distance):
        parser.error('--nav2-position-then-yaw는 연속 중간점/최종 진입점 모드와 함께 사용할 수 없습니다.')
    if not math.isfinite(args.final_staging_distance) or not (
            args.final_staging_distance == 0 or .12 <= args.final_staging_distance <= .25):
        parser.error('--final-staging-distance는 0 또는 0.12~0.25 m여야 합니다.')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout은 0보다 큰 유한한 숫자여야 합니다.')
    if not math.isfinite(args.pre_backup_distance) or not 0 <= args.pre_backup_distance <= 0.5:
        parser.error('--pre-backup-distance는 0~0.5 m여야 합니다.')
    if not math.isfinite(args.pre_backup_speed) or not 0.001 <= args.pre_backup_speed <= 0.05:
        parser.error('--pre-backup-speed는 0.001~0.05 m/s여야 합니다.')
    if not math.isfinite(args.pre_turn_angle_deg) or not -180 <= args.pre_turn_angle_deg <= 180:
        parser.error('--pre-turn-angle-deg는 -180~180도여야 합니다.')
    if args.final_yaw_tolerance_deg is not None and (
            not math.isfinite(args.final_yaw_tolerance_deg)
            or not 0 < args.final_yaw_tolerance_deg <= 180):
        parser.error('--final-yaw-tolerance-deg는 0보다 크고 180 이하여야 합니다.')
    if not math.isfinite(args.terminal_approach_distance) or not (
            0.04 <= args.terminal_approach_distance <= TERMINAL_APPROACH_DISTANCE):
        parser.error('--terminal-approach-distance는 0.04~0.10 m여야 합니다.')
    try:
        helper_path = Path(get_package_share_directory('waffle_navigation')) / 'config/arrival_tuning.py'
        spec = importlib.util.spec_from_file_location('arrival_tuning', helper_path)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        tuning = helper.load_tuning(args.waypoints)
        if (args.final_post_turn_xy_tolerance is not None and
                args.final_post_turn_xy_tolerance < tuning.get('xy_goal_tolerance', 0.05)):
            parser.error('최종 회전 후 위치 허용반경은 이동 도착 기준 이상이어야 합니다.')
        waypoints = load_waypoints(args.waypoints)
        if args.final_staging_distance and waypoints[-1]['mode'] != 'forward':
            parser.error('최종 진입점 접근은 전진 경로만 지원합니다.')
        print(f'도착 튜닝: {tuning or "기존 Nav2 설정 사용"}')
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    for index, waypoint in enumerate(waypoints, start=1):
        print(f'{index}: x={waypoint["x"]}, y={waypoint["y"]}, '
              f'yaw={waypoint["yaw"]} deg, mode={waypoint["mode"]}')
    print(f'출차 후진: {args.pre_backup_distance:.2f} m, 최대 {args.pre_backup_speed:.2f} m/s '
          '(거리 0이면 생략)', flush=True)
    if args.pre_backup_open_loop:
        print('출차 후진 방식: 시간제어(라이다/odom 피드백 없이, 실제 거리 오차 가능)', flush=True)
    if args.pre_turn_angle_deg:
        print(f'출차 후 제자리 회전: {args.pre_turn_angle_deg:g}°', flush=True)
    if args.final_yaw_tolerance_deg is not None:
        print(f'최종 방향 목표 허용오차: {args.final_yaw_tolerance_deg:g}°', flush=True)
    if args.continuous_intermediate:
        print('중간 지점: 정지 확인 후 다음 구간의 지정 주행 방향으로 회전', flush=True)
    if args.nav2_precision_pose:
        print('주행 순서: 각 좌표까지 Nav2 전진 피드백 주행 → 5cm 이내 저속 XY/yaw 동시 보정', flush=True)
    elif args.nav2_position_then_yaw:
        print('주행 순서: 각 좌표까지 Nav2 전진 피드백 주행 → 정지 → 해당 목표 yaw 회전', flush=True)
    elif args.final_staging_distance:
        last = waypoints[-1]
        angle = math.radians(last['yaw'])
        print(f'최종 진입점: x={last["x"]-args.final_staging_distance*math.cos(angle):.4f}, '
              f'y={last["y"]-args.final_staging_distance*math.sin(angle):.4f}; '
              f'도착 방향 정렬 후 {TERMINAL_SPEED:.2f} m/s 전진, 도착 후 회전 없음')
    else:
        print(f'마지막 접근 전환: {args.terminal_approach_distance:.2f} m, '
              f'직선 속도 {TERMINAL_SPEED:.2f} m/s', flush=True)
    if args.final_post_turn_xy_tolerance is not None:
        print(f'최종 회전 후 위치 허용반경: {args.final_post_turn_xy_tolerance:.3f} m (회전 전 도착 기준 유지)', flush=True)
    print(f'좌표 도착 후 회전 전 재접근: 최대 {args.position_arrival_retries}회 (위치 오차 8cm 이내)')
    if args.dry_run:
        return 0

    # Ctrl+C 시 ROS context를 먼저 닫지 않고 액션 취소를 요청합니다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = WaypointNavigator(namespace=args.namespace)
    nav.final_yaw_tolerance = (math.radians(args.final_yaw_tolerance_deg)
                               if args.final_yaw_tolerance_deg is not None else None)
    nav.align_large_heading_before_navigation = args.align_large_heading_before_navigation
    nav.position_arrival_retries = args.position_arrival_retries
    nav.final_post_turn_xy_tolerance = args.final_post_turn_xy_tolerance
    nav.terminal_approach_distance = args.terminal_approach_distance
    nav.final_staging_distance = args.final_staging_distance
    nav.nav2_position_then_yaw = args.nav2_position_then_yaw
    nav.nav2_precision_pose = args.nav2_precision_pose
    nav.arrival_tuning = tuning
    nav.intermediate_xy_tolerance = tuning.get('intermediate_xy_tolerance', INTERMEDIATE_XY_TOLERANCE)
    try:
        return run_waypoints(nav, waypoints, args.timeout,
                             pre_backup_distance=args.pre_backup_distance,
                             pre_backup_speed=args.pre_backup_speed,
                             pre_turn_angle_deg=args.pre_turn_angle_deg,
                             pre_backup_open_loop=args.pre_backup_open_loop,
                             continuous_intermediate=args.continuous_intermediate)
    except KeyboardInterrupt:
        print('사용자 중단: 주행 취소 요청', flush=True)
        cancel_and_wait(nav)
        return 130
    except RuntimeError as exc:
        print(f'Nav2 실행 오류: {exc}')
        cancel_and_wait(nav)
        return 1
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
