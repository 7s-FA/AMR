#!/usr/bin/env python3
"""Standalone robot-local docking controller with direct GPIO stop."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from communication_guard import GraphGuard
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from turtlebot3_msgs.msg import SensorState

from docking_vision.docking_config import add_target_arguments, load_docking_config
from docking_control import DockingControl, Settings
from ir_sensor import GPIOInput


class DockingNode(Node):
    def __init__(self, config, gpio, execute=False, auto_start=False, vision_queue=None, recorder=None, graph_guard=None):
        super().__init__('docking_controller', namespace='/burger2')
        self.cfg, self.gpio, self.execute = config, gpio, execute
        self.target_ids = sorted(marker['id'] for marker in config['board_spec']['markers'])
        self.vision_queue = vision_queue
        self.recorder = recorder
        self.vision_health = None
        self.image_age_s = self.detection_ms = None
        self.control = DockingControl(Settings(**config['control']))
        self.last_capture_ns = -1
        self.auto_start = auto_start
        self.auto_start_deadline = time.monotonic()+15.0
        self.last_graph_check = self.last_status = self.last_publish = float('-inf')
        self.graph_error = 'graph_not_checked'
        self.last_state = None
        self.last_diagnostic_log = float("-inf")
        self.last_vision_error = None
        self.start_error = 'inputs_not_ready'
        self.sequence = 0
        self.physical_high = None
        self.motor_torque = self.motor_time = self.motor_request = None
        self.motor_client = self.create_client(SetBool, config['motor_power_service']) if execute else None
        latest_sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.graph_guard = graph_guard
        self.owns_graph_guard = graph_guard is None
        self.last_graph_snapshot = {}
        self.ir_sample_to_publish_ms = None
        self.control_tick_ms = None
        self.create_subscription(SensorState, config['motor_state_topic'], self.motor_state, latest_sensor_qos)
        self.publisher = self.create_publisher(
            TwistStamped, config['cmd_topic'] if execute else '/burger2/docking/preview_cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/burger2/docking/status', 1)
        self.ir_pub = self.create_publisher(Bool, '/burger2/ir/high', 1)
        self.create_subscription(Odometry, config['odom_topic'], self.odometry, latest_sensor_qos)
        self.create_service(Trigger, '/burger2/docking/start', self.start)
        self.create_service(Trigger, '/burger2/docking/stop', self.stop)
        if execute and self.graph_guard is None:
            self.graph_guard = GraphGuard(config['cmd_topic'], self.get_name(), self.get_namespace(), 'docking')
        # GPIO is sampled on the robot even when the host loses communication.
        self.create_timer(.01, self.tick)
        self.get_logger().info('Mode: ' + ('EXECUTE' if execute else 'DRY RUN (preview topic only)'))
        self.get_logger().info(
            f"Docking target: {config.get('docking_mode', 'normal')}; IDs: {self.target_ids}")

    def stamp_fresh(self, stamp, limit):
        ns = int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)
        age = (self.get_clock().now().nanoseconds-ns)/1e9
        return ns > 0 and -.10 <= age <= limit

    def observation(self, msg):
        now = time.monotonic()
        try:
            if len(msg.data) > 20000:
                raise ValueError('oversized observation')
            o = json.loads(msg.data)
            if not isinstance(o, dict) or o.get('status') != 'ok':
                raise ValueError('vision stopped or malformed: '+str(o.get('error', '') if isinstance(o, dict) else ''))
            if o.get('mode') != 'four_marker_board' or o.get('board_spec') != self.cfg['board_spec']:
                raise ValueError('wrong board configuration')
            if o.get('frame_id') != self.cfg['camera_frame'] or (o.get('image_width'), o.get('image_height')) != (640, 480):
                raise ValueError('wrong camera frame or resolution')
            stamp = o['source_stamp']
            ns = int(stamp['sec'])*1_000_000_000+int(stamp['nanosec'])
            age = (self.get_clock().now().nanoseconds-ns)/1e9
            self.image_age_s, self.detection_ms = age, o.get('detection_ms')
            # Source stamp came from THIS robot's camera, not the host clock.
            if ns <= self.last_capture_ns or not -.1 <= age <= self.cfg['max_image_age_s']:
                raise ValueError('stale/replayed camera frame')
            if type(o.get('pose_valid')) is not bool:
                raise ValueError('invalid pose flag')
            # After alignment, only the camera/worker heartbeat is required.
            # Near-field pose failures and changing normals must not stop the
            # already committed straight segment or steer it again.
            if o['pose_valid'] and self.control.state != 'FINAL_APPROACH':
                if sorted(o.get('detected_ids', [])) != self.target_ids:
                    raise ValueError('four unique markers required')
                values = list(o['tvec_m']) + list(o['rvec_rad']) + [o['normal_yaw_rad'], o['reprojection_rms_px']]
                if len(o['tvec_m']) != 3 or len(o['rvec_rad']) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                    raise ValueError('non-finite/invalid pose')
                if o['reprojection_rms_px'] > self.cfg['max_reprojection_rms_px']:
                    raise ValueError('reprojection error too high')
                # Differential-drive geometry assumes an upright board and level camera.
                rx, ry, rz = o['rvec_rad']
                angle = math.sqrt(rx*rx+ry*ry+rz*rz)
                if angle < 1e-8:
                    raise ValueError('board normal faces away from camera')
                ux, uy, uz = rx/angle, ry/angle, rz/angle
                normal_y = uy*uz*(1-math.cos(angle))-ux*math.sin(angle)
                normal_z = math.cos(angle)+uz*uz*(1-math.cos(angle))
                if normal_z >= -.5 or abs(normal_y) > math.sin(math.radians(20)):
                    raise ValueError('board must face camera and be upright within 20 degrees')
            self.last_capture_ns = ns
            self.control.set_observation(o, now-max(0., age))
            self.last_vision_error = None
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.last_vision_error = str(exc)
            self.control.aligned_since = None
            if self.control.state in self.control.ACTIVE and self.control.state != 'STOPPING':
                self.control.halt('vision_invalid: '+str(exc))
                self.publish_velocity(0., 0.)

    def odometry(self, msg):
        if not self.stamp_fresh(msg.header.stamp, self.control.cfg.odom_timeout_s):
            return
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if not math.isfinite(norm) or abs(norm-1) > .05:
            self.control.halt('invalid_odometry_quaternion')
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.control.set_odom(p.x, p.y, yaw, twist.linear.x, twist.angular.z, time.monotonic())

    def graph_check(self):
        if self.graph_guard is None:
            return None
        self.last_graph_snapshot = self.graph_guard.snapshot()
        return self.last_graph_snapshot['error']

    def destroy_node(self):
        if self.graph_guard is not None and self.owns_graph_guard:
            self.graph_guard.close()
            self.graph_guard = None
        return super().destroy_node()

    def read_ir(self):
        self.physical_high = self.gpio.high()
        self.control.set_ir(self.physical_high, time.monotonic())
        return self.physical_high

    def motor_state(self, msg):
        if self.stamp_fresh(msg.header.stamp, .5):
            self.motor_torque, self.motor_time = bool(msg.torque), time.monotonic()

    def motor_error(self, now):
        if not self.execute:
            return None
        if not self.control.fresh(self.motor_time, now, .5):
            return 'motor_state_timeout'
        return None if self.motor_torque else 'motor_torque_off'

    def prepare_start(self, now):
        error = self.control.start_error(now)
        if error:
            return False, error
        error = self.motor_error(now)
        if error == 'motor_torque_off':
            # Only an explicit start may enable torque. Never re-enable during motion.
            self.publish_velocity(0., 0.)
            if self.motor_request is None:
                if not self.motor_client.service_is_ready():
                    return False, 'motor_power_service_unavailable'
                self.motor_request = self.motor_client.call_async(SetBool.Request(data=True))
            if self.motor_request.done():
                try:
                    result = self.motor_request.result()
                    if not result.success:
                        return False, 'motor_power_enable_failed: '+result.message
                except Exception as exc:
                    return False, 'motor_power_enable_failed: '+str(exc)
            return False, 'motor_power_enabling'
        if error:
            return False, error
        return self.control.start(now)

    def start(self, _, response):
        try:
            error = self.graph_check()
            self.read_ir()
            if error:
                response.success, response.message = False, error
            else:
                if self.control.state not in self.control.ACTIVE and not self.auto_start:
                    self.motor_request = None
                response.success, response.message = self.prepare_start(time.monotonic())
                if response.success:
                    self.auto_start = False
                    self.publish_velocity(0., 0.)
                elif response.message == 'motor_power_enabling':
                    self.control.state, self.control.reason = 'IDLE', 'motor_power_enabling'
                    self.auto_start = True
                    self.auto_start_deadline = time.monotonic()+15.
                    response.success = True  # Accepted; status reports completion/failure.
        except Exception as exc:
            self.control.halt('gpio_read_failed')
            self.publish_velocity(0., 0.)
            response.success, response.message = False, str(exc)
        return response

    def stop(self, _, response):
        self.control.halt('operator_stop', fault=False)
        self.auto_start = False
        self.publish_velocity(0., 0.)
        response.success, response.message = True, 'Stopped; restart requires a new start service call.'
        return response

    def publish_velocity(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.publisher.publish(msg)
        if self.control.ir_time is not None:
            self.ir_sample_to_publish_ms = (time.monotonic()-self.control.ir_time)*1000

    def tick(self):
        tick_started = time.monotonic()
        now = tick_started
        previous = self.control.state
        try:
            high = self.read_ir()
            if high and self.control.state == 'IDLE':
                self.control.halt('ir_already_high', fault=False)
            self.ir_pub.publish(Bool(data=high))
            if high:
                self.publish_velocity(0., 0.)
        except Exception as exc:
            self.control.halt('gpio_read_failed: '+str(exc))
            self.physical_high = None
        if self.vision_queue is not None:
            try:
                observation = self.vision_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                self.observation(String(data=json.dumps(observation, allow_nan=False)))
        # Only a bounded shared-memory read; ROS graph calls run elsewhere.
        self.graph_error = self.graph_check()
        if self.graph_error and self.control.state in self.control.ACTIVE:
            self.control.halt(self.graph_error)
        # Re-sample after observation processing, immediately before control.
        try:
            self.read_ir()
        except Exception as exc:
            self.control.halt('gpio_read_failed: '+str(exc))
            self.physical_high = None
        now = time.monotonic()
        motor_error = self.motor_error(now)
        if motor_error and self.control.state in self.control.ACTIVE:
            self.control.halt(motor_error)
        if self.auto_start:
            if self.control.state != 'IDLE':
                self.auto_start = False
            elif now >= self.auto_start_deadline:
                self.control.halt('auto_start_timeout: '+(self.graph_error or self.start_error))
                self.auto_start = False
            elif not self.graph_error:
                started, self.start_error = self.prepare_start(now)
                if started:
                    self.auto_start = False
                else:
                    self.control.reason = 'start_wait: '+self.start_error
        v, w = self.control.tick(now)
        if self.physical_high is not False or self.graph_error:
            v = w = 0.
        # Immediate zero on GPIO HIGH/state change; normal command heartbeat 20Hz.
        if now-self.last_publish >= .05 or previous != self.control.state or self.physical_high is not False:
            self.publish_velocity(v, w)
            self.last_publish = now
        self.control_tick_ms = (time.monotonic()-tick_started)*1000
        if now-self.last_status >= .2 or self.last_state != self.control.state:
            self.sequence += 1
            report = {'state': self.control.state, 'reason': self.control.reason,
                      'execute': self.execute, 'ir_high': self.physical_high,
                      'docking_mode': self.cfg.get('docking_mode', 'normal'),
                      'target_ids': self.target_ids,
                      'linear_mps': v, 'angular_rps': w,
                      'travel_m': self.control.distance, 'final_travel_m': self.control.final_distance,
                      'graph_error': self.graph_error, 'graph_monitor': self.last_graph_snapshot,
                      'ir_sample_to_publish_ms': self.ir_sample_to_publish_ms, 'control_tick_ms': self.control_tick_ms, 'vision_error': self.last_vision_error,
                      'vision_location': 'robot', 'image_age_s': self.image_age_s,
                      'detection_ms': self.detection_ms,
                      'observation_age_now_s': None if self.control.observation_time is None else now-self.control.observation_time,
                      'odom_age_s': now-self.control.odom_time if self.control.odom_time is not None else None,
                      'vision_recoveries': self.control.vision_recoveries,
                      'vision_wait_elapsed_s': (now-self.control.vision_wait_at
                          if self.control.state == 'VISION_WAIT' else None),
                      'vision_workers': self.vision_health() if self.vision_health else None,
                      'motor_torque': self.motor_torque, 'motor_error': motor_error,
                      'sequence': self.sequence}
            pose = self.control.pose()
            report.update(lateral_error_m=None if pose is None else pose[1],
                          yaw_error_deg=None if pose is None else math.degrees(pose[2]),
                          optical_distance_m=None if pose is None else pose[3],
                          observation_reason=(self.control.observation or {}).get('reason'))
            self.status_pub.publish(String(data=json.dumps(report, allow_nan=False)))
            if self.last_state != self.control.state or now-self.last_diagnostic_log >= 1.0:
                self.get_logger().info(json.dumps(report))
                self.last_diagnostic_log = now
            self.last_state, self.last_status = self.control.state, now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('docking.yaml')))
    add_target_arguments(parser)
    parser.add_argument('--execute', action='store_true', help='Publish to actual motor topic; default is preview only')
    parser.add_argument('--auto-start', action='store_true', help='Start once inputs are ready within 15 seconds')
    parser.add_argument('--exit-on-result', action='store_true', help='Exit after DOCKED, STOPPED or FAULT')
    args = parser.parse_args()
    try:
        config = load_docking_config(args.config, args.mode, args.board, args.calibration)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    Settings(**config['control'])  # Validate before acquiring hardware.
    gpio = node = vision = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        if args.execute:
            # Discover existing owners before creating any motor publisher.
            probe = Node('docking_preflight', namespace='/burger2')
            try:
                deadline = time.monotonic()+1.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(probe, timeout_sec=.05)
                if probe.count_publishers(config['cmd_topic']):
                    raise RuntimeError('Motor topic already has a publisher; stop the previous driving process first.')
            finally:
                probe.destroy_node()
        gpio = GPIOInput(config['gpio_chip'], config['gpio_pin'])
        from docking_vision_worker import LocalVision
        vision = LocalVision(config)
        node = DockingNode(config, gpio, args.execute, args.auto_start, vision.channel)
        node.vision_health = vision.health
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if args.exit_on_result and node.control.state in ('DOCKED', 'STOPPED', 'FAULT'):
                return 0 if node.control.state == 'DOCKED' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            for _ in range(3):
                node.publish_velocity(0., 0.)
                time.sleep(.02)
            node.destroy_node()
        if gpio is not None:
            gpio.close()
        if vision is not None:
            vision.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
