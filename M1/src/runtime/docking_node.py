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
from rclpy.qos import qos_profile_sensor_data
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
    def __init__(self, config, gpio, execute=False, auto_start=False, vision_queue=None, recorder=None):
        super().__init__('docking_controller', namespace='/burger1')
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
        self.last_diagnostic_log = float('-inf')
        self.last_vision_error = None
        self.start_error = 'inputs_not_ready'
        self.sequence = 0
        self.physical_high = None
        self.motor_torque = self.motor_time = self.motor_request = None
        self.motor_client = self.create_client(SetBool, config['motor_power_service']) if execute else None
        self.create_subscription(SensorState, config['motor_state_topic'], self.motor_state, qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            TwistStamped, config['cmd_topic'] if execute else '/burger1/docking/preview_cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/burger1/docking/status', 1)
        self.ir_pub = self.create_publisher(Bool, '/burger1/ir/high', 1)
        self.create_subscription(Odometry, config['odom_topic'], self.odometry, qos_profile_sensor_data)
        self.create_service(Trigger, '/burger1/docking/start', self.start)
        self.create_service(Trigger, '/burger1/docking/stop', self.stop)
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
            vision = self.cfg['vision']
            expected_size = (vision.get('image_width', 640), vision.get('image_height', 480))
            if o.get('frame_id') != self.cfg['camera_frame'] or (o.get('image_width'), o.get('image_height')) != expected_size:
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
            if self.recorder:
                self.recorder.record('observation', {
                    'source_stamp_ns': ns, 'image_age_s': age,
                    'pose_valid': o['pose_valid'], 'reason': o.get('reason'),
                    'detected_ids': o.get('detected_ids'), 'tvec_m': o.get('tvec_m'),
                    'normal_yaw_rad': o.get('normal_yaw_rad'),
                    'reprojection_rms_px': o.get('reprojection_rms_px'),
                    'detection_ms': self.detection_ms})
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
        if not self.execute:
            return None
        pubs = self.get_publishers_info_by_topic(self.cfg['cmd_topic'])
        if len(pubs) != 1 or pubs[0].node_name != self.get_name() or pubs[0].node_namespace != self.get_namespace():
            return 'cmd_vel_has_other_publisher_or_graph_not_ready'
        subs = self.get_subscriptions_info_by_topic(self.cfg['cmd_topic'])
        if not subs or any(s.topic_type != 'geometry_msgs/msg/TwistStamped' for s in subs):
            return 'TwistStamped_motor_subscriber_required'
        return None

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
            self.read_ir()
            error = self.graph_check()
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
        if self.recorder:
            self.recorder.record('command', {'linear_mps': float(v), 'angular_rps': float(w),
                                            'execute': self.execute})

    def tick(self):
        now = time.monotonic()
        previous = self.control.state
        try:
            high = self.read_ir()
            if high and self.control.state == 'IDLE':
                self.control.halt('ir_already_high', fault=False)
            self.ir_pub.publish(Bool(data=high))
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
        if now-self.last_graph_check >= .2:
            self.graph_error = self.graph_check()
            self.last_graph_check = now
        if self.graph_error and self.control.state in self.control.ACTIVE:
            self.control.halt(self.graph_error)
        # GPIO was sampled after tick entry; compare freshness against a later time.
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
        if now-self.last_status >= .2 or self.last_state != self.control.state:
            self.sequence += 1
            pose = self.control.pose()
            report = {'state': self.control.state, 'reason': self.control.reason,
                      'execute': self.execute, 'ir_high': self.physical_high,
                      'docking_mode': self.cfg.get('docking_mode', 'normal'),
                      'target_ids': self.target_ids,
                      'linear_mps': v, 'angular_rps': w,
                      'travel_m': self.control.distance, 'final_travel_m': self.control.final_distance,
                      'graph_error': self.graph_error, 'vision_error': self.last_vision_error,
                      'vision_location': 'robot', 'image_age_s': self.image_age_s,
                      'detection_ms': self.detection_ms,
                      'motor_torque': self.motor_torque, 'motor_error': motor_error,
                      'lateral_error_m': pose[1] if pose is not None else None,
                      'yaw_error_deg': math.degrees(pose[2]) if pose is not None else None,
                      'camera_distance_m': pose[3] if pose is not None else None,
                      'observation_reason': (self.control.observation or {}).get('reason'),
                      'observation_age_now_s': (now-self.control.observation_time
                                               if self.control.observation_time is not None else None),
                      'odom_age_s': now-self.control.odom_time if self.control.odom_time is not None else None,
                      'odom_linear_mps': self.control.odom[3] if self.control.odom else None,
                      'odom_angular_rps': self.control.odom[4] if self.control.odom else None,
                      'aligned_hold_elapsed_s': (now-self.control.aligned_since
                                                 if self.control.aligned_since is not None else None),
                      'vision_recoveries': self.control.vision_recoveries,
                      'vision_wait_elapsed_s': (now-self.control.vision_wait_at
                          if self.control.state == 'VISION_WAIT' else None),
                      'vision_workers': self.vision_health() if self.vision_health else None,
                      'log_dropped_records': self.recorder.dropped if self.recorder else 0,
                      'log_writer_error': self.recorder.error if self.recorder else None,
                      'sequence': self.sequence}
            if self.recorder:
                self.recorder.record('status', report)
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
    parser.add_argument('--log-dir', help='Robot-local diagnostic run directory')
    args = parser.parse_args()
    try:
        config = load_docking_config(args.config, args.mode, args.board, args.calibration)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    Settings(**config['control'])  # Validate before acquiring hardware.
    gpio = node = vision = recorder = None
    exit_reason = 'unexpected_exit'
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        if args.log_dir:
            from docking_recorder import DockingRecorder
            recorder = DockingRecorder(args.log_dir, config, [
                args.config, config['calibration_path'], config['board_path'],
                __file__, Path(__file__).with_name('docking_control.py'),
                Path(__file__).with_name('camera_node.py'),
                Path(__file__).with_name('docking_vision_worker.py')])
        if args.execute:
            # Discover existing owners before creating any motor publisher.
            probe = Node('docking_preflight', namespace='/burger1')
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
        node = DockingNode(config, gpio, args.execute, args.auto_start, vision.channel, recorder)
        node.vision_health = vision.health
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if args.exit_on_result and node.control.state in ('DOCKED', 'STOPPED', 'FAULT'):
                exit_reason = node.control.state + ': ' + node.control.reason
                return 0 if node.control.state == 'DOCKED' else 1
        exit_reason = 'ros_shutdown'
    except KeyboardInterrupt:
        exit_reason = 'interrupted'
        return 130
    except Exception as exc:
        exit_reason = f'{type(exc).__name__}: {exc}'
        raise
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
        if recorder is not None:
            try:
                recorder.close(exit_reason)
            except Exception as exc:
                print(f'Diagnostic finalization failed: {exc}', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
