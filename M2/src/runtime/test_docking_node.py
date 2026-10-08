"""Run with ROS_DOMAIN_ID=232; all velocity output goes to an isolated test topic."""
import copy
import json
import math
from pathlib import Path
import time
import queue

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from turtlebot3_msgs.msg import SensorState
import yaml

from docking_node import DockingNode
from docking_vision.docking_config import load_docking_config


class FakeGPIO:
    value = False
    torque = True
    motor_requests = 0

    def high(self):
        return self.value


def test_diagnostics_record_control_and_freshness_without_motion_changes(rig, tmp_path):
    from docking_recorder import DockingRecorder
    node, _, _, _, pump, _ = rig
    recorder = DockingRecorder(tmp_path, node.cfg)
    node.recorder = recorder
    try:
        pump(.3)
    finally:
        node.recorder = None
        recorder.close('test')
    events = [json.loads(line) for line in (tmp_path / 'events.jsonl').read_text().splitlines()]
    assert {'status', 'command', 'observation'} <= {e['kind'] for e in events}
    status = next(e for e in events if e['kind'] == 'status')
    assert status['observation_age_now_s'] is not None
    assert status['odom_angular_rps'] == 0.
    assert status['log_writer_error'] is None


@pytest.fixture
def rig(request):
    config = load_docking_config(Path(__file__).with_name('docking.yaml'),
                                 getattr(request, 'param', 'normal'))
    config['cmd_topic'] = '/docking_test/cmd_vel'
    config['odom_topic'] = '/docking_test/odom'
    config['motor_state_topic'] = '/docking_test/sensor_state'
    config['motor_power_service'] = '/docking_test/motor_power'
    config['control']['aligned_hold_s'] = .05
    config['control']['stopped_hold_s'] = .03
    rclpy.init(args=[])
    io, gpio = Node('docking_test_io'), FakeGPIO()
    local_frames = queue.Queue(maxsize=1)
    node = DockingNode(config, gpio, execute=True, vision_queue=local_frames)
    commands = []
    sub = io.create_subscription(TwistStamped, config['cmd_topic'], lambda m: commands.append((m.twist.linear.x, m.twist.angular.z)), 50)
    odom = io.create_publisher(Odometry, config['odom_topic'], 1)
    motor = io.create_publisher(SensorState, config['motor_state_topic'], 1)
    def power(request, response):
        gpio.motor_requests += 1
        gpio.torque = request.data
        response.success = True
        return response
    io.create_service(SetBool, config['motor_power_service'], power)

    def frame(valid=True, age=0.):
        ns = io.get_clock().now().nanoseconds-int(age*1e9)
        return {'status': 'ok', 'mode': 'four_marker_board', 'board_spec': config['board_spec'],
                'frame_id': config['camera_frame'],
                'image_width': config['vision']['image_width'], 'image_height': config['vision']['image_height'],
                'source_stamp': {'sec': ns//10**9, 'nanosec': ns % 10**9},
                'pose_valid': valid, 'detected_ids': config['target_ids'] if valid else [],
                'tvec_m': [0., 0., .25] if valid else None,
                'rvec_rad': [math.pi, 0., 0.] if valid else None,
                'normal_yaw_rad': 0. if valid else None, 'reprojection_rms_px': .1,
                'reason': 'aligned_observation' if valid else 'markers_missing'}

    def pump(seconds=.1, valid=True):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if not local_frames.empty():
                local_frames.get_nowait()
            local_frames.put_nowait(frame(valid))
            msg = Odometry()
            msg.header.stamp = io.get_clock().now().to_msg()
            msg.pose.pose.orientation.w = 1.
            odom.publish(msg)
            sensor = SensorState()
            sensor.header.stamp = io.get_clock().now().to_msg()
            sensor.torque = gpio.torque
            motor.publish(sensor)
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=.002)
                rclpy.spin_once(io, timeout_sec=.002)

    try:
        pump(.5)
        yield node, io, gpio, commands, pump, frame
    finally:
        io.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_ros_start_missing_markers_ir_stop_and_latch(rig):
    node, io, gpio, commands, pump, _ = rig
    client = io.create_client(Trigger, '/burger2/docking/start')
    assert client.wait_for_service(timeout_sec=1)
    future = client.call_async(Trigger.Request())
    pump(.3)
    assert future.done() and future.result().success, future.result()
    assert node.control.state == 'FINAL_APPROACH'
    pump(.1, valid=False)
    assert commands[-1] == (node.control.cfg.final_speed_mps, 0.)
    gpio.value = True
    pump(.15, valid=False)
    assert node.control.state == 'DOCKED'
    assert commands[-1] == (0., 0.)
    gpio.value = False
    pump(.1, valid=False)
    assert commands[-1] == (0., 0.) and node.control.state == 'DOCKED'


def test_stale_frame_and_wrong_board_stop_active_controller(rig):
    node, _, _, _, pump, frame = rig
    assert node.control.start(time.monotonic())[0]
    node.observation(String(data=json.dumps(frame(age=2))))
    assert node.control.state == 'FAULT' and 'stale' in node.control.reason
    pump(.1)
    assert node.control.start(time.monotonic())[0]
    wrong = copy.deepcopy(frame())
    wrong['board_spec']['markers'][0]['length_m'] = .05
    node.observation(String(data=json.dumps(wrong)))
    assert node.control.state == 'FAULT' and 'configuration' in node.control.reason


def test_other_velocity_publisher_blocks_start(rig):
    node, io, _, _, pump, _ = rig
    other = io.create_publisher(TwistStamped, '/docking_test/cmd_vel', 1)
    pump(.2)
    response = node.start(Trigger.Request(), Trigger.Response())
    assert not response.success and 'other_publisher' in response.message
    io.destroy_publisher(other)


def test_host_observations_are_not_control_inputs(rig):
    node, io, _, _, pump, _ = rig
    assert node.control.start(time.monotonic())[0]
    host = io.create_publisher(String, '/burger2/docking/observation', 1)
    for _ in range(5):
        host.publish(String(data='{"status":"stopped"}'))
        pump(.03)
    assert node.control.state in ('ALIGN', 'FINAL_APPROACH')
    assert io.count_subscribers('/burger2/docking/observation') == 0
    assert node.last_vision_error is None


def test_local_vision_failure_stops_even_with_host_frames(rig):
    node, io, _, _, pump, frame = rig
    assert node.control.start(time.monotonic())[0]
    node.vision_queue = None  # Simulate stalled local worker.
    host = io.create_publisher(String, '/burger2/docking/observation', 1)
    for _ in range(8):
        host.publish(String(data=json.dumps(frame())))
        pump(.1)
    assert node.control.state == 'FAULT'
    assert node.control.reason == 'vision_timeout'


def test_explicit_start_enables_torque_then_confirms_feedback(rig):
    node, io, gpio, commands, pump, _ = rig
    gpio.torque = False
    pump(.1)
    response = node.start(Trigger.Request(), Trigger.Response())
    assert response.success and response.message == 'motor_power_enabling'
    assert node.control.state == 'IDLE'
    assert node.control.last_command == (0., 0.)
    pump(.3)
    assert gpio.motor_requests == 1
    assert node.motor_torque and node.control.state == 'FINAL_APPROACH'
    assert commands[-1] == (node.control.cfg.final_speed_mps, 0.)


def test_torque_loss_during_motion_stops_without_reenable(rig):
    node, _, gpio, commands, pump, _ = rig
    assert node.control.start(time.monotonic())[0]
    pump(.1)
    gpio.torque = False
    pump(.15)
    assert node.control.state == 'FAULT' and node.control.reason == 'motor_torque_off'
    assert commands[-1] == (0., 0.)
    assert gpio.motor_requests == 0


def test_ir_high_never_enables_motor(rig):
    node, _, gpio, _, pump, _ = rig
    gpio.value, gpio.torque = True, False
    pump(.1)
    response = node.start(Trigger.Request(), Trigger.Response())
    assert not response.success
    pump(.1)
    assert gpio.motor_requests == 0


@pytest.mark.parametrize('kind', ['pose_failed', 'board_geometry_inconsistent',
                                'duplicate_target_id', 'unstable_normal'])
def test_final_approach_does_not_revalidate_pose(rig, kind):
    node, _, gpio, commands, pump, frame = rig
    assert node.control.start(time.monotonic())[0]
    pump(.2)
    assert node.control.state == 'FINAL_APPROACH'
    o = frame(valid=kind == 'unstable_normal')
    if kind == 'unstable_normal':
        o['rvec_rad'] = [0., 0., 0.]  # Rejected during ALIGN, unused during FINAL.
        o['reprojection_rms_px'] = 10.
    else:
        o['reason'] = kind
    node.observation(String(data=json.dumps(o)))
    node.tick()
    assert node.last_vision_error is None
    assert node.control.state == 'FINAL_APPROACH'
    assert node.control.last_command == (node.control.cfg.final_speed_mps, 0.)
    gpio.value = True
    node.tick()
    assert node.control.state == 'STOPPING'
    assert node.control.last_command == (0., 0.)


def test_final_approach_still_rejects_stale_camera_frame(rig):
    node, _, _, _, pump, frame = rig
    assert node.control.start(time.monotonic())[0]
    pump(.2)
    assert node.control.state == 'FINAL_APPROACH'
    node.observation(String(data=json.dumps(frame(valid=False, age=2))))
    assert node.control.state == 'FAULT'
    assert 'stale' in node.control.reason


@pytest.mark.parametrize('rig', ['parking'], indirect=True)
def test_parking_accepts_own_board_and_ir_stop(rig):
    node, _, gpio, commands, pump, _ = rig
    assert node.cfg['docking_mode'] == 'parking'
    assert node.last_vision_error is None
    assert node.control.start(time.monotonic())[0]
    pump(.2)
    assert node.control.state == 'FINAL_APPROACH'
    gpio.value = True
    pump(.15, valid=False)
    assert node.control.state == 'DOCKED'
    assert commands[-1] == (0., 0.)


@pytest.mark.parametrize('rig', ['normal', 'parking'], indirect=True)
@pytest.mark.parametrize('wrong_ids', [[0, 1, 2, 3], [4, 5, 6, 7], [8, 9, 10, 11], [4, 4, 6, 7]])
def test_selected_ids_are_required_even_with_matching_board_spec(rig, wrong_ids):
    node, _, _, commands, _, frame = rig
    if wrong_ids == node.target_ids:
        return  # Own target acceptance is covered by normal/parking motion tests.
    assert node.control.start(time.monotonic())[0]
    observation = frame()
    observation['detected_ids'] = wrong_ids
    node.observation(String(data=json.dumps(observation)))
    assert node.control.state == 'FAULT'
    assert 'four unique markers' in node.control.reason
    assert node.control.last_command == (0., 0.)


def test_alignment_vision_gap_publishes_zero_then_resumes(rig):
    node, _, _, commands, pump, _ = rig
    node.control.cfg.aligned_hold_s = 10.
    assert node.control.start(time.monotonic())[0]
    saved_queue, node.vision_queue = node.vision_queue, None
    pump(.8)
    assert node.control.state == 'VISION_WAIT'
    assert commands[-1] == (0., 0.)
    assert node.control.vision_recoveries == 1
    started_at = node.control.started_at
    node.vision_queue = saved_queue
    pump(.6)
    assert node.control.state == 'ALIGN'
    assert node.control.started_at == started_at


def test_alignment_permanent_vision_gap_times_out_while_stopped(rig):
    node, _, _, commands, pump, _ = rig
    node.control.cfg.aligned_hold_s = 10.
    assert node.control.start(time.monotonic())[0]
    node.vision_queue = None
    pump(2.8)
    assert node.control.state == 'FAULT'
    assert node.control.reason == 'vision_recovery_timeout'
    assert commands[-1] == (0., 0.)
