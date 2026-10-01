"""ROS/GPIO-independent docking state machine. SI units; base x forward, y left."""
from dataclasses import dataclass, fields
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass
class Settings:
    camera_forward_m: float = 0.0
    camera_left_m: float = 0.0
    camera_yaw_deg: float = 0.0
    staging_distance_m: float = 0.25
    lateral_tolerance_m: float = 0.005
    yaw_tolerance_deg: float = 2.0
    aligned_hold_s: float = 0.7
    align_speed_mps: float = 0.04
    final_speed_mps: float = 0.03
    max_angular_rps: float = 0.25
    min_angular_rps: float = 0.03
    near_angular_rps: float = 0.16
    steering_accel_rps2: float = 0.8
    far_steering_gain: float = 1.25
    angular_speed_scale: float = 1.0
    inverse_yaw_speed: bool = False
    board_normal_tracking: bool = False
    observation_timeout_s: float = 0.5
    vision_recovery_timeout_s: float = 2.0
    vision_recovery_hold_s: float = 0.3
    vision_recovery_fresh_s: float = 0.25
    max_vision_recoveries: int = 3
    odom_timeout_s: float = 0.3
    ir_timeout_s: float = 0.1
    max_final_distance_m: float = 0.30
    max_final_time_s: float = 12.0
    max_total_distance_m: float = 1.0
    max_total_time_s: float = 90.0
    max_start_distance_m: float = 0.9
    min_visual_distance_m: float = 0.12
    stopped_hold_s: float = 0.5

    def __post_init__(self):
        for f in fields(self):
            x = getattr(self, f.name)
            if f.name in ('inverse_yaw_speed', 'board_normal_tracking'):
                if type(x) is not bool:
                    raise ValueError(f'{f.name} must be Boolean')
                continue
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                raise ValueError(f'{f.name} must be finite')
            if not f.name.startswith('camera_') and x <= 0:
                raise ValueError(f'{f.name} must be positive')
        if type(self.max_vision_recoveries) is not int:
            raise ValueError('max_vision_recoveries must be an integer')
        if not (self.vision_recovery_fresh_s <= self.observation_timeout_s
                and self.vision_recovery_hold_s < self.vision_recovery_timeout_s):
            raise ValueError('Invalid vision recovery timing')
        if not (0.01 <= self.final_speed_mps <= self.align_speed_mps <= .08):
            raise ValueError('Require 0.01 <= final speed <= align speed <= 0.08 m/s')
        if not (self.min_visual_distance_m < self.staging_distance_m < self.max_start_distance_m):
            raise ValueError('Invalid visual/staging/start distances')
        if not (self.min_angular_rps <= self.max_angular_rps <= .5):
            raise ValueError('Invalid angular speed limits')


class DockingControl:
    ACTIVE = {'ALIGN', 'VISION_WAIT', 'FINAL_APPROACH', 'STOPPING'}

    def __init__(self, settings=None):
        self.cfg = settings or Settings()
        self.state, self.reason = 'IDLE', 'waiting_for_start'
        self.ir = self.ir_time = self.odom = self.odom_time = None
        self.observation = self.observation_time = None
        self.aligned_since = self.stopped_since = None
        self.started_at = self.final_at = None
        self.distance = self.final_distance = 0.0
        self.last_command = (0.0, 0.0)
        self.vision_recoveries = 0
        self.vision_wait_at = self.vision_good_since = self.recovery_stamp = None
        self.recovery_frames = 0
        self.last_steering_time = None

    def halt(self, reason, fault=True):
        self.state = 'FAULT' if fault else 'STOPPED'
        self.reason = reason
        self.last_command = (0.0, 0.0)
        self.aligned_since = None

    def set_ir(self, high, now):
        if type(high) is not bool:
            raise ValueError('IR must be a physical Boolean level')
        self.ir, self.ir_time = high, now
        if high:
            if self.state == 'FINAL_APPROACH':
                self.state, self.reason = 'STOPPING', 'ir_high'
                self.stopped_since = None
            elif self.state in ('ALIGN', 'VISION_WAIT'):
                self.halt('ir_high_before_final_approach', fault=False)
            self.last_command = (0.0, 0.0)

    def set_odom(self, x, y, yaw, linear, angular, now):
        values = (x, y, yaw, linear, angular)
        if not all(math.isfinite(v) for v in values):
            self.halt('invalid_odometry')
            return
        if self.odom is not None and self.state in self.ACTIVE:
            step = math.hypot(x-self.odom[0], y-self.odom[1])
            if step > .10 or abs(wrap(yaw-self.odom[2])) > .35:
                self.halt('odometry_jump')
            else:
                self.distance += step
                if self.state == 'FINAL_APPROACH':
                    self.final_distance += step
        self.odom, self.odom_time = values, now

    def set_observation(self, observation, now):
        self.observation, self.observation_time = observation, now
        # A missing frame between aligned frames must break the continuous hold.
        if not observation.get('pose_valid', False):
            self.aligned_since = None
            self.vision_good_since = None
            self.recovery_frames = 0

    @staticmethod
    def fresh(stamp, now, timeout):
        return stamp is not None and 0 <= now-stamp <= timeout

    def input_error(self, now, vision=True):
        c = self.cfg
        if not self.fresh(self.ir_time, now, c.ir_timeout_s):
            return 'ir_timeout'
        if not self.fresh(self.odom_time, now, c.odom_timeout_s):
            return 'odometry_timeout'
        if vision and not self.fresh(self.observation_time, now, c.observation_timeout_s):
            return 'vision_timeout'
        return None

    def pose(self):
        o, c = self.observation, self.cfg
        if not o or not o.get('pose_valid'):
            return None
        try:
            x_right, _, z = o['tvec_m']
            phi = o['normal_yaw_rad']
            if not all(math.isfinite(v) for v in (x_right, z, phi)) or z <= 0:
                return None
            a = math.radians(c.camera_yaw_deg)
            bx = math.cos(a)*z + math.sin(a)*x_right + c.camera_forward_m
            by = math.sin(a)*z - math.cos(a)*x_right + c.camera_left_m
            return bx, by, wrap(a-phi), z
        except (TypeError, ValueError, KeyError):
            return None

    def start_error(self, now):
        if self.state in self.ACTIVE:
            return 'already_running'
        error = self.input_error(now)
        if error:
            return error
        if self.ir:
            return 'ir_already_high'
        pose = self.pose()
        if pose is None:
            return 'four_markers_required'
        if not self.cfg.min_visual_distance_m < pose[3] <= self.cfg.max_start_distance_m:
            return 'start_distance_out_of_range'
        if abs(self.odom[3]) > .01 or abs(self.odom[4]) > .04:
            return 'robot_must_be_stationary'
        return None

    def start(self, now):
        error = self.start_error(now)
        if error:
            return False, error
        self.state, self.reason = 'ALIGN', 'visual_alignment'
        self.started_at, self.final_at = now, None
        self.distance = self.final_distance = 0.0
        self.aligned_since = self.stopped_since = None
        self.vision_recoveries = 0
        self.vision_wait_at = self.vision_good_since = self.recovery_stamp = None
        self.recovery_frames = 0
        return True, self.state

    def turn(self, value, settling=False):
        # During translation, tiny polar-control residuals must not be amplified
        # into alternating minimum-speed turns. Keep minimum-speed correction
        # for the final stationary alignment: OpenCR quantizes angular commands
        # and otherwise stops correcting before the tight lateral target is met.
        if value == 0 or (not settling and abs(value) < self.cfg.min_angular_rps):
            return 0.0
        return math.copysign(max(self.cfg.min_angular_rps,
                                 min(abs(value), self.cfg.max_angular_rps)), value)

    def yaw_speed_limit(self, theta):
        # Larger board-normal yaw errors get a lower speed ceiling.
        # 15 degrees halves the ceiling; preserve the motor's minimum command.
        c = self.cfg
        return max(c.min_angular_rps,
                   c.max_angular_rps/(1.+abs(theta)/math.radians(15.)))

    def tick(self, now):
        previous = self.last_command
        command = self._tick(now)
        limit = None
        if self.cfg.inverse_yaw_speed and self.state == 'ALIGN':
            p = self.pose()
            if p is not None:
                limit = self.yaw_speed_limit(p[2]) * self.cfg.angular_speed_scale
        # Scale visual steering only; stop and straight approach remain immediate.
        command = (command[0], command[1] * self.cfg.angular_speed_scale)
        if limit is not None:
            command = (command[0], clamp(command[1], limit))
        # Stop commands and the latched straight segment are immediate.
        # Slew only valid visual steering, never extrapolate a lost pose.
        if self.state == 'ALIGN' and command != (0.0, 0.0):
            dt = .02 if self.last_steering_time is None else max(.001, min(.1, now-self.last_steering_time))
            step = self.cfg.steering_accel_rps2 * self.cfg.angular_speed_scale * dt
            command = (command[0], previous[1]+clamp(command[1]-previous[1],step))
        # A newly larger yaw error lowers the ceiling immediately, even when
        # the previous command was faster. Acceleration still uses the old slew.
        if limit is not None:
            command = (command[0], clamp(command[1], limit))
        self.last_steering_time = now
        self.last_command = command
        return command

    def track_board_normal(self, bx, by, theta, optical_z):
        """Follow the board-normal line with bounded heading, not a shrinking point."""
        c = self.cfg
        # Signed distance to the board-normal line is invariant under an in-place
        # turn. Camera-center offset alone is not: it can cross zero mid-turn.
        cross_track = by*math.cos(theta)-bx*math.sin(theta)
        limit = c.near_angular_rps
        if abs(cross_track) <= .8*c.lateral_tolerance_m:
            self.reason = 'board_normal_heading_alignment'
            return 0.0, clamp(self.turn(1.2*theta, settling=True), limit)
        # Leave visual clearance. A differential drive cannot remove a lateral
        # offset by rotating in place; do not continue forward beyond this margin.
        clearance = optical_z-(c.min_visual_distance_m+.04)
        if clearance <= .003:
            self.halt('insufficient_visual_alignment_space')
            return 0.0, 0.0
        lookahead = max(.04, min(.12, clearance))
        heading_offset = clamp(math.atan2(cross_track, lookahead), math.radians(15))
        # Fresh local odometry anticipates a fraction of the observed turn lag.
        # This is damping only: an invalid visual pose never reaches this method.
        predicted_theta = theta-self.odom[4]*.15
        heading_error = wrap(predicted_theta+heading_offset)
        angular = clamp(self.turn(1.5*heading_error, settling=True), limit)
        linear = min(c.align_speed_mps, .35*clearance)
        self.reason = 'board_normal_tracking'
        if abs(theta) >= math.radians(18) or abs(heading_error) >= math.radians(8):
            linear = 0.0
            self.reason = 'board_normal_turn_before_forward'
        else:
            linear = max(.01, linear*math.cos(heading_error))
        return linear, angular

    def _tick(self, now):
        zero = (0.0, 0.0)
        c = self.cfg
        if self.state not in self.ACTIVE:
            return zero
        error = self.input_error(now, vision=self.state != 'STOPPING')
        if error:
            if error == 'vision_timeout' and self.state == 'ALIGN':
                if self.vision_recoveries >= c.max_vision_recoveries:
                    self.halt('vision_recovery_limit')
                    return zero
                self.state, self.reason = 'VISION_WAIT', 'vision_timeout_waiting'
                self.vision_recoveries += 1
                self.vision_wait_at = now
                self.vision_good_since = self.recovery_stamp = self.aligned_since = None
                self.recovery_frames = 0
            elif not (error == 'vision_timeout' and self.state == 'VISION_WAIT'):
                self.halt(error)
                return zero
        if self.state == 'STOPPING':
            if abs(self.odom[3]) < .01 and abs(self.odom[4]) < .04:
                if self.stopped_since is None:
                    self.stopped_since = now
                if now-self.stopped_since >= c.stopped_hold_s:
                    self.state, self.reason = 'DOCKED', 'ir_high_and_stationary'
            else:
                self.stopped_since = None
            if now-self.final_at > c.max_final_time_s+c.stopped_hold_s+2:
                self.halt('stop_not_confirmed')
            return zero
        if self.ir:
            self.set_ir(True, now)
            return zero
        if now-self.started_at >= c.max_total_time_s or self.distance >= c.max_total_distance_m:
            self.halt('total_approach_limit')
            return zero
        if self.state == 'VISION_WAIT':
            # Keep publishing zero. Do not reset mission distance/time budgets.
            if now-self.vision_wait_at >= c.vision_recovery_timeout_s:
                self.halt('vision_recovery_timeout')
                return zero
            p = self.pose()
            healthy = (self.fresh(self.observation_time, now, c.vision_recovery_fresh_s)
                       and self.observation_time > self.vision_wait_at
                       and p is not None
                       and c.min_visual_distance_m < p[3] <= c.max_start_distance_m
                       and abs(self.odom[3]) <= .01 and abs(self.odom[4]) <= .04)
            if not healthy:
                self.vision_good_since = None
                self.recovery_frames = 0
            else:
                if self.vision_good_since is None:
                    self.vision_good_since = now
                if self.observation_time != self.recovery_stamp:
                    self.recovery_frames += 1
                    self.recovery_stamp = self.observation_time
                if (self.recovery_frames >= 3
                        and now-self.vision_good_since >= c.vision_recovery_hold_s):
                    self.state, self.reason = 'ALIGN', 'vision_recovered'
                    self.aligned_since = None
            return zero
        if self.state == 'FINAL_APPROACH':
            if now-self.final_at >= c.max_final_time_s or self.final_distance >= c.max_final_distance_m:
                self.halt('final_approach_limit')
                return zero
            # Alignment is latched. Neither visibility nor pose quality controls
            # this straight segment; camera liveness was checked above.
            return c.final_speed_mps, 0.0
        p = self.pose()
        if p is None:
            self.aligned_since = None
            self.reason = 'waiting_for_four_markers'
            return zero
        bx, by, theta, optical_z = p
        if optical_z < c.min_visual_distance_m:
            self.halt('too_close_before_alignment')
            return zero
        aligned = abs(by) <= c.lateral_tolerance_m and abs(theta) <= math.radians(c.yaw_tolerance_deg)
        # Staging distance is optical-camera distance for a forward-facing centered camera.
        stage = c.staging_distance_m + c.camera_forward_m
        if aligned and optical_z <= c.staging_distance_m+.02:
            if abs(self.odom[3]) > .01 or abs(self.odom[4]) > .04:
                self.aligned_since = None
            elif self.aligned_since is None:
                self.aligned_since = now
            elif now-self.aligned_since >= c.aligned_hold_s:
                self.state, self.reason = 'FINAL_APPROACH', 'alignment_confirmed'
                self.final_at, self.final_distance = now, 0.0
            return zero
        self.aligned_since = None
        self.reason = 'visual_alignment'
        if aligned:
            return c.align_speed_mps, 0.0
        if c.board_normal_tracking:
            return self.track_board_normal(bx, by, theta, optical_z)
        # Forward-only polar pose controller to a point on the board normal.
        gx, gy = bx-stage*math.cos(theta), by-stage*math.sin(theta)
        rho = math.hypot(gx, gy)
        # Near the staging point the polar bearing becomes ill-conditioned:
        # a few mm of lateral residual can demand a 90/180 degree turn.
        # Cross-track distance to the board normal is invariant to an in-place turn.
        cross_track = by*math.cos(theta)-bx*math.sin(theta)
        if rho < .02 and abs(cross_track) <= c.lateral_tolerance_m:
            self.reason = 'near_stage_heading_alignment'
            return 0.0, clamp(self.turn(1.2*theta, settling=True), c.near_angular_rps)
        # A forward-only robot cannot chase a staging point beside/behind it.
        # Move the virtual target forward along the board normal, while reserving
        # visual clearance. This is still closed-loop ArUco alignment, never blind.
        if gx <= .02 and abs(theta) < math.radians(45):
            local_stage = max(c.min_visual_distance_m+.025, min(stage, bx-.04))
            gx, gy = bx-local_stage*math.cos(theta), by-local_stage*math.sin(theta)
            if gx <= .003:
                self.halt('insufficient_visual_alignment_space')
                return zero
            rho = math.hypot(gx, gy)
            self.reason = 'near_stage_forward_alignment'
        if rho < .004:
            return 0.0, clamp(self.turn(1.2*theta, settling=True), c.near_angular_rps)
        alpha = math.atan2(gy, gx)
        beta = wrap(theta-alpha)
        blend = max(0., min(1., (optical_z-c.staging_distance_m)/.25))
        limit = min(c.max_angular_rps, c.near_angular_rps + blend*(c.max_angular_rps-c.near_angular_rps))
        # Boost the well-conditioned far-field turn, taper near the staging pose.
        angular = clamp(self.turn((1.+(c.far_steering_gain-1.)*blend)*(1.8*alpha-.7*beta)),limit)
        linear = min(c.align_speed_mps, .4*rho)
        heading_limit = 15 if self.reason == 'near_stage_forward_alignment' else 50
        if abs(alpha) > math.radians(heading_limit):
            linear = 0.0
        elif linear > 0:
            # OpenCR stores velocity in 0.01 m/s increments.
            linear = max(.01, linear*max(0.0, math.cos(alpha)))
        return linear, angular
