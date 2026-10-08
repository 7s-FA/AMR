"""Bounded frame reads for ROS camera topics or a local USB/video source."""
import multiprocessing as mp
import queue
import time

import cv2


def _capture(source, frames):
    cap = cv2.VideoCapture(int(source) if source.isdecimal() else source)
    try:
        if not cap.isOpened():
            frames.put(('error', f'Cannot open camera/video: {source}'))
            return
        while True:
            ok, frame = cap.read()
            if not ok:
                frames.put(('error', 'Camera read failed or video ended'))
                return
            item = ('frame', (frame, {'received_monotonic': time.monotonic()}))
            try:
                frames.put(item, timeout=.05)
            except queue.Full:
                # Keep only recent data; never accumulate frames for docking.
                try:
                    frames.get_nowait()
                except queue.Empty:
                    pass
    finally:
        cap.release()


class VideoSource:
    def __init__(self, source):
        ctx = mp.get_context('spawn')
        self.frames = ctx.Queue(maxsize=1)
        self.process = ctx.Process(target=_capture, args=(source, self.frames), daemon=True)
        self.process.start()

    def read(self, timeout):
        try:
            kind, value = self.frames.get(timeout=timeout)
        except queue.Empty as exc:
            raise RuntimeError(f'No camera frame for {timeout:g}s') from exc
        if kind == 'error':
            raise RuntimeError(value)
        return value

    def close(self):
        self.process.terminate()
        self.process.join(timeout=2)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=2)
        self.frames.close()


class RosSource:
    def __init__(self, topic, compressed=False, observation_topic=None, max_age_s=.5):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from cv_bridge import CvBridge
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage, Image
        from std_msgs.msg import String
        self.rclpy = rclpy
        self.bridge = CvBridge()
        self.compressed = compressed
        self.latest = None
        self.error = None
        self.last_stamp = None
        self.max_age_s = max_age_s
        self.stale_frames = self.accepted_frames = 0
        # Let the CLI's KeyboardInterrupt/finally run before closing the ROS context.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        namespace = '/'+topic.strip('/').split('/')[0]
        if namespace not in ('/burger1', '/burger2'):
            namespace = '/'
        self.node = rclpy.create_node('docking_camera_tools', namespace=namespace)
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.node.create_subscription(
            CompressedImage if compressed else Image, topic, self._receive,
            latest_qos)
        self.publisher = (self.node.create_publisher(String, observation_topic, latest_qos)
                          if observation_topic else None)

    def _receive(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        # Ignore duplicate or delayed frames before detection. Never refresh the
        # controller heartbeat from a capture older than one already consumed.
        if stamp != (0, 0) and self.last_stamp is not None and stamp <= self.last_stamp:
            return
        source_ns = stamp[0]*1_000_000_000+stamp[1]
        age = (self.node.get_clock().now().nanoseconds-source_ns)/1e9
        if source_ns <= 0 or not -.1 <= age <= self.max_age_s:
            self.stale_frames += 1
            return
        try:
            image = (self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') if self.compressed
                     else self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
            self.latest = (image, {'received_monotonic': time.monotonic(),
                                  'source_stamp': {'sec': stamp[0], 'nanosec': stamp[1]},
                                  'frame_id': msg.header.frame_id})
            self.last_stamp = stamp
            self.accepted_frames += 1
        except Exception as exc:
            self.error = str(exc)

    def read(self, timeout):
        deadline = time.monotonic() + timeout
        self.latest = None
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=min(.1, max(0, deadline - time.monotonic())))
            if self.latest is not None:
                return self.latest
        raise RuntimeError(f'No new ROS image for {timeout:g}s; conversion error: {self.error}')

    def publish(self, observation):
        if self.publisher and self.rclpy.ok():
            import json
            from std_msgs.msg import String
            self.publisher.publish(String(data=json.dumps(observation, allow_nan=False)))

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
