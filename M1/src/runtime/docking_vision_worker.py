"""Robot-local vision worker; control observations never traverse the host PC."""
import queue
import time


def put_latest(channel, value):
    try:
        channel.put_nowait(value)
    except queue.Full:
        try:
            channel.get_nowait()
        except queue.Empty:
            pass
        try:
            channel.put_nowait(value)
        except queue.Full:
            pass


def put_preview(frames, value):
    # Never drain a large multiprocessing pipe in the control-data producer.
    # A stalled viewer may lose display frames, but cannot block detection.
    try:
        frames.put_nowait(value)
    except queue.Full:
        pass


def run_preview(config, frames, stop):
    import cv2
    import rclpy
    from sensor_msgs.msg import CompressedImage
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init(args=[])
    node = rclpy.create_node('docking_result_video', namespace='/burger1')
    preview = node.create_publisher(CompressedImage, config['vision']['preview_topic'],
                                   QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        while not stop.is_set():
            try:
                annotated, metadata = frames.get(timeout=.2)
            except queue.Empty:
                continue
            if not preview.get_subscription_count():
                continue
            ok, jpeg = cv2.imencode('.jpg', annotated,
                                   [cv2.IMWRITE_JPEG_QUALITY, config['vision']['preview_jpeg_quality']])
            if ok:
                msg = CompressedImage()
                msg.header.frame_id = metadata['frame_id']
                msg.header.stamp.sec = metadata['source_stamp']['sec']
                msg.header.stamp.nanosec = metadata['source_stamp']['nanosec']
                msg.format = 'jpeg'
                msg.data = jpeg.tobytes()
                preview.publish(msg)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_vision(config, channel, frames, stop, progress=None, active=None, mode_index=None, configurations=None):
    import cv2
    from docking_vision.board import BoardDetector, DockingBoard
    from docking_vision.vision import load_calibration
    from docking_vision.source import RosSource

    source = None
    try:
        cv2.setNumThreads(1)
        configurations = configurations or [config]
        detectors = [BoardDetector(DockingBoard.load(c['board_path']),
                                   load_calibration(c['calibration_path'])) for c in configurations]
        source = RosSource(config['vision']['camera_topic'])
        sequence, last_preview, last_detection = 0, float('-inf'), float('-inf')
        while not stop.is_set():
            try:
                frame, metadata = source.read(.2)
            except RuntimeError:
                # Controller's local freshness watchdog stops on missing camera data.
                continue
            started = time.monotonic()
            full_rate = active is None or active.is_set()
            if not full_rate and started-last_detection < .5:
                continue
            last_detection = started
            if progress is not None:
                progress[0] = started
            selected = int(mode_index.value) if mode_index is not None else 0
            selected_config = configurations[selected]
            observation, annotated = detectors[selected].detect(frame)
            if progress is not None:
                progress[1] = time.monotonic()
            sequence += 1
            observation.update(metadata, sequence=sequence, status='ok',
                               vision_mode=selected_config.get('docking_mode', 'normal'),
                               detection_ms=(time.monotonic()-started)*1000)
            # Deliver control data BEFORE optional display work. No ROS observation input.
            put_latest(channel, observation)
            now = time.monotonic()
            if now-last_preview >= 1/(config['vision']['preview_fps'] if full_rate else 1.0):
                last_preview = now
                # Viewer/network failure cannot block detector or GPIO process.
                put_preview(frames, (annotated, metadata))
    except Exception as exc:
        put_latest(channel, {'status': 'error', 'error': str(exc)})
    finally:
        if source is not None:
            source.close()


class LocalVision:
    def __init__(self, config, modes=None):
        import multiprocessing as mp
        ctx = mp.get_context('spawn')
        modes = modes or {config.get('docking_mode', 'normal'): config}
        self.mode_names = list(modes)
        self.mode_index = ctx.Value('i', 0, lock=False)
        configurations = list(modes.values())
        self.channel = ctx.Queue(maxsize=1)
        self.frames = ctx.Queue(maxsize=1)
        self.stop = ctx.Event()
        self.active = ctx.Event()
        self.active.set()
        # Best-effort telemetry must not block control if the worker is killed.
        self.progress = ctx.Array('d', [0., 0.], lock=False)
        self.process = ctx.Process(target=run_vision, args=(config, self.channel, self.frames, self.stop, self.progress, self.active, self.mode_index, configurations), daemon=True)
        self.preview = ctx.Process(target=run_preview, args=(config, self.frames, self.stop), daemon=True)
        self.process.start()
        self.preview.start()

    def set_mode(self, mode):
        if mode not in self.mode_names:
            raise ValueError('Vision mode was not preloaded: '+mode)
        self.mode_index.value = self.mode_names.index(mode)
        # Small control records only; never drain the large preview pipe here.
        for _ in range(3):
            try:self.channel.get_nowait()
            except queue.Empty:break

    def set_active(self, active):
        self.active.set() if active else self.active.clear()

    def health(self):
        now = time.monotonic()
        received, detected = self.progress[:]
        return {'mode': self.mode_names[int(self.mode_index.value)], 'available_modes': self.mode_names, 'full_rate': self.active.is_set(), 'detector_alive': self.process.is_alive(), 'preview_alive': self.preview.is_alive(),
                'last_raw_read_age_s': now-received if received else None,
                'last_detection_age_s': now-detected if detected else None}

    def close(self):
        self.stop.set()
        for process in (self.process, self.preview):
            process.join(timeout=.5)
            if process.is_alive():
                process.terminate()
                process.join(timeout=.5)
            if process.is_alive():
                process.kill()
                process.join(timeout=.5)
        self.channel.close()
        self.frames.close()
