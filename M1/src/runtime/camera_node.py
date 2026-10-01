#!/usr/bin/env python3
"""Publish IMX708 full-field capture as 640x360 ROS images without padding.

Timestamps mark completed pipe reads, not hardware exposure time. The reader
continuously drains the camera; only the latest frame is kept for publication.
"""
import argparse
import os
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
import yaml


def read_frames(stream, frames, stamp, width=1920, height=1080):
    size = width * height * 3
    with stream:
        while True:
            data = stream.read(size)
            if len(data) != size:
                return
            item = (data, stamp())
            try:
                frames.get_nowait()
            except queue.Empty:
                pass
            frames.put_nowait(item)


def image_message(data, stamp, frame_id, width=1920, height=1080):
    pixels = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    if (width, height) != (640, 360):
        pixels = cv2.resize(pixels, (640, 360), interpolation=cv2.INTER_AREA)
    pixels = pixels[:, :, :3]
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = pixels.shape[:2]
    msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, 640 * 3
    msg.data = pixels.tobytes()
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('docking.yaml')))
    args = parser.parse_args()
    # Avoid competing OpenCV worker pools on the robot; vision also uses one thread.
    cv2.setNumThreads(1)
    config = yaml.safe_load(Path(args.config).read_text())
    capture_fps = float(os.environ.get("CAMERA_CAPTURE_FPS", "15"))
    frame_us = int(os.environ.get("CAMERA_FRAME_US", "66667"))
    binary = Path(os.environ.get('CAMERA_BINARY', config['vision']['camera_binary'])).expanduser()
    if not binary.is_file():
        parser.error(f'Missing Burger1 camera runtime: {binary}')
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, interrupted)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('camera', namespace='/burger1')
    publisher = node.create_publisher(Image, config['vision']['camera_topic'],
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    frames = queue.Queue(maxsize=1)
    process = reader = None
    capture_settings = tempfile.TemporaryDirectory(prefix='burger1-camera-')
    try:
        script = Path(capture_settings.name) / 'capture.yaml'
        # Keep full sensor crop with 1920x1080; smaller streams choose a cropped sensor mode.
        # libcamera RGB888 is packed BGR in memory (V4L2 BGR24): 6.22 MB/frame.
        script.write_text(yaml.safe_dump({
            'properties': [{'loop': 1}],
            'frames': [{0: {'FrameDurationLimits': [frame_us, frame_us]}}],
        }))
        read_fd, write_fd = os.pipe()
        stream = os.fdopen(read_fd, 'rb')
        env = os.environ.copy()
        env.pop('LD_LIBRARY_PATH', None)
        try:
            process = subprocess.Popen([
                str(binary), '-c', '1', '--capture',
                '--stream=width=1920,height=1080,pixelformat=RGB888',
                '--strict-formats', f'--script={script}',
                f'--file=/proc/self/fd/{write_fd}'],
                env=env, pass_fds=(write_fd,), stdout=subprocess.DEVNULL)
        except Exception:
            stream.close()
            raise
        finally:
            os.close(write_fd)
        reader = threading.Thread(target=read_frames, args=(
            stream, frames, lambda: node.get_clock().now().to_msg(), 1920, 1080), daemon=True)
        reader.start()
        def publish_latest():
            try:
                data, stamp = frames.get_nowait()
            except queue.Empty:
                return
            publisher.publish(image_message(data, stamp, config['camera_frame'], 1920, 1080))
        node.create_timer(1 / capture_fps, publish_latest)
        node.get_logger().info(f'Burger1 full-field 1920x1080 -> 640x360, capture {capture_fps:g} fps -> ' + config['vision']['camera_topic'])
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if not reader.is_alive():
                raise RuntimeError('Camera stream ended; check camera ownership and cam stderr')
    except KeyboardInterrupt:
        return 130
    finally:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if reader is not None:
            reader.join(timeout=2)
        capture_settings.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
