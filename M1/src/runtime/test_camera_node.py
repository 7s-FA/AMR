"""Camera transport tests without opening physical hardware."""
import io
import queue
import numpy as np
import cv2
from builtin_interfaces.msg import Time
from camera_node import image_message, read_frames


def test_bgr_conversion_preserves_full_field_image_and_stamp():
    stamp = Time(sec=123, nanosec=456)
    msg = image_message(bytes([10, 20, 30]) * (1920 * 1080), stamp,
                        'burger1_camera_optical_frame')
    assert (msg.width, msg.height, msg.step, msg.encoding) == (640, 360, 1920, 'bgr8')
    assert bytes(msg.data) == bytes([10, 20, 30]) * (640 * 360)
    assert msg.header.stamp == stamp
    assert msg.header.frame_id == 'burger1_camera_optical_frame'


def test_only_latest_complete_frame_is_retained():
    frames = queue.Queue(maxsize=1)
    first = bytes([1]) * (1920 * 1080 * 3)
    second = bytes([2]) * len(first)
    stamps = iter([10, 20])
    read_frames(io.BytesIO(first + second + b'incomplete'), frames, lambda: next(stamps))
    assert frames.get_nowait() == (second, 20)
    assert frames.empty()


def test_full_field_preserves_edges_without_padding():
    pixels = np.zeros((1080, 1920, 3), dtype=np.uint8)
    pixels[:, :960, :3] = [10, 20, 30]
    pixels[:, 960:, :3] = [40, 50, 60]
    msg = image_message(pixels.tobytes(), Time(), 'camera', 1920, 1080)
    assert (msg.width, msg.height) == (640, 360)
    output = np.frombuffer(msg.data, dtype=np.uint8).reshape(360, 640, 3)
    assert np.all(output[:, 0] == [10, 20, 30])
    assert np.all(output[:, -1] == [40, 50, 60])


def test_resize_optimization_preserves_calibration_pixels():
    pixels = np.random.default_rng(42).integers(0, 256, (1080, 1920, 3), dtype=np.uint8)
    expected = cv2.resize(pixels[:, :, :3], (640, 360), interpolation=cv2.INTER_AREA)
    msg = image_message(pixels.tobytes(), Time(), 'camera', 1920, 1080)
    actual = np.frombuffer(msg.data, dtype=np.uint8).reshape(360, 640, 3)
    np.testing.assert_array_equal(actual, expected)
