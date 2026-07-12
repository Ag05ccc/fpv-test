import numpy as np
import pytest

from kenet.camera import CameraCapture, create_camera_capture, is_gz_source
from kenet.gz_camera import (
    PIXEL_BGR_INT8,
    PIXEL_BGRA_INT8,
    PIXEL_L_INT8,
    PIXEL_RGB_INT8,
    GazeboCameraCapture,
    UnsupportedPixelFormatError,
    image_msg_to_bgr,
)


class FakeImageMsg:
    def __init__(self, width, height, pixel_format_type, data, step=0):
        self.width = width
        self.height = height
        self.pixel_format_type = pixel_format_type
        self.data = data
        self.step = step


class FakeNode:
    def __init__(self, subscribe_ok=True):
        self.subscribe_ok = subscribe_ok
        self.subscriptions = []
        self.unsubscribed = []

    def subscribe(self, msg_type, topic, callback):
        self.subscriptions.append((msg_type, topic, callback))
        return self.subscribe_ok

    def unsubscribe(self, topic):
        self.unsubscribed.append(topic)
        return True


def rgb_frame_bytes(width, height):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 0] = 200  # R
    frame[:, :, 1] = 100  # G
    frame[:, :, 2] = 50   # B
    return frame.tobytes()


def test_rgb_converts_to_bgr():
    bgr = image_msg_to_bgr(4, 2, PIXEL_RGB_INT8, rgb_frame_bytes(4, 2))
    assert bgr.shape == (2, 4, 3)
    assert bgr[0, 0].tolist() == [50, 100, 200]


def test_bgr_passthrough():
    data = np.full((2, 3, 3), 7, dtype=np.uint8).tobytes()
    bgr = image_msg_to_bgr(3, 2, PIXEL_BGR_INT8, data)
    assert bgr[1, 2].tolist() == [7, 7, 7]


def test_mono_expands_to_three_channels():
    data = bytes(range(6))
    bgr = image_msg_to_bgr(3, 2, PIXEL_L_INT8, data)
    assert bgr.shape == (2, 3, 3)
    assert bgr[1, 2].tolist() == [5, 5, 5]


def test_bgra_drops_alpha():
    pixel = [10, 20, 30, 255]
    data = bytes(pixel * 6)
    bgr = image_msg_to_bgr(3, 2, PIXEL_BGRA_INT8, data)
    assert bgr[0, 0].tolist() == [10, 20, 30]


def test_row_stride_is_respected():
    # 2x2 RGB with 8-byte stride (2 padding bytes per row)
    row0 = bytes([1, 2, 3, 4, 5, 6, 0, 0])
    row1 = bytes([7, 8, 9, 10, 11, 12, 0, 0])
    bgr = image_msg_to_bgr(2, 2, PIXEL_RGB_INT8, row0 + row1, step=8)
    assert bgr[0, 0].tolist() == [3, 2, 1]
    assert bgr[1, 1].tolist() == [12, 11, 10]


def test_short_payload_raises():
    with pytest.raises(ValueError):
        image_msg_to_bgr(4, 4, PIXEL_RGB_INT8, b"\x00" * 10)


def test_unknown_format_raises():
    with pytest.raises(UnsupportedPixelFormatError):
        image_msg_to_bgr(2, 2, 15, b"\x00" * 16)  # BAYER_RGGB8


def make_capture(node=None):
    node = node or FakeNode()
    capture = GazeboCameraCapture(
        topic="/kenet/fpv_camera",
        node_factory=lambda: node,
        image_msg_type=FakeImageMsg,
    )
    return capture, node


def test_capture_receives_frames_and_counts():
    capture, node = make_capture()
    capture.start()
    assert capture.read() is None
    _, topic, callback = node.subscriptions[0]
    assert topic == "/kenet/fpv_camera"
    callback(FakeImageMsg(4, 2, PIXEL_RGB_INT8, rgb_frame_bytes(4, 2)))
    frame = capture.read()
    assert frame is not None
    assert frame.shape == (2, 4, 3)
    assert frame[0, 0].tolist() == [50, 100, 200]
    assert capture.frame_count == 1
    assert capture.last_frame_age() is not None
    # read() returns a copy: mutating it does not poison the buffer
    frame[:] = 0
    assert capture.read()[0, 0].tolist() == [50, 100, 200]


def test_capture_decode_error_is_counted_not_raised():
    capture, node = make_capture()
    capture.start()
    _, _, callback = node.subscriptions[0]
    callback(FakeImageMsg(4, 4, PIXEL_RGB_INT8, b"\x00" * 3))
    assert capture.decode_errors == 1
    assert capture.read() is None


def test_capture_stop_unsubscribes_and_clears():
    capture, node = make_capture()
    capture.start()
    _, _, callback = node.subscriptions[0]
    callback(FakeImageMsg(4, 2, PIXEL_RGB_INT8, rgb_frame_bytes(4, 2)))
    capture.stop()
    assert node.unsubscribed == ["/kenet/fpv_camera"]
    assert capture.read() is None
    # late frames after stop are ignored
    callback(FakeImageMsg(4, 2, PIXEL_RGB_INT8, rgb_frame_bytes(4, 2)))
    assert capture.read() is None


def test_capture_subscribe_failure_raises():
    capture, _ = make_capture(FakeNode(subscribe_ok=False))
    with pytest.raises(RuntimeError):
        capture.start()


def test_wait_first_frame_timeout_raises():
    node = FakeNode()
    capture = GazeboCameraCapture(
        topic="/x", node_factory=lambda: node, image_msg_type=FakeImageMsg,
        wait_first_frame_seconds=0.05)
    with pytest.raises(RuntimeError):
        capture.start()
    assert node.unsubscribed == ["/x"]


def test_factory_routes_sources():
    assert is_gz_source("gz:/kenet/fpv_camera")
    assert not is_gz_source("test-2.mp4")
    assert not is_gz_source(0)
    gz_capture = create_camera_capture("gz:/kenet/fpv_camera")
    assert isinstance(gz_capture, GazeboCameraCapture)
    assert gz_capture.topic == "/kenet/fpv_camera"
    default_topic = create_camera_capture("gz:")
    assert default_topic.topic == "/kenet/fpv_camera"
    file_capture = create_camera_capture("test-2.mp4")
    assert isinstance(file_capture, CameraCapture)
