"""
gz_camera - Live Gazebo Sim camera capture over gz-transport.

Subscribes to a Gazebo camera sensor topic (e.g. /kenet/fpv_camera injected by
run_gazebo_betaflight.sh --iris-forward-camera) and exposes the frames through
the same start()/read()/stop() interface as kenet.camera.CameraCapture, so the
pipeline and the SITL mixer can consume the simulated FPV camera directly.

The gz Python bindings live in the system dist-packages (python3-gz-transport13
/ python3-gz-msgs10), not in fpv_env. Both interpreters are CPython 3.12, so
appending the dist-packages path is enough; _import_gz_modules() does that
lazily and only when a Gazebo source is actually requested.
"""

import os
import sys
import time
import threading
import logging

import numpy as np

logger = logging.getLogger(__name__)

DEFAULT_FPV_TOPIC = "/kenet/fpv_camera"
GZ_DIST_PACKAGES = "/usr/lib/python3/dist-packages"

# gz.msgs10.image_pb2.PixelFormatType values (protobuf enum numbers are stable)
PIXEL_L_INT8 = 1
PIXEL_RGB_INT8 = 3
PIXEL_RGBA_INT8 = 4
PIXEL_BGRA_INT8 = 5
PIXEL_BGR_INT8 = 8

_CHANNELS = {
    PIXEL_L_INT8: 1,
    PIXEL_RGB_INT8: 3,
    PIXEL_RGBA_INT8: 4,
    PIXEL_BGRA_INT8: 4,
    PIXEL_BGR_INT8: 3,
}


class GzTransportUnavailableError(RuntimeError):
    """Raised when the gz-transport Python bindings cannot be imported."""


class UnsupportedPixelFormatError(ValueError):
    """Raised for camera pixel formats this bridge does not convert."""


def _import_gz_modules():
    try:
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
    except ImportError:
        if GZ_DIST_PACKAGES not in sys.path and os.path.isdir(GZ_DIST_PACKAGES):
            sys.path.append(GZ_DIST_PACKAGES)
        try:
            from gz.transport13 import Node
            from gz.msgs10.image_pb2 import Image
        except ImportError as e:
            raise GzTransportUnavailableError(
                "gz-transport Python bindings not found. Install the system "
                "packages python3-gz-transport13 and python3-gz-msgs10 "
                "(Gazebo Harmonic)."
            ) from e
    return Node, Image


def image_msg_to_bgr(width, height, pixel_format, data, step=0):
    """Convert a gz Image message payload to an OpenCV BGR ndarray.

    Pure function so it is unit-testable without gz installed. `step` is the
    row stride in bytes; 0 means tightly packed.
    """
    channels = _CHANNELS.get(pixel_format)
    if channels is None:
        raise UnsupportedPixelFormatError(
            "Unsupported gz camera pixel_format_type=%s" % pixel_format)

    row_bytes = width * channels
    stride = step if step and step >= row_bytes else row_bytes
    expected = stride * height
    if len(data) < expected:
        raise ValueError(
            "gz image payload too short: got %d bytes, need %d (%dx%d fmt=%s)"
            % (len(data), expected, width, height, pixel_format))

    flat = np.frombuffer(data, dtype=np.uint8, count=expected)
    rows = flat.reshape(height, stride)[:, :row_bytes]
    if channels == 1:
        gray = rows.reshape(height, width)
        return np.repeat(gray[:, :, np.newaxis], 3, axis=2).copy()

    img = rows.reshape(height, width, channels)
    if pixel_format == PIXEL_RGB_INT8:
        return img[:, :, ::-1].copy()
    if pixel_format == PIXEL_RGBA_INT8:
        return img[:, :, [2, 1, 0]].copy()
    if pixel_format == PIXEL_BGRA_INT8:
        return img[:, :, :3].copy()
    return img.copy()  # BGR_INT8


class GazeboCameraCapture:
    """Threaded-callback capture from a Gazebo camera topic.

    Mirrors kenet.camera.CameraCapture: start() begins receiving, read()
    returns the latest BGR frame (or None before the first frame arrives),
    stop() unsubscribes. Frames arrive on a gz-transport thread; read() only
    copies under a lock, so the control loop never blocks on the sim.
    """

    def __init__(self, topic=DEFAULT_FPV_TOPIC, width=640, height=480, fps=30,
                 node_factory=None, image_msg_type=None,
                 wait_first_frame_seconds=0.0):
        # width/height/fps are advisory: the actual size comes from the topic.
        self.topic = topic
        self.width = width
        self.height = height
        self.fps = fps
        self.wait_first_frame_seconds = wait_first_frame_seconds
        self._node_factory = node_factory
        self._image_msg_type = image_msg_type
        self._node = None
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._frame_count = 0
        self._last_frame_monotonic = None
        self._decode_errors = 0

    @property
    def frame_count(self):
        with self._lock:
            return self._frame_count

    @property
    def decode_errors(self):
        with self._lock:
            return self._decode_errors

    def last_frame_age(self, now=None):
        """Seconds since the last frame arrived, or None before any frame."""
        with self._lock:
            last = self._last_frame_monotonic
        if last is None:
            return None
        now = time.monotonic() if now is None else now
        return now - last

    def start(self):
        if self._node_factory is not None:
            self._node = self._node_factory()
            image_type = self._image_msg_type
        else:
            node_cls, image_type = _import_gz_modules()
            self._node = node_cls()
        self._running = True
        ok = self._node.subscribe(image_type, self.topic, self._on_image)
        if ok is False:
            self._running = False
            self._node = None
            raise RuntimeError("Cannot subscribe to gz topic %s" % self.topic)
        logger.info("Gazebo camera subscribed: %s", self.topic)
        if self.wait_first_frame_seconds > 0:
            deadline = time.monotonic() + self.wait_first_frame_seconds
            while time.monotonic() < deadline:
                if self.read() is not None:
                    return
                time.sleep(0.02)
            self.stop()
            raise RuntimeError(
                "No frame on gz topic %s within %.1fs (is the sim running "
                "with --iris-forward-camera?)"
                % (self.topic, self.wait_first_frame_seconds))

    def _on_image(self, msg):
        if not self._running:
            return
        try:
            frame = image_msg_to_bgr(
                msg.width, msg.height, msg.pixel_format_type, msg.data,
                getattr(msg, "step", 0))
        except (UnsupportedPixelFormatError, ValueError) as e:
            with self._lock:
                self._decode_errors += 1
                count = self._decode_errors
            if count == 1:
                logger.warning("gz camera frame decode failed: %s", e)
            return
        with self._lock:
            self._frame = frame
            self._frame_count += 1
            self._last_frame_monotonic = time.monotonic()

    def read(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._running = False
        node = self._node
        self._node = None
        if node is not None:
            try:
                node.unsubscribe(self.topic)
            except Exception:  # gz node may already be torn down
                pass
        with self._lock:
            self._frame = None
        logger.info("Gazebo camera stopped: %s", self.topic)
