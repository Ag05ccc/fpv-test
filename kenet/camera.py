"""
camera - Threaded video capture using OpenCV.
"""

import time
import threading
import logging

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Camera sources with this prefix are Gazebo camera topics, e.g.
# "gz:/kenet/fpv_camera" (see kenet.gz_camera).
GZ_SOURCE_PREFIX = "gz:"


def is_gz_source(source):
    return isinstance(source, str) and source.startswith(GZ_SOURCE_PREFIX)


def create_camera_capture(source, width=640, height=480, fps=30, **gz_kwargs):
    """Build the right capture backend for a camera source.

    int / digit-string -> V4L2 device index, other string -> video file,
    "gz:<topic>" -> live Gazebo camera topic over gz-transport.
    """
    if is_gz_source(source):
        from .gz_camera import DEFAULT_FPV_TOPIC, GazeboCameraCapture
        topic = source[len(GZ_SOURCE_PREFIX):] or DEFAULT_FPV_TOPIC
        return GazeboCameraCapture(topic=topic, width=width, height=height,
                                   fps=fps, **gz_kwargs)
    return CameraCapture(source, width, height, fps)


class CameraCapture:
    """Threaded camera capture. Grabs frames in a background thread so the
    control loop never blocks on I/O."""

    def __init__(self, source=0, width=640, height=480, fps=30):
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self._is_video = isinstance(source, str)
        self._cap = None
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self._cap = cv2.VideoCapture(self.source)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self._cap.isOpened():
            raise RuntimeError("Cannot open camera source %s" % self.source)
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info("Camera started: %dx%d @ %dfps", self.width, self.height, self.fps)

    def _capture_loop(self):
        delay = 1.0 / self.fps if self._is_video else 0
        while self._running:
            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._frame = frame
            elif self._is_video:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            if delay > 0:
                time.sleep(delay)

    def read(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        with self._lock:
            self._frame = None
        logger.info("Camera stopped")
