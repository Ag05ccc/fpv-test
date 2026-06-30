"""
tracker - Object tracking backends (CSRT, KCF).
"""

import logging
from dataclasses import dataclass

import cv2

logger = logging.getLogger(__name__)


class TrackerType:
    CSRT = "CSRT"
    KCF = "KCF"


class TrackerUnavailableError(RuntimeError):
    """Raised when the requested OpenCV tracker backend is not installed."""


@dataclass
class TrackResult:
    found: bool = False
    bbox: tuple = (0, 0, 0, 0)      # x, y, w, h
    center: tuple = (0.0, 0.0)


class ObjectTracker:
    """Wraps OpenCV trackers (CSRT / KCF).

    Usage::

        tracker.init(frame, bbox)
        result = tracker.update(frame)
    """

    def __init__(self, tracker_type=TrackerType.CSRT):
        self.tracker_type = tracker_type
        self._tracker = None
        self._initialized = False

    @property
    def is_initialized(self):
        return self._initialized

    def _create_cv_tracker(self):
        if self.tracker_type not in (TrackerType.CSRT, TrackerType.KCF):
            raise ValueError("Unknown tracker type: %s" % self.tracker_type)

        tracker_name = "Tracker%s" % self.tracker_type
        last_error = None
        for factory in _tracker_factories(tracker_name):
            try:
                return factory()
            except Exception as e:
                last_error = e

        raise TrackerUnavailableError(
            "%s is unavailable in this OpenCV build. Install "
            "opencv-contrib-python (or opencv-contrib-python-headless on "
            "headless systems) and make sure opencv-python is not installed "
            "in the same environment." % tracker_name
        ) from last_error

    def init(self, frame, bbox):
        """Initialize tracker with a bounding box (x, y, w, h)."""
        self._tracker = self._create_cv_tracker()
        self._tracker.init(frame, bbox)
        self._initialized = True
        logger.info("Tracker initialized: %s, bbox=%s", self.tracker_type, bbox)

    def reset(self):
        """Deactivate tracker. update() will return found=False."""
        self._tracker = None
        self._initialized = False

    def update(self, frame):
        if not self._initialized:
            return TrackResult()

        ok, bbox = self._tracker.update(frame)
        if not ok:
            return TrackResult(found=False)

        x, y, w, h = [int(v) for v in bbox]
        cx, cy = x + w / 2, y + h / 2
        return TrackResult(found=True, bbox=(x, y, w, h), center=(cx, cy))


def _tracker_factories(tracker_name):
    """Yield OpenCV tracker constructors across common OpenCV API shapes."""
    create_func = getattr(cv2, "%s_create" % tracker_name, None)
    if create_func is not None:
        yield create_func

    tracker_cls = getattr(cv2, tracker_name, None)
    if tracker_cls is not None and hasattr(tracker_cls, "create"):
        yield tracker_cls.create

    legacy = getattr(cv2, "legacy", None)
    if legacy is None:
        return

    legacy_create_func = getattr(legacy, "%s_create" % tracker_name, None)
    if legacy_create_func is not None:
        yield legacy_create_func

    legacy_tracker_cls = getattr(legacy, tracker_name, None)
    if legacy_tracker_cls is not None and hasattr(legacy_tracker_cls, "create"):
        yield legacy_tracker_cls.create
