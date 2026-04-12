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

    def _create_cv_tracker(self):
        if self.tracker_type == TrackerType.CSRT:
            return cv2.TrackerCSRT.create()
        elif self.tracker_type == TrackerType.KCF:
            return cv2.TrackerKCF.create()
        raise ValueError("Unknown tracker type: %s" % self.tracker_type)

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
