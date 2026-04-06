"""
tracker - Object tracking backends (CSRT, KCF, HSV color).
"""

import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class TrackerType:
    CSRT = "CSRT"
    KCF = "KCF"
    COLOR = "COLOR"


@dataclass
class TrackResult:
    found: bool = False
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # x, y, w, h
    center: Tuple[float, float] = (0.0, 0.0)


class ObjectTracker:
    """Wraps OpenCV trackers (CSRT / KCF) or simple HSV color tracking.

    Usage::

        tracker.init(frame, bbox)
        result = tracker.update(frame)
    """

    def __init__(self, tracker_type: str = TrackerType.CSRT,
                 hsv_lower: Tuple[int, int, int] = (0, 100, 100),
                 hsv_upper: Tuple[int, int, int] = (10, 255, 255)):
        self.tracker_type = tracker_type
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        self._tracker: Optional[cv2.Tracker] = None
        self._initialized = False

    def _create_cv_tracker(self):
        if self.tracker_type == TrackerType.CSRT:
            return cv2.TrackerCSRT.create()
        elif self.tracker_type == TrackerType.KCF:
            return cv2.TrackerKCF.create()
        raise ValueError(f"Unknown tracker type: {self.tracker_type}")

    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]):
        """Initialize tracker with a bounding box (x, y, w, h)."""
        if self.tracker_type in (TrackerType.CSRT, TrackerType.KCF):
            self._tracker = self._create_cv_tracker()
            self._tracker.init(frame, bbox)
        self._initialized = True
        logger.info("Tracker initialized: %s, bbox=%s", self.tracker_type, bbox)

    def update(self, frame: np.ndarray) -> TrackResult:
        if not self._initialized:
            return TrackResult()

        if self.tracker_type == TrackerType.COLOR:
            return self._update_color(frame)

        ok, bbox = self._tracker.update(frame)
        if not ok:
            return TrackResult(found=False)

        x, y, w, h = [int(v) for v in bbox]
        cx, cy = x + w / 2, y + h / 2
        return TrackResult(found=True, bbox=(x, y, w, h), center=(cx, cy))

    def _update_color(self, frame: np.ndarray) -> TrackResult:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return TrackResult(found=False)
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w / 2, y + h / 2
        return TrackResult(found=True, bbox=(x, y, w, h), center=(cx, cy))

    def select_roi(self, frame: np.ndarray) -> Tuple[int, int, int, int]:
        """Interactive ROI selection (requires a display)."""
        bbox = cv2.selectROI("Select Target", frame, fromCenter=False,
                             showCrosshair=True)
        cv2.destroyWindow("Select Target")
        return tuple(int(v) for v in bbox)
