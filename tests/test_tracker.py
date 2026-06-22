from types import SimpleNamespace
import unittest
from unittest.mock import patch

import kenet.tracker as tracker_module
from kenet.tracker import ObjectTracker, TrackerType, TrackerUnavailableError


class ClassFactory:
    @staticmethod
    def create():
        return "class-tracker"


def test_tracker_factory_supports_class_create_api():
    fake_cv2 = SimpleNamespace(TrackerCSRT=ClassFactory)

    with patch.object(tracker_module, "cv2", fake_cv2):
        assert ObjectTracker(TrackerType.CSRT)._create_cv_tracker() == "class-tracker"


def test_tracker_factory_supports_legacy_function_api():
    fake_cv2 = SimpleNamespace(
        legacy=SimpleNamespace(TrackerKCF_create=lambda: "legacy-tracker")
    )

    with patch.object(tracker_module, "cv2", fake_cv2):
        assert ObjectTracker(TrackerType.KCF)._create_cv_tracker() == "legacy-tracker"


def test_tracker_factory_reports_missing_contrib_backend():
    with patch.object(tracker_module, "cv2", SimpleNamespace()):
        with unittest.TestCase().assertRaisesRegex(
            TrackerUnavailableError, "opencv-contrib-python"
        ):
            ObjectTracker(TrackerType.CSRT)._create_cv_tracker()
