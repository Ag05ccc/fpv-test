"""Unit tests for the SITL timing probe's RTF-collapse classification and the
/stats parsing it relies on. Pure functions, no Gazebo required."""

import pathlib
import sys
import unittest

TOOLS = pathlib.Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import sitl_timing_probe as probe
import gazebo_stats_monitor as stats


SAMPLE_STATS = """
sim_time {
  sec: 333
  nsec: 500000000
}
real_time {
  sec: 342
  nsec: 250000000
}
real_time_factor: 0.034
iterations: 83250
step_size {
  sec: 0
  nsec: 1000000
}
"""


class TestClassifyRtf(unittest.TestCase):
    def test_none_is_unknown(self):
        self.assertEqual(probe.classify_rtf(None), "unknown")

    def test_collapsed_below_threshold(self):
        self.assertEqual(probe.classify_rtf(0.034), "collapsed")
        self.assertEqual(probe.classify_rtf(0.39), "collapsed")

    def test_degraded_band(self):
        # >= collapse (0.4), < degraded (0.8)
        self.assertEqual(probe.classify_rtf(0.40), "degraded")
        self.assertEqual(probe.classify_rtf(0.79), "degraded")

    def test_healthy_at_or_above_degraded(self):
        self.assertEqual(probe.classify_rtf(0.80), "healthy")
        self.assertEqual(probe.classify_rtf(1.31), "healthy")

    def test_custom_thresholds(self):
        self.assertEqual(probe.classify_rtf(0.5, collapse_rtf=0.6), "collapsed")


class TestDeltaRtf(unittest.TestCase):
    def test_basic_ratio(self):
        prev = {"sim_time": 10.0, "real_time": 100.0}
        cur = {"sim_time": 11.0, "real_time": 130.0}
        # 1s sim advanced over 30s wall -> ~0.0333
        self.assertAlmostEqual(probe.delta_rtf(prev, cur), 1.0 / 30.0, places=6)

    def test_non_positive_real_delta_is_none(self):
        prev = {"sim_time": 10.0, "real_time": 100.0}
        cur = {"sim_time": 11.0, "real_time": 100.0}
        self.assertIsNone(probe.delta_rtf(prev, cur))

    def test_missing_fields_is_none(self):
        self.assertIsNone(probe.delta_rtf(None, {"sim_time": 1, "real_time": 2}))
        self.assertIsNone(probe.delta_rtf({"sim_time": None, "real_time": 1},
                                          {"sim_time": 1, "real_time": 2}))


class TestSummarize(unittest.TestCase):
    def test_empty_is_none(self):
        self.assertIsNone(probe.summarize([]))

    def test_stats(self):
        s = probe.summarize([0.03, 0.05, 0.04])
        self.assertEqual(s["count"], 3)
        self.assertAlmostEqual(s["min"], 0.03)
        self.assertAlmostEqual(s["max"], 0.05)
        self.assertAlmostEqual(s["mean"], 0.04)
        self.assertAlmostEqual(s["spread"], 0.02)


class TestStatsParsing(unittest.TestCase):
    def test_parse_rtf(self):
        self.assertAlmostEqual(stats.parse_rtf(SAMPLE_STATS), 0.034)

    def test_parse_step_size_is_1ms(self):
        self.assertAlmostEqual(stats.parse_step_size(SAMPLE_STATS), 0.001)

    def test_parse_block(self):
        parsed = stats.parse_stats(SAMPLE_STATS)
        self.assertAlmostEqual(parsed["sim_time"], 333.5)
        self.assertAlmostEqual(parsed["real_time"], 342.25)
        self.assertEqual(parsed["iterations"], 83250)

    def test_collapsed_sample_classifies_as_collapsed(self):
        # end-to-end: a real-looking collapsed /stats dump should bucket as collapsed
        parsed = stats.parse_stats(SAMPLE_STATS)
        self.assertEqual(probe.classify_rtf(parsed["rtf"]), "collapsed")


if __name__ == "__main__":
    unittest.main()
