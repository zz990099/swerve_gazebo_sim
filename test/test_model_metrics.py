"""Independent timestamp and relative-motion oracles for plant measurements."""

import math
import unittest

from model_metrics import ModelMetrics, interpolate


class ModelMetricsTests(unittest.TestCase):
    def test_exact_timestamps_gap_and_no_extrapolation(self):
        history = [(0, 0, 0, 0), (20_000_000, 0.02, 0, 0)]
        self.assertEqual(interpolate(history, 0), (0, 0, 0))
        self.assertEqual(interpolate(history, 10_000_000), (0.01, 0, 0))
        self.assertIsNone(interpolate(history, -1))
        self.assertIsNone(interpolate(history, 30_000_000))
        self.assertIsNone(
            interpolate([(0, 0, 0, 0), (100_000_000, 1, 0, 0)], 50_000_000)
        )

    def test_yaw_interpolation_crosses_wrap(self):
        result = interpolate([(0, 0, 0, math.pi - 0.1), (20, 0, 0, -math.pi + 0.1)], 10)
        self.assertAlmostEqual(abs(result[2]), math.pi)

    def test_relative_motion_removes_origin_and_heading_offset(self):
        metrics = ModelMetrics()
        metrics.forecast(
            (1, 1), 0, 100_000_000, (10, 20, math.pi / 2), (10, 20.1, math.pi / 2)
        )
        metrics.forecast((1, 1), 0, 100_000_000, (0, 0, 0), (9, 0, 0))
        metrics.collect([(0, 3, 4, 0), (100_000_000, 3.1, 4, 0)])
        report = metrics.summary()
        self.assertEqual(report["moving_intervals"], 1)
        self.assertLess(report["moving_translation_error_m"]["max"], 1e-12)

    def test_real_prediction_error_and_missing_truth(self):
        metrics = ModelMetrics()
        metrics.forecast((1, 1), 0, 100_000_000, (0, 0, 0), (0.1, 0, 0.1))
        metrics.collect([(0, 0, 0, 0)])
        self.assertEqual(metrics.summary()["pending_intervals"], 1)
        metrics.collect([(0, 0, 0, 0), (100_000_000, 0.07, 0, 0.06)])
        report = metrics.summary()
        self.assertAlmostEqual(report["moving_translation_error_m"]["max"], 0.03)
        self.assertAlmostEqual(report["moving_yaw_error_rad"]["max"], 0.04)
        metrics.forecast((1, 2), 110_000_000, 210_000_000, (0, 0, 0), (0.1, 0, 0))
        metrics.collect([(200_000_000, 0, 0, 0), (220_000_000, 0, 0, 0)])
        self.assertEqual(metrics.summary()["unmatched_intervals"], 1)

    def test_invalid_prediction_and_session_change(self):
        metrics = ModelMetrics()
        for end in (0, 200_000_000):
            with self.assertRaises(ValueError):
                metrics.forecast((1, 1), 0, end, (0, 0, 0), (1, 0, 0))
        metrics.forecast((1, 1), 0, 100_000_000, (0, 0, 0), (0.1, 0, 0))
        metrics.forecast((2, 1), 0, 100_000_000, (0, 0, 0), (0.1, 0, 0))
        self.assertEqual(metrics.summary()["unmatched_intervals"], 1)
        self.assertEqual(metrics.summary()["pending_intervals"], 1)

    def test_unpredicted_motion_is_not_hidden_by_stationary_samples(self):
        metrics = ModelMetrics()
        metrics.forecast((1, 1), 0, 100_000_000, (0, 0, 0), (0, 0, 0))
        metrics.collect([(0, 0, 0, 0), (100_000_000, 0.01, 0, 0)])
        self.assertEqual(metrics.summary()["moving_intervals"], 1)
        self.assertEqual(metrics.summary()["moving_translation_error_m"]["max"], 0.01)


if __name__ == "__main__":
    unittest.main()
