import math
from dataclasses import replace
from pathlib import Path
import unittest

from hexapod.calibration import CalibrationError, PwmMapper, angle_to_pwm
from hexapod.config import WIRE_CHANNELS, load_config
from hexapod.gait import GaitPlanner, Twist, support_margin
from hexapod.planning import simulate_gait


CONFIG = Path(__file__).resolve().parents[1] / "config" / "hexapod.toml"


class CalibrationAndGaitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(CONFIG)

    def test_channel_order_and_centers(self):
        self.assertEqual(WIRE_CHANNELS[0:3], ("FL.Tibia", "FL.Femur", "FL.Coxa"))
        self.assertEqual(WIRE_CHANNELS[-3:], ("FR.Tibia", "FR.Femur", "FR.Coxa"))
        self.assertEqual(PwmMapper(self.config).centers(), (1500,) * 18)

    def test_invalid_pwm_is_rejected_not_clipped(self):
        item = self.config.servos["FL.Coxa"]
        with self.assertRaises(CalibrationError):
            angle_to_pwm(item, item.center_geometry + math.radians(46))

    def test_support_polygon_margin(self):
        square = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        self.assertAlmostEqual(support_margin(square, (0.0, 0.0)), 1.0)
        self.assertLess(support_margin(square, (2.0, 0.0)), 0.0)

    def test_full_gait_and_stop_are_within_pwm_limits(self):
        result = simulate_gait(self.config)
        self.assertEqual(result["legs_transferred"], 6)
        self.assertGreaterEqual(result["minimum_pwm_us"], 1000)
        self.assertLessEqual(result["maximum_pwm_us"], 2000)
        self.assertLess(result["return_error_m"], 1e-9)

    def test_missing_verified_contact_has_bounded_search_and_latches_hold(self):
        config = replace(
            self.config,
            contact_levels_verified=True,
            contact_active_high=True,
            contact_search_limit=0.001,
        )
        planner = GaitPlanner(config)
        for _ in range(40):
            planner.update(Twist(config.vx_max, 0.0, 0.0), 1.0 / config.control_hz, contact_mask=0)
        self.assertIsNotNone(planner.fault_reason)
        self.assertIn("no contact within", planner.fault_reason)
        frozen = dict(planner.feet)
        planner.update(Twist(config.vx_max, 0.0, 0.0), 1.0 / config.control_hz, contact_mask=0)
        self.assertEqual(planner.feet, frozen)


if __name__ == "__main__":
    unittest.main()
