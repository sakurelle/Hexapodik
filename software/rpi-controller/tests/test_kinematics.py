import math
import random
import unittest

from hexapod.kinematics import Angles, KinematicsError, LegKinematics, Point


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.solver = LegKinematics((0.044, 0.080, 0.13978))

    def assertPointClose(self, first, second, places=10):
        self.assertAlmostEqual(first.x, second.x, places=places)
        self.assertAlmostEqual(first.y, second.y, places=places)
        self.assertAlmostEqual(first.z, second.z, places=places)

    def test_control_pose(self):
        angles = Angles(0.0, math.radians(35.2666666667), math.radians(-108.0833333333))
        self.assertPointClose(self.solver.forward(angles), Point(0.150613118, 0.0, -0.087350210), places=8)

    def test_documented_target(self):
        angles = self.solver.inverse(Point(0.150, 0.0, -0.100))
        self.assertAlmostEqual(math.degrees(angles.coxa), 0.0, places=6)
        self.assertAlmostEqual(math.degrees(angles.femur), 26.346439, places=6)
        self.assertAlmostEqual(math.degrees(angles.tibia), -102.137656, places=6)

    def test_random_fk_ik_fk(self):
        random.seed(12345)
        for _ in range(1000):
            angles = Angles(
                math.radians(random.uniform(-35, 35)),
                math.radians(random.uniform(5, 65)),
                math.radians(random.uniform(-135, -75)),
            )
            point = self.solver.forward(angles)
            recovered = self.solver.inverse(point)
            self.assertPointClose(self.solver.forward(recovered), point)

    def test_unreachable_and_nonfinite_rejected(self):
        with self.assertRaises(KinematicsError):
            self.solver.inverse(Point(1.0, 0.0, 0.0))
        with self.assertRaises(KinematicsError):
            self.solver.inverse(Point(float("nan"), 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
