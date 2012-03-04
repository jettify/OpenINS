import unittest

from tools.calibration import IMUCalibration

class TestIMUCalibration(unittest.TestCase):

    def setUp(self):
        pass

    def test_abstract_methods(self):
        """
        Try to instantiate abstract class
        """

        with self.assertRaises(TypeError):
            imu = IMUCalibration()
