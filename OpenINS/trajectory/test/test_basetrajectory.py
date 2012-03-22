import numpy as np
import unittest

from trajectory.basetrajectory import RotateIMU
from trajectory.basetrajectory import RotationSequence
from trajectory.basetrajectory import BasicCalibTraj

from visualisation.plotter import plot_trinity
class RotateIMUTest(unittest.TestCase):
    """
    Unit test for DCM matrix module
    """
    
    def setUp(self):
        self.time = 20
        self.angle = np.array([np.pi, 0., 0.])
        self.move = RotateIMU(self.time, np.pi, 0., 0.)
        
        self.test_time = 10

    def test_calc_rate(self):
        """
        Check for known output
        """
        test_rate = self.move.rate(self.time)
        test_angle = self.move.angle(self.time)

        np.testing.assert_almost_equal(test_angle, self.angle, decimal=12)

    def test_is_right_time(self):
        """
        Test for right time of function.
        """

        test_b = self.move.is_right_time(10)
        self.assertTrue(test_b)
        
        test_b = self.move.is_right_time(-5) 
        self.assertFalse(test_b)
        
class RotationSequenceTest(unittest.TestCase):
    """
    Test rotation sequence
    """
    def setUp(self):
        """
        Setup rotation sequences
        """
        self.turn_time = 20.
        self.stady_time = 10.
        self.rot_1 = RotateIMU(self.stady_time, 0., 0., 0.)
        self.rot_2 = RotateIMU(self.turn_time, np.pi, 0., 0.)
        self.rot_3 = RotateIMU(self.stady_time, 0., 0., 0.)
        self.rot_4 = RotateIMU(self.turn_time, np.pi, 0., 0.)

        self.rot_5 = RotateIMU(self.stady_time, 0., 0., 0.)
        self.rot_6 = RotateIMU(self.turn_time, 0., 0., np.pi)
        self.rot_7 = RotateIMU(self.stady_time, 0., 0., 0.)



        self.rs = RotationSequence()
        self.rs.add(self.rot_1) 
        self.rs.add(self.rot_2) 
        self.rs.add(self.rot_3) 
        self.rs.add(self.rot_4) 
        self.rs.add(self.rot_5) 
        self.rs.add(self.rot_6)
        self.rs.add(self.rot_7)

    def test_run(self):
        """
        Test calibration table movement.
        """
        time = np.arange(0., 100., 1.)
        a= np.array(zip(*map(self.rs.run, time)))

        #a = np.array([a])
        leg_a = ['roll', 'pitch', 'yaw']
        plot_trinity(time, a[0], lgnd=leg_a)

        leg_a = ['wx', 'wy', 'wz']
        plot_trinity(time, a[1], lgnd=leg_a)

class BasicCalibTrajTest(unittest.TestCase):
    """
    Test basic calibration trajectory generator.
    """
    def setUp(self):
        """
        Setup rotation sequences
        """
        self.turn_time = 20.
        self.stady_time = 10.
        rot_1 = RotateIMU(self.stady_time, 0., 0., 0.)
        rot_2 = RotateIMU(self.turn_time, np.pi, 0., 0.)
        rot_3 = RotateIMU(self.stady_time, 0., 0., 0.)
        rot_4 = RotateIMU(self.turn_time, np.pi, 0., 0.)

        rot_5 = RotateIMU(self.stady_time, 0., 0., 0.)
        rot_6 = RotateIMU(self.turn_time, 0., 0., np.pi)
        rot_7 = RotateIMU(self.stady_time, 0., 0., 0.)

        self.rs = RotationSequence()
        self.rs.add(rot_1)
        self.rs.add(rot_2)
        self.rs.add(rot_3)
        self.rs.add(rot_4)
        self.rs.add(rot_5)
        self.rs.add(rot_6)
        self.rs.add(rot_7)

        self.time = np.arange(0., 100., 0.1)
        self.fixture = BasicCalibTraj(self.rs)

    def test_gyros_plot(self):
        """
        Plot gyros versus time.
        """
        gyros = np.array(map(self.fixture.gyros, self.time))
        leg_a = ['wx', 'wy', 'wz']
        plot_trinity(self.time, gyros, lgnd=leg_a)

    def test_accs_plot(self):
        """
        Plot acceleration versus time.
        """
        accs = np.array(map(self.fixture.accs, self.time))
        leg_a = ['ax', 'ay', 'az']
        plot_trinity(self.time, accs, lgnd=leg_a)

    def test_accs_abs(self):
        """
        Absolute acceleration, measured by 3 orgtogonal
        accelerometers must be equal to local gravity acceleration.
        """
        accs = np.array(map(self.fixture.accs, self.time))
        [self.assertAlmostEqual(np.sqrt(np.sum(a**2)),
            self.fixture._g, places=8)  for a in accs]

    def test_gyros_abs(self):
        """
        Angular rate measured by gyros, include Earth rate, so
        gyros rate minus true rate must be equal to Earth rate projected
        on body frame. Lets find absolute value of Earth rate and test it.
        """
        a= np.array(zip(*map(self.rs.run, self.time)))
        gyros = np.array(map(self.fixture.gyros, self.time))
        earth_rate = gyros - a[1]
        earth_abs = [np.sqrt(np.sum(rate**2)) for rate in earth_rate]
        [self.assertAlmostEqual(i, self.fixture.datum.rate) for i in earth_abs]

if __name__ == '__main__':
    unittest.main()

