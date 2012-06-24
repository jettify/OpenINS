import numpy as np
import unittest

from openins.trajectory.calibtrajectory import RotateIMU
from openins.trajectory.calibtrajectory import RotationSequence
from openins.trajectory.calibtrajectory import BasicCalibTraj
from openins.orientationmath.orientation import euler2dcm
from openins.orientationmath.orientation import dcm2euler
from openins.visualisation.plotter import plot_trinity

class RotateIMUTest(unittest.TestCase):
    """
    Unit test for DCM matrix module
    """

    def setUp(self):
        self.time = 20
        self.a = np.array([np.pi, 0., 0.])
        self.move = RotateIMU(self.time, self.a[0], self.a[1], self.a[2])
        self.test_time = 10

    def test_calc_rate(self):
        """
        Check for known output
        """
        test_rate = self.move.rate(self.time)
        ref_rate = self.a/self.time
        result = np.ma.allequal(test_rate, ref_rate)
        self.assertTrue(result)

    def test_is_right_time(self):
        """
        Test for right time of function.
        """

        test_b = self.move.is_right_time(10)
        self.assertTrue(test_b)
        test_b = self.move.is_right_time(-5)
        self.assertFalse(test_b)

    def test_orientation_dcm(self):
        """
        Test orientation_dcm.
        """
        test_dcm = self.move.orientation_dcm(self.time)
        ref_dcm = euler2dcm(self.a[0], self.a[1], self.a[2])
        result = np.ma.allclose(test_dcm, ref_dcm, atol=1e-08)
        self.assertTrue(result)


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
        a = zip(*map(self.rs.run, time))

        #a = np.array([a])
        leg_a = ['roll', 'pitch', 'yaw']
        angle = [dcm2euler(dcm) for dcm in a[0]]

        plot_trinity(time, angle, lgnd=leg_a)

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

    def test_accs_abs(self):
        """
        Absolute acceleration, measured by 3 orgtogonal
        accelerometers must be equal to local gravity acceleration.
        """
        accs = np.array(map(self.fixture.accs, self.time))
        [self.assertAlmostEqual(np.sqrt(np.sum(a**2)),
            self.fixture.ipos.gravity, places=8)  for a in accs]


    def test_gyros_abs(self):
        """
        Angular rate measured by gyros, include Earth rate, so
        gyros rate minus true rate must be equal to Earth rate projected
        on body frame. Lets find absolute value of Earth rate and test it.
        """
        a = zip(*map(self.rs.run, self.time))
        gyros = np.array(map(self.fixture.gyros, self.time))
        earth_rate = gyros - np.array(a[1])
        earth_abs = [np.sqrt(np.sum(rate**2)) for rate in earth_rate]
        [self.assertAlmostEqual(i, self.fixture.ipos.datum.rate) for i in earth_abs]


class BasicCalibTrajTest2(unittest.TestCase):
    """
    Test basic calibration trajectory generator.
    """
    def setUp(self):
        """
        Setup rotation sequences
        """
        self.turn_time = 10.
        self.stady_time = 10.

        #Rotation set 1
        self.rs1 = RotationSequence()
        self.rs1.set_init_orientation(0., 0., 0. )

        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs1.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs1.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs1.add(RotateIMU(self.turn_time, 0., 0., np.pi))
        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))




        #Rotation set 2
        self.rs2 = RotationSequence()
        self.rs2.set_init_orientation( -np.pi/2., -np.pi/2,  0. )

        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs2.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs2.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs2.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))

        #Rotation set 3
        self.rs3 = RotationSequence()
        self.rs3.set_init_orientation(np.pi/2, 0., np.pi/2)

        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs3.add(RotateIMU(self.turn_time, 0., 0., np.pi))
        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs3.add(RotateIMU(self.turn_time, 0., 0., np.pi))
        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs3.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))

        self.time = np.arange(0., 70., 0.1)
        self.fixture1 = BasicCalibTraj(self.rs1)
        self.fixture2 = BasicCalibTraj(self.rs2)
        self.fixture3 = BasicCalibTraj(self.rs3)

    def test_gyros_plot(self):
        """
        Plot gyros versus time.
        """
        gyros1 = np.array(map(self.fixture1.gyros, self.time))
        leg_a = ['wx-f1', 'wy', 'wz']
        plot_trinity(self.time, gyros1, lgnd=leg_a)
#
        gyros2 = np.array(map(self.fixture2.gyros, self.time))
        leg_a = ['wx-f2', 'wy', 'wz']
        plot_trinity(self.time, gyros2, lgnd=leg_a)

        gyros3 = np.array(map(self.fixture3.gyros, self.time))
        leg_a = ['wx-f3', 'wy', 'wz']
        plot_trinity(self.time, gyros3, lgnd=leg_a)

    def test_accs_plot(self):
        """
        Plot acceleration versus time.
        """
        accs1 = np.array(map(self.fixture1.accs, self.time))
        leg_a = ['ax-f1', 'ay', 'az']
        plot_trinity(self.time, accs1, lgnd=leg_a)

        accs2 = np.array(map(self.fixture2.accs, self.time))
        leg_a = ['ax-f2', 'ay', 'az']
        plot_trinity(self.time, accs2, lgnd=leg_a)

        accs3 = np.array(map(self.fixture3.accs, self.time))
        leg_a = ['ax-f3', 'ay', 'az']
        plot_trinity(self.time, accs3, lgnd=leg_a)

#    def test_run(self):
#        """
#        Test calibration table movement.
#        """
#
#        rot_set1 = zip(*map(self.rs1.run, self.time))
#        leg_a = ['roll', 'pitch', 'yaw']
#        plot_trinity(self.time, rot_set1[0], lgnd=leg_a)
#
#
#        rot_set2 = np.array(zip(*map(self.rs2.run, self.time)))
#        leg_a = ['roll', 'pitch', 'yaw']
#        plot_trinity(self.time, rot_set2[0], lgnd=leg_a)
#
#        rot_set3 = np.array(zip(*map(self.rs3.run, self.time)))
#        leg_a = ['roll', 'pitch', 'yaw']
#        plot_trinity(self.time, rot_set3[0], lgnd=leg_a)
#


if __name__ == '__main__':
    unittest.main()





