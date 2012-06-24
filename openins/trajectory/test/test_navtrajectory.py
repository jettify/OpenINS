import numpy as np
import unittest

from openins.trajectory.navtrajectory import NavTrajectory
from openins.visualisation.plotter import plot_trinity, plot_trajectory

class NavTrajectoryTest(unittest.TestCase):
    """
    Unit test for DCM matrix module
    """
    pd = np.array([[55.0 * (np.pi / 180.0), 0.0, 0.0004, 2. * np.pi / 600.0, 0.],
        [30.0 * (np.pi / 180.0), 0.00004, 0., 2. * np.pi / 600.0, 0.],
        [1000.0, 0.0, -1000.0, 2. * np.pi / 600.0, 0.]])

    profile = NavTrajectory(pd=pd)
    time = np.arange(0., 1000., 1)


    def test_position(self):
        """
        Plot position versus time.
        """

        phi =  map(self.profile.phi, self.time)
        lam =  map(self.profile.lam, self.time)
        h =  map(self.profile.h, self.time)

        phi = (phi - phi[0])*6378137.0
        lam = (lam - lam[0])*6378137.0


        plot_trajectory(phi, lam, h)


    def test_ned_velocity(self):
        """
        Plot NED velocity versus time
        """

        vn =  map(self.profile.vn, self.time)
        ve =  map(self.profile.ve, self.time)
        vd =  map(self.profile.vd, self.time)

        lgnd = ['vn', 've', 'vd']
        plot_trinity(self.time, np.array([vn, ve, vd]).T, lgnd)


    def test_ned_acceleration(self):
        self.skipTest('temp')
        an =  map(self.profile.an, self.time)
        ae =  map(self.profile.ae, self.time)
        ad =  map(self.profile.ad, self.time)

        lgnd = ['an', 'ae', 'ad']
        plot_trinity(self.time,np.array([an, ae, ad]).T, lgnd)


    def test_accs(self):
        self.skipTest('temp')
        a = map(self.profile.accs, self.time)
        lgnd = ['ax', 'ay', 'az']
        plot_trinity(self.time, a, lgnd)


    def test_gyro(self):
        """
        Plot gyros angular velocity.
        """
        self.skipTest('temp')
        gyro = map(self.profile.gyros, self.time)
        lgnd = ['wx', 'wy', 'wz']
        print np.shape(gyro)
        plot_trinity(self.time, gyro, lgnd)

    @unittest.skip
    def test_orientation(self):
        """
        Plot Euler angles versus time.
        """
        self.skipTest('temp')
        gamma =  map(self.profile.gamma, self.time)
        theta =  map(self.profile.theta, self.time)
        psi =  map(self.profile.psi, self.time)

        lgnd = ['gamma', 'theta', 'psi']
        plot_trinity(self.time,np.array([gamma, theta, psi]).T, lgnd)