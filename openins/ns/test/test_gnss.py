import numpy as np
import unittest


from openins.ns.gnss import SimpleGNSS
from openins.visualisation.plotter import plot_compare_states, plot_compare_states_diff
from openins.trajectory.navtrajectory_opt import NavTrajectoryOpt

class SimpleGNNSTest(unittest.TestCase):
    """
    Test each method in SimpleINSComputer.
    """

    def setUp(self):
        """
        Make sure that every test runs on clean trajectory.
        """

        self.pf = NavTrajectoryOpt()
        self.dt = 1.
        self.gps = SimpleGNSS()
        self.gps.dt = self.dt

        self.time = np.arange(0., 100.,  self.dt)

    def test_gnss(self):

        ref_state = np.array([self.pf.state(t)[:6]  for t in self.time])
        gps_state = np.array([self.gps(self.pf.state(t)[:6])  for t in self.time])

        # Uncoment if you want to see plots
        # lgnd = ['phi', 'lam', 'h', 'vn', 've', 'vd']
        # plot_compare_states(self.time, gps_state, ref_state, lgnd)
        # plot_compare_states_diff(self.time, gps_state, ref_state, lgnd)

        gps_std = np.std(gps_state-ref_state, axis=0)
        ref_gps_std = self.gps._noise_std
        np.testing.assert_allclose(gps_std, ref_gps_std, rtol=1)
