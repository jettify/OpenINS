import unittest
import numpy as np

from openins.trajectory.navtrajectory_opt import NavTrajectoryOpt
from openins.trajectory.navtrajectory import NavTrajectory
from openins.visualisation.plotter import plot_compare_states_diff, plot_compare_states



class NavTrajectoryOptTest(unittest.TestCase):
    """
    Compare NavTrajectoryOpt and NavTrajectory.

    This trajectory generator should give same result
    but NavTrajectoryOpt considerably faster.
    """
    pf_opt = NavTrajectoryOpt()
    pf_reg = NavTrajectory()

    dt = 1.
    time = np.arange(0., 100.,  dt)

    def test_functions(self):
        ref_state = np.array([self.pf_reg.state(t) for t in self.time])
        test_state = np.array([self.pf_opt.state(t) for t in self.time])

        lgnd = ['phi', 'lam', 'h', 'vn', 've', 'vd', 'an', 'ae', 'ad',
                 'q0', 'q1', 'q2', 'q3']
        plot_compare_states(self.time, test_state, ref_state, lgnd)
        plot_compare_states_diff(self.time, test_state, ref_state, lgnd)

        np.testing.assert_almost_equal(test_state, ref_state, decimal=10)

    def test_gyros(self):
        ref_gyros = np.array([self.pf_reg.gyros(t) for t in self.time])
        test_gyros = np.array([self.pf_opt.gyros(t) for t in self.time])

        ## uncomment lines if you to check the plot and diff between
        ## test and ref values
        # lgnd = ['wx', 'wy', 'wz']
        # plot_compare_states(self.time, test_gyros, ref_gyros, lgnd)
        # plot_compare_states_diff(self.time, test_gyros, ref_gyros, lgnd)

        np.testing.assert_almost_equal(test_gyros, ref_gyros, decimal=10)

    def test_accs(self):
        ref_accs = np.array([self.pf_reg.accs(t) for t in self.time])
        test_accs = np.array([self.pf_opt.accs(t) for t in self.time])

        ## uncomment lines if you to check the plot and diff between
        ## test and ref values
        lgnd = ['ax', 'ay', 'az']
        plot_compare_states(self.time, test_accs, ref_accs, lgnd)
        plot_compare_states_diff(self.time, test_accs, ref_accs, lgnd)

        np.testing.assert_almost_equal(test_accs, ref_accs, decimal=10)


if __name__ == '__main__':
    unittest.main()