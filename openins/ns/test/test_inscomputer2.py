"""
Test for inscomputer module
"""
import numpy as np
import unittest
import pylab



from openins.ns.inscomputer import SimpleINSComputer
from openins.trajectory.navtrajectory import NavTrajectory
from openins.visualisation.plotter import plot_trinity, plot_trajectory, plot_basic
from openins.visualisation.plotter import plot_ins_state, plot_compare_states, plot_compare_states_diff
from openins.trajectory.navtrajectory_opt import navtrajectory_state, navtrajectory_statex
from openins.trajectory.navtrajectory_opt import NavTrajectoryOpt

from openins.orientationmath.orientation import euler2quat
from openins.orientationmath.orientation import dcm2quat
from openins.orientationmath.orientation import quat2dcm
from openins.orientationmath.orientation import dcm2euler


class ComprehensiveTest(unittest.TestCase):
    """
    Test each method in SimpleINSComputer.
    """

    def setUp(self):
        """
        Make sure that every test runs on clean trajectory.
        """

        self.pf = NavTrajectoryOpt()
        #        self.pf = NavTrajectory(pd)
        self.dt = 0.005
        self.ins = SimpleINSComputer()
        self.ins.dt = self.dt

        init_state = self.pf.init_state()
        self.ins.set_state(init_state)
        self.time = np.arange(0., 1000.,  self.dt)


