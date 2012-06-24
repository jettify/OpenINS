#!/usr/bin/python
"""
Test plotting functions.
"""
import numpy as np
import unittest

from openins.visualisation.plotter import *
from openins.tools.analysis import avar
from openins.trajectory.navtrajectory import NavTrajectory

#@unittest.skip
class PlotterTest(unittest.TestCase):
    """
    Plotter test container.
    """

    def setUp(self):
        """
        Lets define some variances for diferent noise models in inertial sensors.
        """

        self.data = 1 * np.random.randn(10000) + 0
        self.sigma, self.ltime = avar(self.data, dt=0.1)

        self.time = np.arange(0,10,0.1)
        self.val1 = np.sin(self.time)
        self.val2 = np.cos(self.time)
        self.val3 = 0.01*self.time


    def test_plot_avar(self):
        """
        Basic test for allan variance plotter.
        """
        plot_avar( self.ltime, self.sigma)



    def test_plot_trajectory(self):
        """
        Basic test for 3D trajectory visualisation.
        """
        
        plot_trajectory(self.val1, self.val2, self.val3)

    def test_plot_euler(self):
        """
        Basic test for Euler angles plotter.
        """
        plot_euler(self.time, self.val1, self.val2, self.val3)

    def test_plot_trinity(self):
        """
        Basic test 3 var plotter
        """
        plot_trinity(self.time,np.array([self.val1, self.val2, self.val3]).T)


class PlotterINSStateTest(unittest.TestCase):
    """
    Plotter test container.
    """
    # Setup flight profile
    pd = np.array([[55.0 * (np.pi / 180.0), 0.0, 0.004, 2. * np.pi / 600.0, 0.],
        [30.0 * (np.pi / 180.0), 0.00004, 0, 2. * np.pi / 600.0, 0.],
        [5000.0, 0.0, -5000.0, 2. * np.pi / 600.0, 0.]])

    profile = NavTrajectory(pd)

    # define sample time and time vector
    dt = 1.
    time = np.arange(0., 1000., dt)


    def test_plot_ins_state(self):
        # compute all state for given time
        state = np.array([self.profile.state(t) for t in self.time])
        # plot data
        plot_ins_state(self.time, state)
