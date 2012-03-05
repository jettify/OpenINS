#!/usr/bin/python
"""
Test plotting functions.
"""
import numpy as np
import unittest

from visualisation.plotter import *


class PlotterTest(unittest.TestCase):
    """
    Plotter test container.
    """

    def setUp(self):
        """
        Lets define some variances for diferent noise models in inertial sensors.
        """
        self.time = np.arange(0,10,0.1)
        self.val1 = np.sin(self.time)
        self.val2 = np.cos(self.time)
        self.val3 = 0.01*self.time
 
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

