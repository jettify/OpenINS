#!/usr/bin/python
"""
Functions for analysis of sensors stochastic model.
"""

import unittest

from tools.analysis import *
from visualisation.plotter import plot_avar

class AnalysisTest(unittest.TestCase):

    def setUp(self):
        """
        Lets define some variances for diferent noise models in inertial sensors.
        """
        # Sample tim
        self.h = 0.01
        # Quantisation Noise variance
        self.Q = 15
        # Random Walk variance
        self.N = 15
        # Bias Instability variance
        self.B = 10
        # Rate random walk variance
        self.K = 15
        # Rate ramp variance
        self.R = 15
        
        # time const
        self.beta = 0.01

        # unit zero-mean white gauss noise
        self.data = 1 * np.random.randn(100000) + 0
        self.dataQ = self.Q * np.diff(self.data) / self.h
        self.dataN = self.N*self.data[:-1] / np.sqrt(self.h)
        self.dataK = self.K * np.cumsum(self.data[:-1]) * np.sqrt(self.h)

    def test_avar_q(self):
        """
        Quantisation Noise
        """
        # should see line (curve) with -1 slope
        time, sigma = avar(self.dataQ,dt=self.h, ppd=20.)
        plot_avar(time, sigma)

    def test_avar_n(self):
        """
        Random Walk (angular or velosity)
        """
        # should see line (curve) with -0.5 slope
        sigma, time = avar(self.dataN, dt=self.h, ppd=50.)
        plot_avar(time, sigma)

    def test_avar_k(self):
        """
        Rate random walk.
        """
        # should see line (curve) with 0.5 slope
        sigma, time = avar(self.dataK, dt=self.h, ppd=100.)
        plot_avar(time, sigma)


    def test_avar(self):
#        Bias Instability, flicker noise, first order Gauss-Markov model
#        dataB = np.zeros(len(self.data[:-2])+1)
#        for i in range(len(self.data[:-2])):
#            dataB[i+1] =  (1-self.beta*self.h)*dataB[i] + self.beta*self.B*self.data[i]

        #Implement Gauss-Markov Model
        #dataR = 
        
        # total variance
        self.dataT = self.dataQ + self.dataN + self.dataK  # +dataB +dataR
        sigma, time = avar(self.dataT, dt=self.h, ppd=100.)
        plot_avar(time, sigma)

        #TODO: implement 1st and 2dn order Gauss-Markov models of noise
        #sigma, time = avar(dataB,dt=self.h,ppd=100.)
        #sigma, time = avar(dataR,dt=self.h,ppd=100.)
        


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.test_avar']
    unittest.main()
