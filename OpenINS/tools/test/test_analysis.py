#!/usr/bin/python
"""
Functions for analysis of sensors stochastic model.
"""

import unittest

#import sys
#sys.path.append("/home/nickolai/OpenINS")
#print sys.path


from tools.analysis import *

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

         
    def test_avar(self):

        # Quantisation Noise 
        dataQ = self.Q * np.diff(self.data) / self.h
        # Random Walk (angular or velosity)
        dataN = self.N*self.data[:-1] / np.sqrt(self.h)
#        Bias Instability, flicker noise, first order Gauss-Markov model
#        dataB = np.zeros(len(self.data[:-2])+1)
#        for i in range(len(self.data[:-2])):
#            dataB[i+1] =  (1-self.beta*self.h)*dataB[i] + self.beta*self.B*self.data[i]
        dataK = self.K * np.cumsum(self.data[:-1]) * np.sqrt(self.h)
        
        #Implement Gauss-Markov Model
        #dataR = 
        
        # total variance
        dataT = dataQ + dataN + dataK  # +dataB +dataR
        
        # Allan variance (deviation) for different type of inertial sensors
        avar(dataN,dt=self.h, ppd=20.)
        # should see line (curve) with -1 slope

        sigma, time = avar(dataN, dt=self.h, ppd=100.)
        # should see line (curve) with -0.5 slope

        #TODO: implemetn 1st and 2dn order Gauss-Markov models of noise
        #sigma, time = avar(dataB,dt=self.h,ppd=100.)
        #sigma, time = avar(dataR,dt=self.h,ppd=100.)
        
        sigma, time = avar(dataK, dt=self.h, ppd=100.)
        # should see line (curve) with 0.5 slope

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.test_avar']
    unittest.main()
