#!/usr/bin/env python
# coding=utf8

import unittest
import numpy as np

from environnement.datum import WGS84
from environnement.datum import InitPosition



#Constants for WGS84 datum

##
refa = 6378137.0
refb = 6356752.3142

refe = 0.0818191909289
refe2 = 0.00669438000426

reff = 0.00335281067183

refrate = 7.292115E-5

refg = 9.826847049616367

refRE = 6.390702044221264e+06
refRN = 6.372955925724127e+06


class test_WGS84TestCase(unittest.TestCase):
    """
    Unit test for EarthDatum class
    """

    def setUp(self):
        self.wgs84 = WGS84()

    def test_WGS84_init(self):
        self.assertAlmostEqual(self.wgs84.a, refa, places=12)
        self.assertAlmostEqual(self.wgs84.b, refb, places=12)
        self.assertAlmostEqual(self.wgs84.e, refe, places=12)
        self.assertAlmostEqual(self.wgs84.e2,  refe2, places=12)
        self.assertAlmostEqual(self.wgs84.f, reff, places=12)
        self.assertAlmostEqual(self.wgs84.rate, refrate, places=12)

    def test_gravity(self):

        testg = self.wgs84.gravity(50 * (np.pi / 180), 100.0)
        self.assertAlmostEqual(testg, refg, places=12)

    def test_curvature_knowninput(self):
        testRE,testRN = self.wgs84.curvature(50 * (np.pi / 180))
        self.assertAlmostEqual(testRN,refRN,places=8)
        self.assertAlmostEqual(testRE,refRE,places=8)

    def test_dcurvature_knowninput(self):
        testRE,testRN,testdRE,testdRN = self.wgs84.dcurvature(50*(np.pi/180),0.0)
        self.assertAlmostEqual(testRE,refRE,places=8)
        self.assertAlmostEqual(testRN,refRN,places=8)
        self.assertAlmostEqual(testdRE,0,places=8)
        self.assertAlmostEqual(testdRN,0,places=8)

class InitPositionTest(unittest.TestCase):
    """
    Test fo init position class
    """

    def setUp(self):
        """
        Setup instance of class.
        """
        self.pos = InitPosition(np.deg2rad(50), np.deg2rad(30), 100.)
        self.wgs84 = WGS84()

    def properties_test(self):
        """
        Test properties.
        """
        self.assertAlmostEqual(self.pos.datum.rate, self.wgs84.rate, places=12)




if __name__ == '__main__':
    unittest.main()
