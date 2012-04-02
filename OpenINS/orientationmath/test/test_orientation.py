#!/usr/bin/env python
# coding=utf8

import random
import numpy as np
import unittest
from numpy import pi

from orientationmath.orientation import  *


# SET OF REFERENCE CONTSTS

# reference values: DCM
refDCM = np.matrix([[0.572061402817684, 0.201527081218441,  0.795067661863969],
                    [0.415626937777453, 0.764451983799883, -0.492815800333310],
                    [-0.707106781186547, 0.612372435695795, 0.353553390593274]])

refupdDCM = np.matrix([[0.571962824169272, 0.201434989062221, 0.795161916184254],
                       [0.415954809761541, 0.764277965291772, -0.492809078655932],
                       [-0.706993722747629, 0.612619895980348, 0.353350730923932]])

# reference values: angles, related for previous defined DCM
refGamma = pi/3.
refTheta = pi/4.
refPsi = pi/5.

# reference gyro rate

gyro_rrate = 0.01*0.01 #rad/sec
gyro_prate = 0.02*0.01 #rad/sec
gyro_yrate = 0.03*0.01 #rad/sec

# reference values: quaternions, related for previous defined angles
refq1 = 0.820071151975675
refq2 = 0.336918398289752
refq3 = 0.457940277325801
refq4 = 0.06526868310243991
#refq4 = 0.4292222551314542

refmq1 = 0.345033388605420
refmq2 = 0.552594118014552
refmq3 = 0.751087221525259
refmq4 = 0.107049928279506

# quaternion rotated with *gyro_rate* speed on dt
refupdq1 = 0.819998707374740
refupdq2 = 0.337021560123966
refupdq3 = 0.457975002101253
refupdq4 = 0.065402487458216

class DCMTest(unittest.TestCase):
    """
    Unit test for DCM matrix module
    """
    def test_euler2dcm(self):
        """
        Check for known output
        """
        testDCM = euler2dcm(refGamma,refTheta,refPsi)
        for i in [0,1,2]:
            for j in [0,1,2]:
                self.assertAlmostEqual(testDCM[i,j],refDCM[i,j],places=12)



class DCM2AnglesTest(unittest.TestCase):
    """
    Test angles calculation from DCMbn
    """
    def test_dcm2pitch(self):
        """
        Check for known values
        """
        testTheta = dcm2pitch(refDCM)
        self.assertAlmostEqual(testTheta,refTheta,places=12)


    def test_dcm2roll(self):
        """
        Check for known values
        """
        testGamma = dcm2roll(refDCM)
        self.assertAlmostEqual(testGamma,refGamma,places=12)

    def test_dcm2euler(self):
        """
        Check for known values
        """
        testGamma, testTheta, testPsi = dcm2euler(refDCM)

        self.assertAlmostEqual(testTheta,refTheta,places=12)
        self.assertAlmostEqual(testGamma,refGamma,places=12)
        self.assertAlmostEqual(testPsi,refPsi,places=12)

class Dcm2QuaternionTest(unittest.TestCase):

    def test_dcm2quat(self):
        """
        Check for known values
        """
        testq1,testq2,testq3,testq4 = dcm2quat(refDCM)

        self.assertAlmostEqual(testq1,refq1,places=12)
        self.assertAlmostEqual(testq2,refq2,places=12)
        self.assertAlmostEqual(testq3,refq3,places=12)
        self.assertAlmostEqual(testq4,refq4,places=12)

    def test_eulr2quat(self):
        """
        Check for known values
        """
        testq1,testq2,testq3,testq4 = euler2quat(refGamma,refTheta,refPsi)

        self.assertAlmostEqual(testq1,refq1,places=12)
        self.assertAlmostEqual(testq2,refq2,places=12)
        self.assertAlmostEqual(testq3,refq3,places=12)
        self.assertAlmostEqual(testq4,refq4,places=12)

    def test_quat2dcm(self):
        """
        Check for known values
        """
        testDCM = quat2dcm(np.array([refq1,refq2,refq3,refq4]))
        #TODO: this is ugly
        for i in [0,1,2]:
            for j in [0,1,2]:
                self.assertAlmostEqual(testDCM[i,j],refDCM[i,j],places=12)

    def test_skew(self):
        """
        Check for known values
        """

        refVect_x = (random.random()-0.5)*10*pi/2
        refVect_y = (random.random()-0.5)*10*pi/2
        refVect_z = (random.random()-0.5)*10*pi/2
        ref_skew = np.array([[0,      -refVect_z, refVect_y],
            [refVect_z, 0,       -refVect_x],
            [-refVect_y,refVect_x, 0]])

        test_skew = skew(np.array([refVect_x,refVect_y,refVect_z]))
        for i in [0,1,2]:
            for j in [0,1,2]:
                self.assertAlmostEqual(test_skew[i,j],ref_skew[i,j],places=12)


    def test_quat_mult(self):
        """
        Check for known values
        """
        q1 = np.array([refq1,refq2,refq3,refq4])
        testquat = quat_mult(q1,q1)

        self.assertAlmostEqual(testquat[0],refmq1,places=12)
        self.assertAlmostEqual(testquat[1],refmq2,places=12)
        self.assertAlmostEqual(testquat[2],refmq3,places=12)
        self.assertAlmostEqual(testquat[3],refmq4,places=12)

    def test_dcm_prop(self):
        """
        Check for known values
        """
        testDCM = dcm_prop(refDCM, np.array([gyro_rrate, gyro_prate, gyro_yrate]))
        for i in [0,1,2]:
            for j in [0,1,2]:
                self.assertAlmostEqual(testDCM[i,j],refupdDCM[i,j],places=12)

    def test_quat_prop_1o(self):
        """
        Check for known values
        """
        q = np.array([refq1, refq2, refq3, refq4])
        gyro_output = np.array([gyro_rrate, gyro_prate, gyro_yrate])
        testquat = quat_prop_1o(q, gyro_output)

        self.assertAlmostEqual(testquat[0], refupdq1, places=12)
        self.assertAlmostEqual(testquat[1], refupdq2, places=12)
        self.assertAlmostEqual(testquat[2], refupdq3, places=12)
        self.assertAlmostEqual(testquat[3], refupdq4, places=12)

    def test_quatprop_4o(self):
        """
        Check for known values
        """

        q = np.array([refq1,refq2,refq3,refq4])
        gyro_output = np.array([gyro_rrate, gyro_prate, gyro_yrate])
        testquat = quat_prop_4o(q, gyro_output)

        # self.assertAlmostEqual(testquat[0],refupdq1,places=12)
        self.assertAlmostEqual(testquat[1], refupdq2, places=12)
        self.assertAlmostEqual(testquat[2], refupdq3, places=12)
        self.assertAlmostEqual(testquat[3], refupdq4, places=12)

class ModuleCrossTest(unittest.TestCase):
    def test_dcm_euler(self):

#        refGamma = random.random()
#        refTheta = random.random()
#        refPsi = random.random()

        refGamma = -np.pi/2.
        refTheta = -np.pi/2.
        refPsi = 0

        testDCM = euler2dcm(refGamma, refTheta, refPsi)
        testGamma, testTheta, testPsi = dcm2euler(testDCM)

        self.assertAlmostEqual(testTheta, refTheta, places=12)
        self.assertAlmostEqual(testGamma, refGamma, places=12)
        self.assertAlmostEqual(testPsi, refPsi, places=12)

    def test_dcm_quat(self):

        testq1,testq2,testq3,testq4 = dcm2quat(refDCM)
        testDCM = quat2dcm(np.array([testq1, testq2, testq3, testq4]))

        for i in [0,1,2]:
            for j in [0,1,2]:
                self.assertAlmostEqual(testDCM[i,j],refDCM[i,j],places=12)

if __name__ == '__main__':
    unittest.main()


