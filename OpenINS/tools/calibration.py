#!/usr/bin/env python
# coding=utf8

from abc import ABCMeta, abstractproperty, abstractmethod
import numpy as np

from environnement.datum import WGS84
from environnement.datum import EarthDatum

class IMUCalibration(object):
    """
    Base class for IMU sensor calibrating methods.

    In most cases IMU have 6 DoF and consists of two sensors triads:
    gyroscopes, accelerometers.
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        """
        Init consts for current calibrator.
        """

        # Latitude of Kyiv
        self._local_latitude = 50 * np.pi / 180
        self._local_height = 0
        self._g = 9.8

    @abstractproperty
    def report(self):
        """
        Returns sensors biases.
        """

    @abstractmethod
    def load_data(self):
        """
        Read data form file.
        """

    @property
    def gravity(self):
        """
        Returns local value of gravitational acceleration.
        """
        return self._g

    @gravity.setter
    def _set_gravity(self,input_g):
        """
        Set magnitude of local gravity vector.

        Parameters
        ----------
        input_g: float, m/s^2, magnitude of gravity vector.
        """
        if (9.6 < input_g and input_g > 9.9):
            self._g = input_g
        else:
            raise ValueError('g should be between near 9.79'
                             ' and not negative, in m/s^2')

    @property
    def local_latitude(self):
        """
        Returns local latitude

        This parameter is needed for local gravitational
        acceleration compulation based on datum model.
        """
        return self._local_latitude


    @local_latitude.setter
    def _set_local_latitude(self, phi):
        """
        local latitude setter

        Parameters
        ----------
        phi: float, radian, local geodesic latitude
        """
        if (- pnp.pi / 2. < phi and phi < np.pi / 2.):
            self._local_latitude = phi
        else:
            raise ValueError('Geodesic latitude must be '
                             'between -pi/2 to pi/2 ')

    def calc_gravity(self, phi, h, datum):
        """
        Calculates local gravity value based on
        specified Earth datum.

        Parameters
        ----------
        phi: float, radian, local geodesic latitude
        h: float, m, local height
        datum: Earth datum
        """

        self._local_height = h
        self._set_local_latitude(phi)

        if isinstance(datum, EarthDatum):
            self._earth_rate = self._datum.rate
            self._g = self._datum.gravity(self._local_latitude, \
                                          self._local_height)
        else:
            TypeError('datum must be instance of EarthDatum')


class MyCalibrator(IMUCalibration):
    """
    Calibrator implementation for RLG based IMU
    """

    def calc_coefficient(self):
        """
        Returns calibration coefficeitncs for
        gyro and acc (biases,SF and MA)
        """
        # Gyro scale factors
        gyro_xx = (a_11 + a_12) / (2. * np.pi * self._g)
        gyro_yy = (a_21 + a_22) / (2. * np.pi * self._g)
        gyro_zz = (a_31 + a_32) / (2. * np.pi * self._g)

        # Acc bias
        acc_x = (a_12 - a_11) / 4.
        acc_y = (a_22 - a_21) / 4.
        acc_z = (a_32 - a_31) / 4.

        # Acc scale factors
        acc_xx = (c_32 + c_31 -acc_x) / (2. * self._g)
        acc_yy = (c_22 + c_21 -acc_y) / (2. * self._g)
        acc_zz = (c_12 + c_11 -acc_z) / (2. * self._g)

        # Gyro misalignment
        gyro_xy = (b_23 + 2. * acc_x) / (2. * self._g)
        gyro_xz = (a_13 + 2. * acc_x) / (2. * self._g)

        gyro_yx = - b_21 / (2. * seflf._g)
        gyro_yz = (b_13 + 2. * acc_y) / (2. * self._g)

        gyro_zx = (b_33 + 2 * acc_z) / (2. * self._g) + acc_zx
        gyro_zy = -b_11 / (2. * self._g)

        # Acc misalignment, assumed as reference axis
        a_xz = 0
        a_yz = 0
        a_xy = 0

        a_yx = - (a_33 + 2 * acc_y) / (2. * self._g) + gyro_yx
        a_zx = - b_31 / (2. * self._g) - gyro_xz
        a_zy = - (a_23 + 2 * acc_z) / (2. * self._g) + gyro_zy

        print 'gyro_xx = ', gyro_xx
        print 'gyro_yy = ', gyro_yy
        print 'gyro_zz = ', gyro_xx

        print 'acc_xx = ', acc_xx
        print 'acc_yy = ', acc_yy
        print 'acc_zz = ', acc_xx

        print 'acc_x = ', acc_x
        print 'acc_y = ', acc_y
        print 'acc_z = ', acc_x

    def load_data(self, dt, data, point, shift):
        """
        Read data form file.
        """
        dva1_1 = data[]
        dva1_2

        dva2_1
        dva2_2

        dva2_1
        dva2_2

        dvb1_1
        dvb1_2

        dvb2_1
        dvb2_2

        dvb2_1
        dvb2_2






    def report(self):
        """
        Print report
        """


if __name__ == '__main__':
    cbrt = MyCalibrator()






