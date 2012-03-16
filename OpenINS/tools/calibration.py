#!/usr/bin/env python
# coding=utf8

"""
Base classes for calibration procedure.
"""

from abc import ABCMeta, abstractproperty, abstractmethod 
import numpy as np 
from environnement.datum import WGS84 
#from environnement.datum import EarthDatum
from tools.profile import create_profile


class CalibrationSensorModel(object):
    """
    Basic model for sensors.
    """
    _bias = np.array([0., 0., 0.])
    _sfma = np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
    coefficients = {
        'bias': _bias,
        'sfma': _sfma,
        # sensors bias
        'x': _bias[0],
        'y': _bias[1],
        'z': _bias[2],
        # sensors scale factor               
        'xx': _sfma[0, 0],
        'yy': _sfma[1, 1],
        'zz': _sfma[2, 2],
        # sensors misalignment 
        'xy': _sfma[0, 1],
        'xz': _sfma[0, 2],

        'yx': _sfma[1, 0],
        'yz': _sfma[1, 2],

        'zx': _sfma[2, 0],
        'zy': _sfma[2, 1],
        }



    def __init__(self):
        """
        Construct sensor coeffs. 
        """
    
    def __getattr__(self, name):
        # maps getting of x, y, z, to computation from k
        try:
            coeff = self.coefficients[name]
        except KeyError:
            # unknown name, give error message
            raise AttributeError, name

        return coeff  

    def __setattr__(self, name, value):
        # maps settings of x, y, z, to setting of k; forbids others
       
        #if name == 'bias' and value.shape() != (1, 3):
        #    raise ValueError, value

        #if name == 'sfma' and value.shape() != (3, 3):
        #    raise ValueError, value

        try:
            self.coefficients[name] = value
        except KeyError:
            # unknown name, give error message
            raise AttributeError, name
        self.coefficients[name] = value



    #def __str__(self):
    #    # readable, concise representation as string
    #    return "%s K" % self.k

    #def __repr__(self):
    #    # detailed, precise representation as string
    #    return "Temperature(k=%r)" % self.k

    #@property
    #def bias(self):
    #    """
    #    Return sensor bias
    #    """
    #    return self._bias

    #@property
    #def sfma(self):
    #    """
    #    Return sensors scale factor/misalignment matrix
    #    """
    #    return self._sfma
    #
    #@bias.setter
    #def _set_bias(self, data):
    #    """
    #    Bias sensor setter.

    #    Parameters
    #    ----------
    #    data: 1x3 array, float, sensors bias
    #    """
    #    if data.shape() != (1, 3):
    #        raise ValueError('Vector size should be 3x3')
    #    
    #    self._sfma = data 
    #    self._bias = data
    #
    #@sfma.setter    
    #def _set_sfma(self, data):
    #    """
    #    Set sensors scale factor and misalignment.

    #    Parameters
    #    ----------
    #    data: 3x3 array, float, scale factor + misalignment + identity matirx
    #        SF + MA + I, SF: diagonal, I: identity, MA: elements of diagonal
    #        matrix is zero.
    #    """
    #    if data.shape() != (3, 3):
    #        raise ValueError('Matrix size should be 3x3')
    #    self._sfma = data 


#class IMUCalibration(object):
#    """
#    Base class for IMU sensor calibrating methods.
#
#    In most cases IMU have 6 DoF and consists of two sensors triads:
#    gyroscopes, accelerometers.
#    """
#    __metaclass__ = ABCMeta
#
#    def __init__(self):
#        """
#        Init consts for current calibrator.
#        """
#
#        # Latitude of Kyiv
#        self._local_latitude = 50 * np.pi / 180
#        self._local_height = 0
#        self._g = 9.8
#        
#        self.gyro_bias = np.array([0., 0., 0.])
#        self.acc_bias = np.array([0., 0., 0.])
#        
#        self.g
#        
#
#
#
#        sensor_parameter = {
#                "gyro": gyro_report,
#                "acc": acc_report,
#                "mag": mag_report,
#                }
#
#
#
#    @abstractproperty
#    def report(self, sensor):
#        """
#        Returns sensors biases.
#        """
#        
#
#    @abstractmethod
#    def load_data(self):
#        """
#        Read data form file.
#        """
#
#    @property
#    def gravity(self):
#        """
#        Returns local value of gravitational acceleration.
#        """
#        return self._g
#
#    @gravity.setter
#    def _set_gravity(self, input_g):
#        """
#        Set magnitude of local gravity vector.
#
#        Parameters
#        ----------
#        input_g: float, m/s^2, magnitude of gravity vector.
#        """
#        if (9.6 < input_g and input_g > 9.9):
#            self._g = input_g
#        else:
#            raise ValueError('g should be between near 9.79'
#                             ' and not negative, in m/s^2')
#
#    @property
#    def local_latitude(self):
#        """
#        Returns local latitude
#
#        This parameter is needed for local gravitational
#        acceleration compulation based on datum model.
#        """
#        return self._local_latitude
#
#
#    @local_latitude.setter
#    def _set_local_latitude(self, phi):
#        """
#        local latitude setter
#
#        Parameters
#        ----------
#        phi: float, radian, local geodesic latitude
#        """
#        if (- np.pi / 2. < phi and phi < np.pi / 2.):
#            self._local_latitude = phi
#        else:
#            raise ValueError('Geodesic latitude must be '
#                             'between -pi/2 to pi/2 ')
#
#    def calc_gravity(self, phi, h, datum):
#        """
#        Calculates local gravity value based on
#        specified Earth datum.
#
#        Parameters
#        ----------
#        phi: float, radian, local geodesic latitude
#        h: float, m, local height
#        datum: Earth datum
#        """
#
#        self._local_height = h
#        self._set_local_latitude(phi)
#
#        if isinstance(datum, EarthDatum):
#            self._earth_rate = self._datum.rate
#            self._g = self._datum.gravity(self._local_latitude, \
#                                          self._local_height)
#        else:
#            TypeError('datum must be instance of EarthDatum')
#
#
#class MyCalibrator(IMUCalibration):
#    """
#    Calibrator implementation for RLG based IMU
#    """
#   
#    
#    def integrate(self, dtime, data): 
#        """
#        Solve navigation for position of calibration fixture.
#        """
#        rows, cols = np.shape(data)
#        if rows != 6:
#            raise ValueError('data must have 6 rows') 
#        
#
#        gyro_x = data[0, :] 
#        gyro_y = data[1, :] 
#        gyro_z = data[2, :]
#        
#        acc_x = data[3, :] 
#        acc_y = data[4, :] 
#        acc_z = data[5, :]
#        
#        
#
#
#
#
#
#   # def calc_coefficient(self,data,shift):
#   #     """
#   #     Returns calibration coefficients for
#   #     gyro and acc (biases,SF and MA)
#   #     """
#   #     a_11 = data 
#
#   #     
#   #     # Gyro scale factors
#   #     gyro_xx = (a_11 + a_12) / (2. * np.pi * self._g)
#   #     gyro_yy = (a_21 + a_22) / (2. * np.pi * self._g)
#   #     gyro_zz = (a_31 + a_32) / (2. * np.pi * self._g)
#
#   #     # Acc bias
#   #     acc_x = (a_12 - a_11) / 4.
#   #     acc_y = (a_22 - a_21) / 4.
#   #     acc_z = (a_32 - a_31) / 4.
#
#   #     # Acc scale factors
#   #     acc_xx = (c_32 + c_31 - acc_x) / (2. * self._g)
#   #     acc_yy = (c_22 + c_21 - acc_y) / (2. * self._g)
#   #     acc_zz = (c_12 + c_11 - acc_z) / (2. * self._g)
#
#   #     # Gyro misalignment
#   #     gyro_xy = (b_23 + 2. * acc_x) / (2. * self._g)
#   #     gyro_xz = (a_13 + 2. * acc_x) / (2. * self._g)
#
#   #     gyro_yx = - b_21 / (2. * seflf._g)
#   #     gyro_yz = (b_13 + 2. * acc_y) / (2. * self._g)
#
#   #     gyro_zx = (b_33 + 2 * acc_z) / (2. * self._g) + acc_zx
#   #     gyro_zy = -b_11 / (2. * self._g)
#
#   #     # Acc misalignment, assumed as reference axis
#   #     a_xz = 0
#   #     a_yz = 0
#   #     a_xy = 0
#
#   #     a_yx = - (a_33 + 2 * acc_y) / (2. * self._g) + gyro_yx
#   #     a_zx = - b_31 / (2. * self._g) - gyro_xz
#   #     a_zy = - (a_23 + 2 * acc_z) / (2. * self._g) + gyro_zy
#   #     
#   #     self.gyro_bias = np.array(
#
#
#   #     print 'gyro_xx = ', gyro_xx
#   #     print 'gyro_yy = ', gyro_yy
#   #     print 'gyro_zz = ', gyro_xx
#
#   #     print 'acc_xx = ', acc_xx
#   #     print 'acc_yy = ', acc_yy
#   #     print 'acc_zz = ', acc_xx
#
#   #     print 'acc_x = ', acc_x
#   #     print 'acc_y = ', acc_y
#   #     print 'acc_z = ', acc_x
#
#   # def load_data(self, dt, data, point, shift):
#   #     """
#   #     Read data form file.
#   #     """
#
#   #     dva2_1
#   #     dva2_2
#
#   #     dva2_1
#   #     dva2_2
#
#   #     dvb1_1
#   #     dvb1_2
#
#   #     dvb2_1
#   #     dvb2_2
#
#   #     dvb2_1
#   #     dvb2_2
#
#   # def report(self):
#   #     """
#   #     Print report"""
#
#
#if __name__ == '__main__':
#
#    pfl = np.array([[10, 0,        0,        0, 0,  0,     0],
#                   [20, np.pi/20, 0,        0, 0,  0,     0],
#                   [10, 0,        np.pi,    0, 0,  0,     0],
#                   [20, np.pi/20, np.pi,    0, 0,  0,     0],
#                   [10, 0,        2*np.pi,  0, 0,  0,     0],
#                   [20, 0,        2*np.pi,  0, 0,  np.pi/20., 0],
#                   [10, 0,        2*np.pi,  0, 0,  0,   np.pi],
#                   [30, 0,        2*np.pi,  0, 0,  0,   np.pi]])
#
#    
#
#
#    dt = 0.1
#    base1 = np.array([0, 0, 0])
#    base2 = np.array([-np.pi/2., -np.pi/2., 0.])
#    base3 = np.array([np.pi/2., 0, np.pi/2.]) 
#    
#    (time1, rate1, angle1) = create_profile(dt, pfl, base1)
#    (time2, rate2, angle2) = create_profile(dt, pfl, base2)
#    (time3, rate3, angle3) = create_profile(dt, pfl, base3)
#



if __name__ == '__main__':
    gyro = CalibrationSensorModel()
    print gyro.x 
    gyro.y = 5.
    print gyro.bias
    print gyro.y
