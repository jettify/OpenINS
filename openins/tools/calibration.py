#!/usr/bin/env python
# coding=utf8

"""
Base classes for calibration procedure.
"""

from abc import ABCMeta, abstractproperty, abstractmethod 
import numpy as np


from environnement.datum import InitPosition
from orientationmath.orientation import quat_prop_4o, quat2dcm, euler2quat, dcm2euler
from orientationmath.orientation import quat_prop
from visualisation.plotter import plot_trinity

from tools.profile import create_profile


class IMUCalibration(object):
    """
    Base class for IMU sensor calibrating methods.1qs

    In most cases IMU have 6 DoF and consists of two sensors triads:
    gyroscopes, accelerometers.
    """
    __metaclass__ = ABCMeta

    def __init__(self, lat=0.87, lon=0.52, h=0.):
        """
        Init consts for current calibrator.
        """

        # total calibration time
        self.time = 0.
        # sample time
        self.dt = 0.1

        ## Essential consts
        # initial position
        self.ipos = InitPosition(lat, lon ,h)
        # gravity in NED frame
        self._g_n = np.array([0., 0., self.ipos.gravity ])

        ## Basic sensors parameters
        # gyro parameters: bias and scalefactor, misalignment matrix
        self.gyro_model = {'x':0., 'y':0., 'z':0.,
                         'xx':0., 'xy':0., 'xz':0.,
                         'yx':0., 'yy':0., 'yz':0.,
                         'zx':0., 'zy':0., 'zz':0.,}

        # acc parameters: bias and scalefactor, misalignment matrix
        self.acc_model = {'x':0., 'y':0., 'z':0.,
                         'xx':0., 'xy':0., 'xz':0.,
                         'yx':0., 'yy':0., 'yz':0.,
                         'zx':0., 'zy':0., 'zz':0.,}

    def set_gravity(self, g):
        """
        Set gravity magnitude

        Parameters
        ----------
        g: float, m/s^2, local gravity
        """
        self._g_n = np.array([0., 0., g ])


    @abstractmethod
    def gyro_report(self):
        """
        Report about calibration coefficients of IMU.

        Coefficients include biases, scale factors, misalignment of
        gyros and accelerometers.
        """

    @abstractmethod
    def acc_report(self):
        """
        Report about calibration coefficients of IMU.

        Coefficients include biases, scale factors, misalignment of
        gyros and accelerometers.
        """

    @abstractmethod
    def load_data(self):
        """
        Read data form file.
        """



class DieselCalibrator(IMUCalibration):
    """
    Calibrator implementation for RLG based IMU designed by Diesel[1].

    This method was covered in some other publications [2, 3].

    References
    ----------
    [1] Diesel, J. W., Calibration of A Ring Laser Gyro Inertial Navigation
        System, Paper 87-7, 13th Biennial Guidance Test Symposium, Holloman
    [2] Rogers, R. M., Applied Mathematics in Integrated Navigation Systems
    [3] Christensen R., Fogh N., Inertial Navigation System, AALBORG UNIVERSITY

    """
    def __init__(self, lat=0.87, lon=0.52, h=0.):
        """
        Constructor, init necessary consts.

        Parameters
        ----------
        lat: float, rad, latitude
        lon: float, rad, longitude
        h: float, meters, height above the sea
        """
        super(DieselCalibrator, self).__init__(lat, lon, h)

        # initial orientation for each set of rotation
        self.init_quat_set1 = euler2quat(0., 0., 0.)
        self.init_quat_set2 = euler2quat(-np.pi/2., -np.pi/2., 0.)
        self.init_quat_set3 = euler2quat(np.pi/2., 0., np.pi/2.)

    def load_data(self, set1, set2, set3):
        """
        Load data, measured from IMU during calibration.
        """

        # data set 1
        self.gyro1 = set1[:,:3]
        self.acc1 = set1[:,3:6]
        # data set 2
        self.gyro2 = set2[:,:3]
        self.acc2 = set2[:,3:6]
        # data set 3
        self.gyro3 = set3[:,:3]
        self.acc3 = set3[:,3:6]


    def integrate(self, gyro1, acc1):
        """
        Solve navigation for position of calibration fixture.
        """

        # Project Earth rate to body frame
        omega_b = np.dot(quat2dcm(self.quat).T, self.ipos.omega_n)

        # form matrix for gyro and acc errors models

        gyro_sfma = np.array([
            [self.gyro_model['xx'], self.gyro_model['xy'], self.gyro_model['xz']],
            [self.gyro_model['yx'], self.gyro_model['yy'], self.gyro_model['yz']],
            [self.gyro_model['zx'], self.gyro_model['zy'], self.gyro_model['zz']]])

        gyro_bias = np.array([self.gyro_model['x'],
                              self.gyro_model['y'],
                              self.gyro_model['z']])

        acc_sfma = np.array([
            [self.acc_model['xx'], self.acc_model['xy'], self.acc_model['xz']],
            [self.acc_model['yx'], self.acc_model['yy'], self.acc_model['yz']],
            [self.acc_model['zx'], self.acc_model['zy'], self.acc_model['zz']]])

        acc_bias = np.array([self.acc_model['x'],
                             self.acc_model['y'],
                             self.acc_model['z']])




        # Compensate bias, ma and sf
        gyro = np.dot(np.linalg.inv(gyro_sfma + np.eye(3)), gyro1 - gyro_bias*self.dt).T
        acc = np.dot(np.linalg.inv(acc_sfma + np.eye(3)), acc1 - acc_bias).T

        # calc INS acceleration
        dv_n = np.dot(quat2dcm(self.quat), acc) - self._g_n
        # update IMU orientation
        self.quat = quat_prop_4o(self.quat, gyro - omega_b*self.dt)

        return dv_n

    def calc_coefficient(self, tbl):
        """
        Returns calibration coefficients for
        gyro and acc (biases,SF and MA)

        st, t1, t2, t3, t4 --> set1
        st, t1, t2, t3, t4 --> set2
        st, t1, t2, t3, t4 --> set3
        """
        gyro_model = {'x':0., 'y':0., 'z':0.,
                           'xx':0., 'xy':0., 'xz':0.,
                           'yx':0., 'yy':0., 'yz':0.,
                           'zx':0., 'zy':0., 'zz':0.,}

        # acc parameters: bias and scalefactor, misalignment matrix
        acc_model = {'x':0., 'y':0., 'z':0.,
                      'xx':0., 'xy':0., 'xz':0.,
                      'yx':0., 'yy':0., 'yz':0.,
                      'zx':0., 'zy':0., 'zz':0.,}

        self.quat = self.init_quat_set1
        dvn1 = np.array(zip(*map(self.integrate, self.gyro1, self.acc1)))

        self.quat = self.init_quat_set2
        dvn2 = np.array(zip(*map(self.integrate, self.gyro2, self.acc2)))

        self.quat = self.init_quat_set3
        dvn3 = np.array(zip(*map(self.integrate, self.gyro3, self.acc3)))


        # coefficients: a - x channel, b - y channel
        a = np.eye(3)
        b = np.eye(3)

        d = np.array([0., 0., 0.])
        drx = np.array([0., 0., 0.])
        dry = np.array([0., 0., 0.])

        for i, dvn in enumerate([dvn1, dvn2, dvn3]):

            # find difference between start and end of rotation: dV[T] - dV[0]
            r1 = np.mean(dvn[:, tbl[i][2]:tbl[i][2] + tbl[i][0]], axis=1) - \
                 np.mean(dvn[:, tbl[i][1]:tbl[i][1] + tbl[i][0]], axis=1)
            r2 = np.mean(dvn[:, tbl[i][3]:tbl[i][3] + tbl[i][0]], axis=1) - \
                 np.mean(dvn[:, tbl[i][2]:tbl[i][2] + tbl[i][0]], axis=1)
            r3 = np.mean(dvn[:, tbl[i][4]:tbl[i][4] + tbl[i][0]], axis=1) - \
                 np.mean(dvn[:, tbl[i][3]:tbl[i][3] + tbl[i][0]], axis=1)

            # sum of before start and after end of each rotation
            # used to find accelerometer scalefactor
            d[i] = np.mean(dvn[2, tbl[i][2]:tbl[i][2] + tbl[i][0]], axis=0) + \
                   np.mean(dvn[2, tbl[i][1]:tbl[i][1] + tbl[i][0]], axis=0)

            # used in polyfit function
            # purpose of polyfit is to find velocity of acceleration projected on NED
            # frame. Then this value used in gyro bias calculation. Proper
            # way is to use mini Kalman filter in single channel, witch returns
            # estimation of acceleration velocity and smooths output data.
            # fo details see [1].
            t = np.arange(0, len(dvn[0, tbl[i][4]:tbl[i][4] + tbl[i][0]])*self.dt, self.dt )[:-1]
            print np.shape(t)
            print len(dvn[1, tbl[i][4]:tbl[i][4] + tbl[i][0]])
            print np.shape(dvn[1, tbl[i][4]:tbl[i][4] + tbl[i][0]])
#

#            t = np.empty_like(dvn[1, tbl[i][4]:tbl[i][4] + tbl[i][0]])



            dry[i] = np.polyfit(t[:], dvn[1, tbl[i][4]:tbl[i][4] + tbl[i][0]], 1)[0] -\
                    np.polyfit(t[:], dvn[1, tbl[i][3]:tbl[i][3] + tbl[i][0]], 1)[0]

            drx[i] = np.polyfit(t[:], dvn[0, tbl[i][4]:tbl[i][4] + tbl[i][0]], 1)[0] -\
                     np.polyfit(t[:], dvn[0, tbl[i][3]:tbl[i][3] + tbl[i][0]], 1)[0]

            for j, r in enumerate([r1, r2, r3]):

                a[i, j] = r[0]
                b[i, j] = r[1]

        # Gyro scale factors
        gyro_model['xx'] = (b[0, 0] + b[0, 1]) / (- 2. * np.pi * self.ipos.gravity)
        gyro_model['yy'] = (b[1, 0] + b[1, 1]) / (- 2. * np.pi * self.ipos.gravity)
        gyro_model['zz'] = (b[2, 0] + b[2, 1]) / (- 2. * np.pi * self.ipos.gravity)

        # Gyro bias
        gyro_model['x'] = dry[0]/(2.*self.ipos.gravity)
        gyro_model['y'] = dry[1]/(2.*self.ipos.gravity)
        gyro_model['z'] = dry[2]/(2.*self.ipos.gravity)

        gyro_model['x'] = drx[0]/(2.*self.ipos.gravity)
        gyro_model['y'] = drx[1]/(2.*self.ipos.gravity)
        gyro_model['z'] = drx[2]/(2.*self.ipos.gravity)


        # Acc bias
        acc_model['x'] = (b[2, 1] - b[2, 0]) / 4.
        acc_model['y'] = (b[0, 1] - b[0, 0]) / 4.
        acc_model['z'] = (b[1, 1] - b[1, 0]) / 4.


        # Acc scale factors
        acc_model['xx'] = (d[1]) / (2. * self.ipos.gravity)
        acc_model['yy'] = (d[2]) / (2. * self.ipos.gravity)
        acc_model['zz'] = (d[0]) / (2. * self.ipos.gravity)

        # Gyro misalignment

        gyro_model['xy'] = (a[1, 0])/(-2.*self.ipos.gravity)

        gyro_model['xz'] = (a[0, 2] + 2.*acc_model['x'])/\
                                 (2.*self.ipos.gravity)
        gyro_model['yx'] = (a[1, 2] + 2.*acc_model['y'])/\
                                 (2.*self.ipos.gravity)
        gyro_model['yz'] = (b[0, 2] + 2.*acc_model['y'])/\
                                 (2.*self.ipos.gravity)
        gyro_model['zx'] = (a[0, 0])/(-2.*self.ipos.gravity)



        # Acc misalignment, next assumed as reference axis
        acc_model['yx'] = 0.
        acc_model['yz'] = 0.
        acc_model['xz'] = 0.


        acc_model['zx'] = -(b[1, 2] + 2.*acc_model['z'])/\
                              (2*self.ipos.gravity) + gyro_model['zx']
        acc_model['xy'] = -(b[2, 2] + 2.*acc_model['x'])/\
                              (2.*self.ipos.gravity) + gyro_model['xy']

        acc_model['zy'] = -a[2, 0]/(2*self.ipos.gravity) -\
                              gyro_model['yz']

        gyro_model['zy'] =  (a[2, 2] + 2.*acc_model['z'])/\
                              (2.*self.ipos.gravity) + acc_model['zy']


        # adds two dictionaries
        self.gyro_model = dict( (n, gyro_model.get(n, 0)+self.gyro_model.get(n, 0))
        for n in set(gyro_model)|set(self.gyro_model) )
        self.acc_model = dict( (n, acc_model.get(n, 0)+self.acc_model.get(n, 0))
        for n in set(acc_model)|set(self.acc_model) )
        print self.gyro_model

        return dvn1, dvn2, dvn3

    def gyro_report(self):
        """
        Report about calibration coefficients of IMU.

        Coefficients include biases, scale factors, misalignment of
        gyros and accelerometers.
        """
        print '*****************************'
        print 'g b1=', self.gyro_model['x']
        print 'g b2=', self.gyro_model['y']
        print 'g b3=', self.gyro_model['z']
        print '*****************************'
        print 'g xx', self.gyro_model['xx']
        print 'g yy', self.gyro_model['yy']
        print 'g zz', self.gyro_model['zz']
        print '*****************************'
        print 'g xy', self.gyro_model['xy']
        print 'g xz', self.gyro_model['xz']
        print 'g yx', self.gyro_model['yx']
        print 'g yz', self.gyro_model['yz']
        print 'g zx', self.gyro_model['zx']
        print 'g zy', self.gyro_model['zy']


    def acc_report(self):
        """
        Report about calibration coefficients of IMU.

        Coefficients include biases, scale factors, misalignment of
        gyros and accelerometers.
        """
        print '*****************************'
        print 'a xy', self.acc_model['xy']
        print 'a xz', self.acc_model['xz']
        print 'a yx', self.acc_model['yx']
        print 'a yz', self.acc_model['yz']
        print 'a zx', self.acc_model['zx']
        print 'a zy', self.acc_model['zy']
        print '*****************************'
        print 'a xx', self.acc_model['xx']
        print 'a yy', self.acc_model['yy']
        print 'a zz', self.acc_model['zz']
        print '*****************************'
        print 'a b1=', self.acc_model['x']
        print 'a b2=', self.acc_model['y']
        print 'a b3=', self.acc_model['z']

if __name__ == '__main__':
    pass
