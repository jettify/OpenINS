"""
Base module for trajectory generators
"""

from abc import ABCMeta, abstractmethod
import numpy as np

from environnement.datum import WGS84
from orientationmath.orientation import euler2dcm

class BasicTrajectory(object):
    """
    Represents basic class for all trajectory
    """

    __metaclass__ = ABCMeta

    @abstractmethod
    def gyros(self, time):
        """
        Generate output of angular rate sensors in inertial frame.
        """

    @abstractmethod
    def accs(self, time):
        """
        Generate output of accelerometers in inertial frame.
        """

    @abstractmethod
    def init_position(self, time):
        """
        Returns initial position of IMU.
        """

    @abstractmethod
    def init_orientation(self):
        """
        Returns initial orientation of IMU.
        """

    @abstractmethod
    def position(self, time):
        """
        Returns 3D trajectory of body movement.
        """

    @abstractmethod
    def orientation(self, time):
        """
        Returns orientation of body versus time.
        """

class CalibTrajectory(BasicTrajectory):
    """
    Trajectory generator for calibration purposes.

    IMU installed on stationary base, means position remains the same. Only
    orientation is changing.
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        """
        Init consts for current calibrator.
        """

        # Latitude of Kyiv, you probably should change to your IMU position :)
        # via set_position()
        self._latitude  = np.deg2rad(50)
        self._longitude = np.deg2rad(30)

        self._height = 0
        # Set default Earth Datum
        self.datum = WGS84()
        # Local gravity in m/s^2
        self._g = 9.8
        # Earth angular speed projected to NED frame
        self._omega_n = np.array([np.cos(self._latitude), 0.,
                                 -np.sin(self._latitude)])*self.datum.rate
        # Initial orientation
#        self._roll = 0.0
#        self._pitch = 0.0
#        self._yaw = 0.0

    def init_position(self):
        """
        Returns initial position of IMU.
        """

        return self._latitude, self._longitude, self._height

    def position(self):
        """
        Since INS stationary during calibration, so
        current position equals to init position.
        """

        return self.init_position()

    def init_orientation(self):
        """
        Returns initial orientation of IMU.
        """

        return self._roll, self._pitch, self._yaw



    @property
    def gravity(self):
        """
        Returns local value of gravitational acceleration.
        """
        return self._g

    @gravity.setter
    def _set_gravity(self, input_g):
        """
        Set magnitude of local gravity vector.

        Useful if precise g magnitude available for position of calibration
        table (could be measured by gravimeter or super precise accelerometer).

        Parameters
        ----------
        input_g: float, m/s^2, magnitude of gravity vector.
        """

        if 9.6 < input_g and input_g > 9.9:
            self._g = input_g
        else:
            raise ValueError('g should be between near 9.79'
                             ' and not negative, in m/s^2')

    def set_position(self, phi, lat, h, calc_g=True):
        """
        Set local position of IMU (calibration table) and calc local gravity
        based on position and Earth Datum.

        Parameters
        ----------
        phi: float, radian, local geodesic latitude
        lat: float, radian, local geodesic longitude
        h: float, radian, local geodesic height
        g: bool, if True (default), will calc and set local gravity, from Eath Datum
        """

        if (np.abs(phi) <= np.pi / 2.) and (np.abs(lat) <= np.pi ):
            # Set position
            self._latitude = phi
            self._longitude = lat
            self._height = h
            # Calc local gravity based on position and Earth Datum
            if calc_g:
                self._g = self.datum.gravity(self._latitude, self._height)

            self._omega_n = np.array([np.cos(self._latitude), 0.,
                                     -np.sin(self._latitude)])*self.datum.rate
        else:
            raise ValueError('Latitude must be in interval +/- pi/2 rad'
                             'Longitude must be in interval +- pi rad')






if __name__ == "__main__":
    pass


