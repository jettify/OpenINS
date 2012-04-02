"""
Base module for trajectory generators
"""

from abc import ABCMeta, abstractmethod
import numpy as np

from environnement.datum import InitPosition

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

    def __init__(self, lat=0.87, lon=0.52, h=0.):
        """
        Init consts for current calibrator.
        """

        # Latitude of Kyiv, you probably should change to your IMU position :)
        # via set_position()

        # set local position to Kyiv)
        # you probably want to change this))
        self.ipos = InitPosition(lat, lon, h)

        # Earth angular speed projected to NED frame
        self._omega_n = np.array([np.cos(self.ipos.lat), 0.,
                                 -np.sin(self.ipos.lat)])*self.ipos.datum.rate
        # Initial orientation
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def init_position(self):
        """
        Returns initial position of IMU.
        """

        return self.ipos.lat, self.ipos.lon, self.ipos.h

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

if __name__ == "__main__":
    pass


