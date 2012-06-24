"""
Base module for trajectory generators
"""

from abc import ABCMeta, abstractmethod
import numpy as np
import sympy as sp


from openins.environnement.datum import InitPosition


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

if __name__ == "__main__":
    pass