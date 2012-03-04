"""
Base module for trajectory generators
"""

from abc import ABCMeta, abstractmethod

class BasicTrajectory(object):
    """
    Represents basic class for all trajectory
    """

    __metaclass__ = ABCMeta

    @abstractmethod
    def gyros(self):
        """
        Generate output of angular rate sensors in inertial frame. 
        """

    @abstractmethod
    def accs(self):
        """
        Generate output of accelerometers in inertial frame.
        """

    @abstractmethod
    def init_position(self):
        """
        Returns initial position of IMU.
        """

    @abstractmethod
    def init_orientation(self):
        """
        Returns initial orientation of IMU.
        """
   
    @abstractmethod
    def position(self):
        """
        Returns 3D trajectory of body movement.
        """
    
    @abstractmethod
    def orientation(self):
        """
        Returns orientation of body versus time.
        """



