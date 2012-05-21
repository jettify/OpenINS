"""
INS computer implementation.
"""

from abc import ABCMeta, abstractmethod
import numpy as np
from environnement.datum import InitPosition

class BasicINS(object):
    """
    Represents basic class for all INS computers. 
    """

    __metaclass__ = ABCMeta

    @abstractmethod
    def alignment(self):
        """
        Perform initial alignment of INS.
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

