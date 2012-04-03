import numpy as np

from basetrajectory import BasicTrajectory

class FGTrajectory(BasicTrajectory):
    """
    Trajectory generator based on FlightGear data.
    """

    def __init__(self):
        super(FGTrajectory, self).__init__()

        self._host = host
        self._port = port

    def connect(self,  host, port):
        """
        Connect to FG via telnet
        """

    def gyros(self, time):
        """
        Generate output of angular rate sensors in inertial frame.
        """

    def accs(self, time):
        """
        Generate output of accelerometers in inertial frame.
        """

    def init_position(self, time):
        """
        Returns initial position of IMU.
        """

    def init_orientation(self):
        """
        Returns initial orientation of IMU.
        """

    def position(self, time):
        """
        Returns 3D trajectory of body movement.
        """


    def orientation(self, time):
        """
        Returns orientation of body versus time.
        """