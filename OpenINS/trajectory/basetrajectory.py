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



class BasicCalibTraj(CalibTrajectory):
    """
    Basic calibration table movement gentrator.
    """
    def __init__(self, rs):
        """
        Parameters
        ----------
        rs: rotation sequence object
        """
        super(BasicCalibTraj, self).__init__()

        if isinstance(rs, RotationSequence):
            self.rs = rs
        else:
            raise TypeError('Must be instance of RotationSequence')



    def orientation(self, time):
        angle, rate = self.rs.run(time)
        return angle

    def gyros(self, time):
        angle, rate = self.rs.run(time)
        w = rate + np.dot(euler2dcm(angle[0], angle[1], angle[2]), self._omega_n)
        return w

    def accs(self, time):
        angle, rate = self.rs.run(time)
        gravity  = np.array([0., 0., self._g])

        return np.dot(euler2dcm(angle[0], angle[1], angle[2]), gravity)

class RotateIMU(object):
    """
    Represents simple rotation of calibration fixture.

    A bunch fo such objects creates whole movement of
    calibration table.
    """

    def __init__(self, time1, x_turn, y_turn, z_turn ):
        """
        Init rotation object

        Parameters
        ----------
        time: float, sec
        x_turn: float, rad, relative turn around x axis of IMU
        y_turn: float, rad , relative turn around y axis of IMU
        z_turn: float, rad, relative turn around z axis of IMU
        """

        # start time of current rotation
        self.time_start = 0.
        # time between start and end of current rotation
        self.time_total = time1

        self.angle_start = np.array([0., 0., 0.,])
        self.angle_total = np.array([x_turn, y_turn, z_turn])

        self.turn_rate = self._calc_rate(self.time_total, self.angle_total)

    def _calc_rate(self, t, angle):
        """
        Calculate turn rate of fixture based on time and angle.

        Basic relationship between angle and time is linear function. You could
        redefine this for more complex relationship, like exponential or
        quadratic or even random, you need only keep in mind that integral from
        this function over specified time must be equal to resulting angle.

        Parameters
        ----------
        t: float, sec, time
        angle: float, rad, angle of IMU turn.

        Returns
        -------
        rate: float, rad/s, turn rate
        """
        # Very basic implementation
        rate = angle/t
        return rate

    def angle(self, t):
        """
        Calculate angle at specified time.

        Parameters
        ----------
        t: float, sec, time
        """

        return self.angle_start + self.turn_rate * (t - self.time_start)

    def rate(self, t):
        """
        Calculate angular rate of turn.

        Parameters
        ----------
        t: float, sec, time
        """
        return self.turn_rate


    def is_right_time(self, time):
        """
        Check for right time.
        """
        if self.time_start <= time and \
           time < self.time_start + self.time_total:

            return True
        else:
            return False

class RotationSequence(object):
    """
    Container for rotation sequence
    """

    def __init__(self):

        self.time = [0.]
        self.angle = [np.array([0., 0., 0])]
        self.sequence = []

    def set_init_orientation(self, gamma, theta, psi):
        """
        Set initial orientation of calibration table.

        Parameters
        ----------
        gamma: float, rad, roll
        theta: float, rad, pitch
        psi: float, rad, yaw
        """
        self.angle = np.array([gamma, theta, psi])

    def add(self, move):
        """
        Add calibration table movement.

        Parameters
        ----------
        move: obj, one simple rotation
        """
        if isinstance(move, RotateIMU):

            move.time_start = sum(self.time)
            self.time.append(move.time_total)

            move.angle_start = sum(self.angle)
            self.angle.append(move.angle_total)

            self.sequence.append(move)
        else:
            raise TypeError('Must be instance of RotateIMU')

    def run(self, time):
        """
        Returns orientation and angular rate of calibration
        table.

        Parameters
        ----------
        time: float, s

        Returns
        -------
        angle: np array 1x3, rad, current angle
        rate: np array 1x3, rad/s, current rate
        """

        for sq in self.sequence:

            if sq.is_right_time(time):
                rate = sq.rate(time)
                angle = sq.angle(time)

                return angle, rate


if __name__ == "__main__":
    pass


