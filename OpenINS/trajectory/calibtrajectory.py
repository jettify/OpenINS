import numpy as np

from basetrajectory import CalibTrajectory
from orientationmath.orientation import euler2dcm

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
        if self.time_start <= time and\
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
        self.angle.append(np.array([gamma, theta, psi]))

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