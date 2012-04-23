import numpy as np

from basetrajectory import CalibTrajectory
from orientationmath.orientation import euler2dcm
from orientationmath.orientation import dcm2euler
from orientationmath.orientation import euler2quat
from orientationmath.orientation import quat2dcm

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
        y_turn: float, rad, relative turn around y axis of IMU
        z_turn: float, rad, relative turn around z axis of IMU
        """

        # start time of current rotation
        self.time_start = 0.
        # time between start and end of current rotation
        self.time_total = time1

        self.init_dcm = euler2dcm(0., 0., 0.)
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

    def orientation_dcm(self, t):
        """
        Calculate angle at specified time.

        Parameters
        ----------
        t: float, sec, time
        """
        a = self.turn_rate * (t - self.time_start)
        turn_dcm = euler2dcm(a[0], a[1], a[2])

        return np.dot(self.init_dcm, turn_dcm)

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
        if self.time_start <= time < self.time_start + self.time_total:

            return True
        else:
            return False

class RotationSequence(object):
    """
    Container for rotation sequence
    """

    def __init__(self):

        self.time = [0.]
        self.dcm = [euler2dcm(0., 0., 0.)]
        self.sequence = []

    #    def get_init_orientation(self):
    #        """
    #        Returns initial orientation of rotation sequence.
    #        """
    #        return self.angle[0]

    def set_init_orientation(self, gamma, theta, psi):
        """
        Set initial orientation of calibration table.

        Parameters
        ----------
        gamma: float, rad, roll
        theta: float, rad, pitch
        psi: float, rad, yaw
        """
        self.dcm[0] = euler2dcm(gamma, theta, psi)

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

            move.init_dcm = self.dcm[-1]

            turn_dcm = euler2dcm(move.angle_total[0], move.angle_total[1], move.angle_total[2])

            self.dcm.append(np.dot(self.dcm[-1], turn_dcm))
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
                dcm = sq.orientation_dcm(time)
                return dcm, rate
            elif sq is self.sequence[-1]:
                rate = sq.rate(time)
                dcm = sq.orientation_dcm(time)
                return dcm, rate

class BasicCalibTraj(CalibTrajectory):
    """
    Basic calibration table movement generator.
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
        dcmbn, rate = self.rs.run(time)
        return dcm2euler(dcmbn)

    def gyros(self, time):

        dcm, rate = self.rs.run(time)
        # add Earth angular rate to gyro measurement

        w = rate + np.dot(dcm.T, self._omega_n)
        return w

    def accs(self, time):
        dcm, rate = self.rs.run(time)
        g  = np.array([0., 0., self.ipos.gravity])

        return np.dot(dcm.T, g)

