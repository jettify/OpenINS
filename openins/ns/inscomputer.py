"""
INS computer implementation.
"""

from abc import ABCMeta, abstractmethod, abstractproperty
import numpy as np

from environnement.datum import InitPosition
from ns.navsystem import NavSystem
from orientationmath.orientation import skew
from orientationmath.orientation import quat_prop
from orientationmath.integrators import TrapeziumRule
from orientationmath.integrators import SimpsonRule
from environnement.datum import WGS84
from orientationmath.orientation import quat_mult
from orientationmath.orientation import quat2dcm
from orientationmath.orientation import rate2quat
import copy

class INSComputer(NavSystem):
    """
    Base class for INS computers.
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        """
        Init data for Simple INS.
        """
        # sample time
        self.dt = 0.005
        # datum
        self.datum = WGS84()
        # state vector
        self._state = np.zeros(13)
        # initial quaternion

    @abstractmethod
    def __call__(self,gyros, acces ):
        """
        Run INS computer.

        Parameters
        ----------
        t: float, sec, time
        gyros, array, rad, incremental angle
        accs, array, m/s, incremental velocity
        """

        self._integrate_orientation(gyros)
        self._integrate_velocity(acces)
        self._integrate_position()
        return self._state


    @abstractmethod
    def _integrate_orientation(self, inc_angle):
        """
        Integrate orientation quaternion matrix.
        """

    @abstractmethod
    def _integrate_velocity(self, inc_angle):
        """
        Integrate NED acceleration to NED velocity.
        """

    @abstractmethod
    def _integrate_position(self):
        """
        Integrate velocity in NED frame to position on Earth.
        """


    @abstractproperty
    def orientation_dcm(self):
        """
        Returns orientation matrix.
        """

class SimpleINSComputer(INSComputer):
    """
    Base class for INS computers.
    """

    def __init__(self):
        """
        Init data for Simple INS.
        """
        super(SimpleINSComputer, self).__init__()

        self._set_integrators(self.state[0:3], self.state[3:6],
            self.state[9:])

    def __call__(self, gyros, accs):
        """
        Run INS computer.

        Parameters
        ----------
        t: float, sec, time
        gyros, array, rad per dt, incremental angle
        accs, array, m/s, incremental velocity
        """
        q = self._integrate_orientation(gyros)
#        self._state = np.concatenate((self._state[:9], q), axis=1)

        v, a = self._integrate_velocity(accs)
#        self._state = np.concatenate((self._state[:3], v, a, self._state[9:]), axis=1)


        pos = self._integrate_position()
#        self._state[0:3] = pos
#        c =  self._state - np.concatenate((pos, v, a, q), axis=1)
#        self._state = np.concatenate((pos, self._state[3:]), axis=1)
#        self._state = np.concatenate((self._state[:9], q), axis=1)
        self._state = np.concatenate((pos, v, a, q), axis=1)

        return  copy.copy(self._state)

    def _set_integrators(self, init_pos, init_vel, init_q):
        """
        Set initial condition of integrators.
        """
        self.pos_integrator = SimpsonRule(init_pos)
        self.vel_integrator = SimpsonRule(init_vel)
        self.att_integrator = SimpsonRule(init_q)


    @property
    def state(self):
        """
        Returns state of inertial navigation computer.

        Returns
        -------
        state: array, float, with next structure
            [3*position, 3*velocity, 3*acceleration, 4*quaternion]
        """
        return self._state

    #@state.setter
    def set_state(self, state):
        """
        Set state of INS computer.

        Parameters
        ----------
        state: array, float, with next structure
            [3*position, 3*velocity, 3*acceleration, 4*quaternion]
        """
        assert np.shape(self._state) == np.shape(state),\
        'wrong length of input parameter'
        self._state = state
        self._set_integrators(self._state[0:3], self._state[3:6],
                              self._state[9:])


    def _integrate_orientation(self, inc_angle):
        """
        Integrate orientation quaternion/matrix.

        Parameters
        ----------
        inc_angel: array of float, rad, incremental vector
        """


        phi = self._state[0]
        lam = self._state[1]
        h = self._state[2]

        vn = self._state[3]
        ve = self._state[4]
        vd = self._state[5]


        RE, RN = self.datum.curvature(phi)

        wn_en = np.array([ve/(RE + h),
                          - vn/(RN + h),
                          - ve*np.tan(phi)/(RE + h)])

        wn_ie = np.array([self.datum.rate*np.cos(phi),
                          0.,
                          - self.datum.rate*np.sin(phi)])


        q_n_1 = self._state[9:]
        wn_in_q = rate2quat(wn_en + wn_ie)
        wb_ib_q = rate2quat(inc_angle)

        dq = 0.5*quat_mult(q_n_1, wb_ib_q) -\
             0.5*quat_mult(wn_in_q, q_n_1)

#        dq = 0.5*quat_mult(wn_in_q, q_n_1)

#        q = quat_prop(q_n_1, inc_angle - (wn_en + wn_ie)*self.dt)

        q = self.att_integrator(dq, self.dt)
        return  q

    def _integrate_velocity(self, inc_vel):
        """
        Integrate NED acceleration to NED velocity.
        """

        phi = self.state[0]
        lam = self.state[1]
        h = self.state[2]

        vn = self.state[3]
        ve = self.state[4]
        vd = self.state[5]


        RE, RN = self.datum.curvature(phi)

        wn_en = np.array([ve/(RE + h),
                          -vn/(RN + h),
                          -ve*np.tan(phi)/(RE + h)])

        wn_ie = np.array([self.datum.rate*np.cos(phi),
                          0., - self.datum.rate*np.sin(phi)])

        vp = np.array([vn, ve, vd])


        miu = 398600.44 * (10 ** 9)   # m^3/c^2
        ge = miu / (self.datum.a ** 2)

        # Gravity model
        g = ge * (1. - 2. * (h / self.datum.a) + \
                  0.75 * self.datum.e2 * (np.sin(phi) ** 2))
        gn = np.array([0., 0., g])

        f_ned = np.dot(self.orientation_dcm, inc_vel)
        # main navigation equation in matrix form
        dvn = f_ned - np.dot(skew(2.*wn_ie + wn_en), vp) + gn

        v = self.vel_integrator(dvn, self.dt)
        return v, f_ned

    def _integrate_position(self):
        """
        Integrate velocity in NED frame to position on Earth.
        """
        phi = self.state[0]
        lam = self.state[1]
        h = self.state[2]

        vn = self.state[3]
        ve = self.state[4]
        vd = self.state[5]


        RE, RN = self.datum.curvature(phi)

        dphi = vn/(RN + h)
        dlam = ve/((RE + h)*np.cos(phi))
        dh = -vd

        dpos = np.array([dphi, dlam, dh])

        pos = self.pos_integrator(dpos, self.dt)

        #self._state[0:3] = pos
        return pos[:]


    @property
    def orientation_dcm(self):
        """
        Returns orientation matrix.
        """
        return quat2dcm(self.state[9:])


    @property
    def position(self):
        """
        Returns position: lat, lon, h.
        """
        return self.state[0:3]


    @property
    def velocity(self):
        """
        Returns velocity in NED frame.
        """
        return self.state[3:6]

    @property
    def acceleration(self):
        """
        Returns acceleration in NED frame.
        """
        return self.state[6:9]


class SimpleINSComputer2(SimpleINSComputer):
    """
    Base class for INS computers.
    """
    def _integrate_orientation(self, inc_angle):
        """
        Integrate orientation quaternion/matrix.

        Parameters
        ----------
        inc_angel: array of float, rad, incremental vector
        """


        phi = self._state[0]
        lam = self._state[1]
        h = self._state[2]

        vn = self._state[3]
        ve = self._state[4]
        vd = self._state[5]


        RE, RN = self.datum.curvature(phi)

        wn_en = np.array([ve/(RE + h),
                          - vn/(RN + h),
                          - ve*np.tan(phi)/(RE + h)])

        wn_ie = np.array([self.datum.rate*np.cos(phi),
                          0.,
                          - self.datum.rate*np.sin(phi)])


        q_n_1 = self._state[9:]
#        wn_in_q = rate2quat(wn_en + wn_ie)
#        wb_ib_q = rate2quat(inc_angle)

#        dq = 0.5*quat_mult(q_n_1, wb_ib_q) -\
#             0.5*quat_mult(wn_in_q, q_n_1)

#        dq = 0.5*quat_mult(wn_in_q, q_n_1)

        q = quat_prop(q_n_1, inc_angle - (wn_en + wn_ie)*self.dt)

#        q = self.att_integrator(dq, self.dt)
        return  q
