from abc import ABCMeta, abstractmethod, abstractproperty
import numpy as np

from openins.environnement.datum import WGS84
from openins.ns.navsystem import NavSystem

class GNSS(NavSystem):
    """
    Base class for INS computers.
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        """
        Init data for Simple INS.
        """
        # sample time
        self.dt = 1.
        # datum
        self.datum = WGS84()
        # state vector
        self._state = np.zeros(6)
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

        return self._state


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


    #    @state.setter
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

    @property
    def position(self):
        """
        Returns position: lat, lon, h.
        """
        return self._state[:3]

    @property
    def velocity(self):
        """
        Returns velocity in NED frame.
        """
        return self._state[3:]




class SimpleGNSS(NavSystem):
    """
    Base class for INS computers.
    """

    def __init__(self):
        """
        Init data for Simple INS.
        """
        # sample time
        self.dt = 1.
        # datum
        self.datum = WGS84()
        # state vector
        self._state = np.zeros(6)
        # initial quaternion
        self._noise_std = np.array([
            20/self.datum.a, # latitude std deviation, rad
            20/self.datum.a, # longitude std deviation, rad
            40, # height std deviation, m
            0.05, # latitude std deviation, m/s
            0.05, # latitude std deviation, m/s
            0.05, # latitude std deviation, m/s
        ])

    def __call__(self, ideal_state ):
        """
        Run GNSS computer.

        Parameters
        ----------
        ideal_state: vector of ideal coordinates
            1:3 lat, lon, h
            4:6 vn, ve, vd
        """
        self._state = ideal_state + self._noise_std* \
                                    np.random.randn(np.size(self._state))
        return self._state[:]


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


    #    @state.setter
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

    @property
    def position(self):
        """
        Returns position: lat, lon, h.
        """
        return self._state[:3]

    @property
    def velocity(self):
        """
        Returns velocity in NED frame.
        """
        return self._state[3:]
