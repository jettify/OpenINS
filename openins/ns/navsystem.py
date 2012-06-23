from abc import ABCMeta, abstractmethod, abstractproperty
import numpy as np

from environnement.datum import WGS84

class NavSystem(object):
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

    @abstractproperty
    def position(self):
        """
        Returns position: lat, lon, h.
        """

    @abstractproperty
    def velocity(self):
        """
        Returns velocity in NED frame.
        """
