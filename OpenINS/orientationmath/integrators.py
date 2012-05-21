"""
Iterative integrator implementations.
"""

from abc import ABCMeta, abstractmethod
from copy import copy

class Integrator(object):
    """
    Base class for integrators.
    """
    __metaclass__ = ABCMeta

    def __init__(self, init_value):
        """
        Initialise integrator.

        Parameters
        ----------
        init_value: Initial value for integral.
        """
        self._accumulator = copy(init_value)

    @abstractmethod
    def __call__(self, sample_value, dt):
        """
        Update the integral with a new sample value and time step.

        Returns
        --------
        The updated integral value.
        """
        pass

class RectangleRule(Integrator):
    """
    Integration by the rectangle rule.
    """

    def __call__(self, sample_value, dt):
        self._accumulator += sample_value * dt
        return copy(self._accumulator)

class TrapeziumRule(Integrator):
    """
    Integration by the trapezium rule.
    """

    def __init__(self, init_value):
        """

        """
        super(TrapeziumRule, self).__init__(init_value)
        self._previous_value = None

    def __call__(self, sample_value, dt):
        """

        """
        if self._previous_value is not None:
            self._accumulator += dt * (self._previous_value + sample_value) / 2.
        else:
            self._accumulator += sample_value * dt
        self._previous_value = sample_value
        return copy(self._accumulator)

class SimpsonRule(Integrator):
    """
    Integration by the trapezium rule.
    """

    def __init__(self, init_value):
        """

        """
        super(SimpsonRule, self).__init__(init_value)
        self._prev_value_1 = None
        self._prev_value_2 = None
        self.a2 =0.
    def __call__(self, sample_value, dt):
        """

        """

        if self._prev_value_1 is not None and self._prev_value_2 is not None:
            self._accumulator += dt * (self._prev_value_2 +
                                       4.*self._prev_value_1 + sample_value) / 3.
            self._prev_value_2 =  copy(sample_value)
            self._prev_value_1 =  None
            self.a2 =0.
        else:
            if self._prev_value_2 is not None:

                self._prev_value_1 =  copy(sample_value)
                return copy(self._accumulator +self.a2 + dt * (self._prev_value_2 + sample_value) / 2.)
            else:
                self._prev_value_2 =  copy(sample_value)
                self.a2 = copy(self._accumulator + sample_value * dt)
                return copy(self._accumulator + sample_value * dt)

#        self._prev_value_2 =  copy(self._prev_value_1)
#        self._prev_value_1 =  copy(sample_value)



        return copy(self._accumulator)

