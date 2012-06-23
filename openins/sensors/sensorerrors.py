import numpy as np

from base import DataHandler


class StaticError(DataHandler):
    """
    Adds static errors to input signal.

    Generally static errors don't depend on time of sample time,
    they just add static components to output or implement
    nonlinear components.
    """


class ScaleFactor(StaticError):
    """
    Corrupt data with scalefactor error.

    Parameters
    ----------
    input: array like
    k: scale factor

    Returns
    -------
    output: arraylike
    """

    def __init__(self,k=1.):
        """
        Init error's parameters

        Parameters
        ----------
        k: flooat
            scale factor +1
        """

        self.k = k

    def handle(self,data):
        """
        Corrupt data with scale factor error
        """

        return self.k*data


class IdealOut(StaticError):
    """
    Ideal Error means no error at all.

    This object used to crate ideal sensor.
    Input == Output
    """
    def handle(self,data):
        """
        Ideal sensor output.
        """

        return data


class Bias(StaticError):
    """
    Add's bias to input data
    """

    def __init__(self, b=0):
        """
        Init error's parameters

        Parameters
        ----------
        b: float
            value error additive error
        """
        self.b = b

    def handle(self,data):
        """
        Corrupt data with bias error

        Parameters
        ----------
        data: float

        Returns
        -------
        output: float
        """
        return data + self.b


class DeadZone(StaticError):
    """
    Deadzone nonlinearity for input signal.

    Function implement deadzone nonlinearity for input signal. If data match
    specified band, output is 0. So deadzone = [-deadband; deadband]

    """

    def __init__(self,deadband=0.):
        """
        Init error's parameters

        Parameters
        ----------
        deadband: float
            deadzone = [-deadband; deadband]
        """
        self.deadband = deadband

    def handle(self,data):
        """
        Add's daedzone to input data

        If data occours in interval [-deadband; deadband] so output
        signal = 0.

        Parameters
        ----------
        data: float

        Returns
        -------
        data: float
            output data with effect of deadzone

        """
        if np.absolute(data) < np.absolute(self.deadband):
            data = 0

        return data


class StochasticError(DataHandler):
    """
    Implements stochastic errors of sensors.

    Generally this errors depend on time.

    References
    ----------
    [1] Grewal M. S. Andrews A.P. How Good Is Your Gyro?
        IEEE CONTROL SYSTEMS MAGAZINE, FEBRUARY 2010
    [2] Grewal M. S. Global Positioning Systems,
        Inertial Navigation and Integration
    """

    def __init__(self, dt):
        """
        Initialise data handler

        Parameters
        ----------
        dt: float,
            sec, sample time of input data
        """
        self._dt = dt

    @property
    def sample_time(self):
        """
        Returns sensor sample time
        """
        return self._dt


class RandomWalk(StochasticError):
    """
    Implements angular random walk.

    Modeled as white gaussian noise, witch after integration will add
    angular or velocity random walk.
    """
    def __init__(self, dt,deviation):
        """
        Initialise data handler

        Parameters
        ----------
        dt: float,
            sec, sample time of input data
        """
        self._dt = dt
        self.deviation = deviation

    def handle(self, tk, data):
        """
        Parameters
        ----------
        tk: array like
            current time
        data: array like
            input signal
        """

        return data + self.deviation*np.random.randn()/np.sqrt(self._dt)




