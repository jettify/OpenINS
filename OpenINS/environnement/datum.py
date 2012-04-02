import numpy as np
from abc import ABCMeta, abstractproperty, abstractmethod

from tools.documentation import  copy_method_doc

class EarthDatum(object):
    """
    Init and store data concerned Earth model.

    Model of the Earth is a oblate spheroid which approximates
    more closely to the true geometry.
    In accordance with this model, the following parameters may be defined:
    a = Semimajor axis, m
    b = Semiminor axis, m
    f = flattening of ellipsoid
    e = Major eccentricity (first eccentricity)
    e2 = Major eccentricity squared
    rate = rate of the Earth, rad/s

    """
    __metaclass__ = ABCMeta

    def __init__(self, a, b, rate):
        """
        Constructor of Earth datum, inits the Earth elipsoid class
        """
        ## Semimajor axis, m
        self._a = a
        ## Semiminor axis, m
        self._b = b
        ## Earth Rate
        self._rate = rate

    @abstractmethod
    def gravity(self, phi, h):
        """
        Calculate normal gravity for defined latitude and height above
        the Earth, dusing model of the Earth

        Parameters
        ----------
        phi: latitude, rad
        h: height above the Earth, m
        Return
        ------
        g: gravity acceleration, m/s^2

        TODO
        ----
        implement more precise method
        """

    @abstractmethod
    def curvature(self, phi):
        """
        Modelling the Earth in accordance with a reference ellipsoid, we need
        to know a *meridian radius of curvature* (RN) and a *transverse radius
        of curvature* (RE).

        Parameters
        ----------
        phi: float
            rad, geodedic latitude

        Returns
        -------
        RE: float
            m, transverse radius of curvature
        RN: float
            m, meridian radius of curvature
        """

    def dcurvature(self, phi, dphi):
        """
        Calculate derivative for curvature.

        Parameters
        ----------
        phi: rad
            geodedic latitude
        dphi:
            derivative of geodedic latitude ('latitude velocity')

        Returns
        -------
        RE: m
            transverse radius of curvature
        RN: m
            meridian radius of curvature
        dRN:
            derivative of meridian radius of curvature
        dRE:
            derivative of transverse radius of curvature
        """

    @property
    def a(self):
        """
        Returns semimajor axis of specified model of the Earth
        """
        return self._a

    @property
    def b(self):
        """
        Returns semiminor axis of specified model of the Earth
        """
        return self._b

    @abstractproperty
    def f(self):
        """
        Returns flattering of specified model of the Earth
        """

    @abstractproperty
    def e(self):
        """
        Returns the Earth's model curvature
        """

    @property
    def e2(self):
        """
        Returns square of e (Earth curvature)
        Note
        ----
        e2 = e^2
        """
        self._e2 = self.e ** 2
        return self._e2

    @property
    def rate(self):
        """
        Returns square of e (Earth curvature)
        Note
        ----
        e2 = e^2
        """
        return self._rate


class EarthBase(EarthDatum):
    """
    Class represents of WGS84 Earth Datum.

    Most Earth datums have similar math description, but differences exist
    in constants (for instance semiminor and semimajor axis), so you can
    use WGS based math description of ellipsoid, but provide consts for
    chosen Earth Datum. In other case you need subclass EathDatum and implement
    Earth model math, specified for you Earth datum description.
    """
    @property
    def f(self):
        self._f = (self._a - self._b) / self._a
        return self._f

    @property
    def e(self):
        self._e = np.sqrt(1 - ((self._b / self._a) ** 2))
        return self._e

    @copy_method_doc(EarthDatum)
    def gravity(self, phi, h):
        # TODO: check formula
        miu = 398600.44 * (10 ** 9)   # m^3/c^2
        self.ge = miu / (self.a ** 2)
        g = self.ge * (1. - 2. * (h / self.a) + \
                       0.75 * self.e2 * (np.sin(phi) ** 2))
        return g

    @copy_method_doc(EarthDatum)
    def curvature(self, phi):
        """
        Curvature for WGS84 Datum.

        Reference
        ---------
        [1] Titterton D., J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
        """
        RE = self.a / np.sqrt(1 - self.e2 * np.sin(phi) ** 2)
        RN = self.a * (1 - self.e2) / ((1 - self.e2 *  \
                                            np.sin(phi) ** 2) ** 1.5)

        return RE, RN

    @copy_method_doc(EarthDatum)
    def dcurvature(self, phi, dphi):
        """
        First derivative from curvature of WGS84 Datum.
        """

        RE, RN = self.curvature(phi)
        dRE = (self.a * self.e2 * np.sin(phi) * np.cos(phi) * dphi) / \
              (1 - self.e2 * np.sin(phi) ** 2) ** 1.5
        dRN = dRE * ((1 - self.e2) / (1 - self.e2 * np.sin(phi) ** 2)) +\
              RE * ((1 - self.e2) * 2 * self.e2 * np.sin(phi) *\
              np.cos(phi) * dphi) /\
              (1 - self.e2 * np.sin(phi) ** 2) ** 2
        return RE, RN, dRE, dRN


class WGS84(EarthBase):
    """
    Class represents WGS84 Earth Datum.

    Reference
    ---------
    [1] Titterton D., Weston J., STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] NIMA TR8350.2: "Department of Defense World Geodetic System 1984,
        Its Definition and Relationship with Local Geodetic Systems."
    """
    def __init__(self):
        ## Semimajor axis, m
        self._a = 6378137.0
        ## Semiminor axis, m
        self._b = 6356752.3142
        ## Earth rate, rad/sec
        self._rate = 7.292115E-5


class PZ90(EarthBase):
    """
    Class represents Russian (GLONASS) PZ-90 Earth Datum.
    """

    def __init__(self):
        # TODO: Find and change to actual values
        ## Semimajor axis, m
        self._a = 6378137.0
        ## Semiminor axis, m
        self._b = 6356752.3142
        ## Earth rate, rad/sec
        self._rate = 7.292115E-5


class InitPosition(object):
    """
    Holds data about initial position and orientation of object.

    By default gravity magnitude and Earth rate calculated based on
    data from WGS84 datum. If you have more precise values you probably
    should change it.

    Reference
    ---------
    [1] Titterton D., Weston J., STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] NIMA TR8350.2: "Department of Defense World Geodetic System 1984,
            Its Definition and Relationship with Local Geodetic Systems."
    [3] Paul D. Groves, Principles of GNSS, Inertial, and Multisensor
        Integrated Navigation Systems
    """

    def __init__(self, lat, lon, height):

        # Position
        self._latitude = None
        self._longitude = None
        self._height = None
        # Gravity magnitude for current position
        self._g = 0.

        # Set default Earth Datum
        self._datum = WGS84()

        # Special method construct position and gravity
        self.set_position(lat, lon, height)

        # Local gravity in m/s^2

        # Earth angular speed projected to NED frame
        self._omega_n = np.array([np.cos(self._latitude), 0.,
                                  -np.sin(self._latitude)])*self.datum.rate
        # Initial orientation
        # TODO: delete or not delete?
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    @property
    def gravity(self):
        """
        Returns local value of gravitational acceleration.
        """
        return self._g

    @property
    def lat(self):
        """
        Returns initial latitude.
        """
        return self._latitude

    @property
    def lon(self):
        """
        Returns initial longitude.
        """
        return self._longitude

    @property
    def h(self):
        """
        Returns initial height.
        """
        return self._height

    @property
    def datum(self):
        """
        Return Earth datum object.
        """
        return self._datum

    @property
    def omega_n(self):
        """
        Return Earth rate projected on NED frame.
        """
        return self._omega_n

    @datum.setter
    def _set_datum(self, datum):
        """
        Earth datum setter.
        """
        if isinstance(datum, EarthDatum):
            self._datum = datum
        else:
            raise TypeError('Must be instance of EarthDatum')

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

        if 9.6 < input_g > 9.9:
            self._g = input_g
        else:
            raise ValueError('g should be between near 9.79'
                             ' and not negative, in m/s^2')

    def set_position(self, phi, lam, h, calc_g=True):
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

        if (np.abs(phi) <= np.pi / 2.) and (np.abs(lam) <= np.pi ):
            # Set position
            self._latitude = phi
            self._longitude = lam
            self._height = h
            # Calc local gravity based on position and Earth Datum
            if calc_g:
                self._g = self.datum.gravity(self._latitude, self._height)

            self._omega_n = np.array([np.cos(self._latitude), 0.,
                                      -np.sin(self._latitude)])*self.datum.rate
        else:
            raise ValueError('Latitude must be in interval +/- pi/2 rad'
                             'Longitude must be in interval +- pi rad')
