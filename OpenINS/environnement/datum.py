import numpy as np
from abc import ABCMeta, abstractproperty, abstractmethod

from tools.documentation import  copy_method_doc

#    ## WGS-84 Earth datum (GPS)
#    wgs84 = (6378137.0,6356752.3142,7.292115E-5)
#    ## PZ-90  Earth datum (GLONASS)
#    PZ90 = (6378136.0, 6335438.4220,7.292115E-5)
#    ## SK42 old Russian datum
#    SK42 = (6378245.0, 6335552.6796,7.292115E-5)
#
#from tools.documentation import copy_method_doc


class EarthDatum(object):
    """
    Init and store data concerned Earth model.

    Model of the Earth is a oblate spheroid which approximates
    more closely to the true geometry.
    In accordance with this model, the following parameters may be defined:
    _a = Semimajor axis, m
    _b = Semiminor axis, m
    _f = flattening of ellipsoid
    _e = Major eccentricity (first eccentricity)
    _e2 = Major eccentricity squared
    _rate = rate of the Earth, rad/s

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
    #        miu = 398600.44*(10**9) # m^3/c^2
    #        self.ge = miu/(self._a**2);
    #        g = self.ge*(1 - 2*(h/self._a)+0.75*self._e2*(np.sin(phi)**2));
    #        return g

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

    Most Earth datums have similar math description, but differntces exist
    in constants (for instance semiminor and semimajor axis), so you can
    use WGS based math description of ellipsoid, but provide consts for
    chosen Eath Datum. In othe case you need subclass EathDatum and implement
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

        miu = 398600.44 * (10 ** 9)   # m^3/c^2
        self.ge = miu / (self.a ** 2)
        g = self.ge * (1 - 2 * (h / self.a) + \
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
        First derivative from WGS84 Datum.

        Reference
        ---------
        [1] Titterton D., Weston J., STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
        [2] NIMA TR8350.2: "Department of Defense World Geodetic System 1984,
            Its Definition and Relationship with Local Geodetic Systems."
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
    Class represents WGS84 Earth Datum
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
    Class represents Russian (GLONASS) PZ-90 Earth Datum
    """

    def __init__(self):
        ## Semimajor axis, m
        self._a = 6378137.0
        ## Semiminor axis, m
        self._b = 6356752.3142
        ## Earth rate, rad/sec
        self._rate = 7.292115E-5
