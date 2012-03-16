"""
Base module for trajectory generators
"""

from abc import ABCMeta, abstractmethod
import numpy as np
import sympy as sp

from environnement.datum import WGS84


class BasicTrajectory(object):
    """
    Represents basic class for all trajectory
    """

    __metaclass__ = ABCMeta

    @abstractmethod
    def gyros(self):
        """
        Generate output of angular rate sensors in inertial frame. 
        """

    @abstractmethod
    def accs(self):
        """
        Generate output of accelerometers in inertial frame.
        """

    @abstractmethod
    def init_position(self):
        """
        Returns initial position of IMU.
        """

    @abstractmethod
    def init_orientation(self):
        """
        Returns initial orientation of IMU.
        """
   
    @abstractmethod
    def position(self):
        """
        Returns 3D trajectory of body movement.
        """
    
    @abstractmethod
    def orientation(self):
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
        self._latitude  = np.deg2rad(50)
        self._longitude = np.deg2rad(30)
        
        self._height = 0
        # Set default Earth Datum
        self.datum = WGS84()
        # Local gravity in m/s^2
        self._g = 9.8
        # Initial orientation
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def init_position(self):

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

        if (9.6 < input_g and input_g > 9.9):
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
            if g:
                self._g = self.datum.gravity(self._latitude, self._height)

        else:
            raise ValueError('Latitude must be in interval +/- pi/2 rad'
                             'Longitude must be in interval +- pi rad')
            
class DiselCalibrator(CalibTrajectory):
    """
    Trajectory for Disel calibration method.
    """
    
    def orientation(self, t):
        """
        Returns orientation for specified time. 
        """
                  
class RotationSequence(object):
    """
    Container for rotation sequence
    """

    def __init__(self, arg):
        self.arg = arg
        self.sequence = []



class RotatateIMU(object):
    """
    """
    def __init__(self, t, axis, angle):
        """
        Init rotation object
        """
        self.time = t
        self.start_time = 0.0
        self.axis = axis
        self.angle = angale
        self.rate = self.calc_rate(self.t, self.angle)

    def calc_rate(self, t, angle):
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

class ProfileBuilder(object):
    """
    Build movement of calibration table.
    """
    def __init__(self, rate = np.pi/20.):
        """
        Init table turn rate.

        By default you need 20 seconds to rotate calibration table on pi
        angle.

        Parameters
        ----------
        rate: float, rad, turn rate of calibration table
        """
        self._turn_rate = rate

    def rotate(self, t):
        return self._turn_rate * t
    
    def rate(self, t):
        return self._turn_rate 


#class NavTrajectory(BasicTrajectory):
#
#    """
#    Base class for trajectory generators for INS that
#    works in Local Geodesic Frame. 
#    
#    You still could use it even if you prefer Inertial 
#    Frame, sensors output and 3D trajectory will be the same.
#    """
#    
#    def __init__(self, pd):
#        """
#        Init some essential consts.
#        """
#
#        self.datum = WGS84()
#        self.profile(pd)        
#       
#       
#       # Symbolic variables 
#       # self.t = sp.Symbol('t') 
#       # self.phi, self.lam, self.h = sp.symbols('phi lam h')
#       # self.ve, self.vn, self.vu = sp.symbols('ve vn vu')
#       # self.ae, self.an, self.au = sp.symbols('ae an au')
#       # self.theta, self.psi, self.gamma = sp.symbols('theta psi gamma')
#       # self.wx, self.wy, self.wz = sp.symbols('wx wy wz')
#       # self.ax, self.ay, self.az = sp.symbols('ax ay az')
#    
#       # self.g = sp.symbols('g')
#       # self.RE, self.RN = sp.symbols('RE RN')
#
#
#    def profile(self, pd):
#        """
#        Set basic trajectory equation. 
#        """
#        kg = 0.01
#        
#        t = sp.Symbol('t') 
#        phi, lam, h = sp.symbols('phi lam h')
#        ve, vn, vd = sp.symbols('vn ve vd')
#        ae, an, ad = sp.symbols('an ae ad')
#        theta, psi, gamma = sp.symbols('theta psi gamma')
#        wx, wy, wz = sp.symbols('wx wy wz')
#        ax, ay, az = sp.symbols('ax ay az')
#    
#        g = sp.symbols('g')
#        re, rn = sp.symbols('re rn')
#
#        phi = pd[0, 0] + pd[0, 1] * t + pd[0, 2] * sp.sin(pd[0, 3] \
#                   * t + pd[0, 4])
#        lam = pd[1, 0] + pd[1, 1] * t + pd[1, 2] * sp.sin(pd[1, 3] \
#                   * t + pd[1, 4])
#        h = pd[2, 0] + pd[2, 1] * t + pd[2, 2] * sp.cos(pd[2, 3] \
#                   * t + pd[2, 4])
#        
#        re = self.datum.a / sp.sqrt(1 - self.datum.e2 * sp.sin(phi) **2)
#        rn = self.datum.a*(1 - self.datum.e2) / (sp.sqrt(1 - self.datum.e2 * \
#             sp.sin(phi)**2)**3)
#        
#        vn = sp.diff(phi, t)*(rn + h)
#        ve = sp.diff(lam, t)*(re + h)*sp.cos(phi)
#        vd = - sp.diff(h, t)
#        vr = sp.sqrt(ve ** 2 + vn ** 2)  
#        
#        miu = 398600.44*(10**9) # m^3/c^2  
#        ge = miu/(self.datum.a**2);
#        g = ge*(1.0 - 2.0*(h/self.datum.a)+0.75*self.datum.e2*(sp.sin(phi)**2)); 
#        
# 
#        q = sp.diff(lam, t) + 2 * self.datum.rate       
#                
#        an = sp.diff(vn, t) + q * sp.sin(phi) * ve - \
#             sp.diff(phi, t) * vd
#        ae = sp.diff(ve, t) - q * sp.sin(phi) * vn - \
#             q * sp.cos(vd)
#        ad = sp.diff(vd, t) + q * sp.cos(phi) * ve + \
#             sp.diff(phi,t) * vn - g 
#
#        theta = sp.atan(-vd/vr)
#        psi = sp.atan(ve/vn)
#        gamma = kg*(vn*sp.diff(ve,t) - \
#                ve*sp.diff(vn,t))/vr*sp.cos(theta) 
#        
#        
#        wib_x = sp.diff(gamma,t) + sp.diff(psi,t)*sp.sin(theta)
#        wib_y = sp.diff(psi,t) * sp.cos(theta) * sp.cos(gamma) + \
#                sp.diff(theta,t) * sp.sin(gamma)
#        wib_z = sp.diff(theta,t)*sp.cos(gamma) - sp.diff(psi,t)* \
#                sp.cos(theta) * sp.sin(gamma)
#        
#        omega_n = (self.datum.rate + sp.diff(lam, t))*sp.cos(phi)
#        omega_e = -sp.diff(phi, t)
#        omega_d = - (self.datum.rate + sp.diff(lam, t))*sp.sin(phi)
#        
#        omega = sp.Matrix([omega_n,omega_e,omega_d])
#      
#
#
#
#        # rotation psi about z axis
#        C1 = sp.Matrix([[sp.cos(psi), sp.sin(psi), 0], \
#                        [-sp.sin(psi), sp.cos(psi), 0], \
#                        [0, 0 , 1]])
#                 
#        # rotation theta about y axis
#        C2 = sp.Matrix([[sp.cos(theta), 0, -sp.sin(theta)] , \
#                        [0, 1, 0          ], \
#                        [sp.sin(theta), 0, sp.cos(theta)]])        
#        
#        # rotation gamma about x axis
#        C3 = sp.Matrix([[1, 0, 0         ], \
#                        [0, sp.cos(gamma), sp.sin(gamma)], \
#                        [0, -sp.sin(gamma), sp.cos(gamma)]])
#        
#        # calculate body to navigation DCM    
#        #DCMbn = np.dot(np.dot(C1.T , C2.T) , C3.T)
#        #DCMbn = (C1.T.dot(C2.T)).dot(C3.T)
#        
#        dcm =  C1.T*C2.T*C3.T 
#        ## win = dcm'*omega
#        win = dcm.T*omega
#        
#        ax, ay, az = dcm.T*sp.Matrix([an, ae, ad])
#
#        ## Body rate measured by gyros
#        ## Sum of body rotarion in inertial space and movement of 
#        ## navigation frame (transport rate)
#
#        wx = wib_x + win[0]
#        wy = wib_y + win[1]
#        wz = wib_z + win[2] 
#    
#        self.phi = sp.lambdify(t, phi, "numpy") 
#        self.lam = sp.lambdify(t, lam, "numpy")
#        self.h = sp.lambdify(t,h, "numpy")
#        
#
#        self.vn = sp.lambdify(t, vn, "numpy")
#        self.ve = sp.lambdify(t, ve, "numpy") 
#        self.vd = sp.lambdify(t, vd, "numpy")
#       
#        self.an = sp.lambdify(t, an, "numpy")
#        self.ae = sp.lambdify(t, ae, "numpy") 
#        self.ad = sp.lambdify(t, ad, "numpy")
#       
#        self.wx = sp.lambdify(t, wx, "numpy")
#        self.wy = sp.lambdify(t, wy, "numpy") 
#        self.wz = sp.lambdify(t, wz, "numpy")
#        
# 
#        self.ax = sp.lambdify(t, ax, "numpy")
#        self.ay = sp.lambdify(t, ay, "numpy") 
#        self.az = sp.lambdify(t, az, "numpy")
# 
#        self.gamma = sp.lambdify(t, gamma, "numpy")
#        self.theta = sp.lambdify(t, theta, "numpy") 
#        self.psi = sp.lambdify(t, psi, "numpy")




if __name__ == "__main__":
    from visualisation.plotter import plot_trinity
    from visualisation.plotter import plot_trajectory

    pd = np.array([[55.0 * (np.pi / 180.0), 0.0, 0.004, 2 * np.pi / 600.0, 0],
                   [30.0 * (np.pi / 180.0), 0.00004, 0, 2 * np.pi / 600.0, 0],
                   [5000.0, 0.0, -5000.0, 2 * np.pi / 600.0, 0]])  
    
    #print 'start create profile'
    #profile = NavTrajectory(pd)
    #print 'end create profile'
    #print 'start CALCULATE'
    #time = np.arange(0,1000, 1)
    #phi =  map(profile.phi, time)
    #lam =  map(profile.lam, time)
    #h =  map(profile.h, time)
    
    #vn =  map(profile.vn, time)
    #ve =  map(profile.ve, time)
    #vd =  map(profile.vd, time)

    #an =  map(profile.an, time)
    #ae =  map(profile.ae, time)
    #ad =  map(profile.ad, time)
   
    #ax =  map(profile.ax, time)
    #ay =  map(profile.ay, time)
    #az =  map(profile.az, time)

    #wx =  map(profile.wx, time)
    #wy =  map(profile.wy, time)
    #wz =  map(profile.wz, time)
 
 
    #gamma =  map(profile.gamma, time)
    #theta =  map(profile.theta, time)
    #psi =  map(profile.psi, time)
 

    #print 'End CALCULATE'
    #plot_trajectory(phi, lam, h) 
    #lgnd = ['vn', 've', 'vd']
    #plot_trinity(time,np.array([vn, ve, vd]).T, lgnd)
    #
    #lgnd = ['an', 'ae', 'ad']
    #plot_trinity(time,np.array([an, ae, ad]).T, lgnd)
    
    #lgnd = ['ax', 'ay', 'az']
    #plot_trinity(time,np.array([ax, ay, az]).T, lgnd)
    #
    #lgnd = ['wx', 'wy', 'wz']
    #plot_trinity(time,np.array([wx, wy, wz]).T, lgnd)

    #lgnd = ['gamma', 'theta', 'psi']
    #plot_trinity(time,np.array([gamma, theta, psi]).T, lgnd)

