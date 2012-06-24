"""
Base module for trajectory generators
"""

from abc import ABCMeta, abstractmethod
import numpy as np
import sympy as sp
import scipy as scp


from openins.environnement.datum import InitPosition
from openins.trajectory.basetrajectory import BasicTrajectory
from openins.orientationmath.orientation import euler2dcm
from openins.orientationmath.orientation import dcm2quat

class NavTrajectory(BasicTrajectory):
    """
    Base class for trajectory generators for INS that
    works in Local Geodesic Frame.

    You still could use it even if you prefer Inertial
    Frame, sensors output and 3D trajectory will be the same.
    """
    pd = np.array([[55.0 * (np.pi / 180.0), 0.0, 0.004, 2. * np.pi / 600.0, 0.],
        [30.0 * (np.pi / 180.0), 0.00004, 0, 2. * np.pi / 600.0, 0.],
        [5000.0, 0.0, -5000.0, 2. * np.pi / 600.0, 0.]])

    def __init__(self, pd=pd, kg=0.01):
        """
        Init some essential consts.
        """

        self.ipos = InitPosition(lat=0.87, lon=0.52, height=0.)
        self._profile(pd, kg)


    def _profile(self, pd, kg):
        """
        Set basic trajectory equation.
        """

        # Definition of symbolic variables
        t = sp.Symbol('t')
        phi, lam, h = sp.symbols('phi lam h')
        ve, vn, vd = sp.symbols('vn ve vd')
        ae, an, ad = sp.symbols('an ae ad')
        theta, psi, gamma = sp.symbols('theta psi gamma')
        wx, wy, wz = sp.symbols('wx wy wz')
        ax, ay, az = sp.symbols('ax ay az')

        g = sp.symbols('g')
        re, rn = sp.symbols('re rn')

        # defines lat, lon and height as functions of time
        phi = pd[0, 0] + pd[0, 1] * t + pd[0, 2] * sp.sin(pd[0, 3]\
                                                          * t + pd[0, 4])
        lam = pd[1, 0] + pd[1, 1] * t + pd[1, 2] * sp.sin(pd[1, 3]
                                                          * t + pd[1, 4])
        h = pd[2, 0] + pd[2, 1] * t + pd[2, 2] * sp.cos(pd[2, 3]\
                                                        * t + pd[2, 4])

        # curvature of Earth as function of phi
        re = self.ipos.datum.a / sp.sqrt(1 - self.ipos.datum.e2 * sp.sin(phi) **2)
        rn = self.ipos.datum.a*(1. - self.ipos.datum.e2) / (sp.sqrt(1 - self.ipos.datum.e2 *
                                                                       sp.sin(phi)**2)**3)

        # find velocities in NED frame as derivative of lat, lon and h
        vn = sp.diff(phi, t)*(rn + h)
        ve = sp.diff(lam, t)*(re + h)*sp.cos(phi)
        vd = - sp.diff(h, t)
        vr = sp.sqrt(ve ** 2 + vn ** 2)


        # gravity model
        miu = 398600.44*(10**9) # m^3/c^2
        ge = miu/(self.ipos.datum.a**2)
        g = ge*(1.0 - 2.0*(h/self.ipos.datum.a) +
                0.75*self.ipos.datum.e2*(sp.sin(phi)**2))

        # find acceleration in NED frame as derivative of velocities
        u = sp.diff(lam, t) + 2.*self.ipos.datum.rate

        an = sp.diff(vn, t) + u*sp.sin(phi)*ve -\
             sp.diff(phi, t)*vd
        ae = sp.diff(ve, t) - u*sp.sin(phi)*vn -\
             u*sp.cos(phi)*vd
        ad = sp.diff(vd, t) + u*sp.cos(phi)*ve +\
             sp.diff(phi,t)*vn - g

        # define attitude of IMU through NED velocities
        if vr == 0.:
            theta = 0.
            gamma = 0.
            psi = 0.
        else:

            theta = self._atan2mod(-vd, vr)
            if vn == 0.:
                theta = 0.
#            psi = sp.atan(ve/vn)
            psi = self._atan2mod(ve, vn)
            gamma = kg*(vn*sp.diff(ve,t) -
                        ve*sp.diff(vn,t))/vr*sp.cos(theta)

        # find angular rate and derivative of orientation
        # projected on body frame

        wib_x = sp.diff(gamma, t) - sp.diff(psi, t)*sp.sin(theta)
        wib_y = sp.diff(theta, t)*sp.cos(gamma) + \
                sp.diff(psi, t)*sp.cos(theta)*sp.sin(gamma)
        wib_z = sp.diff(psi, t)*sp.cos(theta)*sp.cos(gamma) - \
                sp.diff(theta, t)*sp.sin(gamma)




        # define transport rate

        omega_n = self.ipos.datum.rate*sp.cos(phi) + \
                  sp.diff(lam, t)*sp.cos(phi)
        omega_e = - sp.diff(phi, t)
        omega_d = - self.ipos.datum.rate*sp.sin(phi) - \
                  sp.diff(lam, t)*sp.sin(phi)


        # rotation psi about z axis
        C1 = sp.Matrix([[sp.cos(psi), sp.sin(psi), 0.],
            [-sp.sin(psi), sp.cos(psi), 0.],
            [0., 0. , 1.]])

        # rotation theta about y axis
        C2 = sp.Matrix([[sp.cos(theta), 0., -sp.sin(theta)] ,
            [0., 1., 0.          ],
            [sp.sin(theta), 0., sp.cos(theta)]])

        # rotation gamma about x axis
        C3 = sp.Matrix([[1., 0., 0.         ],
            [0., sp.cos(gamma), sp.sin(gamma)],
            [0., -sp.sin(gamma), sp.cos(gamma)]])

        # calculate body to navigation DCM
        dcm =  C1.T*C2.T*C3.T

        # project transport rat on body frame
        win = dcm.T*sp.Matrix([omega_n, omega_e, omega_d])

        ax, ay, az = dcm.T*sp.Matrix([an, ae, ad])


        # Body rate measured by gyros
        # Sum of body rotation in inertial space and movement of
        # navigation frame (transport rate) plus Earth rate

        wx = wib_x + win[0]
        wy = wib_y + win[1]
        wz = wib_z + win[2]

        #print sp.pretty_print(wz)
        #print sp.latex(wz)

        self.dcmbn = sp.lambdify(t, dcm, "numpy")


        self.omega_n = sp.lambdify(t, omega_n, "numpy")
        self.omega_e = sp.lambdify(t, omega_e, "numpy")
        self.omega_d = sp.lambdify(t, omega_d, "numpy")


        self.wib_x = sp.lambdify(t, wib_x, "numpy")
        self.wib_y = sp.lambdify(t, wib_y, "numpy")
        self.wib_z = sp.lambdify(t, wib_z, "numpy")

        self.phi = sp.lambdify(t, phi, "numpy")
        self.lam = sp.lambdify(t, lam, "numpy")
        self.h = sp.lambdify(t, h, "numpy")

        self.vn = sp.lambdify(t, vn, "numpy")
        self.ve = sp.lambdify(t, ve, "numpy")
        self.vd = sp.lambdify(t, vd, "numpy")

        self.an = sp.lambdify(t, an, "numpy")
        self.ae = sp.lambdify(t, ae, "numpy")
        self.ad = sp.lambdify(t, ad, "numpy")

        self.wx = sp.lambdify(t, wx, "numpy")
        self.wy = sp.lambdify(t, wy, "numpy")
        self.wz = sp.lambdify(t, wz, "numpy")

        self.ax = sp.lambdify(t, ax, "numpy")
        self.ay = sp.lambdify(t, ay, "numpy")
        self.az = sp.lambdify(t, az, "numpy")

        self.gamma = sp.lambdify(t, gamma, "numpy")
        self.theta = sp.lambdify(t, theta, "numpy")
        self.psi = sp.lambdify(t, psi, "numpy")

    def _atan2mod(self, y, x):
        """
        Custom symbolic atan2.

        Sympy has its own atan2 function, but you cannot obtain symbolic
        derivative from it. Look like Sympy do not know how to deel with
        signum or something.. So it just quick-hack we just substitute
        atan2 with equivalent function [1].

        Parameters
        ----------
        y: symbolic expr
        x: symbolic epr

        References
        ----------
        [1] http://en.wikipedia.org/wiki/Atan2
        """
        return 2*sp.atan(y/(sp.sqrt(x**2 + y**2 +x)))


    def accs(self, time):
        """
        Returns measurements of ideal accelerometers
        """
        acc = np.array([self.ax(time), self.ay(time), self.az(time)])
        return acc

    def gyros(self, time):
        """
        Returns measurements of ideal gyros
        """
        gyros = np.array([self.wx(time), self.wy(time), self.wz(time)])
        return gyros




    def init_orientation(self):
        """
        Returns initial orientation of IMU.
        """
        return np.array([self.gamma(0.), self.theta(0.), self.psi(0.)])

    def init_position(self):
        """
        Returns initial position of IMU.
        """
        return np.array([self.phi(0.), self.lam(0.), self.h(0.)])

    def init_velocity(self):
        """
        Returns initial velocity of IMU.
        """
        return np.array([self.vn(0.), self.ve(0.), self.vd(0.)])

    def init_state(self):
        """
        Returns initial state of IMU.
        """
        q = dcm2quat(self.dcmbn(0.))
        return np.array([self.phi(0.), self.lam(0.), self.h(0.),
                         self.vn(0.), self.ve(0.), self.vd(0.),
                         self.an(0.), self.ae(0.), self.ad(0.),
                         q[0], q[1], q[2], q[3]])

    def orientation(self, time):
        """
        Returns orientation of IMU in given time.
        """
        euler = np.array([self.gamma(time), self.theta(time), self.psi(time)])
        return euler


    def position(self, time):
        """
        Returns position of IMU in given time.
        """
        pos = np.array([self.phi(time), self.lam(time), self.h(time)])
        return pos

    def velocity_ned(self, time):
        """
        Returns position of IMU in given time.
        """
        vel = np.array([self.vn(time), self.ve(time), self.vd(time)])
        return vel

    def state(self, time):
        """
        Returns entire state of ideal ns.
        """
        q = dcm2quat(self.dcmbn(time))
        return np.array([self.phi(time), self.lam(time), self.h(time),
                         self.vn(time), self.ve(time), self.vd(time),
                         self.an(time), self.ae(time), self.ad(time),
                         q[0], q[1], q[2], q[3]])


    def state_extended(self, time):
        """
        Returns entire state of ideal ns.
        """
        q = dcm2quat(self.dcmbn(time))
        return np.array([self.phi(time), self.lam(time), self.h(time),
                         self.vn(time), self.ve(time), self.vd(time),
                         self.an(time), self.ae(time), self.ad(time),
                         q[0], q[1], q[2], q[3],
                         self.wx(time), self.wy(time), self.wz(time),
                         self.ax(time), self.ay(time), self.az(time)])


class NavTrajectoryIntegral(NavTrajectory):
    def gyros_inc_angle(self, time, dt):
        """

        """
        if time == 0.:
            return np.zeros(3)
        a = time - dt
        b = tiem

        return scp.integrate.romberg(self.gyros, a, b, vec_func=True)

    def accs_inc_velocity(self, time, dt):
        """

        """
        if time == 0.:
            return np.zeros(3)

        a = time - dt
        b = tiem
        return scp.integrate.romberg(self.gyros, a, b, vec_func=True)



if __name__ == "__main__":

   pass
