from scipy.integrate import romberg
import numpy as np


from navtrajectory_cf cimport *

cdef extern from "math.h":
    double cos(double theta)
    double sin(double theta)
    double atan(double theta)
    double acos(double theta)
    double asin(double theta)

#cdef PathParam pd
#
#pd.k11 = 55*pi/180.0
#pd.k12 = 0.0
#pd.k13 = 0.004
#pd.k14 = 2*pi/600.0
#pd.k15 = 0.0
#
#pd.k21 = 30.0*pi/180
#pd.k22 = 0.00004
#pd.k23 = 0.0
#pd.k24 = 2.0*pi/600.0
#pd.k25 = 0.0
#
#pd.k31 = 5000.0
#pd.k32 = 0.0
#pd.k33 = -5000.0
#pd.k34 = 2*pi/600.0
#pd.k35 = 0.0
#
#pd.kg = 0.01
#pd.datum_rate = 7.292115E-5
#pd.datum_a = 6378137.0
#pd.datum_e2 = 0.00669438000426
#pd.ge =398600.44*(10**9)/(6378137.0**2)


cdef class NavTrajectoryOpt:

    cdef PathParam pd

    def __cinit__(self):

        self.pd.k11 = 55*pi/180.0
        self.pd.k12 = 0.0
        self.pd.k13 = 0.004
        self.pd.k14 = 2*pi/600.0
        self.pd.k15 = 0.0

        self.pd.k21 = 30.0*pi/180
        self.pd.k22 = 0.00004
        self.pd.k23 = 0.0
        self.pd.k24 = 2.0*pi/600.0
        self.pd.k25 = 0.0

        self.pd.k31 = 5000.0
        self.pd.k32 = 0.0
        self.pd.k33 = -5000.0
        self.pd.k34 = 2*pi/600.0
        self.pd.k35 = 0.0

        self.pd.kg = 0.01
        self.pd.datum_rate = 7.292115E-5
        self.pd.datum_a = 6378137.0
        self.pd.datum_e2 = 0.00669438000426

        self.pd.ge =398600.44*(10**9)/(6378137.0**2)

    def phi(self, double time):
        return phi(time, &self.pd)
    def lam(self, double time):
        return lam(time, &self.pd)
    def h(self, double time):
        return h(time, &self.pd)

    def vn(self, double time):
        return vn(time, &self.pd)
    def ve(self, double time):
        return ve(time, &self.pd)
    def vd(self, double time):
        return vd(time, &self.pd)

    def an(self, double time):
        return an(time, &self.pd)
    def ae(self, double time):
        return ae(time, &self.pd)
    def ad(self, double time):
        return ad(time, &self.pd)

    def gamma(self, double time):
        return gamma(time, &self.pd)
    def theta(self, double time):
        return theta(time, &self.pd)
    def psi(self, double time):
        return psi(time, &self.pd)

    def wx(self, double time):
        return wx(time, &self.pd)
    def wy(self, double time):
        return wy(time, &self.pd)
    def wz(self, double time):
        return wz(time, &self.pd)

    def ax(self, double time):
        return ax(time, &self.pd)
    def ay(self, double time):
        return ay(time, &self.pd)
    def az(self, double time):
        return az(time, &self.pd)

    def gyros(self, double time):
        return   np.array([wx(time, &self.pd),  wy(time, &self.pd), wz(time, &self.pd)])

    def accs(self, double time):
        return  np.array([ax(time, &self.pd), ay(time, &self.pd), az(time, &self.pd)])

    ###################################################################

    def euler2quat(self, double gamma, double theta, double psi):
        cdef double q0, q1, q2, q3

        q0 = cos(gamma / 2) * cos(theta / 2) * cos(psi / 2) +\
             sin(gamma / 2) * sin(theta / 2) * sin(psi / 2)
        q1 = sin(gamma / 2) * cos(theta / 2) * cos(psi / 2) -\
             cos(gamma / 2) * sin(theta / 2) * sin(psi / 2)
        q2 = cos(gamma / 2) * sin(theta / 2) * cos(psi / 2) +\
             sin(gamma / 2) * cos(theta / 2) * sin(psi / 2)
        q3 = cos(gamma / 2) * cos(theta / 2) * sin(psi / 2) -\
             sin(gamma / 2) * sin(theta / 2) * cos(psi / 2)

        return np.array([q0, q1, q2, q3])

    def navtrajectory_statex(self, double t):

        q = self.euler2quat(gamma(t, &self.pd), theta(t, &self.pd), psi(t, &self.pd))
        return np.array([phi(t, &self.pd), lam(t, &self.pd), h(t, &self.pd),
                         vn(t, &self.pd), ve(t, &self.pd), vd(t, &self.pd),
                         an(t, &self.pd), ae(t, &self.pd), ad(t, &self.pd),
                         q[0], q[1],q[2],q[3],
                         wx(t, &self.pd), wy(t, &self.pd), wz(t, &self.pd),
                         ax(t, &self.pd), ay(t, &self.pd), az(t, &self.pd)])

    def navtrajectory_state(self, double t):

        q = self.euler2quat(gamma(t, &self.pd), theta(t, &self.pd), psi(t, &self.pd))
        return  np.array([phi(t, &self.pd), lam(t, &self.pd), h(t, &self.pd),
                          vn(t, &self.pd), ve(t, &self.pd), vd(t, &self.pd),
                          an(t, &self.pd), ae(t, &self.pd), ad(t, &self.pd),\
                          q[0], q[1],q[2],q[3]])



    ###################################################################
    def state(self, double time):
        return self.navtrajectory_state(time)

    def state_extended(self, time):
        return self.navtrajectory_statex(time)

    def orientation(self, time):
        return  np.array([gamma(time, &self.pd), theta(time, &self.pd), psi(time, &self.pd)])

    def orientation_q(self, time):
        q = self.euler2quat(gamma(time, &self.pd), theta(time, &self.pd), psi(time, &self.pd))
        return np.array([q[0], q[1],q[2],q[3]])

    def position(self, time):
        return  np.array([phi(time, &self.pd), lam(time, &self.pd), h(time, &self.pd)])

    def velocity_ned(self, time):
        return  np.array([vn(time, &self.pd), ve(time, &self.pd), vd(time, &self.pd)])

    def acceleration_ned(self, time):
        return  np.array([an(time, &self.pd), ae(time, &self.pd), ad(time, &self.pd)])




    def navtrajectory_euler(self, double t):
        return  np.array([gamma(t, &self.pd), theta(t, &self.pd), psi(t, &self.pd)])



    def init_state(self ):
        return self.navtrajectory_state(0.0)

    def gyros_inc_angle(self, time, dt):
        """
        Gyros with incermental angle ouput, typical afre VDC or ADC.

        From real IMU in most case we obrain incermental angles rather then
        anglual velocities. Its easier to use in algorithms.

        """
        if time < dt:
            return np.zeros(3)

        a = time - dt
        b = time
        inc_angle = np.zeros(3)
        inc_angle[0] = romberg(self.wx, a, b, vec_func=False)
        inc_angle[1] = romberg(self.wy, a, b, vec_func=False)
        inc_angle[2] = romberg(self.wz, a, b, vec_func=False)
        return inc_angle

    def accs_inc_velocity(self, time, dt):
        """

        """
        if time < dt:
            return np.zeros(3)

        a = time - dt
        b = time
        inc_angle = np.zeros(3)
        inc_angle[0] = romberg(self.ax, a, b, vec_func=False)
        inc_angle[1] = romberg(self.ay, a, b, vec_func=False)
        inc_angle[2] = romberg(self.az, a, b, vec_func=False)
        return inc_angle

