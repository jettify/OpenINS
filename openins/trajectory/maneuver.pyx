from navtrajectory_opt cimport NavTrajectoryOpt
from navtrajectory_cf cimport pi

cdef class VHelix(NavTrajectoryOpt):

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


cdef class HHelix(NavTrajectoryOpt):

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


cdef class Cirle(NavTrajectoryOpt):

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