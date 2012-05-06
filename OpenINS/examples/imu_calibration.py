import numpy as np

from tools.calibration import DieselCalibrator


if __name__ == '__main__':

    # First we need set up initial conditions, like local gravity,
    # latitude, maybe Earth rate.

    # dummy variables, used if we want find local gravity
    # based on WSS84 (or any other datum) model
    longitude = np.deg2rad(30.)
    h = 0.

    local_g = 9.8105526 # m/s^2
    latitude = np.deg2rad(50.43899) # rad

    mc = DieselCalibrator(latitude, longitude, h)

    # Set magnitude of local gravity vector
    mc.set_gravity(local_g)
    # Set sample time
    mc.dt = 0.005 # s, (200Hz)

    # LOAD DATA FROM FILES
    # assumed that, each file has 6 colomns
    data_set1 = np.loadtxt('filename1.txt')
    data_set2 = np.loadtxt('filename1.txt')
    data_set3 = np.loadtxt('filename1.txt')

    mc.load_data(data_set1, data_set2, data_set3)



    t0 = 20./self.dt -self.dt #
    # set time before each rotation
    t1 = 0./self.dt
    t2 = (20. + 5.)/self.dt
    t3 = (20. + 5. + 20. + 5.)/self.dt
    t4 = (20. + 5. + 20. + 5. + 20. + 5.)/self.dt


    # schedule of rotations
    # t0: time shift
    tbl = np.array([[t0, t1, t2, t3, t4],
        [t0, t1, t2, t3, t4],
        [t0, t1, t2, t3, t4]])

    dv1, dv2, dv3 = mc.calc_coefficient(tbl)
    mc.gyro_report()
    mc.acc_report()
