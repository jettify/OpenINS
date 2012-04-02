import unittest
import numpy as np

from trajectory.calibtrajectory import RotationSequence
from trajectory.calibtrajectory import BasicCalibTraj
from trajectory.calibtrajectory import RotateIMU
from visualisation.plotter import plot_trinity
from tools.calibration import DieselCalibrator


class TestDieselCalibrator(unittest.TestCase):

    def setUp(self):

        self.turn_time = 20.
        self.stady_time = 20.
        self.dt = 0.05


        #Rotation set 1
        self.rs1 = RotationSequence()
        self.rs1.set_init_orientation(0., 0., 0. )

        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs1.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs1.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs1.add(RotateIMU(self.turn_time, 0., 0., np.pi))
        self.rs1.add(RotateIMU(self.stady_time, 0., 0., 0.))

        #Rotation set 2
        self.rs2 = RotationSequence()
        self.rs2.set_init_orientation( -np.pi/2., -np.pi/2,  0. )

        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs2.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs2.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs2.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
        self.rs2.add(RotateIMU(self.stady_time, 0., 0., 0.))

        #Rotation set 3
        self.rs3 = RotationSequence()
        self.rs3.set_init_orientation(np.pi/2, 0., np.pi/2)

        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs3.add(RotateIMU(self.turn_time, 0., 0., np.pi))
        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs3.add(RotateIMU(self.turn_time, 0., 0., np.pi))
        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))
        self.rs3.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
        self.rs3.add(RotateIMU(self.stady_time, 0., 0., 0.))



        self.fixture1 = BasicCalibTraj(self.rs1)
        self.fixture2 = BasicCalibTraj(self.rs2)
        self.fixture3 = BasicCalibTraj(self.rs3)


        self.time = np.arange(0., 140., self.dt)


        gyros1 = np.array(map(self.fixture1.gyros, self.time))
        gyros2 = np.array(map(self.fixture2.gyros, self.time))
        gyros3 = np.array(map(self.fixture3.gyros, self.time))

        accs1 = np.array(map(self.fixture1.accs, self.time))
        accs2 = np.array(map(self.fixture2.accs, self.time))
        accs3 = np.array(map(self.fixture3.accs, self.time))

        to_rad = (np.pi/180.)/3600.
        self.gyro_model = {'x': 0.03*to_rad, 'y':0.04*to_rad, 'z':0.05*to_rad,
                           'xx':0.0007, 'xy':0.0003, 'xz':0.0002,
                           'yx':0.0004, 'yy':0.0007, 'yz':0.0003,
                           'zx':0.0005, 'zy':0.0006, 'zz':0.0007,}

#        self.gyro_model = {'x': 0.00*to_rad, 'y':0.00*to_rad, 'z':0.00*to_rad,
#                           'xx':0.00005, 'xy':0.0003, 'xz':0.0000,
#                           'yx':0.0005, 'yy':0.00005, 'yz':0.0000,
#                           'zx':0.0000, 'zy':0.0000, 'zz':0.00005,}

        # acc parameters: bias and scalefactor, misalignment matrix
        self.acc_model = {'x': 0.003, 'y':0.004, 'z':0.005,
                           'xx':0.007, 'xy':0.000, 'xz':0.000,
                           'yx':0.000, 'yy':0.007, 'yz':0.000,
                           'zx':0.005, 'zy':0.0045, 'zz':0.007,}


        gsfma = np.array([[self.gyro_model['xx'], self.gyro_model['xy'], self.gyro_model['xz']],
            [self.gyro_model['yx'], self.gyro_model['yy'], self.gyro_model['yz']],
            [self.gyro_model['zx'], self.gyro_model['zy'], self.gyro_model['zz']]])

        gb = np.array([self.gyro_model['x'],
                              self.gyro_model['y'],
                              self.gyro_model['z']])

        asfma = np.array([
            [self.acc_model['xx'], self.acc_model['xy'], self.acc_model['xz']],
            [self.acc_model['yx'], self.acc_model['yy'], self.acc_model['yz']],
            [self.acc_model['zx'], self.acc_model['zy'], self.acc_model['zz']]])

        ab = np.array([self.acc_model['x'],
                       self.acc_model['y'],
                       self.acc_model['z']])


        # add scalefactor, misalignment and bias to accs and gyros
        gyros1 = (np.dot(gyros1, gsfma.T + np.eye(3)) + gb)*self.dt
        gyros2 = (np.dot(gyros2, gsfma.T + np.eye(3)) + gb)*self.dt
        gyros3 = (np.dot(gyros3, gsfma.T + np.eye(3)) + gb)*self.dt

        accs1 = (np.dot(accs1, asfma.T + np.eye(3)) + ab)
        accs2 = (np.dot(accs2, asfma.T + np.eye(3)) + ab)
        accs3 = (np.dot(accs3, asfma.T + np.eye(3)) + ab)

        self.data_set1 = np.concatenate((gyros1, accs1), axis=1)
        self.data_set2 = np.concatenate((gyros2, accs2), axis=1)
        self.data_set3 = np.concatenate((gyros3, accs3), axis=1)

        self.mc = DieselCalibrator()
        self.mc.dt = self.dt
        self.mc.time = self.time
        self.mc.load_data(self.data_set1, self.data_set2, self.data_set3)


        t0 = 20/self.dt -self.dt
        t1 = 0/self.dt
        t2 = 40/self.dt
        t3 = 80/self.dt
        t4 = 120/self.dt


        # schedule of rotations
        tbl = np.array([[t0, t1, t2, t3, t4],
                        [t0, t1, t2, t3, t4],
                        [t0, t1, t2, t3, t4]])

        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)
        self.mc.gyro_report()
        print '************'
        self.delta = 0.05

    def test_coeff(self):
        self.skipTest('Work in progress')
        is_failure = False

        print self.mc.gyro_model.keys()

        for key in self.mc.gyro_model.keys():

            # calc relative error for each dict key
            if self.gyro_model[key] !=0.:
                gyro_res = (self.mc.gyro_model[key] - self.gyro_model[key])/self.gyro_model[key]
            if self.acc_model[key] !=0.:
                acc_res = (self.mc.acc_model[key] - self.acc_model[key])/self.acc_model[key]


            # test for gyro coefficients
            try:
                if self.gyro_model[key] !=0.:
                    self.assertLess(np.abs(gyro_res), self.delta)
            except AssertionError:
                print  'gyro[' + key + '] something wrong!'

                is_failure = True

            # test for acc coefficients
            try:
                if self.acc_model[key] !=0.:
                    self.assertLess(np.abs(acc_res), self.delta)
            except AssertionError:
                print  'acc[' + key + '] something wrong!'
                is_failure = True

        # fail if any AssertionError occurred
        if is_failure:
            self.fail()



if __name__ == '__main__':
    unittest.main()