import unittest
import numpy as np

from openins.trajectory.calibtrajectory import RotationSequence
from openins.trajectory.calibtrajectory import BasicCalibTraj
from openins.trajectory.calibtrajectory import RotateIMU
from openins.visualisation.plotter import plot_trinity
from openins.tools.calibration import DieselCalibrator


class TestDieselCalibrator(unittest.TestCase):

    def setUp(self):

        self.turn_time = 5
        self.stady_time = 20
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

#        self.rs1.add(RotateIMU(self.turn_time, 0., 0., np.pi))
#        self.rs1.add(RotateIMU(15.*self.stady_time, 0., 0., 0.))
#        self.rs1.add(RotateIMU(self.turn_time, 0., 0., np.pi))
#        self.rs1.add(RotateIMU(15.*self.stady_time, 0., 0., 0.))


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

#        self.rs2.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
#        self.rs2.add(RotateIMU(15.*self.stady_time, 0., 0., 0.))
#        self.rs2.add(RotateIMU(self.turn_time, np.pi, 0., 0.))
#        self.rs2.add(RotateIMU(15*self.stady_time, 0., 0., 0.))



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

#        self.rs3.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
#        self.rs3.add(RotateIMU(15*self.stady_time, 0., 0., 0.))
#        self.rs3.add(RotateIMU(self.turn_time, 0., np.pi, 0.))
#        self.rs3.add(RotateIMU(15*self.stady_time, 0., 0., 0.))

        self.fixture1 = BasicCalibTraj(self.rs1)
        self.fixture2 = BasicCalibTraj(self.rs2)
        self.fixture3 = BasicCalibTraj(self.rs3)


        self.time = np.arange(0., 150., self.dt)

        to_rads = (np.pi/180.)/3600.
        shape = len(self.time)
        # 0.000001454
        gyro_rand1 = (0.15e-05)*np.random.randn(shape,3)
        gyro_rand2 = (0.15e-05)*np.random.randn(shape,3)
        gyro_rand3 = (0.15e-05)*np.random.randn(shape,3)

        acc_rand1 = 0.0005*np.random.randn(shape,3)
        acc_rand2 = 0.0005*np.random.randn(shape,3)
        acc_rand3 = 0.0005*np.random.randn(shape,3)


        gyros1 = np.array(map(self.fixture1.gyros, self.time)) + gyro_rand1
        gyros2 = np.array(map(self.fixture2.gyros, self.time)) + gyro_rand2
        gyros3 = np.array(map(self.fixture3.gyros, self.time)) + gyro_rand3

        accs1 = np.array(map(self.fixture1.accs, self.time)) + 0*acc_rand1
        accs2 = np.array(map(self.fixture2.accs, self.time)) + 0*acc_rand2
        accs3 = np.array(map(self.fixture3.accs, self.time)) + 0*acc_rand3


        self.gyro_model = {'x': 3e-07, 'y':5e-07, 'z':7e-07,
                           'xx':0.0001, 'xy':0.0002, 'xz':0.0003,
                           'yx':0.0004, 'yy':0.0005, 'yz':0.0006,
                           'zx':0.0007, 'zy':0.0008, 'zz':0.0009,}


        # acc parameters: bias and scalefactor, misalignment matrix
        self.acc_model = {'x': 0.003, 'y':0.004, 'z':0.005,
                           'xx':0.001, 'xy':0.002, 'xz':0.000,
                           'yx':0.000, 'yy':0.003, 'yz':0.000,
                           'zx':0.004, 'zy':0.005, 'zz':0.006,}

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


        t0 = 20./self.dt -self.dt
        t1 = 0./self.dt
        t2 = (20. + 5.)/self.dt
        t3 = (20. + 5. + 20. + 5.)/self.dt
        t4 = (20. + 5. + 20. + 5. + 20. + 5.)/self.dt


        # schedule of rotations
        # t0: time shift
        tbl = np.array([[t0, t1, t2, t3, t4],
                        [t0, t1, t2, t3, t4],
                        [t0, t1, t2, t3, t4]])



#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)

#        self.mc.gyro_report()
        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)

#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)
#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)
#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)
#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)
#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)
#        dv1, dv2, dv3 = self.mc.calc_coefficient(tbl)

        plot_trinity(self.time, dv1.T)
        plot_trinity(self.time, dv2.T)
        plot_trinity(self.time, dv3.T)

        self.mc.gyro_report()
        self.delta = 0.05

    def test_coeff(self):
#        self.skipTest('Work in progress')
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
                print  'gyro[' + key + ']' ,
                print  'test= ', self.mc.gyro_model[key], 'ref= ',self.gyro_model[key],

                print ' delta = ', self.delta,
                print ' tol =',  gyro_res

                is_failure = True

            # test for acc coefficients
            try:
                if self.acc_model[key] !=0.:
                    self.assertLess(np.abs(acc_res), self.delta)
            except AssertionError:
                print  'acc[' + key + ']' ,
                print  'test= ', self.mc.acc_model[key], 'ref= ',self.acc_model[key],

                print ' delta = ', self.delta,
                print ' tol =',  acc_res
                is_failure = True

        # fail if any AssertionError occurred
        if is_failure:
            self.fail()



if __name__ == '__main__':
    unittest.main()