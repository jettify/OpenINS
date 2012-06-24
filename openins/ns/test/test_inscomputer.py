"""
Test for inscomputer module
"""
import numpy as np
import unittest
import pylab



from openins.ns.inscomputer import SimpleINSComputer
from openins.trajectory.navtrajectory import NavTrajectory
from openins.visualisation.plotter import plot_trinity, plot_trajectory, plot_basic
from openins.visualisation.plotter import plot_ins_state, plot_compare_states, plot_compare_states_diff, plot_compare_quat2euler
from openins.trajectory.navtrajectory_opt import navtrajectory_state, navtrajectory_statex
from openins.trajectory.navtrajectory_opt import NavTrajectoryOpt

from openins.orientationmath.orientation import euler2quat
from openins.orientationmath.orientation import dcm2quat
from openins.orientationmath.orientation import quat2dcm
from openins.orientationmath.orientation import dcm2euler


class ComprehensiveTest(unittest.TestCase):
    """
    Test each method in SimpleINSComputer.
    """

    def setUp(self):
        """
        Make sure that every test runs on clean trajectory.
        """

        self.pf = NavTrajectoryOpt()
#        self.pf = NavTrajectory(pd)
        self.dt = 0.01
        self.ins = SimpleINSComputer()
        self.ins.dt = self.dt

        init_state = self.pf.init_state()
        self.ins.set_state(init_state)
        self.time = np.arange(self.dt, 100.,  self.dt)



    def test_position(self):
        """
        Test position integration pof inscomputer.

        We just integrate position, velocity in INS state replaced by ideal 
        one. Other variables in state vector is not changed coz they do not 
        influence on position integration.
        """

        self.skipTest('temp')
        test_pos = np.empty
        for t in self.time:
            rez = self.ins._integrate_position()

            self.ins._state[0:3] = rez
            self.ins._state[3:6] = self.pf.state(t)[3:6]

            try:
                test_pos = np.vstack((test_pos, rez))
            except ValueError:
                test_pos = rez


        # get ideal values for: latitude, longitude and height
        tru_pos = np.array([self.pf.position(t) for t in self.time])

        lgnd = ['phi', 'lam', 'h']
        plot_compare_states(self.time, test_pos, tru_pos, lgnd)
        plot_compare_states_diff(self.time, 6378137.0*test_pos, 6378137.0*tru_pos, lgnd)

        np.testing.assert_allclose(test_pos, tru_pos, rtol=1e-4, atol=1e-2 )


    def test_velocity_acc(self):
        """
        Test velocity integration of inscomputer.

        Here we integrate only velocity and check acceleration so, position
        and orientation replaced by ideal values.
        """

        self.skipTest('temp')
        vel = np.empty
        acc = np.empty

        for t in self.time:

            if t != self.time[0]:
                # replace INS variables with ideal one
                q = self.pf.orientation_q(t)
                self.ins._state[9:] = q

                rez, aned = self.ins._integrate_velocity(self.pf.accs(t))
                self.ins._state[3:6] = rez
                self.ins._state[6:9] = aned
                self.ins._state[0:3] = self.pf.position(t)

                # save result in next vectors
                vel = np.vstack((vel, rez))
                acc = np.vstack((acc, aned))

            else:
                vel = self.ins._state[3:6]
                acc = self.ins._state[6:9]


        tru_v = np.array([self.pf.velocity_ned(t) for t in self.time])
        tru_acc = np.array([self.pf.acceleration_ned(t) for t in self.time])

        lgnd = ['vn', 've', 'vd']
        plot_compare_states(self.time, vel, tru_v, lgnd)
        plot_compare_states_diff(self.time, vel, tru_v, lgnd)

        lgnd = ['an', 'ae', 'ad']
        plot_compare_states(self.time, acc, tru_acc, lgnd)
        plot_compare_states_diff(self.time, acc, tru_acc, lgnd)

        np.testing.assert_allclose(vel, tru_v, rtol=0.001, atol=0.01 )
        np.testing.assert_allclose(acc, tru_acc, rtol=1e-4, atol=0.01)



    def test_attitude(self):
        """
        Test position integration of inscomputer.
        """
#        self.skipTest('temp')
        q = self.ins._state[9:]
        test_quat = self.ins._state[9:]
        for t in self.time:

            if t != 0.:


                self.ins._state[0:3] = self.pf.position(t)
                self.ins._state[3:6] = self.pf.velocity_ned(t)
#                q = self.ins._integrate_orientation(self.pf.gyros_inc_angle(t, self.dt))
                q = self.ins._integrate_orientation(self.pf.gyros(t))
                self.ins._state[9:] = q
                test_quat = np.vstack((test_quat, q))

            else:
                q = self.ins._state[9:]


#        pitch = np.rad2deg(np.array([self.pf.theta(t) for t in self.time]))
#        roll = np.rad2deg(np.array([self.pf.gamma(t) for t in self.time]))
#        yaw = np.rad2deg(np.array([self.pf.psi(t) for t in self.time]))

        ref_quat = np.array([self.pf.orientation_q(t) for t in self.time])

        lgnd = ['q0', 'q1', 'q2', 'q3']



        plot_compare_states(self.time, test_quat[:-1], ref_quat, lgnd)
        plot_compare_states_diff(self.time, test_quat[:-1], ref_quat, lgnd)

        lgnd = ['roll', 'pitch', 'yaw']
        plot_compare_quat2euler(self.time, test_quat[:-1], ref_quat, lgnd)

    def test_complex_system(self):
        """
        Test full functional system.
        """
        self.skipTest('temp')
        test_state = np.array([self.ins(self.pf.gyros(t), self.pf.accs(t))
                      for t in self.time])
        ref_state = np.array([self.pf.state(t) for t in self.time])


        lgnd = ['phi', 'lam', 'h', 'vn', 've', 'vd', 'an', 'ae', 'ad',
                'q0', 'q1', 'q2', 'q3']
        plot_compare_states(self.time, test_state, ref_state, lgnd)
        plot_compare_states_diff(self.time, test_state, ref_state, lgnd)


    def test_velocity_pos(self):
        """
        Test velocity integration of inscomputer.

        Here we integrate only velocity and check acceleration so, position
        and orientation replaced by ideal values.
        """
        self.skipTest('temp')
        vel = np.empty
        acc = np.empty
        pos = np.empty
        quat = np.empty
        quati = np.empty
        test_state = np.empty

        for t in self.time:

#            print 't = ', t
            if t != self.time[0]:
                # replace INS variables with ideal one
#                q = dcm2quat(self.pf.dcmbn(t))
#                self.ns._state[9:] = q

                q = self.ins._integrate_orientation(self.pf.gyros(t))
                qi = self.pf.orientation_q(t)

                self.ins._state[9:] = qi
                rez, aned = self.ins._integrate_velocity(self.pf.accs(t))
                self.ins._state[3:6] = rez
                self.ins._state[6:9] = aned
                self.ins._state[0:3] = self.ins._integrate_position()
#                self.ns._state[9:] = q

                # save result in next vectors
                vel = np.vstack((vel, self.ins._state[3:6]))
                acc = np.vstack((acc, aned))
                pos = np.vstack((pos, self.ins._state[0:3]))
                quat = np.vstack((quat, q))
                quati = np.vstack((quati, qi))
                test_state =  np.vstack((test_state, self.ins._state[:]))

            else:
                vel = self.ins._state[3:6]
                acc = self.ins._state[6:9]
                pos = self.ins._state[0:3]
                quat = self.ins._state[9:]
                quati = self.ins._state[9:]
                test_state =  self.ins._state[:]


        lgnd = ['phi', 'lam', 'h', 'vn', 've', 'vd', 'an', 'ae', 'ad',
                'q0', 'q1', 'q2', 'q3']

        ref_state = np.array([self.pf.state(t) for t in self.time])
        print np.shape(ref_state[1:,:])
        print np.shape(test_state[1:,:])

        plot_compare_states(self.time[1:], test_state[1:,:], ref_state[1:,:], lgnd)
        plot_compare_states_diff(self.time[1:], test_state[1:,:], ref_state[1:,:], lgnd)


#        np.testing.assert_almost_equal(vel[1:,0], tru_vn[1:], decimal=4)
#        np.testing.assert_almost_equal(vel[1:,1], tru_ve[1:], decimal=4)
#        np.testing.assert_almost_equal(vel[1:,2], tru_vd[1:], decimal=4)
#
#        np.testing.assert_almost_equal(acc[1:,0], tru_an[1:], decimal=4)
#        np.testing.assert_almost_equal(acc[1:,1], tru_ae[1:], decimal=4)
#        np.testing.assert_almost_equal(acc[1:,2], tru_ad[1:], decimal=4)

if __name__ == '__main__':
    unittest.main()

