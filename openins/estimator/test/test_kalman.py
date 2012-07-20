from __future__ import division
import numpy as np
from openins.estimator.kalman import  KalmanFilter
from numpy import testing

import logging

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)




t, dt = np.linspace(0, 10, 1000, retstep=True)
arr_size = len(t)


state = np.array([0, 0])
state_trans_matrix = np.array([[1,dt],[0,1]])

control_matrix = np.array([[dt**2/2.],[dt]])

measurment_matrix = np.array([[1,0]])
posteriory_cov = 0.2**2 * np.array([[dt**4/4,dt**3/2],[dt**3/2, dt**2]])

process_cov = 0.2**2 * np.array([[dt**4/4,dt**3/2],[dt**3/2, dt**2]])
measurment_cov = np.array([10])

controlFunction =  lambda t: np.ones_like(t)





def checkKalmanFilter( state, state_trans_matrix, measurment_matrix,
                       posteriory_cov, process_cov, measurment_cov, control_matrix):

    logging.info('checkKalmanFilter start')


    filter = KalmanFilter(state, state_trans_matrix, measurment_matrix,
        posteriory_cov, process_cov, measurment_cov, control_matrix)


    measurements = measurment_matrix
    controls = control_matrix



    testing.assert_equal(filter.state_trans_matrix, state_trans_matrix)
    testing.assert_equal(filter.control_matrix, control_matrix)
    testing.assert_equal(filter.measurment_matrix, measurment_matrix)
    testing.assert_equal(filter.measurment_cov, measurment_cov)
    testing.assert_equal(filter.process_cov, process_cov)

    logging.info('checkKalmanFilter end')

    signal = np.empty((len(state), arr_size))

    est = np.empty_like(signal)
    meas = np.zeros((measurment_matrix.shape[0], arr_size))

    meas = np.zeros((1, arr_size))

    controls = control_matrix.shape[1]
    con =  controlFunction(t).reshape(controls, arr_size)


    signal[:,1] = state

    process_dev = np.sqrt(process_cov)
    measurement_dev = np.sqrt(measurment_cov)

    for i in range(1,arr_size):


        signal[:,i] = np.dot(state_trans_matrix, signal[:,i-1]) + \
            np.dot(control_matrix, con[:,i]) +  \
            0*np.dot(process_dev, np.random.normal((len(state), 1)) )
        print np.dot(process_dev, np.random.normal((len(state), 1)) )
        meas[:, i] = np.dot(measurment_matrix, signal[:,i]) + \
            np.dot(measurement_dev, np.random.normal(size=1))





        filter.predict(con[:,i])
        filter.correct(meas[:,i])
        est[:,i] = filter.state

#    assert np.all(np.var(est-signal, axis=1) < np.var(meas-signal, axis=1))
    return signal.T, meas.T, est.T



def testKalmanFilter():
    np.random.seed(0)
    return (checkKalmanFilter(state, state_trans_matrix, measurment_matrix,
        posteriory_cov, process_cov, measurment_cov, control_matrix))

def drawFigures():
    import pylab
    np.random.seed(0)

    pylab.figure(1)
    s,m,e = checkKalmanFilter(state, state_trans_matrix, measurment_matrix,
        posteriory_cov, process_cov, measurment_cov, control_matrix)

    pylab.plot(t, s, label='signal')
    pylab.plot(t, m, label='meas')
    pylab.plot(t, e, label='est')
    pylab.legend()
    pylab.show()

    pylab.figure(2)
    pylab.plot(t, e[:,0]-s[:,0], label='est')
    pylab.legend()
    pylab.show()


if __name__ == '__main__':

    drawFigures()
