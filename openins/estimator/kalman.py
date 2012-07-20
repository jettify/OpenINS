"""
Standard and unscented Kalman filters.
"""
from __future__ import division
import numpy as np
from numpy.linalg import inv
import logging

LOG = logging.getLogger(__name__)


class KalmanFilter(object):
    """
    Basic Kalman Filter implementation
    """

    def __init__(self, state, state_trans_matrix, measurment_matrix,
                 posteriory_cov, process_cov, measurment_cov, control_matrix=None):
        """
        Construct Kalman filter.

        Parameters
        ----------
        state: (x) initial state estimation
        state_trans_matrix: (A) state transition model which is applied to the previous state
        measurment_matrix: (H) measurement sensitivity matrix or observation matrix
        posteriory_cov: (P(+)) predicted or a priori value of estimation covariance
        process_cov: (Q) covariance of dynamic disturbance noise
        measurment_cov: (R) covariance of sensor noise or measurement uncertainty.
        control_matrix: (B) control-input model which is applied to the vector u
        """
        self.state = state
        self.state_trans_matrix = state_trans_matrix
        self.control_matrix = control_matrix
        self.measurment_matrix = measurment_matrix
        self.posteriory_cov = posteriory_cov
        self.process_cov = process_cov
        self.measurment_cov = measurment_cov

    @property
    def state(self):
        return self._x

    @state.setter
    def state(self, state):
        self._x = state
        self._I = np.eye(len(self._x))

    @property
    def control_matrix(self):
        return self._B

    @control_matrix.setter
    def control_matrix(self, control_matrix):
        self._B = None if control_matrix is None else control_matrix


    @property
    def state_trans_matrix(self):
        return self._F

    @state_trans_matrix.setter
    def state_trans_matrix(self, f):
        self._F = f


    @property
    def measurment_matrix(self):
        return self._H

    @measurment_matrix.setter
    def measurment_matrix(self, h):
        self._H = h


    @property
    def apriory_cov(self):
        return self._Pm

    @apriory_cov.setter
    def apriory_cov(self, p):
        self._Pm = p


    @property
    def process_cov(self):
        return self._Q

    @process_cov.setter
    def process_cov(self, q):
        self._Q = q

    @property
    def measurment_cov(self):
        return self._R

    @measurment_cov.setter
    def measurment_cov(self, r):
        self._R = r

    @property
    def gain(self):
        return self._K

    @property
    def posteriory_cov(self):
        return self._Pp

    @posteriory_cov.setter
    def posteriory_cov(self, p):
        self._Pp = p

    def predict(self, control=None):
        """
        Predict the system state at the next timestep, updating the estimate.

        Parameters
        ----------
        control:(u) control inputs, or None if there are no control inputs to the system.
        """
        x, F, B, Pp, Q = self._x, self._F, self._B, self._Pp, self._Q,

        assert (control is None) == (B is None),\
        "Control input must be given if and only if control matrix is set."

        if B is None:
            B = 0
            u = 0
        else:
            u = control

        self._x = np.dot(F, x) + np.dot(B, u)
        self._Pm = np.dot(np.dot(F, Pp), F.T) + Q

    def innovation(self, measurement):
        """
        Calculate the filter innovation and covariance for a given measurement.

        Parameters
        ----------
        measurement: (z) array of measurement inputs

        @return: Nx1 innovation vector, NxN covariance matrix of the
            innovation, and MxN cross covariance from states to measurements.
        """
        H, Pm, R, x = self._H, self._Pm, self._R, self._x
        z = measurement

        innovation = z - np.dot(H, x)
        cross_covariance = np.dot(Pm, H.T)
        innovation_covariance = np.dot(H, np.dot(Pm, H.T)) + R

        return innovation, cross_covariance, innovation_covariance

    def correct(self, measurement):
        """
        Correct the filter state given a new measurement.

        Parameters
        ----------
        measurement: (z) array of measurement inputs
        """
#        H, Pm, R, x = self._H, self._Pm, self._R, self._x
#        z = measurement

        innovation = measurement - np.dot(self._H, self._x)
        cross_covariance = np.dot(self._Pm, self._H.T)
        innovation_covariance = np.dot(self._H, cross_covariance) + self._R


        self._K = np.dot(cross_covariance, inv(innovation_covariance))
        self._x = self._x + np.dot(self._K, innovation)
        self._Pp = np.dot(self._I - np.dot(self._K, self._H), self._Pm)


    def update(self, measurement, control=None):
        """
        Run filter update given measurement and control inputs.

        Parameters
        ----------
        measurement: (z) array of measurement inputs
        control: (u) control inputs, or None if there are no control inputs to the system.
        """
        self.predict(control)
        self.correct(measurement)


if __name__ == '__main__':
    pass