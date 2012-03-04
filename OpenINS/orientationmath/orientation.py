"""
Functions for attitude representation and orientation translation.
"""

import numpy as np
from numpy import linalg as la
from numpy import cos, sin, arctan, arctan2, arcsin,pi


def euler2dcm(gamma, theta , psi):
    """
    Calculate direction cosine matrix from Euler angels

    Parameters
    ----------
    gamma: float, rad, roll
    theta: float, rad, pitch
    psi: float, rad, yaw

    Returns
    -------
    DCMbn: 3x3 numpy array DCM from body to navigation ref system

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """

    # rotation psi about z axis
    C1 = np.array([[cos(psi), sin(psi), 0],
        [-sin(psi), cos(psi), 0],
        [0, 0 , 1]])

    # rotation theta about y axis
    C2 = np.array([[cos(theta), 0, -sin(theta)] ,
        [0, 1, 0          ],
        [sin(theta), 0, cos(theta)]])

    # rotation gamma about x axis
    C3 = np.array([[1, 0, 0         ],
        [0, cos(gamma), sin(gamma)],
        [0, -sin(gamma), cos(gamma)]])

    # calculate body to navigation DCM
    dcm_bn = np.dot(np.dot(C1.T , C2.T) , C3.T)

    return dcm_bn

def dcm2euler(dcm_bn):
    """
    Calculate Euler angles from dcm_bn (direction cosine matrix, from body to navigation)

    Parameters
    ----------
    dcm_bn: 3x3 numpy array DCM from body to navigation ref system

    Returns
    -------
    gamma: float, rad, roll
    theta: float, rad, pitch
    psi: float, rad, yaw

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """

    if dcm_bn.shape != (3,3):
        raise TypeError('command must be instance of SensorError')

    gamma = dcm2roll(dcm_bn)
    theta = dcm2pitch(dcm_bn)
    psi = dcm2yaw(dcm_bn)

    return gamma, theta, psi

def dcm2pitch(dcm_bn):
    """
    Find pitch angle form DCM

    Parameters
    ----------
    DCMbn: 3x3 numpy array DCM from body to navigation ref system

    Returns
    -------
    theta: float, rad, pitch

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """
    # check input values

    if dcm_bn.shape != (3,3):
        raise ValueError('Input value must be numpy matrix of 3x3 size')



    theta = arcsin(-dcm_bn[2, 0])
    return theta

def dcm2roll(dcm_bn):
    """
    Find roll angle form DCMbn

    Parameters
    ----------
    DCMbn: 3x3 numpy array DCM from body to navigation ref system

    Returns
    -------
    gamma: roll, rad

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """
    # check input values

    if dcm_bn.shape != (3,3):
        raise ValueError('Input value must be numpy matrix of 3x3 size')



    # TODO: check for theta be pi/2 or -pi/2
    theta = arcsin(-dcm_bn[2, 1])
    if (np.abs((pi / 2) - theta) < 0.000001) or (np.abs((pi / 2) + theta) < 0.000001):
        print 'Warning pitch near 90 deg'



    gamma = arctan(dcm_bn[2, 1] / dcm_bn[2, 2])
    return gamma

def dcm2yaw( dcm_bn):
    """
    Calculate Euler angles from DCMbn (direction cosine matrix, from body to navigation)

    Parameters
    ----------
    DCMbn: 3x3 numpy array DCM from body to navigation ref system

    Returns
    -------
    psi: yaw, rad

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """
    # check input values

    if dcm_bn.shape != (3,3):
        raise ValueError('Input value must be numpy matrix of 3x3 size')

    theta = arcsin(-dcm_bn[2, 1])
    if (np.abs((pi / 2) - theta) < 0.000001) or (np.abs((pi / 2) + theta) < 0.000001):
        print 'Unable determine psi and gamma, cos pitch near pi/2 or -pi/2, \
        see REFERENCE: '

    psi = arctan2(dcm_bn[1, 0] , dcm_bn[0, 0])
    return psi

def dcm2quat(dcm_bn):
    """
    Find quaternion representation from DCM

    Parameters
    ----------
    DCMbn: 3x3 direction cosine matrix (array)

    Returns
    -------
    q0: real part of quaternion
    q1: i part of quaternion
    q2: j part of quaternion
    q3: k part of quaternion

    References
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """
    if dcm_bn.shape != (3,3):
        raise ValueError('Input value must be numpy matrix of 3x3 size')

    q0 = 0.5 * np.power((1 + dcm_bn[0, 0] + dcm_bn[1, 1] + dcm_bn[2, 2]), 0.5)
    q1 = (1 / (4 * q0)) * (dcm_bn[2, 1] - dcm_bn[1, 2])
    q2 = (1 / (4 * q0)) * (dcm_bn[0, 2] - dcm_bn[2, 0])
    q3 = (1 / (4 * q0)) * (dcm_bn[1, 0] - dcm_bn[0, 1])

    return q0, q1, q2, q3


def euler2quat(gamma, theta, psi):
    """
    Find quaternion from euler angles

    Parameters
    ----------
    gamma: roll, radian
    theta: pitch, radian
    psi: yaw, radian

    Returns
    -------
    q0: real part of quaternion
    q1: i part of quaternion
    q2: j part of quaternion
    q3: k part of quaternion

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """

    q0 = cos(gamma / 2) * cos(theta / 2) * cos(psi / 2) + \
         sin(gamma / 2) * sin(theta / 2) * sin(psi / 2)
    q1 = sin(gamma / 2) * cos(theta / 2) * cos(psi / 2) - \
         cos(gamma / 2) * sin(theta / 2) * sin(psi / 2)
    q2 = cos(gamma / 2) * sin(theta / 2) * cos(psi / 2) + \
         sin(gamma / 2) * cos(theta / 2) * sin(psi / 2)
    q3 = cos(gamma / 2) * cos(theta / 2) * sin(psi / 2) - \
         sin(gamma / 2) * sin(theta / 2) * cos(psi / 2)

    #return np.array([q0, q1, q2, q3])
    return q0, q1, q2, q3

def quat2dcm(q):
    """
    Find DCM from quaternion representation

    Parameters
    ----------
    q0: real part of orientation quaternion
    q1: i part of orientation quaternion
    q2: j part of orientation quaternion
    q3: k part of orientation quaternion

    Returns
    -------
    DCMbn: 3x3 numpy array, DCM from body to navigation ref sys

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """


    dcm_bn = np.ones((3,3))

    dcm_bn[0, 0] = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    dcm_bn[0, 1] = 2 * (q[1] * q[2] - q[0] * q[3])
    dcm_bn[0, 2] = 2 * (q[1] * q[3] + q[0] * q[2])

    dcm_bn[1, 0] = 2 * (q[1] * q[2] + q[0] * q[3])
    dcm_bn[1, 1] = q[0] **2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    dcm_bn[1, 2] = 2 * (q[2] * q[3] - q[0] * q[1])

    dcm_bn[2, 0] = 2 * (q[1] * q[3] - q[0] * q[2])
    dcm_bn[2, 1] = 2 * (q[2] * q[3] + q[0] * q[1])
    dcm_bn[2, 2] = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2

    return dcm_bn

def skew(v):
    """
    Create skew symmetric matrix form 1x3 vector

    Parameters
    ----------
    v: [1x3], input vector

    Returns
    -------
    skew_sym: [3x3] skew symetric matrix based on

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """
    skew_sym = np.array([[0,   -v[2], v[1]],
                         [v[2], 0,   -v[0]],
                         [-v[1],v[0], 0]])

    return skew_sym


def quat_mult(q1,q2):
    """
    Quaternion multiplication.

    Parameters
    ----------
    quat1: quaternion1[1x4], vector
    quat2: quaternion2[1x4], vector

    Returns
    -------
    quat_new: quat1*qut2, [1x4] vector

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """
    A = np.array([[q1[0], -q1[1], -q1[2], -q1[3]],
                  [q1[1],  q1[0], -q1[3], q1[2]],
                  [q1[2],  q1[3],  q1[0], -q1[1]],
                  [q1[3], -q1[2],  q1[1], q1[0]]])

    return np.dot(A,q2)


def dcm_prop(dcm_nb_old,angular_vel):
    """
    Update the direction cosine matrix for body motion (relative to inertial space).
    The function is thus acting upon the strapdown gyro outputs.

    Parameters
    ----------
    dcm_nb_old: direction cosine matrix t(k)
    angular_vel: vector [1x3] gyro output, t(k)

    Returns
    -------
    DCMbn_new: direction cosine matrix at state t(k+1)

    References
    ----------
    Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """
    skw = skew(angular_vel)
    magnitude = la.norm(angular_vel)

    if magnitude == 0.:
        B = np.eye(3)
    else:
        B = np.eye(3) + (sin(magnitude) / magnitude) * skw +\
            ((1 - cos(magnitude)) / ((magnitude ** 2))) * np.dot(skw,skw)

    dcm_nb_new = np.dot(dcm_nb_old,B)

    return dcm_nb_new

def quat_prop_1o(q,angular_vel):
    """
    Update the quaternion for body motion (relative to inertial space).
    The function is thus acting upon the strapdown gyro outputs.
    This is FIRST order integrator.

    Parameters
    ----------
    quat1: [1x4] array
        initial orientation quaternion
    angular_vel: 1x3 array
        gyro output, t(k)

    Returns
    -------
    quat_new: 1x4 array
        new orientation quaternion

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """

    magnitude = la.norm(angular_vel)
    if magnitude == 0.:
        r = np.array([1.0,0,0,0])
    else:
        a_c = cos(magnitude/2)
        a_s = sin(magnitude/2)/magnitude
        r = np.array([a_c,
                      a_s*angular_vel[0],
                      a_s*angular_vel[1],
                      a_s*angular_vel[2]])

    return quat_mult(q,r)

def quat_prop_4o(q,ang_vel):
    """
    Update the quaternion for body motion (relative to inertial space).
    The function is thus acting upon the strapdown gyro outputs.
    This is FOURTH order integrator.

    Parameters
    ----------
    quat1: [1x4] array
        initial orientation quaternion, q[0] real part of quaternion
    ang_vel: 1x3 array
        gyro output (incermental anlge)

    Returns
    -------
    quat_new: 1x4 array
        new orientation quaternion

    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    [2] http://en.wikipedia.org/wiki/Quaternion
    """

    magn = la.norm(ang_vel)
    if magn == 0.:
        r = np.array([1.0,0,0,0])
    else:
        q0 = 1. - (1./8.)*magn**2 + (1. / 384.) * magn ** 4 - \
             (1./460080.)*(magn**2)**3
        q1 = 0.5 * (1 - (1. / 24.) * magn ** 2 + (1. / 1920.) *
                    magn**4 - (1./322560.) * (magn**2) ** 3) * ang_vel[0]
        q2 = 0.5 * (1 - (1. / 24.) * magn ** 2 + (1. / 1920.) *
                    magn**4 - (1./322560.) * (magn**2) **3 ) * ang_vel[1]
        q3 = 0.5 * (1 - (1. / 24.) * magn ** 2 + (1. / 1920.) *
                    magn**4 - (1./322560.) * (magn**2) **3 ) * ang_vel[2]
        r = np.array([q0, q1, q2, q3])

    return quat_mult(q,r)


if __name__ == '__main__':
    print euler2dcm(0.,0.,0.)
    print '***********************'
    print np.round(euler2dcm(-np.pi/2.,-np.pi/2.,0.))
    print '***********************'
    print np.round(euler2dcm(np.pi/2.,0.,np.pi/2.))
    print '***********************'




