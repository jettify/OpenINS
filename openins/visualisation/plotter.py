"""
Plotting functions for different purposes.
"""

import pylab
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time as timelib
import numpy as np
from openins.orientationmath.orientation import quat2dcm
from openins.orientationmath.orientation import dcm2euler

def plot_avar(time, sigma):
    """
    Compute the overlaped Allan variance for sampled data.
    
    Parameters
    ----------
    sigma: array
        computed allan variance
    time: array
        corespondet time for avar
       
    Returns
    -------
    avar_plot: plot
       generate avar_plot
    """
    pylab.figure()
    pylab.loglog(time, sigma,'-o')
    pylab.xlabel('$time (s)$')
    pylab.ylabel('$\sigma(\\tau)$')
    pylab.title('Allan deviation')
    pylab.grid(True)
    pylab.show()

def plot_trajectory(phi, lam, height):
    """
    Plots 3D trajectory
    
    Parameters
    ----------
    phi: array, deg, latitude
    lam: array, deg, longitude
    h: array, deg, height
    """

    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    axis = fig.gca(projection='3d')

    axis.plot(lam, phi, height, label='trajectory')
    axis.legend()
    axis.set_title('Helix like craft movement')
    axis.set_xlabel('lam, longitude, deg')
    axis.set_ylabel('phi, latitude, deg')
    axis.set_zlabel('h, height, m')
    plt.show()

def plot_euler(time, roll, pitch, yaw):
    """
    Plot Euler angles.

    Parameters
    ----------
    roll: array, deg
    yaw: array, deg
    pitch: array, deg
    """
    pylab.figure()
    pylab.subplot(311)
    pylab.plot(time, pitch,'r')
    pylab.xlabel('time (s)')
    pylab.ylabel('$\\theta$, deg')
    pylab.title('Actual Pitch')
    pylab.grid(True)
    
    pylab.subplot(312)
    pylab.plot(time, yaw,'g')
    pylab.xlabel('time (s)')
    pylab.ylabel('$\\psi$,deg')
    pylab.title('Actual Yaw')
    pylab.grid(True)
    
    pylab.subplot(313)
    pylab.plot(time, roll,'b')
    pylab.xlabel('time, s')
    pylab.ylabel('$\\gamma$,deg')
    pylab.title('Actual Roll')
    pylab.grid(True)
    pylab.show()
       
def plot_trinity(time, data, lgnd=None):
    """
    Plot 3 lines in one plot

    Parameters
    ----------
    time: array, sec
    data:
    """
    pylab.figure()
    pylab.plot(time, data)
    pylab.xlabel('time, s')
    pylab.ylabel('data')
    pylab.title('Triad Plotter')
    if lgnd != None:
        pylab.legend((lgnd[0], lgnd[1], lgnd[2]))
    pylab.grid(True)

    pylab.show()


def plot_basic(time, data, lgnd=None):
    """
    Plot 3 lines in one plot

    Parameters
    ----------
    time: array, sec
    data:
    """
    pylab.figure()
    pylab.plot(time, data)
    pylab.xlabel('time, s')
    pylab.ylabel('data')
    pylab.title('Basic Plotter')
    if lgnd != None:
        pylab.legend(lgnd)
    pylab.grid(True)
    pylab.show()

def plot_ins_state(time, state):
    """
    Plot ns state data.

    First plots, trajectory in 3d space, then
    """
    pylab.ion()

    plot_trajectory(state[:,0], state[:,1], state[:,2])


    # Plot position vs. time


    pylab.figure()
    pylab.subplot(311)
    pylab.plot(time, state[:,0],'r')
    pylab.xlabel('time (s)')
    pylab.ylabel('$\\phi$, rad')
    pylab.title('Latitude')
    pylab.grid(True)

    pylab.subplot(312)
    pylab.plot(time, state[:,1],'g')
    pylab.xlabel('time (s)')
    pylab.ylabel('$\\lambda$, rad')
    pylab.title('Longitude')
    pylab.grid(True)

    pylab.subplot(313)
    pylab.plot(time, state[:,2],'b')
    pylab.xlabel('time, s')
    pylab.ylabel('$h$, m')
    pylab.title('Altitude')
    pylab.grid(True)
    pylab.show()


    # Plot velocity vs. time
    pylab.figure()
    pylab.plot(time, state[:,3:6])
    pylab.xlabel('time, s')
    pylab.ylabel('Vn, Ve, Vd')
    pylab.title('Velocity vs. time')

    pylab.grid(True)
    pylab.show()

    # Plot acceleration vs. time
    pylab.figure()
    pylab.plot(time, state[:,6:9])
    pylab.xlabel('time, s')
    pylab.ylabel('an, ae, ad')
    pylab.title('Acceleration vs. time')

    pylab.grid(True)
    pylab.show()
    pylab.ioff()

    # Plot quaternions vs. time
    pylab.figure()
    pylab.plot(time, state[:,9:])
    pylab.xlabel('time, s')
    pylab.ylabel('q0, q1, q2, q3')
    pylab.title('Quaternion vs. time')

    pylab.grid(True)
    pylab.show()
    pylab.ioff()


def plot_state_vector(time, ref_state, lgnd = None):
    """
    Compare two states
    """
    # assert len(ref_state) == len(lgnd)

    row, cols = np.shape(ref_state)
    legend = None
    for i in range(0, cols):
        if lgnd != None:
            legend = [lgnd[i]]
        plot_basic(time, ref_state[:,i], legend )


def plot_compare_states(time, test_state, ref_state, lgnd = None):
    """
    Compare two states
    """
    assert len(test_state) == len(ref_state)

    row, cols = np.shape(test_state)
    legend = None
    for i in range(0, cols):
        if lgnd != None:
            legend = [lgnd[i] + '_test', lgnd[i] + '_ref']
        plot_basic(time, np.array([test_state[:,i], ref_state[:,i]]).T, legend )


#    for i in range(0, cols):
#        if lgnd != None:
#            legend = [lgnd[i]]
#        plot_basic(time, np.array([test_state[:,i] - ref_state[:,i]]).T, legend )


def plot_compare_states_diff(time, test_state, ref_state, lgnd = None):
    """
    Compare two states
    """
    assert len(test_state) == len(ref_state)

    row, cols = np.shape(test_state)
    legend = None
    for i in range(0, cols):
        if lgnd != None:
            legend = [lgnd[i] + '_test', lgnd[i] + '_ref']
        plot_basic(time, np.array([test_state[:,i]-ref_state[:,i]]).T, legend )



def plot_compare_quat2euler(time, test_state, ref_state, lgnd = None):
    """
    Compare two states
    """

    test_state = np.rad2deg(np.array([dcm2euler(quat2dcm(q)) for q in test_state]))
    ref_state =  np.rad2deg(np.array([dcm2euler(quat2dcm(q)) for q in ref_state]))

    assert len(test_state) == len(ref_state)


    row, cols = np.shape(test_state)
    legend = None
    for i in range(0, cols):
        if lgnd != None:
            legend = [lgnd[i] + '_test', lgnd[i] + '_ref']
        plot_basic(time, np.array([test_state[:,i]-ref_state[:,i]]).T, legend )