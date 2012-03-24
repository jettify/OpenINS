"""
Plotting functions for different purposes.
"""

import pylab
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

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
    
    pylab.plot(time, data)
    pylab.xlabel('time, s')
    pylab.ylabel('data')
    pylab.title('Triad Plotter')
    if lgnd != None:
        pylab.legend((lgnd[0], lgnd[1], lgnd[2]))
    pylab.grid(True)
    pylab.show()



