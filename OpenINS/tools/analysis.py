#!/usr/bin/env python

# coding=utf8

"""
Identification and analysis module
---------------------

Set of functions for identifieng stochastic model of
inertial sensors, for instance: allan variance, psd and maybe
arma models
"""

import numpy as np
import pylab


def avar(data, dt=0.05, ppd=10, plot=1):
    """
    Compute the overlaped Allan variance for sampled data.

    Parameters
    ----------
    data: array
        observed data from inertial sensor
    dt: value
        discrete-time observation interval
    ppd: int
        amount of points for avar plot

    Returns
    -------
    sigma^2: array
        computed Allan deviation(sqrt(Allan variance))
    logtime: array
        total time interval in sec

    Reference:
    ---------
    [1] IEEE Std 647-1995, IEEE Standard Specification
        Format Guide and Test Procedure for Single-Axis. Laser Gyros
    [2] http://instk.org/blog/?p=64
    [3] http://instk.org/
    [4] C.C.M. NARANJO, “Analysis and Modeling of MEMS based
        Inertial Sensors” (n.d.).
    [5] http://tf.nist.gov/timefreq/general/pdf/2220.pdf
    """

    ## Total number of samples
    N = np.shape(data)[0]

    ## max cluster size
    kmax = np.round(N/3, 0)

    if ppd==0:
        ## time for plot
        steps = np.arange(0,(kmax-1))
    else:
        a = np.arange(0, np.log2(kmax), np.log2(kmax) / ppd)
        steps = np.round(2 ** (a))

    sigma = np.zeros(np.shape(steps)[0])
    idata = np.cumsum(data)
    i = 0

    for k in steps:

        idata_a = idata[2*k:N-1]
        idata_b = idata[k:N-k-1]
        idata_c = idata[:N-2*k-1]

        sigma[i] = np.sqrt(np.mean((idata_c - 2 * idata_b + idata_a) \
                   ** 2)) / (np.sqrt(2) * k)
        i = i + 1

    if plot == 1:
        plot_avar(sigma, steps*dt)

    return sigma, steps*dt


def plot_avar(sigma, time):
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

    pylab.loglog(time, sigma, '-o')
    pylab.xlabel('$time (s)$')
    pylab.ylabel('$\sigma(\\tau)$')
    pylab.title('Allan deviation')
    pylab.grid(True)
    pylab.show()


def psd(data, dt=0.005, NFFT=512, plot=1):
    """
    Compute power spectrum destinity for input time array.

    This function just wraper for pylab.psd implementation

    Paramters:
    ----------
    data: array
        input time series
    dt: float
        Sample time (1/fs). It is used to calculate the Fourier frequencies,
        freqs, in cycles per time unit. The default value is 0.005
    NFFT: The number of data points used in each block for the FFT.
        Must be even; a power 2 is most efficient. The default value is 256.

    Returns:
    -------
    Pxx: array
        power spectral destinity
    freqs: array
        coresponding frequencies

    References:
    ----------
    [1] http://matplotlib.sourceforge.net/api/pyplot_api.htmld
    [2] IEEE Std 647-1995, IEEE Standard Specification Format Guide and Test
        Procedure for Single-Axis. Laser Gyros
    """

    (Pxx, freqs) = pylab.psd(data, NFFT, dt)

    ## Total number of samples
    N = np.shape(data)[0]
    time = np.arange(0,(N))

    pylab.subplot(211)
    pylab.plot(time,data)
    pylab.subplot(212)

    if plot == 1:
        pylab.show()
    return Pxx, freqs

if __name__ == '__main__':
    str1 = '/home/nickolai/Arsenal/applanix/IMU_Data.txt'
#    data = np.loadtxt(str1,float).T
#    a,b = avar(data[6,:]/np.mean(data[6,:]),dt = 0.005,ppd=19)
#    psd(data, dt = 0.005, NFFT=256, plot=1)
