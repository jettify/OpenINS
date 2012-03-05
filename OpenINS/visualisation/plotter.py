"""
Plotting functions for different purposes.
"""

import pylab

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
