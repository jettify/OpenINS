import numpy as np

from openins.trajectory.navtrajectory_opt import NavTrajectoryOpt
from openins.visualisation.plotter import plot_state_vector

# init profile generator
pf = NavTrajectoryOpt()
# set sample time
dt = 0.1
# define time vector
time = np.arange(0., 1000.,  dt)

# generate state vector for each time
ref_state = np.array([pf.state_extended(t) for t in time])

# plot each var versus time
lgnd = ['phi', 'lam', 'h', 'vn', 've', 'vd', 'an', 'ae', 'ad',
'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz', 'ax', 'ay', 'az']
plot_state_vector(time, ref_state, lgnd)

