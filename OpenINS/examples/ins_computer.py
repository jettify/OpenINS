import numpy as np

from ns.inscomputer import SimpleINSComputer
from visualisation.plotter import plot_compare_states
from visualisation.plotter import plot_compare_states_diff
from trajectory.navtrajectory_opt import NavTrajectoryOpt

# init trajectory of body
pf = NavTrajectoryOpt()
# set sample time and time vector
dt = 0.005
time = np.arange(0., 100.,  dt)

# init INS Computer
ins = SimpleINSComputer()
ins.dt = dt

# set initial state of INS
init_state = pf.init_state()
ins.set_state(init_state)

# save ins state
ins_state = np.array([ins(pf.gyros(t), pf.accs(t))
                       for t in time])
# save trajectory
ref_state = np.array([pf.state(t) for t in time])

# plot each variable versus time
lgnd = ['phi', 'lam', 'h', 'vn', 've', 'vd', 'an', 'ae', 'ad',
        'q0', 'q1', 'q2', 'q3']

plot_compare_states(time, ins_state, ref_state, lgnd)
plot_compare_states_diff(time, ins_state, ref_state, lgnd)
