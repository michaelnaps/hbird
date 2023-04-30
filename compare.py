from root import *
from pidform import *
from mpcform import *

if __name__ == '__main__':
    # simulation time
    sim_time = 10.0;

    # initial position w/ disturbance
    eps = 0.1;

    # disturbance sets for final presentation
    disturbList = (2,);
    # disturbList = (2,3,4);
    # disturbList = (0,);
    # disturbList = (1,);
    disturbance = [[eps*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # run pid/mpc simulation
    tpid, xpid, upid = pidSimulation(sim_time, x0);
    tmpc, xmpc, umpc = mpcSimulation(sim_time, x0, output=0);

    # plot comparisons
    fig, axsList = plotTrajectories(tpid, xpid, xd,
        legend='PID');
    fig, axsList = plotTrajectories(tmpc, xmpc, xd,
        legend='MPC', fig=fig, axsList=axsList);
    plt.show();
