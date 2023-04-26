from root import *
from pidform import *
from mpcform import *

if __name__ == '__main__':
    # simulation time
    sim_time = 1.0;

    # initial position w/ disturbance
    eps = 1.0;
    disturbList = (2,);
    disturbance = [[noise(eps)*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # run pid simulation
    tpid, xpid, upid = pidSimulation(sim_time, x0);

    # run mpc simulation
    tmpc, xmpc, umpc = mpcSimulation(sim_time, x0, output=1);

    # plot comparisons
    fig, axsList = plotTrajectories(tpid, xpid, xd,
        legend='PID');
    fig, axsList = plotTrajectories(tmpc, xmpc, xd,
        fig=fig, axsList=axsList, legend='MPC');
    plt.show();
