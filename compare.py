from root import *
from pidform import *
from mpcform import *

if __name__ == '__main__':
    # simulation time
    sim_time = 5.0;

    # initial position w/ disturbance
    eList = (
        10, 1, 10, np.pi/2, np.pi,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0);

    # disturbance sets for final presentation
    # disturbList = (2,);  plotList = (0,);
    disturbList = (2,);
    # disturbList = (0,);  plotList = (0,2);
    # disturbList = (1,);

    disturbance = [[eps*eList[i]*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # run pid/mpc simulation
    tpid, xpid, upid = pidSimulation(sim_time, x0);
    tmpc, xmpc, umpc = mpcSimulation(sim_time, x0, output=0);

    # plot comparisons
    fig, axsList = plotTrajectories(tpid, xpid,
        xRef=xd, eList=eList,
        legend='PID');
    fig, axsList = plotTrajectories(tmpc, xmpc,
        eList=eList,
        fig=fig, axsList=axsList, legend='MPC');
    plt.show();

    animateComparisons(tpid, xpid, xmpc);
