from root import *
from pidform import *
from mpcform import *

if __name__ == '__main__':
    # simulation time
    sim_time = 5.0;

    # initial position w/ disturbance
    # limits = (
    #     10, 1, 10, np.pi/2, np.pi,
    #     0, 0, 0, 0, 0,
    #     0, 0, 0, 0, 0);

    # disturbance sets for final presentation
    # disturbList = (0,);
    # disturbList = (1,);
    # disturbList = (2,);
    # disturbList = (2,3,4);
    disturbList = (0,2,3,4);
    # disturbList = (0,1,2,3,4);

    disturbance = [[eps*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # run pid/mpc simulation
    tPID, xPID, uPID = pidSimulation(sim_time, x0);
    tMPC, xMPC, uMPC = mpcSimulation(sim_time, x0, output=0);

    # plot comparisons
    fig, axsList = plotTrajectories(tPID, xPID,
        xRef=xd, limits=limits,
        legend='PID');
    fig, axsList = plotTrajectories(tMPC, xMPC,
        limits=limits,
        fig=fig, axsList=axsList, legend='MPC');
    plt.show(block=0);

    # create time series for animation
    dtsim = 0.05;
    tSim = [ [i*dtsim for i in range( round(sim_time/dtsim)+1 )] ];
    stepPID = round( dtsim/dtpid );
    stepMPC = round( dtsim/dtmpc );

    # animate comparisons
    StatesPID = StatePlots(tSim, xPID[:,0], xd,
        label='PID', limits=limits,
        color='royalblue', zorder=10);
    StatesMPC = StatePlots(tSim, xMPC[:,0], xd,
        label='MPC', limits=limits,
        fig=StatesPID.fig, axsMat=StatesPID.axsMat,
        color='orange', linestyle='--', zorder=100);

    input('Press ENTER after positioning...');

    # animation loop
    iPID = stepPID;
    iMPC = stepMPC;
    for t in tSim[0][1:]:
        print(t, iPID*dtpid, iMPC*dtmpc)
        StatesPID.update(t, xPID[:,iPID]);
        StatesMPC.update(t, xMPC[:,iMPC]);
        iPID += stepPID;
        iMPC += stepMPC;

    input('Press ENTER to exit...');