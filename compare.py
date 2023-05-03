from root import *
from pidform import *
from mpcform import *

if __name__ == '__main__':
    # simulation time
    sim_time = 1;

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
    tPID, xPID, uPID = pidSimulation(sim_time, x0);
    tMPC, xMPC, uMPC = mpcSimulation(sim_time, x0, output=0);

    # plot comparisons
    fig, axsList = plotTrajectories(tPID, xPID,
        xRef=xd, eList=eList,
        legend='PID');
    fig, axsList = plotTrajectories(tMPC, xMPC,
        eList=eList,
        fig=fig, axsList=axsList, legend='MPC');
    plt.show(block=0);

    # create time series for animation
    dtsim = 0.1;
    tSim = [ [i*dtsim for i in range( round(sim_time/dtsim)+1 )] ];
    stepPID = round( dtsim/dtpid );
    stepMPC = round( dtsim/dtmpc );

    # animate comparisons
    StatesPID = StatePlots(tSim, xPID[:,0], xd, limits=eList,
        color='royalblue', zorder=10);
    StatesMPC = StatePlots(tSim, xMPC[:,0], xd, limits=eList,
        fig=StatesPID.fig, axsMat=StatesPID.axsMat,
        color='orange', linestyle='--', zorder=20);

    # animation loop
    iPID = stepPID;
    iMPC = stepMPC;
    for t in tSim[0][1:]:
        print(t, iPID*dtpid, iMPC*dtmpc)
        StatesPID.update(t, xPID[:,iPID]);
        StatesMPC.update(t, xMPC[:,iMPC]);
        iPID += stepPID;
        iMPC += stepMPC;