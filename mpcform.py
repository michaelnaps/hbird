from root import *
import pidform as pid

kmpc = [
    50, 50, 120, 10, 10,        # position costs
    5, 5, 5, 1, 1,              # velocity costs
    0.1, 0.1, 0.1, 0.1, 0.1     # steady state error costs
];

# cost function
def cost(mvar, xList, uList):
    xd = mvar.params.xd;

    C = 0;  j = 0;
    for i, x in enumerate(xList):
        C += sum([kmpc[i]*(x[i] - xd[i])**2 for i in range(cNx)]);

    return C;

# mpc simulation function
def mpcSimulation(sim_time, x0, output=0):
    # create MPC class variable
    xdmpc = [xd[i][0] for i in range(cNx)];
    modelmpc = lambda x, u, _: dmodel(x,u);
    PH = 5;
    kl = 2;
    max_iter = 200;
    model_type = 'discrete';
    params = Vehicle(np.zeros((cNx,)), xdmpc);
    mvar = mpc.ModelPredictiveControl('ngd', modelmpc, cost, params, Nu,
        num_ssvar=cNx, PH_length=PH, knot_length=kl, time_step=dtmpc,
        max_iter=max_iter, model_type=model_type);
    mvar.setAlpha(1);

    # solve for given time frame
    uinit = [0 for i in range(Nu*PH)];
    x0mpc = [x0[i][0] for i in range(cNx)];
    mpc_results = mvar.sim_root(sim_time, x0mpc, uinit, output=output);

    # formatting for use in plotting function
    tList = [mpc_results[0]];
    xList = np.array( mpc_results[1] ).T;
    uList = np.array( mpc_results[2] ).T;

    print('MPC Complete.');
    return tList, xList, uList;

# main execution block
if __name__ == "__main__":
    # initial position w/ disturbance
    disturbList = (2,3,4);
    disturbance = [[eps*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # get MPC results
    sim_time = 10.0;
    tList, xList, uList = mpcSimulation(sim_time, x0, output=0);

    # plot results
    plotTrajectories(tList, xList, xd, eList=eList, legend='MPC');
    plt.show(block=0);

    # execute simulation
    dtsim = 0.1;
    isim = round(dtsim/dtmpc);

    tSim = [ [i*dtsim for i in range( round(sim_time/dtsim)+1 )] ];
    spEntity = StatePlots(tSim, xList[:,0], xd, limits=eList);

    j = isim;
    for t in tSim[0][1:]:
        print(spEntity.UPDATE_NUM, t, tList[0][j]);
        spEntity.update(t, xList[:,j]);
        j += isim;