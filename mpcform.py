from root import *
import pidform as pid

# cost function
def cost(mvar, xList, uList):
    xd = mvar.params.xd;
    k = [
        30, 30, 50, 10, 10,         # position costs
        5, 5, 5, 1, 1,              # velocity costs
        0.1, 0.1, 0.1, 0.1, 0.1     # steady state error costs
    ];

    C = 0;  j = 0;
    for i, x in enumerate(xList):
        C += sum([k[i]*(x[i] - xd[i])**2 for i in range(2*dNx)]);

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
    eps = 1.0;
    disturbList = (0,1,2,3,4);
    disturbance = [[eps*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # get MPC results
    sim_time = 10.0;
    tList, xList, uList = mpcSimulation(sim_time, x0, output=1);

    # plot results
    fig, axsList = plotTrajectories(tList, xList, xd, legend='MPC');
    plt.show();