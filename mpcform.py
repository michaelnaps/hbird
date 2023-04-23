from root import *
import pidform as pid

# cost function
def cost(mvar, xList, uList):
    xd = mvar.params.xd;
    k = [25,25,100,15,15,1,1,1,1,1];

    C = 0;  j = 0;
    for i, x in enumerate(xList):
        C += sum([k[i]*(x[i] - xd[i])**2 for i in range(2*dNx)]);

    return C;

# main execution block
if __name__ == "__main__":
    # initial position w/ disturbance
    eps = 0.1;
    disturbance = [[(i in states[:,0])*eps] for i in range(cNx)];
    x0 = xd + disturbance;

    # simulation length
    T = 20;  Nt = round(T/dt) + 1;
    tList = [[i*dt for i in range(Nt)]];

    # create MPC class variable
    xdmpc = [xd[i][0] for i in range(cNx)];
    modelmpc = lambda x, u, _: dmodel(x,u);
    PH = 10;
    kl = 5;
    max_iter = 250;
    model_type = 'discrete';
    params = Vehicle(np.zeros((cNx,)), xdmpc);
    mvar = mpc.ModelPredictiveControl('ngd', modelmpc, cost, params, Nu,
        num_ssvar=cNx, PH_length=PH, knot_length=kl, time_step=dt,
        appx_zero=1e-3, max_iter=max_iter, model_type=model_type);
    mvar.setAlpha(1);

    # get pid results
    pid_results = pid.pidSimulation(tList, x0)
    print('PID Complete.');

    # solve single step
    sim_time = 10;
    uinit = [0 for i in range(Nu*PH)];
    x0mpc = [x0[i][0] for i in range(cNx)];  print(x0mpc);
    mpc_results = mvar.sim_root(sim_time, x0mpc, uinit);
    print('MPC Complete.');

    # comparison plots
    fig, axsList = plotTrajectories(labels, states, tList, pid_results);

    tmpc = [mpc_results[0]];
    xmpc = np.array( mpc_results[1] ).T;
    fig, axsList = plotTrajectories(labels, states, tmpc, xmpc, fig, axsList);

    plt.show();