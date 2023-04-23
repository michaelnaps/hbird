from root import *

# model and cost functions
def dmodel(x, u, _):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    xplus = [
        x[0] + dt/m*F*math.sin(x[3])*math.cos(x[4]),
        x[1] + dt/m*F*math.sin(x[3])*math.sin(x[4]),
        x[2] + dt/m*(F*math.cos(x[3]) - m*g),
        x[3] + dt*TauZ,
        x[4] + dt*TauXY
    ];

    return xplus;

def cost(mvar, xlist, ulist):
    xd = mvar.params.xd;
    k = [10,10,100,1,1];
    ku = 1;

    C = 0;  j = 0;
    for i, x in enumerate(xlist):
        C += sum([k[i]*(x[i] - xd[i])**2 for i in range(dNx)]);
        # if i < mvar.PH:
        #     C += ku/(10 - abs(ulist[j]));
        #     j += Nu;

    return C;


# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0 for i in range(dNx)];
    x0 = [1*(i==2) for i in range(dNx)];

    # create MPC class variable
    PH = 10;
    kl = 1;
    model_type = 'discrete';
    params = Vehicle(np.zeros((dNx,)), xd);
    mvar = mpc.ModelPredictiveControl('ngd', dmodel, cost, params, Nu,
        num_ssvar=dNx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=100, model_type=model_type);
    mvar.setAlpha(1);

    # solve single step
    sim_time = 0.50;
    uinit = [0 for i in range(Nu*PH)];
    sim_results = mvar.sim_root(sim_time, x0, uinit, output=1);

    T = np.array( sim_results[0] );
    xlist = np.array( sim_results[1] );
    ulist = np.array( sim_results[2] );

    # plot results
    fig, axs = plt.subplots(3,1);
    axs[0].plot(T, xlist[:,0]);
    axs[0].set_ylim( (-1,1) );

    axs[1].plot(T, xlist[:,1]);
    axs[1].set_ylim( (-1,1) );

    axs[2].plot(T, xlist[:,2]);
    # axs[1].set_ylim( (-1,1) );

    plt.show();