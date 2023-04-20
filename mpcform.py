from root import *

# model and cost functions
def model(x, u, _):
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

def linearized(x, u, _):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    w = dt/m;

    xd = _.params;
    x1 = xd[0];  x2 = xd[1];
    x3 = xd[2];  x4 = xd[3];
    x5 = xd[4];

    A = np.array( [
        [1, 0, 0, -w*F*np.sin(x4)*np.cos(x5), -w*F*np.cos(x4)*np.sin(x5)],
        [0, 1, 0, -w*F*np.sin(x4)*np.sin(x5),  w*F*np.cos(x4)*np.sin(x5)],
        [0, 0, 1,  w*np.cos(x4),               0                        ],
        [0, 0, 0,  1,                          0                        ],
        [0, 0, 0,  0,                          1                        ]
    ] );

    B = np.array( [
        [w*np.cos(x4)*np.cos(x5), 0,  0 ],
        [w*np.cos(x4)*np.sin(x5), 0,  0 ],
        [w*np.sin(x4)           , 0,  0 ],
        [0,                       dt, 0 ],
        [0,                       0,  dt]
    ] );

    print(A);
    print(B);

    xplus = A@np.array(x)[:,None] + B@np.array(u)[:,None];

    return xplus.reshape(Nx,);

def control(x):
    return [m*g,0,0];

def cost(mvar, xlist, ulist):
    xd = mvar.params.xd;
    k = [10,10,100,1,1];
    ku = 1;

    C = 0;  j = 0;
    for i, x in enumerate(xlist):
        C += (a**i)*sum([k[i]*(x[i] - xd[i])**2 for i in range(Nx)]);
        # if i < mvar.PH:
        #     C += ku/(10 - abs(ulist[j]));
        #     j += Nu;

    return C;


# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0,0,1,0,0];
    x0 = [0,0,0,0,0];

    # create MPC class variable
    PH = 15;
    kl = 1;
    model_type = 'discrete';
    params = Vehicle(np.zeros((Nx,)), xd);
    mvar = mpc.ModelPredictiveControl('ngd', model, cost, params, Nu,
        num_ssvar=Nx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=1000, model_type=model_type);
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