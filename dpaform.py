from root import *

# model and cost functions
def model(x, u):
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

def pcost(x, u):
    kx = 100;
    ku = 0;
    J = sum( [kx*x[i]**2 for i in range(dNx)] );
    J += sum( [ku*u[i]**2 for i in range(Nu)] );
    return J;

def tcost(xN):
    gN = sum( [xN[i]**2 for i in range(dNx)] );
    return gN;

# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0,0,0,0,0];
    x0 = [0,0,0,0,0];

    # create MPC class variable
    N = 10;
    model_type = 'discrete';
    vhc = Vehicle(x0, xd);
    dpvar = dpa.DynamicProgramming(pcost, tcost, model, N, dNx, Nu,
        params=vhc, max_iter=-1);
    dpvar.setAlpha(1);

    # test DPA
    uinit = [0 for i in range(Nu)]
    cost = lambda u: pcost(x0,u) + tcost( model(x0,u) );
    ustar, Jstar = dpvar.minimize(cost, uinit);

    print(ustar, Jstar);
    print(model(x0,ustar));


    # T = np.array( sim_results[0] );
    # xlist = np.array( sim_results[1] );
    # ulist = np.array( sim_results[2] );

    # # plot results
    # fig, axs = plt.subplots(3,1);
    # axs[0].plot(T, xlist[:,0]);
    # axs[0].set_ylim( (-1,1) );

    # axs[1].plot(T, xlist[:,1]);
    # axs[1].set_ylim( (-1,1) );

    # axs[2].plot(T, xlist[:,2]);
    # # axs[1].set_ylim( (-1,1) );

    # plt.show();