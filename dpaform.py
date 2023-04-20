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
    Nx = len(x);
    Nu = len(u);
    J = sum( [x[i]**2 for i in range(Nx)] );
    J += sum( [(u[i]-1)**2 for i in range(Nu)] );
    return J;

def tcost(x):
    Nx = len(x);
    J = sum( [x[i]**2 for i in range(Nx)] );
    return J;

# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0,0,0,0,0];
    x0 = [0,0,0,0,0];

    # create MPC class variable
    PH = 15;
    kl = 1;
    model_type = 'discrete';
    vhc = Vehicle(x0, xd);
    dpvar = dpa.DynamicProgramming(pcost, tcost, model, PH, Nx, Nu,
        params=vhc, max_iter=100);
    dpvar.setAlpha(0.1);

    # test DPA
    uinit = [0 for i in range(Nu)]
    print( dpvar.forward(x0, uinit, N=1) );

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