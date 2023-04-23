from root import *
import pidform as pid

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

def cost(mvar, xList, uList):
    xd = mvar.params.xd;
    k = [10,10,100,1,1];

    C = 0;  j = 0;
    for i, x in enumerate(xList):
        C += sum([k[i]*(x[i] - xd[i])**2 for i in range(dNx)]);

    return C;

# main execution block
if __name__ == "__main__":
    # initial position w/ disturbance
    eps = 1;
    disturbance = [[(i in states[:,0])*eps] for i in range(cNx)];
    x0 = xd + disturbance;

    # simulation length
    T = 20;  Nt = round(T/dt) + 1;
    tList = [[i*dt for i in range(Nt)]];

    # create MPC class variable
    PH = 10;
    kl = 1;
    model_type = 'discrete';
    params = Vehicle(np.zeros((cNx,)), xd);
    mvar = mpc.ModelPredictiveControl('ngd', dmodel, cost, params, Nu,
        num_ssvar=cNx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=100, model_type=model_type);
    mvar.setAlpha(1);

    # get pid results
    fig, axsList, pid_results = pid.pidSimulation(tList, x0)

    # solve single step
    sim_time = 20;
    uinit = [0 for i in range(Nu*PH)];
    mpc_results = mvar.sim_root(sim_time, x0.T[0], uinit, output=0);