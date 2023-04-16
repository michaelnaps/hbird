import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
from numpy import random as rd

import math
import matplotlib.pyplot as plt


# hyper parameters
m = 1;
g = 9.81;
Nx = 10;
Nu = 3;
dt = 0.01;


# model and cost functions
def model(x, u, mpc_var):
    F     = u[0];
    tauZ  = u[1];
    tauXY = u[2];

    dx = [
        x[0] + dt*x[5],
        x[1] + dt*x[6],
        x[2] + dt*x[7],
        x[3] + dt*x[8],
        x[4] + dt*x[9],
        x[5] + dt/m*F*math.cos(x[3])*math.cos(x[4]),
        x[6] + dt/m*F*math.cos(x[3])*math.sin(x[4]),
        x[7] + dt/m*(F*math.sin(x[3]) - m*g),
        x[8] + dt*tauZ,
        x[9] + dt*tauXY
    ];

    return dx;

def cost(mpc_var, xlist, ulist):
    xd = mpc_var.params;
    k = [10,10,10,1,1,1,1,1,1,1];

    C = 0;
    for x in xlist:
        C += sum([k[i]*(x[i] - xd[i])**2 for i in range(Nx)]);

    print(C);
    return C;

if __name__ == "__main__":
    # initialize starting and goal states
    x0 = [0 for i in range(2*Nx)];
    xd = [0,0,1,0,0,0,0,0,0,0];

    # create MPC class variable
    PH = 2;
    kl = 1;
    model_type = 'discrete';
    params = xd;
    mpc_var = mpc.ModelPredictiveControl('ngd', model, cost, params, Nu,
        num_ssvar=Nx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=10, model_type=model_type);
    mpc_var.setAlpha(0.01);

    # solve single time-step
    uinit = [0 for i in range(Nu*PH)];
    print( mpc_var.solve(x0, uinit, output=1)[0] );