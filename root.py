import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
from numpy import random as rd

import math
import matplotlib.pyplot as plt


# hyper parameters
m = 1;
g = 9.81;
Nx = 5;
Nu = 3;
dt = 0.01;


# model and cost functions
def model(x, u, mpc_var):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    dx = [
        x[0] + dt/m*F*math.cos(x[3])*math.cos(x[4]),
        x[1] + dt/m*F*math.cos(x[3])*math.sin(x[4]),
        x[2] + dt/m*(F*math.sin(x[3]) - m*g),
        x[3] + dt*TauZ,
        x[4] + dt*TauXY
    ];

    return dx;

def cost(mpc_var, xlist, ulist):
    xd = mpc_var.params;
    k = [10,10,10,1,1];

    C = 0;
    for x in xlist:
        C += sum([k[i]*(x[i] - xd[i])**2 for i in range(Nx)]);

    return C;

if __name__ == "__main__":
    # initialize starting and goal states
    x0 = [0 for i in range(Nx)];
    xd = [0,0,1,0,0];

    # create MPC class variable
    PH = 10;
    kl = 1;
    model_type = 'discrete';
    params = xd;
    mpc_var = mpc.ModelPredictiveControl('ngd', model, cost, params, Nu,
        num_ssvar=Nx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=100, model_type=model_type);
    mpc_var.setAlpha(1);

    # solve single time-step
    uinit = [0 for i in range(Nu*PH)];
    print( mpc_var.solve(x0, uinit, output=1)[3] );