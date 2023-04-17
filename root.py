import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
import numpy as np
from numpy import pi
from numpy import random as rd

import math
import matplotlib.pyplot as plt


# hyper parameters
m = 1;
g = 9.81;
a = 1.00;  # discount rate
eps = 0.1;
Nx = 5;
Nu = 3;
dt = 0.01;


# model and cost functions
def model(x, u, _):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    xplus = [
        x[0] + dt/m*F*math.cos(x[3])*math.cos(x[4]),
        x[1] + dt/m*F*math.cos(x[3])*math.sin(x[4]),
        x[2] + dt/m*(F*math.sin(x[3]) - m*g),
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

def cost(mpc_var, xlist, ulist):
    xd = mpc_var.params;
    k = [10,10,100,1,1];

    C = 0;
    for i, x in enumerate(xlist):
        C += (a**i)*sum([k[i]*(x[i] - xd[i])**2 for i in range(Nx)]);

    return C;

if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0,0,1,pi/2,0];
    x0 = [xd[i]+(2*eps*rd.rand()-eps) for i in range(Nx)];

    # create MPC class variable
    PH = 10;
    kl = 1;
    model_type = 'discrete';
    params = xd;
    mpc_var = mpc.ModelPredictiveControl('ngd', model, cost, params, Nu,
        num_ssvar=Nx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=100, model_type=model_type);
    mpc_var.setAlpha(0.1);

    # solve single step
    uinit = [0 for i in range(Nu*PH)];
    u0 = mpc_var.solve(x0, uinit, output=1)[0];

    j = 0;
    for i in range(PH):
        print( u0[j:j+Nu] );
        j += Nu;