import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
import numpy as np
from numpy import pi
from numpy import random as rd

import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
import matplotlib.patches as patch
import matplotlib.path as path


# hyper parameters
m = 1;
g = 9.81;
a = 1.00;  # discount rate
eps = 0.1;
Nx = 5;
Nu = 3;
dt = 0.01;


# simulation vehicle entity
class Vehicle:
    def __init__(self, x0, xd, fig=None, axs=None, buffer_length=10, pause=1e-3):
        # initialize figure properties
        if fig is None:
            self.fig = plt.figure();
            self.axs = plt.axes(projection='3d');
        else:
            self.fig = fig;
            self.axs = axs;

        self.xd = xd;

        self.axs.set_xlim3d(-10, 10);
        self.axs.set_ylim3d(-10, 10);
        self.axs.set_zlim3d(-10, 10);
        # self.axs.axis('equal');
        # self.fig.tight_layout();

        self.axs.set_xlabel('x');
        self.axs.set_ylabel('y');
        self.axs.set_zlabel('z');

        # initialize buffer (trail)
        self.buffer = np.kron( np.ones( (buffer_length, 1) ), x0 );
        self.trail_patch = patch.PathPatch(path.Path(self.buffer[:,0:2]));

        # initialize force directions
        x = x0[0];
        y = x0[1];
        z = x0[2];

        self.Fx_patch = patch.Arrow(x, y, 4, 0, color='r');
        self.Fy_patch = patch.Arrow(x, y, 0, 4, color='g');
        self.Fz_patch = patch.Arrow(z, x, 0, 4, color='b');

        self.axs.add_patch(self.Fx_patch);
        self.axs.add_patch(self.Fy_patch);
        self.axs.add_patch(self.Fz_patch);
        self.axs.add_patch(self.trail_patch);

        plt3d.art3d.pathpatch_2d_to_3d(self.Fx_patch, zdir='z');
        plt3d.art3d.pathpatch_2d_to_3d(self.Fy_patch, zdir='z');
        plt3d.art3d.pathpatch_2d_to_3d(self.Fz_patch, zdir='y');
        plt3d.art3d.pathpatch_2d_to_3d(self.trail_patch, z=self.buffer[:,2]);

        # plot pause
        self.pause = pause;

    def update(self, mvar, t, x, u):
        # update trail
        self.Fx_patch.remove();
        self.Fy_patch.remove();
        self.Fz_patch.remove();
        self.trail_patch.remove();

        self.buffer[:-1] = self.buffer[1:];
        self.buffer[-1] = x;

        xpos = x[0];
        ypos = x[1];
        zpos = x[2];
        delta = x[3];
        theta = x[4];

        F = u[0];
        Fx = F*np.cos(delta)*np.cos(theta);
        Fy = F*np.cos(delta)*np.sin(theta);
        Fz = F*np.sin(delta) - m*g;

        self.Fx_patch = patch.Arrow(xpos, ypos, 4*Fx, 0, color='r');
        self.Fy_patch = patch.Arrow(xpos, ypos, 0, 4*Fy, color='g');
        self.Fz_patch = patch.Arrow(zpos, xpos, 0, 4*Fz, color='b');
        self.trail_patch = patch.PathPatch(path.Path(self.buffer[:,0:2]), fill=0);

        self.axs.add_patch(self.Fx_patch);
        self.axs.add_patch(self.Fy_patch);
        self.axs.add_patch(self.Fz_patch);
        self.axs.add_patch(self.trail_patch);

        plt3d.art3d.pathpatch_2d_to_3d(self.Fx_patch, zdir='z');
        plt3d.art3d.pathpatch_2d_to_3d(self.Fy_patch, zdir='z');
        plt3d.art3d.pathpatch_2d_to_3d(self.Fz_patch, zdir='y');
        plt3d.art3d.pathpatch_2d_to_3d(self.trail_patch, z=self.buffer[:,2]);

        plt.show(block=0);
        plt.pause(self.pause);

        return self;


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

def cost(mvar, xlist, ulist):
    xd = mvar.params.xd;
    k = [10,10,100,1,1];

    C = 0;  j = 0;
    for i, x in enumerate(xlist):
        C += (a**i)*sum([k[i]*(x[i] - xd[i])**2 for i in range(Nx)]);
        if i < mvar.PH:
            C += 1/(10 - ulist[j]);
            j += Nu;


    return C;


# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0,0,1,pi/2,0];
    x0 = [0,0,0,pi/2,0];

    # create MPC class variable
    PH = 10;
    kl = 1;
    model_type = 'discrete';
    params = Vehicle(np.zeros((Nx,)), xd);
    mvar = mpc.ModelPredictiveControl('ngd', model, cost, params, Nu,
        num_ssvar=Nx, PH_length=PH, knot_length=kl, time_step=dt,
        max_iter=1000, model_type=model_type);
    mvar.setAlpha(0.1);

    # solve single step
    sim_time = 1;
    uinit = [0 for i in range(Nu*PH)];
    mvar.sim_root(sim_time, x0, uinit, output=1);
