import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import dpa
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

        # plt.show(block=0);
        # plt.close('all');

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