import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
import numpy as np
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
import matplotlib.patches as patch
import matplotlib.path as path

# suppress print statements
np.set_printoptions(precision=5, suppress=True);


# hyper parameters
m = 1;
g = 9.81;
a = 1.00;  # discount rate
eps = 0.1;
Nx = 15;
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

        plt.show(block=0);
        plt.close('all');

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

    dx0 = np.array( [
        x[5],
        x[6],
        x[7],
        x[8],
        x[9]
    ] ).reshape(5,1);

    dx1 = np.array( [
        F*np.sin(x[3])*np.cos(x[4]),
        F*np.sin(x[3])*np.sin(x[4]),
        (F*np.cos(x[3]) - m*g),
        m*TauZ,
        m*TauXY
    ] ).reshape(5,1);

    dx2 = np.array( [
        x[0],
        x[1],
        x[2],
        x[3],
        x[4]
    ] ).reshape(5,1);

    dx = np.vstack( (dx0, dx1, dx2) );

    return dx;

def linearized(x, u, xd):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    x1 = xd[0];  x2 = xd[1];
    x3 = xd[2];  x4 = xd[3];
    x5 = xd[4];

    n = 5;
    Ax0 = np.hstack( (np.zeros( (n,n) ), np.eye(n), np.zeros( (n,n) )) );
    Ay  = np.hstack( (np.eye(n), np.zeros( (n,2*n) )) );
    Ax1 = np.array( [
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1]
    ] );

    print(A);
    print(B);

    xplus = A@np.array(x)[:,None] + B@np.array(u)[:,None];

    return xplus.reshape(Nx,);

def control(x):
    k = [0.95, 0.05, 0];
    d = [0, 0, 0];

    u1 = d[0] - k[0]*x[2] - k[1]*x[7] - k[2]*x[12];
    u2 = d[1] - k[0]*x[3] - k[1]*x[8] - k[2]*x[13];
    u3 = d[2] - k[0]*x[4] - k[1]*x[9] - k[2]*x[14];

    u = np.vstack( (u1, u2, u3) );

    return u;

def noise(eps, shape=(1,1)):
    if len(shape) == 1:
        return 2*eps*np.random.rand(shape[0]) - eps;
    return 2*eps*np.random.rand(shape[0], shape[1]) - eps;

# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = np.zeros( (Nx,1) );

    eps = 0.1;
    disturbance = noise(eps, xd.shape);
    x0 = xd + disturbance;
    # print(x0);

    # simulation
    T = 2.5;  Nt = round(T/dt) + 1;
    tlist = [[i*dt for i in range(Nt)]];

    # main simulation loop
    x = x0;
    xlist = np.empty( (Nx,Nt) );
    for i in range(Nt):
        x = model(x, control(x));
        xlist[:,i] = x[:,0];
        print(x);

    # plot results
    fig, axsList = plt.subplots(Nx,1);

    for i, axs in enumerate(axsList):
        axs.plot(tlist[0], xlist[i,:]);

    plt.show();