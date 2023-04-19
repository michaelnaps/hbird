import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
import numpy as np
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
import matplotlib.patches as patch
import matplotlib.path as path


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
    k = [1, 1, 1];

    v1 = -k[0]*x[0] - k[1]*x[5] - k[2]*x[10];
    v2 = -k[0]*x[1] - k[1]*x[6] - k[2]*x[11];
    v3 = -k[0]*x[2] - k[1]*x[7] - k[2]*x[12];
    v = np.vstack( (v1, v2, v3) );

    u1 = [np.linalg.norm( v )];
    u2 = -k[0]*x[3] - k[1]*x[8] - k[2]*x[13];
    u3 = -k[0]*x[4] - k[1]*x[9] - k[2]*x[14];

    u = np.vstack( (u1, u2, u3) );

    return u;

def noise(eps, shape=(1,1)):
    return 2*eps*np.random.rand(shape[0], shape[1]) - eps;

# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    eps = 1;
    xd = np.zeros( (Nx,1) );
    x0 = xd + noise(eps, (Nx,1));

    # simulate for 10 seconds
    simControl = lambda t, x: model(x, control(x[:,None])).reshape(Nx,);
    sim_results = solve_ivp(simControl, (0, 10), x0.reshape(Nx,));

    T = sim_results.t/10;
    xlist = sim_results.y;

    # plot results
    fig, axs = plt.subplots(3,1);
    axs[0].plot(T, xlist[0,:]);
    # axs[0].set_ylim( (-1,1) );

    axs[1].plot(T, xlist[1,:]);
    # axs[1].set_ylim( (-1,1) );

    axs[2].plot(T, xlist[2,:]);
    # axs[1].set_ylim( (-1,1) );

    plt.show();