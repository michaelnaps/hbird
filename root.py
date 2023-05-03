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
eps = 0.5;      # disturbance range -> [-eps, eps]
m = 3.00;       # hummingbird mass [g]
g = 9.81;       # gravitational energy
c = 2.00;       # coefficient of air friction
N3 = 3;
dNx = 5;
cNx = 15;
Nu = 3;

dtpid = 0.01;
dtmpc = 0.025;

# label and state grouping meta-data
xd = np.zeros( (cNx,1) );
labels = ['$x_{'+str(i+1)+'}$' for i in range(cNx)];
states = np.array( [
    [0, 5, 10],
    [1, 6, 11],
    [2, 7, 12],
    [3, 8, 13],
    [4, 9, 14]] );
eList = (
    eps, eps, eps, eps, eps,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0);

# simulation vehicle entity
class Vehicle:
    def __init__(self, x0, xd,
        fig=None, axs=None, title=None, legend=None,
        buffer_length=100, pause=1e-3):
        # initialize figure properties
        if fig is None:
            self.fig = plt.figure();
            self.axs = plt.axes(projection='3d');
        else:
            self.fig = fig;
            self.axs = axs;

        if title is not None:
            plt.set_title( title );

        # class variables
        self.xd = xd;
        self.pause = pause;
        self.legend=legend;

        self.axs.set_xlim3d(-eps, eps);
        self.axs.set_ylim3d(-eps, eps);
        self.axs.set_zlim3d(-eps, eps);
        # self.axs.axis('equal');
        # self.fig.tight_layout();

        self.axs.set_xlabel('x');
        self.axs.set_ylabel('y');
        self.axs.set_zlabel('z');

        # initialize buffer (trail)
        self.buffer = np.kron( np.ones( (buffer_length, 1) ), x0 );
        self.trail_patch = patch.PathPatch(path.Path(self.buffer[:,0:2]),
            label=self.legend);

        # initialize force directions
        x = x0[0];
        y = x0[1];
        z = x0[2];

        # self.Fx_patch = patch.Arrow(x, y, 4, 0, color='r');
        # self.Fy_patch = patch.Arrow(x, y, 0, 4, color='g');
        # self.Fz_patch = patch.Arrow(z, x, 0, 4, color='b');

        # self.axs.add_patch(self.Fx_patch);
        # self.axs.add_patch(self.Fy_patch);
        # self.axs.add_patch(self.Fz_patch);
        self.axs.add_patch(self.trail_patch);

        # plt3d.art3d.pathpatch_2d_to_3d(self.Fx_patch, zdir='z');
        # plt3d.art3d.pathpatch_2d_to_3d(self.Fy_patch, zdir='z');
        # plt3d.art3d.pathpatch_2d_to_3d(self.Fz_patch, zdir='y');
        plt3d.art3d.pathpatch_2d_to_3d(self.trail_patch, z=self.buffer[:,2]);

    def update(self, t, x):
        # update trail
        # self.Fx_patch.remove();
        # self.Fy_patch.remove();
        # self.Fz_patch.remove();
        self.trail_patch.remove();

        self.buffer[:-1] = self.buffer[1:];
        self.buffer[-1] = x;

        # xpos = x[0];
        # ypos = x[1];
        # zpos = x[2];
        # delta = x[3];
        # theta = x[4];

        # F = u[0];
        # Fx = F*np.sin(delta)*np.cos(theta);
        # Fy = F*np.sin(delta)*np.sin(theta);
        # Fz = F*np.cos(delta) - m*g;

        # self.Fx_patch = patch.Arrow(xpos, ypos, 4*Fx, 0, color='r');
        # self.Fy_patch = patch.Arrow(xpos, ypos, 0, 4*Fy, color='g');
        # self.Fz_patch = patch.Arrow(zpos, xpos, 0, 4*Fz, color='b');
        self.trail_patch = patch.PathPatch(path.Path(self.buffer[:,0:2]), fill=0);

        # self.axs.add_patch(self.Fx_patch);
        # self.axs.add_patch(self.Fy_patch);
        # self.axs.add_patch(self.Fz_patch);
        self.axs.add_patch(self.trail_patch);

        # plt3d.art3d.pathpatch_2d_to_3d(self.Fx_patch, zdir='z');
        # plt3d.art3d.pathpatch_2d_to_3d(self.Fy_patch, zdir='z');
        # plt3d.art3d.pathpatch_2d_to_3d(self.Fz_patch, zdir='y');
        plt3d.art3d.pathpatch_2d_to_3d(self.trail_patch, z=self.buffer[:,2]);

        self.axs.legend();
        plt.show(block=0);
        plt.pause(self.pause);

        return self;

class StatePlots:
    def __init__(self, tList, x0, xd, limits=None,
        fig=None, axsMat=None, color='k', linestyle=None,
        pause=1e-3):
        self.n = len(states);
        self.m = len(states[0]);
        if (fig is None) or (axsMat is None):
            self.fig, self.axsMat = plt.subplots(self.n, self.m);

            titleList = ('Position', 'Velocity', 'Error');
            for i, axs in enumerate(self.axsMat[0]):
                axs.set_title(titleList[i]);
        else:
            self.fig = fig;
            self.axsMat = axsMat;

        # plotting variables
        self.UPDATE_NUM = 0;
        self.limits = limits;
        self.pause = pause;
        self.color = color;
        self.linestyle = linestyle;

        # state space variables
        self.xd = xd;
        self.tList = tList;
        self.Nt = len(self.tList[0]);

        for i, axsList in enumerate(self.axsMat):
            for j, axs in enumerate(axsList):
                stateid = states[i][j];
                axs.plot([tList[0][0], tList[0][-1]], [xd[i], xd[i]],
                    color='r', linestyle=':');
                axs.set_xlim(0, self.tList[0][-1]);
                axs.grid(1);
                if self.limits[stateid] != 0:
                    axs.set_ylim(-self.limits[stateid], self.limits[stateid])
                else:
                    axs.set_ylim(-2*eps,2*eps);

        self.bufferMat = np.empty( (cNx, self.Nt) );
        self.pathPatchList = np.empty( (cNx,), dtype=patch.PathPatch );
        for i in range(self.n*self.m):
            self.bufferMat[i] = [x0[i] for j in range(self.Nt)];

            bufferPath = np.array( [self.tList[0], self.bufferMat[i]] );
            bufferPathPatch = path.Path( bufferPath.T );

            self.pathPatchList[i] = patch.PathPatch( bufferPathPatch );

        for i, axsList in enumerate(self.axsMat):
            for j, axs in enumerate(axsList):
                stateid = states[i][j];
                axs.add_patch( self.pathPatchList[stateid] )

    def update(self, t, x):
        for i, pathEntity in enumerate(self.pathPatchList):
            pathEntity.remove();

        self.UPDATE_NUM += 1;

        for i in range(self.n*self.m):
            self.bufferMat[i][self.UPDATE_NUM] = x[i];

            buffer = self.bufferMat[i][:self.UPDATE_NUM+1];
            tList = self.tList[0][:self.UPDATE_NUM+1];
            bufferPath = np.array( [tList, buffer] );
            bufferPathPatch = path.Path( bufferPath.T );

            self.pathPatchList[i] = patch.PathPatch( bufferPathPatch,
                color=self.color, linestyle=self.linestyle, fill=0 );

        for i, axsList in enumerate(self.axsMat):
            for j, axs in enumerate(axsList):
                axs.add_patch( self.pathPatchList[states[i][j]] )

        plt.show(block=0);
        plt.pause(self.pause)

        return self;

# model functions
def cmodel(x, u):
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
        1/m*(F*np.sin(x[3])*np.cos(x[4]) - c*x[5]),
        1/m*(F*np.sin(x[3])*np.sin(x[4]) - c*x[6]),
        1/m*(F*np.cos(x[3]) - m*g - c*x[7]),
        TauZ,
        TauXY
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

def dmodel(x, u):
    dt = dtmpc;

    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    xplus = [
        x[0] + dt*x[5],
        x[1] + dt*x[6],
        x[2] + dt*x[7],
        x[3] + dt*x[8],
        x[4] + dt*x[9],

        x[5] + dt/m*F*math.sin(x[3])*math.cos(x[4]),
        x[6] + dt/m*F*math.sin(x[3])*math.sin(x[4]),
        x[7] + dt/m*(F*math.cos(x[3]) - m*g),
        x[8] + dt*TauZ,
        x[9] + dt*TauXY,

        x[10] + dt*x[0],
        x[11] + dt*x[1],
        x[12] + dt*x[2],
        x[13] + dt*x[3],
        x[14] + dt*x[4]
    ];

    return xplus;

def noise(eps, shape=None):
    if shape is None:
        return 2*eps*np.random.rand() - eps;
    if len(shape) == 1:
        return 2*eps*np.random.rand(shape[0]) - eps;
    return 2*eps*np.random.rand(shape[0], shape[1]) - eps;

# assorted plotting functions
def plotTrajectories(tList, xList, xRef=None,
    plotList=None, eList=None,
    fig=None, axsList=None, linestyle=None, legend=None):
    if fig is None:
        fig, axsList = plt.subplots(len(states), len(states[0]));

    for j, axs in enumerate(axsList):

        for k, i in enumerate(states[j]):
            if xRef is not None:
                axs[k].plot([tList[0][0], tList[0][-1]], [xRef[i], xRef[i]],
                    color='r', linestyle='--');

            axs[k].plot(tList[0], xList[i,:], linestyle=linestyle, label=legend);
            axs[k].set_ylabel(labels[i]);

            if eList[i] > 0:
                axs[k].set_ylim(-eList[i],eList[i])
            elif max( abs(xList[i,:]) ) < 1:
                axs[k].set_ylim(-1,1);

    axsList[0,0].set_title('Position');
    axsList[0,1].set_title('Velocity');
    axsList[0,2].set_title('Error');

    if legend is not None:
        axsList[0,-1].legend(loc='upper right');

    fig.tight_layout();
    return fig, axsList;

def animateSingle(tList, xList, legend=None):
    # initialize plot and vehicle variables
    vhc = Vehicle(xList[:,0], xd, legend=legend);
    plt.show(block=0);

    # iterate through list of states
    Nt = len(tList[0]);
    for i in range(Nt):
        vhc.update(i, xList[:,i]);

    return vhc;

def animateComparisons(tList, xPID, xMPC):
    # initialize vehicles
    vhc_pid = Vehicle(xPID[:N3,0], xd, legend='PID');
    vhc_mpc = Vehicle(xMPC[:N3,0], xd, fig=vhc_pid.fig, axs=vhc_pid.axs,
        legend='MPC');

    # simulation loop
    Nt = len(tList[0]);
    j = 0;
    skip = round( dtmpc/dtpid );
    for i in range(Nt):
        vhc_pid.update(i, xPID[:N3,i]);
        if i % skip == 0:
            vhc_mpc.update(i, xMPC[:N3,j]);
            j += skip;