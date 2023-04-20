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
eps = 0.1;
dNx = 5;
cNx = 15;
Nu = 3;
dt = 0.1;


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


# class for dynamic programming
class DynamicProgramming:
    def __init__(self, pcost, tcost, model, N, Nx, Nu,
        params=None, max_iter=10,
        appx_zero=1e-6, grad_step=1e-3):
        self.pcost = pcost;
        self.tcost = tcost;
        self.model = model;
        self.params = params;

        self.N = N;
        self.Nx = Nx;
        self.Nu = Nu;

        self.zero = appx_zero;
        self.step = grad_step;

        # if nmax is -1 -> no iteration cap
        self.nmax = max_iter;

        self._alpha = 1;

    def setAlpha(self, a):
        self._alpha = a;
        return;

    def forward(self, x0, uinit, N=None, output=0):
        # if N (prediction horizon) not given
        if N is None:
            # start alg. with class N
            return self.forward(x0, uinit, self.N, output);

        # if N = 1 end recursion by minimizing with terminal cost
        if N == 1:
            cost = lambda u: self.pcost(x0,u) + self.tcost( self.model(x0,u) );
            ustar, Jstar = self.minimize(cost, uinit);
            return ustar, Jstar, self.model( x0,ustar );

        cost = lambda u: self.pcost( x0,u ) + self.forward( self.model(x0, u), u, N=N-1 )[1];
        ustar, Jstar = self.minimize(cost, uinit);
        return ustar, Jstar;

    def minimize(self, cost, uinit, h=1e-3):
        un = [uinit[i] for i in range(self.Nu)];
        gn = self.fdm(cost, un, h=self.step);
        gnorm = sum([gn[i]**2 for i in range(self.Nu)]);

        # gradient descent loop
        count = 1;
        while gnorm > self.zero**2:
            un = [un[i] - self._alpha*gn[i] for i in range(self.Nu)];
            gn = self.fdm(cost, un, h=self.step);
            gnorm = sum([gn[i]**2 for i in range(self.Nu)]);

            count += 1;
            if (self.nmax != -1) and (count > self.nmax-1):  # iteration limit
                print('WARNING: Iteration break in DynamicProgramming.minimize()');
                break;

        return un, cost(un);

    def fdm(self, cost, u, h=1e-3):
        dJ = [0 for i in range(self.Nu)];

        for i in range(self.Nu):
            un1 = [u[j] - (i==j)*h for j in range(self.Nu)];
            up1 = [u[j] + (i==j)*h for j in range(self.Nu)];

            Jn1 = cost(un1);
            Jp1 = cost(up1);

            dJ[i] = (Jp1 - Jn1)/(2*h);

        return dJ;
