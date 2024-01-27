import numpy as np
import matplotlib.pyplot as plt

# System dimensions.
n = 12          # Dimensions of state space.
m = 4           # Dimensions of controller.

# Hyper parameter(s).
M = 1.00        # Center of mass [g].
g = -9.81       # Gravity coefficient [m/s^2].
c = 1e-1        # Coefficient of air friction.
dt = 1e-4       # Simulation time-step [s].

# 3-D rotation functions.
# Rotation about the x-axis.
def rotx(theta):
    R = np.array( [
        [1, 0, 0],
        [0, np.cos( theta ), -np.sin( theta )],
        [0, np.sin( theta ),  np.cos( theta )]
    ] )
    return R

# Rotation about the y-axis.
def roty(theta):
    R = np.array( [
        [np.cos( theta ), 0, -np.sin( theta )],
        [0, 1, 0],
        [np.sin( theta ), 0,  np.cos( theta )]
    ] )
    return R

# Rotation about the z-axis.
def rotz(theta):
    R = np.array( [
        [np.cos( theta ), -np.sin( theta ), 0],
        [np.cos( theta ),  np.sin( theta ), 0],
        [0, 0, 1]
    ] )
    return R

# Hummingbird model function.
def model(X, U):
    # Lift force.
    w = X.shape[1]
    F = np.vstack( (np.zeros( (2,w) ), U[0]) )
    G = np.array( [[0],[0],[M*g]] )

    # First derivative.
    dX = X[round( n/2 ):]

    # Second derivative.
    ddX = np.empty( (round( n/2 ),w) )
    for i in range( w ):
        ddx = np.vstack( (
            rotx( X[3,i] )@roty( X[4,i] )@rotz( X[5,i] )@F[:,i,None] + G,
            U[1:,i,None]
        ) ) - c*dX[:,i,None]
        ddX[:,i] = ddx[:,0]

    # Return next value of simulation.
    return X + dt*np.vstack( (dX, ddX) )
