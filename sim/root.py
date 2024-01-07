import numpy as np

# System dimensions.
n = 12          # Dimensions of state space.
m = 4           # Number of control inputs.

# Hyper parameter(s).
M = 4.00        # Center of mass [g].
g = -9.81       # Gravity coefficient [m/s^2].
c = 1e-3        # Coefficient of air friction.
dt = 1e-3       # Simulation time-step [s].

# 3-D rotation functions.
def rotx(theta):
    R = np.array( [
        [1, 0, 0],
        [0, np.cos(theta[0]), -np.sin(theta[0])],
        [0, np.sin(theta[0]),  np.cos(theta[0])]
    ] )
    return R

def roty(theta):
    R = np.array( [
        [np.cos(theta[0]), 0, -np.sin(theta[0])],
        [0, 1, 0],
        [np.sin(theta[0]), 0,  np.cos(theta[0])]
    ] )
    return R

def rotz(theta):
    R = np.array( [
        [np.cos(theta[0]), -np.sin(theta[0]), 0],
        [np.cos(theta[0]),  np.sin(theta[0]), 0],
        [0, 0, 1]
    ] )
    return R

# Model function.
def model(x, u):
    # Lift force.
    Fz = np.array( [[0],[0],(u[0] + M*g)] )

    # First derivative.
    dx = x[round( n/2 ):]

    # Second derivative.
    ddx = np.vstack( (
        rotz( x[3] )@roty( x[4] )@rotx( x[5] )@Fz,
        u[1:]
    ) ) - c*dx

    # Return next value of simulation.
    return x + dt*np.vstack( (dx, ddx) )
