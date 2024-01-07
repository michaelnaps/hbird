import numpy as np

# Hyper parameter(s).
g = -9.81
m = 1
dt = 1e-3

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
    n = 6
    Fz = (u[0] + m*g)*np.array( [[0],[0],[1]] )
    dx = np.vstack( (
        x[:n],
        rotz( x[3] )@roty( x[4] )@rotx( x[5] )@Fz,
        u[1:]
    ) )
    return x + dt*dx
