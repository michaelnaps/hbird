import numpy as np
import matplotlib.pyplot as plt

# System dimensions.
n = 12          # Dimensions of state space.
m = 4           # Dimensions of controller.

# Hyper parameter(s).
M = 1.00        # Center of mass [g].
g = -9.81       # Gravity coefficient [m/s^2].
c = 1e-1        # Coefficient of air friction.
dt = 1e-3       # Simulation time-step [s].

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
        [np.sin( theta ),  np.cos( theta ), 0],
        [0, 0, 1]
    ] )
    return R

# Cumulative rotation function.
def rot(x):
    R = rotz( x[2] )@roty( x[1] )@rotx( x[0] )
    return R

# Skew-symmetrix matrix.
def skew(x):
    y = np.array( [
        [0, -x[2], x[1]],
        [x[2], 0, -x[0]],
        [-x[1], x[0], 0]
    ] )
    return y

# Hummingbird model function.
def model(X, U):
    # Lift force.
    w = X.shape[1]
    F = np.vstack( (np.zeros( (2,w) ), U[0]) )
    G = np.array( [[0],[0],[M*g]] )

    # First derivative.
    dX = X[round( n/2 ):]

    # Second derivative.
    ddX = np.empty( (round( n/2 ), w) )
    for i in range( w ):
        ddx = np.vstack( (
            rot( X[3:6,i] )@F[:,i,None] + G,
            U[1:,i,None]
        ) ) - c*dX[:,i,None]
        ddX[:,i] = ddx[:,0]

    # Return next value of simulation.
    return X + dt*np.vstack( (dX, ddX) )

# Testing algebraic model function.
def model2(X, U):
    # Lift force.
    w = X.shape[1]
    F = U[0]
    G = M*g

    # First derivative.
    dX = X[round( n/2 ):]

    # Second derivative.
    ddX = np.empty( (round( n/2 ), w) )
    for i in range( w ):
        ddx = np.vstack( (
            F*(np.sin( X[3,i] )*np.sin( X[5,i] ) - np.cos( X[3,i] )*np.sin( X[4,i] )*np.cos( X[5,i] )),
            F*(-np.sin( X[3,i] )*np.cos( X[5,i] ) - np.cos( X[3,i] )*np.sin( X[4,i] )*np.sin( X[5,i] )),
            F*(np.cos( X[3,i] )*np.cos( X[4,i] )) + G,
            U[1:,i,None]
        ) ) - c*dX[:,i,None]
        ddX[:,i] = ddx[:,0]

    # Return new states.
    return X + dt*np.vstack( (dX, ddX) )


# Plot functions.
# State plots split between two rows.
def plotStates(tlist, Xlist, slist=None):
    # Plot dimensions and initialization.
    n, w, _ = Xlist.shape
    r = round( n/2 )
    fig, axslist = plt.subplots( 2, r )

    # If slist=None, show every trajectory.
    slist = [True for j in range( w )] if slist is None else slist

    # Iterate through axes and plot states.
    i = 0
    for axsrow in axslist:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                linestyle='--', color='indianred' )
            for j in range( w ):
                if slist[j]:
                    axs.plot( tlist, Xlist[i,j] )
            i = i + 1

    # Return figure variables.
    return fig, axslist


# System state plots.
def plotPosVel2D(tlist, Xlist, slist=None):
    # Plot position and velocity separately.
    fig1, axspos = plotStates( tlist, Xlist[:6], slist=slist )
    fig2, axsvel = plotStates( tlist, Xlist[6:], slist=slist )

    # Return figure variables.
    return (fig1, fig2), (axspos, axsvel)

# System state plots in 3-D.
def plotPosVel3D(Xlist, slist=None):
    # If slist=None, show every trajectory.
    slist = [True for j in range( w )] if slist is None else slist

    # Initialize plot variable.
    n, w, _ = Xlist.shape
    fig = plt.figure()

    # Iterate through adding 3-D axes and labels.
    axslist = []
    for i, k in enumerate( range( 3,n+1,3 ) ):
        X = Xlist[(k-3):k]
        axs = fig.add_subplot( 2, 2, i+1, projection='3d' )
        for j in range( w ):
            if slist[j]:
                axs.plot( X[0,j], X[1,j], X[2,j] )
                axs.plot( X[0,j,-1], X[1,j,-1], X[2,j,-1],
                    marker='x', color='indianred' )
        axs.set_xlabel( '$x_{%i}$'%(k-2) )
        axs.set_ylabel( '$x_{%i}$'%(k-1) )
        axs.set_zlabel( '$x_{%i}$'%(k-0) )
        axs.axis( 'equal' )
        axslist = axslist + [axs]

    # Return plot variables.
    return fig, axslist

# Candidate function plot for all initial conditions.
def plotLyapunovCandidate(tlist, Xlist, Vlist, ilist, slist):
    # Initialize figure variables.
    fig, axslist = plt.subplots( 1,2 )

    # Plot candidate trends.
    axslist[0].plot( tlist, Vlist.T )

    # Plot initial conditions with appropriate colors.
    colors = ['cornflowerblue', 'indianred']
    for i, x0 in enumerate( Xlist[:,:,0].T ):
        axslist[1].plot( x0[ilist[0]-1], x0[ilist[1]-1], marker='x',
            color=colors[0] if slist[i] else colors[1] )

    # Return figure variables.
    return fig, axslist

# Error in rotation derivative for all initial conditions.
def plotRotationError(tlist, Rlist):
    # Initialize figure variables.
    fig, axs = plt.subplots()

    # Plot rotation derivative error.
    axs.plot( tlist, Rlist.T )

    # Return figure variables.
    return fig, axs
