from root import *
from initial import *

w = 5

k = -10.0
c = -2.50
h = 1/-k
r = 1/-c

TOL = 2.5e4

# PD controller.
def control(X):
    U = np.array( [
        (k*X[2] + c*X[8] - M*g),
        (k*X[3] + c*X[9]) + (h*X[1] + r*X[7]),
        (k*X[4] + c*X[10]) + (h*X[0] + r*X[6]),
        (k*X[5] + c*X[11])
    ] )
    return U

# Lyapunov candidate function.
def lyapunovCandidate(X):
    n, w = X.shape
    V = np.empty( (1,w) )
    # V1 = np.empty( (1,w) )
    # V2 = np.empty( (1,w) )
    # V3 = np.empty( (1,w) )
    # V4 = np.empty( (1,w) )
    for i, x in enumerate( X.T ):
        V[:,i] = x[:,None].T@x[:,None]
        # R = rot( x[3:6,None] )
        # v = x[0:3,None];  dv = x[3:6,None]
        # w = x[6:9,None];  dw = x[9:12,None]
        # V1[:,i] = v.T@v;  V2[:,i] = dv.T@dv
        # V3[:,i] = w.T@w;  V4[:,i] = dw.T@dw
    # V = V1 + V2 + V3 + V4
    return V

if __name__ == '__main__':
    # Simulation length.
    T = 1;  Nt = round( T/dt ) + 1
    tlist = np.array( [i*dt for i in range( Nt )] )

    # Date set initialization.
    A = np.pi;  ilist = [2,6]
    Xlist, w = initrand( n, w, Nt, A, ilist )

    # Candidate function initialization.
    Vlist = np.empty( (w,Nt) )
    Vlist[:,0] = lyapunovCandidate( Xlist[:,:,0] )

    # Testing rotation derivative...
    Rlist = np.empty( (w,Nt) )
    Rlist[:,0] = np.zeros( (w,) )

    # Simulation block.
    Tstep = 1
    for t in range( Nt-1 ):
        for i, x in enumerate( Xlist[:,:,t].T ):
            # TODO: Simplify this if-else check.
            if not np.isfinite( x[0] ):
                continue
            if lyapunovCandidate( x[:,None] ) > TOL:
                Xlist[:,i,t+1:] = np.inf
                continue

            # Update state and save.
            xn = model( x[:,None], control( x[:,None] ) )
            Xlist[:,i,t+1] = xn[:,0]

            # # Save error in rotation derivative.
            # R = rot( x[3:6] )
            # S = skew( x[9:12] )
            # Rn = rot( xn[3:6] )
            # # Rlist[i,t+1] = np.linalg.norm( Rn.T@xn[6:9] )
            # Rlist[i,t+1] = np.linalg.norm( Rn - (R + dt*R@S) )

            # # Test trace equation.
            # R = rot( x[3:6] )@rot( x[3:6] )
            # Rx = rotx( x[3] );  Ry = roty( x[4] );  Rz = rotz( x[5] )
            # trR = 0
            # for i1 in range( 3 ):
            #     for i2 in range( 3 ):
            #         for i3 in range( 3 ):
            #             for i4 in range( 3 ):
            #                 for i5 in range( 3 ):
            #                     for i6 in range( 3 ):
            #                         trR = trR + Rz[i1,i2]*Ry[i2,i3]*Rx[i3,i4]*Rz[i4,i5]*Ry[i5,i6]*Rx[i6,i1]
            # Rlist[i,t+1] = np.linalg.norm( np.trace( R ) - trR )

            # # Test summed inversion equation.
            # I = np.eye( 3 )
            # R = rot( x[3:6] )
            # g = np.trace( R@R )
            # IRn = I - 1/(1 + g)*R@R
            # Rlist[i,t+1] = np.linalg.norm( np.linalg.inv( I + R@R ) - IRn )

        # Calculate LC for each initial condition.
        Vlist[:,t+1] = lyapunovCandidate( Xlist[:,:,t+1] )

        # Print completed time-step.
        if ( (t+1)*dt ) % Tstep == 0:
            print( f"Time: {(t+1)*dt}" )

    # Label broken initial positions.
    slist = np.isfinite( np.sum( Xlist[:,:,-1], axis=0 ) )

    # Plot simulation results (2D).
    (fig1, fig2), (axs1, axs2) = plotPosVel2D( tlist, Xlist, slist=slist )

    # Plot simulation results (3D).
    fig3, axs3 = plotPosVel3D( Xlist, slist=slist )

    # Plot Lyapunov candidate function.
    fig4, axs4 = plotLyapunovCandidate( tlist, Xlist, Vlist, ilist, slist )

    # Plot rotation derivative error.
    fig5, axs5 = plotRotationError( tlist, Rlist )

    # Show generateed plots.
    figlist = plt.get_fignums()
    for i in figlist:
        fig = plt.figure(i)
        fig.tight_layout()
    plt.show()
