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
    for i, x in enumerate( X.T ):
        V[:,i] = x[:,None].T@x[:,None]
    return V

if __name__ == '__main__':
    # Simulation length.
    T = 10;  Nt = round( T/dt ) + 1
    tlist = np.array( [i*dt for i in range( Nt )] )

    # Date set initialization.
    A = np.pi;  ilist = [1,5]
    Xlist, w = initmesh( n, w, Nt, A, ilist )

    # Candidate function initialization.
    Vlist = np.empty( (w,Nt) )
    Vlist[:,0] = lyapunovCandidate( Xlist[:,:,0] )

    # Simulation block.
    Tstep = 1
    for t in range( Nt-1 ):
        for i, x in enumerate( Xlist[:,:,t].T ):
            # TODO: Simplify this if-else check.
            if not np.isfinite( x[0] ):
                continue
            if lyapunovCandidate( x[:,None] ) > TOL:
                Xlist[:,i,t+1:] = np.inf*np.ones( (n,Nt-(t+1)) )
                continue
            xn = model( x[:,None], control( x[:,None] ) )
            Xlist[:,i,t+1] = xn[:,0]
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

    # Show generateed plots.
    plt.show()
