from root import *
from initial import *

w = 15

k = -10.0
c = -2.50
h = 1/-k
r = 1/-c

TOL = np.inf

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
    A = 2*np.pi;  ilist = [1,5]
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
    fig1, axspos = plt.subplots( 2,3 )
    fig2, axsvel = plt.subplots( 2,3 )
    i = 0
    for axsrow in axspos:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                linestyle='--', color='indianred' )
            for j in range( w ):
                if slist[j]:
                    axs.plot( tlist, Xlist[i,j] )
            i = i + 1
    for axsrow in axsvel:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                linestyle='--', color='indianred' )
            for j in range( w ):
                if slist[j]:
                    axs.plot( tlist, Xlist[i,j] )
            i = i + 1

    # Plot simulation results (3D).
    fig3 = plt.figure()
    for i, k in enumerate( range( 3,n+1,3 ) ):
        X = Xlist[(k-3):k]
        axs = fig3.add_subplot( 2, 2, i+1, projection='3d' )
        for j in range( w ):
            if slist[j]:
                axs.plot( X[0,j], X[1,j], X[2,j] )
                axs.plot( X[0,j,-1], X[1,j,-1], X[2,j,-1],
                    marker='x', color='indianred' )
        axs.set_xlabel( '$x_{%i}$'%(k-2) )
        axs.set_ylabel( '$x_{%i}$'%(k-1) )
        axs.set_zlabel( '$x_{%i}$'%(k-0) )
        axs.axis( 'equal' )

    # Plot Lyapunov candidate function.
    fig4, axslcf = plt.subplots( 1,2 )
    axslcf[0].plot( tlist, Vlist.T )
    colors = ['cornflowerblue', 'indianred']
    for i, x0 in enumerate( Xlist[:,:,0].T ):
        axslcf[1].plot( x0[ilist[0]-1], x0[ilist[1]-1], marker='x',
            color=colors[0] if slist[i] else colors[1] )

    # Show generateed plots.
    plt.show()
