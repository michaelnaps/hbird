from root import *
from initial import *

w = 10

k = -10.0
c = -2.50
h = 1/-k
r = 1/-c

TOL = 50e3

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
def lyapunovCandidate(x):
    V = x.T@x
    return V

if __name__ == '__main__':
    # Simulation length.
    T = 10;  Nt = round( T/dt ) + 1
    tlist = np.array( [i*dt for i in range( Nt )] )

    # Date set initialization.
    A = 4*np.pi
    Xlist = np.empty( (n,w,Nt) )
    Xlist[:,:,0] = initeven( n,w,A,[1,5] )

    # Simulation block.
    T = 1
    for t in range( Nt-1 ):
        for i, x in enumerate( Xlist[:,:,t].T ):
            if lyapunovCandidate( x[:,None] ) > TOL:
                Xlist[:,i,t+1] = np.inf*np.ones( (n,) )
                continue
            xn = model( x[:,None], control( x[:,None] ) )
            Xlist[:,i,t+1] = xn[:,0]

        # Print completed time-step.
        if ( (t+1)*dt ) % T == 0:
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
                color='indianred', linestyle='--' )
            for j in range( w ):
                axs.plot( tlist, Xlist[i,j] )
            i = i + 1
    for axsrow in axsvel:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                color='indianred', linestyle='--' )
            for j in range( w ):
                axs.plot( tlist, Xlist[i,j] )
            i = i + 1

    # Plot simulation results (3D).
    fig3 = plt.figure()
    # datalists = [Xlist[(k-3):k] for k in range( 3,n+1,3 )]
    for i, k in enumerate( range( 3,n+1,3 ) ):
        X = Xlist[(k-3):k]
        axs = fig3.add_subplot( 2, 2, i+1, projection='3d' )
        for j in range( w ):
            axs.plot( X[0,j], X[1,j], X[2,j] )
            axs.plot( X[0,j,-1], X[1,j,-1], X[2,j,-1],
                marker='x', color='indianred' )
        axs.set_xlabel( '$x_{%i}$'%(k-2) )
        axs.set_ylabel( '$x_{%i}$'%(k-1) )
        axs.set_zlabel( '$x_{%i}$'%(k-0) )
        axs.axis( 'equal' )

    # Show generateed plots.
    plt.show()
