from root import *

w = 10

k = -10.0
c = -2.50
h = 1/5.00
r = 1/1.00 # -1/250

# PD controller.
def control(X):
    U = np.array( [
        (k*X[2] + c*X[8] - M*g),
        (k*X[3] + c*X[9]) + (h*X[1] + r*X[7]),
        (k*X[4] + c*X[10]) + (h*X[0] + r*X[6]),
        (k*X[5] + c*X[11])
    ] )
    return U

if __name__ == '__main__':
    # Simulation length.
    T = 30;  Nt = round( T/dt ) + 1
    tlist = np.array( [i*dt for i in range( Nt )] )

    # Date set initialization.
    A = np.pi/2
    Xlist = np.empty( (n,w,Nt) )
    Xlist[:,:,0] = 2*A*np.random.rand( n,w ) - A

    # Simulation block.
    for i in range( Nt-1 ):
        x = Xlist[:,:,i]
        Xlist[:,:,i+1] = model( x, control( x ) )

    # Plot simulation results (2D).
    fig1, axspos = plt.subplots( 2,3 )
    fig2, axsvel = plt.subplots( 2,3 )
    i = 0
    for axsrow in axspos:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                color='indianred', linestyle='--' )
            for j in range( w ):
                axs.plot( tlist, Xlist[i,j,:] )
            i = i + 1
    for axsrow in axsvel:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                color='indianred', linestyle='--' )
            for j in range( w ):
                axs.plot( tlist, Xlist[i,j,:] )
            i = i + 1

    # Plot simulation results (3D).
    fig3 = plt.figure()
    datalists = [Xlist[(k-3):k] for k in range( 3,n+1,3 )]
    for i, X in enumerate( datalists ):
        axs = fig3.add_subplot( 2, 2, i+1, projection='3d' )
        for j in range( w ):
            axs.plot( X[0,j], X[1,j], X[2,j] )
            axs.plot( X[0,j,-1], X[1,j,-1], X[2,j,-1],
                marker='x', color='indianred' )

    # Show generateed plots.
    plt.show()
