from root import *

k = -np.array( [10, 2.5] )

def control(x):
    u = np.array( [
        k[0]*x[2] + k[1]*x[8],
        k[0]*x[3] + k[1]*x[9],
        k[0]*x[4] + k[1]*x[10],
        k[0]*x[5] + k[1]*x[11]
    ] )
    return u

if __name__ == '__main__':
    # Simulation length.
    Nt = round( 60/dt ) + 1
    tlist = np.array( [i*dt for i in range( Nt )] )

    # Date set initialization.
    Xlist = np.empty( (n,Nt) )
    Xlist[:,0] = np.ones( (n,) )

    # Simulation block.
    for i in range( Nt-1 ):
        x = Xlist[:,i,None]
        Xlist[:,i+1] = model( x, control( x ) )[:,0]

    # Plot simulation.
    fig, axspos = plt.subplots( 3,2 )
    fig, axsvel = plt.subplots( 3,2 )
    i = 0
    for axsrow in axspos:
        for axs in axsrow:
            axs.plot( tlist, Xlist[i] )
            i = i + 1
    for axsrow in axsvel:
        for axs in axsrow:
            axs.plot( tlist, Xlist[i] )
            i = i + 1
    plt.show()
