from root import *

k = -10.0
c = -2.50
h = 1/5.00
r = 1/1.00 # -1/250

def control(x):
    u = np.array( [
        (k*x[2] + c*x[8] - M*g),
        (k*x[3] + c*x[9]) + (h*x[1] + r*x[7]),
        (k*x[4] + c*x[10]) + (h*x[0] + r*x[6]),
        (k*x[5] + c*x[11])
    ] )
    return u

if __name__ == '__main__':
    # Simulation length.
    T = 30;  Nt = round( T/dt ) + 1
    tlist = np.array( [i*dt for i in range( Nt )] )

    # Date set initialization.
    # A = np.pi/2
    Xlist = np.empty( (n,Nt) )
    # Xlist[:,0] = np.array( [float(i==4) for i in range( n )] )
    Xlist[:,0] = np.ones( (n,) )

    # Simulation block.
    for i in range( Nt-1 ):
        x = Xlist[:,i,None]
        Xlist[:,i+1] = model( x, control( x ) )[:,0]

    # Plot simulation.
    fig, axspos = plt.subplots( 2,3 )
    fig, axsvel = plt.subplots( 2,3 )
    i = 0
    for axsrow in axspos:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                color='indianred', linestyle='--' )
            axs.plot( tlist, Xlist[i] )
            i = i + 1
    for axsrow in axsvel:
        for axs in axsrow:
            axs.plot( [tlist[0], tlist[-1]], [0,0],
                color='indianred', linestyle='--' )
            axs.plot( tlist, Xlist[i] )
            i = i + 1
    plt.show()
