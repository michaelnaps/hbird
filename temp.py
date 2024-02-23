from root import *

def powR(R, n):
    if n == 0:
        return np.eye( R.shape[0], R.shape[1] )
    W = np.eye( R.shape[0] )
    for _ in range( n ):
        W = W@R
    return W

def invert(R, n):
    IRn = np.zeros( R.shape )
    for i in range( n+1 ):
        IRn = IRn + (-1)**i*powR(R, i)
    return IRn

if __name__ == '__main__':
    A = 2*np.pi
    x = 2*A*np.random.rand( 3,1 ) - A

    R = rot( x )
    RR = R@R

    N = 10
    Rlast = np.zeros( R.shape )
    for n in range( N ):
        Rnew = invert( RR, n )
        print( np.linalg.norm( Rnew - Rlast ) )
        Rlast = Rnew

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
