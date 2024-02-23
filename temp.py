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
