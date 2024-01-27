import numpy as np

def initrand(n=12, w=1, A=1):
    X0 = 2*A*np.random.rand( n,w ) - A
    return X0

def initzero(n=12, w=1):
    X0 = np.zeros( (n,w) )
    return X0

def initeven(n=12, w=1, A=1, ilist=None):
    ilist = [1] if ilist is None or isinstance(ilist, int) else ilist
    X0 = initzero( n,w )
    for i in ilist:
        assert i > 0, "ERROR: Even spacing initialization starts at x1."
        X0[i-1,:] = np.linspace( -A,A,w )
    return X0
