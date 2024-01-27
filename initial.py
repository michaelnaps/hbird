import numpy as np

def initseries(X0, Nt=None):
    assert Nt is not None, "ERROR: Time series length cannot be None."
    n, w = X0.shape
    Xlist = np.empty( (n, w, Nt) )
    Xlist[:,:,0] = X0
    return Xlist

def initrand(n=12, w=1, Nt=None, A=1):
    X0 = 2*A*np.random.rand( n, w ) - A
    return initseries( X0, Nt ), w

def initzero(n=12, w=1, Nt=None):
    X0 = np.zeros( (n,w) )
    return initseries( X0, Nt ), w

def initeven(n=12, w=1, Nt=None, A=1, ilist=None):
    ilist = [1] if ilist is None or isinstance(ilist, int) else ilist
    X0 = initzero( n, w, Nt )[0][:,:,0]
    for i in ilist:
        assert i > 0, "ERROR: Even spacing initialization starts at x1."
        X0[i-1,:] = np.linspace( -A, A, w )
    return initseries( X0, Nt ), w

def initmesh(n=12, w=1, Nt=None, A=1, ilist=None):
    assert len( ilist ) == 2, "ERROR: Mesh function only takes two axes."
    # Initialize even and mesh sets.
    Xeven = initeven( n, w, Nt, A, ilist )[0][[i-1 for i in ilist],:,0]
    Xmesh = np.empty( (2, w*w) )
    # Iterate through to form mesh.
    j = 0
    for x in Xeven[0]:
        for y in Xeven[1]:
            Xmesh[:,j] = [x,y]
            j = j + 1
    # Initialize remainder of states and return.
    Xlist = initzero( n, w*w, Nt )[0]
    Xlist[[i-1 for i in ilist],:,0] = Xmesh
    return Xlist, w*w
