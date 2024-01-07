from root import *

def control(x):
    return np.zeros( (m, x.shape[1]) )

if __name__ == '__main__':
    x = np.zeros( (n,1) )
    x[6] = [1]
    u = np.zeros( (m,1) )
    for x, y in zip( x, model(x, u) ):
        print( x, y )