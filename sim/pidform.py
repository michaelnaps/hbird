from root import *

if __name__ == '__main__':
    x = np.zeros( (12,1) )
    u = np.zeros( (4,1) )
    for x, y in zip( x, model(x, u) ):
        print( x, y )