from root import *

# linear controller gains
k = np.array( [
    [0.99, 0.80, 0.001],
    [0.99, 0.80, 0.001],
    [0.99, 0.80, 0.001] ] );

# model and cost functions
def linearize(x):
    x = x.reshape(cNx,)

    x1 = x[0];  x2 = x[1];
    x3 = x[2];  x4 = x[3];
    x5 = x[4];

    x6 = x[5];  x7 = x[6];
    x8 = x[7];  x9 = x[8];
    x10 = x[9];

    x11 = x[5];  x12 = x[6];
    x13 = x[7];  x14 = x[8];
    x15 = x[9];

    uList = control(x)[:,0];
    u1 = uList[0];
    u2 = uList[1];
    u3 = uList[2];

    Ax0 = np.hstack( (
        np.zeros( (dNx,dNx) ), np.eye(dNx), np.zeros( (dNx,dNx) ) ) );
    Ax1 = np.array( [
        [0, 0, -k[0], -np.sin(x4)*np.cos(x5)*u1, -np.cos(x4)*np.sin(x5)*u1, 0, 0, -k[1],     0,     0, 0, 0, -k[1],     0,     0],
        [0, 0, -k[0], -np.sin(x4)*np.cos(x5)*u1, -np.cos(x4)*np.sin(x5)*u1, 0, 0, -k[1],     0,     0, 0, 0, -k[1],     0,     0],
        [0, 0, -k[0],             np.cos(x4)*u1,                         0, 0, 0, -k[1],     0,     0, 0, 0, -k[1],     0,     0],
        [0, 0,     0,                     -k[0],                         0, 0, 0,     0, -k[1],     0, 0, 0,     0, -k[1],     0],
        [0, 0,     0,                         0,                     -k[0], 0, 0,     0,     0, -k[1], 0, 0,     0,     0, -k[1]] ] );
    Ay  = np.hstack( (
        np.eye(dNx), np.zeros( (dNx,2*dNx) )) );

    Ax = np.vstack( (Ax0, Ax1, Ay) );

    return Ax;

def control(x):
    d = [m*g, 0, 0];

    u1 = d[0] - k[0,0]*x[2] - k[0,1]*x[7] - k[0,2]*x[12];
    u2 = d[1] - k[1,0]*x[3] - k[1,1]*x[8] - k[1,2]*x[13];
    u3 = d[2] - k[2,0]*x[4] - k[2,1]*x[9] - k[2,2]*x[14];

    u = np.vstack( (u1, u2, u3) );

    return u;

def noise(eps, shape=(1,1)):
    if len(shape) == 1:
        return 2*eps*np.random.rand(shape[0]) - eps;
    return 2*eps*np.random.rand(shape[0], shape[1]) - eps;

# pid simulation function (for MPC comparison)
def pidSimulation(tList, x0):
    # simulation time
    Nt = len(tList[0]);

    # main simulation loop
    x = x0;
    xList = np.empty( (cNx,Nt) );
    for i in range(Nt):
        x = x + dt*cmodel(x, control(x));
        xList[:,i] = x[:,0];

    return xList;

# main execution block
if __name__ == "__main__":
    # initial position w/ disturbance
    eps = 1;
    disturbance = [[(i in states[:,0])*eps] for i in range(cNx)];
    x0 = xd + disturbance;

    # simulation length
    T = 100;  Nt = round(T/dt) + 1;
    tList = [[i*dt for i in range(Nt)]];

    # execute simulation
    xList = pidSimulation(tList, x0);
    fig, axsList = plotTrajectories(labels, states, tList, xList);
    print( xList[:,-1] );
    plt.show();