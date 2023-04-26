from root import *

# linear controller gains
k = np.array( [
    [10.0, 8.00, 0.010],
    [10.0, 8.00, 0.010],
    [10.0, 8.00, 0.010] ] );

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

# pid simulation function (for MPC comparison)
def pidSimulation(sim_time, x0):
    # simulation time
    Nt = round(sim_time/dtpid) + 1;
    tList = [ [i*dtpid for i in range(Nt)] ];

    # main simulation loop
    xList = np.empty( (cNx,Nt) );
    uList = np.empty( (Nu,Nt-1) );

    x = x0;
    xList[:,0] = x[:,0];
    for i in range(Nt-1):
        u = control(x);
        x = x + dtpid*cmodel(x, u);
        xList[:,i+1] = x[:,0];
        uList[:,i] = u[:,0];

    print('PID Complete.');
    return tList, xList, uList;

# main execution block
if __name__ == "__main__":
    # initial position w/ disturbance
    eps = 1.0;
    disturbList = (0,1,2,3,4);
    disturbance = [[noise(eps)*(i in disturbList)] for i in range(cNx)];
    x0 = xd + disturbance;

    # simulation length
    sim_time = 10;

    # execute simulation
    tList, xList, uList = pidSimulation(sim_time, x0);
    fig, axsList = plotTrajectories(tList, xList, xd, legend='PID');
    plt.show();