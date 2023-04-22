from root import *

# linear controller gains
k = [0.95, 0.05, 0];

# model and cost functions
def model(x, u):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    dx0 = np.array( [
        x[5],
        x[6],
        x[7],
        x[8],
        x[9]
    ] ).reshape(5,1);

    dx1 = np.array( [
        F*np.sin(x[3])*np.cos(x[4]),
        F*np.sin(x[3])*np.sin(x[4]),
        (F*np.cos(x[3]) - m*g),
        m*TauZ,
        m*TauXY
    ] ).reshape(5,1);

    dx2 = np.array( [
        x[0],
        x[1],
        x[2],
        x[3],
        x[4]
    ] ).reshape(5,1);

    dx = np.vstack( (dx0, dx1, dx2) );

    return dx;

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

    u1 = -k[0]*x3 - k[1]*x8 - k[2]*x13;
    u2 = -k[0]*x4 - k[1]*x9 - k[2]*x14;
    u3 = -k[0]*x5 - k[1]*x10 - k[2]*x15;

    Ax0 = np.hstack( (
        np.zeros( (cNx,cNx) ), np.eye(cNx), np.zeros( (cNx,cNx) ) ) );
    Ax1 = np.array( [
        [0, 0, -k[0], -np.sin(x4)*np.cos(x5)*u1, -np.cos(x4)*np.sin(x5)*u1, 0, 0, -k[1],     0,     0, 0, 0, -k[1],     0,     0],
        [0, 0, -k[0], -np.sin(x4)*np.cos(x5)*u1, -np.cos(x4)*np.sin(x5)*u1, 0, 0, -k[1],     0,     0, 0, 0, -k[1],     0,     0],
        [0, 0, -k[0],             np.cos(x4)*u1,                         0, 0, 0, -k[1],     0,     0, 0, 0, -k[1],     0,     0],
        [0, 0,     0,                     -k[0],                         0, 0, 0,     0, -k[1],     0, 0, 0,     0, -k[1],     0],
        [0, 0,     0,                         0,                     -k[0], 0, 0,     0,     0, -k[1], 0, 0,     0,     0, -k[1]] ] );
    Ay  = np.hstack( (
        np.eye(cNx), np.zeros( (cNx,2*cNx) )) );

    Ax = np.vstack( (Ax0, Ax1, Ay) );

    return Ax;

def control(x):
    d = [0, 0, 0];

    u1 = d[0] - k[0]*x[2] - k[1]*x[7] - k[2]*x[12];
    u2 = d[1] - k[0]*x[3] - k[1]*x[8] - k[2]*x[13];
    u3 = d[2] - k[0]*x[4] - k[1]*x[9] - k[2]*x[14];

    u = np.vstack( (u1, u2, u3) );

    return u;

def noise(eps, shape=(1,1)):
    if len(shape) == 1:
        return 2*eps*np.random.rand(shape[0]) - eps;
    return 2*eps*np.random.rand(shape[0], shape[1]) - eps;

# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = np.zeros( (cNx,1) );
    A0 = linearize( xd );
    print(A0);

    eps = 0.1;
    disturbance = noise(eps, xd.shape);
    x0 = xd + disturbance;
    # print(x0);

    # simulation
    T = 2.5;  Nt = round(T/dt) + 1;
    tlist = [[i*dt for i in range(Nt)]];

    # main simulation loop
    x = x0;
    xlist = np.empty( (cNx,Nt) );
    for i in range(Nt):
        x = model(x, control(x));
        xlist[:,i] = x[:,0];
        print(x);

    # plot results
    fig, axsList = plt.subplots(cNx,1);

    for i, axs in enumerate(axsList):
        axs.plot(tlist[0], xlist[i,:]);

    plt.show();