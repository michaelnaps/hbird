from root import *

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

def linearized(x, u, xd):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    x1 = xd[0];  x2 = xd[1];
    x3 = xd[2];  x4 = xd[3];
    x5 = xd[4];

    n = 5;
    Ax0 = np.hstack( (np.zeros( (n,n) ), np.eye(n), np.zeros( (n,n) )) );
    Ay  = np.hstack( (np.eye(n), np.zeros( (n,2*n) )) );
    Ax1 = np.array( [
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1]
    ] );

    print(A);
    print(B);

    xplus = A@np.array(x)[:,None] + B@np.array(u)[:,None];

    return xplus.reshape(cNx,);

def control(x):
    d = [0, 0, 0];
    k = [0.95, 0.05, 0];

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