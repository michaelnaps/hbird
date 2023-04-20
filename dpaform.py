from root import *

# class for dynamic programming
class DynamicProgramming:
    def __init__(self, pcost, tcost, model, N, Nx, Nu,
        params=None, max_iter=10,
        appx_zero=1e-6, grad_step=1e-3):
        self.pcost = pcost;
        self.tcost = tcost;
        self.model = model;
        self.params = params;

        self.N = N;
        self.Nx = Nx;
        self.Nu = Nu;

        self.zero = appx_zero;
        self.step = grad_step;

        # if nmax is -1 -> no iteration cap
        self.nmax = max_iter;

        self._alpha = 1;

    def setAlpha(self, a):
        self._alpha = a;
        return;

    def forward(self, x0, uinit, N=None, output=0):
        # if N (prediction horizon) not given
        if N is None:
            # start alg. with class N
            return self.forward(x0, uinit, self.N, output);

        # if N = 1 end recursion by minimizing with terminal cost
        if N == 1:
            cost = lambda u: self.pcost(x0,u) + self.tcost( self.model(x0,u) );
            ustar, Jstar = self.minimize(cost, uinit);
            print(ustar);
            return ustar, Jstar, self.model( x0,ustar );

        cost = lambda u: self.pcost( x0,u ) + self.forward( self.model(x0, u), u, N=N-1 )[1];
        ustar, Jstar = self.minimize(cost, uinit);
        return ustar, Jstar;

    def minimize(self, cost, uinit, h=1e-3):
        un = [uinit[i] for i in range(self.Nu)];
        gn = self.fdm(cost, un, h=self.step);
        gnorm = sum([gn[i]**2 for i in range(self.Nu)]);

        # gradient descent loop
        count = 1;
        while gnorm > self.zero**2:
            un = [un[i] - self._alpha*gn[i] for i in range(self.Nu)];
            gn = self.fdm(cost, un, h=self.step);
            gnorm = sum([gn[i]**2 for i in range(self.Nu)]);

            count += 1;
            if (self.nmax != -1) and (count > self.nmax-1):  # iteration limit
                print('WARNING: Iteration break in DynamicProgramming.minimize()');
                break;

        return un, cost(un);

    def fdm(self, cost, u, h=1e-3):
        dJ = [0 for i in range(self.Nu)];

        for i in range(self.Nu):
            un1 = [u[j] - (i==j)*h for j in range(self.Nu)];
            up1 = [u[j] + (i==j)*h for j in range(self.Nu)];

            Jn1 = cost(un1);
            Jp1 = cost(up1);

            dJ[i] = (Jp1 - Jn1)/(2*h);

        return dJ;

# model and cost functions
def model(x, u):
    F     = u[0];
    TauZ  = u[1];
    TauXY = u[2];

    xplus = [
        x[0] + dt/m*F*math.sin(x[3])*math.cos(x[4]),
        x[1] + dt/m*F*math.sin(x[3])*math.sin(x[4]),
        x[2] + dt/m*(F*math.cos(x[3]) - m*g),
        x[3] + dt*TauZ,
        x[4] + dt*TauXY
    ];

    return xplus;

def pcost(x, u):
    kx = 100;
    ku = 0;
    J = sum( [kx*x[i]**2 for i in range(dNx)] );
    J += sum( [ku*u[i]**2 for i in range(Nu)] );
    return J;

def tcost(xN):
    gN = sum( [xN[i]**2 for i in range(dNx)] );
    return gN;

# main execution block
if __name__ == "__main__":
    # initialize starting and goal states
    xd = [0,0,0,0,0];
    x0 = [0,0,0,0,0];

    # create MPC class variable
    N = 2;
    model_type = 'discrete';
    vhc = Vehicle(x0, xd);
    dpvar = DynamicProgramming(pcost, tcost, model, N, dNx, Nu,
        params=vhc, max_iter=-1);
    dpvar.setAlpha(0.9);

    # test DPA
    uinit = [0 for i in range(Nu)]
    cost = lambda u: pcost(x0,u) + tcost( model(x0,u) );
    ustar, Jstar = dpvar.forward(x0, uinit);

    print(ustar, Jstar);
    print(model(x0, ustar));


    # T = np.array( sim_results[0] );
    # xlist = np.array( sim_results[1] );
    # ulist = np.array( sim_results[2] );

    # # plot results
    # fig, axs = plt.subplots(3,1);
    # axs[0].plot(T, xlist[:,0]);
    # axs[0].set_ylim( (-1,1) );

    # axs[1].plot(T, xlist[:,1]);
    # axs[1].set_ylim( (-1,1) );

    # axs[2].plot(T, xlist[:,2]);
    # # axs[1].set_ylim( (-1,1) );

    # plt.show();