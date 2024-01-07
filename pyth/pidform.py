from root import *

# linear controller gains
k = np.array( [
    [10.0, 8.00, 0],
    [10.0, 8.00, 0],
    [10.0, 8.00, 0] ] )

def control(x):
    d = [m*g, 0, 0]

    F  = d[0] - k[0,0]*x[2] - k[0,1]*x[7] - k[0,2]*x[12]
    u2 = d[1] - k[1,0]*x[3] - k[1,1]*x[8] - k[1,2]*x[13]
    u3 = d[2] - k[2,0]*x[4] - k[2,1]*x[9] - k[2,2]*x[14]

    u = np.vstack( (u1, u2, u3) )

    return u

# pid simulation function (for MPC comparison)
def pidSimulation(sim_time, x0):
    # simulation time
    Nt = round(sim_time/dtpid) + 1
    tList = [ [i*dtpid for i in range(Nt)] ]

    # main simulation loop
    xList = np.empty( (cNx,Nt) )
    uList = np.empty( (Nu,Nt-1) )

    x = x0
    xList[:,0] = x[:,0]
    for i in range(Nt-1):
        u = control(x)
        x = x + dtpid*cmodel(x, u)
        xList[:,i+1] = x[:,0]
        uList[:,i] = u[:,0]

    print('PID Complete.')
    return tList, xList, uList

# main execution block
if __name__ == "__main__":
    # initial position w/ disturbance
    disturbList = (2,3,4)
    disturbance = [[eps*(i in disturbList)] for i in range(cNx)]
    x0 = xd + disturbance

    # simulation length
    sim_time = 5.0

    # execute simulation
    tList, xList, uList = pidSimulation(sim_time, x0)

    # stationary plot
    plotTrajectories(tList, xList, xd, limits=limits_upper, legend='PID')
    plt.show(block=0)

    # execute simulation
    dtsim = 0.05
    isim = round(dtsim/dtpid)

    tSim = [ [i*dtsim for i in range( round(sim_time/dtsim)+1 )] ]
    spEntity = StatePlots(tSim, xList[:,0], xd,
        color='royalblue', limits_upper=limits_upper, limits_lower=limits_lower)

    j = isim
    for t in tSim[0][1:]:
        spEntity.update(t, xList[:,j])
        j += isim