import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');


import mpc
from numpy import random as rd

import math
import matplotlib.pyplot as plt

def model(x, u):
    F     = u[0];
    tauZ  = u[1];
    tauXY = u[2];

    dx = [
        x[5],
        x[6],
        x[7],
        x[8],
        x[9],
        1/m*F*math.cos(x[3])*math.cos(x[4]),
        1/m*F*math.cos(x[3])*math.sin(x[4]),
        1/m*(F*math.sin(x[3]) - m*g),
        tauZ,
        tauXY
    ];

    return dx;
