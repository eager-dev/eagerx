from numba import jit
from math import sin, exp


@jit(nopython=True)
def eom_pendulum(x, t, u):
    J = 0.000189238
    m = 0.0563641
    g = 9.81
    l = 0.0437891
    b0 = 0.000142205
    K = 0.0502769
    R = 9.83536
    c = 1.49553
    a = 0.00183742

    b = b0 + a * exp(-x[1] * x[1] / (c * c))

    ddx = (m * g * l * sin(x[0]) - x[1] * (b + K * K / R) + K * u / R) / J

    return [x[1], ddx]
