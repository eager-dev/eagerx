# from numba import jit
from math import sin, exp


# @jit(nopython=True)
def mops_ode(x, t, u, J, m, l, b0, K, R, c, a):
    g = 9.81

    b = b0 + a * exp(- x[1] * x[1] / (c * c))

    ddx = (m * g * l * sin(x[0]) - x[1] * (b + K * K / R) + K * u / R) / J

    return [x[1], ddx]
