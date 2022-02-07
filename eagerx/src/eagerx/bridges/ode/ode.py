from math import sin, exp


def eom_pendulum(x, _, u, J=0.000189, m=0.0563, g=9.81, l=0.0438, b0=0.000142, K=0.0503, R=9.84, c=1.50, a=0.00184):
    b = b0 + a * exp(- x[1] * x[1] / (c * c))
    ddx = (m * g * l * sin(x[0]) - x[1] * (b + K * K / R) + K * u / R) / J
    return [x[1], ddx]
