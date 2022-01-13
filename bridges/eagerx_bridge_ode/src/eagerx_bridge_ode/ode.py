from math import sin, exp


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

    b = b0 + a * exp(- x[1] * x[1] / (c * c))

    tau_gravity = m * g * l * sin(x[0])
    tau_damping = (b + K * K / R) * x[1]
    tau_motor = K * u / R

    return [x[1], (tau_gravity - tau_damping + tau_motor) / J]
