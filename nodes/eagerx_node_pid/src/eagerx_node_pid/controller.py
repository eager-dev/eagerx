from collections import deque


class PID:
    def __init__(self, u0, kp, kd, ki, dt):
        self.u0 = u0
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt

        self.F = [kp + ki*dt + kd/dt,
                  -kp - 2*kd/dt,
                  kd/dt]

        self.window = None
        self.u = None

    def reset(self):
        self.u = self.u0
        self.window = deque(maxlen=3)

    def next_action(self, y, ref=0):
        # Add error
        self.window.appendleft(ref-y)

        # Calculate action
        for idx, e in enumerate(self.window):
            self.u += self.F[idx] * e

        return self.u


controller = PID(u0=0, kp=8, kd=1, ki=0, dt=0.05)

