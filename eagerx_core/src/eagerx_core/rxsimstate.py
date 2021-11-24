class RxSimState(object):
    def __init__(self, name, simulator, test_arg):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])

        # If node is simulator, we will probably use this in reset
        self.simulator = simulator

    def reset(self, state, done):
        # print('INSIDE SIMSTATE RESET: (%s, %s)' % (state, done))
        return None
