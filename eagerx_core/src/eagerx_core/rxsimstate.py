import abc


class SimStateBase(object):
    def __init__(self, ns, name, simulator, color='grey', print_mode='termcolor'):
        self.ns = ns
        self.name = name

        # If node is simulator, we will probably use this in reset
        self.simulator = simulator
        self.color = color
        self.print_mode = print_mode

    @abc.abstractmethod
    def reset(self, state, done):
        pass
        # print('INSIDE SIMSTATE RESET: (%s, %s)' % (state, done))
        return None

###########################################################################
# Specific implementations ################################################
###########################################################################


class TestSimState(SimStateBase):
    def __init__(self, test_arg, **kwargs):
        super().__init__(**kwargs)

    def reset(self, state, done):
        # print('INSIDE SIMSTATE RESET: (%s, %s)' % (state, done))
        return None