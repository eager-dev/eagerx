from eagerx_core.simstates import SimStateBase


class TestSimState(SimStateBase):
    def __init__(self, test_arg, **kwargs):
        super().__init__(**kwargs)

    def reset(self, state, done):
        # print('INSIDE SIMSTATE RESET: (%s, %s)' % (state, done))
        return None