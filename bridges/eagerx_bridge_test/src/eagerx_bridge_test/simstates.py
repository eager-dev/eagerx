from typing import Optional
from eagerx_core.entities import SimState
import eagerx_core.registration as register
from std_msgs.msg import UInt64


class TestSimState(SimState):

    @staticmethod
    @register.spec('TestSimState', SimState)
    def spec(spec, test_arg: Optional[str] = 'test_argument'):
        # Initialize simstate
        spec.initialize(TestSimState)

        # Modify parameters based on arguments
        spec.set_parameter('test_arg', test_arg)
        return spec

    @register.simstate_params(test_arg='test')
    def initialize(self, test_arg):
        self.test_arg = test_arg

    def reset(self, state, done):
        # print('INSIDE SIMSTATE RESET: (%s, %s)' % (state, done))
        return
