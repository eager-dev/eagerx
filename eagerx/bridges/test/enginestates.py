from typing import Optional
from eagerx.core.entities import EngineState
import eagerx.core.register as register


class TestEngineState(EngineState):
    @staticmethod
    @register.spec("TestEngineState", EngineState)
    def spec(spec, test_arg: Optional[str] = "test_argument"):
        # Initialize simstate
        spec.initialize(TestEngineState)

        # Modify parameters based on arguments
        spec.set_parameter("test_arg", test_arg)
        return spec

    def initialize(self, test_arg):
        self.test_arg = test_arg

    def reset(self, state, done):
        # print('INSIDE SIMSTATE RESET: (%s, %s)' % (state, done))
        return
