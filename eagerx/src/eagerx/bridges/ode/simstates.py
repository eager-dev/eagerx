import numpy as np
from eagerx.core.entities import EngineState
import eagerx.core.register as register


class OdeEngineState(EngineState):
    @staticmethod
    @register.spec('OdeSimState', EngineState)
    def spec(spec):
        pass

    def initialize(self):
        self.obj_name = self.agnostic_params['name']

    def reset(self, state, done):
        self.simulator[self.obj_name]['state'] = np.squeeze(state.data)
