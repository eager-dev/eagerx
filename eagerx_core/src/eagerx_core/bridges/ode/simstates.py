import numpy as np
from eagerx_core.core.entities import SimState
import eagerx_core.core.registration as register


class OdeSimState(SimState):
    @staticmethod
    @register.spec('OdeSimState', SimState)
    def spec(spec):
        pass

    def initialize(self):
        self.obj_name = self.agnostic_params['name']

    def reset(self, state, done):
        self.simulator[self.obj_name]['state'] = np.squeeze(state.data)
