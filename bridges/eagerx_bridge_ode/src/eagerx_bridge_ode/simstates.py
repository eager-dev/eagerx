import numpy as np
from eagerx_core.core.entities import SimState


class OdeSimState(SimState):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.obj_name = self.object_params['name']

    def reset(self, state, done):
        self.simulator[self.obj_name]['state'] = np.squeeze(state.data)
        return None