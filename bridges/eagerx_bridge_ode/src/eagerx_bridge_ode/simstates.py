from eagerx_core.simstates import SimStateBase


class OdeSimState(SimStateBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.obj_name = self.object_params['name']

    def reset(self, state, done):
        self.simulator[self.obj_name]['state'] = state
        return None