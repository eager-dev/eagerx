from eagerx_core.basesimstate import SimStateBase
import numpy as np


class PendulumState(SimStateBase):
    def __init__(self, use_sampled_state, **kwargs):
        self.use_sampled_state = use_sampled_state
        self.obj_name = kwargs['object_params']['name']
        super().__init__(**kwargs)

    def reset(self, state, done):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        if not done:
            if self.use_sampled_state:
                self.simulator[self.obj_name]['env'].state = np.array(state.data, dtype='float32')
                obs = self.simulator[self.obj_name]['env'].unwrapped._get_obs()
            else:
                obs = self.simulator[self.obj_name]['env'].reset()
        else:
            state = self.simulator[self.obj_name]['env'].state
            self.simulator[self.obj_name]['env'].reset()
            self.simulator[self.obj_name]['env'].state = state
            obs = self.simulator[self.obj_name]['env']._get_obs()
        self.simulator[self.obj_name]['last_obs'] = obs
