from eagerx_core.basesimstate import SimStateBase
import numpy as np


class PendulumState(SimStateBase):
    def __init__(self, use_sampled_state, **kwargs):
        self.use_sampled_state = use_sampled_state
        super().__init__(**kwargs)

    def reset(self, state, done):
        assert isinstance(self.simulator, dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator
        if not done:
            if self.use_sampled_state:
                _ = self.simulator['env'].reset()
                self.simulator['env'].state = np.array(state.data, dtype='float32')
                obs = self.simulator['env']._get_obs()
            else:
                obs = self.simulator['env'].reset()
        else:
            state = self.simulator['env'].state
            self.simulator['env'].reset()
            self.simulator['env'].state = state
            obs = self.simulator['env']._get_obs()
        self.simulator['last_obs'] = obs
