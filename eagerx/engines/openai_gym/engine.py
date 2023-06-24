# OTHER
from typing import Optional
import gymnasium as gym

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.core.entities import Engine


class GymEngine(Engine):
    @classmethod
    def make(
        cls,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        sync: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = ERROR,
    ):
        """TestEngine spec"""
        spec = cls.get_specification()

        # Modify default engine params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.sync = sync
        spec.config.real_time_factor = real_time_factor
        spec.config.simulate_delays = simulate_delays
        spec.config.log_level = log_level
        spec.config.color = "magenta"
        return spec

    def initialize(self, spec):
        self.simulator = dict()

    def add_object(self, name: str, env_id: str, seed: int = None):
        """
        Adds an object whose dynamics are governed by a registered OpenAI gym environment.

        :param name: Name of the :class:`~eagerx.core.entities.Object` that is to be added.
        :param env_id: Gym id of the environment.
        :param seed: Seed for the environment.
        """
        # add object to simulator (we have a ref to the simulator with self.simulator)
        self.backend.loginfo(f'Adding object "{name}" to the simulator.')

        # Create new env, and add to simulator
        env = gym.make(env_id, render_mode="rgb_array")
        if seed is not None:
            env.seed(seed)
        self.simulator[name].update(
            env_id=env_id,
            env=env,
            buffer_obs=[],
            buffer_reward=None,
            buffer_terminated=None,
            buffer_truncated=None,
            next_action=None,
        )

    def pre_reset(self):
        pass

    @register.states()
    def reset(self):
        for _obj_name, sim in self.simulator.items():
            obs, info = sim["env"].reset()
            sim["buffer_obs"] = [obs]
            sim["buffer_reward"] = []
            sim["buffer_terminated"] = []
            sim["buffer_truncated"] = []

    def callback(self, t_n: float):
        for _obj_name, sim in self.simulator.items():
            next_action = sim["next_action"]
            obs, reward, is_terminated, truncated, _ = sim["env"].step(next_action)
            sim["buffer_obs"].append(obs)
            sim["buffer_reward"].append(reward)
            sim["buffer_terminated"].append(is_terminated)
            sim["buffer_truncated"].append(truncated)
