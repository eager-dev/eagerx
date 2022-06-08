# OTHER
from typing import Optional
import gym

# ROS IMPORTS
import rospy
from gym.spaces import Discrete

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.core.entities import Engine
from eagerx.core.specs import EngineSpec


class GymEngine(Engine):
    def initialize(self):
        self.simulator = dict()

    @staticmethod
    @register.spec("GymEngine", Engine)
    def spec(
        spec: EngineSpec,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        sync: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = ERROR,
    ):
        """TestEngine spec"""
        # Modify default engine params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.sync = sync
        spec.config.real_time_factor = real_time_factor
        spec.config.simulate_delays = simulate_delays
        spec.config.log_level = log_level
        spec.config.color = "magenta"

    @register.engine_config(env_id=None)
    def add_object(self, config, engine_config, node_params, state_params):
        """
        Adds an object whose dynamics are governed by a registered OpenAI gym environment.

        :param config: The (agnostic) config of the :class:`~eagerx.core.entities.Object` that is to be added.
        :param engine_config: The engine-specific config of the :class:`~eagerx.core.entities.Object` that is to be added.
                              This dict contains the registered parameters:

                              - **env_id**: A string ID of the OpenAI gym environment.
                                            See https://gym.openai.com/envs/#classic_control for all available flags.

        :param node_params: A list containing the config of every :class:`~eagerx.core.entities.EngineNode` that represents
                            an :class:`~eagerx.core.entities.Object`'s sensor or actuator that is to be added.
        :param state_params: A list containing the parameters of every the :class:`~eagerx.core.entities.Object`'s
                             :class:`~eagerx.core.entities.EngineState` that is to be added.
        """
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(f'Adding object "{config["name"]}" of type "{config["entity_id"]}" to the simulator.')

        # Extract relevant object_params
        obj_name = config["name"]
        id = engine_config["env_id"]

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(
            env=gym.make(id),
            buffer_obs=[],
            buffer_reward=None,
            buffer_done=None,
            next_action=None,
        )

    def pre_reset(self):
        pass

    @register.states()
    def reset(self):
        for _obj_name, sim in self.simulator.items():
            obs = sim["env"].reset()
            sim["buffer_obs"] = [obs]
            sim["buffer_reward"] = []
            sim["buffer_done"] = []

    def callback(self, t_n: float):
        for _obj_name, sim in self.simulator.items():
            next_action = sim["next_action"]
            obs, reward, is_done, _ = sim["env"].step(next_action)
            sim["buffer_obs"].append(obs)
            sim["buffer_reward"].append(reward)
            sim["buffer_done"].append(is_done)
