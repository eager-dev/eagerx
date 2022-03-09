# OTHER
from typing import Optional, Dict, Union, List
import gym

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.core.entities import Bridge
from eagerx.core.specs import BridgeSpec


class GymBridge(Bridge):
    def initialize(self):
        self.simulator = dict()

    @staticmethod
    @register.spec("GymBridge", Bridge)
    def spec(
        spec: BridgeSpec,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        is_reactive: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = ERROR,
    ):
        """TestBridge spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(GymBridge)

        # Modify default bridge params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.is_reactive = is_reactive
        spec.config.real_time_factor = real_time_factor
        spec.config.simulate_delays = simulate_delays
        spec.config.log_level = log_level
        spec.config.color = "magenta"

    @register.bridge_config(env_id=None)
    def add_object(self, config, bridge_config, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(f'Adding object "{config["name"]}" of type "{config["entity_id"]}" to the simulator.')

        # Extract relevant object_params
        obj_name = config["name"]
        id = bridge_config["env_id"]

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

    @register.outputs(tick=UInt64)
    def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        for _obj_name, sim in self.simulator.items():
            next_action = sim["next_action"]
            obs, reward, is_done, _ = sim["env"].step(next_action)
            sim["buffer_obs"].append(obs)
            sim["buffer_reward"].append(reward)
            sim["buffer_done"].append(is_done)
