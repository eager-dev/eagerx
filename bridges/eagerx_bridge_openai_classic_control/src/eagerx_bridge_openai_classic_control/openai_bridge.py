# OTHER
from typing import Dict, List, Union
import gym
import numpy as np

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx_core.basebridge import BridgeBase


class OpenAIBridge(BridgeBase):
    def __init__(self, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        simulator = dict()
        super(OpenAIBridge, self).__init__(simulator=simulator, **kwargs)

    def add_object_to_simulator(self, object_params, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo('Adding object "%s" of type "%s.yaml" from package "%s" to the simulator.' % (object_params['name'], object_params['config_name'], object_params['package_name']))

        # Extract relevant object_params
        obj_name = object_params['name']
        id = object_params['bridge']['id']

        # Assert that dt of OpenAI environment is same as 1/rate of bridge
        assert (1 / self.rate) == object_params['bridge']['dt'], 'Cannnot register object "%s" because the dt of gym_env "%s" and bridge "%s" are not equal (%s vs %s).' % (obj_name, id, self.name, (1 / self.rate), object_params['bridge']['dt'])

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(env=gym.make(id), last_obs=None, last_reward=None, last_is_done=None, next_action=None)
        return object_params

    def pre_reset(self):
        for obj_name, sim in self.simulator.items():
            obs = sim['env'].reset()
            sim['last_obs'] = obs
            sim['next_action'] = None
            sim['last_reward'] = None
            sim['last_is_done'] = None
        return

    def reset(self):
        return

    def callback(self, node_tick: int, t_n: float,  **kwargs: Dict[str, Union[List[Message], float, int]]):
        for obj_name, sim in self.simulator.items():
            next_action = sim['next_action']
            obs, reward, is_done, _ = sim['env'].step(next_action)
            sim['last_obs'] = obs
            sim['last_reward'] = reward
            sim['last_is_done'] = is_done

        # Fill output msg with number of node ticks
        return dict(tick=UInt64(data=node_tick+1))