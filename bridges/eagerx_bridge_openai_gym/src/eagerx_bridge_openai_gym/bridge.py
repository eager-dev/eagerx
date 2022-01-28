# OTHER
from typing import Optional, Dict, Union, List
import gym

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx_core.utils.utils import Msg
from eagerx_core.core.entities import Bridge


class GymBridge(Bridge):
    msg_types = {'outputs': {'tick': UInt64},
                 'states': {}}

    def __init__(self, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        simulator = dict()
        super().__init__(simulator=simulator, **kwargs)

    def add_object(self, object_params, node_params, state_params) -> Dict:
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo('Adding object "%s" of type "%s.yaml" from package "%s" to the simulator.' % (
        object_params['name'], object_params['config_name'], object_params['package_name']))

        # Extract relevant object_params
        obj_name = object_params['name']
        id = object_params['gym_id']

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(env=gym.make(id), buffer_obs=[], buffer_reward=None, buffer_done=None, next_action=None)
        return object_params

    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    def reset(self, **kwargs: Optional[Msg]):
        for obj_name, sim in self.simulator.items():
            obs = sim['env'].reset()
            sim['buffer_obs'] = [obs]
            sim['buffer_reward'] = []
            sim['buffer_done'] = []

    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        for obj_name, sim in self.simulator.items():
            next_action = sim['next_action']
            obs, reward, is_done, _ = sim['env'].step(next_action)
            sim['buffer_obs'].append(obs)
            sim['buffer_reward'].append(reward)
            sim['buffer_done'].append(is_done)
