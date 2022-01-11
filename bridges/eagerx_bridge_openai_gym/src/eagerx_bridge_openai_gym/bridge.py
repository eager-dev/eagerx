# OTHER
from typing import Optional, Dict, Union, List
import gym

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx_core.utils.utils import Msg
from eagerx_core.basebridge import BridgeBase


class GymBridge(BridgeBase):
    msg_types = {'outputs': {'tick': UInt64},
                 'states': {}}

    def __init__(self, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        simulator = dict()
        super().__init__(simulator=simulator, **kwargs)

    def add_object_to_simulator(self, object_params, node_params, state_params) -> Dict:
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo('Adding object "%s" of type "%s.yaml" from package "%s" to the simulator.' % (
        object_params['name'], object_params['config_name'], object_params['package_name']))

        # Extract relevant object_params
        obj_name = object_params['name']
        id = object_params['gym_id']

        # Assert that rate of OpenAI gym environment is same as rate of bridge
        # todo: reconsider this assertion
        assert self.rate == object_params['rate'], 'Cannnot register object "%s" because the rate of gym_env "%s" and bridge "%s" are not equal (%s vs %s).' % (obj_name, id, self.name, self.rate, object_params['rate'])

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(env=gym.make(id), last_obs=None, last_reward=None, last_done=None, next_action=None)
        return object_params

    def pre_reset(self, **kwargs: Optional[Msg]):
        for obj_name, sim in self.simulator.items():
            obs = sim['env'].reset()
            sim['last_obs'] = obs
            sim['next_action'] = None
            sim['last_reward'] = None
            sim['last_done'] = None
        return

    def reset(self, **kwargs: Optional[Msg]):
        pass

    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        for obj_name, sim in self.simulator.items():
            next_action = sim['next_action']
            obs, reward, is_done, _ = sim['env'].step(next_action)
            sim['last_obs'] = obs
            sim['last_reward'] = reward
            sim['last_done'] = is_done

            # Fill output msg with number of node ticks
        return dict(tick=UInt64(data=node_tick + 1))
