# OTHER
from typing import Optional, Dict, Union, List


# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx_core.utils.utils import Msg
from eagerx_core.utils.node_utils import launch_node
from eagerx_core.bridge import BridgeBase


class RealBridge(BridgeBase):
    msg_types = {'outputs': {'tick': UInt64}}

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

        if 'launch_file' in object_params['bridge']:
            launch_file = object_params['bridge']['launch_file']
            launch_args = object_params['bridge']['launch_args'] if 'launch_args' in object_params['bridge'] else []
            launch_node(launch_file, launch_args)

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(state=None, input=None)
        return object_params

    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    def reset(self, **kwargs: Optional[Msg]):
        pass

    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        # Fill output msg with number of node ticks
        return dict(tick=UInt64(data=node_tick + 1))
