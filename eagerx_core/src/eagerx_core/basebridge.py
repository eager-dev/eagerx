# OTHER IMPORTS
import abc
import os
from typing import Dict, Union, List
import psutil

# ROS IMPORTS
import rospy
from genpy.message import Message
from std_msgs.msg import UInt64

# EAGERx IMPORTS
from eagerx_core.constants import process
from eagerx_core.basenode import NodeBase
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.utils.utils import initialize_state


class BridgeBase(NodeBase):
    def __init__(self, simulator, target_addresses, node_names, is_reactive, real_time_factor, **kwargs):
        self.simulator = simulator
        self.target_addresses = target_addresses
        self.node_names = node_names
        self.is_reactive = is_reactive
        self.real_time_factor = real_time_factor

        # Check real_time_factor & reactive args
        assert is_reactive or (not is_reactive and real_time_factor > 0), 'Cannot have a real_time_factor=0 while not reactive. Will result in synchronization issues. Set is_reactive=True or real_time_factor > 0'

        # Initialized nodes
        self.is_initialized = dict()

        super(BridgeBase, self).__init__(is_reactive=is_reactive, real_time_factor=real_time_factor, **kwargs)

    def register_node(self, node_params):
        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        initialize_nodes(node_params, process.BRIDGE, self.ns, self.ns, self.message_broker, self.is_initialized, sp_nodes, launch_nodes)
        for name, node in sp_nodes.items():
            # Set simulator
            if hasattr(node.node, 'set_simulator'):
                node.node.set_simulator(self.simulator)
            # Initialize
            node.node_initialized()
        wait_for_node_initialization(self.is_initialized)
        return node_params, sp_nodes, launch_nodes

    def register_object(self, object_params, node_params, state_params):
        # Use obj_params to initialize object in simulator --> object info parameter dict is optionally added to simulation nodes
        params = self.add_object_to_simulator(object_params, node_params, state_params)

        # Initialize states
        for i in state_params:
            i['state']['name'] = i['name']
            i['state']['simulator'] = self.simulator
            i['state']['ns'] = self.ns
            i['state'] = initialize_state(i['state'])

        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        node_args = dict(object_params=params, simulator=self.simulator)
        initialize_nodes(node_params, process.BRIDGE, self.ns, self.ns, self.message_broker, self.is_initialized, sp_nodes, launch_nodes, node_args=node_args)
        for name, node in sp_nodes.items():
            # Initialize
            node.node_initialized()
        wait_for_node_initialization(self.is_initialized)
        return state_params, sp_nodes, launch_nodes

    def pre_reset_cb(self, **kwargs):
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg['done']:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg['msg'][0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.pre_reset(**kwargs)

    def reset_cb(self, **kwargs):
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg['done']:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg['msg'][0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.reset(**kwargs)

    def callback_cb(self, node_tick, t_n, **kwargs):
        # todo: reactive=True & real_time_factor!= 0: rospy.rate behavior --> implement with rospy.sleep()?
        return self.callback(node_tick, t_n, **kwargs)

    @abc.abstractmethod
    def add_object_to_simulator(self, object_params, node_params, state_params) -> Dict:
        pass

    @abc.abstractmethod
    def pre_reset(self, **kwargs: Message):
        pass

    @abc.abstractmethod
    def reset(self, **kwargs: Message):
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        pass


###########################################################################
# Specific implementations ################################################
###########################################################################


class TestBridgeNode(BridgeBase):
    def __init__(self, num_substeps, nonreactive_address, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        simulator = None

        # If real_time bridge, assert that real_time_factor == 1 & is_reactive=False.

        # Initialize nonreactive input
        self.nonreactive_pub = rospy.Publisher(kwargs['ns'] + nonreactive_address, UInt64, queue_size=0, latch=True)

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20
        super(TestBridgeNode, self).__init__(simulator=simulator, **kwargs)

    def add_object_to_simulator(self, object_params, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo('Adding object "%s" of type "%s.yaml" from package "%s" to the simulator.' % (object_params['name'], object_params['config_name'], object_params['package_name']))
        return object_params

    def pre_reset(self, param_1: UInt64 = None):
        return 'PRE RESET RETURN VALUE'

    def reset(self, param_1: UInt64 = None):
        # Publish nonreactive input (this is only required for simulation setup)
        self.num_ticks = 0
        self.nonreactive_pub.publish(UInt64(data=0))
        return 'POST RESET RETURN VALUE'

    def callback(self, node_tick: int, t_n: float,  **kwargs: Dict[str, Union[List[Message], float, int]]):
        # Publish nonreactive input
        self.nonreactive_pub.publish(UInt64(data=node_tick))
        self.nonreactive_pub.publish(UInt64(data=node_tick*2))

        # Verify that # of ticks equals internal counter
        if not self.num_ticks == node_tick:
            rospy.logerr('[%s][callback]: ticks not equal (self.num_ticks=%d, node_tick=%d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in kwargs:
                t_i = kwargs[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks + 1
        for i in self.outputs:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs