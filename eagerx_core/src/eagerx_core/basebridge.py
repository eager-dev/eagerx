import abc
import os
from typing import Dict

import psutil
import rospy
from std_msgs.msg import UInt64

from eagerx_core.basenode import NodeBase
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.utils.utils import initialize_state


class BridgeBase(NodeBase):
    def __init__(self, simulator, target_addresses, node_names, **kwargs):
        self.simulator = simulator
        self.target_addresses = target_addresses
        self.node_names = node_names

        # Initialized nodes
        self.is_initialized = dict()

        super(BridgeBase, self).__init__(**kwargs)

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
        initialize_nodes(node_params, self.ns, self.ns, self.message_broker, self.is_initialized, sp_nodes, launch_nodes)
        for name, node in sp_nodes.items():
            # Set object parameters
            if hasattr(node.node, 'set_object_params'):
                node.node.set_object_params(params)
            # Set simulator
            if hasattr(node.node, 'set_simulator'):
                node.node.set_simulator(self.simulator)
            # Initialize
            node.node_initialized()
        wait_for_node_initialization(self.is_initialized)
        return state_params, sp_nodes, launch_nodes

    @abc.abstractmethod
    def add_object_to_simulator(self, object_params, node_params, state_params) -> Dict:
        pass

    @abc.abstractmethod
    def pre_reset(self):
        pass

    @abc.abstractmethod
    def reset(self):
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs):
        pass


###########################################################################
# Specific implementations ################################################
###########################################################################


class TestBridgeNode(BridgeBase):
    def __init__(self, num_substeps, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        simulator = None

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

    def pre_reset(self):
        return 'PRE RESET RETURN VALUE'

    def reset(self):
        self.num_ticks = 0
        return 'POST RESET RETURN VALUE'

    def callback(self, node_tick: int, t_n: float,  **kwargs):
        # Verify that # of ticks equals internal counter
        if not self.num_ticks == node_tick:
            print('[%s]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in kwargs:
                t_i = kwargs[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    print('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

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