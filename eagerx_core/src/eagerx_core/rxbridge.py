#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt64

# Rx imports
from eagerx_core.rxnode import NodeBase
from eagerx_core.utils.utils import get_attribute_from_module, initialize_converter, initialize_state, get_ROS_log_level
from eagerx_core import get_param_with_blocking
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.converter import IdentityConverter
import eagerx_core

# Other imports
from typing import List, Union, Dict, Tuple, Callable
import abc

# Memory usage
from threading import Condition
import os, psutil


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
    def pre_reset(self, ticks):
        pass

    @abc.abstractmethod
    def reset(self):
        pass

    @abc.abstractmethod
    def callback(self, inputs):
        pass


class RxBridge(object):
    def __init__(self, name, message_broker):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        dt, inputs, outputs, node_names, target_addresses, self.bridge = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects = eagerx_core.init_bridge(self.ns, dt, self.bridge, inputs, outputs, node_names, target_addresses, self.mb)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)
        self.mb.add_rx_objects(node_name=name + '/dynamically_registered', node=self)
        self.mb.connect_io()
        self.cond_reg = Condition()

        # Prepare closing routine
        rospy.on_shutdown(self._close)

    def node_initialized(self):
        with self.cond_reg:
            # Wait for all nodes to be initialized
            wait_for_node_initialization(self.bridge.is_initialized)

            # Notify env that node is initialized
            if not self.initialized:
                init_pub = rospy.Publisher(self.name + '/initialized', UInt64, queue_size=0, latch=True)
                init_pub.publish(UInt64(data=1))
                rospy.loginfo('Node "%s" initialized.' % self.name)
                self.initialized = True

    def _prepare_io_topics(self, name):
        params = get_param_with_blocking(name)
        node_names = params['node_names']
        target_addresses = params['target_addresses']
        rate = params['rate']
        dt = 1 / rate

        # Get node
        node_cls = get_attribute_from_module(params['module'], params['node_type'])
        node = node_cls(ns=self.ns, message_broker=self.mb, **params)

        # Prepare input topics
        for i in params['inputs']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
            elif 'converter' not in i:
                i['converter'] = IdentityConverter()

        # Prepare output topics
        for i in params['outputs']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
            elif 'converter' not in i:
                i['converter'] = IdentityConverter()

        return dt, params['inputs'], tuple(params['outputs']), node_names, target_addresses, node

    def _close(self):
        return True


if __name__ == '__main__':

    log_level = get_ROS_log_level(rospy.get_name())

    rospy.init_node('rxbridge', log_level=eagerx_core.core.log_levels_ROS[log_level])

    message_broker = eagerx_core.RxMessageBroker(owner=rospy.get_name())

    pnode = RxBridge(name=rospy.get_name(), message_broker=message_broker)

    message_broker.connect_io()

    pnode.node_initialized()

    rospy.spin()


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

    def pre_reset(self, ticks):
        return 'PRE RESET RETURN VALUE'

    def reset(self):
        self.num_ticks = 0
        return 'POST RESET RETURN VALUE'

    def callback(self, inputs):
        # Verify that # of ticks equals internal counter
        node_tick = inputs['node_tick']
        if not self.num_ticks == node_tick:
            print('[%s]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in inputs:
                t_i = inputs[name]['t_i']
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
