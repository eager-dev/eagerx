#!/usr/bin/env python3

# ROS imports
import rospy
import logging
from std_msgs.msg import UInt64

# Rx imports
from eagerx_core.utils.utils import get_attribute_from_module, initialize_converter, get_ROS_log_level
from eagerx_core import get_param_with_blocking
from eagerx_core.converter import IdentityConverter
import eagerx_core

from eagerx_core.core import DEBUG, TERMCOLOR, INFO, ROS, ERROR

# Other imports
import abc

# Memory usage
import os, psutil, time
import numpy as np


class NodeBase:
    def __init__(self, ns, message_broker, name, config_name, package_name, node_type, module, rate, launch_locally, single_process,
                 inputs, outputs, states, feedthroughs,  targets, launch_file=None,
                 color='grey', print_mode=TERMCOLOR, log_level=ERROR):
        self.ns = ns
        self.name = name
        self.ns_name = '%s/%s' % (ns, name)
        self.message_broker = message_broker
        self.config_name = config_name
        self.package_name = package_name
        self.node_type = node_type
        self.module = module
        self.rate = rate
        self.launch_locally = launch_locally
        self.single_process = single_process
        self.launch_file = launch_file
        self.inputs = inputs
        self.outputs = outputs
        self.states = states
        self.feedthroughs = feedthroughs
        self.targets = targets
        self.color = color
        self.print_mode = print_mode
        self.log_level = log_level


class Node(NodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @abc.abstractmethod
    def reset(self, ticks):
        pass

    @abc.abstractmethod
    def callback(self, inputs):
        pass


class SimNode(Node):
    def __init__(self, **kwargs):
        self.simulator = None
        self.object_params = None
        super().__init__(**kwargs)

    def set_object_params(self, object_params):
        self.object_params = object_params

    def set_simulator(self, simulator):
        self.simulator = simulator

    @abc.abstractmethod
    def reset(self, ticks):
        pass

    @abc.abstractmethod
    def callback(self, inputs):
        pass


class ResetNode(NodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @abc.abstractmethod
    def reset(self, ticks):
        pass

    @abc.abstractmethod
    def callback(self, inputs, targets):
        pass


class ObservationsNode(Node):
    def __init__(self, inputs, **kwargs):
        # Define observation buffers
        self.observation_buffer = dict()
        for i in inputs:
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.observation_buffer[i['name']] = {'msg': None, 'converter': converter}
        super().__init__(inputs=inputs, **kwargs)

    def reset(self, ticks):
        # Set all messages to None
        for name, buffer in self.observation_buffer.items():
            buffer['msg'] = None
        return ticks

    def callback(self, inputs):
        # Set all observations to messages in inputs
        for name, buffer in self.observation_buffer.items():
            buffer['msg'] = inputs[name]['msg']

        # Send output_msg
        output_msgs = dict(set=UInt64())
        return output_msgs


class ActionsNode(Node):
    def __init__(self, outputs, **kwargs):
        # Define action/observation buffers
        self.action_buffer = dict()
        for i in outputs:
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.action_buffer[i['name']] = {'msg': None, 'converter': converter}
        super().__init__(outputs=outputs, **kwargs)

    def reset(self, ticks):
        # Set all messages to None
        for name, buffer in self.action_buffer.items():
            buffer['msg'] = None
        return ticks

    def callback(self, inputs):
        # Fill output_msg with buffered actions
        output_msgs = dict()
        for name, buffer in self.action_buffer.items():
            output_msgs[name] = buffer['msg']
        return output_msgs


class RxNode(object):
    def __init__(self, name, message_broker):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        dt, inputs, outputs, feedthroughs, states, targets, self.node = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects = eagerx_core.init_node(self.ns, dt, self.node, inputs, outputs, feedthrough=feedthroughs,
                                           state_inputs=states, targets=targets)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

    def node_initialized(self):
        # Notify env that node is initialized
        init_pub = rospy.Publisher(self.name + '/initialized', UInt64, queue_size=0, latch=True)
        init_pub.publish(UInt64())

        if not self.initialized:
            rospy.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name):
        params = get_param_with_blocking(name)
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

        # Prepare state topics
        for i in params['states']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
            elif 'converter' not in i:
                i['converter'] = IdentityConverter()

        # Prepare target topics
        for i in params['targets']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
            elif 'converter' not in i:
                i['converter'] = IdentityConverter()

        # Prepare feedthrough topics
        for i in params['feedthroughs']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
            elif 'converter' not in i:
                i['converter'] = IdentityConverter()

        return dt, tuple(params['inputs']), tuple(params['outputs']), tuple(params['feedthroughs']), tuple(
            params['states']), tuple(params['targets']), node


if __name__ == '__main__':

    log_level = get_ROS_log_level(rospy.get_name())

    rospy.init_node('rxnode', log_level=eagerx_core.core.log_levels_ROS[log_level])

    message_broker = eagerx_core.RxMessageBroker(owner=rospy.get_name())

    pnode = RxNode(name=rospy.get_name(), message_broker=message_broker)

    message_broker.connect_io()

    pnode.node_initialized()

    rospy.spin()


###########################################################################
# Specific implementations ################################################
###########################################################################


class RealResetNode(ResetNode):
    # MSG_TYPE = {'in_1': 'std_msgs.msg/UInt64',
    #             'in_2': 'std_msgs.msg/UInt64',}

    def __init__(self, test_arg, **kwargs):

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20
        super().__init__(**kwargs)

    def reset(self, ticks):
        self.num_resets += 1
        if True:
            if self.num_resets % self.print_iter == 0:
                if self.iter_start:
                    iter_stop = time.time()
                    if self.iter_ticks > 0:
                        iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                    else:
                        iter_time = float(iter_stop - self.iter_start)
                    # Memory usage request
                    mem_use = (np.array(self.py.memory_info()[0:2]) / 2. ** 30) * 1000  # memory use in MB...I think

                    rospy.logdebug("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                        self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0
        return ticks

    def callback(self, inputs, targets):
        node_tick = inputs['node_tick']

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in inputs:
                t_i = inputs[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.outputs:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg

        # Fill state done msg with number of node ticks
        for i in self.targets:
            name = i['name']
            msg = UInt64()
            if self.num_ticks > 2:
                msg.data = 1
            else:
                msg.data = 0
            output_msgs[name + '/done'] = msg
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs


class ProcessNode(Node):
    def __init__(self, test_arg, **kwargs):
        # If node is simulator, we will probably use this in callback & reset
        self.simulator = None

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20
        super().__init__(**kwargs)

    def set_simulator(self, simulator):
        self.simulator = simulator

    def reset(self, ticks):
        self.num_resets += 1
        if True:
            if self.num_resets % self.print_iter == 0:
                if self.iter_start:
                    iter_stop = time.time()
                    if self.iter_ticks > 0:
                        iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                    else:
                        iter_time = float(iter_stop - self.iter_start)
                    # Memory usage request
                    mem_use = (np.array(self.py.memory_info()[0:2]) / 2. ** 30) * 1000  # memory use in MB...I think

                    rospy.logdebug("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                    self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0

        # Send initial message for outputs with 'start_with_msg' = True
        init_msgs = dict()
        for i in self.outputs:
            if not i['start_with_msg']: continue
            name = i['name']
            init_msgs[name] = UInt64(data=999)
        return init_msgs

    def callback(self, inputs):
        # Verify that # of ticks equals internal counter
        node_tick = inputs['node_tick']
        if not self.num_ticks == node_tick:
            rospy.logerr('[%s][callback]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

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
        Nc = self.num_ticks
        for i in self.outputs:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs