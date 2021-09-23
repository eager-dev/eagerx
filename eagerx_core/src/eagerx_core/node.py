# ROS imports
import rospy, rosparam
from std_msgs.msg import UInt64, String

# Rx imports
from eagerx_core.utils.utils import get_attribute_from_module, get_param_with_blocking
import eagerx_core

# Memory usage
from threading import current_thread
import os, psutil, time
import numpy as np


class StateNode(object):
    def __init__(self, name, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.params = get_param_with_blocking(self.name)

        # Append states on rosparam server that are reset by this node
        sim_pub = rospy.Publisher(self.ns + '/resettable/sim', String, queue_size=0)
        rospy.sleep(0.1)
        states = dict()
        for i in self.params['states_in']:
            address = i['address']
            sim_pub.publish(String(address))
            states[address.replace('/', '.')] = True
        rosparam.upload_params(self.ns, {'states': states})

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0
        self.dt = 1 / self.params['rate']

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20

    def reset(self, ticks):
        # todo: differentiate between real_reset and sim_reset states. Use rosparam server?
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

                    rospy.loginfo("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                    self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0
        return ticks

    def callback(self, topics_in):
        # Verify that # of ticks equals internal counter
        node_tick = topics_in['node_tick']
        if not self.num_ticks == node_tick:
            print('[%s][callback]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * self.dt
        for i in self.params['topics_in']:
            name = i['name']
            if name in topics_in:
                t_i = topics_in[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    print('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.params['topics_out']:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs


class RealResetNode(object):
    def __init__(self, name, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.params = get_param_with_blocking(self.name)
        if 'node_args' in self.params.keys():
            self.node_args = self.params['node_args']

        # Append states on rosparam server that are reset by this node
        # real_pub = rospy.Publisher(self.ns + '/resettable/real', String, queue_size=0)
        real_reset = dict()
        for i in self.params['states_in']:
            address = i['address']
            # real_pub.publish(String(address))
            real_reset[address.replace('/', '.')] = True
        rosparam.upload_params(self.ns, {'real_reset': real_reset})
        rosparam.upload_params(self.ns, {'states': real_reset})  # Also upload it to states that are available

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0
        self.dt = 1 / self.params['rate']

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20

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

                    rospy.loginfo("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                        self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0
        return ticks

    def callback(self, cb_input_states):
        cb_input = cb_input_states[0]
        cb_states = cb_input_states[1]
        node_tick = cb_input['node_tick']

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * self.dt
        for i in self.params['topics_in']:
            name = i['name']
            if name in cb_input:
                t_i = cb_input[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    print('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.params['topics_out']:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg

        # Fill state done msg with number of node ticks
        for i in self.params['states_in']:
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


class ProcessNode(object):
    def __init__(self, name, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])

        # Message counter
        self.params = get_param_with_blocking(self.name)
        self.num_ticks = 0
        self.num_resets = 0
        self.dt = 1 / self.params['rate']

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20

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

                    rospy.loginfo("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                    self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0
        return ticks

    def callback(self, topics_in):
        # Verify that # of ticks equals internal counter
        node_tick = topics_in['node_tick']
        if not self.num_ticks == node_tick:
            print('[%s][callback]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * self.dt
        for i in self.params['topics_in']:
            name = i['name']
            if name in topics_in:
                t_i = topics_in[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    print('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.params['topics_out']:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs


class RxNode(object):
    def __init__(self, name, message_broker, scheduler=None, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        # todo: pass rx_objects to message_broker
        dt, topics_in, topics_out, feedthrough_in, states_in, node = self._prepare_io_topics(self.name, **kwargs)

        # Initialize reactive pipeline
        # todo: pass rx_objects to message_broker
        rx_objects = eagerx_core.init_node(self.ns, dt, node.callback, node.reset, topics_in, topics_out,
                                           feedthrough=feedthrough_in, state_inputs=states_in,
                                           node_name=self.name, scheduler=scheduler)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

    def node_initialized(self):
        # Notify env that node is initialized
        init_pub = rospy.Publisher(self.name + '/initialized', UInt64, queue_size=0, latch=True)
        init_pub.publish(UInt64())

        if not self.initialized:
            rospy.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, **kwargs):
        # params = rospy.get_param(name)
        params = get_param_with_blocking(name)
        rate = params['rate']
        dt = 1 / rate

        # Get node
        node_cls = get_attribute_from_module(params['module'], params['node_type'])
        node = node_cls(name, **kwargs)

        # Prepare input topics
        for i in params['topics_in']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

        # Prepare output topics
        for i in params['topics_out']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

        # Prepare action topics
        for i in params['feedthrough_in']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

        # Prepare state topics
        for i in params['states_in']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

        return dt, tuple(params['topics_in']), tuple(params['topics_out']), tuple(params['feedthrough_in']),\
               tuple(params['states_in']), node

