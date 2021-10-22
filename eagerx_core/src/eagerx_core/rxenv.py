#!/usr/bin/env python3

from __future__ import print_function

# ROS imports
import rospy
import rosparam
from std_msgs.msg import UInt64, String

# Rx imports
from eagerx_core.params import RxObjectParams
from eagerx_core.utils.utils import get_attribute_from_module, get_param_with_blocking, initialize_converter
from eagerx_core.converter import IdentityConverter
import eagerx_core

# OTHER
from threading import Event


class EnvironmentNode(object):
    def __init__(self, name, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.params = get_param_with_blocking(self.name)
        self.subjects = None

        # Initialize buffer to hold desired reset states
        self.state_buffer = dict()
        for i in self.params['states']:
            if 'converter' in i:
                converter = initialize_converter(i['converter'])
            else:
                converter = None
            self.state_buffer[i['name']] = {'msg': None, 'converter': converter}

        # Required for reset
        self.return_event = Event()
        self.observation_event = Event()
        self.step_counter = 0

    def set_subjects(self, subjects):
        self.subjects = subjects

    def register_object(self, object: RxObjectParams, bridge_name: str):
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        # Check if object name is unique
        obj_name = object.name
        assert rospy.get_param(self.ns + '/' + obj_name + '/nodes', None) is None, 'Object name "%s" already exists. Object names must be unique.' % self.ns + '/' + obj_name

        params, nodes = object.get_params(ns=self.ns, bridge=bridge_name)

        # Upload object params to rosparam server
        rosparam.upload_params(self.ns, params)

        # Upload parameters to ROS param server
        for node in nodes:
            params = node.get_params(ns=self.ns)
            node_name = node.name

            # Check if node name is unique
            assert rospy.get_param(self.ns + '/' + node_name, None) is None, 'Node name "%s" already exists. Node names must be unique.' % self.ns + '/' + node_name

            # Upload params to rosparam server
            rosparam.upload_params(self.ns, params)

        self.subjects['register'].on_next(String(self.ns + '/' + obj_name))

    def _get_states(self, reset_msg):
        # Fill output_msg with buffered states
        state_msgs = dict()
        done_msgs = dict()
        for name, buffer in self.state_buffer.items():
            if buffer['msg'] is None:
                done_msgs[name] = UInt64(data=1)
            else:
                done_msgs[name] = UInt64(data=0)
                state_msgs[name] = buffer['msg']
                buffer['msg'] = None  # After sending state, set msg to None
        return dict(state=state_msgs, done=done_msgs)

    def _observations_clear(self, msg):
        self.observation_event.clear()
        return msg

    def _observations_wait(self, msg):
        self.observation_event.wait()
        return msg

    def _observations_set(self, msg):
        self.observation_event.set()
        return msg

    def _return_clear(self, msg):
        self.return_event.clear()
        return msg

    def _return_wait(self, msg):
        self.return_event.wait()
        return msg

    def _return_set(self, msg):
        self.return_event.set()
        return msg

    def reset(self):
        self.return_event.clear()
        self.subjects['reset'].on_next(True)
        self.step_counter = 0
        return self.return_event.wait()

    def step(self):
        self.return_event.clear()
        self.subjects['step'].on_next(True)
        self.step_counter += 1
        return self.return_event.wait()


class RxEnvironment(object):
    def __init__(self, name, message_broker, scheduler=None, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        states, self.node = self._prepare_io_topics(self.name, **kwargs)

        # Initialize reactive pipeline
        rx_objects, env_subjects = eagerx_core.init_env_reset(self.ns, state_inputs=states, node_name=self.name, scheduler=scheduler)
        self.node.set_subjects(env_subjects)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

    def node_initialized(self):
        # Notify env that node is initialized
        init_pub = rospy.Publisher(self.name + '/initialized', UInt64, queue_size=0, latch=True)
        init_pub.publish(UInt64())

        if not self.initialized:
            rospy.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, **kwargs):
        params = get_param_with_blocking(name)

        # Get node
        node_cls = get_attribute_from_module(params['module'], params['node_type'])
        node = node_cls(name, **kwargs)

        # Prepare state topics
        for i in params['states']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i:
                i['converter'] = initialize_converter(i['converter'])
            else:
                i['converter'] = IdentityConverter()

        return tuple(params['states']), node


if __name__ == '__main__':

    rospy.init_node('env', log_level=rospy.INFO)

    message_broker = eagerx_core.RxMessageBroker(owner=rospy.get_name())

    pnode = RxEnvironment(name=rospy.get_name(), message_broker=message_broker)

    message_broker.connect_io()

    pnode.node_initialized()

    rospy.spin()