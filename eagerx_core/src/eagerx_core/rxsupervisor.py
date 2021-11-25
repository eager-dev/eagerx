#!/usr/bin/env python3

from __future__ import print_function

# ROS imports
import rospy
import rosparam
from std_msgs.msg import UInt64, String

# Rx imports
import eagerx_core.rxmessage_broker
import eagerx_core.rxoperators
import eagerx_core.rxpipelines
from eagerx_core.basenode import NodeBase
from eagerx_core.params import RxObjectParams
from eagerx_core.utils.utils import get_attribute_from_module, initialize_converter, get_param_with_blocking
from eagerx_core.utils.node_utils import initialize_nodes
from eagerx_core.baseconverter import IdentityConverter
import eagerx_core

# OTHER
from threading import Event


class SupervisorNode(NodeBase):
    def __init__(self, states, **kwargs):
        self.subjects = None

        self._is_initialized = dict()
        self._launch_nodes = dict()
        self._sp_nodes = dict()

        # Initialize buffer to hold desired reset states
        self.state_buffer = dict()
        for i in states:
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.state_buffer[i['name']] = {'msg': None, 'converter': converter}

        # Required for reset
        self._reset_event = Event()
        self._obs_event = Event()
        self._step_counter = 0
        super().__init__(states=states, **kwargs)

    def _set_subjects(self, subjects):
        self.subjects = subjects

    def register_object(self, object: RxObjectParams, bridge_name: str):
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        # Check if object name is unique
        obj_name = object.name
        assert rospy.get_param(self.ns + '/' + obj_name + '/nodes', None) is None, 'Object name "%s" already exists. Object names must be unique.' % self.ns + '/' + obj_name

        params, nodes = object.get_params(ns=self.ns, bridge=bridge_name)

        # Upload object params to rosparam server
        rosparam.upload_params(self.ns, params)

        # Upload node parameters to ROS param server
        for node in nodes:
            params = node.get_params(ns=self.ns, in_object=True)
            node_name = node.name

            # Check if node name is unique
            assert rospy.get_param(self.ns + '/' + node_name, None) is None, 'Node name "%s" already exists. Node names must be unique.' % self.ns + '/' + node_name

            # Upload params to rosparam server
            rosparam.upload_params(self.ns, params)

            # If simulation node must run in separate process, the environment must launch it (i.e. the main thread).
            if not params[node_name]['single_process']:
                assert not params[node_name]['launch_locally'], 'If simulation node "%s" must run in a separate process, the environment must launch it (i.e. the main thread). Hence, launch_locally in "%s.yaml" via the config of the object must be set to "False".' % (node_name, params[node_name]['config_name'])
                params[node_name]['launch_locally'] = True
                initialize_nodes(params[node_name], self.ns, self.ns, message_broker=None, is_initialized=self._is_initialized,
                                 sp_nodes=self._sp_nodes, launch_nodes=self._launch_nodes)

        self.subjects['register'].on_next(String(self.ns + '/' + obj_name))

    def _get_states(self, reset_msg):
        # Fill output_msg with buffered states
        msgs = dict()
        for name, buffer in self.state_buffer.items():
            if buffer['msg'] is None:
                msgs[name + '/done'] = UInt64(data=1)
            else:
                msgs[name + '/done'] = UInt64(data=0)
                msgs[name] = buffer['msg']
                buffer['msg'] = None  # After sending state, set msg to None
        return msgs

    def _clear_obs_event(self, msg):
        self._obs_event.clear()
        return msg

    def _set_obs_event(self, msg):
        self._obs_event.set()
        return msg

    def _set_reset_event(self, msg):
        self._reset_event.set()
        return msg

    def _get_step_counter_msg(self):
        return UInt64(data=self._step_counter)

    def reset(self):
        self._reset_event.clear()
        self.subjects['start_reset'].on_next(self._get_step_counter_msg())
        self._step_counter = 0
        self._reset_event.wait()
        print('RESET END')
        self._obs_event.wait()
        print('FIRST OBS RECEIVED!')

    def step(self):
        self._obs_event.clear()
        self.subjects['step'].on_next(self._get_step_counter_msg())
        self._step_counter += 1
        self._obs_event.wait()
        print('STEP END')


class RxSupervisor(object):
    def __init__(self, name, message_broker):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        outputs, states, self.node = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects, env_subjects = eagerx_core.rxpipelines.init_supervisor(self.ns, self.node, outputs=outputs, state_outputs=states)
        self.node._set_subjects(env_subjects)
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

        # Get node
        node_cls = get_attribute_from_module(params['module'], params['node_type'])
        node = node_cls(ns=self.ns, message_broker=self.mb, **params)

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

        return tuple(params['outputs']), tuple(params['states']), node


if __name__ == '__main__':

    rospy.init_node('env', log_level=rospy.INFO)

    message_broker = eagerx_core.rxmessage_broker.RxMessageBroker(owner=rospy.get_name())

    pnode = RxSupervisor(name=rospy.get_name(), message_broker=message_broker)

    message_broker.connect_io()

    pnode.node_initialized()

    rospy.spin()