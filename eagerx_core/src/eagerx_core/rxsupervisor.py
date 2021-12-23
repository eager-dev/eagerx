#!/usr/bin/env python3

from __future__ import print_function

# ROS imports
import rospy
import rosparam
from std_msgs.msg import UInt64, String, Bool

# Rx imports
from eagerx_core.constants import process
from eagerx_core.rxnode import RxNode
import eagerx_core.rxmessage_broker
import eagerx_core.rxoperators
import eagerx_core.rxpipelines
from eagerx_core.basenode import NodeBase
from eagerx_core.params import RxObjectParams, RxNodeParams
from eagerx_core.utils.utils import get_attribute_from_module, initialize_converter, get_param_with_blocking, get_opposite_msg_cls
from eagerx_core.utils.node_utils import initialize_nodes
from eagerx_core.baseconverter import IdentityConverter
from eagerx_core.srv import ImageUInt8
import eagerx_core

# OTHER
from threading import Event


class SupervisorNode(NodeBase):
    msg_types = {'outputs': {'step': UInt64}}

    def __init__(self, ns, states, **kwargs):
        self.subjects = None

        # Render
        self._render_service_ready = False
        self.render_toggle = False
        self.get_last_image_service = rospy.ServiceProxy('%s/env/render/get_last_image' % ns, ImageUInt8)
        self.render_toggle_pub = rospy.Publisher('%s/env/render/toggle' % ns, Bool, queue_size=0)

        # Initialize nodes
        self.is_initialized = dict()
        self.launch_nodes = dict()
        self.sp_nodes = dict()

        # Initialize buffer to hold desired reset states
        self.state_buffer = dict()
        for i in states:
            if isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            else:
                converter = i['converter']
            self.state_buffer[i['name']] = {'msg': None, 'converter': converter}

        # Required for reset
        self._reset_event = Event()
        self._obs_event = Event()
        self._step_counter = 0
        super().__init__(ns=ns, states=states, **kwargs)

    def _set_subjects(self, subjects):
        self.subjects = subjects

    def start_render(self):
        if not self.render_toggle:
            self.render_toggle = True
            self.render_toggle_pub.publish(Bool(data=self.render_toggle))

    def stop_render(self):
        if self.render_toggle:
            self.render_toggle = False
            self.render_toggle_pub.publish(Bool(data=self.render_toggle))

    def get_last_image(self):
        if not self._render_service_ready:
            rospy.wait_for_service('%s/env/render/get_last_image' % self.ns)
        return self.get_last_image_service().image

    def register_node(self, node: RxNodeParams):
        node_name = node.name
        initialize_nodes(node, process.ENVIRONMENT, self.ns, self.ns, self.message_broker, self.is_initialized, self.sp_nodes, self.launch_nodes, rxnode_cls=RxNode)
        self.subjects['register_node'].on_next(String(self.ns + '/' + node_name))

    def register_object(self, object: RxObjectParams, bridge_name: str):
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        # Check if object name is unique
        obj_name = object.name
        assert rospy.get_param(self.ns + '/' + obj_name + '/nodes', None) is None, 'Object name "%s" already exists. Object names must be unique.' % self.ns + '/' + obj_name

        # Upload object params to rosparam server
        params, nodes = object.get_params(ns=self.ns, bridge=bridge_name)
        rosparam.upload_params(self.ns, params)

        # Upload node parameters to ROS param server
        initialize_nodes(nodes, process.ENVIRONMENT, self.ns, self.ns, message_broker=self.message_broker, in_object=True,
                         is_initialized=self.is_initialized, sp_nodes=self.sp_nodes, launch_nodes=self.launch_nodes)
        self.subjects['register_object'].on_next(String(self.ns + '/' + obj_name))

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
        rospy.logdebug('RESET END')
        self._obs_event.wait()
        rospy.logdebug('FIRST OBS RECEIVED!')

    def step(self):
        self._obs_event.clear()
        self.subjects['step'].on_next(self._get_step_counter_msg())
        self._step_counter += 1
        self._obs_event.wait()
        rospy.logdebug('STEP END')


class RxSupervisor(object):
    def __init__(self, name, message_broker, is_reactive, real_time_factor):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        outputs, states, self.node = self._prepare_io_topics(self.name, is_reactive, real_time_factor)

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

    def _prepare_io_topics(self, name, is_reactive, real_time_factor):
        params = get_param_with_blocking(name)

        # Get node
        node_cls = get_attribute_from_module(params['node_type'])
        node = node_cls(ns=self.ns, message_broker=self.mb, is_reactive=is_reactive, real_time_factor=real_time_factor,
                        **params)

        # Prepare output topics
        for i in params['outputs']:
            i['msg_type'] = get_attribute_from_module(i['msg_type'])
            if isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])

        # Prepare state topics
        for i in params['states']:
            i['msg_type'] = get_attribute_from_module(i['msg_type'])
            if isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])

        return tuple(params['outputs']), tuple(params['states']), node


if __name__ == '__main__':

    rospy.init_node('env', log_level=rospy.INFO)

    message_broker = eagerx_core.rxmessage_broker.RxMessageBroker(owner=rospy.get_name())

    pnode = RxSupervisor(name=rospy.get_name(), message_broker=message_broker)

    message_broker.connect_io()

    pnode.node_initialized()

    rospy.spin()