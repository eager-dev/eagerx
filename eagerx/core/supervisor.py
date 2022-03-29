from __future__ import print_function

# ROS imports
import rospy
import rosparam
from std_msgs.msg import UInt64, String, Bool
from sensor_msgs.msg import Image

# Rx imports
from eagerx.core.constants import process
from eagerx.core.executable_node import RxNode
import eagerx.core.rx_message_broker
import eagerx.core.rx_operators
import eagerx.core.rx_pipelines
from eagerx.core.entities import BaseNode
from eagerx.core.specs import NodeSpec, ObjectSpec
from eagerx.utils.utils import (
    get_attribute_from_module,
    initialize_converter,
    get_param_with_blocking,
)
from eagerx.utils.node_utils import initialize_nodes
from eagerx.core.nodes import EnvNode
import eagerx

# OTHER
from threading import Event


class SupervisorNode(BaseNode):
    def __init__(self, ns, states, **kwargs):
        self.subjects = None
        self.env_node: EnvNode = None

        # Render
        self.last_image = None
        self._image_event = Event()
        self.render_toggle = False
        self.pub_get_last_image = rospy.Publisher("%s/env/render/get_last_image" % ns, Bool, queue_size=0, latch=True)
        self.sub_set_last_image = rospy.Subscriber("%s/env/render/set_last_image" % ns, Image, self._last_image_callback)
        self.render_toggle_pub = rospy.Publisher("%s/env/render/toggle" % ns, Bool, queue_size=0, latch=True)

        # Initialize nodes
        self.cum_registered = 0
        self.is_initialized = dict()
        self.launch_nodes = dict()
        self.sp_nodes = dict()

        # Initialize buffer to hold desired reset states
        self.state_buffer = dict()
        for i in states:
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
                converter = i["converter"]
            else:
                converter = i["converter"]
            self.state_buffer[i["name"]] = {"msg": None, "converter": converter}

        # Required for reset
        self._step_counter = 0
        super().__init__(ns=ns, states=states, **kwargs)

    def set_environment(self, env_node: EnvNode):
        self.env_node = env_node

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
        self._image_event.clear()
        self.pub_get_last_image.publish(Bool())
        self._image_event.wait()
        return self.last_image

    def _last_image_callback(self, msg):
        self.last_image = msg
        self._image_event.set()

    def register_node(self, node: NodeSpec):
        # Increase cumulative registered counter. Is send as '/start_reset' message.
        self.cum_registered += 1

        # Initialize node
        node_name = node.config.name
        initialize_nodes(
            node,
            process.ENVIRONMENT,
            self.ns,
            self.message_broker,
            self.is_initialized,
            self.sp_nodes,
            self.launch_nodes,
            rxnode_cls=RxNode,
        )
        self.subjects["register_node"].on_next(String(self.ns + "/" + node_name))

    def register_object(self, object: ObjectSpec, bridge_name: str):
        # Increase cumulative registered counter. Is send as '/start_reset' message.
        self.cum_registered += 1

        # Check if object name is unique
        obj_name = object.config.name
        assert (
            rospy.get_param(self.ns + "/" + obj_name + "/nodes", None) is None
        ), f'Object name "{self.ns}/{obj_name}" already exists. Object names must be unique.'

        # Upload object params to rosparam server
        params, nodes = object.build(ns=self.ns, bridge_id=bridge_name)
        rosparam.upload_params(self.ns, params)

        # Set node args
        node_args = dict(
            object_name=f"{self.ns}/{obj_name}",
        )

        # Upload node parameters to ROS param server
        initialize_nodes(
            nodes,
            process.ENVIRONMENT,
            self.ns,
            message_broker=self.message_broker,
            is_initialized=self.is_initialized,
            sp_nodes=self.sp_nodes,
            launch_nodes=self.launch_nodes,
            node_args=node_args,
            object_name=obj_name,
        )
        self.subjects["register_object"].on_next(String(f"{self.ns}/{obj_name}"))

    def _get_states(self, reset_msg):
        # Fill output_msg with buffered states
        msgs = dict()
        for name, buffer in self.state_buffer.items():
            if buffer["msg"] is None:
                msgs[name + "/done"] = Bool(data=True)
            else:
                msgs[name + "/done"] = Bool(data=False)
                msgs[name] = buffer["msg"]
                buffer["msg"] = None  # After sending state, set msg to None
        return msgs

    def reset(self):
        self.env_node.obs_event.clear()
        self.env_node.must_reset = True
        self.env_node.action_event.set()
        self.subjects["start_reset"].on_next(UInt64(data=self.cum_registered))
        self._step_counter = 0
        try:
            flag = self.env_node.obs_event.wait()
            if not flag:
                raise KeyboardInterrupt
        except (KeyboardInterrupt, SystemExit):
            print("[reset] KEYBOARD INTERRUPT")
            raise
        rospy.logdebug("FIRST OBS RECEIVED!")

    def step(self):
        self.env_node.obs_event.clear()
        self.env_node.action_event.set()
        self._step_counter += 1
        try:
            flag = self.env_node.obs_event.wait()
            if not flag:
                raise KeyboardInterrupt
        except (KeyboardInterrupt, SystemExit):
            print("[step] KEYBOARD INTERRUPT")
            raise
        rospy.logdebug("STEP END")

    def shutdown(self):
        self.env_node.action_event.set()
        self.pub_get_last_image.unregister()
        self.sub_set_last_image.unregister()
        self.render_toggle_pub.unregister()


class Supervisor(object):
    def __init__(self, name, message_broker, is_reactive, real_time_factor, simulate_delays):
        self.name = name
        self.ns = "/".join(name.split("/")[:2])
        self.mb = message_broker
        self.initialized = False
        self.is_reactive = is_reactive
        self.has_shutdown = False

        # Prepare input & output topics
        outputs, states, self.node = self._prepare_io_topics(self.name, is_reactive, real_time_factor, simulate_delays)

        # Initialize reactive pipeline
        rx_objects, env_subjects = eagerx.core.rx_pipelines.init_supervisor(
            self.ns, self.node, outputs=outputs, state_outputs=states
        )
        self.node._set_subjects(env_subjects)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

    def node_initialized(self):
        # Notify env that node is initialized
        self.init_pub = rospy.Publisher(self.name + "/initialized", UInt64, queue_size=0, latch=True)
        self.init_pub.publish(UInt64())

        if not self.initialized:
            rospy.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, is_reactive, real_time_factor, simulate_delays):
        params = get_param_with_blocking(name)

        # Get node
        node_cls = get_attribute_from_module(params["node_type"])
        node = node_cls(
            ns=self.ns,
            message_broker=self.mb,
            is_reactive=is_reactive,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            **params,
        )

        # Prepare output topics
        for i in params["outputs"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])

        # Prepare state topics
        for i in params["states"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])

        return tuple(params["outputs"]), tuple(params["states"]), node

    def _shutdown(self):
        rospy.logdebug(f"[{self.name}] Supervisor._shutdown() called.")
        self.init_pub.unregister()

    def node_shutdown(self):
        if not self.has_shutdown:
            rospy.logdebug(f"[{self.name}] Supervisor.node_shutdown() called.")
            rospy.loginfo(f"[{self.name}] Shutting down.")
            self._shutdown()
            self.node.shutdown()
            self.mb.shutdown()
            self.has_shutdown = True
