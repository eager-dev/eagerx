#!/usr/bin/env python3

# ROS imports
import rospy
from std_msgs.msg import UInt64

# Rx imports
from eagerx.core.constants import log_levels_ROS
import eagerx.core.rx_message_broker
import eagerx.core.rx_pipelines
from eagerx.utils.utils import (
    get_attribute_from_module,
    initialize_converter,
    get_param_with_blocking,
    get_opposite_msg_cls,
)

# Other imports
import sys


class RxNode(object):
    def __init__(self, name, message_broker, **kwargs):
        self.name = name
        self.ns = "/".join(name.split("/")[:2])
        self.mb = message_broker
        self.initialized = False
        self.has_shutdown = False

        # Prepare input & output topics
        (
            rate,
            inputs,
            outputs,
            feedthroughs,
            states,
            targets,
            self.node,
        ) = self._prepare_io_topics(self.name, **kwargs)

        # Initialize reactive pipeline
        rx_objects = eagerx.core.rx_pipelines.init_node(
            self.ns,
            rate,
            self.node,
            inputs,
            outputs,
            feedthrough=feedthroughs,
            state_inputs=states,
            targets=targets,
        )
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

        # Prepare closing routine
        rospy.on_shutdown(self.node_shutdown)

    def node_initialized(self):
        # Notify env that node is initialized
        self.init_pub = rospy.Publisher(self.name + "/initialized", UInt64, queue_size=0, latch=True)
        self.init_pub.publish(UInt64())

        if not self.initialized:
            rospy.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, **kwargs):
        params = get_param_with_blocking(name)
        rate = params["rate"]

        # Get info from bridge on reactive properties
        is_reactive = get_param_with_blocking(self.ns + "/bridge/is_reactive")
        real_time_factor = get_param_with_blocking(self.ns + "/bridge/real_time_factor")
        simulate_delays = get_param_with_blocking(self.ns + "/bridge/simulate_delays")

        # Get node
        node_cls = get_attribute_from_module(params["node_type"])
        node = node_cls(
            ns=self.ns,
            message_broker=self.mb,
            is_reactive=is_reactive,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            **kwargs,
            **params,
        )

        # Prepare input topics
        for i in params["inputs"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            i["msg_type"] = get_opposite_msg_cls(i["msg_type"], i["converter"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
            # else:  # Converter already initialized

        # Prepare output topics
        for i in params["outputs"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            i["msg_type"] = get_opposite_msg_cls(i["msg_type"], i["converter"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
            # else:  # Converter already initialized

        # Prepare state topics
        for i in params["states"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            i["msg_type"] = get_opposite_msg_cls(i["msg_type"], i["converter"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
            # else:  # Converter already initialized

        # Prepare target topics
        for i in params["targets"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            i["msg_type"] = get_opposite_msg_cls(i["msg_type"], i["converter"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
            # else:  # Converter already initialized

        # Prepare feedthrough topics
        for i in params["feedthroughs"]:
            i["msg_type"] = get_attribute_from_module(i["msg_type"])
            i["msg_type"] = get_opposite_msg_cls(i["msg_type"], i["converter"])
            if isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
            # else:  # Converter already initialized

        return (
            rate,
            tuple(params["inputs"]),
            tuple(params["outputs"]),
            tuple(params["feedthroughs"]),
            tuple(params["states"]),
            tuple(params["targets"]),
            node,
        )

    def _shutdown(self):
        rospy.logdebug(f"[{self.name}] RxNode._shutdown() called.")
        self.init_pub.unregister()

    def node_shutdown(self):
        if not self.has_shutdown:
            rospy.logdebug(f"[{self.name}] RxNode.node_shutdown() called.")
            rospy.loginfo(f"[{self.name}] Shutting down.")
            self._shutdown()
            self.node.shutdown()
            self.mb.shutdown()
            self.has_shutdown = True


if __name__ == "__main__":
    try:
        executable, ns, name, object_name = sys.argv[0], sys.argv[-3], sys.argv[-2], sys.argv[-1]

        log_level = get_param_with_blocking(ns + "/log_level")

        rospy.init_node(
            f"{name}".replace("/", "_"),
            log_level=log_levels_ROS[log_level],
            anonymous=True,
        )

        message_broker = eagerx.core.rx_message_broker.RxMessageBroker(owner=f"{ns}/{name}")

        pnode = RxNode(name=f"{ns}/{name}", message_broker=message_broker, object_name=f"{ns}/{object_name}")

        message_broker.connect_io()

        pnode.node_initialized()

        rospy.spin()
    finally:
        if not pnode.has_shutdown:
            rospy.loginfo(f"[{ns}/{name}] Send termination signal to '{ns}/{name}'.")
            rospy.signal_shutdown(f"Terminating '{ns}/{name}'")
