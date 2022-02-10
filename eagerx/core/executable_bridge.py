#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt64

# Rx imports
import eagerx.core.rx_message_broker
import eagerx.core.rx_operators
import eagerx.core.rx_pipelines
from eagerx.utils.utils import (
    get_attribute_from_module,
    initialize_converter,
    get_param_with_blocking,
    get_opposite_msg_cls,
)
from eagerx.utils.node_utils import wait_for_node_initialization
from eagerx.core.constants import log_levels_ROS

# Other imports
from threading import Condition
import sys


class RxBridge(object):
    def __init__(self, name, message_broker):
        self.name = name
        self.ns = "/".join(name.split("/")[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        (
            rate,
            inputs,
            outputs,
            states,
            node_names,
            target_addresses,
            self.bridge,
        ) = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects = eagerx.core.rx_pipelines.init_bridge(
            self.ns,
            rate,
            self.bridge,
            inputs,
            outputs,
            states,
            node_names,
            target_addresses,
            self.mb,
        )
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)
        self.mb.add_rx_objects(node_name=name + "/dynamically_registered", node=self)
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
                init_pub = rospy.Publisher(self.name + "/initialized", UInt64, queue_size=0, latch=True)
                init_pub.publish(UInt64(data=1))
                rospy.loginfo('Node "%s" initialized.' % self.name)
                self.initialized = True

    def _prepare_io_topics(self, name):
        params = get_param_with_blocking(name)
        node_names = params["node_names"]
        target_addresses = params["target_addresses"]
        rate = params["rate"]

        # Get node
        node_cls = get_attribute_from_module(params["node_type"])
        node = node_cls(ns=self.ns, message_broker=self.mb, **params)

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

        return (
            rate,
            params["inputs"],
            tuple(params["outputs"]),
            tuple(params["states"]),
            node_names,
            target_addresses,
            node,
        )

    def _close(self):
        return True


if __name__ == "__main__":
    executable, ns, name = sys.argv

    log_level = get_param_with_blocking(ns + "/log_level")

    rospy.init_node(f"{name}".replace("/", "_"), log_level=log_levels_ROS[log_level], anonymous=True)

    message_broker = eagerx.core.rx_message_broker.RxMessageBroker(owner=f"{ns}/{name}")

    pnode = RxBridge(name=f"{ns}/{name}", message_broker=message_broker)

    message_broker.connect_io()

    pnode.node_initialized()

    rospy.spin()
