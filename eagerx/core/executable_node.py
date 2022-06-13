#!/usr/bin/env python3

# source ROS if in colab
import os

if bool(eval(os.environ.get("EAGERX_COLAB", "0"))):
    import site

    site.addsitedir("/opt/ros/melodic/lib/python2.7/dist-packages")
    site.addsitedir("/usr/lib/python2.7/dist-packages")

# Rx imports
import eagerx.core.ros1 as bnd
import eagerx.core.rx_message_broker
import eagerx.core.rx_pipelines
from eagerx.utils.utils import (
    load,
    initialize_processor,
    dict_to_space, get_param_with_blocking,
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
        bnd.on_shutdown(self.node_shutdown)

    def node_initialized(self):
        # Notify env that node is initialized
        self.init_pub = bnd.Publisher(self.name + "/initialized", "int64")
        self.init_pub.publish(0)

        if not self.initialized:
            bnd.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, **kwargs):
        params = get_param_with_blocking(name)
        rate = params["rate"]

        # Get info from engine on reactive properties
        sync = get_param_with_blocking(self.ns + "/engine/sync")
        real_time_factor = get_param_with_blocking(self.ns + "/engine/real_time_factor")
        simulate_delays = get_param_with_blocking(self.ns + "/engine/simulate_delays")

        # Get node
        node_cls = load(params["node_type"])
        node = node_cls(
            ns=self.ns,
            message_broker=self.mb,
            sync=sync,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            **kwargs,
            **params,
        )

        # Prepare input topics
        for i in params["inputs"]:
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_processor(i["processor"])
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

        # Prepare output topics
        for i in params["outputs"]:
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_processor(i["processor"])
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

        # Prepare state topics
        for i in params["states"]:
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_processor(i["processor"])
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

        # Prepare target topics
        for i in params["targets"]:
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_processor(i["processor"])
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

        # Prepare feedthrough topics
        for i in params["feedthroughs"]:
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_processor(i["processor"])
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

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
        bnd.logdebug(f"[{self.name}] RxNode._shutdown() called.")
        self.init_pub.unregister()

    def node_shutdown(self):
        if not self.has_shutdown:
            bnd.logdebug(f"[{self.name}] RxNode.node_shutdown() called.")
            bnd.loginfo(f"[{self.name}] Shutting down.")
            self._shutdown()
            self.node.shutdown()
            self.mb.shutdown()
            self.has_shutdown = True


if __name__ == "__main__":
    try:
        executable, ns, name, object_name = sys.argv[0], sys.argv[-3], sys.argv[-2], sys.argv[-1]

        log_level = get_param_with_blocking(ns + "/log_level")

        bnd.set_log_level(log_level)

        message_broker = eagerx.core.rx_message_broker.RxMessageBroker(owner=f"{ns}/{name}")

        pnode = RxNode(name=f"{ns}/{name}", message_broker=message_broker, object_name=f"{ns}/{object_name}")

        message_broker.connect_io()

        pnode.node_initialized()

        bnd.spin()
    finally:
        if not pnode.has_shutdown:
            bnd.loginfo(f"[{ns}/{name}] Send termination signal to '{ns}/{name}'.")
            bnd.signal_shutdown(f"Terminating '{ns}/{name}'")
