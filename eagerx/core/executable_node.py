#!/usr/bin/env python3

# source ROS if in colab
# import os
# if bool(eval(os.environ.get("EAGERX_COLAB", "0"))):
#     import site
#
#     site.addsitedir("/opt/ros/melodic/lib/python2.7/dist-packages")
#     site.addsitedir("/usr/lib/python2.7/dist-packages")

# Rx imports
from eagerx.core.entities import Backend
import eagerx.core.rx_message_broker
import eagerx.core.rx_pipelines
from eagerx.utils.utils import (
    load,
    initialize_processor,
    get_param_with_blocking,
)

# Other imports
import argparse


class RxNode(object):
    def __init__(self, name, message_broker, **kwargs):
        self.name = name
        self.ns = "/".join(name.split("/")[:2])
        self.mb = message_broker
        self.backend = message_broker.bnd
        self.initialized = False
        self.has_shutdown = False

        # Prepare input & output topics
        # todo: what is inside kwargs? Simulator?
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
        # self.backend.on_shutdown(self.node_shutdown)

    def node_initialized(self):
        # Notify env that node is initialized
        self.init_pub = self.backend.Publisher(self.name + "/initialized", "int64")
        self.init_pub.publish(0)

        if not self.initialized:
            self.backend.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, **kwargs):
        params = get_param_with_blocking(name, self.backend)
        rate = params["config"]["rate"]

        # Get info from engine on reactive properties
        sync = get_param_with_blocking(self.ns + "/engine/config/sync", self.backend)
        real_time_factor = get_param_with_blocking(self.ns + "/engine/config/real_time_factor", self.backend)
        simulate_delays = get_param_with_blocking(self.ns + "/engine/config/simulate_delays", self.backend)

        # Prepare input topics
        for i in params["inputs"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Prepare output topics
        for i in params["outputs"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Prepare state topics
        for i in params["states"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Prepare target topics
        for i in params["targets"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Prepare feedthrough topics
        for i in params["feedthroughs"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Convert lists to dicts
        params["inputs"] = {i["name"]: i for i in params["inputs"]}
        params["outputs"] = {i["name"]: i for i in params["outputs"]}
        params["states"] = {i["name"]: i for i in params["states"]}
        params["targets"] = {i["name"]: i for i in params["targets"]}
        params["feedthroughs"] = {i["feedthrough_to"]: i for i in params["feedthroughs"]}

        # Get node
        node_cls = load(params["node_type"])
        node = node_cls(
            ns=self.ns,
            message_broker=self.mb,
            sync=sync,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            params=params,
            **kwargs,
        )

        # Convert to tuple for reactive pipeline.
        inputs = tuple([value for key, value in params["inputs"].items()])
        outputs = tuple([value for key, value in params["outputs"].items()])
        feedthroughs = tuple([value for key, value in params["feedthroughs"].items()])
        states = tuple([value for key, value in params["states"].items()])
        targets = tuple([value for key, value in params["targets"].items()])

        return rate, inputs, outputs, feedthroughs, states, targets, node

    def _shutdown(self):
        self.backend.logdebug(f"[{self.name}] RxNode._shutdown() called.")
        self.init_pub.unregister()

    def node_shutdown(self):
        if not self.has_shutdown:
            self.backend.logdebug(f"[{self.name}] RxNode.node_shutdown() called.")
            self.backend.loginfo(f"[{self.name}] Shutting down.")
            self._shutdown()
            self.node.shutdown()
            self.mb.shutdown()
            self.backend.shutdown()
            self.has_shutdown = True


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--backend", nargs=1, help="The selected backend (format: '<module>/<BackendClass>').", type=str)
    parser.add_argument("-l", "--loglevel", nargs=1, help="The log level for the environment.", type=int, default=30)
    parser.add_argument("-e", "--env", nargs=1, help="The environment name.", type=str)
    parser.add_argument("-n", "--name", nargs=1, help="The node name.", type=str)
    parser.add_argument(
        "-o",
        "--object",
        nargs=1,
        help="The object name if the node is part of an engine-specific implementation.",
        type=str,
        default=[""],
    )
    args, unknown = parser.parse_known_args()
    backend_id = args.backend[0]
    log_level = args.loglevel[0]
    ns = f"/{args.env[0]}"
    name = args.name[0]
    object_name = args.object[0]

    backend = Backend.from_cmd(ns, backend_id, log_level)

    message_broker = eagerx.core.rx_message_broker.RxMessageBroker(owner=f"{ns}/{name}", backend=backend)

    pnode = None

    try:
        pnode = RxNode(name=f"{ns}/{name}", message_broker=message_broker, object_name=f"{ns}/{object_name}")

        message_broker.connect_io()

        pnode.node_initialized()

        backend.spin()
    finally:
        if pnode is not None and not pnode.has_shutdown:
            backend.loginfo(f"Terminating '{ns}/{name}'")
            pnode.node_shutdown()
