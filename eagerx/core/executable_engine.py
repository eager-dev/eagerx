#!/usr/bin/env python3

# import os
# if bool(eval(os.environ.get("EAGERX_COLAB", "0"))):
#     import site
#
#     site.addsitedir("/opt/ros/melodic/lib/python2.7/dist-packages")
#     site.addsitedir("/usr/lib/python2.7/dist-packages")

# Rx imports
from eagerx.core.entities import Backend
import eagerx.core.rx_message_broker
import eagerx.core.rx_operators
import eagerx.core.rx_pipelines
from eagerx.utils.utils import (
    load,
    replace_None,
    initialize_processor,
    dict_to_space,
    get_param_with_blocking,
)
from eagerx.core.executable_node import RxNode
from eagerx.utils.node_utils import wait_for_node_initialization

# Other imports
import yaml
from threading import Condition
import sys


class RxEngine(object):
    def __init__(self, name, message_broker):
        self.name = name
        self.ns = "/".join(name.split("/")[:2])
        self.mb = message_broker
        self.backend = message_broker.bnd
        self.initialized = False
        self.has_shutdown = False

        # Prepare input & output topics
        (
            rate,
            inputs,
            outputs,
            states,
            node_names,
            target_addresses,
            self.engine,
        ) = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects = eagerx.core.rx_pipelines.init_engine(
            self.ns,
            rate,
            self.engine,
            inputs,
            outputs,
            states,
            node_names,
            target_addresses,
            self.mb,
        )
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)
        self.mb.connect_io()
        self.cond_reg = Condition()

        # Prepare closing routine
        # self.backend.on_shutdown(self.node_shutdown)

    def node_initialized(self):
        with self.cond_reg:
            # Wait for all nodes to be initialized
            wait_for_node_initialization(self.engine.is_initialized, self.backend)

            # Notify env that node is initialized
            if not self.initialized:
                self.init_pub = self.backend.Publisher(self.name + "/initialized", "int64")
                self.init_pub.publish(0)
                self.backend.loginfo('Node "%s" initialized.' % self.name)
                self.initialized = True

    def _prepare_io_topics(self, name):
        params = get_param_with_blocking(name, self.backend)
        node_names = params["config"]["node_names"]
        target_addresses = params["config"]["target_addresses"]
        rate = params["config"]["rate"]

        # Prepare input topics
        assert len(params["inputs"]) == 0, "Engine inputs are dynamically added."

        # Prepare output topics
        for i in params["outputs"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

        # Prepare state topics
        for i in params["states"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])

        # Convert lists to dicts
        params["inputs"] = {i["name"]: i for i in params["inputs"]}
        params["outputs"] = {i["name"]: i for i in params["outputs"]}
        params["states"] = {i["name"]: i for i in params["states"]}

        # Get node
        node_cls = load(params["node_type"])
        node = node_cls(
            ns=self.ns,
            message_broker=self.mb,
            sync=params["config"]["sync"],
            real_time_factor=params["config"]["real_time_factor"],
            simulate_delays=params["config"]["simulate_delays"],
            params=params,
            target_addresses=target_addresses,
            node_names=node_names,
        )

        # Convert to tuple for reactive pipeline.
        inputs = tuple([value for key, value in params["inputs"].items()])
        outputs = tuple([value for key, value in params["outputs"].items()])
        states = tuple([value for key, value in params["states"].items()])

        return rate, inputs, outputs, states, node_names, target_addresses, node

    def _shutdown(self):
        self.backend.logdebug(f"[{self.name}] RxEngine._shutdown() called.")
        self.init_pub.unregister()

    def node_shutdown(self):
        if not self.has_shutdown:
            self.backend.logdebug(f"[{self.name}] RxEngine.node_shutdown() called.")
            for address, node in self.engine.launch_nodes.items():
                self.backend.loginfo(f"[{self.name}] Send termination signal to '{address}'.")
                node.terminate()
            for _, rxnode in self.engine.sp_nodes.items():
                rxnode: RxNode
                if not rxnode.has_shutdown:
                    self.backend.loginfo(f"[{self.name}] Shutting down '{rxnode.name}'.")
                    rxnode.node_shutdown()
            self.backend.loginfo(f"[{self.name}] Shutting down.")
            self._shutdown()
            self.engine.shutdown()
            self.mb.shutdown()
            self.backend.shutdown()
            self.has_shutdown = True


if __name__ == "__main__":

    executable, bnd_params, ns, name, _ = sys.argv[0], sys.argv[-4], sys.argv[-3], sys.argv[-2], sys.argv[-1]

    bnd_params = replace_None(yaml.safe_load(bnd_params), to_null=False)
    backend = Backend.from_params(ns, bnd_params)

    message_broker = eagerx.core.rx_message_broker.RxMessageBroker(owner=f"{ns}/{name}", backend=backend)

    pnode = None

    try:
        pnode = RxEngine(name=f"{ns}/{name}", message_broker=message_broker)

        message_broker.connect_io()

        pnode.node_initialized()

        backend.spin()
    finally:
        if pnode is not None and not pnode.has_shutdown:
            backend.loginfo(f"Terminating '{ns}/{name}'")
            pnode.node_shutdown()
