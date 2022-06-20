# RxEAGER
from eagerx.core.constants import process

# OTHER
import importlib
import subprocess
from time import sleep
from typing import List, Dict, Union, Any
from functools import partial


def launch_node_as_subprocess(executable: str, bnd_params: str, ns: str, name: str, object_name: str):
    node_type, file = executable.split(":=")
    if "python" in node_type:
        if ".py" not in file:
            file = importlib.import_module(file).__file__
    p = subprocess.Popen([file] + [bnd_params, ns, name, object_name])
    return p


def initialize_nodes(
    nodes: Union[Union[Any, Dict], List[Union[Any, Dict]]],
    process_id: int,
    ns: str,
    message_broker: Any,
    is_initialized: Dict,
    sp_nodes: Dict,
    launch_nodes: Dict,
    rxnode_cls: Any = None,
    node_args: Dict = None,
    object_name: str = "",
):
    if rxnode_cls is None:
        from eagerx.core.executable_node import RxNode

        rxnode_cls = RxNode

    from eagerx.core.specs import BaseNodeSpec

    if isinstance(nodes, (BaseNodeSpec, dict)):
        nodes = [nodes]

    for node in nodes:
        # Check if we still need to upload params to param server (env)
        if not isinstance(node, dict):
            params = node.build(ns=ns)

            # Check if node name is unique
            name = node.config.name
            assert message_broker.bnd.get_param(f"{ns}/{name}/rate", None) is None, (
                f"Node name '{ns + '/' + name}' already exists. " "Node names must be unique."
            )

            # Upload params to param server
            message_broker.bnd.upload_params(ns, params)

            # Make params consistent when directly grabbing params from param server
            params = params[name]
        else:
            params = node
            name = params["name"]
        # Flag to check if node is initialized
        is_initialized[name] = False

        # Verify that node name is unique
        node_address = ns + "/" + name
        assert (node_address not in sp_nodes) and (node_address not in launch_nodes), (
            'Node "%s" already exists. Node names must be unique!' % name
        )

        # Block env until all nodes are initialized
        def initialized(msg, name):
            is_initialized[name] = True

        sub = message_broker.bnd.Subscriber(node_address + "/initialized", "int64", partial(initialized, name=name))
        message_broker.subscribers.append(sub)

        # Initialize node
        if params["process"] == process_id:  # Initialize inside this process
            if node_args is None:
                node_args = dict()
            sp_nodes[node_address] = rxnode_cls(name=node_address, message_broker=message_broker, **node_args)
            sp_nodes[node_address].node_initialized()
        elif (
            params["process"] == process.NEW_PROCESS and process_id == process.ENVIRONMENT
        ):  # Only environment can launch new processes (as it is the main_thread)
            assert "executable" in params, (
                'No executable defined. Node "%s" can only be launched as a separate process if an executable is specified.'
                % name
            )
            bnd_params = message_broker.bnd.spec_string
            launch_nodes[node_address] = launch_node_as_subprocess(params["executable"], bnd_params, ns, name, object_name)
        elif params["process"] == process.EXTERNAL:
            message_broker.bnd.loginfo(
                'Node "%s" must be manually launched as the process is specified as process.EXTERNAL' % name
            )
        # else: node is launched in another (already launched) node's process (e.g. engine process).


def wait_for_node_initialization(is_initialized, backend, wait_time=0.3):
    iter = 0

    # Wait for nodes to be initialized
    while True:
        if iter == 0:
            sleep(0.1)
        else:
            sleep(wait_time)
        iter += 1
        not_init = []
        for name, flag in is_initialized.items():
            if not flag:
                not_init.append(name)
        if len(not_init) > 0:
            backend.loginfo_once('Waiting for nodes "%s" to be initialized.' % (str(not_init)))
        else:
            break
