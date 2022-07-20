# RxEAGER
from eagerx.core.constants import process, EXTERNAL, ENVIRONMENT, NEW_PROCESS, BackendException

# OTHER
import importlib
import subprocess
from time import sleep
from typing import List, Dict, Union, Any, TYPE_CHECKING
from functools import partial

if TYPE_CHECKING:
    from eagerx.core.entities import Backend


def get_launch_cmd(executable: str, bnd: "Backend", ns: str, name: str, object_name: str, external: bool = False):
    node_type, file = executable.split(":=")
    if "python" in node_type:
        if ".py" not in file:
            file = importlib.import_module(file).__file__
    if external:
        file = "/".join(["<path>", "<to>", "<package>"] + file.split("/")[-3:])

    cmd_args = [
        file,
        "--backend",
        f"{bnd.entity_id}",
        "--loglevel",
        f"{bnd.log_level}",
        "--env",
        f"{ns.split('/')[1]}",
        "--name",
        f"{name}",
    ]

    if len(object_name) > 0:
        cmd_args += ["--object", f"{object_name}"]

    return cmd_args


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

    bnd = message_broker.bnd

    for node in nodes:
        # Check if we still need to upload params to param server (env)
        if not isinstance(node, dict):
            params = node.build(ns=ns)

            # Check if node name is unique
            name = node.config.name
            assert message_broker.bnd.get_param(f"{ns}/{name}/rate", None) is None, (
                f"Node name '{ns + '/' + name}' already exists. " "Node names must be unique."
            )

            # If no backend multiprocessing support, overwrite NEW_PROCESS to ENVIRONMENT
            if node.config.process == NEW_PROCESS and not bnd.MULTIPROCESSING_SUPPORT:
                bnd.logwarn_once(
                    f"Backend '{bnd.BACKEND}' does not support multiprocessing, "
                    "so all nodes are launched in the ENVIRONMENT process."
                )
                params[name]["config"]["process"] = ENVIRONMENT
            elif node.config.process == EXTERNAL and not bnd.DISTRIBUTED_SUPPORT:
                raise BackendException(
                    f"Backend '{bnd.BACKEND}' does not support distributed computation. "
                    f"Therefore, this backend is incompatible with node '{name}', "
                    f"because {name}.config.process=EXTERNAL."
                )

            # Upload params to param server
            message_broker.bnd.upload_params(ns, params)

            # Make params consistent when directly grabbing params from param server
            params = params[name]
        else:
            params = node
            name = params["config"]["name"]
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
        if params["config"]["process"] == process_id:  # Initialize inside this process
            if node_args is None:
                node_args = dict()
            sp_nodes[node_address] = rxnode_cls(name=node_address, message_broker=message_broker, **node_args)
            sp_nodes[node_address].node_initialized()
        elif (
            params["config"]["process"] == process.NEW_PROCESS and process_id == process.ENVIRONMENT
        ):  # Only environment can launch new processes (as it is the main_thread)
            assert "executable" in params["config"], (
                'No executable defined. Node "%s" can only be launched as a separate process if an executable is specified.'
                % name
            )
            cmd = get_launch_cmd(params["config"]["executable"], bnd, ns, name, object_name, external=False)
            launch_nodes[node_address] = subprocess.Popen(cmd)
        elif params["config"]["process"] == process.EXTERNAL:
            cmd = get_launch_cmd(params["config"]["executable"], bnd, ns, name, object_name, external=True)
            cmd_joined = " ".join(cmd).replace("\n", "\\n")
            message_broker.bnd.loginfo(f'Launch node "{name}" externally with: python3 {cmd_joined}')
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
