# ROS SPECIFIC
import roslaunch
import rospy
import rosparam
from roslaunch.core import RLException
from std_msgs.msg import UInt64

# RXEAGER
from eagerx_core.params import RxNodeParams
from eagerx_core.utils.utils import substitute_xml_args
from eagerx_core.rxnode import RxNode
from eagerx_core.constants import process

# OTHER
from time import sleep
from typing import List, Dict, Union, Any
from functools import partial


def launch_roscore():
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    roscore = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)

    try:
        roscore.start()
    except RLException as e:
        rospy.logwarn('Roscore cannot run as another roscore/master is already running. Continuing without re-initializing the roscore.')
        pass
    return roscore


def launch_node(launch_file, args):
    cli_args = [substitute_xml_args(launch_file)] + args
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)  # THIS RESETS the log level. Can we do without this line? Are ROS logs stil being made?
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    return launch


def initialize_nodes(nodes: Union[Union[RxNodeParams, Dict], List[Union[RxNodeParams, Dict]]],
                     process_id: int,
                     ns: str,
                     owner: str,
                     message_broker: Any,
                     is_initialized: Dict,
                     sp_nodes: Dict,
                     launch_nodes: Dict,
                     in_object: bool = False,
                     rxnode_cls: Any = RxNode,
                     node_args: Dict = None,
                     ):
    if isinstance(nodes, (RxNodeParams, dict)):
        nodes = [nodes]

    for node in nodes:
        # Check if we still need to upload params to rosparam server (env)
        if not isinstance(node, dict):
            params = node.get_params(ns=ns, in_object=in_object)

            # Check if node name is unique
            name = node.name
            assert rospy.get_param(('%s/%s/rate') % (ns, name), None) is None, 'Node name "%s" already exists. Node names must be unique.' % (ns + '/' + name)

            # Upload params to rosparam server
            rosparam.upload_params(ns, params)

            # Make params consistent when directly grabbing params from rosparam server
            params = params[name]
        else:
            params = node
            name = params['name']
        # Flag to check if node is initialized
        is_initialized[name] = False

        # Verify that node name is unique
        node_address = ns + '/' + name
        assert (node_address not in sp_nodes) and (node_address not in launch_nodes), 'Node "%s" already exists. Node names must be unique!' % name

        # Block env until all nodes are initialized
        def initialized(msg, name):
            is_initialized[name] = True
        rospy.Subscriber(node_address + '/initialized', UInt64, partial(initialized, name=name))

        # Initialize node
        if params['process'] == process_id:  # Initialize inside this process
            if node_args is None:
                node_args = dict()
            sp_nodes[node_address] = rxnode_cls(name=node_address, message_broker=message_broker, **node_args)
            sp_nodes[node_address].node_initialized()
        elif params['process'] == process.NEW_PROCESS and process_id == process.ENVIRONMENT:  # Only environment can launch new processes (as it is the main_thread)
            assert 'launch_file' in params, 'No launch_file defined. Node "%s" can only be launched as a separate process if a launch_file is specified.' % name
            owner_with_node_ns = '/'.join(owner.split('/') + name.split('/')[:-1])
            name_without_node_ns = name.split('/')[-1]
            launch_nodes[ns + '/' + name] = launch_node(params['launch_file'],
                                                        args=['node_name:=' + name_without_node_ns,
                                                              'owner:=' + owner_with_node_ns])
            launch_nodes[ns + '/' + name].start()
        elif params['process'] == process.EXTERNAL:
            rospy.loginfo('Node "%s" must be manually launched as the process is specified as process.EXTERNAL' % name)
        # else: node is launched in another (already launched) node's process (e.g. bridge process).


def wait_for_node_initialization(is_initialized, wait_time=0.3):
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
            rospy.loginfo_once('Waiting for nodes "%s" to be initialized.' % (str(not_init)))
        else:
            break
