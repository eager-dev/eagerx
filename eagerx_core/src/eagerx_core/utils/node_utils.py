# ROS SPECIFIC
import rospy
import rosparam
import roslaunch
from std_msgs.msg import UInt64

# RXEAGER
from eagerx_core.params import RxNodeParams, RxObjectParams
from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string, substitute_xml_args
from eagerx_core.node import RxNode

# OTHER
from typing import List, Dict, Union, Any
from functools import partial


def configure_connections(connections):
    # type(str) == address
    # type(dict) == action/observation
    # type(tuple) & type(tuple[0]) == RxObject
    # type(tuple) & type(tuple[0]) == RxNode
    # msg_types: A --> B --> C with A=output_type, B=topic_type, C=input_type

    for io in connections:
        source = io['source']
        target = io['target']
        converter = io['converter'] if 'converter' in io else None

        # PREPARE OUTPUT
        # Determine source address & msg_type
        if isinstance(source, str):  # address
            raise ValueError('String type specifications of the source address is currently not supported.')
            address = source
            msg_type_A = None  # Cannot infer from address only
            msg_type_B = msg_type_A
        elif isinstance(source, tuple) and isinstance(source[0], dict):  # action or state
            env_dict = source[0]
            env_cname = source[1]

            # Infer details from target
            assert converter is None, "Cannot have an input converter when the output is an action of the environment. Namely, because a space_converter is used, that must be defined in node/object.yaml."
            node = target[0]
            name = node.name
            component = target[1]
            cname = target[2]

            # Standard naming convention
            address = '%s/%s' % (name, cname)
            msg_type_B = node.params[component][cname]['msg_type']

            # Infer msg_type from space_converter
            space_converter = node.params[component][cname]['space_converter']
            msg_cls_A = get_opposite_msg_cls(msg_type_B, space_converter)
            msg_type_A = get_module_type_string(msg_cls_A)

            # Create input entry for action
            if component == 'states':
                if env_cname in env_dict['default']['states']:
                    address = env_dict['default']['states'][env_cname]
                    # assert address == env_dict['default']['states'][env_cname], 'Conflicting %s for state "%s".' % ('addresses', env_cname)
                    assert space_converter == env_dict['default']['state_converters'][env_cname], 'Conflicting %s for state "%s".' % ('space_converters', env_cname)
                    assert msg_type_B == env_dict['states'][env_cname]['msg_type'], 'Conflicting %s for state "%s".' % ('space_converters', env_cname)
                else:
                    env_dict['default']['states'][env_cname] = address
                    env_dict['default']['state_converters'][env_cname] = space_converter
                    env_dict['states'][env_cname] = {'msg_type': msg_type_B}
            else:
                assert env_cname not in env_dict['default']['outputs'], 'Action name "%s" already defined. Action names must be unique.' % cname
                env_dict['default']['outputs'][env_cname] = address
                env_dict['default']['output_converters'][env_cname] = space_converter
                env_dict['outputs'][env_cname] = {'msg_type': msg_type_B}
        elif isinstance(source, tuple) and isinstance(source[0], RxNodeParams):
            cname = source[1]
            address = source[0].params['default']['outputs'][cname]
            msg_type_A = source[0].params['outputs'][cname]['msg_type']
            if 'output_converters' in source[0].params['default'] and cname in source[0].params['default']['output_converters']:
                msg_cls_B = get_opposite_msg_cls(msg_type_A, source[0].params['default']['output_converters'][cname])
                msg_type_B = get_module_type_string(msg_cls_B)
            else:
                msg_type_B = msg_type_A
        elif isinstance(source, tuple) and isinstance(source[0], RxObjectParams):
            obj_name = source[0].name
            component = source[1]
            cname = source[2]
            address = '%s/%s/%s' % (obj_name, component, cname)
            msg_type_A = source[0].params[component][cname]['msg_type']
            msg_type_B = msg_type_A
        else:
            raise ValueError('Connection entry "%s" is misspecified/unsupported. ' % source)

        # PREPARE  CONVERTER
        if converter:
            msg_cls_C = get_opposite_msg_cls(msg_type_B, converter)
            msg_type_C = get_module_type_string(msg_cls_C)
        else:
            msg_type_C = msg_type_B

        # PREPARE INPUT
        # Set input address, check msg_type consistency, add converter
        if isinstance(target, tuple) and isinstance(target[0], dict):  # observation
            observations = target[0]
            obs_name = target[1]
            repeat = target[2]

            # Infer details from target
            assert converter is None, "Cannot have an input converter when the output is an observation of the environment. Namely, because a space_converter is used, that must be defined in node/object.yaml."
            node = source[0]
            component = source[1]
            cname = source[2]

            # Infer msg_type from space_converter
            space_converter = node.params[component][cname]['space_converter']
            msg_cls_C = get_opposite_msg_cls(msg_type_B, space_converter)
            msg_type_C = get_module_type_string(msg_cls_C)

            # Create input entry for action
            observations['default']['inputs'][obs_name] = address
            observations['default']['input_converters'][obs_name] = space_converter
            observations['inputs'][obs_name] = {'msg_type': msg_type_B, 'repeat': repeat}
        elif isinstance(target, tuple) and isinstance(target[0], RxNodeParams):
            node = target[0]
            component = target[1]
            cname = target[2]

            # Add address
            if node.params['default'][component] is None:
                node.params['default'][component] = dict()
            node.params['default'][component][cname] = address

            # Add converter if specified
            if converter:
                converter_key = '%s_converters' % component[:-1]
                if converter_key not in node.params['default']:
                    node.params['default'][converter_key] = dict()
                node.params['default'][converter_key][cname] = converter

            # Verify that msg_type after converter matches the one specified in the .yaml
            if msg_type_C:
                type_out = get_cls_from_string(msg_type_C)
                type_yaml = get_cls_from_string(node.params[component][cname]['msg_type'])
                assert type_out == type_yaml, 'Msg_type (after conversion?) "%s" does not match msg_type "%s" specified in the .yaml of node "%s".' % (type_out, type_yaml, node.name)
        elif isinstance(target, tuple) and isinstance(target[0], RxObjectParams):
            obj = target[0]
            component = target[1]
            cname = target[2]

            # Add address & converter --> add to every bridge
            if component == 'states':
                for key, bridge_params in obj.params.items():
                    if key in ['default', 'sensors', 'actuators', 'states']: continue
                    state_input = bridge_params[component][cname]['state_input']
                    if 'states' not in bridge_params[component][cname]:
                        bridge_params[component][cname]['states'] = dict()
                    bridge_params[component][cname]['states'][state_input] = address

                    if converter:
                        if 'state_converters' not in bridge_params[component][cname]:
                            bridge_params[component][cname]['state_converters'] = dict()
                        bridge_params[component][cname]['state_converters'][state_input] = converter
            elif component == 'actuators':
                for key, bridge_params in obj.params.items():
                    if key in ['default', 'sensors', 'actuators', 'states']: continue
                    actuator_input = bridge_params[component][cname]['actuator_input']
                    if 'inputs' not in bridge_params[component][cname]:
                        bridge_params[component][cname]['inputs'] = dict()
                    bridge_params[component][cname]['inputs'][actuator_input] = address

                    if converter:
                        if 'input_converters' not in bridge_params[component][cname]:
                            bridge_params[component][cname]['input_converters'] = dict()
                        bridge_params[component][cname]['input_converters'][actuator_input] = converter

            # Verify that msg_type after converter matches the one specified in the .yaml
            if msg_type_C:
                type_out = get_cls_from_string(msg_type_C)
                type_yaml = get_cls_from_string(obj.params[component][cname]['msg_type'])
                assert type_out == type_yaml, 'Msg_type (after conversion?) "%s" does not match msg_type "%s" specified in the .yaml of node "%s".' % (type_out, type_yaml, node.name)
        else:
            raise ValueError('Connection entry "%s" is misspecified/unsupported. ' % target)


def launch_node(launch_file, args):
    cli_args = [substitute_xml_args(launch_file)] + args
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    return launch


def wait_for_node_initialization(is_initialized):
    # Wait for nodes to be initialized
    while True:
        rospy.sleep(1.0)
        not_init = []
        for name, flag in is_initialized.items():
            if not flag:
                not_init.append(name)
        if len(not_init) > 0:
            rospy.loginfo('Waiting for nodes "%s" to be initialized.' % (str(not_init)))
        else:
            break


def initialize_nodes(nodes: Union[Union[RxNodeParams, Dict], List[Union[RxNodeParams, Dict]]],
                     ns: str,
                     owner: str,
                     message_broker: Any,
                     is_initialized: Dict,
                     sp_nodes: Dict,
                     launch_nodes: Dict):
    if isinstance(nodes, RxNodeParams):
        nodes = [nodes]

    for node in nodes:
        # Check if we still need to upload params to rosparam server (env)
        if not isinstance(node, dict):
            params = node.get_params(ns=ns)

            # Check if node name is unique
            name = node.name
            assert rospy.get_param(('%s/%s/rate') % (ns, name), None) is None, 'Node name "%s" already exists. Node names must be unique.' % ns + '/' + name

            # Upload params to rosparam server
            rosparam.upload_params(ns, params)

            # Make params consistent when directly grabbing params from rosparam server
            params = params[name]
        else:
            params = node
            name = params['name']
        # name = params[list(params.keys())[0]]['name']  # todo: how to robustly grasp node_name
        launch_file = params['launch_file']
        launch_locally = params['launch_locally']
        single_process = params['single_process']

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
        if single_process:  # Initialize inside this process
            sp_nodes[node_address] = RxNode(name=node_address, message_broker=message_broker, scheduler=None)
        else:
            if launch_locally and launch_file:  # Launch node as separate process
                launch_nodes[ns + '/' + name] = launch_node(launch_file, args=['node_name:=' + name,
                                                                               'owner:=' + owner])
                launch_nodes[ns + '/' + name].start()