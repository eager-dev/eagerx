# ROS SPECIFIC
import roslaunch
import rospy
import rosparam
from roslaunch.core import RLException
from std_msgs.msg import UInt64

# RXEAGER
from eagerx_core.params import RxNodeParams, RxObjectParams
from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string, \
    substitute_xml_args
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


def configure_connections(connections):
    # msg_types: A --> B --> C with A=output_type, <output_converter>, B=topic_type, <input_converter>, C=input_type
    for io in connections:
        source = io['source']
        target = io['target']
        converter = io['converter'] if 'converter' in io else None
        delay = io['delay'] if 'delay' in io else None
        repeat = io['repeat'] if 'repeat' in io else None

        # PROCESS SOURCE
        if isinstance(source[0], RxNodeParams):  # source=Node
            node = source[0]
            cname = source[1]
            address = '%s/outputs/%s' % (node.name, cname)

            # Grab output entry
            if not node.name == 'env/actions':
                msg_type_A = node.params['outputs'][cname]['msg_type']
                if 'converter' in node.params['outputs'][cname]:
                    msg_type_B = get_module_type_string(get_opposite_msg_cls(msg_type_A, node.params['outputs'][cname]['converter']))
                else:
                    msg_type_B = msg_type_A
                assert cname in node.params['default']['outputs'], '"%s" was not selected as %s in node "%s" during its initialization.' % (cname, 'output', node.name)
        elif isinstance(source[0], RxObjectParams):  # source=Object
            obj = source[0]
            component = source[1]
            cname = source[2]
            address = '%s/%s/%s' % (obj.name, component, cname)
            msg_type_A = obj.params[component][cname]['msg_type']
            msg_type_B = msg_type_A
            assert cname in obj.params['default'][component], '"%s" was not selected in %s of object "%s" during its initialization.' % (cname, component, obj.name)
        else:
            raise ValueError('Connection entry "%s" is misspecified/unsupported. Source and target must either be an object or node.' % io)

        # PREPARE TARGET
        if isinstance(target[0], RxNodeParams) and target[0].name == 'env/observations':  # target=observations node
            node = target[0]
            cname = target[1]
            assert converter is None, "Cannot have an input converter when the source is an observation of the environment. Namely, because a space_converter is used, that must be defined in node/object.yaml."

            # Infer target properties from source
            if isinstance(source[0], RxNodeParams):
                space_converter = source[0].params['outputs'][source[1]]['space_converter']
            elif isinstance(source[0], RxObjectParams):
                space_converter = source[0].params['sensors'][source[2]]['space_converter']
            else:
                raise ValueError('Cannot infer properties from source "%s".' % source)
            msg_type_C = get_module_type_string(get_opposite_msg_cls(msg_type_B, space_converter))
            node.params['default']['inputs'].append(cname)
            node.params['inputs'][cname] = dict(address=address, msg_type=msg_type_B, converter=space_converter)
            if delay:
                node.params['inputs'][cname]['delay'] = delay
            if repeat:
                node.params['inputs'][cname]['repeat'] = repeat
        elif isinstance(target[0], RxNodeParams) and not target[0].name == 'env/observations':  # target=Node
            node = target[0]
            component = target[1]
            cname = target[2]

            if source[0].name == 'env/actions':  # source=actions node
                assert component in ['inputs', 'feedthroughs'], 'Cannot connect an action to anything other than inputs/feedthroughs. Entry "%s" is misspecified.' % io
                action_node = source[0]
                action_cname = source[1]
                assert action_cname != 'set', 'Invalid action name. Name "set" is a reserved action name. Please specify a different name.'

                if component == 'feedthroughs':
                    action_component = 'outputs'
                else:
                    action_component = 'inputs'

                # Infer source properties from target node
                msg_type_B = target[0].params[action_component][cname]['msg_type']
                space_converter = target[0].params[action_component][cname]['space_converter']
                msg_type_A = get_module_type_string(get_opposite_msg_cls(msg_type_B, space_converter))
                if converter:  # Overwrite msg_type_B if input converter specified
                    msg_type_B = get_module_type_string(get_opposite_msg_cls(msg_type_B, converter))
                if action_cname in action_node.params['outputs']:
                    assert get_cls_from_string(msg_type_B) == get_cls_from_string(action_node.params['outputs'][action_cname]['msg_type']), 'Conflicting %s for action "%s". Occurs with connection %s' % ('msg_types', action_cname, io)
                    if not space_converter == action_node.params['outputs'][action_cname]['converter']:
                        # Overwrite msg_type_B if input converter specified
                        rospy.logwarn('Conflicting %s for action "%s". Overwriting space_converter of %s[%s][%s]' % ('space_converters', action_cname, node.name, component, cname))
                        target[0].params[action_component][cname]['space_converter'] = action_node.params['outputs'][action_cname]['converter']
                else:
                    action_node.params['default']['outputs'].append(action_cname)
                    action_node.params['outputs'][action_cname] = dict(msg_type=msg_type_B, converter=space_converter)

            # Fill in target node properties
            node.params[component][cname]['address'] = address
            assert (component == 'feedthroughs' and cname in node.params['default']['outputs']) or cname in node.params['default'][component], '"%s" was not selected in %s of node "%s" during its initialization.' % (cname, component, node.name)
            if converter:
                node.params[component][cname]['converter'] = converter
            assert delay is None or (delay is not None and component == 'inputs'), 'Cannot specify a delay for entry "%s".' % io
            if delay:
                node.params['inputs'][cname]['delay'] = delay
            assert repeat is None or (repeat is not None and component == 'inputs'), 'Cannot specify a repeat for entry "%s".' % io
            if repeat:
                node.params['inputs'][cname]['repeat'] = repeat
            if component == 'feedthroughs':
                msg_type_C = node.params['outputs'][cname]['msg_type']
            else:
                msg_type_C = node.params[component][cname]['msg_type']

            # Verify that msg_type after converter matches the one specified in the .yaml
            if msg_type_C and not component == 'feedthroughs':
                type_out = get_cls_from_string(msg_type_C)
                type_yaml = get_cls_from_string(node.params[component][cname]['msg_type'])
                assert type_out == type_yaml, 'Msg_type (after conversion?) "%s" does not match msg_type "%s" specified in the .yaml of node "%s".' % (type_out, type_yaml, node.name)
        elif isinstance(target[0], RxObjectParams):  # target=Object
            obj = target[0]
            component = target[1]
            cname = target[2]

            if source[0].name == 'env/actions':  # source=actions node
                action_node = source[0]
                action_cname = source[1]
                assert action_cname != 'set', 'Invalid action name. Name "set" is a reserved action name. Please specify a different name.'

                # Infer source properties from target object
                msg_type_B = target[0].params['actuators'][cname]['msg_type']
                space_converter = target[0].params['actuators'][cname]['space_converter']
                msg_type_A = get_module_type_string(get_opposite_msg_cls(msg_type_B, space_converter))
                if converter:  # Overwrite msg_type_B if input converter specified
                    msg_type_B = get_module_type_string(get_opposite_msg_cls(msg_type_B, converter))
                if action_cname in action_node.params['outputs']:
                    assert get_cls_from_string(msg_type_B) == get_cls_from_string(action_node.params['outputs'][action_cname]['msg_type']), 'Conflicting %s for action "%s".' % ('msg_types', action_cname)
                    if not space_converter == action_node.params['outputs'][action_cname]['converter']:
                        # Overwrite msg_type_B if input converter specified
                        rospy.logwarn('Conflicting %s for action "%s". Overwriting space_converter of %s[%s][%s]' % ( 'space_converters', action_cname, obj.name, component, cname))
                        target[0].params['actuators'][cname]['space_converter'] = action_node.params['outputs'][action_cname]['converter']
                else:
                    action_node.params['default']['outputs'].append(action_cname)
                    action_node.params['outputs'][action_cname] = dict(msg_type=msg_type_B, converter=space_converter)

            # Fill in target object properties
            obj.params[component][cname]['address'] = address
            assert cname in obj.params['default'][component], '"%s" was not selected in %s of object "%s" during its initialization.' % (cname, component, obj.name)
            if converter:
                obj.params[component][cname]['converter'] = converter
            assert delay is None or (delay is not None and component == 'actuators'), 'Cannot specify a delay for entry "%s".' % io
            if delay:
                obj.params['actuators'][cname]['delay'] = delay
            assert repeat is None or (repeat is not None and component == 'actuators'), 'Cannot specify a repeat for entry "%s".' % io
            if repeat:
                obj.params['actuators'][cname]['repeat'] = repeat
            msg_type_C = obj.params[component][cname]['msg_type']

            # Verify that msg_type after converter matches the one specified in the .yaml
            if msg_type_C:
                type_out = get_cls_from_string(msg_type_C)
                type_yaml = get_cls_from_string(obj.params[component][cname]['msg_type'])
                assert type_out == type_yaml, 'Msg_type (after conversion?) "%s" does not match msg_type "%s" specified in the .yaml of object "%s".' % (type_out, type_yaml, obj.name)
        else:
            raise ValueError('Connection entry "%s" is misspecified/unsupported. Source and target must either be an object or node.' % io)


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
            rospy.loginfo('Waiting for nodes "%s" to be initialized.' % (str(not_init)))
        else:
            break


