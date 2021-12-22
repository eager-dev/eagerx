from yaml import dump
import rospy
from eagerx_core.params import RxNodeParams, RxObjectParams
from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string
from eagerx_core.constants import source_types, target_types


def register_connections(connections, as_nodes=False):
    # msg_types: A --> B --> C with A=output_type, <output_converter>, B=topic_type, <input_converter>, C=input_type
    for io in connections:
        src_type, target_type = determine_connection_types(io['source'], io['target'], as_nodes)
        connect(src_type, target_type, **io)


def determine_connection_types(source, target, as_nodes):
    # Determine source type
    if isinstance(source[0], RxObjectParams):
        if source[1] == 'states':
            src_type = source_types.STATE
        elif source[1] == 'sensors':
            src_type = source_types.SENSOR
        else:
            raise ValueError('Connection source entry "%s" is misspecified/unsupported. Source must either be an object or node.' % source)
    elif isinstance(source[0], RxNodeParams):
        if not as_nodes and source[0].name == 'env/actions':
            # Treating 'env/actions' different than other nodes.
            src_type = source_types.ACTION
        else:
            # Treating 'env/actions' as any other node.
            src_type = source_types.OUTPUT
    else:
        raise ValueError('Connection source entry "%s" is misspecified/unsupported. Source must either be an object or node.' % source)

    # Determine target type
    if isinstance(target[0], RxObjectParams):
        target_type = target_types.ACTUATOR
    elif isinstance(target[0], RxNodeParams):
        if not as_nodes and target[0].name == 'env/observations':
            # Treating 'env/observations' different than other nodes.
            target_type = target_types.OBSERVATION
        else:
            # Treating 'env/actions' as any other node.
            if target[1] == 'inputs':
                target_type = target_types.INPUT
            elif target[1] == 'feedthroughs':
                target_type = target_types.FEEDTHROUGH
            elif target[1] == 'targets':
                target_type = target_types.TARGET
            else:
                raise ValueError('Connection target entry "%s" is misspecified/unsupported. Target must either be an object or node.' % target)

    else:
        raise ValueError('Connection target entry "%s" is misspecified/unsupported. Target must either be an object or node.' % target)
    return src_type, target_type


def connect(src_type, target_type, source, target, converter=None, delay=None, window=None):
    if src_type == source_types.OUTPUT and target_type == target_types.INPUT:
        msg_types = output_to_input(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.OUTPUT and target_type == target_types.ACTUATOR:
        msg_types = output_to_actuator(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.OUTPUT and target_type == target_types.FEEDTHROUGH:
        msg_types = output_to_feedthrough(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.OUTPUT and target_type == target_types.OBSERVATION:
        msg_types = output_to_observation(*source, *target, converter=converter, delay=delay, window=window)

    elif src_type == source_types.ACTION and target_type == target_types.INPUT:
        msg_types = action_to_input(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.ACTION and target_type == target_types.ACTUATOR:
        msg_types = action_to_actuator(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.ACTION and target_type == target_types.FEEDTHROUGH:
        msg_types = action_to_feedthrough(*source, *target, converter=converter, delay=delay, window=window)

    elif src_type == source_types.SENSOR and target_type == target_types.INPUT:
        msg_types = sensor_to_input(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.SENSOR and target_type == target_types.ACTUATOR:
        msg_types = sensor_to_actuator(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.SENSOR and target_type == target_types.FEEDTHROUGH:
        msg_types = sensor_to_feedthrough(*source, *target, converter=converter, delay=delay, window=window)
    elif src_type == source_types.SENSOR and target_type == target_types.OBSERVATION:
        msg_types = sensor_to_observation(*source, *target, converter=converter, delay=delay, window=window)

    elif src_type == source_types.STATE and target_type == target_types.TARGET:
        msg_types = state_to_target(*source, *target, converter=converter, delay=delay, window=window)
    else:
        raise ValueError('Connection entry "%s" is misspecified/unsupported. Cannot connect source_type "%s" with target_type "%s".' % (tuple([source, target]), src_type, target_type))
    return msg_types


def sensor_to_input(obj, obj_comp, obj_cname, node, node_comp, node_cname, converter=None, delay=None, window=None):
    assert obj_cname in obj.params['default'][obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (obj_cname, obj_comp, obj.name)
    assert node_cname in node.params['default'][node_comp], '"%s" was not selected in %s of node "%s" during its initialization.' % (node_cname, node_comp, node.name)

    address = '%s/%s/%s' % (obj.name, obj_comp, obj_cname)
    msg_type_A = get_cls_from_string(obj.params[obj_comp][obj_cname]['msg_type'])
    msg_type_B = msg_type_A  # Convert with output converter if specified (not applicable to object outputs (sensors))

    # Fill in target node properties
    node.params[node_comp][node_cname]['address'] = address
    if converter:
        node.params[node_comp][node_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B
    if delay:
        node.params[node_comp][node_cname]['delay'] = delay
    if window:
        node.params[node_comp][node_cname]['window'] = window

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(node.params[node_comp][node_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of node "%s".' % (msg_type_C, msg_type_C_yaml, node.name)
    return msg_type_A, msg_type_B, msg_type_C


def sensor_to_actuator(src_obj, src_obj_comp, src_obj_cname, target_obj, target_obj_comp, target_obj_cname, converter=None, delay=None, window=None):
    assert window is None, 'Cannot specify a window for actuators.'
    assert src_obj_cname in src_obj.params['default'][src_obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (src_obj_cname, src_obj_comp, src_obj.name)
    assert target_obj_cname in target_obj.params['default'][target_obj_comp], '"%s" was not selected in %s of target_obj "%s" during its initialization.' % (target_obj_cname, target_obj_comp, target_obj.name)

    address = '%s/%s/%s' % (src_obj.name, src_obj_comp, src_obj_cname)
    msg_type_A = get_cls_from_string(src_obj.params[src_obj_comp][src_obj_cname]['msg_type'])
    msg_type_B = msg_type_A  # Convert with output converter if specified (not applicable to object outputs (sensors))

    # Fill in target target_obj properties
    target_obj.params[target_obj_comp][target_obj_cname]['address'] = address
    if converter:
        target_obj.params[target_obj_comp][target_obj_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B
    if delay:
        target_obj.params['inputs'][target_obj_cname]['delay'] = delay

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(target_obj.params[target_obj_comp][target_obj_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of target_obj "%s".' % (msg_type_C, msg_type_C_yaml, target_obj.name)
    return msg_type_A, msg_type_B, msg_type_C


def sensor_to_feedthrough(obj, obj_comp, obj_cname, node, node_comp, node_cname, converter=None, delay=None, window=None):
    assert obj_cname in obj.params['default'][obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (obj_cname, obj_comp, obj.name)
    assert node_cname in node.params['default'][node_comp], '"%s" was not selected in %s of node "%s" during its initialization.' % (node_cname, node_comp, node.name)

    address = '%s/%s/%s' % (obj.name, obj_comp, obj_cname)
    msg_type_A = get_cls_from_string(obj.params[obj_comp][obj_cname]['msg_type'])
    msg_type_B = msg_type_A  # Convert with output converter if specified (not applicable to object outputs (sensors))

    # Fill in target node properties
    node.params[node_comp][node_cname]['address'] = address
    if converter:
        node.params[node_comp][node_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B
    if delay:
        node.params[node_comp][node_cname]['delay'] = delay
    if window:
        node.params[node_comp][node_cname]['window'] = window

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(node.params[node_comp][node_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of node "%s".' % (msg_type_C, msg_type_C_yaml, node.name)
    return msg_type_A, msg_type_B, msg_type_C


def sensor_to_observation(obj, obj_comp, obj_cname, node, node_cname, node_comp='inputs', converter=None, delay=None, window=None):
    assert obj_cname in obj.params['default'][obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (obj_cname, obj_comp, obj.name)
    assert converter is not None or 'space_converter' in obj.params[obj_comp][obj_cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s". Either specify it there, or add an input converter that acts as a space_converter to this connection entry.' % (obj_cname, obj_comp, obj.name)

    address = '%s/%s/%s' % (obj.name, obj_comp, obj_cname)
    msg_type_A = get_cls_from_string(obj.params[obj_comp][obj_cname]['msg_type'])
    msg_type_B = msg_type_A  # Convert with output converter if specified (not applicable to object outputs (sensors))
    if converter is None:
        converter = obj.params[obj_comp][obj_cname]['space_converter']
    msg_type_C = get_opposite_msg_cls(msg_type_B, converter)

    # Fill in target node properties
    node.params['default'][node_comp].append(node_cname)
    node.params[node_comp][node_cname] = dict(address=address, msg_type=get_module_type_string(msg_type_B), converter=converter)
    if delay:
        node.params[node_comp][node_cname]['delay'] = delay
    if window:
        node.params[node_comp][node_cname]['window'] = window
    return msg_type_A, msg_type_B, msg_type_C


def action_to_input(src_node, src_node_cname, target_node, target_node_comp, target_node_cname, src_node_comp='outputs', converter=None, delay=None, window=None):
    assert src_node_cname != 'set', 'Cannot define an action with the reserved word "set".'
    assert 'space_converter' in target_node.params[target_node_comp][target_node_cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".' % (target_node_cname, target_node_comp, target_node.name)
    assert target_node_cname in target_node.params['default'][target_node_comp], '"%s" was not selected in %s of target_node "%s" during its initialization.' % (target_node_cname, target_node_comp, target_node.name)

    address = '%s/%s/%s' % (src_node.name, src_node_comp, src_node_cname)
    space_converter = target_node.params[target_node_comp][target_node_cname]['space_converter']

    # Infer source properties (converter & msg_type) from target node
    msg_type_C = get_cls_from_string(target_node.params[target_node_comp][target_node_cname]['msg_type'])
    if converter:  # Overwrite msg_type_B if input converter specified
        msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
    else:
        msg_type_B = msg_type_C

    # If action is already defined as an input to another node
    if src_node_cname in src_node.params[src_node_comp]:
        space_converter_yaml = src_node.params[src_node_comp][src_node_cname]['converter']
        msg_type_B_yaml = get_cls_from_string(src_node.params[src_node_comp][src_node_cname]['msg_type'])
        assert msg_type_B == msg_type_B_yaml, 'Conflicting %s for action "%s" that is already used in another connection. Occurs with connection %s' % ('msg_types', src_node_cname, tuple([target_node, target_node_comp, target_node_cname]))
        if not space_converter == space_converter_yaml:
            rospy.logwarn('Conflicting %s for action "%s". Not using the space_converter of %s[%s][%s]' % ('space_converters', src_node_cname, target_node.name, target_node_comp, target_node_cname))
        msg_type_A = get_opposite_msg_cls(msg_type_B_yaml, space_converter_yaml)
    else:
        src_node.params['default']['outputs'].append(src_node_cname)
        src_node.params['outputs'][src_node_cname] = dict(msg_type=get_module_type_string(msg_type_B), converter=space_converter)
        msg_type_A = get_opposite_msg_cls(msg_type_C, space_converter)
        if converter:
            assert msg_type_B == msg_type_C, 'Cannot have a converter that maps to a different msg_type when also using the space_converter.'

    # Fill in target node properties
    target_node.params[target_node_comp][target_node_cname]['address'] = address
    if converter:
        target_node.params[target_node_comp][target_node_cname]['converter'] = converter
    if delay:
        target_node.params[target_node_comp][target_node_cname]['delay'] = delay
    if window:
        target_node.params[target_node_comp][target_node_cname]['window'] = window

    return msg_type_A, msg_type_B, msg_type_C


def action_to_feedthrough(src_node, src_node_cname, target_node, target_node_comp, target_node_cname, src_node_comp='outputs', converter=None, delay=None, window=None):
    target_node_comp = 'outputs'  # For feedthroughs, all information is stored under the 'outputs' component of the node.
    assert src_node_cname != 'set', 'Cannot define an action with the reserved word "set".'
    assert 'space_converter' in target_node.params[target_node_comp][target_node_cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".' % (target_node_cname, target_node_comp, target_node.name)
    assert target_node_cname in target_node.params['default'][target_node_comp], '"%s" was not selected in %s of target_node "%s" during its initialization.' % (target_node_cname, target_node_comp, target_node.name)
    assert window is None or window > 0, 'Feedthroughs must have a window > 0, else no action can be fed through.'

    address = '%s/%s/%s' % (src_node.name, src_node_comp, src_node_cname)
    space_converter = target_node.params[target_node_comp][target_node_cname]['space_converter']

    # Infer source properties (converter & msg_type) from target node
    msg_type_C = get_cls_from_string(target_node.params[target_node_comp][target_node_cname]['msg_type'])
    if converter:  # Overwrite msg_type_B if input converter specified
        msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
    else:
        msg_type_B = msg_type_C

    # If action is already defined as an input to another node
    if src_node_cname in src_node.params[src_node_comp]:
        space_converter_yaml = src_node.params[src_node_comp][src_node_cname]['converter']
        msg_type_B_yaml = get_cls_from_string(src_node.params[src_node_comp][src_node_cname]['msg_type'])
        assert msg_type_B == msg_type_B_yaml, 'Conflicting %s for action "%s" that is already used in another connection. Occurs with connection %s' % ('msg_types', src_node_cname, tuple([target_node, target_node_comp, target_node_cname]))
        if not space_converter == space_converter_yaml:
            rospy.logwarn('Conflicting %s for action "%s". Not using the space_converter of %s[%s][%s]' % ('space_converters', src_node_cname, target_node.name, target_node_comp, target_node_cname))
        msg_type_A = get_opposite_msg_cls(msg_type_B_yaml, space_converter_yaml)
    else:
        src_node.params['default']['outputs'].append(src_node_cname)
        src_node.params['outputs'][src_node_cname] = dict(msg_type=get_module_type_string(msg_type_B), converter=space_converter)
        msg_type_A = get_opposite_msg_cls(msg_type_C, space_converter)
        if converter:
            assert msg_type_B == msg_type_C, 'Cannot have a converter that maps to a different msg_type when also using the space_converter.'

    # Fill in target node properties
    target_node_comp = 'feedthroughs'  # For feedthroughs, all information is stored under the 'outputs' component of the node.
    target_node.params[target_node_comp][target_node_cname]['address'] = address
    if converter:
        target_node.params[target_node_comp][target_node_cname]['converter'] = converter
    if delay:
        target_node.params[target_node_comp][target_node_cname]['delay'] = delay
    if window:
        target_node.params[target_node_comp][target_node_cname]['window'] = window

    return msg_type_A, msg_type_B, msg_type_C


def action_to_actuator(node, node_cname, obj, obj_comp, obj_cname, node_comp='outputs', converter=None, delay=None, window=None):
    assert node_cname != 'set', 'Cannot define an action with the reserved word "set".'
    assert 'space_converter' in obj.params[obj_comp][obj_cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".' % (obj_cname, obj_comp, obj.name)
    assert obj_cname in obj.params['default'][obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (obj_cname, obj_comp, obj.name)

    address = '%s/%s/%s' % (node.name, node_comp, node_cname)
    space_converter = obj.params[obj_comp][obj_cname]['space_converter']

    # Infer source properties (converter & msg_type) from target node
    msg_type_C = get_cls_from_string(obj.params[obj_comp][obj_cname]['msg_type'])
    if converter:  # Overwrite msg_type_B if input converter specified
        msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
    else:
        msg_type_B = msg_type_C

    # If action is already defined as an input to another node/object
    if node_cname in node.params[node_comp]:
        space_converter_yaml = node.params[node_comp][node_cname]['converter']
        msg_type_B_yaml = get_cls_from_string(node.params[node_comp][node_cname]['msg_type'])
        assert msg_type_B == msg_type_B_yaml, 'Conflicting %s for action "%s" that is already used in another connection. Occurs with connection %s' % (
        'msg_types', node_cname, tuple([obj, obj_comp, obj_cname]))
        if not space_converter == space_converter_yaml:
            rospy.logwarn('Conflicting %s for action "%s". Not using the space_converter of %s[%s][%s]' % ('space_converters', node_cname, obj.name, obj_comp, obj_cname))
        msg_type_A = get_opposite_msg_cls(msg_type_B_yaml, space_converter_yaml)
    else:
        node.params['default']['outputs'].append(node_cname)
        node.params['outputs'][node_cname] = dict(msg_type=get_module_type_string(msg_type_B), converter=space_converter)
        msg_type_A = get_opposite_msg_cls(msg_type_C, space_converter)
        if converter:
            assert msg_type_B == msg_type_C, 'Cannot have a converter that maps to a different msg_type when also using the space_converter.'

    # Fill in target node properties
    obj.params[obj_comp][obj_cname]['address'] = address
    if converter:
        obj.params[obj_comp][obj_cname]['converter'] = converter
    if delay:
        obj.params[obj_comp][obj_cname]['delay'] = delay
    if window:
        obj.params[obj_comp][obj_cname]['window'] = window

    return msg_type_A, msg_type_B, msg_type_C


def output_to_input(src_node, src_node_cname, target_node, target_node_comp, target_node_cname, src_node_comp='outputs', converter=None, delay=None, window=None):
    assert target_node_cname in target_node.params['default'][target_node_comp], '"%s" was not selected in %s of target_node "%s" during its initialization.' % (target_node_cname, target_node_comp, target_node.name)
    assert src_node_cname in src_node.params['default'][src_node_comp], '"%s" was not selected in %s of src_node "%s" during its initialization.' % (src_node_cname, src_node_comp, src_node.name)

    address = '%s/%s/%s' % (src_node.name, src_node_comp, src_node_cname)
    msg_type_A = get_cls_from_string(src_node.params[src_node_comp][src_node_cname]['msg_type'])
    if 'converter' in src_node.params[src_node_comp][src_node_cname]:
        msg_type_B = get_opposite_msg_cls(msg_type_A, src_node.params[src_node_comp][src_node_cname]['converter'])
    else:
        msg_type_B = msg_type_A

    # Fill in target node properties
    target_node.params[target_node_comp][target_node_cname]['address'] = address
    if converter:
        target_node.params[target_node_comp][target_node_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B
    if delay:
        target_node.params[target_node_comp][target_node_cname]['delay'] = delay
    if window:
        target_node.params[target_node_comp][target_node_cname]['window'] = window

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(target_node.params[target_node_comp][target_node_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of target_node "%s".' % (msg_type_C, msg_type_C_yaml, target_node.name)
    return msg_type_A, msg_type_B, msg_type_C


def output_to_actuator(node, node_cname, obj, obj_comp, obj_cname, node_comp='outputs', converter=None, delay=None, window=None):
    assert obj_cname in obj.params['default'][obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (obj_cname, obj_comp, obj.name)
    assert node_cname in node.params['default'][node_comp], '"%s" was not selected in %s of node "%s" during its initialization.' % (node_cname, node_comp, node.name)

    address = '%s/%s/%s' % (node.name, node_comp, node_cname)
    msg_type_A = get_cls_from_string(node.params[node_comp][node_cname]['msg_type'])
    if 'converter' in node.params[node_comp][node_cname]:
        msg_type_B = get_opposite_msg_cls(msg_type_A, node.params[node_comp][node_cname]['converter'])
    else:
        msg_type_B = msg_type_A

    # Fill in target node properties
    obj.params[obj_comp][obj_cname]['address'] = address
    if converter:
        obj.params[obj_comp][obj_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B
    if delay:
        obj.params[obj_comp][obj_cname]['delay'] = delay
    if window:
        obj.params[obj_comp][obj_cname]['window'] = window

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(obj.params[obj_comp][obj_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of object "%s".' % (msg_type_C, msg_type_C_yaml, obj.name)
    return msg_type_A, msg_type_B, msg_type_C


def output_to_feedthrough(src_node, src_node_cname, target_node, target_node_comp, target_node_cname, src_node_comp='outputs', converter=None, delay=None, window=None):
    assert target_node_cname in target_node.params['default']['outputs'], '"%s" was not selected in %s of target_node "%s" during its initialization.' % (target_node_cname, 'outputs', target_node.name)
    assert src_node_cname in src_node.params['default'][src_node_comp], '"%s" was not selected in %s of src_node "%s" during its initialization.' % (src_node_cname, src_node_comp, src_node.name)

    address = '%s/%s/%s' % (src_node.name, src_node_comp, src_node_cname)
    msg_type_A = get_cls_from_string(src_node.params[src_node_comp][src_node_cname]['msg_type'])
    if 'converter' in src_node.params[src_node_comp][src_node_cname]:
        msg_type_B = get_opposite_msg_cls(msg_type_A, src_node.params[src_node_comp][src_node_cname]['converter'])
    else:
        msg_type_B = msg_type_A

    # Fill in target node properties
    target_node.params[target_node_comp][target_node_cname]['address'] = address
    if converter:
        target_node.params[target_node_comp][target_node_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B
    if delay:
        target_node.params[target_node_comp][target_node_cname]['delay'] = delay
    if window:
        target_node.params[target_node_comp][target_node_cname]['window'] = window

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(target_node.params['outputs'][target_node_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of target_node "%s".' % (msg_type_C, msg_type_C_yaml, target_node.name)
    return msg_type_A, msg_type_B, msg_type_C


def output_to_observation(src_node, src_node_cname, target_node, target_node_cname, target_node_comp='inputs', src_node_comp='outputs', converter=None, delay=None, window=None):
    assert src_node_cname in src_node.params['default'][src_node_comp], '"%s" was not selected in %s of src_node "%s" during its initialization.' % (src_node_cname, src_node_comp, src_node.name)
    assert converter is not None or 'space_converter' in src_node.params[src_node_comp][src_node_cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".. Either specify it there, or add an input converter that acts as a space_converter to this connection entry.' % (src_node_cname, src_node_comp, src_node.name)

    address = '%s/%s/%s' % (src_node.name, src_node_comp, src_node_cname)
    msg_type_A = get_cls_from_string(src_node.params[src_node_comp][src_node_cname]['msg_type'])
    if 'converter' in src_node.params[src_node_comp][src_node_cname]:
        msg_type_B = get_opposite_msg_cls(msg_type_A, src_node.params[src_node_comp][src_node_cname]['converter'])
    else:
        msg_type_B = msg_type_A
    if converter is None:
        converter = src_node.params[src_node_comp][src_node_cname]['space_converter']
    msg_type_C = get_opposite_msg_cls(msg_type_B, converter)

    # Fill in target node properties
    target_node.params['default'][target_node_comp].append(target_node_cname)
    target_node.params[target_node_comp][target_node_cname] = dict(address=address, msg_type=get_module_type_string(msg_type_B), converter=converter)
    if delay:
        target_node.params[target_node_comp][target_node_cname]['delay'] = delay
    if window:
        target_node.params[target_node_comp][target_node_cname]['window'] = window
    return msg_type_A, msg_type_B, msg_type_C


def state_to_target(obj, obj_comp, obj_cname, node, node_comp, node_cname, converter=None, delay=None, window=None):
    assert delay is None, 'Cannot specify a delay for targets.'
    assert window is None, 'Cannot specify a window for targets.'
    assert obj_cname in obj.params['default'][obj_comp], '"%s" was not selected in %s of object "%s" during its initialization.' % (obj_cname, obj_comp, obj.name)
    assert node_cname in node.params['default'][node_comp], '"%s" was not selected in %s of node "%s" during its initialization.' % (node_cname, node_comp, node.name)

    address = '%s/%s/%s' % (obj.name, obj_comp, obj_cname)
    msg_type_A = get_cls_from_string(obj.params[obj_comp][obj_cname]['msg_type'])
    msg_type_B = msg_type_A  # Convert with output converter if specified (not applicable to object outputs (sensors))

    # Fill in target node properties
    node.params[node_comp][node_cname]['address'] = address
    if converter:
        node.params[node_comp][node_cname]['converter'] = converter
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    else:
        msg_type_C = msg_type_B

    # Verify that msg_type after converter matches the one specified in the .yaml
    msg_type_C_yaml = get_cls_from_string(node.params[node_comp][node_cname]['msg_type'])
    assert msg_type_C == msg_type_C_yaml, 'Msg_type "%s" does not match msg_type "%s" specified in the .yaml of node "%s".' % (
    msg_type_C, msg_type_C_yaml, node.name)
    return msg_type_A, msg_type_B, msg_type_C