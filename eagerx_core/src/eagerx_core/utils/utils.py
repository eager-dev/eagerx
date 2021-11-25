# ROS SPECIFIC
import rosparam
import rospkg
import rospy
from rosgraph.masterapi import Error
from roslaunch.substitution_args import resolve_args, _resolve_args
from roslaunch.substitution_args import _collect_args

# OTHER
import time
from functools import reduce
import importlib
import inspect
from time import sleep
from six import raise_from
from copy import deepcopy


def get_attribute_from_module(module, attribute):
    module = importlib.import_module(module)
    attribute = getattr(module, attribute)
    return attribute


def initialize_converter(args):
    converter_args = deepcopy(args)
    converter_args.pop('converter_type')
    converter_cls = get_attribute_from_module(*args['converter_type'].split('/'))
    return converter_cls(**converter_args)


def initialize_state(args):
    state_cls = get_attribute_from_module(*args['state_type'].split('/'))
    del args['state_type']
    return state_cls(**args)


def get_opposite_msg_cls(msg_type, args):
    if isinstance(msg_type, str):
        msg_type = get_attribute_from_module(*msg_type.split('/'))
    converter_cls = get_attribute_from_module(*args['converter_type'].split('/'))
    if msg_type == converter_cls.MSG_TYPE_A:
        return converter_cls.MSG_TYPE_B
    elif msg_type == converter_cls.MSG_TYPE_B:
        return converter_cls.MSG_TYPE_A
    else:
        raise ValueError(
            'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.' %
            (msg_type, converter_cls.MSG_TYPE_A, converter_cls.MSG_TYPE_B))


def get_module_type_string(cls):
    module = inspect.getmodule(cls).__name__
    return '%s/%s' % (module, cls.__name__)


def get_cls_from_string(cls_string):
    return get_attribute_from_module(*cls_string.split('/'))


def merge_dicts(a, b):
    if isinstance(b, list):
        b.insert(0, a)
        return reduce(merge, b)
    else:
        return merge(a, b)


def merge(a, b, path=None):
    "merges b into a"
    if path is None: path = []
    for key in b:
        if key in a:
            if isinstance(a[key], dict) and isinstance(b[key], dict):
                merge(a[key], b[key], path + [str(key)])
            elif a[key] == b[key]:
                pass  # same leaf value
            else:
                raise Exception('Conflict at %s' % '.'.join(path + [str(key)]))
        else:
            a[key] = b[key]
    return a


def load_yaml(package_name, object_name):
    try:
        pp = rospkg.RosPack().get_path(package_name)
        filename = pp + "/config/" + object_name + ".yaml"
        params = rosparam.load_file(filename)[0][0]
    except Exception as ex:
        raise_from(RuntimeError(('Unable to load %s from package %s' % (object_name, package_name))), ex)
    return params


def get_param_with_blocking(name, timeout=5):
    params = None
    start = time.time()
    it = 0
    while params is None:

        try:
            params = rospy.get_param(name)
        except (Error, KeyError):
            sleep_time = 0.01
            if it % 20 == 0:
                rospy.loginfo('Parameters under namespace "%s" not (yet) uploaded on parameter server. Retry with small pause (%s s).' % (name, sleep_time))
            sleep(sleep_time)
            pass
        if time.time() - start > timeout:
            break
        it += 1
    return params


def substitute_xml_args(param):
    # substitute string
    if isinstance(param, str):
        param = resolve_args(param)
        return param

    # For every key in the dictionary (not performing deepcopy!)
    if isinstance(param, dict):
        for key in param:
            # If the value is of type `(Ordered)dict`, then recurse with the value
            if isinstance(param[key], dict):
                substitute_xml_args(param[key])
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key])


def resolve_yaml_args(arg_str, context, commands):
    valid = ['env_name', 'obj_name']
    resolved = arg_str
    for a in _collect_args(arg_str):
        splits = [s for s in a.split(' ') if s]
        if not splits[0] in valid:
            raise ValueError("Unknown substitution command [%s]. Valid commands are %s"%(a, valid))
        command = splits[0]
        args = splits[1:]
        if command in commands:
            resolved = commands[command](resolved, a, args, context)
    return resolved


def substitute_yaml_args(param, context):
    commands = {
        'env_name': lambda resolved, a, args, context: resolved.replace("$(%s)" % a, context[a]),
        'obj_name': lambda resolved, a, args, context: resolved.replace("$(%s)" % a, context[a]),
    }

    # substitute string
    if isinstance(param, str):
        param = resolve_yaml_args(param, context, commands)
        return param

    # For every key in the dictionary (not performing deepcopy!)
    if isinstance(param, dict):
        for key in param:
            # If the value is of type `(Ordered)dict`, then recurse with the value
            if isinstance(param[key], dict):
                substitute_yaml_args(param[key], context)
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key], context, commands)


def get_ROS_log_level(name):
    ns = '/'.join(name.split('/')[:2])
    return get_param_with_blocking(ns + '/log_level')
