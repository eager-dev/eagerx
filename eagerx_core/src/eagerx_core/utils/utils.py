# ROS SPECIFIC
import rosparam
import rospkg
import rospy
from rosgraph.masterapi import Error
from roslaunch.substitution_args import resolve_args, _resolve_args
from roslaunch.substitution_args import _collect_args
from std_msgs.msg import Bool

# OTHER
from typing import List, NamedTuple, Any, Optional, Dict, Union
import time
import importlib
import inspect
import glob
from functools import reduce
from time import sleep
from six import raise_from
from copy import deepcopy
import os

from eagerx_core import constants

def get_attribute_from_module(attribute, module=None):
    if module is None:
        module, attribute = attribute.split('/')
    module = importlib.import_module(module)
    attribute = getattr(module, attribute)
    return attribute


def initialize_converter(args):
    converter_args = deepcopy(args)
    converter_args.pop('converter_type')
    converter_cls = get_attribute_from_module(args['converter_type'])
    return converter_cls(**converter_args)


def initialize_state(args):
    state_cls = get_attribute_from_module(args['state_type'])
    del args['state_type']
    return state_cls(**args)


def get_opposite_msg_cls(msg_type, converter_cls):
    if isinstance(msg_type, str):
        msg_type = get_attribute_from_module(msg_type)
    if isinstance(converter_cls, dict):
        converter_cls = get_attribute_from_module(converter_cls['converter_type'])
    return converter_cls.get_opposite_msg_type(converter_cls, msg_type)


def get_module_type_string(cls):
    module = inspect.getmodule(cls).__name__
    return '%s/%s' % (module, cls.__name__)


def get_cls_from_string(cls_string):
    return get_attribute_from_module(cls_string)


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


def get_list_of_files(dirName):
    # create a list of file and sub directories
    # names in the given directory
    list_of_file = os.listdir(dirName)
    all_files = list()
    # Iterate over all the entries
    for entry in list_of_file:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of files in this directory
        if os.path.isdir(fullPath):
            all_files = all_files + get_list_of_files(fullPath)
        else:
            all_files.append(fullPath)

    return all_files


def load_yaml(package_name, object_name):
    try:
        pp = rospkg.RosPack().get_path(package_name)
        all_files = get_list_of_files(pp + '/config')
        all_filenames = dict()
        for path in all_files:
            filename = path.split('/')[-1]
            assert filename not in all_filenames or object_name + '.yaml' != filename, 'Config file "%s" exists multiple times in "%s" and its subdirectories. ' % (filename, pp + '/config')
            all_filenames[filename] = path
        config_filename = all_filenames[object_name + '.yaml']
        params = rosparam.load_file(config_filename)[0][0]
    except Exception as ex:
        raise_from(RuntimeError(('Unable to load %s from package %s/config' % (object_name, package_name))), ex)
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


def get_yaml_type(yaml):
    if 'node_type' in yaml:
        if 'targets' in yaml:
            type = 'reset_node'
        else:
            type = 'node'
    else:
        type = 'object'
    return type


def get_nodes_and_objects_library():
    library = {'reset_node': {}, 'node': {}, 'object': {}}
    config_dict = {}
    ros_package_list = rospkg.RosPack().list()
    eagerx_packages = [package for package in ros_package_list if 'eagerx' in package]
    for package in eagerx_packages:
        file_type = '.yaml'
        package_path = substitute_xml_args('$(find {})'.format(package))
        files = glob.glob(package_path + "/config/**/*{}".format(file_type), recursive=True)
        config_dict[package] = [path.split('/')[-1][:-len(file_type)] for path in files]
    for package, configs in config_dict.items():
        for config in configs:
            if package in constants.GUI_CONFIG_TO_IGNORE and config in constants.GUI_CONFIG_TO_IGNORE[package]:
                continue
            if package is not None and config is not None:
                yaml = load_yaml(package, config)
                yaml_type = get_yaml_type(yaml)
                if package not in library[yaml_type].keys():
                    library[yaml_type][package] = []
                library[yaml_type][package].append({'name': config, 'default': yaml})
    return library


Stamp = NamedTuple('Stamp', [('seq', int), ('sim_stamp', float), ('wc_stamp', float)])
Stamp.__new__.__defaults__ = (None,) * len(Stamp._fields)
Info = NamedTuple('Info', [('name', str), ('node_tick', int), ('rate_in', float), ('t_node', List[Stamp]), ('t_in', List[Stamp]), ('done', bool)])
Info.__new__.__defaults__ = (None,) * len(Info._fields)
Msg = NamedTuple('Msg', [('info', Info), ('msgs', List[Any])])


def arg_typehint(msg_type):
    return Optional[NamedTuple('Msg', [('info', Info), ('msgs', List[msg_type])])]


def return_typehint(msg_type, done=True):
    if done:
        return Optional[Dict[str, Union[msg_type, Bool]]]
    else:
        return Optional[Dict[str, msg_type]]


def check_msg_type(name, component, cname, node_cls, msg_type):
    msg_type_yaml = get_cls_from_string(msg_type)
    try:
        msg_type_py = node_cls.get_msg_type(node_cls, component, cname)
        node_str = get_module_type_string(node_cls)
        assert msg_type_py == msg_type_yaml, 'Inconsistent msg types (.py="%s" vs (converted) .yaml="%s") specified for node "%s". \n Hint: compare the msg_types within python class "%s" with the msg_types specified in the .yaml under [%s][%s].' % (msg_type_py, msg_type_yaml, name, node_str, component, cname)
    except Exception as e:
        if name in ['env/supervisor', 'env/observations', 'env/actions']:
            return
        else:
            raise(e)
