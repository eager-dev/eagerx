# ROS SPECIFIC
import roslaunch
import rospy
import rospkg
import rosparam
from roslaunch.substitution_args import resolve_args
from roslaunch.core import RLException
from rosgraph.masterapi import Error

# OTHER
from functools import reduce
import importlib
from six import raise_from
import inspect
import time


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


def get_attribute_from_module(module, attribute):
    module = importlib.import_module(module)
    attribute = getattr(module, attribute)
    return attribute


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
            rospy.sleep(sleep_time)
            pass
        if time.time() - start > timeout:
            break
        it += 1
    return params


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


def initialize_converter(args):
    converter_cls = get_attribute_from_module(*args['converter_type'].split('/'))
    del args['converter_type']
    return converter_cls(**args)


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