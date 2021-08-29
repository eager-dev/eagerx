from roslaunch.substitution_args import resolve_args
from roslaunch.core import RLException
from rosgraph.masterapi import Error
import importlib
import roslaunch
import rospy
import rospkg
import rosparam
from six import raise_from


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


def load_yaml(package_name, object_name):
    try:
        pp = rospkg.RosPack().get_path(package_name)
        filename = pp + "/config/" + object_name + ".yaml"
        params = rosparam.load_file(filename)[0][0]
    except Exception as ex:
        raise_from(RuntimeError(('Unable to load %s from package %s' % (object_name, package_name))), ex)
    return params


def get_param_with_blocking(name):
    params = None
    while params is None:
        try:
            params = rospy.get_param(name)
        except Error:
            sleep_time = 0.01
            rospy.loginfo('Parameters under namespace "%s" not (yet) uploaded on parameter server. Retry with small pause (%s s).' % (name, sleep_time))
            rospy.sleep(sleep_time)
            pass
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