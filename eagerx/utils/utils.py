# ROS SPECIFIC
import rosparam
import rospkg
import rospy
from rosgraph.masterapi import Error
from roslaunch import substitution_args as sub
from roslaunch.substitution_args import _collect_args
from std_msgs.msg import Bool

# OTHER
from typing import List, NamedTuple, Any, Optional, Dict, Union, Tuple
import time
import importlib
import inspect
from functools import reduce, wraps
from time import sleep
from six import raise_from
import copy
import ast
import json
from yaml import dump
import os


def dict_null(items):
    result = {}
    for key, value in items:
        if value is None:
            value = "null"
        result[key] = value
    return result


def dict_None(items):
    result = {}
    for key, value in items:
        if value == "null":
            value = None
        result[key] = value
    return result


def replace_None(d, to_null=True):
    dict_str = json.dumps(d)
    if to_null:
        object_pairs_hook = dict_null
    else:
        object_pairs_hook = dict_None
    return json.loads(dict_str, object_pairs_hook=object_pairs_hook)


def pretty_print(params):
    print(dump(params))


def get_attribute_from_module(attribute, module=None):
    if module is None:
        module, attribute = attribute.split("/")
    module = importlib.import_module(module)
    attribute = getattr(module, attribute)
    return attribute


def initialize_converter(args):
    converter_args = copy.deepcopy(args)
    converter_args.pop("converter_type")
    converter_cls = get_attribute_from_module(args["converter_type"])
    return converter_cls(**converter_args)


def initialize_state(args):
    state_cls = get_attribute_from_module(args["state_type"])
    del args["state_type"]
    return state_cls(**args)


def get_opposite_msg_cls(msg_type, converter_cls):
    if isinstance(msg_type, str):
        msg_type = get_attribute_from_module(msg_type)
    if isinstance(converter_cls, dict):
        converter_cls = get_attribute_from_module(converter_cls["converter_type"])
    return converter_cls.get_opposite_msg_type(converter_cls, msg_type)


def get_module_type_string(cls):
    module = inspect.getmodule(cls).__name__
    return "%s/%s" % (module, cls.__name__)


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
    if path is None:
        path = []
    for key in b:
        if key in a:
            if isinstance(a[key], dict) and isinstance(b[key], dict):
                merge(a[key], b[key], path + [str(key)])
            elif a[key] == b[key]:
                pass  # same leaf value
            else:
                a[key] = b[key]
            # else:
            #     raise Exception('Conflict at %s' % '.'.join(path + [str(key)]))
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
        all_files = get_list_of_files(pp + "/config")
        all_filenames = dict()
        for path in all_files:
            filename = path.split("/")[-1]
            assert (
                filename not in all_filenames or object_name + ".yaml" != filename
            ), 'Config file "%s" exists multiple times in "%s" and its subdirectories. ' % (filename, pp + "/config")
            all_filenames[filename] = path
        config_filename = all_filenames[object_name + ".yaml"]
        params = rosparam.load_file(config_filename)[0][0]
    except Exception as ex:
        raise_from(
            RuntimeError(("Unable to load %s from package %s/config" % (object_name, package_name))),
            ex,
        )
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
                rospy.loginfo(
                    'Parameters under namespace "%s" not (yet) uploaded on parameter server. Retry with small pause (%s s).'
                    % (name, sleep_time)
                )
            sleep(sleep_time)
            pass
        if time.time() - start > timeout:
            break
        it += 1

    return replace_None(params, to_null=False)


def substitute_args(
    param: Union[str, Dict],
    context: Optional[Dict] = None,
    only: Optional[List[str]] = None,
):
    """Substitute arguments based on the context dictionairy (and possibly sourced packages).
    Follows the xacro substition convention of ROS, with a 'default' and 'ns' command added.
    :param param: dict or string we wish to perform substitutions on.
    :param context: dict[command][context] with replacement values .
    :param only: List of possible commands. If only is not provided, all commands are executed.
                 Options are ['env', 'optenv', 'dirname', 'anon', 'arg', 'ns', 'default'].
    :return: Substituted param file
    """
    # substitute string
    if isinstance(param, str):
        param = resolve_args(param, context, only=only)
        return param

    # For every key in the dictionary (not performing copy.deepcopy!)
    if isinstance(param, dict):
        for key in param:
            # If the value is of type `(Ordered)dict`, then recurse with the value
            if isinstance(param[key], dict):
                substitute_args(param[key], context, only=only)
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key], context, only=only)
    return param


def resolve_args(arg_str, context=None, resolve_anon=True, filename=None, only=None):
    """
    Resolves substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args
        in. arg_str may be None, in which case resolve_args will
        return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of
        the 'anon' and 'arg' substitution args. multiple calls to
        resolve_args should use the same context so that 'anon'
        substitions resolve consistently. If no context is provided, a
        new one will be created for each call. Values for the 'arg'
        context should be stored as a dictionary in the 'arg' key.
    @type  context: dict
    @param resolve_anon bool: If True (default), will resolve $(anon
        foo). If false, will leave these args as-is.
    @type  resolve_anon: bool

    @return str: arg_str with substitution args resolved
    @rtype:  str
    @raise sub.SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    if not arg_str:
        return arg_str
    # special handling of $(eval ...)
    if arg_str.startswith("$(eval ") and arg_str.endswith(")"):
        return sub._eval(arg_str[7:-1], context)
    # first resolve variables like 'env' and 'arg'
    commands = {
        "env": sub._env,
        "optenv": sub._optenv,
        "dirname": sub._dirname,
        "anon": sub._anon,
        "arg": sub._arg,
        "ns": _ns,
        "default": _default,
    }
    if only is not None:
        exec_commands = {}
        for c in only:
            if c in commands:
                exec_commands[c] = commands[c]
    else:
        exec_commands = commands
    resolved = _resolve_args(arg_str, context, resolve_anon, exec_commands)
    # then resolve 'find' as it requires the subsequent path to be expanded already
    commands = {
        "find": sub._find,
    }
    if only is not None:
        exec_commands = {}
        for c in only:
            if c in commands:
                exec_commands[c] = commands[c]
    else:
        exec_commands = commands
    resolved = _resolve_args(resolved, context, resolve_anon, exec_commands)
    return resolved


def _resolve_args(arg_str, context, resolve_anon, commands):
    ros_valid = ["find", "env", "optenv", "dirname", "anon", "arg"]
    valid = ros_valid + ["ns", "default"]
    resolved = arg_str
    if isinstance(arg_str, (str, list)):
        for a in _collect_args(arg_str):
            splits = [s for s in a.split(" ") if s]
            if not splits[0] in valid:
                raise sub.SubstitutionException("Unknown substitution command [%s]. Valid commands are %s" % (a, valid))
            command = splits[0]
            args = splits[1:]
            if command in commands:
                resolved = commands[command](resolved, a, args, context)
    return resolved


def _eval_default(name, args):
    try:
        return args[name]
    except KeyError:
        raise sub.ArgException(name)


_eval_ns = _eval_default


def tryeval(val):
    try:
        val = ast.literal_eval(val)
    except Exception as e:
        if isinstance(e, ValueError):
            pass
        elif isinstance(e, SyntaxError):
            pass
        else:
            raise
    return val


def _default(resolved, a, args, context):
    """
    process $(default) arg

    :returns: updated resolved argument, ``str``
    :raises: :exc:`sub.ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise sub.SubstitutionException("$(default var) must specify a variable name [%s]" % (a))
    elif len(args) > 1:
        raise sub.SubstitutionException("$(default var) may only specify one arg [%s]" % (a))

    if "default" not in context:
        context["default"] = {}
    try:
        return tryeval(resolved.replace("$(%s)" % a, str(_eval_default(name=args[0], args=context["default"]))))
    except Exception as e:  # sub.ArgException:
        if isinstance(e, sub.ArgException):
            return resolved
        elif isinstance(e, TypeError):
            raise
        else:
            raise


def _ns(resolved, a, args, context):
    """
    process $(ns) arg

    :returns: updated resolved argument, ``str``
    :raises: :exc:`sub.ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise sub.SubstitutionException("$(ns var) must specify a variable name [%s]" % (a))
    elif len(args) > 1:
        raise sub.SubstitutionException("$(ns var) may only specify one arg [%s]" % (a))

    if "ns" not in context:
        context["ns"] = {}
    try:
        return tryeval(resolved.replace("$(%s)" % a, str(_eval_ns(name=args[0], args=context["ns"]))))
    except Exception as e:  # sub.ArgException:
        if isinstance(e, sub.ArgException):
            return resolved
        elif isinstance(e, TypeError):
            raise
        else:
            raise


def get_ROS_log_level(name):
    ns = "/".join(name.split("/")[:2])
    return get_param_with_blocking(ns + "/log_level")


def get_yaml_type(yaml):
    if "node_type" in yaml:
        if "targets" in yaml:
            type = "reset_node"
        else:
            type = "node"
    else:
        type = "object"
    return type


def get_nodes_and_objects_library():
    from eagerx.core.register import REGISTRY

    library = dict()
    for entity_cls, entities in REGISTRY.items():
        library[entity_cls.__name__] = []
        for id, entry in entities.items():
            spec = entry["spec"]
            cls = get_attribute_from_module(entry["cls"])
            library[entity_cls.__name__].append({"id": id, "spec": spec, "entity_cls": entity_cls, "cls": cls})
    return library


Stamp = NamedTuple("Stamp", [("seq", int), ("sim_stamp", float), ("wc_stamp", float)])
Stamp.__new__.__defaults__ = (None,) * len(Stamp._fields)
Info = NamedTuple(
    "Info",
    [
        ("name", str),
        ("node_tick", int),
        ("rate_in", float),
        ("t_node", List[Stamp]),
        ("t_in", List[Stamp]),
        ("done", bool),
    ],
)
Info.__new__.__defaults__ = (None,) * len(Info._fields)
Msg = NamedTuple("Msg", [("info", Info), ("msgs", List[Any])])


def arg_typehint(msg_type):
    return Optional[NamedTuple("Msg", [("info", Info), ("msgs", List[msg_type])])]


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
        assert msg_type_py == msg_type_yaml, (
            'Inconsistent msg types (.py="%s" vs (converted) .yaml="%s") specified for node "%s". \n Hint: compare the msg_types within python class "%s" with the msg_types specified in the .yaml under [%s][%s].'
            % (msg_type_py, msg_type_yaml, name, node_str, component, cname)
        )
    except Exception as e:
        if name in ["env/supervisor", "env/observations", "env/actions"]:
            return
        else:
            raise (e)


def check_valid_rosparam_type(param):
    valid_types = (str, int, list, float, bool, dict)
    if isinstance(param, valid_types) or param is None:
        if isinstance(param, dict):
            for key, value in param.items():
                assert isinstance(
                    key, str
                ), 'Only keys of type "str" are supported in dictionaries that are uploaded to the rosparam server.'
                check_valid_rosparam_type(value)
        if isinstance(param, list):
            for value in param:
                check_valid_rosparam_type(value)
    else:
        raise ValueError(
            'Type "%s" of a specified param with value "%s" is not supported by the rosparam server.'
            % (type(param), param.__name__)
        )


def msg_type_error(
    source,
    target,
    msg_type_out,
    converter_out,
    msg_type_ros,
    converter_in,
    msg_type_in,
    msg_type_in_yaml,
):
    if isinstance(source, tuple):
        source = list(source)
    if isinstance(target, tuple):
        target = list(target)
    msg_type_str = '\n\nConversion of msg_type from source="%s/%s/%s" ---> target="%s/%s/%s":\n\n' % tuple(
        source + target
    )
    msg_type_str += ">> msg_type_source:  %s (as specified in source)\n         ||\n         \/\n" % msg_type_out
    msg_type_str += ">> output_converter: %s \n         ||\n         \/\n" % converter_out
    msg_type_str += ">> msg_type_ROS:     %s \n         ||\n         \/\n" % msg_type_ros
    msg_type_str += ">> input_converter:  %s \n         ||\n         \/\n" % converter_in
    msg_type_str += (
        ">> msg_type_target:  %s (inferred from converters)\n         /\ \n         || (These must be equal, but they are not!!)\n         \/\n"
        % msg_type_in
    )
    msg_type_str += ">> msg_type_target:  %s (as specified in target)\n" % msg_type_in_yaml
    return msg_type_str


def deepcopy(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        return copy.deepcopy(func(*args, **kwargs))

    return wrapper


def is_supported_type(param: Any, types: Tuple, none_support):
    if isinstance(param, types) or (param is None and none_support):
        if isinstance(param, dict):
            for key, value in param.items():
                assert isinstance(key, str), f'Invalid key "{key}". Only type "str" is supported as dictionary key.'
                is_supported_type(value, types, none_support)
        elif not isinstance(param, str) and hasattr(param, "__iter__"):
            for value in param:
                is_supported_type(value, types, none_support)
    else:
        raise ValueError(
            f'Type "{type(param)}" of a specified (nested) param "{param}" is not supported. Only types {types} are supported.'
        )


def supported_types(*types: Tuple, is_classmethod=True):
    # Check if we support NoneType
    none_support = False
    for a in types:
        if a is None:
            none_support = True
            break

    # Remove None from types
    types = tuple([t for t in types if t is not None])

    def _check(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if is_classmethod:
                check_args = list(args[1:]) + [value for _, value in kwargs.items()]
            else:
                check_args = list(args) + [value for _, value in kwargs.items()]
            for param in check_args:
                is_supported_type(param, types, none_support)
            return func(*args, **kwargs)

        return wrapper

    return _check


def exists(func):
    argspec = inspect.getfullargspec(func)
    if argspec.defaults:
        positional_count = len(argspec.args) - len(argspec.defaults)
        defaults = dict(zip(argspec.args[positional_count:], argspec.defaults))
    else:
        defaults = []

    def _exists(self, *args, **kwargs):
        check_args = dict()
        for _args in [zip(argspec.args[1:], args), kwargs.items()]:
            for arg, value in _args:
                if arg in ["component", "cname", "parameter", "bridge_id", "level"]:
                    check_args[arg] = value

        if "level" not in check_args and "level" in defaults:
            check_args["level"] = defaults["level"]

        # Remove level from check_args if level='agnostic'
        if "level" in check_args and check_args["level"] == "agnostic":
            check_args.pop("level")

        params = self.params
        _args = check_args
        if "level" in _args:
            level = _args["level"]
            assert level in params, f"Level '{level}' not found. Available keys({params})={params.keys()}."
            if "component" not in _args and "parameter" in _args:
                parameter = _args["parameter"]
                assert (
                    parameter in params[level]
                ), f"Parameter '{parameter}' not found. Available keys(params[{level}])={params[level].keys()}."
        if "component" in _args:
            component = _args["component"]
            assert component in params, f"Component '{component}' not found. Available keys(params)={params.keys()}."
            assert (
                "level" not in check_args or component in params["default"]
            ), f"Component '{component}' not found. Available keys(params['default'])={params['default'].keys()}."
            if "cname" in _args:
                cname = _args["cname"]
                assert (
                    cname in params[component]
                ), f"Cname '{cname}' not found. Available keys(params[{component}])={params[component].keys()}."
                if "parameter" in _args:
                    parameter = _args["parameter"]
                    assert (
                        parameter in params[component][cname]
                    ), f"Parameter '{parameter}' not found. Available keys(params[{component}][{cname}])={params[component][cname].keys()}."
        if "bridge_id" in _args:
            bridge_id = _args["bridge_id"]
            assert bridge_id in params, f"Bridge (id) '{bridge_id}' not found. Available keys(params)={params.keys()}."
            if "component" in _args:
                component = _args["component"]
                assert (
                    component in params[bridge_id]
                ), f"Component '{component}' not found. Available keys(params[{bridge_id}])={params[bridge_id].keys()}."
                if "cname" in _args:
                    cname = _args["cname"]
                    assert (
                        cname in params[bridge_id][component]
                    ), f"Cname '{cname}' not found. Available keys(params[{bridge_id}][{component}])={params[bridge_id][component].keys()}."

        return func(self, *args, **kwargs)

    return _exists


def get_default_params(func):
    argspec = inspect.getfullargspec(func)
    if argspec.defaults:
        positional_count = len(argspec.args) - len(argspec.defaults)
        defaults = dict(zip(argspec.args[positional_count:], argspec.defaults))
    else:
        defaults = dict()
        positional_count = len(argspec.args)
    for arg in argspec.args[:positional_count]:
        if arg == "self":
            continue
        defaults[arg] = None
    return defaults
