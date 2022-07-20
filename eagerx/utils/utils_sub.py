"""
Adapted from the roslaunch.substitution_args package.
"""

import yaml
import math
import ast
import os
from io import StringIO
from typing import Union, Dict, Optional, List


class SubstitutionException(Exception):
    """
    Base class for exceptions in substitution_args routines
    """

    pass


class ArgException(SubstitutionException):
    """
    Exception for missing $(arg) values
    """

    pass


def substitute_args(
    param: Union[str, Dict],
    context: Optional[Dict] = None,
    only: Optional[List[str]] = None,
):
    """Substitute arguments based on the context dictionairy (and possibly sourced packages).
    Follows the xacro substition convention of ROS, with a 'config' and 'ns' command added.
    :param param: dict or string we wish to perform substitutions on.
    :param context: dict[command][context] with replacement values .
    :param only: List of possible commands. If only is not provided, all commands are executed.
                 Options are ['env', 'optenv', 'dirname', 'anon', 'arg', 'ns', 'config'].
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
            if isinstance(param[key], (dict, list)):
                substitute_args(param[key], context, only=only)
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key], context, only=only)
    elif isinstance(param, list):
        for idx, i in enumerate(param):
            if isinstance(i, (dict, list)):
                substitute_args(i, context, only=only)
            elif isinstance(i, str):
                param[idx] = resolve_args(i, context, only=only)
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
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    if not arg_str:
        return arg_str
    # special handling of $(eval ...)
    if arg_str.startswith("$(eval ") and arg_str.endswith(")"):
        return _eval(arg_str[7:-1], context)
    # first resolve variables like 'env' and 'arg'
    commands = {
        "env": _env,
        "optenv": _optenv,
        "dirname": _dirname,
        "arg": _arg,
        "ns": _ns,
        "config": _config,
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
        "find": _find,
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


def _find(resolved, a, args, context):
    try:
        import roslaunch
    except ImportError as e:
        msg = f"You can only use $(find ...) if ROS is sourced: {e}"
        raise ImportError(msg)
    return roslaunch.substitution_args._find(resolved, a, args, context)


class _DictWrapper(object):
    def __init__(self, args, functions):
        self._args = args
        self._functions = functions

    def __getitem__(self, key):
        try:
            return self._functions[key]
        except KeyError:
            return convert_value(self._args[key], "auto")


def convert_value(value, type_):
    """
    Convert a value from a string representation into the specified
    type

    @param value: string representation of value
    @type  value: str
    @param type_: int, double, string, bool, or auto
    @type  type_: str
    @raise ValueError: if parameters are invalid
    """
    type_ = type_.lower()
    # currently don't support XML-RPC date, dateTime, maps, or list
    # types
    if type_ == "auto":
        # attempt numeric conversion
        try:
            if "." in value:
                return float(value)
            else:
                return int(value)
        except ValueError:
            pass
        # bool
        lval = value.lower()
        if lval == "true" or lval == "false":
            return convert_value(value, "bool")
        # string
        return value
    elif type_ == "str" or type_ == "string":
        return value
    elif type_ == "int":
        return int(value)
    elif type_ == "double":
        return float(value)
    elif type_ == "bool" or type_ == "boolean":
        value = value.lower().strip()
        if value == "true" or value == "1":
            return True
        elif value == "false" or value == "0":
            return False
        raise ValueError("%s is not a '%s' type" % (value, type_))
    elif type_ == "yaml":
        try:
            return yaml.safe_load(value)
        except yaml.parser.ParserError as e:
            raise ValueError(e)
    else:
        raise ValueError("Unknown type '%s'" % type_)


def _eval(s, context):
    if "anon" not in context:
        context["anon"] = {}
    if "arg" not in context:
        context["arg"] = {}

    # inject arg context
    def _eval_arg_context(name):
        return convert_value(_eval_arg(name, args=context["arg"]), "auto")

    # inject dirname context
    def _eval_dirname_context():
        return _eval_dirname(context["filename"])

    functions = {"arg": _eval_arg_context, "dirname": _eval_dirname_context}
    functions.update(_eval_dict)

    # ignore values containing double underscores (for safety)
    # http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
    if s.find("__") >= 0:
        raise SubstitutionException("$(eval ...) may not contain double underscore expressions")
    return str(eval(s, {}, _DictWrapper(context["arg"], functions)))


def _eval_env(name):
    try:
        return os.environ[name]
    except KeyError as e:
        raise SubstitutionException("environment variable %s is not set" % str(e))


def _env(resolved, a, args, context):
    """
    process $(env) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException("$(env var) command only accepts one argument [%s]" % a)
    return resolved.replace("$(%s)" % a, _eval_env(args[0]))


def _eval_dirname(filename):
    if not filename:
        raise SubstitutionException("Cannot substitute $(dirname), no file/directory information available.")
    return os.path.abspath(os.path.dirname(filename))


def _dirname(resolved, a, args, context):
    """
    process $(dirname)
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if no information about the current launch file is available, for example
           if XML was passed via stdin, or this is a remote launch.
    """
    return resolved.replace("$(%s)" % a, _eval_dirname(context.get("filename", None)))


def _eval_optenv(name, default=""):
    if name in os.environ:
        return os.environ[name]
    return default


def _optenv(resolved, a, args, context):
    """
    process $(optenv) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(optenv var) must specify an environment variable [%s]" % a)
    return resolved.replace("$(%s)" % a, _eval_optenv(args[0], default=" ".join(args[1:])))


def _eval_arg(name, args):
    try:
        return args[name]
    except KeyError:
        raise ArgException(name)


def _arg(resolved, a, args, context):
    """
    process $(arg) arg

    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(arg var) must specify a variable name [%s]" % (a))
    elif len(args) > 1:
        raise SubstitutionException("$(arg var) may only specify one arg [%s]" % (a))

    if "arg" not in context:
        context["arg"] = {}
    return resolved.replace("$(%s)" % a, _eval_arg(name=args[0], args=context["arg"]))


def _resolve_args(arg_str, context, resolve_anon, commands):
    ros_valid = ["find", "env", "optenv", "dirname", "anon", "arg"]
    valid = ros_valid + ["ns", "config"]
    resolved = arg_str
    if isinstance(arg_str, (str, list)):
        for a in _collect_args(arg_str):
            splits = [s for s in a.split(" ") if s]
            if not splits[0] in valid:
                raise SubstitutionException("Unknown substitution command [%s]. Valid commands are %s" % (a, valid))
            command = splits[0]
            args = splits[1:]
            if command in commands:
                resolved = commands[command](resolved, a, args, context)
    return resolved


def _eval_config(name, args):
    try:
        return args[name]
    except KeyError:
        raise ArgException(name)


_eval_ns = _eval_config


def tryeval(val):
    try:
        val = ast.literal_eval(val)
    except (ValueError, SyntaxError):
        pass
    return val


def _config(resolved, a, args, context):
    """
    process $(config) arg

    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(config var) must specify a variable name [%s]" % (a))
    elif len(args) > 1:
        raise SubstitutionException("$(config var) may only specify one arg [%s]" % (a))

    if "config" not in context:
        context["config"] = {}
    try:
        return tryeval(resolved.replace("$(%s)" % a, str(_eval_config(name=args[0], args=context["config"]))))
    except ArgException as e:  # ArgException:
        if isinstance(e, ArgException):
            return resolved
        elif isinstance(e, TypeError):
            raise
        else:
            raise


def _ns(resolved, a, args, context):
    """
    process $(ns) arg

    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(ns var) must specify a variable name [%s]" % (a))
    elif len(args) > 1:
        raise SubstitutionException("$(ns var) may only specify one arg [%s]" % (a))

    if "ns" not in context:
        context["ns"] = {}
    try:
        return tryeval(resolved.replace("$(%s)" % a, str(_eval_ns(name=args[0], args=context["ns"]))))
    except ArgException as e:  # ArgException:
        if isinstance(e, ArgException):
            return resolved
        elif isinstance(e, TypeError):
            raise
        else:
            raise


_OUT = 0
_DOLLAR = 1
_LP = 2
_IN = 3


def _collect_args(arg_str):
    """
    State-machine parser for resolve_args. Substitution args are of the form:
    $(find package_name)/scripts/foo.py $(export some/attribute blar) non-relevant stuff

    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: list of arguments
    @rtype: [str]
    """
    buff = StringIO()
    args = []
    state = _OUT
    for c in arg_str:
        # No escapes supported
        if c == "$":
            if state == _OUT:
                state = _DOLLAR
            elif state == _DOLLAR:
                pass
            else:
                raise SubstitutionException("Dollar signs '$' cannot be inside of substitution args [%s]" % arg_str)
        elif c == "(":
            if state == _DOLLAR:
                state = _LP
            elif state != _OUT:
                raise SubstitutionException("Invalid left parenthesis '(' in substitution args [%s]" % arg_str)
        elif c == ")":
            if state == _IN:
                # save contents of collected buffer
                args.append(buff.getvalue())
                buff.truncate(0)
                buff.seek(0)
                state = _OUT
            else:
                state = _OUT
        elif state == _DOLLAR:
            # left paren must immediately follow dollar sign to enter _IN state
            state = _OUT
        elif state == _LP:
            state = _IN

        if state == _IN:
            buff.write(c)
    return args


# Create a dictionary of global symbols that will be available in the eval
# context.  We disable all the builtins, then add back True and False, and also
# add true and false for convenience (because we accept those lower-case strings
# as boolean values in XML).
_eval_dict = {
    "true": True,
    "false": False,
    "True": True,
    "False": False,
    "__builtins__": {k: __builtins__[k] for k in ["list", "dict", "map", "str", "float", "int"]},
    "env": _eval_env,
    "optenv": _eval_optenv,
}
# also define all math symbols and functions
_eval_dict.update(math.__dict__)
