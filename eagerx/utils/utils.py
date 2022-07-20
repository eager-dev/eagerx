# OTHER
import time
from typing import List, NamedTuple, Any, Tuple
import importlib
import inspect
from functools import wraps
import copy
import json

from eagerx.core.constants import BackendException, Unspecified


# def space_to_dict(space):
#     msg = f"Invalid space provided that is not supported ({type(space)}). Supported spaces are ({SUPPORTED_SPACES})"
#     assert isinstance(space, SUPPORTED_SPACES), msg
#     if isinstance(space, gym.spaces.box.Box):
#         low = np.nan_to_num(space.low)
#         high = np.nan_to_num(space.high)
#         if np.all(low == low.reshape(-1)[0]):
#             low = low.reshape(-1)[0].item()
#         if np.all(high == high.reshape(-1)[0]):
#             high = high.reshape(-1)[0].item()
#         if isinstance(low, np.ndarray):
#             low = low.tolist()
#         if isinstance(high, np.ndarray):
#             high = high.tolist()
#         d = dict(low=low, high=high, shape=list(space.shape), dtype=space.dtype.name)
#     elif isinstance(space, gym.spaces.discrete.Discrete):
#         d = dict(n=space.n, shape=list(space.shape), dtype=space.dtype.name)
#     else:
#         raise NotImplementedError(f"Provided space has not yet been implemented ({type(space)}).")
#     return d


# def dict_to_space(d: Dict):
#     assert isinstance(d, dict), f"Invalid input provided that is not supported ({type(d)}). Argument should be a dict."
#     if "n" in d:
#         space = gym.spaces.discrete.Discrete(n=d["n"])
#     elif all([arg in d for arg in ["low", "high", "shape", "dtype"]]):
#         if isinstance(d["low"], list):
#             low = np.array(d["low"], dtype=d["dtype"])
#         else:
#             low = d["low"]
#         if isinstance(d["high"], list):
#             high = np.array(d["high"], dtype=d["dtype"])
#         else:
#             high = d["high"]
#         space = gym.spaces.box.Box(low=low, high=high, shape=d["shape"], dtype=d["dtype"])
#     else:
#         raise NotImplementedError(f"Cannot infer type of space to initialize with the provided dict ({d}).")
#     return space


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


def get_attribute_from_module(attribute, module=None):
    if module is None:
        module, attribute = attribute.split("/")
    module = importlib.import_module(module)
    attribute = getattr(module, attribute)
    return attribute


def load(mod_attr: str):
    return get_attribute_from_module(mod_attr)


def initialize_processor(spec):
    processor = load(spec.params["processor_type"])()
    processor.initialize(spec)
    return processor


def is_compatible(dtype_source, dtype_target):
    msg = f"Dtype of source ({dtype_source}) does not match with the dtype of target ({dtype_target})."
    assert dtype_target == dtype_source, msg


class Stamp(NamedTuple):
    """A dataclass for timestamping received messages."""

    #: Sequence number of received message since the last reset.
    seq: int
    #: Timestamp according to the simulated clock (seconds).
    sim_stamp: float
    #: Timestamp according to the wall clock (seconds).
    wc_stamp: float


class Info(NamedTuple):
    """A dataclass containing info about the received messages in :attr:`~eagerx.utils.utils.Msg.msgs`."""

    #: name of the registered input.
    name: str
    #: Number of times :func:`~eagerx.core.entities.Node.callback` has been called since the last reset.
    node_tick: int
    #: Rate (Hz) of the input.
    rate_in: float
    #: Simulated timestamp that states during which cycle the message was received since the last reset according
    #: to :attr:`~eagerx.core.entities.Node.rate` and :attr:`~eagerx.utils.utils.Info.node_tick`.
    t_node: List[Stamp]
    #: Simulated timestamp that states at what time the message was received
    #: according to :attr:`~eagerx.utils.utils.Info.rate_in` and :attr:`~eagerx.utils.utils.Stamp.seq`.
    t_in: List[Stamp]
    #: Only concerns states. A flag that states if it must be reset.
    done: bool


class Msg(NamedTuple):
    """A dataclass representing a (windowed) input that is passed to :func:`~eagerx.core.entities.Node.callback`."""

    #: Info on the received messages in :attr:`~eagerx.utils.utils.Msg.msgs`.
    info: Info
    #: The received messages with indexing `msgs[-1]` being the most recent message and `msgs[0]` the oldest.
    msgs: List[Any]


# Set default values
Stamp.__new__.__defaults__ = (None,) * len(Stamp._fields)
Info.__new__.__defaults__ = (None,) * len(Info._fields)


# def check_valid_rosparam_type(param):
#     valid_types = (str, int, list, float, bool, dict)
#     if isinstance(param, valid_types) or param is None:
#         if isinstance(param, dict):
#             for key, value in param.items():
#                 assert isinstance(
#                     key, str
#                 ), 'Only keys of type "str" are supported in dictionaries that are uploaded to the rosparam server.'
#                 check_valid_rosparam_type(value)
#         if isinstance(param, list):
#             for value in param:
#                 check_valid_rosparam_type(value)
#     else:
#         raise ValueError(
#             'Type "%s" of a specified param with value "%s" is not supported by the rosparam server.'
#             % (type(param), param.__name__)
#         )


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
        raise TypeError(
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


# A singleton that is used to check if an argument was specified.
_unspecified = Unspecified()


def get_param_with_blocking(name, backend, default=_unspecified, timeout=2.0):
    params = Unspecified()
    start = time.time()
    while isinstance(params, Unspecified):
        if time.time() - start > timeout:
            raise KeyError(f"Timeout. Parameter '{name}' not available on parameter server.")
        try:
            params = backend.get_param(name, default=default)
        except (BackendException, KeyError):
            if not isinstance(default, Unspecified):
                return default
            sleep_time = 0.01
            time.sleep(sleep_time)
    return replace_None(params, to_null=False)
