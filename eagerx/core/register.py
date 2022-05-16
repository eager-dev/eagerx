import functools
import inspect
import rospy
import copy
# from unittest.mock import MagicMock
from eagerx.utils.utils import deepcopy, load, SUPPORTED_SPACES
from typing import TYPE_CHECKING, Callable, Any, Union, List, Dict, Optional
import os


if TYPE_CHECKING:
    from eagerx.core.graph_engine import EngineGraph  # noqa: F401
    from eagerx.core.Entities import Entity, Engine  # noqa: F401

# Global registry with registered entities (engines, objects, nodes, converters, simnodes, etc..)
REGISTRY = dict()
# Global registry with registered I/O types of (.callback, .reset/.pre_reset, .initialize, .add_object)
TYPE_REGISTER = dict()


class ReverseRegisterLookup:
    def __init__(self, d):
        self._dict = d

    def __getitem__(self, spec_lookup):
        for _entity_cls, entity_id in self._dict.items():
            for entity_idd, entry in entity_id.items():
                if entry["spec"] == spec_lookup:
                    return entity_idd


class LookupType:
    def __init__(self, d):
        self._dict = d

    @deepcopy
    def __getitem__(self, func_lookup):
        name_split = func_lookup.__qualname__.split(".")
        cls_name = name_split[0]
        return self._dict[cls_name]


# Global (reversed) registry of REGISTER:
REVERSE_REGISTRY = ReverseRegisterLookup(REGISTRY)
LOOKUP_TYPES = LookupType(TYPE_REGISTER)


def spec(entity_id: str, entity_cls: "Entity") -> Callable:
    """A decorator to register a spec function.

    :param entity_id: A unique string id.
    :param entity_cls: The entity's baseclass.
    """
    # """Register a spec function to make an entity"""

    def _register(func):
        entry = func.__module__ + "/" + func.__qualname__
        rospy.logdebug("[register]: entity_id=%s, entity=%s, entry=%s" % (entity_id, entity_cls.__name__, entry))

        @functools.wraps(func)
        def _spec(*args, **kwargs):
            """Make an entity with the registered spec function"""
            entity_type = func.__module__ + "/" + func.__qualname__[:-5]
            if not entity_id == "Identity":
                rospy.logdebug("[make]: entity_id=%s, entity=%s, entry=%s" % (entity_id, entity_cls.__name__, entity_type))
            spec = entity_cls.pre_make(entity_id, entity_type)

            # Initialize spec
            spec.initialize(load(entity_type))

            try:
                func(spec, *args, **kwargs)
            except TypeError as e:
                if "spec()" in e.args[0]:
                    from eagerx.core.info import get_info

                    sig_msg = get_info(entity_cls, entity_id, no_cls=True, return_msg=True)
                    err = (
                        f'You can only specify arguments according to the signature of the spec function of "{entity_id}".\n\n'
                    )
                    err += sig_msg
                    raise TypeError(err) from e
                else:
                    raise
            return spec

        if entity_cls not in REGISTRY:
            """Add entity if this is the first registration of entity kind"""
            REGISTRY[entity_cls] = dict()
        if entity_id in REGISTRY[entity_cls]:
            """Check if spec of duplicate entity_id corresponds to same spec function"""
            flag = _spec == REGISTRY[entity_cls][entity_id]["spec"] or bool(eval(os.environ.get("EAGERX_RELOAD", "0")))
            assert flag, f'There is already a {entity_cls.__name__} with entity_id "{entity_id}" registered.'
        cls = f"{func.__module__}/{func.__qualname__[:-5]}"
        REGISTRY[entity_cls][entity_id] = {"spec": _spec, "cls": cls}
        return _spec

    return _register


def make(entity, id, *args, **kwargs):
    """Make an entity with the registered spec function"""
    assert entity in REGISTRY, f'No entities of type "{entity.__name__}" registered.'
    assert (
        id in REGISTRY[entity]
    ), f'No entities of type "{entity.__name__}" registered under entity_id "{id}". Available entities "{[s for s in list(REGISTRY[entity].keys())]}".'
    return REGISTRY[entity][id]["spec"](*args, **kwargs)


def get_spec(entity, id, verbose=True):
    """Get information on the entity's spec function"""
    if verbose:
        help(REGISTRY[entity][id]["spec"])
    return inspect.signature(REGISTRY[entity][id]["spec"])


# TYPES


def _register_types(TYPE_REGISTER, component, cnames, func, space_only=True):
    name_split = func.__qualname__.split(".")
    cls_name = name_split[0]
    fn_name = name_split[1]
    entry = func.__module__ + "/" + func.__qualname__
    if space_only:
        for key, space in cnames.items():
            if space is None:
                continue
            flag = isinstance(space, SUPPORTED_SPACES)
            assert (
                flag
            ), f'TYPE REGISTRATION ERROR: [{cls_name}][{fn_name}][{component}]: "{space}" is an invalid space. Please provide a valid space for "{key}"instead.'
    rospy.logdebug(f"[{cls_name}][{fn_name}]: {component}={cnames}, entry={entry}")

    @functools.wraps(func)
    def registered_fn(*args, **kwargs):
        """Call the registered function"""
        return func(*args, **kwargs)

    if cls_name not in TYPE_REGISTER:
        """Add class if this is the first registration of class kind"""
        TYPE_REGISTER[cls_name] = dict()

    if component in TYPE_REGISTER[cls_name]:
        """Check if already registered component of duplicate function matches."""
        rospy.logdebug(f"[{cls_name}][{component}]: {component}={cnames}, entry={entry}")
        flag = cnames == TYPE_REGISTER[cls_name][component] or bool(eval(os.environ.get("EAGERX_RELOAD", "0")))
        assert (
            flag
        ), f'There is already a [{cls_name}][{component}] registered with cnames "{TYPE_REGISTER[cls_name][component]}", and they do not match the cnames of this function: "{cnames}".'
    TYPE_REGISTER[cls_name][component] = cnames
    return registered_fn


def inputs(**inputs: Any) -> Callable:
    """A decorator to register the inputs to a :func:`~eagerx.core.entities.Node.callback`.

    The :func:`~eagerx.core.entities.Node.callback` method should be decorated.

    :param inputs: The input's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "inputs", inputs)


def outputs(**outputs) -> Callable:
    """A decorator to register the outputs of a :func:`~eagerx.core.entities.Node.callback`.

    The :func:`~eagerx.core.entities.Node.callback` method should be decorated.

    :param outputs: The output's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "outputs", outputs)


def states(**states) -> Callable:
    """A decorator to register the states for a :func:`~eagerx.core.entities.Node.reset`.

    The :func:`~eagerx.core.entities.Node.reset` method should be decorated.

    :param outputs: The state's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "states", states)


def targets(**targets) -> Callable:
    """A decorator to register the targets of a :func:`~eagerx.core.entities.ResetNode.callback`.

    The :func:`~eagerx.core.entities.ResetNode.callback` method should be decorated.

    :param targets: The target's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "targets", targets)


def sensors(**sensors) -> Callable:
    """A decorator to register the sensors of an :class:`~eagerx.core.entities.Object`.

    The :func:`~eagerx.core.entities.Object.agnostic` method should be decorated.

    :param sensors: The sensor's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "sensors", sensors)


def actuators(**actuators) -> Callable:
    """A decorator to register the actuators of an :class:`~eagerx.core.entities.Object`.

    The :func:`~eagerx.core.entities.Object.agnostic` method should be decorated.

    :param actuators: The actuator's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "actuators", actuators)


def engine_states(**engine_states) -> Callable:
    """A decorator to register the engine states of an :class:`~eagerx.core.entities.Object`.

    The :func:`~eagerx.core.entities.Object.agnostic` method should be decorated.

    :param engine_states: The engine state's msg_type class.
    """
    return functools.partial(_register_types, TYPE_REGISTER, "engine_states", engine_states)


def engine_config(**params: Optional[Union[bool, int, float, str, List, Dict]]) -> Callable:
    """A decorator to register engine specific parameters that are required to
    add an :class:`~eagerx.core.entities.Object` to the engine's :attr:`~eagerx.core.entities.Engine.simulator`.

    The :func:`~eagerx.core.entities.Engine.add_object` method should be decorated.

    :param params: Engine specific parameters with default values (i.e. only optional arguments are allowed).
    """
    return functools.partial(_register_types, TYPE_REGISTER, "engine_config", params, space_only=False)


def config(**params: Optional[Union[bool, int, float, str, List, Dict]]) -> Callable:
    """A decorator to register an :class:`~eagerx.core.entities.Object`'s default config.

    The :func:`~eagerx.core.entities.Object.agnostic` method should be decorated.

    :param params: Engine specific parameters with default values (i.e. only optional arguments are allowed).
    """
    return functools.partial(
        _register_types,
        TYPE_REGISTER,
        "config",
        params,
        space_only=False,
    )


# ENGINES


def engine(entity_id: str, engine_cls: "Engine") -> Callable:
    """A decorator to register an engine implementation of an :class:`~eagerx.core.entities.Object`.

    .. note:: In our running example, the :func:`~eagerx.core.entities.Object.example_engine` method would be decorated.

    :param entity_id: A unique string id.
    :param engine_cls: The Engine's subclass (not the baseclass :class:`~eagerx.core.entities.Engine`).
    """
    engine_config = LOOKUP_TYPES[engine_cls.add_object]["engine_config"]
    engine_id = REVERSE_REGISTRY[engine_cls.spec]

    def _register(func):
        name_split = func.__qualname__.split(".")
        if len(name_split) > 1:
            cls_name = name_split[0]
            fn_name = name_split[1]
        else:
            cls_name = "N/A"
            fn_name = name_split[0]
        entry = func.__module__ + "/" + func.__qualname__
        rospy.logdebug(f"[{cls_name}][{fn_name}]: engine_id={engine_id}, entry={entry}")

        @functools.wraps(func)
        def _engine(spec):
            """First, initialize spec with object_info, then call the engine function"""
            # Add default engine_config parameters
            spec._initialize_engine_config(engine_id, copy.deepcopy(engine_config))
            # Initialize engine graph
            graph = spec._initialize_object_graph()
            # Modify engine_config with user-defined engine implementation
            func(spec, graph)
            # Add graph to spec & remove redundant states
            spec._add_graph(engine_id, graph)
            return graph

        # Register engine implementation for object
        from eagerx.core.entities import Object

        msg = f"Cannot register engine '{engine_id}' for object '{entity_id}'. "
        assert Object in REGISTRY, msg + "No Objects have been registered. Make sure to import the object."
        assert entity_id in REGISTRY[Object], (
            msg + "No object with this entity_id was registered. Make sure to import the object."
        )

        # Check if spec of duplicate entity_id corresponds to same spec function
        flag = engine_id in REGISTRY[Object][entity_id] and _engine == REGISTRY[Object][entity_id][engine_id]
        flag = not flag or bool(eval(os.environ.get("EAGERX_RELOAD", "0")))
        assert flag, msg + "There is already an engine implementation of this type registered."

        # Register engine implementation
        REGISTRY[Object][entity_id][engine_id] = _engine
        return _engine

    return _register


def add_engine(spec, engine_id):
    """Add engine based on registered entity_id"""
    entity_id = spec.config.entity_id

    # Register engine implementation for object
    from eagerx.core.entities import Object

    msg = f"Cannot ad engine implementation '{engine_id}' for object '{entity_id}'. "
    assert Object in REGISTRY, msg + "No Objects have been registered. Make sure to import the object."
    assert entity_id in REGISTRY[Object], msg + "No object with this entity_id was registered. Make sure to import the object."

    graph = REGISTRY[Object][entity_id][engine_id](spec)
    return graph
