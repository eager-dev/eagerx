import functools
import copy

import eagerx.core.log as log
from eagerx.utils.utils import deepcopy
from eagerx.core.space import Space
from typing import TYPE_CHECKING, Callable, Any
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
        entity_id = func_lookup.__module__ + "/" + cls_name
        return self._dict[entity_id]


# Global (reversed) registry of REGISTER:
REVERSE_REGISTRY = ReverseRegisterLookup(REGISTRY)
LOOKUP_TYPES = LookupType(TYPE_REGISTER)


# TYPES


def _register_types(TYPE_REGISTER, component, cnames, func, space_only=True):
    name_split = func.__qualname__.split(".")
    cls_name = name_split[0]
    fn_name = name_split[1]
    entity_id = func.__module__ + "/" + cls_name
    entry = func.__module__ + "/" + func.__qualname__
    if space_only:
        for key, space in cnames.items():
            if space is None:
                continue
            flag = isinstance(space, Space)
            assert (
                flag
            ), f'TYPE REGISTRATION ERROR: [{cls_name}][{fn_name}][{component}]: "{space}" is an invalid space. Please provide a valid space for "{key}"instead.'
    log.logdebug(f"[{cls_name}][{fn_name}]: {component}={cnames}, entry={entry}")

    @functools.wraps(func)
    def registered_fn(*args, **kwargs):
        """Call the registered function"""
        return func(*args, **kwargs)

    if entity_id not in TYPE_REGISTER:
        """Add class if this is the first registration of class kind"""
        TYPE_REGISTER[entity_id] = dict()

    if component in TYPE_REGISTER[entity_id]:
        """Check if already registered component of duplicate function matches."""
        log.logdebug(f"[{entity_id}][{component}]: {component}={cnames}, entry={entry}")
        flag = cnames == TYPE_REGISTER[entity_id][component] or bool(eval(os.environ.get("EAGERX_RELOAD", "0")))
        assert (
            flag
        ), f'There is already a [{entity_id}][{component}] registered with cnames "{TYPE_REGISTER[entity_id][component]}", and they do not match the cnames of this function: "{cnames}".'
    TYPE_REGISTER[entity_id][component] = cnames
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


# ENGINES


def engine(engine_cls: "Engine", entity=None) -> Callable:
    """A decorator to register an engine implementation of an :class:`~eagerx.core.entities.Object`.

    .. note:: In our running example, the :func:`~eagerx.core.entities.Object.example_engine` method would be decorated.

    :param engine_cls: The Engine's subclass (not the baseclass :class:`~eagerx.core.entities.Engine`).
    :param entity: The entity that corresponds to the engine implementation. If left unspecified, the engine is
                   registered to the class that owns the method.
    """
    from eagerx.utils.utils import get_default_params

    engine_config = get_default_params(engine_cls.add_object)
    assert "name" in engine_config, "The object `name` must be an argument to engine.add_object()"
    engine_id = engine_cls.__module__ + "/" + engine_cls.__qualname__

    def _register(func, engine_id=engine_id):
        name_split = func.__qualname__.split(".")
        if entity is not None:
            cls_name = entity.__qualname__
            fn_name = name_split[0]
            entity_id = entity.__module__ + "/" + cls_name
        elif len(name_split) > 1:
            cls_name = name_split[0]
            fn_name = name_split[1]
            entity_id = func.__module__ + "/" + cls_name
        else:
            cls_name = "N/A"
            fn_name = name_split[0]
            entity_id = func.__module__ + "/" + cls_name
        entry = func.__module__ + "/" + func.__qualname__

        log.logdebug(f"[{cls_name}][{fn_name}]: entry={entry}")

        @functools.wraps(func)
        def _engine(spec, engine) -> "EngineGraph":
            """First, initialize spec with object_info, then call the engine function"""
            # Add default engine_config parameters
            engine._initialize_engine_config(spec, copy.deepcopy(engine_config))
            # Initialize engine graph
            graph = spec._initialize_object_graph()
            # Modify engine_config with user-defined engine implementation
            func(spec, graph)
            # Add engine-specific parameters to engine.objects
            engine_params = {key: value for key, value in spec.engine.items() if key != "states"}
            engine.add_object(**engine_params)
            # Add engine-specific states to engine.objects
            engine._add_engine_states(engine_params["name"], spec)
            return graph

        msg = f"Cannot register engine '{entry}' for object '{entity_id}'. "

        # Check if spec of duplicate entity_id corresponds to same spec function
        if entity_id not in REGISTRY:
            REGISTRY[entity_id] = {}
        flag = engine_id in REGISTRY[entity_id] and _engine == REGISTRY[entity_id][engine_id]
        flag = not flag or bool(eval(os.environ.get("EAGERX_RELOAD", "0")))
        assert flag, msg + "There is already an engine implementation of this type registered."

        # Register engine implementation
        REGISTRY[entity_id][engine_id] = _engine
        return _engine

    return _register


def add_engine(spec, engine):
    """Add engine based on registered entity_id"""
    entity_id = spec.config.entity_id
    engine_id = engine.config.entity_id

    msg = f"Cannot add object '{entity_id}' to engine '{engine_id}'. "
    assert entity_id in REGISTRY, msg + f"The Object '{entity_id}' has not been registered yet."
    msg_2 = (
        "If launching the environment as a subprocess, "
        "make sure to reload (i.e. import) the engine implementations in each subprocess."
    )
    assert engine_id in REGISTRY[entity_id], msg + "No engine implementation was registered. " + msg_2
    graph = REGISTRY[entity_id][engine_id](spec, engine)
    return graph
