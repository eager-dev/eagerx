import functools
import inspect
import rospy
import copy
from eagerx.utils.utils import deepcopy


# Global registry with registered entities (bridges, objects, nodes, converters, simnodes, etc..)
REGISTRY = dict()
# Global registry with registered I/O types of (.callback, .reset/.pre_reset, .initialize, .add_object)
TYPE_REGISTER = dict()


class ReverseRegisterLookup:
    def __init__(self, d):
        self._dict = d

    def __getitem__(self, spec_lookup):
        for _entity_cls, entity_id in self._dict.items():
            for entity_id, entry in entity_id.items():
                if entry["spec"] == spec_lookup:
                    return entity_id


class LookupType:
    def __init__(self, d):
        self._dict = d

    @deepcopy
    def __getitem__(self, func_lookup):
        name_split = func_lookup.__qualname__.split(".")
        cls_name = name_split[0]
        # fn_name = name_split[1]
        return self._dict[cls_name]


# Global (reversed) registry of REGISTER:
REVERSE_REGISTRY = ReverseRegisterLookup(REGISTRY)
LOOKUP_TYPES = LookupType(TYPE_REGISTER)


def spec(entity_id, entity_cls):
    """Register a spec function to make an entity"""

    def _register(func):
        entry = func.__module__ + "/" + func.__qualname__
        rospy.logdebug("[register]: entity_id=%s, entity=%s, entry=%s" % (entity_id, entity_cls.__name__, entry))

        @functools.wraps(functools.partial(func, None))
        def _spec(*args, **kwargs):
            """Make an entity with the registered spec function"""
            entity_type = func.__module__ + "/" + func.__qualname__[:-5]
            if not entity_id == "Identity":
                rospy.logdebug("[make]: entity_id=%s, entity=%s, entry=%s" % (entity_id, entity_cls.__name__, entity_type))
            spec = entity_cls.pre_make(entity_id, entity_type)
            try:
                func(spec, *args, **kwargs)
            except TypeError as e:
                if "spec()" in e.args[0]:
                    signature = entity_cls.get_spec(entity_id, verbose=False)
                    err = (
                        f'You can only specify arguments according to the signature of the spec function of "{entity_id}".\n\n'
                    )
                    err += f"The signature for this spec looks like: \n\n{signature}"
                    raise TypeError(err) from e
                else:
                    raise
            return spec

        if entity_cls not in REGISTRY:
            """Add entity if this is the first registration of entity kind"""
            REGISTRY[entity_cls] = dict()
        if entity_id in REGISTRY[entity_cls]:
            """Check if spec of duplicate entity_id corresponds to same spec function"""
            assert (
                _spec == REGISTRY[entity_cls][entity_id]["spec"]
            ), f'There is already a {entity_cls.__name__} with entity_id "{entity_id}" registered.'
        cls = f"{func.__module__}/{func.__qualname__[:-5]}"
        REGISTRY[entity_cls][entity_id] = {"spec": _spec, "cls": cls}
        return _spec

    return _register


def make(entity, id, *args, **kwargs):
    """Make an entity with the registered spec function"""
    assert entity in REGISTRY, f'No entities of type "{entity.__name__}" registered.'
    assert (
        id in REGISTRY[entity]
    ), f'No entities of type "{entity.__name__}" registered under id "{id}". Available entities "{[s for s in list(REGISTRY[entity].keys())]}".'
    return REGISTRY[entity][id]["spec"](*args, **kwargs)


def get_spec(entity, id, verbose=True):
    """Get information on the entity's spec function"""
    if verbose:
        help(REGISTRY[entity][id]["spec"])
    return inspect.signature(REGISTRY[entity][id]["spec"])


# TYPES


def _register_types(TYPE_REGISTER, component, cnames, func, cls_only=True):
    name_split = func.__qualname__.split(".")
    cls_name = name_split[0]
    fn_name = name_split[1]
    entry = func.__module__ + "/" + func.__qualname__
    if cls_only:
        for key, cls in cnames.items():
            assert inspect.isclass(
                cls
            ), f'TYPE REGISTRATION ERROR: [{cls_name}][{fn_name}][{component}]: An instance "{cls}" of class "{cls.__class__}" was provided for "{key}". Please provide the class instead.'
    rospy.logdebug(f"[{cls_name}][{fn_name}]: {component}={cnames}, entry={entry}")

    @functools.wraps(func)
    def registered_fn(*args, **kwargs):
        """Call the registered function"""
        # todo: perform type checking in I/O using TYPE_REGISTER
        return func(*args, **kwargs)

    if cls_name not in TYPE_REGISTER:
        """Add class if this is the first registration of class kind"""
        TYPE_REGISTER[cls_name] = dict()

    if component in TYPE_REGISTER[cls_name]:
        """Check if already registered component of duplicate function matches."""
        rospy.logdebug(f"[{cls_name}][{component}]: {component}={cnames}, entry={entry}")
        assert (
            cnames == TYPE_REGISTER[cls_name][component]
        ), f'There is already a [{cls_name}][{component}] registered with cnames "{TYPE_REGISTER[cls_name][component]}", and they do not match the cnames of this function: "{cnames}".'
    TYPE_REGISTER[cls_name][component] = cnames
    return registered_fn


def inputs(**inputs):
    """Register input msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, "inputs", inputs)


def outputs(**outputs):
    """Register output msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, "outputs", outputs)


def states(**states):
    """Register state msg_types of reset"""
    return functools.partial(_register_types, TYPE_REGISTER, "states", states)


def targets(**targets):
    """Register target msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, "targets", targets)


def sensors(**sensors):
    """Register sensor msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, "sensors", sensors)


def actuators(**actuators):
    """Register actuator msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, "actuators", actuators)


def simstates(**simstates):
    """Register simstates msg_type"""
    return functools.partial(_register_types, TYPE_REGISTER, "states", simstates)


def bridge_config(**bridge_config):
    """Register default bridge_config arguments"""
    return functools.partial(_register_types, TYPE_REGISTER, "bridge_config", bridge_config, cls_only=False)


def config(**params):
    """Register default config arguments"""
    return functools.partial(
        _register_types,
        TYPE_REGISTER,
        "config",
        params,
        cls_only=False,
    )


# BRIDGES


def bridge(entity_id, bridge_cls):
    bridge_config = LOOKUP_TYPES[bridge_cls.add_object]["bridge_config"]
    bridge_id = REVERSE_REGISTRY[bridge_cls.spec]

    def _register(func):
        name_split = func.__qualname__.split(".")
        cls_name = name_split[0]
        fn_name = name_split[1]
        entry = func.__module__ + "/" + func.__qualname__
        rospy.logdebug(f"[{cls_name}][{fn_name}]: bridge_id={bridge_id}, entry={entry}")

        @functools.wraps(func)
        def _bridge(spec):
            """First, initialize spec with object_info, then call the bridge function"""
            # Add default bridge_config parameters
            spec._initialize_bridge_config(bridge_id, copy.deepcopy(bridge_config))
            # Initialize engine graph
            graph = spec._initialize_object_graph()
            # Modify bridge_config with user-defined bridge implementation
            func(spec, graph)
            # Add graph to spec & remove redundant states
            spec._add_graph(bridge_id, graph)

        # Register bridge implementation for object
        from eagerx.core.entities import Object

        msg = f"Cannot register bridge '{bridge_id}' for object '{entity_id}'. "
        assert Object in REGISTRY, msg + "No Objects have been registered. Make sure to import the object."
        assert entity_id in REGISTRY[Object], msg + "No object with this id was registered. Make sure to import the object."

        # Check if spec of duplicate entity_id corresponds to same spec function
        flag = bridge_id in REGISTRY[Object][entity_id] and _bridge == REGISTRY[Object][entity_id][bridge_id]
        assert not flag, msg + "There is already a bridge implementation of this type registered."

        # Register bridge implementation
        REGISTRY[Object][entity_id][bridge_id] = _bridge
        return _bridge

    return _register


def add_bridge(spec, bridge_id):
    """Add bridge based on registered id"""
    entity_id = spec.config.entity_id

    # Register bridge implementation for object
    from eagerx.core.entities import Object

    msg = f"Cannot ad bridge implementation '{bridge_id}' for object '{entity_id}'. "
    assert Object in REGISTRY, msg + "No Objects have been registered. Make sure to import the object."
    assert entity_id in REGISTRY[Object], msg + "No object with this id was registered. Make sure to import the object."

    _ = REGISTRY[Object][entity_id][bridge_id](spec)
