import functools
import inspect
import rospy
from eagerx_core.utils.utils import deepcopy
from eagerx_core.specs import EntitySpec


# Global registry with registered entities (bridges, objects, nodes, converters, simnodes, etc..)
REGISTRY = dict()
# Global registry with registered I/O types of (.callback, .reset/.pre_reset, .initialize, .add_object)
TYPE_REGISTER = dict()


class ReverseRegisterLookup:
    def __init__(self, d):
        self._dict = d

    def __getitem__(self, spec_lookup):
        for entity_cls, entity_id in self._dict.items():
            for entity_id, spec in entity_id.items():
                if spec == spec_lookup:
                    return entity_id


class LookupType:
    def __init__(self, d):
        self._dict = d

    @deepcopy
    def __getitem__(self, func_lookup):
        name_split = func_lookup.__qualname__.split('.')
        cls_name = name_split[0]
        fn_name = name_split[1]
        return self._dict[cls_name]


# Global (reversed) registry of REGISTER:
REVERSE_REGISTRY = ReverseRegisterLookup(REGISTRY)
LOOKUP_TYPES = LookupType(TYPE_REGISTER)

def spec(entity_id, entity_cls):
    """Register a spec function to make an entity"""
    def _register(func):
        entry = func.__module__ + '/' + func.__qualname__
        rospy.logdebug('[register]: entity_id=%s, entity=%s, entry=%s' % (entity_id, entity_cls.__name__, entry))
        @functools.wraps(functools.partial(func, None))
        def _spec(*args, **kwargs):
            """Make an entity with the registered spec function"""
            entity_type = func.__module__ + '/' + func.__qualname__[:-5]
            if not entity_id == 'Identity':
                rospy.logdebug('[make]: entity_id=%s, entity=%s, entry=%s' % (entity_id, entity_cls.__name__, entity_type))

            if len(args) > 0 and isinstance(args[0], EntitySpec):
                return func(*args, **kwargs)
            else:
                return func(entity_cls.pre_make(entity_type), *args, **kwargs)
        if entity_cls not in REGISTRY:
            """Add entity if this is the first registration of entity kind"""
            REGISTRY[entity_cls] = dict()
        if entity_id in REGISTRY[entity_cls]:
            """Check if spec of duplicate entity_id corresponds to same spec function"""
            assert _spec == REGISTRY[entity_cls][entity_id], f'There is already a {entity_cls.__name__} with entity_id "{entity_id}" registered.'
        REGISTRY[entity_cls][entity_id] = _spec
        return _spec
    return _register


def make(entity, id, *args, **kwargs):
    """Make an entity with the registered spec function"""
    return REGISTRY[entity][id](*args, **kwargs)


def get_spec(entity, id):
    """Get information on the entity's spec function"""
    help(REGISTRY[entity][id])
    return inspect.signature(REGISTRY[entity][id])


############# TYPES ###############


def _register_types(TYPE_REGISTER, component, cnames, func, cls_only=True):
    name_split = func.__qualname__.split('.')
    cls_name = name_split[0]
    fn_name = name_split[1]
    entry = func.__module__ + '/' + func.__qualname__
    if cls_only:
        for key, cls in cnames.items():
            assert inspect.isclass(cls), f'TYPE REGISTRATION ERROR: [{cls_name}][{fn_name}][{component}]: An instance "{cls}" of class "{cls.__class__}" was provided for "{key}". Please provide the class instead.'
    rospy.logdebug(f'[{cls_name}][{fn_name}]: {component}={cnames}, entry={entry}')

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
        rospy.logdebug(f'[{cls_name}][{component}]: {component}={cnames}, entry={entry}')
        assert cnames == TYPE_REGISTER[cls_name][
            component], f'There is already a [{cls_name}][{component}] registered with cnames "{TYPE_REGISTER[cls_name][component]}", and they do not match the cnames of this function: "{cnames}".'
    TYPE_REGISTER[cls_name][component] = cnames
    return registered_fn


def inputs(**inputs):
    """Register input msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, 'inputs', inputs)

def outputs(**outputs):
    """Register output msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, 'outputs', outputs)

def states(**states):
    """Register state msg_types of reset"""
    return functools.partial(_register_types, TYPE_REGISTER, 'states', states)

def targets(**targets):
    """Register target msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, 'targets', targets)

def sensors(**sensors):
    """Register sensor msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'sensors', sensors)

def actuators(**actuators):
    """Register actuator msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'actuators', actuators)

def simstates(**simstates):
    """Register simstate msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'states', simstates)

def object_params(**object_params):
    """Register default object_params arguments"""
    return functools.partial(_register_types, TYPE_REGISTER, 'object_params', object_params, cls_only=False)

def agnostic_params(**agnostic_params):
    """Register default agnostic_params arguments"""
    return functools.partial(_register_types, TYPE_REGISTER, 'agnostic_params', agnostic_params, cls_only=False)

def node_params(**node_params):
    """Register default node_params arguments"""
    return functools.partial(_register_types, TYPE_REGISTER, 'node_params', node_params, cls_only=False)


############# BRIDGES ###############


def bridge(bridge_cls):
    object_params = LOOKUP_TYPES[bridge_cls.add_object]['object_params']
    bridge_id = REVERSE_REGISTRY[bridge_cls.spec]
    def _bridge(func):
        name_split = func.__qualname__.split('.')
        cls_name = name_split[0]
        fn_name = name_split[1]
        entry = func.__module__ + '/' + func.__qualname__
        rospy.logdebug(f'[{cls_name}][{fn_name}]: bridge_id={bridge_id}, entry={entry}')
        @functools.wraps(func)
        def bridge_fn(cls, spec):
            """First, initialize spec with object_info, then call the bridge function"""
            spec._initialize_bridge(bridge_id, object_params)
            return func(cls, spec, bridge_id)
        return bridge_fn
    return _bridge
