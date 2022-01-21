import functools
import inspect
import rospy
from eagerx_core.specs import EntitySpec

# Global registry with registered entities (bridges, objects, nodes, converters, simnodes, etc..)
REGISTRY = dict()
# Global registry with registered I/O types of (.callback, .reset/.pre_reset, .initialize, .add_object)
TYPE_REGISTER = dict()


def spec(id, entity_cls):
    """Register a spec function to make an entity"""
    def _register(func):
        entry = func.__module__ + '/' + func.__qualname__
        rospy.logdebug('[register]: id=%s, entity=%s, entry=%s' % (id, entity_cls.__name__, entry))
        @functools.wraps(functools.partial(func, None))
        def _spec(*args, **kwargs):
            """Make an entity with the registered spec function"""
            entity_type = func.__module__ + '/' + func.__qualname__[:-5]
            rospy.logdebug('[make]: id=%s, entity=%s, entry=%s' % (id, entity_cls.__name__, entity_type))

            if len(args) > 0 and isinstance(args[0], EntitySpec):
                return func(*args, **kwargs)
            else:
                return func(entity_cls.pre_make(entity_type), *args, **kwargs)
        if entity_cls not in REGISTRY:
            """Add entity if this is the first registration of entity kind"""
            REGISTRY[entity_cls] = dict()
        if id not in REGISTRY[entity_cls]:
            """Add id if this is the first registration with this id"""
            REGISTRY[entity_cls][id] = dict()
        if 'spec' in REGISTRY[entity_cls][id]:
            """Check if spec of duplicate id corresponds to same spec function"""
            assert _spec == REGISTRY[entity_cls][id]['spec'], f'There is already a {entity_cls.__name__} with id "{id}" registered.'
        REGISTRY[entity_cls][id]['spec'] = _spec
        return _spec
    return _register


def make(entity, id, *args, **kwargs):
    """Make an entity with the registered spec function"""
    return REGISTRY[entity][id]['spec'](*args, **kwargs)


def get_spec(entity, id):
    """Get information on the entity's spec function"""
    help(REGISTRY[entity][id]['spec'])
    return inspect.signature(REGISTRY[entity][id]['spec'])


############# MSG_TYPES ###############


def _register_types(TYPE_REGISTER, component, cnames, func):
    name_split = func.__qualname__.split('.')
    cls_name = name_split[0]
    fn_name = name_split[1]
    entry = func.__module__ + '/' + func.__qualname__
    for key, cls in cnames.items():
        assert inspect.isclass(cls), f'TYPE REGISTRATION ERROR: [{cls_name}][{fn_name}][{component}]: An instance "{cls}" of class "{cls.__class__}" was provided for "{key}". Please provide the class instead.'
    if cls_name == 'spec':
        from eagerx_core.entities import Object
        # data[set(keys).intersection(data.keys()).pop()]
        fn_name = cls_name
        cls_name = '<unknown_obj>'
    rospy.logdebug(f'[{cls_name}][{fn_name}]: {component}={cnames}, entry={entry}')

    @functools.wraps(func)
    def registered_fn(*args, **kwargs):
        """Call the registered function"""
        # todo: perform type checking in I/O using TYPE_REGISTER
        return func(*args, **kwargs)

    if registered_fn in TYPE_REGISTER:
        """Check if already registered cnames of duplicate function match."""
        rospy.logdebug(f'[{cls_name}][{fn_name}]: {component}={cnames}, entry={entry}')
        assert cnames == TYPE_REGISTER[registered_fn][component], f'There is already a [{cls_name}][{fn_name}][{component}] registered with cnames "{TYPE_REGISTER[registered_fn][component]}", and they do not match the cnames of this function: "{cnames}".'
    TYPE_REGISTER[registered_fn] = {component: cnames}
    return registered_fn


def inputs(id, entity_cls, **inputs):
    """Register input msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, 'inputs', inputs)

def outputs(id, entity_cls, **outputs):
    """Register output msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, 'outputs', outputs)

def states(id, entity_cls, **states):
    """Register state msg_types of reset"""
    return functools.partial(_register_types, TYPE_REGISTER, 'states', states)

def targets(id, entity_cls, **targets):
    """Register target msg_types of callback"""
    return functools.partial(_register_types, TYPE_REGISTER, 'targets', targets)

def sensors(id, entity_cls, **sensors):
    """Register sensor msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'sensors', sensors)

def actuators(id, entity_cls, **actuators):
    """Register actuator msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'actuators', actuators)

def simstates(id, entity_cls, **simstates):
    """Register simstate msg_types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'simstates', simstates)

def object_params(id, entity_cls, **object_params):
    """Register object_params argument types"""
    return functools.partial(_register_types, TYPE_REGISTER, 'object_params', object_params)