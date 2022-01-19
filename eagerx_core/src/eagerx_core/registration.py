import functools
import inspect
import rospy

# Global registry with registered entities (bridges, objects, nodes, converters, simnodes, etc..)
REGISTRY = dict()


def register(id, entity_cls):
    """Register a spec function to make an entity"""
    def _register(func):
        entry = func.__module__ + '/' + func.__qualname__
        rospy.logdebug('[register]: id=%s, entity=%s, entry=%s' % (id, entity_cls.__name__, entry))
        @functools.wraps(functools.partial(func, None))
        def spec(*args, **kwargs):
            """Make an entity with the registered spec function"""
            entity_type = func.__module__ + '/' + func.__qualname__[:-5]
            rospy.logdebug('[make]: id=%s, entity=%s, entry=%s' % (id, entity_cls.__name__, entity_type))
            return func(entity_cls.pre_make(entity_type), *args, **kwargs)
        if entity_cls not in REGISTRY:
            """Add entity if this is the first registration"""
            REGISTRY[entity_cls] = dict()
        if id in REGISTRY[entity_cls]:
            """Check if spec of duplicate id corresponds to same spec function"""
            assert spec == REGISTRY[entity_cls][id], f'There is already a {entity_cls.__class__} with id "{id}" registered.'
        REGISTRY[entity_cls][id] = spec
        return spec
    return _register


def make(entity, id, *args, **kwargs):
    """Make an entity with the registered spec function"""
    return REGISTRY[entity][id](*args, **kwargs)


def get_spec(entity, id):
    """Get information on the entity's spec function"""
    help(REGISTRY[entity][id])
    return inspect.signature(REGISTRY[entity][id])
