import functools
from eagerx_core.params import (RxBridgeParams,
                                RxObjectParams,
                                RxNodeParams,
                                RxSimNodeParams,
                                RxConverterParams,
                                RxSpaceConverterParams,
                                RxSimStateParams,
                                )

# Global registry with registered entities (bridges, objects, nodes, converters, simnodes, etc..)
REGISTRY = dict()


@functools.singledispatch
def _build_entity(entity, *args, **kwargs):
    """Flatten a data point from a space.

    This is useful when e.g. points from spaces must be passed to a neural
    network, which only understands flat arrays of floats.

    Accepts an entity and the params to the entity's build function.
    Raises ``NotImplementedError`` if the entity is not defined in ``REGISTRY``.
    """
    raise NotImplementedError(f"Unknown entity: `{entity}`")


@_build_entity.register(RxObjectParams)
def build_node(entity, *args, **kwargs):
    raise NotImplementedError(f"Entity: `{entity}`")
    return entity


@_build_entity.register(RxBridgeParams)
def build_converter(entity, *args, **kwargs):
    raise NotImplementedError(f"Entity: `{entity}`")
    entity.create_default_parameters()
    # todo: move below to entity specific build.
    entity.add_input('in_1', window=1.0, skip=True, delay=1.0, converter=converter, space_converter=space_converter)
    entity.add_output('in_1', converter=converter, space_converter=space_converter)
    return entity


@_build_entity.register(RxNodeParams)
@_build_entity.register(RxSimNodeParams)
def build_node(entity, *args, **kwargs):
    raise NotImplementedError(f"Entity: `{entity}`")
    return entity


@_build_entity(RxSimStateParams)
@_build_entity.register(RxConverterParams)
@_build_entity.register(RxSpaceConverterParams)
def build_converter(entity, *args, **kwargs):
    raise NotImplementedError(f"Entity: `{entity}`")
    return entity


def register(id, entity_cls):
    """Register a build function to build an entity"""
    def _register(func):
        entry = func.__module__ + '/' + func.__qualname__
        print('[third]: id=%s, entity=%s, entry=%s' % (id, entity_cls.__name__, entry))
        @functools.wraps(functools.partial(func, None))
        def build_entity(*args, **kwargs):
            entry = func.__module__ + '/' + func.__qualname__
            print('[fourth]: id=%s, entity=%s, entry=%s' % (id, entity_cls.__class__, entry))
            entity = _build_entity(entity_cls(), *args, **kwargs)
            return func(entity, *args, **kwargs)
        REGISTRY[entity_cls][id] = build_entity
        return build_entity
    return _register


def make(entity, id, *args, **kwargs):
    return REGISTRY[entity][id](*args, **kwargs)


# class TestNode(object):
#
#     @staticmethod
#     @register('test_node', NodeParams)
#     def build(node: NodeParams, name) -> NodeParams:
#         print(f"Yo {name}, together we are the awesomest! Params set to {node}")
#         return node
#
#
# class TestConverter(object):
#
#     @staticmethod
#     @register('test_converter', ConverterParams)
#     def build(converter: ConverterParams, name) -> ConverterParams:
#         print(f"Yo {name}, together we are the awesomest! Entity set to {converter}")
#         return converter

# if __name__ == '__main__':
    # todo: implement classmethod for RxParams that implements registration.make(entity, id)
    # todo: implement building methods for RxParams to construct .yaml
    # todo: How to infer msg_types from node class?
    # todo: How to pass through node args to space_converter of observation
    # todo: render node does not work for test bridge. Why does display change to True?
    # todo: action_node --> set window=0,  None message for empty

    # node = NodeParams.make('test_node', name='node_name')
    # converter = ConverterParams.make('test_converter', name='conv_name')

    # from inspect import signature
    # print(signature(PLUGINS[NodeParams]['test_node']))