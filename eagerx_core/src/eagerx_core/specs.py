from typing import Dict, List, Tuple, Union, Any, Optional
import inspect
from yaml import dump
from eagerx_core.utils.utils import deepcopy, supported_types, get_module_type_string, exists

def merge(a, b, path=None):
    "merges b into a"
    # If it is a spec, convert to params
    if path is None: path = []
    for key in b:
        if isinstance(b[key], EntitySpec):
            b[key] = b[key].params
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


class EntitySpec(object):
    def __init__(self, params):
        self._params = params

    @property
    @deepcopy
    def params(self):
        return self._params


class ConverterSpec(EntitySpec):
    def set_parameter(self, parameter: str, value: Any):
        self.set_parameters({parameter: value})

    @supported_types(str, int, list, float, bool, dict, None)
    def set_parameters(self, mapping: Dict):
        self._params.update(mapping)

    def get_parameter(self, parameter: str, default: Optional[Any] = None):
        return self.params.get(parameter, default)

    def get_parameters(self):
        return self.params


class BaseNodeSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)
        from eagerx_core.entities import BaseConverter
        self.identity = BaseConverter.make('Identity')

    def initialize(self, spec_cls):
        import eagerx_core.registration as register
        params = register.LOOKUP_TYPES[spec_cls.callback]

        # Set default params
        self._set({'default': params.pop('node_params')})

        # Remove object_params (only applies to bridge nodes)
        if 'object_params' in params:
            params.pop('object_params')

        # Set default components
        for component, cnames in params.items():
            for cname, msg_type in cnames.items():
                msg_type = get_module_type_string(msg_type)
                if component == 'outputs':
                    mapping = dict(msg_type=msg_type, rate=1, converter=self.identity.params, space_converter=None)
                elif component == 'inputs':
                    mapping = dict(msg_type=msg_type, rate=1, delay=0.0, window=1, skip=False, external_rate=False,
                                   converter=self.identity.params, space_converter=None)
                elif component == 'targets':
                    mapping = dict(msg_type=msg_type, converter=self.identity.params, space_converter=None)
                else:
                    component = 'states'
                    mapping = dict(msg_type=msg_type, converter=self.identity.params, space_converter=None)
                self._set({component: {cname: mapping}})

    def _remove_component(self, component: str, cname: str):
        self._params[component].pop(cname)
        if cname in self._params['default'][component]:
            self._params['default'][component].remove(cname)

    def remove_input(self, cname: str):
        self._remove_component('inputs', cname)

    def remove_output(self, cname: str):
        self._remove_component('outputs', cname)

    def remove_state(self, cname: str):
        self._remove_component('states', cname)

    def remove_target(self, cname: str):
        self._remove_component('targets', cname)

    def _add_component(self, component: str, cname: str, mapping: dict):
        self._set({component: {cname: mapping}})

    def add_input(self, cname: str, msg_type: Any, window: int = 1, delay: float = 0.0, skip: bool = False, external_rate: float = 0, address: str = None, converter: Optional[ConverterSpec] = None, space_converter: Optional[ConverterSpec] = None):
        if not isinstance(msg_type, str):
            assert inspect.isclass(msg_type), f'An instance "{msg_type}" of class "{msg_type.__class__}" was provided. Please provide the class instead.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type, window=window, delay=delay, skip=skip, external_rate=external_rate, address=address)
        mapping['converter'] = converter.params if converter else self.identity.params
        mapping['space_converter'] = space_converter.params if space_converter else None
        self._add_component('inputs', cname, mapping)

    def add_output(self, cname: str, msg_type: Any, converter: Optional[ConverterSpec] = None, space_converter: Optional[ConverterSpec] = None):
        if not isinstance(msg_type, str):
            assert inspect.isclass(msg_type), f'An instance "{msg_type}" of class "{msg_type.__class__}" was provided. Please provide the class instead.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type)
        mapping['converter'] = converter.params if converter else self.identity.params
        mapping['space_converter'] = space_converter.params if space_converter else None
        self._add_component('outputs', cname, mapping)

    def add_state(self, cname: str, msg_type: Any, space_converter: ConverterSpec):
        if not isinstance(msg_type, str):
            assert inspect.isclass(msg_type), f'The provided msg_type "{msg_type}" is not a class. Make sure you are *not* providing an instance of the class, instead of the class itself.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type, space_converter=space_converter.params)
        self._add_component('states', cname, mapping)

    def add_target(self, cname: str, msg_type: Any, converter: Optional[ConverterSpec] = None):
        if not isinstance(msg_type, str):
            assert inspect.isclass(msg_type), f'The provided msg_type "{msg_type}" is not a class. Make sure you are *not* providing an instance of the class, instead of the class itself.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type)
        mapping['converter'] = converter.params if converter else self.identity.params
        self._add_component('targets', cname, mapping)

    # CHANGE COMPONENT PARAMETER
    @exists
    def set_component_parameter(self, component: str, cname: str, parameter: str, value: Any):
        self._set({component: {cname: {parameter: value}}})

    @exists
    def set_component_parameters(self, component: str, cname: str, mapping: Dict):
        for parameter, value in mapping.items():
            self._set({component: {cname: {parameter: value}}})

    @exists
    def get_component_parameter(self, component: str, cname: str, parameter: str):
        return self.params[component][cname].get(parameter)

    @exists
    def get_component_parameters(self, component: str, cname: str):
        return self.params[component][cname]

    # CHANGE NODE PARAMETERS. level=('default')
    @exists
    def set_parameter(self, parameter: str, value: Any, level='default'):
        self._set({level: {parameter: value}})

    @exists
    def set_parameters(self, mapping: Dict, level='default'):
        for parameter, value in mapping.items():
            self.set_parameter(parameter, value, level=level)

    @exists
    def get_parameter(self, parameter: str, level='default'):
        return self.params[level].get(parameter)

    @exists
    def get_parameters(self, level='default'):
        return self.params[level]

    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)


class NodeSpec(BaseNodeSpec):
    # todo: add assertion on adding states (could make graph engine-specific)
    # todo: define mutation functions here
    pass


class SimNodeSpec(BaseNodeSpec):
    # todo: add assertion on adding states (could make graph engine-specific)
    # todo: define mutation functions here
    pass


class BridgeSpec(BaseNodeSpec):
    pass

class ObjectSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)
        from eagerx_core.entities import BaseConverter
        self.identity = BaseConverter.make('Identity')

    def initialize(self, spec_cls):
        import eagerx_core.registration as register
        agnostic = register.LOOKUP_TYPES[spec_cls.agnostic]

        # Set default agnostic params
        self._set({'default': agnostic.pop('agnostic_params')})

        # Set default components
        for component, cnames in agnostic.items():
            for cname, msg_type in cnames.items():
                msg_type = get_module_type_string(msg_type)
                if component == 'sensors':
                    mapping = dict(msg_type=msg_type, rate=1, converter=self.identity.params, space_converter=None)
                elif component == 'actuators':
                    mapping = dict(msg_type=msg_type, rate=1, delay=0.0, window=1, skip=False, external_rate=False, converter=self.identity.params, space_converter=None)
                else:
                    component = 'states'
                    mapping = dict(msg_type=msg_type, converter=self.identity.params, space_converter=None)
                self._set({component: {cname: mapping}})
        spec_cls.agnostic(self)

    def _initialize_bridge(self, bridge_id, object_params):
        # Create param mapping
        bridge_params = {bridge_id: object_params}
        self._set(bridge_params)
        return bridge_id

    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    # CHANGE COMPONENT
    @exists
    def set_component_parameter(self, bridge_id: str, component: str, cname: str, parameter: str, value: Any):
        self._set_component(bridge_id, component, cname, {parameter: value})

    def _set_component(self, bridge_id: str, component: str, cname: str, mapping: Dict):
        self._set_components(bridge_id, component, {cname: mapping})

    def _set_components(self, bridge_id: str, component: str, mapping: Dict):
        self.set_parameters({component: mapping}, level=bridge_id)

    # CHANGE AGNOSTIC COMPONENT PARAMETERS
    def set_space_converter(self, component: str, cname: str, space_converter: ConverterSpec):
        self.set_agnostic_parameter(component, cname, 'space_converter', space_converter.params)

    @exists
    def set_agnostic_parameter(self, component: str, cname: str, parameter: str = None, value: Any = None):
        self._set({component: {cname: {parameter: value}}})

    @exists
    def set_agnostic_parameters(self, component: str, cname: str, mapping: Dict):
        for parameter, value in mapping.items():
            self._set({component: {cname: {parameter: value}}})

    # CHANGE OBJECT PARAMETERS. level=('default', bridge_id)
    @exists
    def set_parameter(self, parameter: str, value: Any, level='default'):
        self._set({level: {parameter: value}})

    @exists
    def set_parameters(self, mapping: Dict, level='default'):
        for parameter, value in mapping.items():
            self.set_parameter(parameter, value, level=level)

    @exists
    def get_parameter(self, parameter: str, level='default'):
        return self.params[level].get(parameter)

    @exists
    def get_parameters(self, level='default'):
        return self.params[level]


