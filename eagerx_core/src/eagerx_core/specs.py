from typing import Dict, List, Tuple, Union, Any, Optional
import inspect
from eagerx_core.utils.utils import deepcopy, supported_types, get_module_type_string


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

    @supported_types(str, int, list, float, bool, dict, None)
    def _add_component(self, component: str, cname: str, mapping: dict):
        self._params[component][cname] = mapping

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

    def set_parameter(self, parameter: str, value: Any):
        self.set_parameters({parameter: value})

    @supported_types(str, int, list, float, bool, dict, None)
    def set_parameters(self, mapping: Dict):
        self._params['default'].update(mapping)

    def get_parameter(self, parameter: str, default: Optional[Any] = None):
        return self.params['default'].get(parameter, default)

    def get_parameters(self):
        return self.params['default']

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
    def set_parameter(self, parameter: str, value: Any):
        self.set_parameters({parameter: value})

    @supported_types(str, int, list, float, bool, dict, None)
    def set_parameters(self, mapping: Dict):
        self._params['default'].update(mapping)

    def get_parameter(self, parameter: str, default: Optional[Any] = None):
        return self.params['default'].get(parameter, default)

    def get_parameters(self):
        return self.params['default']


