from typing import Dict, List, Tuple, Union, Any, Optional
import inspect
from yaml import dump
from eagerx_core.utils.utils import replace_None, deepcopy, supported_types, get_module_type_string, exists, get_default_params, substitute_args


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

    def __str__(self):
        return dump(self._params)

    @property
    @deepcopy
    def params(self):
        return self._params


class ConverterSpec(EntitySpec):
    def initialize(self, spec_cls):
        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        self._set(defaults)

    def set_parameter(self, parameter: str, value: Any):
        self.set_parameters({parameter: value})

    @supported_types(str, int, list, float, bool, dict, None)
    def set_parameters(self, mapping: Dict):
        self._params.update(mapping)

    def get_parameter(self, parameter: str, default: Optional[Any] = None):
        return self.params.get(parameter, default)

    def get_parameters(self):
        return self.params

    @supported_types(str, int, list, float, bool, dict, None)
    def _set(self, mapping):
        merge(self._params, mapping)


class SimStateSpec(EntitySpec):
    def initialize(self, spec_cls):
        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        self._set(defaults)

    def set_parameter(self, parameter: str, value: Any):
        self.set_parameters({parameter: value})

    @supported_types(str, int, list, float, bool, dict, None)
    def set_parameters(self, mapping: Dict):
        self._params.update(mapping)

    def get_parameter(self, parameter: str, default: Optional[Any] = None):
        return self.params.get(parameter, default)

    def get_parameters(self):
        return self.params

    @supported_types(str, int, list, float, bool, dict, None)
    def _set(self, mapping):
        merge(self._params, mapping)


class BaseNodeSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)
        from eagerx_core.entities import BaseConverter
        self.identity = BaseConverter.make('Identity')

    def initialize(self, spec_cls):
        import eagerx_core.registration as register
        params = register.LOOKUP_TYPES[spec_cls.callback]

        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        self._set({'default': defaults})

        if 'object_params' in params:
            params.pop('object_params')

        if 'targets' in params:
            from eagerx_core.entities import ResetNode
            assert issubclass(spec_cls, ResetNode), f'You can only have targets registered for nodes that inherit from the ResetNode baseclass.'
            add_ft = True
        else:
            add_ft = False

        # Set default components
        for component, cnames in params.items():
            for cname, msg_type in cnames.items():
                msg_type = get_module_type_string(msg_type)
                if component == 'outputs':
                    mapping = dict(msg_type=msg_type, rate='$(default rate)', converter=self.identity.params, space_converter=None)
                    # Add feedthrough entries for each output if node is a reset node (i.e. when it has a target)
                    if add_ft:
                        mapping_ft = dict(msg_type=msg_type, delay=0.0, window=1, skip=False, external_rate=False,
                                          converter=self.identity.params, space_converter=None, address=None)
                        self._set({'feedthroughs': {cname: mapping_ft}})

                elif component == 'inputs':
                    address = '$(ns env_name)/bridge/outputs/tick' if cname == 'tick' else None
                    mapping = dict(msg_type=msg_type, delay=0.0, window=1, skip=False, external_rate=False,
                                   converter=self.identity.params, space_converter=None, address=address)
                elif component == 'targets':
                    mapping = dict(msg_type=msg_type, converter=self.identity.params, space_converter=None, address=None)
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
        mapping = dict(msg_type=msg_type, rate='$(default rate)')
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

    def build(self, ns):
        from eagerx_core.params.params import RxInput, RxOutput, RxState, RxSimState, RxFeedthrough
        params = self.params  # Creates a deepcopy
        default = self.get_parameters()  # Creates a deepcopy
        name = default['name']
        default['node_type'] = params['node_type']
        entity_id = default['entity_id']

        # Replace args in .yaml
        context = {'ns': {'env_name': ns, 'node_name': name}, 'default': params['default']}
        substitute_args(params, context, only=['default', 'ns'])

        # Process inputs
        inputs = []
        for cname in default['inputs']:
            assert cname in params['inputs'], f'Received unknown {"input"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            assert 'targets' not in params or cname not in params['targets'], f'Input "{cname}" cannot have the same cname as a target. Change either the input or target cname. Check the spec of "{name}" with entity_id "{entity_id}".'
            n = RxInput(name=cname, **params['inputs'][cname])
            inputs.append(n)

        # Process outputs
        outputs = []
        for cname in default['outputs']:
            assert cname in params['outputs'], f'Received unknown {"output"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            if 'address' in params['outputs'][cname]:
                address = params['outputs'][cname].pop('address')
            else:
                address = '%s/outputs/%s' % (name, cname)
            n = RxOutput(name=cname, address=address, **params['outputs'][cname])
            outputs.append(n)

        states = []
        for cname in default['states']:
            assert cname in params['states'], f'Received unknown {"state"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            if 'address' in params['states'][cname]:  # if 'env/supervisor', the state address is pre-defined (like an input)
                n = RxState(name=cname, **params['states'][cname])
            else:
                address = '%s/states/%s' % (name, cname)
                n = RxState(name=cname, address=address, **params['states'][cname])
            states.append(n)

        targets = []
        if 'targets' in default:
            for cname in default['targets']:
                assert cname in params['targets'], f'Received unknown {"target"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
                n = RxState(name=cname, **params['targets'][cname])
                targets.append(n)

        feedthroughs = []
        if 'feedthroughs' in params:
            for cname in default['outputs']:
                # Add output details (msg_type, space_converter) to feedthroughs
                assert cname in params['feedthroughs'], f'Feedthrough "{cname}" must directly correspond to a selected output. Check the spec of "{name}" with entity_id "{entity_id}".'
                assert params['outputs'][cname]['msg_type'] == params['feedthroughs'][cname]['msg_type'], f'Mismatch between Msg types of feedthrough "{cname}" and output "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
                if 'space_converter' in params['outputs'][cname]:
                    params['feedthroughs'][cname]['space_converter'] = params['outputs'][cname]['space_converter']
                n = RxFeedthrough(feedthrough_to=cname, **params['feedthroughs'][cname])
                feedthroughs.append(n)

        default['outputs'] = [i.get_params(ns=ns) for i in outputs]
        default['inputs'] = [i.get_params(ns=ns) for i in inputs]
        default['states'] = [i.get_params(ns=ns) for i in states]
        default['targets'] = [i.get_params(ns=ns) for i in targets]
        default['feedthroughs'] = [i.get_params(ns=ns) for i in feedthroughs]

        # Create rate dictionary with outputs
        chars_ns = len(ns)+1
        rate_dict = dict()
        for i in default['outputs']:
            address = i['address'][chars_ns:]
            rate_dict[address] = i['rate']  # {'rate': i['rate']}

        # Put parameters in node namespace (watch out, order of dict keys probably matters...)
        node_params = {name: default, 'rate': rate_dict}
        return replace_None(node_params)

class NodeSpec(BaseNodeSpec):
    # todo: define mutation functions here
    pass


class SimNodeSpec(BaseNodeSpec):
    # todo: add assertion on adding states (could make graph engine-specific)
    # todo: define mutation functions here
    pass


class ResetNodeSpec(BaseNodeSpec):
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
        agnostic_spec = AgnosticSpec(dict())
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
                agnostic_spec._set({component: {cname: mapping}})
        spec_cls.agnostic(agnostic_spec)
        self._set(agnostic_spec.params)

    def _initialize_engine_spec(self, object_params):
        # Create param mapping
        spec = EngineSpec(object_params)
        graph = self._initialize_object_graph()

        # Add all states to engine-specific params
        for component in ['states']:
            try:
                cnames = self._get_components(component)
            except AssertionError:
                continue
            for cname, params in cnames.items():
                spec._set({component: {cname: None}})
        return spec, graph

    def _add_engine_spec(self, bridge_id, engine_spec, graph):
        nodes, actuators, sensors = graph.register()
        engine_spec._set({'actuators': actuators})
        engine_spec._set({'sensors': sensors})
        engine_spec._set({'nodes': nodes})
        self._set({bridge_id: engine_spec})

    def _initialize_object_graph(self):
        mapping = dict()
        for component in ['sensors', 'actuators']:
            try:
                mapping[component] = self._get_components(component)
            except AssertionError:
                continue

        from eagerx_core.objectgraph import ObjectGraph
        graph = ObjectGraph.create(**mapping)
        return graph

    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    @exists
    def _get_components(self, component: str):
        return self.get_parameters(level=component)

    # CHANGE COMPONENT
    @exists
    def set_component_parameter(self, bridge_id: str, component: str, cname: str, parameter: str, value: Any):
        self.set_component_parameters(bridge_id, component, cname, {parameter: value})

    @exists
    def set_component_parameters(self, bridge_id: str, component: str, cname: str, mapping: Dict):
        self._set_components(bridge_id, component, {cname: mapping})

    def _set_components(self, bridge_id: str, component: str, mapping: Dict):
        self.set_parameters({component: mapping}, level=bridge_id)

    @exists
    def get_component_parameter(self, bridge_id: str, component: str, cname: str, parameter: str):
        return self.params[bridge_id][component][cname].get(parameter)

    @exists
    def get_component_parameters(self, bridge_id: str, component: str, cname: str):
        return self.params[bridge_id][component][cname]

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

    # def build(self, ns, bridge_id):


class AgnosticSpec(EntitySpec):
    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    # CHANGE AGNOSTIC COMPONENT PARAMETERS
    def set_space_converter(self, component: str, cname: str, space_converter: ConverterSpec):
        self.set_parameter(component, cname, 'space_converter', space_converter.params)

    @exists
    def set_parameter(self, component: str, cname: str, parameter: str = None, value: Any = None):
        self._set({component: {cname: {parameter: value}}})

    @exists
    def set_parameters(self, component: str, cname: str, mapping: Dict):
        for parameter, value in mapping.items():
            self._set({component: {cname: {parameter: value}}})

    @exists
    def set_parameter(self, component: str, cname: str, parameter: str = None, value: Any = None):
        self._set({component: {cname: {parameter: value}}})

    @exists
    def set_parameters(self, component: str, cname: str, mapping: Dict):
        for parameter, value in mapping.items():
            self._set({component: {cname: {parameter: value}}})


class EngineSpec(EntitySpec):
    @exists
    def set_parameter(self, parameter: str, value: Any):
        self._set({parameter: value})

    @exists
    def set_parameters(self, mapping: Dict):
        for parameter, value in mapping.items():
            self.set_parameter(parameter, value)

    @exists
    def get_parameter(self, parameter: str):
        return self.params.get(parameter)

    @exists
    def get_parameters(self):
        return self.params

    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    def set_state_parameter(self, cname: str, parameter: str, value: Any):
        self.set_state(cname, {parameter: value})

    def set_state(self, cname: str, mapping: Dict):
        self._set_component_parameters('states', cname, mapping)

    @exists
    def _set_component_parameters(self, component: str, cname: str, mapping: Dict):
        # assert component in self._params, f"Component '{component}' not found. Available keys(params['default'])={self._params.keys()}."
        # assert cname in self._params[component],  f"Cname '{cname}' not found. Available keys(params[{component}])={self._params[component].keys()}."
        self._set({component: {cname: mapping}})
