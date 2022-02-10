from typing import Dict, Any, Optional
import inspect
from yaml import dump

import eagerx.core.register as register
from eagerx.utils.utils import (
    replace_None,
    deepcopy,
    supported_types,
    get_module_type_string,
    exists,
    get_default_params,
    substitute_args,
)


def merge(a, b, path=None):
    "merges b into a"
    # If it is a spec, convert to params
    if path is None:
        path = []
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


class EngineStateSpec(EntitySpec):
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
        from eagerx.core.converters import BaseConverter

        self.identity = BaseConverter.make("Identity")

    def initialize(self, spec_cls):
        try:
            params = register.LOOKUP_TYPES[spec_cls.callback]
        except KeyError as e:
            if spec_cls.__name__ == "EnvNode":
                params = dict()
            else:
                raise

        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        self._set({"default": defaults})

        if "bridge_params" in params:
            params.pop("bridge_params")

        if "targets" in params:
            from eagerx.core.entities import ResetNode

            assert issubclass(
                spec_cls, ResetNode
            ), f"You can only have targets registered for nodes that inherit from the ResetNode baseclass."
            add_ft = True
        else:
            add_ft = False

        # Set default components
        for component, cnames in params.items():
            for cname, msg_type in cnames.items():
                msg_type = get_module_type_string(msg_type)
                if component == "outputs":
                    self._params["default"]["outputs"].append(cname)
                    mapping = dict(
                        msg_type=msg_type,
                        rate="$(default rate)",
                        converter=self.identity.params,
                        space_converter=None,
                    )
                    # Add feedthrough entries for each output if node is a reset node (i.e. when it has a target)
                    if add_ft:
                        mapping_ft = dict(
                            msg_type=msg_type,
                            delay=0.0,
                            window=1,
                            skip=False,
                            external_rate=None,
                            converter=self.identity.params,
                            space_converter=None,
                            address=None,
                        )
                        self._set({"feedthroughs": {cname: mapping_ft}})
                elif component == "inputs":
                    self._params["default"]["inputs"].append(cname)
                    address = "bridge/outputs/tick" if cname == "tick" else None
                    mapping = dict(
                        msg_type=msg_type,
                        delay=0.0,
                        window=1,
                        skip=False,
                        external_rate=None,
                        converter=self.identity.params,
                        space_converter=None,
                        address=address,
                    )
                elif component == "targets":
                    self._params["default"]["targets"].append(cname)
                    mapping = dict(
                        msg_type=msg_type,
                        converter=self.identity.params,
                        space_converter=None,
                        address=None,
                    )
                else:
                    self._params["default"]["states"].append(cname)
                    component = "states"
                    mapping = dict(
                        msg_type=msg_type,
                        converter=self.identity.params,
                        space_converter=None,
                    )
                self._set({component: {cname: mapping}})

    def _remove_component(self, component: str, cname: str):
        self._params[component].pop(cname)
        if cname in self._params["default"][component]:
            self._params["default"][component].remove(cname)

    def remove_input(self, cname: str):
        self._remove_component("inputs", cname)

    def remove_output(self, cname: str):
        self._remove_component("outputs", cname)

    def remove_state(self, cname: str):
        self._remove_component("states", cname)

    def remove_target(self, cname: str):
        self._remove_component("targets", cname)

    def _add_component(self, component: str, cname: str, mapping: dict):
        self._set({component: {cname: mapping}})

    def add_input(
        self,
        cname: str,
        msg_type: Any,
        window: int = 1,
        delay: float = 0.0,
        skip: bool = False,
        external_rate: float = None,
        address: str = None,
        converter: Optional[ConverterSpec] = None,
        space_converter: Optional[ConverterSpec] = None,
    ):
        if not isinstance(msg_type, str):
            assert inspect.isclass(
                msg_type
            ), f'An instance "{msg_type}" of class "{msg_type.__class__}" was provided. Please provide the class instead.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(
            msg_type=msg_type,
            window=window,
            delay=delay,
            skip=skip,
            external_rate=external_rate,
            address=address,
        )
        mapping["converter"] = converter.params if converter else self.identity.params
        mapping["space_converter"] = space_converter.params if space_converter else None
        self._add_component("inputs", cname, mapping)

    def add_output(
        self,
        cname: str,
        msg_type: Any,
        converter: Optional[ConverterSpec] = None,
        space_converter: Optional[ConverterSpec] = None,
    ):
        if not isinstance(msg_type, str):
            assert inspect.isclass(
                msg_type
            ), f'An instance "{msg_type}" of class "{msg_type.__class__}" was provided. Please provide the class instead.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type, rate="$(default rate)")
        mapping["converter"] = converter.params if converter else self.identity.params
        mapping["space_converter"] = space_converter.params if space_converter else None
        self._add_component("outputs", cname, mapping)

    def add_state(self, cname: str, msg_type: Any, space_converter: ConverterSpec):
        if not isinstance(msg_type, str):
            assert inspect.isclass(
                msg_type
            ), f'The provided msg_type "{msg_type}" is not a class. Make sure you are *not* providing an instance of the class, instead of the class itself.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type, space_converter=space_converter.params)
        self._add_component("states", cname, mapping)

    def add_target(self, cname: str, msg_type: Any, converter: Optional[ConverterSpec] = None):
        if not isinstance(msg_type, str):
            assert inspect.isclass(
                msg_type
            ), f'The provided msg_type "{msg_type}" is not a class. Make sure you are *not* providing an instance of the class, instead of the class itself.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type)
        mapping["converter"] = converter.params if converter else self.identity.params
        self._add_component("targets", cname, mapping)

    # CHANGE NODE PARAMETERS.
    @exists
    def set_parameter(
        self,
        parameter: str,
        value: Any,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if component is None or cname is None:
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            if isinstance(value, dict):
                self._params[level][
                    parameter
                ] = None  # Required to clear the parameter instead of merging into it, if it is a dict.
            self._set({level: {parameter: value}})
        else:
            if isinstance(value, dict):
                self._params[component][cname][
                    parameter
                ] = None  # Required to clear the parameter instead of merging into it, if it is a dict.
            self._set({component: {cname: {parameter: value}}})

    @exists
    def set_parameters(
        self,
        mapping: Dict,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if component is None or cname is None:
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            for parameter, value in mapping.items():
                self.set_parameter(parameter, value, level=level)
        else:
            for parameter, value in mapping.items():
                self.set_parameter(parameter, value, component, cname)

    @exists
    def get_parameter(
        self,
        parameter: str,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if component is None or cname is None:
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            return self.params[level].get(parameter)
        else:
            return self.params[component][cname].get(parameter)

    @exists
    def get_parameters(
        self,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if component is None or cname is None:
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            return self.params[level]
        else:
            return self.params[component][cname]

    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    def build(self, ns):
        params = self.params  # Creates a deepcopy
        default = self.get_parameters()  # Creates a deepcopy
        name = default["name"]
        default["node_type"] = params["node_type"]
        entity_id = default["entity_id"]

        # Replace args in .yaml
        context = {
            "ns": {"env_name": ns, "node_name": name},
            "default": params["default"],
        }
        substitute_args(params, context, only=["default", "ns"])

        # Process inputs
        inputs = []
        for cname in default["inputs"]:
            assert (
                cname in params["inputs"]
            ), f'Received unknown {"input"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            assert (
                "targets" not in params or cname not in params["targets"]
            ), f'Input "{cname}" cannot have the same cname as a target. Change either the input or target cname. Check the spec of "{name}" with entity_id "{entity_id}".'
            n = RxInput(name=cname, **params["inputs"][cname])
            inputs.append(n)

        # Process outputs
        outputs = []
        for cname in default["outputs"]:
            assert (
                cname in params["outputs"]
            ), f'Received unknown {"output"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            if "address" in params["outputs"][cname]:
                address = params["outputs"][cname].pop("address")
            else:
                address = "%s/outputs/%s" % (name, cname)
            n = RxOutput(name=cname, address=address, **params["outputs"][cname])
            outputs.append(n)

        states = []
        for cname in default["states"]:
            assert (
                cname in params["states"]
            ), f'Received unknown {"state"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            if (
                "address" in params["states"][cname]
            ):  # if 'env/supervisor', the state address is pre-defined (like an input)
                n = RxState(name=cname, **params["states"][cname])
            else:
                address = "%s/states/%s" % (name, cname)
                n = RxState(name=cname, address=address, **params["states"][cname])
            states.append(n)

        targets = []
        if "targets" in default:
            for cname in default["targets"]:
                assert (
                    cname in params["targets"]
                ), f'Received unknown {"target"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
                n = RxState(name=cname, **params["targets"][cname])
                targets.append(n)

        feedthroughs = []
        if "feedthroughs" in params:
            for cname in default["outputs"]:
                # Add output details (msg_type, space_converter) to feedthroughs
                assert (
                    cname in params["feedthroughs"]
                ), f'Feedthrough "{cname}" must directly correspond to a selected output. Check the spec of "{name}" with entity_id "{entity_id}".'
                assert (
                    params["outputs"][cname]["msg_type"] == params["feedthroughs"][cname]["msg_type"]
                ), f'Mismatch between Msg types of feedthrough "{cname}" and output "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
                if "space_converter" in params["outputs"][cname]:
                    params["feedthroughs"][cname]["space_converter"] = params["outputs"][cname]["space_converter"]
                n = RxFeedthrough(feedthrough_to=cname, **params["feedthroughs"][cname])
                feedthroughs.append(n)

        default["outputs"] = [i.build(ns=ns) for i in outputs]
        default["inputs"] = [i.build(ns=ns) for i in inputs]
        default["states"] = [i.build(ns=ns) for i in states]
        default["targets"] = [i.build(ns=ns) for i in targets]
        default["feedthroughs"] = [i.build(ns=ns) for i in feedthroughs]

        # Create rate dictionary with outputs
        chars_ns = len(ns) + 1
        rate_dict = dict()
        for i in default["outputs"]:
            assert (
                i["rate"] is not None and isinstance(i["rate"], (int, float)) and i["rate"] > 0
            ), f'The rate of node "{name}" (and output cname "{i["name"]}") is misspecified: rate="{i["rate"]}". Make sure that it is of type(rate)=("int", "float",) and rate > 0.'
            address = i["address"][chars_ns:]
            rate_dict[address] = i["rate"]  # {'rate': i['rate']}

        # Put parameters in node namespace (watch out, order of dict keys probably matters...)
        node_params = {name: default, "rate": rate_dict}
        return replace_None(node_params)


class NodeSpec(BaseNodeSpec):
    pass


class EngineNodeSpec(BaseNodeSpec):
    pass


class ResetNodeSpec(BaseNodeSpec):
    pass


class BridgeSpec(BaseNodeSpec):
    pass


class ObjectSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)
        from eagerx.core.converters import BaseConverter

        self.identity = BaseConverter.make("Identity")

    def initialize(self, spec_cls):
        agnostic = register.LOOKUP_TYPES[spec_cls.agnostic]

        # Set default agnostic params
        self._set({"default": agnostic.pop("agnostic_params")})

        # Set default components
        agnostic_spec = AgnosticSpec(dict())
        for component, cnames in agnostic.items():
            for cname, msg_type in cnames.items():
                msg_type = get_module_type_string(msg_type)
                if component == "sensors":
                    mapping = dict(
                        msg_type=msg_type,
                        rate=1,
                        converter=self.identity.params,
                        space_converter=None,
                    )
                elif component == "actuators":
                    mapping = dict(
                        msg_type=msg_type,
                        rate=1,
                        delay=0.0,
                        window=1,
                        skip=False,
                        external_rate=None,
                        converter=self.identity.params,
                        space_converter=None,
                    )
                else:
                    component = "states"
                    mapping = dict(
                        msg_type=msg_type,
                        converter=self.identity.params,
                        space_converter=None,
                    )
                agnostic_spec._set({component: {cname: mapping}})
        spec_cls.agnostic(agnostic_spec)
        self._set(agnostic_spec.params)

    def _initialize_engine_spec(self, bridge_params):
        # Create param mapping
        spec = SpecificSpec(bridge_params)
        graph = self._initialize_object_graph()

        # Add all states to engine-specific params
        spec._set({"states": dict()})
        for component in ["states"]:
            try:
                cnames = self._get_components(component)
            except AssertionError:
                continue
            for cname, params in cnames.items():
                spec._set({component: {cname: None}})
        return spec, graph

    def _add_engine_spec(self, bridge_id, engine_spec, graph):
        # Register EngineGraph
        nodes, actuators, sensors = graph.register()

        # Check that there is no parameter clash between node and object
        obj_name = self.get_parameter("name")
        agnostic_params = self.get_parameters()
        engine_params = engine_spec.get_parameters()
        for node, params in nodes.items():
            default = params["default"]
            node_name = default["name"]
            for key in default.keys():
                if key in ["name", "states", "entity_id"]:
                    continue
                assert (
                    key not in agnostic_params
                ), f'Possible parameter clash detected. "{key}" is a parameter defined both as an agnostic parameter for object "{obj_name}" and as a parameter of enginenode "{node_name}" in bridge implementation "{bridge_id}".'
                assert (
                    key not in engine_params
                ), f'Possible parameter clash detected. "{key}" is a parameter defined both in the registered object_params of bridge "{bridge_id}" for a bridge implementation of object "{obj_name}" and as a parameter of enginenode "{node_name}".'

        # Check that there is no parameter clash between bridge registered object_params and agnostic object params
        for key in engine_params.keys():
            if key in ["states"]:
                continue
            assert (
                key not in agnostic_params
            ), f'Possible parameter clash detected. "{key}" is a parameter defined both as an agnostic parameter for object "{obj_name}" and as a registered object_params of bridge "{bridge_id}" for a bridge implementation of the object.'

        # Substitute engine_specific object_params (registered in the bridge implementation)
        engine_params.pop("states")
        substitute_args(nodes, context={"default": engine_params}, only=["default"])

        # Pop states that were not implemented.
        for cname in list(engine_spec._params["states"].keys()):
            if engine_spec._params["states"][cname] is None:
                engine_spec._params["states"].pop(cname)

        # Set engine_spec
        engine_spec._set({"actuators": actuators})
        engine_spec._set({"sensors": sensors})
        engine_spec._set({"nodes": nodes})
        self._set({bridge_id: engine_spec})

    def _initialize_object_graph(self):
        mapping = dict()
        for component in ["sensors", "actuators"]:
            try:
                mapping[component] = self.get_parameters(component, level="agnostic")
            except AssertionError as e:
                continue

        from eagerx.core.graph_engine import EngineGraph

        graph = EngineGraph.create(**mapping)
        return graph

    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    @exists
    def _get_components(self, component: str):
        return self.get_parameters(component, level="agnostic")

    # CHANGE OBJECT PARAMETERS. level=('default', bridge_id)
    @exists
    def set_parameter(
        self,
        parameter: str,
        value: Any,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if level == "default":
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            if isinstance(value, dict):
                self._params[level][
                    parameter
                ] = None  # Required to clear the parameter instead of merging into it, if it is a dict.
            self._set({level: {parameter: value}})
        elif level == "agnostic":
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            if isinstance(value, dict):
                self._params[component][cname][
                    parameter
                ] = None  # Required to clear the parameter instead of merging into it, if it is a dict.
            self._set({component: {cname: {parameter: value}}})
        else:  # level=bridge_id
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            if isinstance(value, dict):
                self._params[level][component][cname][
                    parameter
                ] = None  # Required to clear the parameter instead of merging into it, if it is a dict.
            self._set({level: {component: {cname: {parameter: value}}}})

    def set_parameters(
        self,
        mapping: Dict,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if level == "default":
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            for parameter, value in mapping.items():
                self.set_parameter(parameter, value)
        elif level == "agnostic":
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            for parameter, value in mapping.items():
                self.set_parameter(parameter, value, component, cname, level)
        else:  # level=bridge_id
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            for parameter, value in mapping.items():
                self.set_parameter(parameter, value, component, cname, level)

    @exists
    def get_parameter(
        self,
        parameter: str,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if level == "default":
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            return self.params[level].get(parameter)
        elif level == "agnostic":
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            return self.params[component][cname].get(parameter)
        else:  # level=bridge_id
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            return self.params[level][component][cname].get(parameter)

    @exists
    def get_parameters(
        self,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        level: str = "default",
    ):
        if level == "default":
            assert (
                component is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            assert (
                cname is None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            return self.params[level]
        elif level == "agnostic":
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            if cname is None:
                return self.params[component]
            else:
                return self.params[component][cname]
        else:  # level=bridge_id
            assert (
                component is not None
            ), f'The parameters cname "{cname}" and component "{component}" can only be specified together with level="<bridge_id>" or "agnostic" for the agnostic definition. If you wish to obtain the selected cnames of a component, please use the component as the "parameter" argument instead.'
            if cname is None:
                return self.params[level][component]
            else:
                return self.params[level][component][cname]

    def build(self, ns, bridge_id):
        params = self.params  # Creates a deepcopy
        default = self.get_parameters()  # Creates a deepcopy
        name = default["name"]
        entity_id = default["entity_id"]

        # Construct context
        context = {"ns": {"env_name": ns, "obj_name": name}, "default": default}
        substitute_args(default, context, only=["default", "ns"])  # First resolve args within the context
        substitute_args(params, context, only=["default", "ns"])  # Resolve rest of params

        # Get agnostic definition
        agnostic = dict()
        for key in list(params.keys()):
            if key not in ["actuators", "sensors", "states"]:
                if not key in ["default", bridge_id]:
                    params.pop(key)
                continue
            agnostic[key] = params.pop(key)

        # Get bridge definition
        bridge = params.pop(bridge_id)
        nodes = bridge.pop("nodes")
        specific = dict()
        for key in list(bridge.keys()):
            if key not in ["actuators", "sensors", "states"]:
                continue
            specific[key] = bridge.pop(key)

        # Replace node names
        for key in list(nodes.keys()):
            key_sub = substitute_args(key, context, only=["ns"])
            nodes[key_sub] = nodes.pop(key)

        # Sensors & actuators
        sensor_addresses = dict()
        rates = dict()
        dependencies = []
        for obj_comp in ["sensors", "actuators"]:
            for obj_cname in default[obj_comp]:
                try:
                    entry = specific[obj_comp][obj_cname]
                except KeyError:
                    raise KeyError(
                        f'"{obj_cname}" was selected in {obj_comp} of "{name}", but there is no implementation for it in bridge "{bridge_id}".'
                    )
                node_name, node_comp, node_cname = (
                    entry["name"],
                    entry["component"],
                    entry["cname"],
                )
                obj_comp_params = agnostic[obj_comp][obj_cname]
                node_params = nodes[node_name]

                # Determine node dependency
                dependencies += entry["dependency"]

                # Set rate
                rate = obj_comp_params["rate"]
                if node_name in rates:
                    assert (
                        rates[node_name] == rate
                    ), f'Cannot specify different rates ({rates[node_name]} vs {rate}) for a enginenode "{node_name}". If this enginenode is used for multiple sensors/components, then their specified rates must be equal.'
                else:
                    rates[node_name] = rate
                node_params["default"]["rate"] = rate
                for o in node_params["default"]["outputs"]:
                    node_params["outputs"][o]["rate"] = rate

                # Set component params
                node_comp_params = nodes[node_name][node_comp][node_cname]

                if obj_comp == "sensors":
                    node_comp_params.update(obj_comp_params)
                    node_comp_params["address"] = f"{name}/{obj_comp}/{obj_cname}"
                    sensor_addresses[f"{node_name}/{node_comp}/{node_cname}"] = f"{name}/{obj_comp}/{obj_cname}"
                else:  # Actuators
                    node_comp_params.update(obj_comp_params)
                    node_comp_params.pop("rate")

        # Get set of node we are required to launch
        dependencies = list(set(dependencies))
        dependencies = [substitute_args(node_name, context, only=["ns"]) for node_name in dependencies]

        # Verify that no dependency is an unlisted actuator node.
        not_selected = [cname for cname in agnostic["actuators"] if cname not in default["actuators"]]
        for cname in not_selected:
            try:
                entry = specific["actuators"][cname]
                node_name, node_comp, node_cname = (
                    entry["name"],
                    entry["component"],
                    entry["cname"],
                )
                assert (
                    node_name not in dependencies
                ), f'There appears to be a dependency on enginenode "{node_name}" for the implementation of bridge "{bridge_id}" for object "{name}" to work. However, enginenode "{node_name}" is directly tied to an unselected actuator "{cname}".'
            except KeyError:
                # We pass here, because if cname is not selected, but also not implemented,
                # we are sure that there is no dependency.
                pass

        # Replace enginenode outputs that have been renamed to sensor outputs
        for node_address, sensor_address in sensor_addresses.items():
            for _, node_params in nodes.items():
                for cname, comp_params in node_params["inputs"].items():
                    if node_address == comp_params["address"]:
                        comp_params["address"] = sensor_address

        # Create states
        states = []
        state_names = []
        obj_comp = "states"
        for obj_cname in default["states"]:
            args = agnostic[obj_comp][obj_cname]
            args["name"] = f"{name}/{obj_comp}/{obj_cname}"
            args["address"] = f"{name}/{obj_comp}/{obj_cname}"
            try:
                args["state"] = specific[obj_comp][obj_cname]
            except KeyError:
                raise KeyError(
                    f'"{obj_cname}" was selected in {obj_comp} of "{name}", but there is no implementation for it in bridge "{bridge_id}".'
                )
            states.append(RxEngineState(**args))
            state_names.append(f'{ns}/{args["name"]}')

        # Create obj parameters
        obj_params = params["default"]

        # Gather node names
        obj_params["node_names"] = [
            f"{ns}/{node_name}" for node_name in list(nodes.keys()) if node_name in dependencies
        ]
        obj_params["state_names"] = state_names

        # Add bridge
        obj_params["bridge"] = bridge

        # Clean up parameters
        for component in ["sensors", "actuators", "states"]:
            try:
                obj_params.pop(component)
            except KeyError:
                pass

        # Add states
        obj_params["states"] = [s.build(ns) for s in states]
        nodes = [EngineNodeSpec(params) for name, params in nodes.items() if name in dependencies]
        return {name: replace_None(obj_params)}, nodes


class AgnosticSpec(EntitySpec):
    @supported_types(str, int, list, float, bool, dict, EntitySpec, None)
    def _set(self, mapping):
        merge(self._params, mapping)

    # CHANGE AGNOSTIC COMPONENT PARAMETERS
    def set_space_converter(self, component: str, cname: str, space_converter: ConverterSpec):
        self.set_parameter(component, cname, "space_converter", space_converter.params)

    @exists
    def set_parameter(self, component: str, cname: str, parameter: str = None, value: Any = None):
        if isinstance(value, dict):
            self._params["component"][cname][parameter] = None
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

    @exists
    def get_parameter(self, component: str, cname: str, parameter: str):
        return self.params[component][cname].get(parameter)

    @exists
    def get_parameters(self, component: str, cname: str):
        return self.params[component][cname]


class SpecificSpec(EntitySpec):
    @exists
    def set_parameter(self, parameter: str, value: Any):
        if isinstance(value, dict):
            self._params[parameter] = None
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
        self._set_component_parameters("states", cname, mapping)

    @exists
    def _set_component_parameters(self, component: str, cname: str, mapping: Dict):
        # assert component in self._params, f"Component '{component}' not found. Available keys(params['default'])={self._params.keys()}."
        # assert cname in self._params[component],  f"Cname '{cname}' not found. Available keys(params[{component}])={self._params[component].keys()}."
        self._set({component: {cname: mapping}})


# REQUIRED FOR BUILDING SPECS


class Params(object):
    def __init__(self, **kwargs):
        # Iterates over provided arguments and sets the provided arguments as class properties
        for key, value in kwargs.items():
            if key == "__class__":
                continue  # Skip if __class__ type
            setattr(self, key, value)


class RxInput(Params):
    def __init__(
        self,
        name: str,
        address: str,
        msg_type: str,
        window: int = 0,
        converter: Dict = None,
        external_rate: float = None,
        space_converter: Dict = None,
        delay: float = 0.0,
        skip: bool = False,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        if not converter:
            from eagerx.core.converters import Identity

            converter = Identity().get_yaml_definition()
            del Identity

        # If space_converter undefined, remove it
        if not space_converter:
            del space_converter

        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxInput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        if not params["external_rate"]:
            params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxOutput(Params):
    def __init__(
        self,
        name: str,
        address: str,
        msg_type: str,
        rate: float,
        converter: Dict = None,
        space_converter: Dict = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            from eagerx.core.converters import Identity

            converter = Identity().get_yaml_definition()
            del Identity

        if not space_converter:
            del space_converter
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxOutput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxFeedthrough(Params):
    def __init__(
        self,
        address: str,
        msg_type: str,
        feedthrough_to: str,
        window: int = 1,
        converter: Dict = None,
        external_rate: float = None,
        space_converter: Dict = None,
        delay: float = 0.0,
        skip: bool = False,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            from eagerx.core.converters import Identity

            converter = Identity().get_yaml_definition()
            del Identity

        if not space_converter:
            del space_converter

        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxFeedthrough, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxState(Params):
    def __init__(
        self,
        name: str,
        address: str,
        msg_type: str,
        converter: Dict = None,
        space_converter: Dict = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            from eagerx.core.converters import Identity

            converter = Identity().get_yaml_definition()
            del Identity

        if not space_converter:
            del space_converter
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxEngineState(Params):
    def __init__(
        self,
        name: str,
        address: str,
        state: Dict,
        msg_type: str,
        converter: Dict = None,
        space_converter: Dict = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            from eagerx.core.converters import Identity

            converter = Identity().get_yaml_definition()
            del Identity

        if not space_converter:
            del space_converter
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxEngineState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params
