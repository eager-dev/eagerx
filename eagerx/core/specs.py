from typing import Dict, Any, Optional
import inspect
from yaml import dump
import copy

import eagerx.core.register as register
from eagerx.core.view import SpecView, GraphView
from eagerx.utils.utils import (
    replace_None,
    deepcopy,
    get_module_type_string,
    get_default_params,
    substitute_args,
)


class EntitySpec(object):
    def __init__(self, params):
        super(EntitySpec, self).__setattr__("_params", params)
        super(EntitySpec, self).__setattr__("_graph", None)

    def __setattr__(self, name, value):
        raise AttributeError("You cannot set the new attributes to EntitySpec.")

    def __str__(self):
        return dump(self._params)

    @property
    @deepcopy
    def params(self):
        return self._params

    def set_graph(self, graph):
        super(EntitySpec, self).__setattr__("_graph", graph)

    @property
    def has_graph(self):
        return True if self._graph else False

    @property
    def graph(self):
        return self._graph


class ConverterSpec(EntitySpec):
    def initialize(self, spec_cls):
        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        with self.config as d:
            d.update(defaults)

    @property
    def config(self):
        return SpecView(self, depth=[])


class EngineStateSpec(EntitySpec):
    def initialize(self, spec_cls):
        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        with self.config as d:
            d.update(defaults)

    @property
    def config(self):
        return SpecView(self, depth=[])


class BaseNodeSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)
        from eagerx.core.converters import BaseConverter

        super(EntitySpec, self).__setattr__("identity", BaseConverter.make("Identity"))

    def _lookup(self, depth):
        name = self._params["config"]["name"]
        if self.has_graph:
            return GraphView(self.graph, depth=[name, depth], name=name)
        else:
            return SpecView(self, depth=[depth], name=name)

    @property
    def inputs(self):
        return self._lookup("inputs")

    @property
    def outputs(self):
        return self._lookup("outputs")

    @property
    def states(self):
        return self._lookup("states")

    @property
    def config(self):
        return self._lookup("config")

    def initialize(self, spec_cls):
        try:
            params = register.LOOKUP_TYPES[spec_cls.callback]
        except KeyError:
            if spec_cls.__name__ == "EnvNode":
                params = dict()
            else:
                raise

        # Set default params
        defaults = get_default_params(spec_cls.initialize)
        with self.config as d:
            d.update(defaults)

        if "bridge_config" in params:
            params.pop("bridge_config")

        if "targets" in params:
            from eagerx.core.entities import ResetNode

            assert issubclass(
                spec_cls, ResetNode
            ), "You can only have targets registered for nodes that inherit from the ResetNode baseclass."
            add_ft = True
        else:
            add_ft = False

        # Set default components
        for component, cnames in params.items():
            for cname, msg_type in cnames.items():
                msg_type = get_module_type_string(msg_type)
                if component == "outputs":
                    self.config.outputs.append(cname)
                    mapping = dict(
                        msg_type=msg_type,
                        rate="$(config rate)",
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
                        with self.feedthroughs as d:
                            d[cname] = mapping_ft
                        # self._set({"feedthroughs": {cname: mapping_ft}})
                elif component == "inputs":
                    self.config.inputs.append(cname)
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
                    self.config.targets.append(cname)
                    mapping = dict(
                        msg_type=msg_type,
                        converter=self.identity.params,
                        space_converter=None,
                        address=None,
                    )
                else:
                    self.config.states.append(cname)
                    component = "states"
                    mapping = dict(
                        msg_type=msg_type,
                        converter=self.identity.params,
                        space_converter=None,
                    )
                with getattr(self, component) as d:
                    d[cname] = mapping

    def _remove_component(self, component: str, cname: str):
        getattr(self, component).pop(cname)
        if cname in self.config[component]:
            self.config[component].remove(cname)

    def remove_input(self, cname: str):
        self._remove_component("inputs", cname)

    def remove_output(self, cname: str):
        self._remove_component("outputs", cname)

    def remove_state(self, cname: str):
        self._remove_component("states", cname)

    def remove_target(self, cname: str):
        self._remove_component("targets", cname)

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
        with self.inputs as d:
            d[cname] = mapping

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
        mapping = dict(msg_type=msg_type, rate="$(config rate)")
        mapping["converter"] = converter.params if converter else self.identity.params
        mapping["space_converter"] = space_converter.params if space_converter else None
        with self.outputs as d:
            d[cname] = mapping

    def add_state(self, cname: str, msg_type: Any, space_converter: ConverterSpec):
        if not isinstance(msg_type, str):
            assert inspect.isclass(
                msg_type
            ), f'The provided msg_type "{msg_type}" is not a class. Make sure you are *not* providing an instance of the class, instead of the class itself.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type, space_converter=space_converter.params)
        with self.states as d:
            d[cname] = mapping

    def add_target(self, cname: str, msg_type: Any, converter: Optional[ConverterSpec] = None):
        if not isinstance(msg_type, str):
            assert inspect.isclass(
                msg_type
            ), f'The provided msg_type "{msg_type}" is not a class. Make sure you are *not* providing an instance of the class, instead of the class itself.'
            msg_type = get_module_type_string(msg_type)
        mapping = dict(msg_type=msg_type)
        mapping["converter"] = converter.params if converter else self.identity.params

        assert "targets" in self._params["config"], f"Cannot add target '{cname}'. Node is not a 'ResetNode'"
        with self.targets as d:
            d[cname] = mapping

    def build(self, ns):
        params = self.params  # Creates a deepcopy
        default = copy.deepcopy(self.config.to_dict())
        name = default["name"]
        default["node_type"] = params["node_type"]
        entity_id = default["entity_id"]

        # Replace args in .yaml
        context = {
            "ns": {"env_name": ns, "node_name": name},
            "config": params["config"],
        }
        substitute_args(params, context, only=["config", "ns"])

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
            if "address" in params["states"][cname]:  # if 'env/supervisor', the state address is pre-defined (like an input)
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


class ResetNodeSpec(BaseNodeSpec):
    @property
    def targets(self):
        return self._lookup("targets")

    @property
    def feedthroughs(self):
        return self._lookup("feedthroughs")


class BridgeSpec(BaseNodeSpec):
    pass


class ObjectSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)
        from eagerx.core.converters import BaseConverter

        super(EntitySpec, self).__setattr__("identity", BaseConverter.make("Identity"))

    def __getattr__(self, name):
        try:
            return super().__getattribute__(name)
        except AttributeError:
            if name in self._params:
                return SpecView(self, depth=[name], name=self._params["config"]["name"])
            else:
                raise

    def _lookup(self, depth):
        name = self._params["config"]["name"]
        if self.has_graph:
            return GraphView(self.graph, depth=[name, depth], name=name)
        else:
            return SpecView(self, depth=[depth], name=name)

    @property
    def sensors(self):
        return self._lookup("sensors")

    @property
    def actuators(self):
        return self._lookup("actuators")

    @property
    def states(self):
        return self._lookup("states")

    @property
    def config(self):
        return self._lookup("config")

    def initialize(self, spec_cls):
        agnostic = register.LOOKUP_TYPES[spec_cls.agnostic]

        # Set default agnostic params
        with self.config as d:
            d.update(agnostic.pop("config"))

        # Set default components
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
                with getattr(self, component) as d:
                    d[cname] = mapping

    def _initialize_bridge_config(self, bridge_id, bridge_config):
        # Add default config
        with getattr(self, bridge_id) as d:
            d.update(bridge_config)
            d["states"] = {}
            # Add all states to engine-specific params
            with d.states as s:
                for cname in self.states.keys():
                    s[cname] = None

    def _add_graph(self, bridge_id, graph):
        # Register EngineGraph
        nodes, actuators, sensors = graph.register()

        # Pop states that were not implemented.
        with getattr(self, bridge_id).states as d:
            for cname in list(getattr(self, bridge_id).states.keys()):
                if d[cname] is None:
                    d.pop(cname)

        # Set engine_spec
        with getattr(self, bridge_id) as d:
            d.actuators = actuators
            d.sensors = sensors
            d.nodes = nodes

    def _initialize_object_graph(self):
        mapping = dict()
        for component in ["sensors", "actuators"]:
            try:
                mapping[component] = getattr(self, component)
            except AttributeError:
                continue

        from eagerx.core.graph_engine import EngineGraph

        graph = EngineGraph.create(**mapping)
        return graph

    def add_bridge(self, bridge_id):
        # Construct context & replace placeholders
        context = {"config": self.config.to_dict()}
        substitute_args(self._params["config"], context, only=["config"])  # First resolve args within the context
        substitute_args(self._params, context, only=["config"])  # Resolve rest of params

        # Add bridge entry
        self._params[bridge_id] = {}
        register.add_bridge(self, bridge_id)

    def build(self, ns, bridge_id):
        params = self.params  # Creates a deepcopy
        default = copy.deepcopy(self.config.to_dict())  # Creates a deepcopy
        name = default["name"]

        # Construct context
        context = {"ns": {"env_name": ns, "obj_name": name}}
        substitute_args(default, context, only=["ns"])  # First resolve args within the context
        substitute_args(params, context, only=["ns"])  # Resolve rest of params

        # Get agnostic definition
        agnostic = dict()
        for key in list(params.keys()):
            if key not in ["actuators", "sensors", "states"]:
                if key not in ["config", bridge_id]:
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
        dependencies = []
        for obj_comp in ["actuators", "sensors"]:
            for obj_cname in default[obj_comp]:
                try:
                    entry_lst = specific[obj_comp][obj_cname]
                except KeyError:
                    raise KeyError(
                        f'"{obj_cname}" was selected in {obj_comp} of "{name}", but there is no implementation for it in bridge "{bridge_id}".'
                    )
                # todo: here we assume a single node implements the actuator --> could be multiple

                # entry = entry_lst
                for entry in reversed(entry_lst):
                    node_name, node_comp, node_cname = entry["name"], entry["component"], entry["cname"]
                    obj_comp_params = agnostic[obj_comp][obj_cname]
                    node_params = nodes[node_name]

                    # Determine node dependency
                    dependencies += entry["dependency"]

                    # Set rate
                    rate = obj_comp_params["rate"]
                    msg_start = f'Different rate specified for {obj_comp[:-1]} "{obj_cname}" and enginenode "{node_name}": '
                    msg_end = "If an enginenode implements a sensor/actuator, their specified rates must be equal."
                    msg_mid = f'{node_params["config"]["rate"]} vs {rate}. '
                    assert node_params["config"]["rate"] == rate, msg_start + msg_mid + msg_end
                    for o in node_params["config"]["outputs"]:
                        msg_mid = f'{node_params["outputs"][o]["rate"]} vs {rate}. '
                        assert node_params["outputs"][o]["rate"] == rate, msg_start + msg_mid + msg_end

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

        # Verify that no dependency is an unlisted actuator node.
        not_selected = [cname for cname in agnostic["actuators"] if cname not in default["actuators"]]
        for cname in not_selected:
            try:
                for entry in specific["actuators"][cname]:
                    node_name, node_comp, node_cname = (entry["name"], entry["component"], entry["cname"])
                    msg = (
                        f'There appears to be a dependency on enginenode "{node_name}" for the implementation of '
                        f'bridge "{bridge_id}" for object "{name}" to work. However, enginenode "{node_name}" is '
                        f'directly tied to an unselected actuator "{cname}". '
                        "The actuator must be selected to resolve the graph."
                    )
                    assert node_name not in dependencies, msg
            except KeyError:
                # We pass here, because if cname is not selected, but also not implemented,
                # we are sure that there is no dependency.
                pass

        # Replace enginenode outputs that have been renamed to sensor outputs
        for node_address, sensor_address in sensor_addresses.items():
            for _, node_params in nodes.items():
                for _cname, comp_params in node_params["inputs"].items():
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
        obj_params = params["config"]

        # Gather node names
        obj_params["node_names"] = [f"{ns}/{node_name}" for node_name in list(nodes.keys()) if node_name in dependencies]
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
        nodes = [NodeSpec(params) for name, params in nodes.items() if name in dependencies]
        return {name: replace_None(obj_params)}, nodes


# REQUIRED FOR BUILDING SPECS


class Component(object):
    def __init__(self, **kwargs):
        # Iterates over provided arguments and sets the provided arguments as class properties
        for key, value in kwargs.items():
            if key == "__class__":
                continue  # Skip if __class__ type
            setattr(self, key, value)


class RxInput(Component):
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


class RxOutput(Component):
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
        # if not space_converter:
        #     raise NotImplementedError(name)
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxOutput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxFeedthrough(Component):
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
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxFeedthrough, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxState(Component):
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
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params


class RxEngineState(Component):
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
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxEngineState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        return params
