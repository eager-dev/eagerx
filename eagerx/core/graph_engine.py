import yaml
from copy import deepcopy
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Union, Dict, Tuple, Optional, Any

yaml.Dumper.ignore_aliases = lambda *args: True  # todo: check if needed.

from eagerx.utils.utils import (
    get_opposite_msg_cls,
    get_cls_from_string,
    substitute_args,
    msg_type_error,
)
from eagerx.utils.network_utils import (
    episode_graph,
    plot_graph,
    color_nodes,
    color_edges,
    is_stale,
)
from eagerx.core.specs import EngineNodeSpec, ConverterSpec


class EngineGraph:
    def __init__(self, state: Dict):
        self._state = state

    def __str__(self):
        return yaml.dump(self._state)

    @classmethod
    def create(
        cls,
        actuators: Optional[List[Dict]] = None,
        sensors: Optional[List[Dict]] = None,
        nodes: Optional[List] = None,
    ):

        if nodes is None:
            nodes = []
        if isinstance(nodes, EngineNodeSpec):
            nodes = [nodes]

        from eagerx.core.entities import EngineNode
        from eagerx.core.converters import Identity

        identity_conv = Identity().get_yaml_definition()

        # Create actuator node
        outputs = []
        spec = EngineNode.pre_make(None, None)
        spec.set_parameter("name", "actuators")
        nodes.append(spec)
        for cname, params in actuators.items():
            # We use identity converter instead of params['converter'], because this output converter is a "placeholder".
            # When building the actuator node (being the connected engine node), this output converter is not even initialized.
            spec.add_output(
                cname,
                msg_type=params["msg_type"],
                converter=ConverterSpec(identity_conv),
                space_converter=ConverterSpec(params["space_converter"]),
            )
            spec.add_input(
                cname,
                msg_type=params["msg_type"],
                skip=params["skip"],
                converter=ConverterSpec(identity_conv),
                space_converter=ConverterSpec(params["space_converter"]),
            )
            outputs.append(cname)
        spec.set_parameter("outputs", outputs)

        # Create sensor node
        inputs = []
        spec = EngineNode.pre_make(None, None)
        spec.set_parameter("name", "sensors")
        nodes.append(spec)
        for cname, params in sensors.items():
            # We use identity converter instead of params['converter'], because this input converter is a "placeholder".
            # When building the sensor node (being the connected engine node), this input converter is not even initialized.
            spec.add_input(
                cname,
                msg_type=params["msg_type"],
                converter=ConverterSpec(identity_conv),
                space_converter=ConverterSpec(params["space_converter"]),
            )
            inputs.append(cname)
        spec.set_parameter("inputs", inputs)

        # Create a state
        state = dict(nodes=dict(), connects=list())
        cls._add(state, nodes)
        return cls(state)

    def add(self, nodes: Union[EngineNodeSpec, List[EngineNodeSpec]]):
        """
        Add a node or object to the state of this graph.
        """
        self._add(self._state, nodes)

    @staticmethod
    def _add(state: Dict, nodes: Union[Any, List]):
        """
        Add a node/object to the provided state.
        """
        if not isinstance(nodes, list):
            nodes = [nodes]

        for node in nodes:
            name = node.get_parameter("name")
            assert name not in state["nodes"], (
                'There is already a node or object registered in this graph with name "%s".' % name
            )

            # Add node to state
            state["nodes"][name] = dict()
            state["nodes"][name]["params"] = node._params
            state["nodes"][name]["default"] = node.params

    def remove(self, names: Union[str, List[str]]):
        """
        Removes a node.
        First removes all associated connects from self._state.
        Then, removes node/object from self._state.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            self._exist(self._state, name)
            for source, target in deepcopy(self._state["connects"]):
                if name in [source[0], target[0]]:
                    if source[0] == "actuators":
                        actuator = source[2]
                        source = None
                    else:
                        actuator = None
                        source = source
                    if target[0] == "sensors":
                        sensor = target[2]
                        target = None
                    else:
                        sensor = None
                        target = target
                    self.disconnect(source, target, actuator, sensor)
            self._state["nodes"].pop(name)

    def _remove(self, names: Union[str, List[str]]):
        """
        First removes all associated connects from self._state.
        Then, removes node/object from self._state.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            self._exist(self._state, name)
            for source, target in deepcopy(self._state["connects"]):
                if name in [source[0], target[0]]:
                    if source[0] == "actuators":
                        actuator = source[2]
                        source = None
                    else:
                        actuator = None
                        source = source
                    if target[0] == "sensors":
                        sensor = target[2]
                        target = None
                    else:
                        sensor = None
                        target = target
                    self._disconnect(source, target, actuator, sensor)
            self._state["nodes"].pop(name)

    def add_component(
        self,
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        """
        Adds a component entry to the selection list.
        """
        # assert only action, only observation, only name, component, cname
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name, component, cname = ("actuators", "outputs", actuator)
        if sensor:
            name, component, cname = ("sensors", "inputs", sensor)
        self._add_component(name, component, cname)

        # if (name is not None) and (component is not None) and (cname is not None):  # component parameter
        self._add_component(name, component, cname)

    def _add_component(self, name: str, component: str, cname: str):
        """
        Adds a component entry to the selection list.
        """
        # Check that cname exists
        self._exist(self._state, name, component=component, cname=cname)

        # Add cname to selection list if it is not already selected
        params = self._state["nodes"][name]["params"]
        assert cname not in params["default"][component], '"%s" already selected in "%s" under %s.' % (
            cname,
            name,
            component,
        )
        params["default"][component].append(cname)

    def remove_component(
        self,
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        """
        Removes a component entry from the selection list. It will first disconnect all connections in connect.
        """
        # assert only action, only observation, only name, component, cname
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name, component, cname = ("actuators", "outputs", actuator)
        if sensor:
            name, component, cname = ("sensors", "inputs", sensor)
        self._remove_component(name, component, cname)

    def _remove_component(self, name: str, component: str, cname: str):
        """
        Removes a component entry from the selection list. It will first disconnect all connections in connect.
        """
        self._is_selected(self._state, name, component, cname)

        # Disconnect component entry
        self._disconnect_component(name, component, cname)

        # Remove cname from selection list
        params = self._state["nodes"][name]["params"]
        params["default"][component].remove(cname)

    def connect(
        self,
        source: Optional[Tuple[str, str, str]] = None,
        target: Optional[Tuple[str, str, str]] = None,
        actuator: str = None,
        sensor: str = None,
        address: str = None,
        converter: Optional[Dict] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
        external_rate: Optional[float] = None,
    ):
        assert not address or (
            source is None and actuator is None and sensor is None
        ), f'You cannot provide an external address "{address}" together with a sensor, actuator, or source.'
        assert (
            not source or not actuator
        ), f'You cannot specify an actuator if you wish to connect actuator "{actuator}", as the actuator will act as the source.'
        assert (
            not target or not sensor
        ), f'You cannot specify a target if you wish to connect sensor "{sensor}", as the sensor will act as the target.'
        assert not (actuator and sensor), f"You cannot connect an actuator directly to a sensor."
        if address:
            curr_address = self.get_parameter("address", *target)
            assert (
                curr_address is None
            ), f'Cannot connect target "{target}" to external address "{address}", because it is already connected to an external address "{curr_address}". Disconnect target first.'
            self.set_parameter("address", address, *target)
            assert (
                external_rate is not None
            ), f'When providing an external address "{address}", an external rate must also be provided.'
            assert external_rate > 0, f'Invalid external rate "{external_rate}". External rate must be > 0.'
            self.set_parameter("external_rate", external_rate, *target)
            return
        else:
            assert external_rate is None, f"An external rate may only be provided in combination with an address."

        if isinstance(converter, ConverterSpec):
            converter = converter.params

        if actuator:  # source = actuator
            assert (
                converter is None
            ), f'Cannot specify an input converter when connecting actuator "{actuator}". You can only do that in the agnostic object definition.'
            assert (
                window is None
            ), f'Cannot specify a window when connecting actuator "{actuator}". You can only do that in the agnostic object definition.'
            assert (
                delay is None
            ), f'Cannot specify a delay when connecting actuator "{actuator}". You can only do that in the agnostic object definition.'
            assert (
                skip is None
            ), f'Cannot specify a skip when connecting actuator "{actuator}". You can only do that in the agnostic object definition.'
            source = ("actuators", "outputs", actuator)
        elif sensor:  # target = sensor
            assert (
                converter is None
            ), f'Cannot specify an input converter when connecting sensor "{sensor}". You can only do that in the agnostic object definition.'
            assert (
                window is None
            ), f'Cannot specify a window when connecting sensor "{sensor}". You can only do that in the agnostic object definition.'
            assert (
                delay is None
            ), f'Cannot specify a delay when connecting sensor "{sensor}". You can only do that in the agnostic object definition.'
            assert (
                skip is None
            ), f'Cannot specify a skip when connecting sensor "{sensor}". You can only do that in the agnostic object definition.'
            target = ("sensors", "inputs", sensor)
        self._connect(source, target, converter, window, delay, skip)

    def _connect(
        self,
        source: Optional[Tuple[str, str, str]] = None,
        target: Optional[Tuple[str, str, str]] = None,
        converter: Optional[Dict] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
    ):
        """
        Method to connect a source to a target. For actuators/sensors, first a (new) disconnected entry must be created,
        after which an additional call to connect_actuator/sensor is required before calling this method.
        For more info, see self.connect.
        """
        if isinstance(converter, ConverterSpec):
            converter = converter.params

        if isinstance(source, tuple):
            source = list(source)
        if isinstance(target, tuple):
            target = list(target)

        # Perform checks on source
        source_name, source_comp, source_cname = source
        self._is_selected(self._state, source_name, source_comp, source_cname)

        # Perform checks on target
        target_name, target_comp, target_cname = target
        self._is_selected(self._state, target_name, target_comp, target_cname)

        # Make sure that target is not already connected.
        curr_address = self.get_parameter("address", *target)
        assert (
            curr_address is None
        ), f'Cannot connect target "{target}" to source "{source}", because it is already connected to an external address "{curr_address}". Disconnect target first.'
        for _, t in self._state["connects"]:
            t_name, t_comp, t_cname = t
            assert not (
                target_name == t_name and target_comp == t_comp and target_cname == t_cname
            ), f'Target "{target}" is already connected to source "{_}"'

        # Add properties to target params
        if converter is not None:
            self.set_parameter("converter", converter, target_name, target_comp, target_cname)
        if window is not None:
            self.set_parameter("window", window, target_name, target_comp, target_cname)
        if delay is not None:
            self.set_parameter("delay", delay, target_name, target_comp, target_cname)
        if skip is not None:
            self.set_parameter("skip", skip, target_name, target_comp, target_cname)

        # Add connection
        connect = [source, target]
        EngineGraph.check_msg_type(source, target, self._state)
        self._state["connects"].append(connect)

    def disconnect(
        self,
        source: Optional[Tuple[str, str, str]] = None,
        target: Optional[Tuple[str, str, str]] = None,
        actuator: str = None,
        sensor: str = None,
    ):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        self._disconnect(source, target, actuator, sensor)

    def _disconnect(
        self,
        source: Optional[Tuple[str, str, str]] = None,
        target: Optional[Tuple[str, str, str]] = None,
        actuator: str = None,
        sensor: str = None,
    ):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        assert (
            not source or not actuator
        ), f'You cannot specify a source if you wish to disconnect actuator "{actuator}", as the actuator will act as the source.'
        assert (
            not target or not sensor
        ), f'You cannot specify a target if you wish to disconnect sensor "{sensor}", as the sensor will act as the target.'
        assert not (
            sensor and actuator
        ), f"You cannot disconnect an actuator from an sensor, as such a connection cannot exist."
        if isinstance(source, tuple):
            source = list(source)
        if isinstance(target, tuple):
            target = list(target)

        # Create source & target entries
        if actuator:
            source = ["actuators", "outputs", actuator]
        if sensor:
            target = ["sensors", "inputs", sensor]

        # Check if connection exists
        self._is_selected(self._state, *target)

        # Check if connection exists
        if source is None:
            assert (
                self.get_parameter("address", *target) is not None
            ), f"Cannot disconnect. No source was provided, and the target={target} also does not have an address specified."
            self.set_parameter("address", None, *target)
            self.set_parameter("external_rate", None, *target)
        else:
            self._is_selected(self._state, *source)
            connect_exists = False
            idx_connect = None
            for idx, c in enumerate(self._state["connects"]):
                if source == c[0] and target == c[1]:
                    connect_exists = True
                    idx_connect = idx
                    break
            assert (
                connect_exists
            ), f"The connection with source={source} and target={target} cannot be removed, because it does not exist."

            # Pop the connection from the state
            self._state["connects"].pop(idx_connect)

            # Reset source params to disconnected state
            if actuator:
                pass
                # self._disconnect_action(action)
            else:
                # Nothing to do here (for now)
                source_name, source_comp, source_cname = source
                source_params = self._state["nodes"][source_name]["params"]

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/skip/converter.
        if sensor:
            pass
            # self._disconnect_observation(observation)
        else:
            target_name, target_comp, target_cname = target
            target_params = self._state["nodes"][target_name]["params"]
            target_params[target_comp][target_cname] = self._state["nodes"][target_name]["default"][target_comp][
                target_cname
            ]

    def _disconnect_component(self, name: str, component: str, cname: str):
        """
        Disconnects all associated connects from self._state.
        """
        was_connected = False
        for source, target in deepcopy(self._state["connects"]):
            self._is_selected(self._state, *source)
            self._is_selected(self._state, *target)
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            if source_name == "actuators":
                actuator = source_cname
                source = None
            else:
                actuator = None
                source = source
            if target_name == "sensors":
                sensor = target_cname
                target = None
            else:
                sensor = None
                target = target
            if name == source_name and component == source_comp and cname == source_cname:
                self.disconnect(source, target, actuator, sensor)
                was_connected = True
        return was_connected

    def rename(
        self,
        old,
        new,
        name: Optional[str] = None,
        component: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        """
        Renames the node/object, or action/observation if specified.
        """
        self._correct_signature(name=name, component=component, sensor=sensor, actuator=actuator)
        if actuator:
            name = "actuators"
            component = "outputs"
        if sensor:
            name = "sensors"
            component = "inputs"
        if (name is not None) and (component is not None):  # component renaming
            self._rename_component(name, component, old_cname=old, new_cname=new)
        elif (name is None) and (component is None):  # node/object renaming
            self._rename_entity(old_name=old, new_name=new)
        else:
            raise ValueError("Either the arguments {name, component} are None, or they must both be specified.")

    def _rename_component(self, name: str, component: str, old_cname: str, new_cname: str):
        """
        Renames the component name (cname) of an entity (node/object) in _state['nodes'] and self._state[connects].
        We cannot change names for node/object components, because their python implementation could depend on it.
        """
        self._exist(self._state, name, component=component, cname=old_cname)
        default = self._state["nodes"][name]["default"]
        params = self._state["nodes"][name]["params"]

        # For now, we only support changing action/observation cnames
        assert name in [
            "sensors",
            "actuators",
        ], f'Cannot change "{old_cname}" of "{name}". Only name changes to observations and actions are supported.'
        assert new_cname not in params[component], f'"{new_cname}" already defined in "{name}" under {component}.'

        # Rename cname in params
        for d in (params, default):
            if component in d and old_cname in d[component]:
                assert new_cname not in d[component], f'"{new_cname}" already defined in "{name}" under {component}.'
                d[component][new_cname] = d[component].pop(old_cname)
            if component in d["default"] and old_cname in d["default"][component]:
                assert (
                    new_cname not in d["default"][component]
                ), f'"{new_cname}" already defined in "{name}" under {component}.'
                d["default"][component].remove(old_cname)
                d["default"][component].append(new_cname)

        # Rename cname in all connects
        for source, target in self._state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target

            if source_comp == component and source_cname == old_cname:
                source[2] = new_cname
            if target_comp == component and target_cname == old_cname:
                target[2] = new_cname

    def _rename_entity(self, old_name: str, new_name: str):
        """
        Renames the entity (node/object) in _state['nodes'] and self._state[connects]
        """
        self._exist(self._state, old_name)
        assert old_name not in [
            "sensors",
            "actuators",
        ], f'Node name "{old_name}" is fixed and cannot be changed.'
        assert (
            new_name not in self._state["nodes"]
        ), f'There is already a node or object registered in this graph with name "{new_name}".'

        # Rename entity in params
        self._state["nodes"][new_name] = self._state["nodes"].pop(old_name)
        self._state["nodes"][new_name]["default"]["default"]["name"] = new_name
        self._state["nodes"][new_name]["params"]["default"]["name"] = new_name

        # Rename in all connects
        for source, target in self._state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target

            if source_name == old_name:
                source[0] = new_name
            if target_name == old_name:
                target[0] = new_name

    def set_parameter(
        self,
        parameter: str,
        value: Any,
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        """
        A wrapper to set a single parameter. See set_parameters for more info.
        """
        return self.set_parameters(
            {parameter: value},
            name=name,
            component=component,
            cname=cname,
            actuator=actuator,
            sensor=sensor,
        )

    def set_parameters(
        self,
        mapping: Dict[str, Any],
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        """
        Sets parameters in self._state, based on the node/object name. If a component and cname are specified, the
        parameter will be set there. Else, the parameter is set under the "default" key.
        For objects, parameters are set under their agnostic definitions of the components (so not bridge specific).
        If a converter is added, we check if the msg_type changes with the new converter. If so, the component is
        disconnected. See _set_converter for more info.
        """
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name = "actuators"
            component = "outputs"
            cname = actuator
        if sensor:
            name = "sensors"
            component = "inputs"
            cname = sensor
        self._exist(self._state, name, component=component, cname=cname)

        if (component is not None) and (cname is not None):  # component parameter
            for parameter, value in mapping.items():
                self._exist(
                    self._state,
                    name,
                    component=component,
                    cname=cname,
                    parameter=parameter,
                )
                if parameter == "converter":
                    if isinstance(value, ConverterSpec):
                        value = value.params
                    self._set_converter(name, component, cname, value)
                else:
                    self._state["nodes"][name]["params"][component][cname][parameter] = value
        else:  # Default parameter
            for parameter, value in mapping.items():
                self._exist(
                    self._state,
                    name,
                    component=component,
                    cname=cname,
                    parameter=parameter,
                )
                assert parameter not in [
                    "sensors",
                    "actuators",
                    "targets",
                    "states",
                    "inputs",
                    "outputs",
                ], "You cannot modify component parameters with this function. Use _add/remove_component(..) instead."
                assert parameter not in ["name"], "You cannot rename with this function. Use rename_(name) instead."
                default = self._state["nodes"][name]["params"]["default"]
                default[parameter] = value

    def _set_converter(self, name: str, component: str, cname: str, converter: Dict):
        """
        Replaces the converter specified for a node's/object's I/O.
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
        """
        self._exist(self._state, name, component=component, cname=cname, parameter="converter")
        params = self._state["nodes"][name]["params"]

        # Check if converted msg_type of old converter is equal to the msg_type of newly specified converter
        msg_type = get_cls_from_string(params[component][cname]["msg_type"])
        converter_old = params[component][cname]["converter"]
        msg_type_ros_old = get_opposite_msg_cls(msg_type, converter_old)
        msg_type_ros_new = get_opposite_msg_cls(msg_type, converter)
        if not msg_type_ros_new == msg_type_ros_old:
            was_disconnected = self._disconnect_component(name, component, cname)
        else:
            was_disconnected = False

        # If disconnected action/observation, we cannot add converter so raise error.
        assert not (
            was_disconnected and name in ["actuators", "sensors"]
        ), f'Cannot change the converter of actuator/sensor "{cname}", as it changes the msg_type from "{msg_type_ros_old}" to "{msg_type_ros_new}"'

        # Replace converter
        params[component][cname]["converter"] = converter

    def get_parameter(
        self,
        parameter: str,
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
        default=None,
    ):
        """
        Get node/object parameters. If component and cname are specified, get the parameter of them instead.
        If default was specified, get default parameter instead. Else, raise an error.
        """
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name = "actuators"
            component = "outputs"
            cname = actuator
        if sensor:
            name = "sensors"
            component = "inputs"
            cname = sensor
        try:
            self._exist(self._state, name, component, cname, parameter=parameter)
            if (component is not None) and (cname is not None):  # component parameter
                return self._state["nodes"][name]["params"][component][cname][parameter]
            else:  # default parameter
                return self._state["nodes"][name]["params"]["default"][parameter]
        except AssertionError:
            if default:
                return default
            else:
                raise

    def get_parameters(
        self,
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        """
        Get all node/object parameters. If component and cname are specified, get the parameters of them instead.
        """
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name = "actuators"
            component = "outputs"
            cname = actuator
        if sensor:
            name = "sensors"
            component = "inputs"
            cname = sensor
        self._exist(self._state, name, component, cname)
        if (component is not None) and (cname is not None):  # component parameter
            return deepcopy(self._state["nodes"][name]["params"][component][cname])
        else:  # default parameter
            return deepcopy(self._state["nodes"][name]["params"]["default"])

    def _reset_converter(self, name: str, component: str, cname: str):
        """
        Replaces the converter specified for a node's/object's I/O defined in self._state[name]['default'].
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
        """
        default = self._state["nodes"][name]["default"]
        self._exist(
            self._state,
            name,
            component=component,
            cname=cname,
            parameter="converter",
            check_default=True,
        )

        # Grab converter from the default params
        converter_default = default[component][cname]["converter"]

        # Replace the converter with the default converter
        self._set_converter(name, component, cname, converter_default)

    def _node_depenencies(self, state):
        import networkx as nx

        state = deepcopy(state)
        G = self._generate_graph(state)
        G_rev = G.reverse(copy=True)

        # fig_env, ax_env = plt.subplots(nrows=1, ncols=1)
        # ax_env.set_title('Engine-specific graph')
        # _, _, _, pos = plot_graph(G, k=2, ax=ax_env)
        # plt.show()
        #
        # fig_env, ax_env = plt.subplots(nrows=1, ncols=1)
        # ax_env.set_title('Engine-specific graph (reversed)')
        # _, _, _, _ = plot_graph(G_rev, k=2, ax=ax_env, pos=pos)
        # plt.show()

        # Determine sensor dependencies
        dependencies = dict(sensors=dict(), actuators=dict())
        for cname in state["nodes"]["sensors"]["params"]["inputs"]:
            dependencies["sensors"][cname] = []
            target_name = f"sensors/{cname}"
            descendants = nx.descendants(G_rev, target_name)
            for source in descendants:
                node_name, source_cname = source.split("/")
                if node_name in ["actuators", "sensors"]:
                    continue
                dependencies["sensors"][cname].append(node_name)

        # Determine actuator dependency
        for cname in state["nodes"]["actuators"]["params"]["outputs"]:
            dependencies["actuators"][cname] = []
            source_name = f"actuators/{cname}"
            descendants = nx.descendants(G, source_name)
            for target in descendants:
                node_name, target_cname = target.split("/")
                if node_name in ["actuators", "sensors"]:
                    continue
                dependencies["actuators"][cname].append(node_name)
            # Also add dependencies of all targets
            for target in descendants:
                rev_descendants = nx.descendants(G_rev, target)
                for source in descendants:
                    node_name, source_cname = source.split("/")
                    if node_name in ["actuators", "sensors"]:
                        continue
                    dependencies["actuators"][cname].append(node_name)
            dependencies["actuators"][cname] = list(set(dependencies["actuators"].pop(cname)))
        return dependencies

    def register(self):
        """Set the addresses in all incoming components.
        Validate the graph.
        Create params that can be uploaded to the ROS param server.
        """

        # Check if valid graph.
        assert self.is_valid(plot=False), "Graph not valid."

        # Find dependencies
        dependencies = self._node_depenencies(self._state)

        # Add addresses based on connections
        state = deepcopy(self._state)
        actuators = dict()
        sensors = dict()
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            address = EngineGraph._get_address(source, target)
            if source_name == "actuators":
                dependency = [f"$(ns obj_name)/{d}" for d in dependencies["actuators"][source_cname]]
                actuators[source_cname] = {
                    "name": f"$(ns obj_name)/{target_name}",
                    "component": target_comp,
                    "cname": target_cname,
                    "dependency": dependency,
                }
                continue  # we continue here, because the address for actuators is determined by an output from the agnostic graph.
            if target_name == "sensors":
                dependency = [f"$(ns obj_name)/{d}" for d in dependencies["sensors"][target_cname]]
                sensors[target_cname] = {
                    "name": f"$(ns obj_name)/{source_name}",
                    "component": source_comp,
                    "cname": source_cname,
                    "dependency": dependency,
                }
                continue  # we continue here, because the address for actuators is determined by an output from the agnostic graph.
            state["nodes"][target_name]["params"][target_comp][target_cname]["address"] = address

        # Initialize param objects
        nodes = dict()
        from eagerx.core.specs import EngineNodeSpec

        for name, entry in state["nodes"].items():
            params = entry["params"]
            if "node_type" in params:
                if name == "actuators":
                    pass
                elif name == "sensors":
                    pass
                else:
                    # Put node name into object namespace
                    spec = EngineNodeSpec(params)
                    name = f'$(ns obj_name)/{spec.get_parameter("name")}'
                    spec.set_parameter("name", name)
                    params = spec.params

                    # Substitute placeholder args of simnode
                    context = {"ns": {"node_name": name}, "default": params["default"]}
                    substitute_args(params, context, only=["default", "ns"])
                    nodes[name] = params

        assert actuators, "No actuators node defined in the graph."
        assert sensors, "No sensors node defined in the graph."
        return nodes, actuators, sensors

    def save(self, path: str):
        with open(path, "w") as outfile:
            yaml.dump(self._state, outfile, default_flow_style=False)
        pass

    def load(self, path: str):
        with open(path, "r") as stream:
            try:
                self._state = yaml.safe_load(stream)
                # self._state = yaml.load(path)
            except yaml.YAMLError as exc:
                print(exc)

    def gui(self):
        raise NotImplementedError("Gui is not yet supported for engine graphs.")

    @staticmethod
    def _get_address(source: Tuple[str, str, str], target: Tuple[str, str, str]):
        """Determine the address."""
        source_name, source_comp, source_cname = source
        target_name, target_comp, target_cname = target
        if source_name == "actuators":
            assert (
                not target_name == "sensors"
            ), f'A direct connection between a sensor "{target_cname}" and actuator "{source_cname}" cannot exist.'
            address = f"$(ns obj_name)/{source_name}/{source_cname}"
        elif target_name == "sensors":
            assert (
                not source_name == "actuators"
            ), f'A direct connection between a sensor "{target_cname}" and actuator "{source_cname}" cannot exist.'
            address = f"$(ns obj_name)/{target_name}/{target_cname}"
        else:
            address = f"$(ns obj_name)/{source_name}/{source_comp}/{source_cname}"
        return address

    @staticmethod
    def _exist(
        state: Dict,
        name: str,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        parameter: Optional[str] = None,
        check_default: Optional[bool] = False,
    ):
        """
        Check if provided entry exists.
        """
        # Check that node/object exists
        assert name in state["nodes"], f'There is no node or object registered in this graph with name "{name}".'

        # See if we must check both default and current params.
        if check_default:
            check_params = (
                state["nodes"][name]["params"],
                state["nodes"][name]["default"],
            )
        else:
            check_params = (state["nodes"][name]["params"],)

        # Check params
        for params in check_params:
            default = params["default"]

            # Check that components and specific entry (cname) exists
            assert component is None or component in params, f'Component "{component}" not present in "{name}".'
            if component is None:
                assert cname is None, f'Cannot check if "{name}" exists, because no component was specified.'
            assert cname is None or cname in params[component], f'"{cname}" not defined in "{name}" under {component}.'

            # check that parameter exists
            if parameter is not None:
                if (component is not None) and (cname is not None):  # component parameter
                    assert (
                        parameter in params[component][cname]
                    ), f'Cannot set parameter "{parameter}". Parameter does not exist in "{cname}" under {component}.'
                else:
                    assert (
                        parameter in default
                    ), f'Cannot set parameter "{parameter}". Parameter does not exist under "default" of {name}.'

    @staticmethod
    def _is_selected(state: Dict, name: str, component: str, cname: str):
        """
        Check if provided entry was selected in params.
        """
        EngineGraph._exist(state, name, component, cname)
        params = state["nodes"][name]["params"]
        component = "outputs" if component == "feedthroughs" else component
        assert cname in params["default"][component], '"%s" not selected in "%s" under "default" in %s. ' % (
            cname,
            name,
            component,
        )

    @staticmethod
    def _correct_signature(
        name: Optional[str] = None,
        component: Optional[str] = None,
        cname: Optional[str] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        # assert only actuator, only sensor, or only name, component, cname
        if (name is not None) and (component is not None) and (cname is not None):  # component parameter
            assert actuator is None, "If {name, component, cname} are specified, actuator argument cannot be specified."
            assert sensor is None, "If {name, component, cname} are specified, sensor argument cannot be specified."
        if name is not None:  # entity parameter
            assert actuator is None, "If {name, component, cname} are specified, actuator argument cannot be specified."
            assert sensor is None, "If {name, component, cname} are specified, sensor argument cannot be specified."
        if component is not None:  # entity parameter
            assert (
                name is not None
            ), f'Either both or None of component "{component}" and name "{name}" must be specified.'
            assert actuator is None, "If {name, component, cname} are specified, actuator argument cannot be specified."
            assert sensor is None, "If {name, component, cname} are specified, sensor argument cannot be specified."
        if cname is not None:  # entity parameter
            assert (
                name is not None
            ), f'Either both or None of component "{component}" and name "{name}" must be specified.'
            assert (
                component is not None
            ), f'If cname "{cname}" is specified, also component "{component}" and name "{name}" must be specified.'
            assert actuator is None, "If {name, component, cname} are specified, actuator argument cannot be specified."
            assert sensor is None, "If {name, component, cname} are specified, sensor argument cannot be specified."
        if actuator:
            assert sensor is None, "If actuator is specified, sensor must be None."
            assert (
                (name is None) and (component is None) and (cname is None)
            ), "If actuator is specified, arguments {name, component, cname} cannot be specified."
        if sensor:
            assert actuator is None, "If observation is specified, actuator must be None."
            assert (
                (name is None) and (component is None) and (cname is None)
            ), "If actuator is specified, arguments {name, component, cname} cannot be specified."

    def is_valid(self, plot=True):
        return self._is_valid(self._state, plot=plot)

    @staticmethod
    def _is_valid(state, plot=True):
        state = deepcopy(state)
        EngineGraph.check_msg_types_are_consistent(state)
        EngineGraph.check_inputs_have_address(state)
        EngineGraph.check_graph_is_acyclic(state, plot=plot)
        return True

    @staticmethod
    def check_msg_type(source, target, state):
        source_name, source_comp, source_cname = source
        source_params = state["nodes"][source_name]["params"]
        target_name, target_comp, target_cname = target
        target_params = state["nodes"][target_name]["params"]

        # Convert the source msg_type to target msg_type with converters:
        # msg_type_source --> output_converter --> msg_type_ROS --> input_converter --> msg_type_target
        msg_type_out = get_cls_from_string(source_params[source_comp][source_cname]["msg_type"])
        converter_out = source_params[source_comp][source_cname]["converter"]
        msg_type_ros = get_opposite_msg_cls(msg_type_out, converter_out)
        converter_in = target_params[target_comp][target_cname]["converter"]
        msg_type_in = get_opposite_msg_cls(msg_type_ros, converter_in)

        # Verify that this msg_type_in is the same as the msg_type specified in the target
        msg_type_in_yaml = get_cls_from_string(target_params[target_comp][target_cname]["msg_type"])

        msg_type_str = msg_type_error(
            source,
            target,
            msg_type_out,
            converter_out,
            msg_type_ros,
            converter_in,
            msg_type_in,
            msg_type_in_yaml,
        )
        assert msg_type_in == msg_type_in_yaml, msg_type_str

    @staticmethod
    def check_msg_types_are_consistent(state):
        for source, target in state["connects"]:
            EngineGraph.check_msg_type(source, target, state)
        return True

    @staticmethod
    def check_inputs_have_address(state):
        state = deepcopy(state)
        for source, target in state["connects"]:
            address = EngineGraph._get_address(source, target)
            target_name, target_comp, target_cname = target
            state["nodes"][target_name]["params"][target_comp][target_cname]["address"] = address

        for name, entry in state["nodes"].items():
            params = entry["params"]
            for component in params["default"]:
                if component not in [
                    "inputs",
                    "outputs",
                    "targets",
                    "feedthroughs",
                    "states",
                ]:
                    continue
                for cname in params["default"][component]:
                    assert (
                        cname in params[component]
                    ), f'"{cname}" was selected in {component} of "{name}", but has no implementation.'
                    if component not in ["inputs", "targets", "feedthroughs"]:
                        continue
                    if name in ["sensors", "actuators"]:
                        continue
                    assert (
                        params[component][cname]["address"] is not None
                    ), f'"{cname}" was selected in {component} of "{name}", but no address was specified. Either deselect it, or connect it.'
        return True

    @staticmethod
    def check_graph_is_acyclic(state, plot=True):
        # Generate graph
        G = EngineGraph._generate_graph(state)

        # Color nodes based on in/out going edges
        not_active = is_stale(G, exclude_skip=True)
        color_nodes(G)
        color_edges(G)

        # Check if graph is acyclic (excluding 'skip' edges)
        H, cycles = episode_graph(G)
        is_dag = nx.is_directed_acyclic_graph(H)

        # Plot graphs
        if plot:
            fig_env, ax_env = plt.subplots(nrows=1, ncols=1)
            ax_env.set_title("Engine-specific graph")
            _, _, _, pos = plot_graph(G, k=2, ax=ax_env)
            plt.show()

        # Assert if graph is a directed-acyclical graph (DAG)
        cycle_strs = ["Algebraic loops detected: "]
        for idx, connect in enumerate(cycles):
            connect.append(connect[0])
            s = " Loop %s: " % idx
            n = "\n" + "".join([" "] * len(s)) + "...-->"
            s = "\n\n" + s + "...-->"
            for idx in range(len(connect) - 1):
                tmp, target = connect[idx]
                source, tmp2 = connect[idx + 1]
                source_name, source_comp, source_cname = source
                target_name, target_comp, target_cname = target
                assert source_name == target_name, "Source and target not equal: %s, %s" % (source, target)
                connect_name = "%s/%s/%s][%s/%s/%s" % tuple(list(source) + list(target))
                node_name = ("Node: " + source_name).center(len(connect_name), " ")
                s += "[%s]-->" % connect_name
                n += "[%s]-->" % node_name
            s += "..."
            n += "..."
            cycle_strs.append(s)
            cycle_strs.append(n)
            connect.pop(-1)
        assert is_dag, "".join(cycle_strs)

        not_active_excl_sensors = [n for n in not_active if not n.split("/")[0] == "sensors"]
        assert len(not_active_excl_sensors) == 0, (
            'Stale episode graph detected. Nodes "%s" will be stale, while they must be active (i.e. connected) in order for the graph to resolve (i.e. not deadlock).'
            % not_active
        )
        return True

    @staticmethod
    def _generate_graph(state):
        # Add nodes
        G = nx.MultiDiGraph()
        # label_mapping = {'env/observations/set': 'observations', 'env/render/done': 'render'}
        for node, params in state["nodes"].items():
            default = params["params"]["default"]
            if "outputs" in default:
                has_tick = True if "tick" in default["inputs"] else False
                for cname in default["outputs"]:
                    name = "%s/%s" % (node, cname)
                    remain_active = True if node == "actuators" else False
                    always_active = True if node == "actuators" else False
                    G.add_node(
                        name,
                        remain_active=remain_active,
                        always_active=always_active,
                        is_stale=False,
                        has_tick=has_tick,
                    )
            if node == "sensors":
                for cname in default["inputs"]:
                    name = "%s/%s" % (node, cname)
                    G.add_node(
                        name,
                        remain_active=True,
                        always_active=False,
                        is_stale=False,
                        has_tick=False,
                    )

        # Add edges
        target_comps = ["inputs"]
        source_comps = ["outputs"]
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            if source_comp in source_comps and target_comp in target_comps:
                source_edge = "%s/%s" % (source_name, source_cname)
                # Determine target node name
                target_edges = []
                target_default = state["nodes"][target_name]["params"]["default"]
                for cname in target_default["outputs"]:
                    target_edge = "%s/%s" % (target_name, cname)
                    target_edges.append(target_edge)
                if target_name == "sensors":
                    target_edge = "%s/%s" % (target_name, target_cname)
                    target_edges.append(target_edge)

                # Determine stale nodes in real_reset routine via feedthrough edges
                if target_comp == "feedthroughs":
                    feedthrough = True
                else:
                    feedthrough = False

                # Determine edges that do not break DAG property (i.e. edges that are skipped)
                if source_name == "actuators":
                    skip = state["nodes"][source_name]["params"]["inputs"][source_cname]["skip"]
                else:
                    skip = state["nodes"][target_name]["params"][target_comp][target_cname]["skip"]

                # Determine color
                color = "green" if skip else "black"
                style = "dotted" if skip else "solid"

                # Add edge
                for target_edge in target_edges:
                    G.add_edge(
                        source_edge,
                        target_edge,
                        color=color,
                        feedthrough=feedthrough,
                        style=style,
                        alpha=1.0,
                        is_stale=False,
                        skip=skip,
                        source=source,
                        target=target,
                    )

        # Color nodes based on in/out going edges
        not_active = is_stale(G, exclude_skip=True)
        color_nodes(G)
        color_edges(G)
        return G
