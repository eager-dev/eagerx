import rospy
import yaml
from copy import deepcopy
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Union, Dict, Tuple, Optional, Any
from eagerx.utils.utils import (
    supported_types,
    get_opposite_msg_cls,
    get_cls_from_string,
    get_opposite_msg_cls_v2,
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
from eagerx.core.graph import merge
from eagerx.core.specs import NodeSpec, ConverterSpec, EntitySpec
from eagerx.core.view import GraphView

yaml.Dumper.ignore_aliases = lambda *args: True


class EngineGraph:
    def __init__(self, state: Dict):
        self._state = state

    def __str__(self):
        return yaml.dump(self._state)

    @classmethod
    def create(cls, actuators: Optional[List[Dict]] = None, sensors: Optional[List[Dict]] = None):
        nodes = []

        from eagerx.core.entities import EngineNode

        # Create actuator node
        spec = EngineNode.pre_make(None, None)
        spec.config.name = "actuators"
        nodes.append(spec)
        for cname, params in actuators.items():
            # Determine converted msg_type, based on user-defined input converter
            # If input_conv != entity_id but EngineNode connection also requires a converter --> raise error.
            conv_msg_type = get_opposite_msg_cls_v2(params.msg_type, params.converter)
            spec.add_output(
                cname,
                msg_type=conv_msg_type,
                converter=ConverterSpec(params.converter.to_dict()),  # Set input converter as output converter
                space_converter=ConverterSpec(params.space_converter.to_dict()),
            )
            spec.add_input(
                cname,
                msg_type=params.msg_type,
                skip=params.skip,
                converter=ConverterSpec(params.converter.to_dict()),
                space_converter=ConverterSpec(params.space_converter.to_dict()),
            )
            spec.config.outputs.append(cname)

        # Create sensor node
        spec = EngineNode.pre_make(None, None)
        spec.config.name = "sensors"
        nodes.append(spec)
        for cname, params in sensors.items():
            conv_msg_type = get_opposite_msg_cls_v2(params.msg_type, params.converter)
            spec.config.inputs.append(cname)
            spec.add_input(
                cname,
                msg_type=conv_msg_type,
                converter=ConverterSpec(params.converter.to_dict()),
                space_converter=ConverterSpec(params.space_converter.to_dict()),
            )

        # Create a state
        state = dict(nodes=dict(), connects=list(), backup=dict())
        graph = cls(state)
        graph.add(nodes)
        return graph

    def add(self, nodes: Union[NodeSpec, List[NodeSpec]]) -> None:
        """Add nodes to the graph.

        :param nodes: Nodes/objects to add.
        """
        if not isinstance(nodes, list):
            nodes = [nodes]

        for node in nodes:
            name = node.config.name
            assert name not in self._state["nodes"], (
                'There is already a node or object registered in this graph with name "%s".' % name
            )
            assert not node.has_graph, f"NodeSpec '{name}' is already added to a graph."

            # Add node to state
            self._state["nodes"][name] = node.params
            self._state["backup"][name] = node.params

            # Add graph reference to spec
            node.set_graph(self)

    def remove(self, names: Union[Union[str, EntitySpec], List[Union[str, EntitySpec]]]) -> None:
        """Removes a node from the graph.

        - First, all associated connections are disconnected.

        - Then, removes the nodes/objects.

        :param names: Either the name or spec of the node/object that is to be removed.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            if isinstance(name, EntitySpec):
                name = name.params["config"]["name"]
                assert name in self._state["nodes"], f" No entity with name '{name}' in graph."
            for source, target in deepcopy(self._state["connects"]):
                if name in [source[0], target[0]]:
                    if source[0] == "actuators":
                        actuator = source[2]
                        source = None
                    else:
                        actuator = None
                        source = self.get_view(source[0], source[1:])
                    if target[0] == "sensors":
                        sensor = target[2]
                        target = None
                    else:
                        sensor = None
                        target = self.get_view(target[0], target[1:])
                    self.disconnect(source, target, actuator, sensor)
            self._state["nodes"].pop(name)

    def add_component(self, entry: GraphView) -> None:
        """Selects an available component entry (e.g. input, output, etc...) that was not already selected.

        :param entry: Selects the entry, so that it can be connected.
        """
        name, component, cname = entry()
        # Add cname to selection list if it is not already selected
        params = self._state["nodes"][name]
        assert cname not in params["config"][component], f'"{cname}" already selected in "{name}" under {component}.'
        params["config"][component].append(cname)

    def remove_component(self, entry: GraphView) -> None:
        """Deselects a component entry (e.g. input, output, etc...) that was selected.

                - First, all associated connections are disconnected.

                - Then, deselects the component entry.

        :param entry: Deselects the entry.
        """
        self._is_selected(self._state, entry)

        # Disconnect component entry
        self._disconnect_component(entry)

        # Remove cname from selection list
        name, component, cname = entry()
        params = self._state["nodes"][name]
        params["config"][component].remove(cname)

    def connect(
        self,
        source: Optional[GraphView] = None,
        target: Optional[GraphView] = None,
        actuator: str = None,
        sensor: str = None,
        address: str = None,
        converter: Optional[Dict] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
        external_rate: Optional[float] = None,
    ) -> None:
        """Connect an actuator/source to a sensor/target/external topic.

        :param source: Compatible source type is :attr:`~eagerx.core.specs.NodeSpec.outputs`.
        :param target: Compatible target type is :attr:`~eagerx.core.specs.NodeSpec.inputs`.
        :param actuator: String name of the actuator.
        :param sensor: String name of the sensor.
        :param address: A string address of an external ROS topic.

                        .. note:: If an external address is provided,
                            you must also specify the *external_rate* with which messages are published on that topic.
        :param converter: An input converter that converts the received input message before passing it
                          to the node's :func:`~eagerx.core.entities.EngineNode.callback`.

                          .. note:: Output converters can only be set by manipulating the :class:`~eagerx.core.specs.NodeSpec`.
        :param window: A non-negative number that specifies the number of messages to pass to the
                       node's :func:`~eagerx.core.entities.EngineNode.callback`.

                       - *window* = 1: Only the last received input message.

                       - *window* = *x* > 1: The trailing last *x* received input messages.

                       - *window* = 0: All input messages received since the last call to the
                         node's :func:`~eagerx.core.entities.EngineNode.callback`.

                       .. note:: With *window* = 0, the number of input messages may vary and can even be zero.
        :param delay: A non-negative simulated delay (seconds). This delay is ignored if
                      :attr:`~eagerx.core.entities.Bridge.simulate_delays` = True
                      in the bridge's :func:`~eagerx.core.entities.Bridge.spec`.
        :param skip: Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.EngineNode.callback`.
                     May be necessary to ensure that the connected graph is directed and acyclic.
        :param external_rate: The rate (Hz) with which messages are published to the topic specified by
                              *address*.

                              .. warning:: Only add external inputs if you are sure that they are synchronized with respect
                                           to the provided rate and their respective inputs.
                                           Asynchronous external inputs can easily lead to deadlocks if running in synchronized mode
                                           (i.e. :attr:`~eagerx.core.entities.Bridge.is_reactive` = True).
        """
        flag = not address or (source is None and actuator is None and sensor is None)
        assert flag, f'You cannot provide an external address "{address}" together with a sensor, actuator, or source.'
        flag = not source or not actuator
        assert flag, (
            f'You cannot specify an actuator if you wish to connect actuator "{actuator}", '
            "as the actuator will act as the source."
        )
        flag = not target or not sensor
        assert flag, (
            f'You cannot specify a target if you wish to connect sensor "{sensor}", ' "as the sensor will act as the target."
        )
        assert not (actuator and sensor), "You cannot connect an actuator directly to a sensor."
        if address:
            curr_address = target.address
            assert curr_address is None, (
                f'Cannot connect target "{target}" to external address "{address}", '
                f'because it is already connected to an external address "{curr_address}". '
                "Disconnect target first."
            )
            self.set({"address": address}, target)
            assert external_rate is not None, (
                f'When providing an external address "{address}", ' "an external rate must also be provided."
            )
            assert external_rate > 0, f'Invalid external rate "{external_rate}". External rate must be > 0.'
            self.set({"external_rate": external_rate}, target)
            return
        else:
            assert external_rate is None, "An external rate may only be provided in combination with an address."

        if isinstance(converter, ConverterSpec):
            converter = converter.params
        if isinstance(converter, GraphView):
            converter = converter.to_dict()

        if actuator:  # source = actuator
            source = self.get_view("actuators", ["outputs", actuator])
            from eagerx.core.converters import Identity

            id = Identity().get_yaml_definition()
            if converter or target.converter.to_dict() != id:
                msg = (
                    f'Cannot specify an input converter for actuator "{actuator}", '
                    "because one has already been specified in the agnostic graph definition. "
                    "You can only have one input converter."
                )
                assert source.converter.to_dict() == id, msg
            assert window is None, (
                f'Cannot specify a window when connecting actuator "{actuator}". '
                f"You can only do that in the agnostic object definition."
            )
            assert delay is None, (
                f'Cannot specify a delay when connecting actuator "{actuator}". '
                f"You can only do that in the agnostic object definition."
            )
            assert skip is None, (
                f'Cannot specify a skip when connecting actuator "{actuator}". '
                f"You can only do that in the agnostic object definition."
            )
        elif sensor:  # target = sensor
            target = self.get_view("sensors", ["inputs", sensor])
            # Check for output converter clash.
            # I.e. if both agnostic sensor & enginenode output have output converter.
            from eagerx.core.converters import Identity

            id = Identity().get_yaml_definition()
            if source.converter.to_dict() != id:
                src_name, _, src_cname = source()
                msg = (
                    "Output converter clash! "
                    f"Output '{src_cname}' of EngineNode '{src_name}' you attempt to connect to sensor '{sensor}'"
                    f" both have an output converter defined, but only one output converter can be used."
                )
                assert target.converter.to_dict() == id, msg
            assert converter is None, (
                f'Cannot specify an input converter when connecting sensor "{sensor}". '
                "For sensors, you can only do that in the agnostic definition. "
            )
            assert window is None, (
                f'Cannot specify a window when connecting sensor "{sensor}".'
                " You can only do that in the agnostic object definition."
            )
            assert delay is None, (
                f'Cannot specify a delay when connecting sensor "{sensor}".'
                " You can only do that in the agnostic object definition."
            )
            assert skip is None, (
                f'Cannot specify a skip when connecting sensor "{sensor}".'
                " You can only do that in the agnostic object definition."
            )
        self._connect(source, target, converter, window, delay, skip)

    def _connect(
        self,
        source: Optional[GraphView] = None,
        target: Optional[GraphView] = None,
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
        # Perform checks on source
        self._is_selected(self._state, source)

        # Perform checks on target
        self._is_selected(self._state, target)

        # Make sure that target is not already connected.
        curr_address = target.address
        assert curr_address is None, (
            f'Cannot connect target "{target}" to source "{source}", '
            f'because it is already connected to an external address "{curr_address}". '
            "Disconnect target first."
        )
        target_name, target_comp, target_cname = target()
        for s, t in self._state["connects"]:
            t_name, t_comp, t_cname = t
            flag = not (target_name == t_name and target_comp == t_comp and target_cname == t_cname)
            assert flag, f'Target "{target}" is already connected to source "{s}"'

        # Add properties to target params
        if converter is not None:
            self._set_converter(target, converter)
        if window is not None:
            self.set({"window": window}, target)
        if delay is not None:
            self.set({"delay": delay}, target)
        if skip is not None:
            self.set({"skip": skip}, target)

        # Add connection
        EngineGraph.check_msg_type(source, target, self._state)
        connect = [list(source()), list(target())]
        self._state["connects"].append(connect)

    def disconnect(
        self,
        source: Optional[GraphView] = None,
        target: Optional[GraphView] = None,
        actuator: str = None,
        sensor: str = None,
    ) -> None:
        """Disconnect an actuator/source from a sensor/target/external topic.

        :param source: Compatible source type is :attr:`~eagerx.core.specs.NodeSpec.outputs`.
        :param target: Compatible target type is :attr:`~eagerx.core.specs.NodeSpec.inputs`.
        :param actuator: String name of the actuator.
        :param sensor: String name of the sensor.
        """
        assert not source or not actuator, (
            "You cannot specify a source if you wish to disconnect actuator ",
            f'"{actuator}", as the actuator will act as the source.',
        )
        assert not target or not sensor, (
            "You cannot specify a target if you wish to disconnect sensor "
            f'"{sensor}", as the sensor will act as the target.'
        )
        assert not (sensor and actuator), "You cannot disconnect an actuator from a sensor, as such a connection cannot exist."

        # Create source & target entries
        if actuator:
            source = self.get_view("actuators", ["outputs", actuator])
        if sensor:
            target = self.get_view("sensors", ["inputs", sensor])

        self._disconnect(source, target)

    def _disconnect(self, source: GraphView, target: GraphView):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        # Check if connection exists
        self._is_selected(self._state, target)

        # Check if connection exists
        if source is None:
            flag = target.address is not None
            assert flag, (
                f"Cannot disconnect. No source was provided, and the target={target} "
                "also does not have an address specified."
            )
            self.set(None, target, parameter="address")
            self.set(None, target, parameter="external_rate")
        else:
            self._is_selected(self._state, source)
            connect_exists = False
            idx_connect = None
            for idx, c in enumerate(self._state["connects"]):
                if list(source()) == c[0] and list(target()) == c[1]:
                    connect_exists = True
                    idx_connect = idx
                    break
            assert connect_exists, (
                f"The connection with source={source()} and target={target()} cannot be removed," " because it does not exist."
            )

            # Pop the connection from the state
            self._state["connects"].pop(idx_connect)

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/skip/converter.
        target_name, target_comp, target_cname = target()
        target_params = self._state["nodes"][target_name]
        target_params[target_comp][target_cname] = self._state["backup"][target_name][target_comp][target_cname]

    def _disconnect_component(self, entry: GraphView):
        """
        Disconnects all associated connects from self._state.
        """
        was_connected = False
        name, component, cname = entry()
        for source, target in deepcopy(self._state["connects"]):
            source = self.get_view(source[0], source[1:])
            target = self.get_view(target[0], target[1:])
            self._is_selected(self._state, source)
            self._is_selected(self._state, target)
            source_name, source_comp, source_cname = source()
            target_name, target_comp, target_cname = target()
            if name == source_name and component == source_comp and cname == source_cname:
                self.disconnect(source, target)
                was_connected = True
            elif name == target_name and component == target_comp and cname == target_cname:
                self.disconnect(source, target)
                was_connected = True
        return was_connected

    def set(self, mapping: Any, entry: Optional[GraphView], parameter: Optional[str] = None) -> None:
        """Sets the parameters of a node.

        :param mapping: Either a mapping with *key* = *parameter*,
                        or a single value that corresponds to the optional *parameter* argument.
        :param entry: The entry whose parameters are mutated.
        :param parameter: If only a single value needs to be set. See documentation for *mapping*.
        """
        # """
        # Sets parameters in self._state, based on the node/object name. If a component and cname are specified, the
        # parameter will be set there. Else, the parameter is set under the "config" key.
        # For objects, parameters are set under their agnostic definitions of the components (so not bridge specific).
        # If a converter is added, we check if the msg_type changes with the new converter. If so, the component is
        # disconnected. See _set_converter for more info.
        # """
        assert not entry()[0] == "actuators", (
            "Cannot change the actuator parameters here, "
            "in a bridge specific implementation. That is only possible in the "
            "object's agnostic definition."
        )
        assert not entry()[0] == "sensors", (
            "Cannot change the sensor parameters here, "
            "in a bridge specific implementation. That is only possible in the "
            "object's agnostic definition."
        )

        if parameter is None:
            if isinstance(mapping, GraphView):
                mapping = mapping.to_dict()
            assert isinstance(mapping, dict), "Can only set mappings of type dict. Else, also set 'parameter=<param_name>'."
        else:
            mapping = {parameter: mapping}
        for parameter, value in mapping.items():
            if parameter:
                getattr(entry, parameter)  # Check if parameter exists
            if parameter == "converter":
                msg = (
                    "Skipping converter. Cannot change the converter with this method. "
                    "Add output converters before connecting, and input converters when making a connection."
                )
                rospy.logwarn_once(msg)
            else:
                t = entry()
                name = t[0]
                if t[1] == "config":
                    assert parameter not in [
                        "sensors",
                        "actuators",
                        "targets",
                        "states",
                        "inputs",
                        "outputs",
                    ], "You cannot modify component parameters with this function. Use _add/remove_component(..) instead."
                    assert parameter not in ["msg_type"], f"You cannot rename '{name}'."
                    assert parameter not in ["msg_type"], f"You cannot modify msg_type '{value}'."
                    p = self._state["nodes"][name]["config"]
                else:
                    name, component, cname = entry()
                    p = self._state["nodes"][name][component][cname]
                self._set(p, {parameter: value})

    def _set_converter(self, entry: GraphView, converter: Dict):
        """
        Replaces the converter specified for a node's/object's I/O.
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
        """
        if isinstance(converter, ConverterSpec):
            converter = converter.params
        elif isinstance(converter, GraphView):
            converter = converter.to_dict()

        _ = entry.converter  # Check if parameter exists

        # Check if converted msg_type of old converter is equal to the msg_type of newly specified converter
        msg_type = get_cls_from_string(entry.msg_type)
        converter_old = entry.converter.to_dict()
        msg_type_ros_old = get_opposite_msg_cls(msg_type, converter_old)
        msg_type_ros_new = get_opposite_msg_cls(msg_type, converter)
        if not msg_type_ros_new == msg_type_ros_old:
            was_disconnected = self._disconnect_component(entry)
        else:
            was_disconnected = False

        # If disconnected action/observation, we cannot add converter so raise error.
        name, component, cname = entry()
        params = self._state["nodes"][name]
        flag = not (was_disconnected and name in ["actuators", "sensors"])
        assert flag, (
            f'Cannot change the converter of actuator/sensor "{cname}", '
            f'as it changes the msg_type from "{msg_type_ros_old}" to "{msg_type_ros_new}"'
        )

        # Replace converter
        params[component][cname]["converter"] = converter

    def get(
        self,
        entry: Optional[Union[GraphView, EntitySpec]] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
        parameter: Optional[str] = None,
    ) -> Any:
        """Fetches the parameters of a node/actuator/sensor.

        :param entry: The entry whose parameters are fetched.
        :param actuator: Actuator name whose parameters are fetched.
        :param sensor: Sensor name whose parameters are fetched.
        :param parameter: If only a single parameter needs to be fetched.
        :return: Parameters
        """
        if isinstance(entry, EntitySpec):
            name = entry.params["config"]["name"]
            assert name in self._state["nodes"], f" No entity with name '{name}' in graph."
            return self._state["nodes"][name]
        self._correct_signature(entry, actuator, sensor)
        if actuator:
            entry = self.get_view("actuators", ["outputs", actuator])
        if sensor:
            entry = self.get_view("sensors", ["inputs", sensor])
        if parameter:
            return getattr(entry, parameter)
        else:
            return entry

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
        for cname in state["nodes"]["sensors"]["inputs"]:
            dependencies["sensors"][cname] = []
            target_name = f"sensors/{cname}"
            descendants = nx.descendants(G_rev, target_name)
            for source in descendants:
                node_name, source_cname = source.split("/")
                if node_name in ["actuators", "sensors"]:
                    continue
                dependencies["sensors"][cname].append(node_name)

        # Determine actuator dependency
        for cname in state["nodes"]["actuators"]["outputs"]:
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
                nx.descendants(G_rev, target)
                for source in descendants:
                    node_name, source_cname = source.split("/")
                    if node_name in ["actuators", "sensors"]:
                        continue
                    dependencies["actuators"][cname].append(node_name)
            dependencies["actuators"][cname] = list(set(dependencies["actuators"].pop(cname)))
        return dependencies

    def register(self):
        # """Set the addresses in all incoming components.
        # Validate the graph.
        # Create params that can be uploaded to the ROS param server.
        # """

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
                if source_cname not in actuators:
                    actuators[source_cname] = []
                entry = {
                    "name": f"$(ns obj_name)/{target_name}",
                    "component": target_comp,
                    "cname": target_cname,
                    "dependency": dependency,
                }
                # actuators[source_cname] = entry
                actuators[source_cname].append(entry)
                continue  # we continue here, because the address for actuators is determined by an output from the agnostic graph.
            if target_name == "sensors":
                dependency = [f"$(ns obj_name)/{d}" for d in dependencies["sensors"][target_cname]]
                if target_cname not in sensors:
                    sensors[target_cname] = []
                entry = {
                    "name": f"$(ns obj_name)/{source_name}",
                    "component": source_comp,
                    "cname": source_cname,
                    "dependency": dependency,
                }
                # sensors[target_cname] = entry
                sensors[target_cname].append(entry)
                continue  # we continue here, because the address for actuators is determined by an output from the agnostic graph.
            state["nodes"][target_name][target_comp][target_cname]["address"] = address

        # Initialize param objects
        nodes = dict()
        from eagerx.core.specs import NodeSpec

        for name, params in state["nodes"].items():
            if "node_type" in params:
                if name == "actuators":
                    pass
                elif name == "sensors":
                    pass
                else:
                    # Put node name into object namespace
                    spec = NodeSpec(params)
                    name = f"$(ns obj_name)/{spec.config.name}"
                    spec.config.name = name
                    params = spec.params

                    # Substitute placeholder args of simnode
                    context = {"ns": {"node_name": name}, "config": params["config"]}
                    substitute_args(params, context, only=["config", "ns"])
                    nodes[name] = params

        # assert len(actuators) > 0, "No actuators node defined in the graph."
        # assert len(sensors) > 0, "No sensors node defined in the graph."
        return nodes, actuators, sensors

    def gui(self) -> None:
        """Opens a graphical user interface of the engine graph.

        .. note:: Currently, a gui is not yet support for engine graphs.
                  This feature will be added in the near future.
        """
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
    def _is_selected(state: Dict, entry: GraphView):
        """
        Check if provided entry was selected in params.
        """
        name, component, cname = entry()
        params = state["nodes"][name]
        component = "outputs" if component == "feedthroughs" else component
        assert cname in params["config"][component], f'"{cname}" not selected in "{name}" under "config" in {component}.'

    @staticmethod
    def _correct_signature(
        entry: Optional[GraphView] = None,
        actuator: Optional[str] = None,
        sensor: Optional[str] = None,
    ):
        # assert only action, only observation, or only entry
        if entry:  # component parameter
            assert actuator is None, "If 'entry' is specified, actuator argument cannot be specified."
            assert sensor is None, "If 'entry' is specified, sensor argument cannot be specified."
        if actuator:
            assert sensor is None, "If actuator is specified, sensor must be None."
            assert entry is None, "If actuator is specified, the 'entry' argument cannot be specified."
        if sensor:
            assert actuator is None, "If sensor is specified, action must be None."
            assert entry is None, "If actuator is specified, the 'entry' argument cannot be specified."

    def is_valid(self, plot=True) -> bool:
        """Checks the validity of the graph.

        - Checks if all selected
          :attr:`~eagerx.core.specs.NodeSpec.inputs` are connected.

        - Checks if the graph is directed and acyclic.

        :param plot: Flag to plot the graph. Can be helpful to identify cycles in the graph that break the required acyclic property.
        :returns: flag that specifies the validity of the graph.
        """
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
        # Convert the source msg_type to target msg_type with converters:
        # msg_type_source --> output_converter --> msg_type_ROS --> input_converter --> msg_type_target
        msg_type_out = get_cls_from_string(source.msg_type)
        converter_out = source.converter.to_dict()
        msg_type_ros = get_opposite_msg_cls(msg_type_out, converter_out)
        converter_in = target.converter.to_dict()
        msg_type_in = get_opposite_msg_cls(msg_type_ros, converter_in)

        # Verify that this msg_type_in is the same as the msg_type specified in the target
        msg_type_in_registered = get_cls_from_string(target.msg_type)

        msg_type_str = msg_type_error(
            source(),
            target(),
            msg_type_out,
            converter_out,
            msg_type_ros,
            converter_in,
            msg_type_in,
            msg_type_in_registered,
        )
        assert msg_type_in == msg_type_in_registered, msg_type_str

    @staticmethod
    def check_msg_types_are_consistent(state):
        for source, target in state["connects"]:
            if target[0] == "sensors":
                target_view = EngineGraph._get_view(state, target[0], target[1:])
                from eagerx.core.converters import Identity

                id = Identity().get_yaml_definition()
                # If agnostic definition has a non-identity output_converter,
                # enginenode output cannot be inter-connected with other enginenodes,
                # as they expect an unconverted output.
                if target_view.converter.to_dict() != id:
                    flag = [t for s, t in state["connects"] if s == source]
                    msg = (
                        f"Agnostic definition for sensor '{target[2]}' has a non-identity output converter, "
                        f"However, output '{source[2]}' of EngineNode '{source[0]}' is interconnected to other "
                        f"EngineNodes that expect the original (unconverted) output message: {flag}. "
                        f"Non-identity output converters can only be added to a sensor if the corresponding "
                        f"EngineNode output has only a single connection (with the sensor)."
                    )
                    assert len(flag) == 1, msg
        for source, target in state["connects"]:
            source = EngineGraph._get_view(state, source[0], source[1:])
            target = EngineGraph._get_view(state, target[0], target[1:])
            EngineGraph.check_msg_type(source, target, state)
        return True

    @staticmethod
    def check_inputs_have_address(state):
        state = deepcopy(state)
        for source, target in state["connects"]:
            address = EngineGraph._get_address(source, target)
            target_name, target_comp, target_cname = target
            state["nodes"][target_name][target_comp][target_cname]["address"] = address

        for name, params in state["nodes"].items():
            for component in params["config"]:
                if component not in [
                    "inputs",
                    "outputs",
                    "targets",
                    "feedthroughs",
                    "states",
                ]:
                    continue
                for cname in params["config"][component]:
                    flag = cname in params[component]
                    assert flag, f'"{cname}" was selected in {component} of "{name}", but has no implementation.'
                    if component not in ["inputs", "targets", "feedthroughs"]:
                        continue
                    if name in ["sensors", "actuators"]:
                        continue
                    flag = params[component][cname]["address"] is not None
                    assert flag, (
                        f'"{cname}" was selected in {component} of "{name}", but no address was specified. '
                        f"Either deselect it, or connect it."
                    )
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
            default = params["config"]
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
                target_default = state["nodes"][target_name]["config"]
                for cname in target_default["outputs"]:
                    target_edge = "%s/%s" % (target_name, cname)
                    target_edges.append(target_edge)
                if target_name == "sensors":
                    target_edge = "%s/%s" % (target_name, target_cname)
                    target_edges.append(target_edge)

                # Determine edges that do not break DAG property (i.e. edges that are skipped)
                if source_name == "actuators":
                    skip = state["nodes"][source_name]["inputs"][source_cname]["skip"]
                else:
                    skip = state["nodes"][target_name][target_comp][target_cname]["skip"]

                # Determine color
                color = "green" if skip else "black"
                style = "dotted" if skip else "solid"

                # Add edge
                for target_edge in target_edges:
                    G.add_edge(
                        source_edge,
                        target_edge,
                        color=color,
                        feedthrough=False,
                        style=style,
                        alpha=1.0,
                        is_stale=False,
                        skip=skip,
                        source=source,
                        target=target,
                    )

        # Color nodes based on in/out going edges
        # not_active = is_stale(G, exclude_skip=True)
        color_nodes(G)
        color_edges(G)
        return G

    @staticmethod
    def _get_view(state, name: str, depth: Optional[List[str]] = None):
        depth = depth if depth else []
        depth = [name] + depth
        return GraphView(EngineGraph(state), depth=depth, name=name)

    def get_view(self, name: str, depth: Optional[List[str]] = None):
        depth = depth if depth else []
        depth = [name] + depth
        return GraphView(self, depth=depth, name=name)

    @staticmethod
    @supported_types(str, int, list, float, bool, dict, EntitySpec, GraphView, None)
    def _set(state, mapping):
        merge(state, mapping)
