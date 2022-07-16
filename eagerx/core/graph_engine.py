import yaml
from copy import deepcopy
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Union, Dict, Tuple, Optional, Any
from eagerx.utils.utils import (
    is_compatible,
    supported_types,
)
from eagerx.utils.utils_sub import substitute_args
from eagerx.utils.network_utils import (
    episode_graph,
    plot_graph,
    color_nodes,
    color_edges,
    is_stale,
)
from eagerx.core.graph import merge
from eagerx.core.specs import NodeSpec, ProcessorSpec, EntitySpec
from eagerx.core.view import SpecView
import eagerx.core.log as log

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
        spec.config.color = "yellow"
        nodes.append(spec)
        for cname, params in actuators.items():
            # todo: Determine dtype, based on user-defined processor for target and source
            spec.add_output(
                cname,
                space=params.space.to_dict() if params.space is not None else None,
                processor=ProcessorSpec(params.processor.to_dict()) if params.processor is not None else None,
            )
            spec.add_input(
                cname,
                space=params.space.to_dict() if params.space is not None else None,
                skip=params.skip,
                window=params.window,
                processor=ProcessorSpec(params.processor.to_dict()) if params.processor is not None else None,
            )
            spec.config.outputs.append(cname)

        # Create sensor node
        spec = EngineNode.pre_make(None, None)
        spec.config.name = "sensors"
        spec.config.color = "yellow"
        nodes.append(spec)
        for cname, params in sensors.items():
            spec.config.inputs.append(cname)
            spec.add_input(
                cname,
                space=params.space.to_dict() if params.space is not None else None,
                processor=ProcessorSpec(params.processor.to_dict()) if params.processor is not None else None,
            )

        # Create a state
        state = dict(nodes=dict(), connects=list(), backup=dict(), gui_state=dict())
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
            # Check spec
            self._check_spec(node)

            name = node.config.name
            assert name not in self._state["nodes"], (
                'There is already a node or object registered in this graph with name "%s".' % name
            )

            # Add node to state
            self._state["nodes"][name] = node._params
            self._state["backup"][name] = node.params

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

    def add_component(self, entry: SpecView) -> None:
        """Selects an available component entry (e.g. input, output, etc...) that was not already selected.

        :param entry: Selects the entry, so that it can be connected.
        """
        name, component, cname = entry()
        # Add cname to selection list if it is not already selected
        params = self._state["nodes"][name]
        assert cname not in params["config"][component], f'"{cname}" already selected in "{name}" under {component}.'
        params["config"][component].append(cname)

    def remove_component(self, entry: SpecView) -> None:
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
        source: Optional[SpecView] = None,
        target: Optional[SpecView] = None,
        actuator: str = None,
        sensor: str = None,
        # processor: Optional[ProcessorSpec] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
    ) -> None:
        """Connect an actuator/source to a sensor/target.

        :param source: Compatible source type is :attr:`~eagerx.core.specs.NodeSpec.outputs`.
        :param target: Compatible target type is :attr:`~eagerx.core.specs.NodeSpec.inputs`.
        :param actuator: String name of the actuator.
        :param sensor: String name of the sensor.
        :param window: A non-negative number that specifies the number of messages to pass to the
                       node's :func:`~eagerx.core.entities.EngineNode.callback`.

                       - *window* = 1: Only the last received input message.

                       - *window* = *x* > 1: The trailing last *x* received input messages.

                       - *window* = 0: All input messages received since the last call to the
                         node's :func:`~eagerx.core.entities.EngineNode.callback`.

                       .. note:: With *window* = 0, the number of input messages may vary and can even be zero.
        :param delay: A non-negative simulated delay (seconds). This delay is ignored if
                      :attr:`~eagerx.core.entities.Engine.simulate_delays` = True
                      in the engine's :func:`~eagerx.core.entities.Engine.spec`.
        :param skip: Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.EngineNode.callback`.
                     May be necessary to ensure that the connected graph is directed and acyclic.
        """
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

        if actuator:  # source = actuator
            source = self.get_view("actuators", ["outputs", actuator])
            if target.processor is not None:
                msg = (
                    f'Cannot specify a processor for actuator "{actuator}", '
                    "because there is already one specified in the agnostic graph definition. "
                    "You can only have one processor."
                )
                assert source.processor is None, msg
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
            # Check for processor clash.
            # I.e. if both agnostic sensor & enginenode output have a processor.
            if source.processor is not None:
                src_name, src_comp, src_cname = source()
                msg = (
                    "Processor clash detected! "
                    f"You attempt to connect EngineNode output '{src_name}.{src_comp}.{src_cname}' to sensor '{sensor}'. "
                    f"There is a processor defined for '{src_name}.{src_comp}.{src_cname}', but sensor '{sensor}' also has a "
                    f"a processor. Only one can be used."
                )
                assert target.processor is None, msg
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
        self._connect(source, target, window, delay, skip)

    def _connect(
        self,
        source: Optional[SpecView] = None,
        target: Optional[SpecView] = None,
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

        # Make sure that source and target are compatible
        self._is_compatible(self._state, source, target)

        # Make sure that target is not already connected.
        target_name, target_comp, target_cname = target()
        for s, t in self._state["connects"]:
            t_name, t_comp, t_cname = t
            flag = not (target_name == t_name and target_comp == t_comp and target_cname == t_cname)
            assert flag, f'Target "{target}" is already connected to source "{s}"'

        # Add properties to target params
        if window is not None:
            self.set({"window": window}, target)
        if delay is not None:
            self.set({"delay": delay}, target)
        if skip is not None:
            self.set({"skip": skip}, target)

        # Add connection
        connect = [list(source()), list(target())]
        self._state["connects"].append(connect)

    def disconnect(
        self,
        source: Optional[SpecView] = None,
        target: Optional[SpecView] = None,
        actuator: str = None,
        sensor: str = None,
    ) -> None:
        """Disconnect an actuator/source from a sensor/target.

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

    def _disconnect(self, source: SpecView, target: SpecView):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        # Check if connection exists
        self._is_selected(self._state, target)

        # Check if connection exists
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

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/skip.
        target_name, target_comp, target_cname = target()
        target_params = self._state["nodes"][target_name]
        target_params[target_comp][target_cname] = self._state["backup"][target_name][target_comp][target_cname]

    def _disconnect_component(self, entry: SpecView):
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

    def set(self, mapping: Any, entry: Optional[SpecView], parameter: Optional[str] = None) -> None:
        """Sets the parameters of a node.

        :param mapping: Either a mapping with *key* = *parameter*,
                        or a single value that corresponds to the optional *parameter* argument.
        :param entry: The entry whose parameters are mutated.
        :param parameter: If only a single value needs to be set. See documentation for *mapping*.
        """
        assert not entry()[0] == "actuators", (
            "Cannot change the actuator parameters here, "
            "in an engine specific implementation. That is only possible in the "
            "object's agnostic definition."
        )
        assert not entry()[0] == "sensors", (
            "Cannot change the sensor parameters here, "
            "in an engine specific implementation. That is only possible in the "
            "object's agnostic definition."
        )

        if parameter is None:
            if isinstance(mapping, SpecView):
                mapping = mapping.to_dict()
            assert isinstance(mapping, dict), "Can only set mappings of type dict. Else, also set 'parameter=<param_name>'."
        else:
            mapping = {parameter: mapping}
        for parameter, value in mapping.items():
            if parameter:
                getattr(entry, parameter)  # Check if parameter exists
            if parameter == "processor":
                msg = (
                    "Skipping processor. Cannot change the processor with this method. "
                    "Add output processors before connecting, and input processors when making a connection."
                )
                log.logwarn(msg)
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
                    assert parameter not in ["name"], f"You cannot rename '{name}'."
                    p = self._state["nodes"][name]["config"]
                else:
                    name, component, cname = entry()
                    p = self._state["nodes"][name][component][cname]
                self._set(p, {parameter: value})

    def _set_processor(self, entry: SpecView, processor: Dict):
        """Replaces the processor specified for a node's input."""
        if isinstance(processor, ProcessorSpec):
            processor = processor.params
        elif isinstance(processor, SpecView):
            processor = processor.to_dict()

        _ = entry.processor  # Check if parameter exists

        # Replace processor
        name, component, cname = entry()
        params = self._state["nodes"][name]
        params[component][cname]["processor"] = processor

    def get(
        self,
        entry: Optional[Union[SpecView, EntitySpec]] = None,
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

    def get_spec(self, name: str) -> NodeSpec:
        """Get Spec from the graph

        :param name: Name
        :return: The specification of the entity.
        """
        assert name in self._state["nodes"], f" No entity with name '{name}' in graph."
        params = self._state["nodes"][name]
        spec = NodeSpec(params)
        return spec

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
        """Set the addresses in all incoming components. Validate the graph.
        Create params that can be uploaded to the ROS param server.
        """

        # Check if valid graph.
        assert self.is_valid(plot=False), "Graph not valid."

        # Find dependencies
        dependencies = self._node_depenencies(self._state)

        # Copy state
        state = deepcopy(self._state)

        # Add addresses based on connections
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
                sensors[target_cname].append(entry)
                continue  # we continue here, because the address for actuators is determined by an output from the agnostic graph.
            state["nodes"][target_name][target_comp][target_cname]["address"] = address

            # Determine if dtypes match
            s = self.get_view(source[0], source[1:])
            t = self.get_view(target[0], target[1:])
            self._is_compatible(state, s, t)
            source_dtype = state["nodes"][source_name][source_comp][source_cname]["space"]["dtype"]
            state["nodes"][target_name][target_comp][target_cname]["dtype"] = source_dtype

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

        return nodes, actuators, sensors

    def gui(self) -> None:
        """Opens a graphical user interface of the engine graph."""
        try:
            from eagerx_gui import launch_engine_gui
        except ImportError as e:
            log.logwarn(
                f"{e}. You will likely have to install eagerx-gui. It can be installed by running: pip install eagerx-gui"
            )
            return
        self._state = launch_engine_gui(deepcopy(self._state))

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
    def _is_selected(state: Dict, entry: SpecView):
        """
        Check if provided entry was selected in params.
        """
        name, component, cname = entry()
        params = state["nodes"][name]
        component = "outputs" if component == "feedthroughs" else component
        assert cname in params["config"][component], f'"{cname}" not selected in "{name}" under "config" in {component}.'

    @staticmethod
    def _is_compatible(state: Dict, source: SpecView, target: SpecView):
        """Check if provided the provided entries are compatible."""
        # Valid source and target components
        targets = ["inputs"]
        sources = ["outputs"]

        # Provided entries
        target_name, target_component, target_cname = target()
        source_name, source_component, source_cname = source()
        base_msg = (
            f"'{target_name}.{target_component}.{target_cname}' cannot be connected with "
            f"'{source_name}.{source_component}.{source_cname}')."
        )

        # Check if target & source are validly chosen
        assert target_component in targets, f"{base_msg} '{target_component}' cannot be a target."
        assert source_component in sources, f"{base_msg} '{source_component}' cannot be a source."

        # Check if combinations are valid.
        if source_component in ["outputs", "sensors"]:
            valid = ["inputs", "feedthroughs", "actuators"]
            msg = f"{base_msg} '{source_component}' can only be connected to any of the components in '{valid}'."
            assert target_component in valid, msg
        else:  # source_component == "states":
            valid = ["targets"]
            msg = f"{base_msg} '{source_component}' can only be connected to any of the components in '{valid}'."
            assert target_component in valid, msg

            # Check that the state corresponds to an object (i.e. is an engine state)
            msg = (
                f"{base_msg} Only '{source_component}' of Objects can be connected to targets. '{source_name}' "
                "is not of type Object."
            )
            assert "node_type" not in state["nodes"][source_name], msg

        # Check if dtypes match
        source_params = state["nodes"][source_name][source_component][source_cname]
        target_params = state["nodes"][target_name][target_component][target_cname]
        assert (
            source_params["space"] is not None
        ), f"source={source_name}.{source_component}.{source_cname} does not have a space defined."
        assert (
            target_params["space"] is not None
        ), f"target={target_name}.{target_component}.{target_cname} does not have a space defined."
        source_dtype = source_params["space"]["dtype"]
        target_dtype = target_params["space"]["dtype"]
        try:
            is_compatible(source_dtype, target_dtype)
        except AssertionError as e:
            msg = (
                f"Incorrect connection of (source={source_name}.{source_component}.{source_cname}) with "
                f"(target={target_name}.{target_component}.{target_cname}): {e}"
            )
            raise IOError(msg)

    @staticmethod
    def _correct_signature(
        entry: Optional[SpecView] = None,
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
        EngineGraph.check_inputs_have_address(state)
        EngineGraph.check_graph_is_acyclic(state, plot=plot)
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
    def _get_view(spec, name: str, depth: Optional[List[str]] = None):
        depth = depth if depth else []
        # depth = [name] + depth
        return SpecView(spec, depth=depth, name=name)

    def get_view(self, name: str, depth: Optional[List[str]] = None):
        return self._get_view(self.get_spec(name), name, depth)

    @staticmethod
    @supported_types(str, int, list, float, bool, dict, EntitySpec, SpecView, None)
    def _set(state, mapping):
        merge(state, mapping)

    @staticmethod
    def _check_spec(spec):
        if spec.config.name in ["sensors", "actuators"]:
            return
        from eagerx.core.entities import EngineNode

        EngineNode.check_spec(spec)
