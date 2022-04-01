import yaml
from copy import deepcopy
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Union, Dict, Optional, Any
import rospy
from eagerx.utils.utils import (
    get_opposite_msg_cls,
    get_module_type_string,
    get_cls_from_string,
    get_attribute_from_module,
    substitute_args,
    msg_type_error,
    supported_types,
)
from eagerx.utils.network_utils import (
    reset_graph,
    episode_graph,
    plot_graph,
    color_nodes,
    color_edges,
    is_stale,
)
from eagerx.core.entities import Node, BaseConverter, SpaceConverter
from eagerx.core.view import GraphView
from eagerx.core.specs import (
    ResetNodeSpec,
    ObjectSpec,
    ConverterSpec,
    EntitySpec,
    NodeSpec,
)

yaml.Dumper.ignore_aliases = lambda *args: True  # todo: check if needed.


def merge(a, b, path=None):
    "merges b into a"
    # If it is a spec, convert to params
    if path is None:
        path = []
    for key in b:
        if isinstance(b[key], EntitySpec):
            b[key] = b[key].params
        if isinstance(b[key], GraphView):
            b[key] = b[key].to_dict()
        if key in a:
            if isinstance(a[key], dict) and isinstance(b[key], dict):
                merge(a[key], b[key], path + [str(key)])
            elif a[key] == b[key]:
                pass  # same leaf value
            else:
                a[key] = b[key]
        else:
            a[key] = b[key]
    return a


class Graph:
    """The Graph API allows users to form a graph of connected nodes and objects."""

    def __init__(self, state: Dict):
        self._state = state

    def __str__(self):
        return yaml.dump(self._state)

    @classmethod
    def create(
        cls,
        nodes: Optional[List[Union[NodeSpec, ResetNodeSpec]]] = None,
        objects: Optional[List[ObjectSpec]] = None,
    ) -> "Graph":
        """Create a new graph with nodes and objects.

        :param nodes: Nodes to add.
        :param objects: Objects to add.
        :return: The graph.
        """
        if nodes is None:
            nodes = []
        if objects is None:
            objects = []
        if isinstance(nodes, (NodeSpec, ResetNodeSpec)):
            nodes = [nodes]
        if isinstance(objects, ObjectSpec):
            objects = [objects]

        # Add action & observation node to list
        import eagerx.core.nodes  # noqa  Required so that actions & observations node are registered.

        actions = Node.make("Actions")
        observations = Node.make("Observations")
        nodes += [actions, observations]

        # Create a state
        state = dict(nodes=dict(), connects=list(), backup=dict(), gui_state=dict())
        graph = cls(state)
        graph.add(nodes)
        graph.add(objects)
        return graph

    def add(
        self,
        entities: Union[Union[NodeSpec, ResetNodeSpec, ObjectSpec], List[Union[NodeSpec, ResetNodeSpec, ObjectSpec]]],
    ) -> None:
        """Add nodes/objects to the graph.

        :param entities: Nodes/objects to add.
        """
        if not isinstance(entities, list):
            entities = [entities]

        for entity in entities:
            name = entity.config.name
            assert (
                name not in self._state["nodes"]
            ), f'There is already a node or object registered in this graph with name "{name}".'
            assert not entity.has_graph, f"Spec '{name}' is already added to a graph."

            # Add node to state
            self._state["nodes"][name] = entity.params
            self._state["backup"][name] = entity.params

            # Add graph reference to spec
            entity.set_graph(self)

    def remove(self, names: Union[str, EntitySpec, List[Union[str, EntitySpec]]], remove: bool = False) -> None:
        """Removes nodes/objects from the graph.

        - First, all associated connections are disconnected.

        - Then, removes the node/object.

        :param names: Either the name or spec of the node/object that is to be removed.
        :param remove: Flag to also remove observations/actions if they are left disconnected after the node/object was removed.
                       Actions are only removed if they are completely disconnected.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            if isinstance(name, EntitySpec):
                name = name.params["config"]["name"]
                assert name in self._state["nodes"], f" No entity with name '{name}' in graph."
            for source, target in deepcopy(self._state["connects"]):
                if name in [source[0], target[0]]:
                    if source[0] == "env/actions":
                        action = source[2]
                        source = None
                    else:
                        action = None
                        source = self.get_view(source[0], source[1:])
                    if target[0] == "env/observations":
                        observation = target[2]
                        target = None
                    else:
                        observation = None
                        target = self.get_view(target[0], target[1:])
                    self.disconnect(source, target, action, observation, remove=remove)
            self._state["nodes"].pop(name)

    def add_component(
        self,
        entry: Optional[GraphView] = None,
        action: Optional[str] = None,
        observation: Optional[str] = None,
    ) -> None:
        """Selects an available component entry (e.g. input, output, etc...) that was not already selected.

        :param entry: Selects the entry, so that it can be connected.
        :param action: Adds a disconnected action entry.
        :param observation: Adds a disconnected observation entry.
        """
        # assert only action, only observation, only entry
        self._correct_signature(entry, action, observation)
        if entry is not None:  # component parameter
            self._add_component(entry)
        if action:
            self._add_action(action)
        if observation:
            self._add_observation(observation)

    def _add_component(self, entry: GraphView):
        """Adds a component entry to the selection list."""
        name, component, cname = entry()

        # For feedthroughs, add the corresponding output instead
        component = "outputs" if component == "feedthroughs" else component

        # Add cname to selection list if it is not already selected
        params = self._state["nodes"][name]
        assert cname not in params["config"][component], '"%s" already selected in "%s" under %s.' % (
            cname,
            name,
            component,
        )
        params["config"][component].append(cname)

    def _add_action(self, action: str):
        """Adds disconnected action entry to 'env/actions' node in self._state."""
        assert action != "set", 'Cannot define an action with the reserved name "set".'
        params_action = self._state["nodes"]["env/actions"]
        if action not in params_action["outputs"]:  # Action already registered
            params_action["outputs"][action] = dict()
            view = self.get_view("env/actions", ["outputs", action])
            self._add_component(view)

    def _add_observation(self, observation: str):
        """Adds disconnected observation entry to 'env/observations' node in self._state."""
        assert observation != "actions_set", 'Cannot define an observations with the reserved name "actions_set".'
        params_obs = self._state["nodes"]["env/observations"]
        if observation in params_obs["inputs"]:
            assert len(params_obs["inputs"][observation]) == 0, (
                'Observation "%s" already exists' " and is connected." % observation
            )
        else:
            params_obs["inputs"][observation] = dict()
            view = self.get_view("env/observations", ["inputs", observation])
            self._add_component(view)

    def remove_component(
        self,
        entry: Optional[GraphView] = None,
        action: Optional[str] = None,
        observation: Optional[str] = None,
        remove: bool = False,
    ) -> None:
        """Deselects a component entry (e.g. input, output, etc...) that was selected.

        - First, all associated connections are disconnected.

        - Then, deselects the component entry. For feedthroughs, it will also remove the corresponding output entry.

        :param entry: Deselects the entry.
        :param action: Removes an action entry.
        :param observation: Removes an observation entry
        :param remove: Flag to also remove observations/actions if they are left disconnected after the entry was removed.
                       Actions are only removed if they are completely disconnected.
        """
        self._correct_signature(entry, action, observation)
        if entry:  # component parameter
            self._remove_component(entry, remove=remove)
        if action:
            self._remove_action(action)
        if observation:
            self._remove_observation(observation)

    def _remove_component(self, entry: GraphView, remove: bool = False):
        """Removes a component entry from the selection list. It will first disconnect all connections in connect.
        For feedthroughs, it will remove the corresponding output from the selection list.
        """
        # For feedthroughs, remove the corresponding output instead
        name, component, cname = entry()
        component = "outputs" if component == "feedthroughs" else component
        entry = self.get_view(name, [component, cname])
        self._is_selected(self._state, entry)

        # Disconnect component entry
        self._disconnect_component(entry, remove=remove)

        # Remove cname from selection list
        params = self._state["nodes"][name]
        params["config"][component].remove(cname)

    def _remove_action(self, action: str):
        """Method to remove an action. It will first disconnect all connections in connect."""
        view = self.get_view("env/actions", ["outputs", action])
        self._remove_component(view, remove=False)
        params_action = self._state["nodes"]["env/actions"]
        source = ["env/actions", "outputs", action]
        connect_exists = False
        for c in self._state["connects"]:
            if source == c[0]:
                connect_exists = True
                target = c[1]
                break
        assert (
            not connect_exists
        ), f'Action entry "{action}" cannot be removed, because it is not disconnected. Connection with target {target} still exists.'
        params_action["outputs"].pop(action)

    def _remove_observation(self, observation: str):
        """Method to remove an observation. It will first disconnect all connections in connect."""
        view = self.get_view("env/observations", ["inputs", observation])
        self._remove_component(view, remove=False)
        params_obs = self._state["nodes"]["env/observations"]
        target = ["env/observations", "inputs", observation]
        connect_exists = False
        for c in self._state["connects"]:
            if target == c[1]:
                connect_exists = True
                source = c[0]
                break
        assert not connect_exists, (
            'Observation entry "%s" cannot be removed, because it is not disconnected. Connection with source %s still exists.'
            % (observation, source)
        )
        # assert observation in params_obs['inputs'], 'Observation "%s" cannot be removed,
        # because it does not exist.' % observation
        params_obs["inputs"].pop(observation)

    def connect(
        self,
        source: GraphView = None,
        target: GraphView = None,
        action: str = None,
        observation: str = None,
        converter: Optional[ConverterSpec] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
    ) -> None:
        """Connect an action/source (i.e. node/object component) to an observation/target (i.e. node/object component).

        :param source: Compatible source types are
                       :attr:`~eagerx.core.specs.NodeSpec.outputs`,
                       :attr:`~eagerx.core.specs.ObjectSpec.sensors`, and
                       :attr:`~eagerx.core.specs.ObjectSpec.states`.
        :param target: Compatible target types are
                       :attr:`~eagerx.core.specs.NodeSpec.inputs`,
                       :attr:`~eagerx.core.specs.ObjectSpec.actuators`,
                       :attr:`~eagerx.core.specs.ResetNodeSpec.targets`, and
                       :attr:`~eagerx.core.specs.ResetNodeSpec.feedthroughs`.
        :param action: Name of the action to connect (and add).
        :param observation: Name of the observation to connect (and add).
        :param converter: An input converter that converts the received input message before passing it
                          to the node's :func:`~eagerx.core.entities.Node.callback`.

                          .. note:: Output converters can only be set by manipulating the :class:`~eagerx.core.specs.NodeSpec`
                           and :class:`~eagerx.core.specs.ObjectSpec` directly.
        :param window: A non-negative number that specifies the number of messages to pass to the node's :func:`~eagerx.core.entities.Node.callback`.

                       - *window* = 1: Only the last received input message.

                       - *window* = *x* > 1: The trailing last *x* received input messages.

                       - *window* = 0: All input messages received since the last call to the node's :func:`~eagerx.core.entities.Node.callback`.

                       .. note:: With *window* = 0, the number of input messages may vary and can even be zero.

        :param delay: A non-negative simulated delay (seconds). This delay is ignored if
                      :attr:`~eagerx.core.entities.Bridge.simulate_delays` = True
                      in the bridge's :func:`~eagerx.core.entities.Bridge.spec`.
        :param skip: Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.Node.callback`.
                     May be necessary to ensure that the connected graph is directed and acyclic.
        """
        assert not source or not action, (
            'You cannot specify a source if you wish to connect action "%s",' " as the action will act as the source." % action
        )
        assert not target or not observation, (
            'You cannot specify a target if you wish to connect observation "%s",'
            " as the observation will act as the target." % observation
        )
        assert not (observation and action), "You cannot connect an action directly to an observation."

        if isinstance(converter, ConverterSpec):
            converter = converter.params
        if isinstance(converter, GraphView):
            converter = converter.to_dict()

        if action:  # source = action
            self.add_component(action=action)
            self._connect_action(action, target, converter=converter)
            source = self.get_view("env/actions", ["outputs", action])
        elif observation:  # target = observation
            self.add_component(observation=observation)
            converter = self._connect_observation(source, observation, converter=converter)
            target = self.get_view("env/observations", ["inputs", observation])
        self._connect(source, target, converter, window, delay, skip)

    def _connect(
        self,
        source: GraphView = None,
        target: GraphView = None,
        converter: Optional[ConverterSpec] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
    ):
        """Method to connect a source to a target. For actions/observations, first a (new) disconnected entry must be created,
        after which an additional call to connect_action/observation is required before calling this method.
        For more info, see self.connect."""
        # Perform checks on source
        self._is_selected(self._state, source)

        # Perform checks on target
        target_name, target_comp, target_cname = target()
        if target_comp == "feedthroughs":
            assert window is None or window > 0, "Feedthroughs must have a window > 0, " "else no action can be fed through."
            assert not skip, (
                "Feedthroughs cannot skip, as they only feedthrough outputs. "
                "When setting skip=True, no msg can be fed through."
            )
            ft_view = self.get_view(target_name, ["outputs", target_cname])
            self._is_selected(self._state, ft_view)
        else:
            self._is_selected(self._state, target)

        # Make sure that target is not already connected.
        for s, t in self._state["connects"]:
            t_name, t_comp, t_cname = t
            flag = not (target_name == t_name and target_comp == t_comp and target_cname == t_cname)
            assert flag, f'Target "{target}" is already connected to source "{s}"'

        # Add properties to target params
        if converter is not None:
            self.set({"converter": converter}, target)
        if window is not None:
            self.set({"window": window}, target)
        if delay is not None:
            self.set({"delay": delay}, target)
        if skip is not None:
            self.set({"skip": skip}, target)

        # Add connection
        Graph.check_msg_type(source, target, self._state)
        connect = [list(source()), list(target())]
        self._state["connects"].append(connect)

    def _connect_action(self, action, target, converter=None):
        """Method to connect a (previously added) action, that *precedes* self._connect(source, target)."""
        params_action = self._state["nodes"]["env/actions"]
        assert action in params_action["outputs"], f'Action "{action}" must be added, before you can connect it.'
        name, component, cname = target()
        if component == "feedthroughs":
            component = "outputs"
            target = self.get_view(name, [component, cname])
            # Perform checks on target
            self._is_selected(self._state, target)
        else:
            # Perform checks on target
            self._is_selected(self._state, target)

        # Infer source properties (converter & msg_type) from target
        sc = target.space_converter
        space_converter = sc.to_dict() if isinstance(sc, GraphView) else sc
        msg_type_C = get_cls_from_string(target.msg_type)
        if converter:  # Overwrite msg_type_B if input converter specified
            msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
        else:
            msg_type_B = msg_type_C

        # Set properties in node params of 'env/actions'
        if len(params_action["outputs"][action]) > 0:  # Action already registered
            space_converter_state = params_action["outputs"][action]["converter"]
            msg_type_B_state = get_opposite_msg_cls(params_action["outputs"][action]["msg_type"], space_converter_state)
            assert msg_type_B == msg_type_B_state, (
                f'Conflicting msg_types for action "{action}" that is already '
                f"used in another connection. Occurs with connection %s" % tuple([name, component, cname])
            )

            if not space_converter == space_converter_state:
                rospy.logwarn(
                    'Conflicting %s for action "%s". '
                    "Not using the space_converter "
                    "of %s[%s][%s]" % ("space_converters", action, name, component, cname)
                )
            msg_type_A = get_opposite_msg_cls(msg_type_B_state, space_converter_state)
        else:
            assert target.space_converter is not None, (
                f'"{cname}" does not have a space_converter defined for ' f'{component} in the spec of object "{name}".'
            )
            # Verify that converter is not modifying the msg_type (i.e. it is a processor).
            assert msg_type_B == msg_type_C, (
                "Cannot have a converter that maps to a different msg_type as the "
                "converted msg_type will not be compatible with the "
                "space_converter specified in the spec."
            )
            msg_type_A = get_opposite_msg_cls(msg_type_B, space_converter)
            msg_type = get_module_type_string(msg_type_A)
            mapping = dict(
                msg_type=msg_type,
                rate="$(config rate)",
                converter=space_converter,
                space_converter=None,
            )
            self._set(params_action["outputs"], {action: mapping})

    def _connect_observation(self, source, observation, converter):
        """Method to connect a (previously added & disconnected) observation, that *precedes* self._connect(source, target)."""
        params_obs = self._state["nodes"]["env/observations"]
        assert observation in params_obs["inputs"], 'Observation "%s" must be added, before you can connect it.' % observation
        name, component, cname = source()

        assert converter is not None or source.space_converter, (
            f'"{cname}" does not have a space_converter '
            f'defined under {component} in the spec of "{name}". '
            "Either specify it there, or add an input converter "
            "that acts as a space_converter to this connection."
        )

        # Infer target properties (converter & msg_type) from source
        msg_type_A = get_cls_from_string(source.msg_type)
        output_converter = source.converter.to_dict()
        msg_type_B = get_opposite_msg_cls(msg_type_A, output_converter)
        if converter is None:
            converter = source.space_converter.to_dict()
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)

        # Set properties in node params of 'env/observations'
        assert len(params_obs["inputs"][observation]) == 0, 'Observation "%s" already connected.' % observation
        msg_type = get_module_type_string(msg_type_C)
        params_obs["inputs"][observation]["msg_type"] = get_module_type_string(msg_type_C)
        mapping = dict(
            msg_type=msg_type,
            delay=0.0,
            window=1,
            skip=False,
            external_rate=None,
            converter=BaseConverter.make("Identity"),
            space_converter=None,
            address=None,
        )
        self._set(params_obs["inputs"], {observation: mapping})
        return converter

    def disconnect(
        self,
        source: GraphView = None,
        target: GraphView = None,
        action: str = None,
        observation: str = None,
        remove: bool = False,
    ) -> None:
        """Disconnects a source/action from a target/observation.

        :param source: Compatible source types are
                       :attr:`~eagerx.core.specs.NodeSpec.outputs`,
                       :attr:`~eagerx.core.specs.ObjectSpec.sensors`, and
                       :attr:`~eagerx.core.specs.ObjectSpec.states`.
        :param target: Compatible target types are
                       :attr:`~eagerx.core.specs.NodeSpec.inputs`,
                       :attr:`~eagerx.core.specs.ObjectSpec.actuators`,
                       :attr:`~eagerx.core.specs.ResetNodeSpec.targets`, and
                       :attr:`~eagerx.core.specs.ResetNodeSpec.feedthroughs`.
        :param action: Name of the action to connect (and add).
        :param observation: Name of the observation to connect (and add).
        :param remove: Flag to also remove observations/actions if they are left disconnected after the entry was disconnected.
                       Actions are only removed if they are completely disconnected.
        """
        self._disconnect(source, target, action, observation)
        if remove:
            if action:
                connect_exists = False
                source = ["env/actions", "outputs", action]
                for c in self._state["connects"]:
                    if source == c[0]:
                        connect_exists = True
                        break
                if not connect_exists:
                    self.remove_component(action=action)
            if observation:
                self.remove_component(observation=observation)

    def _disconnect(
        self,
        source: GraphView = None,
        target: GraphView = None,
        action: str = None,
        observation: str = None,
    ):
        """Disconnects a source from a target. The target is reset in self._state to its disconnected state."""
        assert not source or not action, (
            'You cannot specify a source if you wish to disconnect action "%s", '
            "as the action will act as the source." % action
        )
        assert not target or not observation, (
            'You cannot specify a target if you wish to disconnect observation "%s", '
            "as the observation will act as the target." % observation
        )
        assert not (observation and action), (
            "You cannot disconnect an action from an observation," " as such a connection cannot exist."
        )

        # Create source & target entries
        if action:
            assert target is not None, (
                f"If you want to disconnect action {action}, " f"please also specify the corresponding target."
            )

            source = self.get_view("env/actions", ["outputs", action])
        if observation:
            assert source is not None, (
                f"If you want to disconnect observation {observation}," f" please also specify the corresponding source."
            )

            target = self.get_view("env/observations", ["inputs", observation])

        # Check if connection exists
        self._is_selected(self._state, source)
        self._is_selected(self._state, target)

        # Check if connection exists
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

        # Reset source params to disconnected state
        if action:
            self._disconnect_action(action)
        else:
            pass  # Nothing to do here (for now)

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/skip/converter.
        if observation:
            self._disconnect_observation(observation)
        else:
            target_name, target_comp, target_cname = target()
            target_params = self._state["nodes"][target_name]
            target_params[target_comp][target_cname] = self._state["backup"][target_name][target_comp][target_cname]

    def _disconnect_component(self, entry: GraphView, remove=False):
        """Disconnects all associated connects from self._state.
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
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
            if source_name == "env/actions":
                action = source_cname
                source = None
            else:
                action = None
                source = source
            if target_name == "env/observations":
                observation = target_cname
                target = None
            else:
                observation = None
                target = target
            if name == source_name and component == source_comp and cname == source_cname:
                self.disconnect(source, target, action, observation, remove=remove)
                was_connected = True
            elif name == target_name and component == target_comp and cname == target_cname:
                self.disconnect(source, target, action, observation, remove=remove)
                was_connected = True
        return was_connected

    def _disconnect_action(self, action: str):
        """Returns the action entry back to its disconnected state.
        That is, remove space_converter if it is not connected to any other targets."""
        params_action = self._state["nodes"]["env/actions"]
        assert action in params_action["outputs"], 'Cannot disconnect action "%s", as it does not exist.' % action
        source = ["env/actions", "outputs", action]
        connect_exists = False
        for c in self._state["connects"]:
            if source == c[0]:
                connect_exists = True
                break
        if not connect_exists:
            params_action["outputs"][action] = dict()

    def _disconnect_observation(self, observation: str):
        """Returns the observation entry back to its disconnected state (i.e. empty dict)."""
        params_obs = self._state["nodes"]["env/observations"]
        assert observation in params_obs["inputs"], 'Cannot disconnect observation "%s", as it does not exist.' % observation
        params_obs["inputs"][observation] = dict()

    def rename(
        self,
        new: str,
        action: Optional[str] = None,
        observation: Optional[str] = None,
    ) -> None:
        """Renames an action/observation.

        :param new: New name.
        :param action: Old action name.
        :param observation: Old observation name.
        """
        if action:
            assert observation is None, "Cannot supply both an action and observation."
            entry = self.get_view("env/actions", ["outputs", action])
        if observation:
            assert action is None, "Cannot supply both an action and observation."
            assert observation is not None, "Either an action or observation must be supplied."
            entry = self.get_view("env/observations", ["inputs", observation])
        self._rename_component(entry, new_cname=new)

    def _rename_component(self, entry: GraphView, new_cname: str):
        """Renames the component name (cname) of an entity (node/object) in _state['nodes'] and self._state[connects].
        We cannot change names for node/object components, because their python implementation could depend on it.
        Does not work for feedthroughs.
        """
        name, component, old_cname = entry()
        default = self._state["backup"][name]
        params = self._state["nodes"][name]

        # For now, we only support changing action/observation cnames
        assert name in ["env/observations", "env/actions"], (
            f'Cannot change "{old_cname}" of "{name}". Only name ' "changes to observations and actions are supported."
        )
        assert new_cname not in params[component], f'"{new_cname}" already defined in "{name}" under {component}.'

        # Rename cname in params
        # Does not work for outputs with feedthroughs. Then, both outputs and feedthroughs cnames must be changed.
        for d in (params, default):
            if component in d and old_cname in d[component]:
                assert new_cname not in d[component], f'"{new_cname}" already defined in "{name}" under {component}.'
                d[component][new_cname] = d[component].pop(old_cname)
            if component in d["config"] and old_cname in d["config"][component]:
                assert new_cname not in d["config"][component], f'"{new_cname}" already defined in "{name}" under {component}.'
                d["config"][component].remove(old_cname)
                d["config"][component].append(new_cname)

        # Rename cname in all connects
        for source, target in self._state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target

            if source_comp == component and source_cname == old_cname:
                source[2] = new_cname
            if target_comp == component and target_cname == old_cname:
                target[2] = new_cname

    def set(
        self,
        mapping: Any,
        entry: Optional[GraphView] = None,
        action: Optional[str] = None,
        observation: Optional[str] = None,
        parameter: Optional[str] = None,
    ) -> None:
        """Sets the parameters of a node/object/action/observation.

        .. note:: If a converter is set, we check if the msg_type changes with the new converter. If so, the entry is disconnected.

        :param mapping: Either a mapping with *key* = *parameter*,
                        or a single value that corresponds to the optional *parameter* argument.
        :param entry: The entry whose parameters are mutated.
        :param action: Action name whose parameters are mutated.
        :param observation: observation name whose parameters are mutated.
        :param parameter: If only a single value needs to be set. See documentation for *mapping*.
        """
        self._correct_signature(entry, action, observation)
        if action:
            entry = self.get_view("env/actions", ["outputs", action])
        if observation:
            entry = self.get_view("env/observations", ["inputs", observation])
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
                if isinstance(value, ConverterSpec):
                    value = value.params
                elif isinstance(value, GraphView):
                    value = value.to_dict()
                self._set_converter(entry, value)
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
                    assert parameter not in ["msg_type"], f"You cannot modify msg_type '{value}'."
                    p = self._state["nodes"][name]["config"]
                else:
                    name, component, cname = entry()
                    p = self._state["nodes"][name][component][cname]
                self._set(p, {parameter: value})

    def _set_converter(self, entry: GraphView, converter: Dict):
        """Replaces the converter specified for a node's/object's I/O.
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
        """
        _ = entry.converter  # Check if parameter exists
        name, component, cname = entry()
        params = self._state["nodes"][name]

        # Check if converted msg_type of old converter is equal to the msg_type of newly specified converter
        if component == "feedthroughs":
            msg_type = get_cls_from_string(params["outputs"][cname]["msg_type"])
        else:
            msg_type = get_cls_from_string(params[component][cname]["msg_type"])
        converter_old = params[component][cname]["converter"]
        msg_type_ros_old = get_opposite_msg_cls(msg_type, converter_old)
        msg_type_ros_new = get_opposite_msg_cls(msg_type, converter)
        if not msg_type_ros_new == msg_type_ros_old:
            was_disconnected = self._disconnect_component(entry)
        else:
            was_disconnected = False

        # If disconnected action/observation, we cannot add converter so raise error.
        assert not (
            was_disconnected and name in ["env/actions", "env/observations"]
        ), 'Cannot change the converter of action/observation "%s", as it changes the msg_type from ' '"%s" to "%s"' % (
            cname,
            msg_type_ros_old,
            msg_type_ros_new,
        )

        # Make sure the converter is a spaceconverter
        if name in ["env/actions", "env/observations"]:
            converter_cls = get_attribute_from_module(converter["converter_type"])
            assert issubclass(converter_cls, SpaceConverter), (
                'Incorrect converter type for "%s". Action/observation converters should always be a subclass of '
                '"%s/%s".' % (cname, SpaceConverter.__module__, SpaceConverter.__name__)
            )

        # Replace converter
        params[component][cname].pop("converter")
        self._set(params[component][cname], {"converter": converter})

    def get(
        self,
        entry: Optional[Union[GraphView, EntitySpec]] = None,
        action: Optional[str] = None,
        observation: Optional[str] = None,
        parameter: Optional[str] = None,
    ) -> Any:
        """Fetches the parameters of a node/object/action/observation.

        :param entry: The entry whose parameters are fetched.
        :param action: Action name whose parameters are fetched.
        :param observation: observation name whose parameters are fetched.
        :param parameter: If only a single parameter needs to be fetched.
        :return: Parameters
        """
        if isinstance(entry, EntitySpec):
            name = entry.params["config"]["name"]
            assert name in self._state["nodes"], f" No entity with name '{name}' in graph."
            return self._state["nodes"][name]
        self._correct_signature(entry, action, observation)
        if action:
            entry = self.get_view("env/actions", ["outputs", action])
        if observation:
            entry = self.get_view("env/observations", ["inputs", observation])
        if parameter:
            return getattr(entry, parameter)
        else:
            return entry

    def register(self):
        # Check if valid graph.
        assert self.is_valid(plot=False), "Graph not valid."

        # Add addresses based on connections
        state = deepcopy(self._state)
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target

            # For actions & observations, replace default args
            if source_name == "env/actions":
                default = state["nodes"][target_name]["config"]
                context = {"config": default}
                cname_params = state["nodes"][source_name][source_comp][source_cname]
                substitute_args(cname_params, context, only=["config", "ns"])
                address = "%s/%s/%s" % ("environment", source_comp, source_cname)
            elif target_name == "env/observations":
                default = state["nodes"][source_name]["config"]
                context = {"config": default}
                cname_params = state["nodes"][target_name][target_comp][target_cname]
                substitute_args(cname_params, context, only=["config", "ns"])
                address = "%s/%s/%s" % (source_name, source_comp, source_cname)
            else:
                address = "%s/%s/%s" % (source_name, source_comp, source_cname)
            state["nodes"][target_name][target_comp][target_cname]["address"] = address

        # Initialize param objects
        nodes = []
        objects = []
        render = None
        actions = None
        observations = None
        for name, params in state["nodes"].items():
            if "node_type" in params:
                if name == "env/actions":
                    actions = NodeSpec(params)
                elif name == "env/observations":
                    observations = NodeSpec(params)
                elif name == "env/render":
                    render = NodeSpec(params)
                else:
                    nodes.append(NodeSpec(params))
            else:
                objects.append(ObjectSpec(params))

        assert actions, "No action node defined in the graph."
        assert observations, "No observation node defined in the graph."
        return nodes, objects, actions, observations, render

    def render(
        self,
        source: GraphView,
        rate: float,
        converter: Optional[ConverterSpec] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
        entity_id: str = "Render",
        **kwargs,
    ):
        """Render the :class:`sensor_msgs.msg.Image` messages produced by a node/sensor in the graph.

        :param source: Compatible source types are :attr:`~eagerx.core.specs.NodeSpec.outputs` and
                       :attr:`~eagerx.core.specs.ObjectSpec.sensors`.
        :param rate: The rate (Hz) at which to render the images.
        :param converter: An input converter that converts the received input message before passing it
                          to the node's :func:`~eagerx.core.entities.Node.callback`.

                          .. note:: Output converters can only be set by manipulating the :class:`~eagerx.core.specs.NodeSpec`
                           and :class:`~eagerx.core.specs.ObjectSpec` directly.
        :param window: A non-negative number that specifies the number of messages to pass to the node's :func:`~eagerx.core.entities.Node.callback`.

                       - *window* = 1: Only the last received input message.

                       - *window* = *x* > 1: The trailing last *x* received input messages.

                       - *window* = 0: All input messages received since the last call to the node's :func:`~eagerx.core.entities.Node.callback`.

                       .. note:: With *window* = 0, the number of input messages may vary and can even be zero.

        :param delay: A non-negative simulated delay (seconds). This delay is ignored if
                      :attr:`~eagerx.core.entities.Bridge.simulate_delays` = True
                      in the bridge's :func:`~eagerx.core.entities.Bridge.spec`.
        :param skip: Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.Node.callback`.
                     May be necessary to ensure that the connected graph is directed and acyclic.
        :param entity_id: The :attr:`~eagerx.core.entities.Node.entity_id` with which the render node was registered
                   with the :func:`eagerx.core.register.spec` decorator. By default, it uses the standard Render node.
        :param kwargs: Optional arguments required by the render node.
        """
        # Delete old render node from self._state['nodes'] if it exists
        if "env/render" in self._state["nodes"]:
            self.remove("env/render")

        # Add (new) render node to self._state['node']
        render = Node.make(entity_id, rate=rate, **kwargs)
        self.add(render)

        # Create connection
        target = self.get_view("env/render", depth=["inputs", "image"])
        self.connect(
            source=source,
            target=target,
            converter=converter,
            window=window,
            delay=delay,
            skip=skip,
        )

    def save(self, file: str):
        """Saves the graph state.

        The state is saved in *.yaml* format and contains the state of every added node, object, action, and observation
        and the connections between them.

        :param file: A string giving the name (and the file if the file isn't in the current working directory).
        """
        with open(file, "w") as outfile:
            yaml.dump(self._state, outfile, default_flow_style=False)
        pass

    def load(self, file: str):
        """Loads the graph state.

        The state is loaded in *.yaml* format and contains the state of every added node, object, action, and observation
        and the connections between them.

        :param file: A string giving the name (and the file if the file isn't in the current working directory).
        """
        with open(file, "r") as stream:
            try:
                self._state = yaml.safe_load(stream)
                # self._state = yaml.load(file)
            except yaml.YAMLError as exc:
                print(exc)

    def gui(self) -> None:
        """Opens a graphical user interface of the graph.

        .. note:: Requires `eagerx-gui`:

                .. highlight:: python
                .. code-block:: python

                    pip3 install eagerx-gui
        """
        try:
            from eagerx_gui import launch_gui
        except ImportError as e:
            rospy.logwarn(
                f"{e}. You will likely have to install eagerx-gui. Please visit the eagerx_packages repository for installation instructions."
            )
            return

        self._state = launch_gui(deepcopy(self._state))

    @staticmethod
    def _is_selected(state: Dict, entry: GraphView):
        """Check if provided entry was selected in params."""
        name, component, cname = entry()
        assert name in state["nodes"], f'No entity with the anme "{name}" added to this graph.'
        params = state["nodes"][name]
        component = "outputs" if component == "feedthroughs" else component
        assert cname in params["config"][component], f'"{cname}" not selected in "{name}" under "config" in {component}.'

    @staticmethod
    def _correct_signature(
        entry: Optional[GraphView] = None,
        action: Optional[str] = None,
        observation: Optional[str] = None,
    ):
        # assert only action, only observation, or only entry
        if entry:  # component parameter
            assert action is None, "If 'entry' is specified, action argument cannot be specified."
            assert observation is None, "If 'entry' is specified, observation argument cannot be specified."
        if action:
            assert observation is None, "If action is specified, observation must be None."
            assert entry is None, "If action is specified, the 'entry' argument cannot be specified."
        if observation:
            assert action is None, "If observation is specified, action must be None."
            assert entry is None, "If action is specified, the 'entry' argument cannot be specified."

    def is_valid(self, plot=True) -> bool:
        """Checks the validity of the graph.

        - Checks if all selected actions, observations,
          :attr:`~eagerx.core.specs.NodeSpec.inputs`,
          :attr:`~eagerx.core.specs.ObjectSpec.actuators`,
          :attr:`~eagerx.core.specs.ResetNodeSpec.targets`, and
          :attr:`~eagerx.core.specs.ResetNodeSpec.feedthroughs` are connected.

        - Checks if the graph is directed and acyclic.

        :param plot: Flag to plot the graph. Can be helpful to identify cycles in the graph that break the required acyclic property.
        :returns: flag that specifies the validity of the graph.
        """
        return self._is_valid(self._state, plot=plot)

    @staticmethod
    def _is_valid(state, plot=True):
        state = deepcopy(state)
        Graph.check_msg_types_are_consistent(state)
        Graph.check_inputs_have_address(state)
        Graph.check_graph_is_acyclic(state, plot=plot)
        # Graph.check_exists_compatible_bridge(state)
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
        target_name, target_comp, target_cname = target()
        if target_comp == "feedthroughs":
            msg_type_in_registered = get_cls_from_string(state["nodes"][target_name]["outputs"][target_cname]["msg_type"])
        else:
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
            source = Graph._get_view(state, source[0], source[1:])
            target = Graph._get_view(state, target[0], target[1:])
            Graph.check_msg_type(source, target, state)
        return True

    @staticmethod
    def check_inputs_have_address(state):
        state = deepcopy(state)
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            address = "%s/%s/%s" % (source_name, source_comp, source_cname)
            state["nodes"][target_name][target_comp][target_cname]["address"] = address

        for name, params in state["nodes"].items():
            if "node_type" in params:
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
                        assert cname in params[component], (
                            f'"{cname}" was selected in {component} of "{name}", ' "but has no implementation."
                        )
                        if component not in ["inputs", "targets", "feedthroughs"]:
                            continue
                        p = params[component][cname]
                        assert "address" in p and p["address"] is not None, (
                            f'"{cname}" was selected in {component} '
                            f'of "{name}", but no address could be '
                            "produced/inferred. Either deselect it, "
                            "or connect it."
                        )
            else:
                for component in params["config"]:
                    if component not in ["sensors", "actuators", "states"]:
                        continue
                    for cname in params["config"][component]:
                        assert cname in params[component], (
                            f'"{cname}" was selected in {component} of "{name}", ' "but has no (agnostic) implementation."
                        )
                        if component not in ["actuators"]:
                            continue
                        assert "address" in params[component][cname], (
                            f'"{cname}" was selected in {component} of "{name}", '
                            "but no address could be produced/inferred. "
                            "Either deselect it, or connect it."
                        )
        return True

    @staticmethod
    def check_graph_is_acyclic(state, plot=True):
        # Add nodes
        G = nx.MultiDiGraph()
        label_mapping = {
            "env/observations/set": "observations",
            "env/render/done": "render",
        }
        for node, params in state["nodes"].items():
            default = params["config"]
            if "node_type" not in state["nodes"][node]:  # Object
                if "sensors" in default:
                    for cname in default["sensors"]:
                        name = "%s/sensors/%s" % (node, cname)
                        G.add_node(
                            "%s/sensors/%s" % (node, cname),
                            remain_active=False,
                            always_active=True,
                            is_stale=False,
                            has_tick=False,
                        )
                        if not ("actuators" in default and cname in default["actuators"]):
                            label_mapping[name] = "%s/%s" % (node, cname)
                if "actuators" in default:
                    for cname in default["actuators"]:
                        name = "%s/actuators/%s" % (node, cname)
                        G.add_node(
                            "%s/actuators/%s" % (node, cname),
                            remain_active=True,
                            always_active=False,
                            is_stale=False,
                            has_tick=False,
                        )
                        if not ("sensors" in default and cname in default["sensors"]):
                            label_mapping[name] = "%s/%s" % (node, cname)
            else:  # node
                if "outputs" in default:
                    for cname in default["outputs"]:
                        name = "%s/%s" % (node, cname)
                        if name == "env/actions/set":
                            continue
                        G.add_node(
                            name,
                            remain_active=False,
                            always_active=False,
                            is_stale=False,
                            has_tick=False,
                        )

        # Add edges
        for cname in state["nodes"]["env/actions"]["config"]["outputs"]:
            if cname == "set":
                continue
            name = "env/actions/%s" % cname
            label_mapping[name] = "actions/%s" % cname
            G.add_edge(
                "env/observations/set",
                name,  # key='%s/%s' % ('inputs', 'observations_set'),
                feedthrough=False,
                style="solid",
                color="black",
                alpha=1.0,
                is_stale=False,
                skip=False,
                source=("env/observations", "outputs", "set"),
                target=("env/actions", "inputs", "observations_set"),
            )
        target_comps = ["inputs", "actuators", "feedthroughs"]
        source_comps = ["outputs", "sensors"]
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            if source_comp in source_comps and target_comp in target_comps:
                # Determine source node name
                if "node_type" in state["nodes"][source_name]:
                    source_edge = "%s/%s" % (source_name, source_cname)
                else:
                    source_edge = "%s/%s/%s" % (source_name, source_comp, source_cname)
                # Determine target node name
                target_edges = []
                target_default = state["nodes"][target_name]["config"]
                if "node_type" in state["nodes"][target_name]:
                    for cname in target_default["outputs"]:
                        target_edge = "%s/%s" % (target_name, cname)
                        target_edges.append(target_edge)
                else:
                    target_edge = "%s/%s/%s" % (target_name, target_comp, target_cname)
                    target_edges.append(target_edge)

                # Determine stale nodes in real_reset routine via feedthrough edges
                if target_comp == "feedthroughs":
                    feedthrough = True
                else:
                    feedthrough = False

                # Determine edges that do not break DAG property (i.e. edges that are skipped)
                skip = state["nodes"][target_name][target_comp][target_cname]["skip"]
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
        not_active = is_stale(G)
        color_nodes(G)
        color_edges(G)

        # Remap action & observation labels to more readable form
        G = nx.relabel_nodes(G, label_mapping)

        # Check if graph is acyclic (excluding 'skip' edges)
        H, cycles = episode_graph(G)
        is_dag = nx.is_directed_acyclic_graph(H)

        # Plot graphs
        if plot:
            fig_env, ax_env = plt.subplots(nrows=1, ncols=1)
            ax_env.set_title("Communication graph (episode)")
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
        assert len(not_active) == 0, (
            'Stale episode graph detected. Nodes "%s" will be stale, while they must be active (i.e. connected) in order for the graph to resolve (i.e. not deadlock).'
            % not_active
        )

        # Create a shallow copy graph that excludes feedthrough edges
        F = reset_graph(G)
        not_active = is_stale(F)
        color_nodes(F)
        color_edges(F)

        # Plot graphs
        if plot:
            fig_reset, ax_reset = plt.subplots(nrows=1, ncols=1)
            ax_reset.set_title("Communication graph (reset)")
            _, _, _, pos = plot_graph(F, pos=pos, ax=ax_reset)
            plt.show()

        # Assert if reset graph is not stale
        has_real_reset = len([e for e, ft in nx.get_edge_attributes(G, "feedthrough").items() if ft]) > 0
        assert len(not_active) == 0 or not has_real_reset, (
            'Stale reset graph detected. Nodes "%s" will be stale, while they must be active (i.e. connected) in order for the graph to resolve (i.e. not deadlock).'
            % not_active
        )
        return True

    @staticmethod
    def check_exists_compatible_bridge(state, tablefmt="fancy_grid"):
        msg = "Bridge implementations are not part of the engine graph anymore."
        raise NotImplementedError(msg)
        # # Bridges are headers
        # from tabulate import tabulate
        # bridges = []
        # objects = []
        # for node, params in state["nodes"].items():
        #     default = params["config"]
        #     # todo: only check bridge if it concerns one of the selected components
        #     if "node_type" not in state["nodes"][node]:  # Object
        #         params = state["nodes"][node]
        #         entity_id = params["config"]["entity_id"]
        #         obj_name = params["config"]["name"]
        #         entry = [obj_name, entity_id]
        #
        #         # Add all (unknown) bridges to the list
        #         for key, _value in params.items():
        #             if key in [
        #                 "entity_type",
        #                 "config",
        #                 "sensors",
        #                 "actuators",
        #                 "states",
        #             ]:
        #                 continue
        #             if key not in bridges:
        #                 bridges.append(key)
        #
        #         # See what bridges support all object components
        #         for b in bridges:
        #             if b in params:
        #                 for component in ["sensors", "actuators", "states"]:
        #                     if component in default:
        #                         for cname in default[component]:
        #                             if component in params[b] and cname in params[b][component]:
        #                                 e_str = "x"  # Component entry is supported
        #                             else:
        #                                 e_str = " "  # Component entry is not supported
        #                                 break  # Break if entry in component is not supported
        #             else:  # Bridge name not even mentioned in object config
        #                 e_str = " "
        #             entry.append(e_str)
        #         objects.append(entry)
        #
        # # Fill up incompatible bridges that were added after object entries
        # for entry in objects:
        #     for _ in bridges[len(entry) - 2 :]:
        #         entry.append(" ")
        #
        # # Get compatible bridges
        # compatible = []
        # for idx, b in enumerate(bridges):
        #     idx = idx + 2
        #     for entry in objects:
        #         c = [True if entry[idx] == "x" else False for entry in objects]
        #     if len(c) == len(objects):
        #         compatible.append(b)
        #
        # # Objects are entries
        # headers = ["name", "object"]
        # for b in bridges:
        #     headers.append(b.replace("/", "/\n"))
        #
        # # Assert if there are compatible bridges
        # tabulate_str = tabulate(
        #     objects,
        #     headers=headers,
        #     tablefmt=tablefmt,
        #     colalign=["center"] * len(headers),
        # )
        # assert len(compatible), (
        #     "No compatible bridges for the selected objects. Ensure that all components, selected in each object, is supported by a common bridge.\n%s"
        #     % tabulate_str
        # )
        # return tabulate_str

    @staticmethod
    @supported_types(str, int, list, float, bool, dict, EntitySpec, GraphView, None)
    def _set(state, mapping):
        merge(state, mapping)

    @staticmethod
    def _get_view(state, name: str, depth: Optional[List[str]] = None):
        depth = depth if depth else []
        depth = [name] + depth
        return GraphView(Graph(state), depth=depth, name=name)

    def get_view(self, name: str, depth: Optional[List[str]] = None):
        depth = depth if depth else []
        depth = [name] + depth
        return GraphView(self, depth=depth, name=name)
