import os
import numpy as np
import yaml
from copy import deepcopy
import warnings
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Union, Dict, Optional, Any, Type, Tuple
import eagerx.core.log as log
from eagerx.utils.utils import (
    load,
    is_compatible,
    supported_types,
)
from eagerx.utils.utils_sub import substitute_args
from eagerx.utils.network_utils import (
    reset_graph,
    episode_graph,
    plot_graph,
    color_nodes,
    color_edges,
    is_stale,
)
import eagerx
from eagerx.core.space import Space
from eagerx.core.entities import Node
from eagerx.core.view import SpecView
from eagerx.core.specs import (
    ResetNodeSpec,
    ObjectSpec,
    EngineSpec,
    ProcessorSpec,
    EntitySpec,
    NodeSpec,
)

yaml.Dumper.ignore_aliases = lambda *args: True  # todo: check if needed.


def merge(a, b, path=None):
    """merges b into a"""
    # If it is a spec, convert to params
    if path is None:
        path = []
    for key in b:
        if isinstance(b[key], EntitySpec):
            b[key] = b[key].params
        if isinstance(b[key], SpecView):
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
        nodes: Optional[Union[Union[NodeSpec, ResetNodeSpec], List[Union[NodeSpec, ResetNodeSpec]]]] = None,
        objects: Optional[Union[ObjectSpec, List[ObjectSpec]]] = None,
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
        from eagerx.core.nodes import EnvNode

        environment = EnvNode.make()
        nodes += [environment]

        # Create a state
        state = dict(nodes=dict(), connects=list(), backup=dict(), gui_state=dict(), engine_id=None)
        graph = cls(state)
        graph.add(nodes)
        graph.add(objects)
        return graph

    def add(
        self,
        entities: Union[
            Union[NodeSpec, ResetNodeSpec, ObjectSpec, EngineSpec],
            List[Union[NodeSpec, ResetNodeSpec, ObjectSpec, EngineSpec]],
        ],
    ) -> None:
        """Add nodes/objects to the graph.

        :param entities: Nodes/objects to add.
        """
        Graph._add(self._state, entities)

    @staticmethod
    def _add(
        state: Dict,
        entities: Union[
            Union[NodeSpec, ResetNodeSpec, ObjectSpec, EngineSpec],
            List[Union[NodeSpec, ResetNodeSpec, ObjectSpec, EngineSpec]],
        ],
    ) -> None:
        """Add nodes/objects to the graph.

        :param entities: Nodes/objects to add.
        """
        if not isinstance(entities, list):
            entities = [entities]

        for entity in entities:
            # Check spec validity
            Graph._check_spec(entity)

            name = entity.config.name
            assert (
                name not in state["nodes"]
            ), f'There is already a node or object registered in this graph with name "{name}".'

            # Add node to state
            state["nodes"][name] = entity._params
            state["backup"][name] = entity.params

    def remove(self, names: Union[str, EntitySpec, List[Union[str, EntitySpec]]], remove: bool = False) -> List:
        """Removes nodes/objects from the graph.

        - First, all associated connections are disconnected.

        - Then, removes the node/object.

        :param names: Either the name or spec of the node/object that is to be removed.
        :param remove: Flag to also remove observations/actions if they are left disconnected after the node/object was removed.
                       Actions are only removed if they are completely disconnected.
        :return: list of disconnected connections.
        """
        if not isinstance(names, list):
            names = [names]
        disconnected = []
        for name in names:
            if isinstance(name, EntitySpec):
                name = name.params["config"]["name"]
                assert name in self._state["nodes"], f" No entity with name '{name}' in graph."
            for source, target in deepcopy(self._state["connects"]):
                if name in [source[0], target[0]]:
                    if source[0] == "environment":
                        action = source[2]
                        source = None
                    else:
                        action = None
                        source = self.get_view(source[0], source[1:])
                    if target[0] == "environment":
                        observation = target[2]
                        target = None
                    else:
                        observation = None
                        target = self.get_view(target[0], target[1:])
                    self.disconnect(source, target, action, observation, remove=remove)
                    disconnected.append([source, target])
            self._state["nodes"].pop(name)
            return disconnected

    def get_spec(self, name: str) -> EntitySpec:
        """Get Spec from the graph

        :param name: Name
        :return: The specification of the entity.
        """
        return Graph._get_spec(self._state, name)

    @staticmethod
    def _get_spec(state, name: str) -> EntitySpec:
        assert name in state["nodes"], f" No entity with name '{name}' in graph."
        params = state["nodes"][name]
        if "node_type" in params:  # Either Node or ResetNode
            if "targets" in params:  # == ResetNode
                spec = ResetNodeSpec(params)
            else:  # == Node
                spec = NodeSpec(params)
        else:  # == Object
            spec = ObjectSpec(params)
        return spec

    def add_component(
        self,
        entry: Optional[SpecView] = None,
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

    def _add_component(self, entry: SpecView):
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
        params_action = self._state["nodes"]["environment"]
        if action not in params_action["outputs"]:  # Action already registered
            params_action["outputs"][action] = dict()
            view = self.get_view("environment", ["outputs", action])
            self._add_component(view)

    def _add_observation(self, observation: str):
        """Adds disconnected observation entry to 'env/observations' node in self._state."""
        assert observation != "actions_set", 'Cannot define an observations with the reserved name "actions_set".'
        params_obs = self._state["nodes"]["environment"]
        if observation in params_obs["inputs"]:
            assert len(params_obs["inputs"][observation]) == 0, (
                'Observation "%s" already exists' " and is connected." % observation
            )
        else:
            params_obs["inputs"][observation] = dict()
            view = self.get_view("environment", ["inputs", observation])
            self._add_component(view)

    def remove_component(
        self,
        entry: Optional[SpecView] = None,
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

    def _remove_component(self, entry: SpecView, remove: bool = False):
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
        view = self.get_view("environment", ["outputs", action])
        self._remove_component(view, remove=False)
        params_action = self._state["nodes"]["environment"]
        source = ["environment", "outputs", action]
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
        view = self.get_view("environment", ["inputs", observation])
        self._remove_component(view, remove=False)
        params_obs = self._state["nodes"]["environment"]
        target = ["environment", "inputs", observation]
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
        source: SpecView = None,
        target: SpecView = None,
        action: str = None,
        observation: str = None,
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
        :param window: A non-negative number that specifies the number of messages to pass to the node's :func:`~eagerx.core.entities.Node.callback`.

                       - *window* = 1: Only the last received input message.

                       - *window* = *x* > 1: The trailing last *x* received input messages.

                       - *window* = 0: All input messages received since the last call to the node's :func:`~eagerx.core.entities.Node.callback`.

                       .. note:: With *window* = 0, the number of input messages may vary and can even be zero.

        :param delay: A non-negative simulated delay (seconds). This delay is ignored if
                      :attr:`~eagerx.core.entities.Engine.simulate_delays` = True
                      in the engine's :func:`~eagerx.core.entities.Engine.spec`.
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

        if action:  # source = action
            self.add_component(action=action)
            self._connect_action(action, target)
            source = self.get_view("environment", ["outputs", action])
        elif observation:  # target = observation
            self.add_component(observation=observation)
            self._connect_observation(source, observation)
            target = self.get_view("environment", ["inputs", observation])
        self._connect(source, target, window, delay, skip)

    def _connect(
        self,
        source: SpecView = None,
        target: SpecView = None,
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

        # Make sure that source and target are compatible
        self._is_compatible(self._state, source, target)

        # Make sure that target is not already connected.
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

    def _connect_action(self, action, target):
        """Method to connect a (previously added) action, that *precedes* self._connect(source, target)."""
        params_action = self._state["nodes"]["environment"]
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

        # Set properties in node params of 'env/actions'
        if len(params_action["outputs"][action]) > 0:  # Action already registered
            space_action = params_action["outputs"][action]["space"]
            space_target = target.space.to_dict()
            if not space_target == space_action:
                log.logwarn(
                    f'Conflicting space ({space_action}) for action "{action}". '
                    f"Not using the space ({space_target}) of {name}.{component}.{cname}"
                )
        else:
            assert target.space is not None, (
                f'"{name}.{component}.{cname}" does not have a space registered in the spec of "{name}". '
                f"Specify a space before connecting the action."
            )
            mapping = dict(
                rate="$(config rate)",
                processor=None,
                space=target.space,
            )
            self._set(params_action["outputs"], {action: mapping})

    def _connect_observation(self, source, observation):
        """Method to connect a (previously added & disconnected) observation, that *precedes* self._connect(source, target)."""

        params_obs = self._state["nodes"]["environment"]
        assert observation in params_obs["inputs"], f'Observation "{observation}" must be added, before you can connect it.'
        name, component, cname = source()

        assert source.space is not None, (
            f'"{name}.{component}.{cname}" does not have a space registered in the spec of "{name}". '
            f"Specify a space before connecting the observation."
        )

        # Set properties in node params of 'env/observations'
        assert len(params_obs["inputs"][observation]) == 0, 'Observation "%s" already connected.' % observation
        mapping = dict(
            delay=0.0,
            window=1,
            skip=False,
            processor=None,
            space=source.space,
            address=None,
        )
        self._set(params_obs["inputs"], {observation: mapping})

    def disconnect(
        self,
        source: SpecView = None,
        target: SpecView = None,
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
                source = ["environment", "outputs", action]
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
        source: SpecView = None,
        target: SpecView = None,
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

            source = self.get_view("environment", ["outputs", action])
        if observation:
            assert source is not None, (
                f"If you want to disconnect observation {observation}," f" please also specify the corresponding source."
            )

            target = self.get_view("environment", ["inputs", observation])

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

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/skip.
        if observation:
            self._disconnect_observation(observation)
        else:
            target_name, target_comp, target_cname = target()
            target_params = self._state["nodes"][target_name]
            target_params[target_comp][target_cname] = self._state["backup"][target_name][target_comp][target_cname]

    def _disconnect_component(self, entry: SpecView, remove=False):
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
            if source_name == "environment":
                action = source_cname
                source = None
            else:
                action = None
                source = source
            if target_name == "environment":
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
        That is, remove space if it is not connected to any other targets."""
        params_action = self._state["nodes"]["environment"]
        assert action in params_action["outputs"], 'Cannot disconnect action "%s", as it does not exist.' % action
        source = ["environment", "outputs", action]
        connect_exists = False
        for c in self._state["connects"]:
            if source == c[0]:
                connect_exists = True
                break
        if not connect_exists:
            params_action["outputs"][action] = dict()

    def _disconnect_observation(self, observation: str):
        """Returns the observation entry back to its disconnected state (i.e. empty dict)."""
        params_obs = self._state["nodes"]["environment"]
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
            entry = self.get_view("environment", ["outputs", action])
        if observation:
            assert action is None, "Cannot supply both an action and observation."
            assert observation is not None, "Either an action or observation must be supplied."
            entry = self.get_view("environment", ["inputs", observation])
        self._rename_component(entry, new_cname=new)

    def _rename_component(self, entry: SpecView, new_cname: str):
        """Renames the component name (cname) of an entity (node/object) in _state['nodes'] and self._state[connects].
        We cannot change names for node/object components, because their python implementation could depend on it.
        Does not work for feedthroughs.
        """
        name, component, old_cname = entry()
        default = self._state["backup"][name]
        params = self._state["nodes"][name]

        # For now, we only support changing action/observation cnames
        assert name in ["environment", "environment"], (
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
        entry: Optional[SpecView] = None,
        action: Optional[str] = None,
        observation: Optional[str] = None,
        parameter: Optional[str] = None,
    ) -> None:
        """Sets the parameters of a node/object/action/observation.

        :param mapping: Either a mapping with *key* = *parameter*,
                        or a single value that corresponds to the optional *parameter* argument.
        :param entry: The entry whose parameters are mutated.
        :param action: Action name whose parameters are mutated.
        :param observation: observation name whose parameters are mutated.
        :param parameter: If only a single value needs to be set. See documentation for *mapping*.
        """
        self._correct_signature(entry, action, observation)
        if action:
            entry = self.get_view("environment", ["outputs", action])
        if observation:
            entry = self.get_view("environment", ["inputs", observation])
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
                self._set_processor(entry, value)
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
        """Replaces the processor specified for a node's/object's input."""
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
            entry = self.get_view("environment", ["outputs", action])
        if observation:
            entry = self.get_view("environment", ["inputs", observation])
        if parameter:
            return getattr(entry, parameter)
        else:
            return entry

    def register(
        self, rate: float, engine: Optional[EngineSpec] = None
    ) -> Tuple["Graph", NodeSpec, EngineSpec, List[NodeSpec], Optional[NodeSpec]]:
        state = deepcopy(self._state)
        engine = deepcopy(engine)
        return self._register(state, rate, engine)

    @staticmethod
    def _register(
        state, rate: float, engine: Optional[EngineSpec] = None
    ) -> Tuple["Graph", NodeSpec, EngineSpec, List[NodeSpec], Optional[NodeSpec]]:
        assert rate >= 0, f"The rate must be non-negative ('{rate}' !=> 0)."

        # Add engine
        if engine:
            Graph._add_engine(state, engine)

        # Check that there is an engine specified.
        assert state["engine_id"] is not None, "Cannot register graph. There was no engine specified."

        # Substitute all objects
        Graph._substitute_all_objects(state)

        # Check if valid graph.
        assert Graph._is_valid(state, plot=False), "Graph not valid."

        # Reload all entities
        Graph._reload(state)

        # Add addresses based on connections
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target

            # For actions & observations, replace default args
            if source_name == "environment":
                default = state["nodes"][target_name]["config"]
                context = {"config": default}
                cname_params = state["nodes"][source_name][source_comp][source_cname]
                substitute_args(cname_params, context, only=["config", "ns"])
                cname_params["rate"] = "$(config rate)"  # Do not copy the rate.
                address = "%s/%s/%s" % ("environment", source_comp, source_cname)
            elif target_name == "environment":
                default = state["nodes"][source_name]["config"]
                context = {"config": default}
                cname_params = state["nodes"][target_name][target_comp][target_cname]
                substitute_args(cname_params, context, only=["config", "ns"])
                address = "%s/%s/%s" % (source_name, source_comp, source_cname)
            else:
                address = "%s/%s/%s" % (source_name, source_comp, source_cname)
            state["nodes"][target_name][target_comp][target_cname]["address"] = address

            # check if dtypes match
            s = Graph._get_view(state, source[0], source[1:])
            t = Graph._get_view(state, target[0], target[1:])
            Graph._is_compatible(state, s, t)
            source_dtype = state["nodes"][source_name][source_comp][source_cname]["space"]["dtype"]
            state["nodes"][target_name][target_comp][target_cname]["dtype"] = source_dtype

        # Extract node_names
        node_names = ["env/supervisor"]
        target_addresses = []
        for _i, params in state["nodes"].items():
            if params["config"]["name"] == "engine":
                continue
            node_names.append(params["config"]["name"])
            if "targets" in params["config"]:
                for cname in params["config"]["targets"]:
                    address = params["targets"][cname]["address"]
                    target_addresses.append(address)
        assert "node_names" not in engine.config, f'Keyword "{"node_names"}" is a reserved keyword.'
        assert "target_addresses" not in engine.config, f'Keyword "{"target_addresses"}" is a reserved keyword.'
        msg = "An Engine can only be launched with the options {ENVIRONMENT, EXTERNAL, NEW_PROCESS}."
        assert not engine.config.process == eagerx.ENGINE, msg
        with engine.config as d:
            d.node_names = node_names
            d.target_addresses = target_addresses

        # Get specs
        environment, engine, nodes, render = Graph._get_all_node_specs(state)

        # Set environment rate
        environment.config.rate = rate
        return Graph(deepcopy(state)), environment, engine, nodes, render

    @staticmethod
    def _get_all_node_specs(state: Dict) -> Tuple[NodeSpec, EngineSpec, List[NodeSpec], Optional[NodeSpec]]:
        # Get specs
        nodes = []
        render: Optional[NodeSpec] = None
        engine: EngineSpec = None
        environment: NodeSpec = None
        for name, params in state["nodes"].items():
            assert "node_type" in params, f"Node `{name}` is not a valid node. Have all Objects been replaced by nodes?"
            if name == "environment":
                environment = NodeSpec(params)
            elif name == "env/render":
                render = NodeSpec(params)
                nodes.append(render)
            elif name == "engine":
                engine = EngineSpec(params)
            else:
                nodes.append(NodeSpec(params))
        assert environment, "No environment node defined in the graph."
        return environment, engine, nodes, render

    def render(
        self,
        source: SpecView,
        rate: float,
        processor: Optional[ProcessorSpec] = None,
        window: Optional[int] = None,
        delay: Optional[float] = None,
        skip: Optional[bool] = None,
        render_cls: Type[Node] = None,
        process: int = eagerx.process.NEW_PROCESS,
        encoding: str = "bgr",
        **kwargs,
    ):
        """Visualize rgb images produced by a node/sensor in the graph. The rgb images must be of `dtype=uint8` and
        `shape=(height, width, 3)`.

        :param source: Compatible source types are :attr:`~eagerx.core.specs.NodeSpec.outputs` and
                       :attr:`~eagerx.core.specs.ObjectSpec.sensors`.
        :param rate: The rate (Hz) at which to render the images.
        :param processor: Processes the received message before passing it
                          to the target node's :func:`~eagerx.core.entities.Node.callback`.
        :param window: A non-negative number that specifies the number of messages to pass to the node's :func:`~eagerx.core.entities.Node.callback`.

                       - *window* = 1: Only the last received input message.

                       - *window* = *x* > 1: The trailing last *x* received input messages.

                       - *window* = 0: All input messages received since the last call to the node's :func:`~eagerx.core.entities.Node.callback`.

                       .. note:: With *window* = 0, the number of input messages may vary and can even be zero.

        :param delay: A non-negative simulated delay (seconds). This delay is ignored if
                      :attr:`~eagerx.core.entities.Engine.simulate_delays` = True
                      in the engine's :func:`~eagerx.core.entities.Engine.spec`.
        :param skip: Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.Node.callback`.
                     May be necessary to ensure that the connected graph is directed and acyclic.
        :param render_cls: The :attr:`~eagerx.core.entities.Node` of the render node.
                           By default, it uses the standard `RenderNode`. In Google colab, the `ColabRender` class is used.
        :param process: Process in which the render node is launched. See :class:`~eagerx.core.constants.process` for all
                        options.
        :param encoding: The encoding (`bgr` or `rgb`) of the render source.
        :param kwargs: Optional arguments required by the render node.
        """
        # Delete old render node from self._state['nodes'] if it exists
        if "env/render" in self._state["nodes"]:
            self.remove("env/render")

        # Add (new) render node to self._state['node']
        if render_cls is None:
            if bool(eval(os.environ.get("EAGERX_COLAB", "0"))):
                from eagerx.core.nodes import ColabRender as render_cls

                process = eagerx.process.ENVIRONMENT
            else:
                from eagerx.core.nodes import RenderNode as render_cls
        render = render_cls.make(rate=rate, process=process, encoding=encoding, **kwargs)
        # todo: How to change space of render.inputs.image when a processor is added.
        render.inputs.image.space = source.space
        self.add(render)

        # Create connection
        target = self.get_view("env/render", depth=["inputs", "image"])
        target.processor = processor
        self.connect(
            source=source,
            target=target,
            window=window,
            delay=delay,
            skip=skip,
        )

    def save(self, file: str) -> None:
        """Saves the graph state.

        The state is saved in *.yaml* format and contains the state of every added node, object, action, and observation
        and the connections between them.

        :param file: A string giving the name (and the file if the file isn't in the current working directory).
        """
        with open(file, "w") as outfile:
            yaml.dump(self._state, outfile, default_flow_style=False)

    @classmethod
    def load(cls, file: str):
        """Loads the graph state.

        The state is loaded in *.yaml* format and contains the state of every added node, object, action, and observation
        and the connections between them.

        :param file: A string giving the name (and the file if the file isn't in the current working directory).
        """
        must_raise = False
        with open(file, "r") as stream:
            try:
                state = yaml.safe_load(stream)
                if "engine_id" not in state:
                    warnings.warn("Old graph fromat. Adding `engine` key to state.")
                    state["engine_id"] = None
            except yaml.YAMLError as exc:
                must_raise = True
                print(exc)
        if must_raise:
            raise yaml.YAMLError(f"Could not load graph from `{file}`.")
        else:
            # Import all Objects & Nodes (to register & see availability).
            for key, params in state["nodes"].items():
                try:
                    load(params["config"]["entity_id"])
                except AttributeError as e:
                    warnings.warn(f"Could not load `{key}`: {e}")
            return cls(state)

    def reload(self):
        """Reloads (ie imports) all entities in the graph."""
        Graph._reload(self._state)

    @staticmethod
    def _reload(state: Dict):
        """Reloads (ie imports) all entities in the graph."""
        for key, params in state["nodes"].items():
            try:
                load(params["config"]["entity_id"])
            except AttributeError as e:
                raise ImportError(f"Could not load `{key}`: {e}")

    def gui(
        self, interactive: Optional[bool] = True, resolution: Optional[List[int]] = None, filename: Optional[str] = None
    ) -> Union[None, np.ndarray]:
        """Opens a graphical user interface of the graph.

        .. note:: Requires `eagerx-gui`:

                .. highlight:: python
                .. code-block:: python

                    pip3 install eagerx-gui

        :param interactive: If `True`, an interactive application is launched.
                            Otherwise, an RGB render of the GUI is returned.
                            This could be useful when using a headless machine.
        :param resolution: Specifies the resolution of the returned render when `interactive` is `False`.
                           If `interactive` is `True`, this argument is ignored.
        :param filename: If provided, the GUI is rendered to an svg file with this name.
                         If `interactive` is `True`, this argument is ignored.
        :return: RGB render of the GUI if `interactive` is `False`.
        """
        # Make a copy of the state
        state = deepcopy(self._state)

        # Replace environment node with environment node
        Graph._substitute_environment_node(state)

        try:
            from eagerx_gui import launch_gui, render_gui
        except ImportError as e:
            log.logwarn(
                f"{e}. You will likely have to install eagerx-gui. It can be installed by running: pip install eagerx-gui"
            )
            return

        # Launch gui
        if interactive:
            # Only overwrite gui_state (node locations, etc...)
            # Note: `gui_state` may have info on nodes 'env/actions' and 'env/observations',
            #       while the graph state actually only includes node `environment`.
            self._state["gui_state"] = launch_gui(state)["gui_state"]
        else:
            return render_gui(state, resolution=resolution, filename=filename)

    @staticmethod
    def _is_selected(state: Dict, entry: SpecView):
        """Check if provided entry was selected in params."""
        name, component, cname = entry()
        assert name in state["nodes"], f'No entity with the anme "{name}" added to this graph.'
        params = state["nodes"][name]
        component = "outputs" if component == "feedthroughs" else component
        assert cname in params["config"][component], f'"{cname}" not selected in "{name}" under "config" in {component}.'

    @staticmethod
    def _is_compatible(state: Dict, source: SpecView, target: SpecView):
        """Check if provided the provided entries are compatible."""
        # Valid source and target components
        targets = ["inputs", "actuators", "feedthroughs", "targets"]
        sources = ["outputs", "states", "sensors"]

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
        Graph.check_inputs_have_address(state)
        Graph.check_graph_is_acyclic(state, plot=plot)
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
                            "produced/inferred. This likely means that it has not been connected. "
                            "Either deselect it, or connect it."
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
                            "but no address could be produced/inferred. This likely means that it has not been connected. "
                            "Either deselect it, or connect it."
                        )
        return True

    @staticmethod
    def check_graph_is_acyclic(state, plot=True):
        # Add nodes
        G = nx.MultiDiGraph()
        # label_mapping = {
        #     "env/render/done": "render",
        # }
        label_mapping = dict()
        for node, params in state["nodes"].items():
            default = params["config"]
            if "node_type" not in state["nodes"][node]:  # Object
                if "sensors" in default:
                    for cname in default["sensors"]:
                        name = "%s/sensors/%s" % (node, cname)
                        G.add_node(
                            name,
                            remain_active=False,
                            always_active=True,
                            is_stale=False,
                            has_tick=False,
                        )
                        if not ("actuators" in default and cname in default["actuators"]):
                            # label_mapping[name] = "%s/%s" % (node, cname)
                            label_mapping[name] = str(len(label_mapping))
                if "actuators" in default:
                    for cname in default["actuators"]:
                        name = "%s/actuators/%s" % (node, cname)
                        G.add_node(
                            name,
                            remain_active=True,
                            always_active=False,
                            is_stale=False,
                            has_tick=False,
                        )
                        if not ("sensors" in default and cname in default["sensors"]):
                            # label_mapping[name] = "%s/%s" % (node, cname)
                            label_mapping[name] = str(len(label_mapping))
            else:  # node
                for cname in default["outputs"]:
                    name = "%s/%s" % (node, cname)
                    label_mapping[name] = str(len(label_mapping))  # if node not in ["environment", "engine"] else name
                    G.add_node(
                        name,
                        remain_active=True if "tick" in default["inputs"] else False,
                        always_active=True if node == "engine" and cname == "tick" else False,
                        is_stale=False,
                        has_tick=False,
                    )

        # Add edges
        target_comps = ["inputs", "actuators", "feedthroughs"]
        source_comps = ["outputs", "sensors"]
        for source, target in state["connects"]:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            # Do not consider engine inputs for DAG check.
            if target_name == "engine":
                continue
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
        # G = nx.relabel_nodes(G, label_mapping)

        # Check if graph is acyclic (excluding 'skip' edges)
        H, cycles = episode_graph(G)
        is_dag = nx.is_directed_acyclic_graph(H)

        # Plot graphs
        if plot:
            fig_env, ax_env = plt.subplots(nrows=1, ncols=1)
            ax_env.set_title("Communication graph (episode)")
            # Remap action & observation labels to more readable form
            G_plot = nx.relabel_nodes(deepcopy(G), label_mapping)
            _, _, _, pos = plot_graph(G_plot, k=2, ax=ax_env)
            plt.show()

        # Assert if graph is a directed-acyclical graph (DAG)
        cycle_strs = ["Circular dependency detected: "]
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
            F_plot = nx.relabel_nodes(deepcopy(F), label_mapping)
            _, _, _, pos = plot_graph(F_plot, pos=pos, ax=ax_reset)
            plt.show()

        # Assert if reset graph is not stale
        has_real_reset = len([e for e, ft in nx.get_edge_attributes(G, "feedthrough").items() if ft]) > 0
        assert len(not_active) == 0 or not has_real_reset, (
            'Stale reset graph detected. Nodes "%s" will be stale, while they must be active (i.e. connected) in order for the graph to resolve (i.e. not deadlock).'
            % not_active
        )
        return True

    @staticmethod
    @supported_types(str, int, list, float, bool, dict, EntitySpec, SpecView, None)
    def _set(state, mapping):
        merge(state, mapping)

    @staticmethod
    def _get_view(state: Dict, name: str, depth: Optional[List[str]] = None):
        spec = Graph._get_spec(state, name)
        depth = depth if depth else []
        return SpecView(spec, depth=depth, name=name)

    def get_view(self, name: str, depth: Optional[List[str]] = None):
        return self._get_view(self._state, name, depth)

    @staticmethod
    def _check_spec(spec):
        if isinstance(spec, NodeSpec):
            from eagerx.core.entities import Node

            Node.check_spec(spec)
        elif isinstance(spec, ResetNodeSpec):
            from eagerx.core.entities import ResetNode

            ResetNode.check_spec(spec)
        elif isinstance(spec, ObjectSpec):
            from eagerx.core.entities import Object

            Object.check_spec(spec)

    def add_engine(self, engine: EngineSpec):
        Graph._add_engine(self._state, engine)

    @staticmethod
    def _add_engine(state, engine: EngineSpec):
        if state["engine_id"] is None:
            state["engine_id"] = engine.config.entity_id
            Graph._add(state, engine)
        else:
            raise ValueError("Cannot add engine. The graph already has an engine.")

    @staticmethod
    def _substitute_environment_node(state: Dict):
        """Substitute `environment` node with separate `env/actions` and `env/observations` node for improved visualization."""
        # Prepare environment spec
        environment = NodeSpec(state["nodes"]["environment"])

        # Add action & observation node.
        from eagerx.core.nodes import ActionsNode, ObservationsNode

        actions = ActionsNode.make()
        observations = ObservationsNode.make()
        Graph._add(state, [actions, observations])

        # Copy outputs to actions
        actions.config.outputs = environment.config.outputs
        with actions.outputs as a:
            a.update(environment.outputs.to_dict())

        # Copy inputs to observations
        observations.config.inputs = environment.config.inputs
        with observations.inputs as o:
            o.update(environment.inputs.to_dict())

        # Replace connections
        for s, t in state["connects"]:
            if s[:2] == ["environment", "outputs"]:
                s[0] = "env/actions"
            elif t[:2] == ["environment", "inputs"]:
                t[0] = "env/observations"

        # Remove environment node
        state["nodes"].pop("environment")
        state["backup"].pop("environment")

    @staticmethod
    def _substitute_all_objects(state: Dict):
        """state must already have an engine (ie _add_engine must be called first)."""
        assert state["engine_id"] is not None, "The graph state must have an engine, before objects can be substituted."
        for name in list(state["nodes"].keys()):
            # Skip if node
            if "node_type" in state["nodes"][name]:
                continue
            # Substitute object graph
            Graph._substitute_object(state, name)

    @staticmethod
    def _substitute_object(state: Dict, name: str):
        """state must already have an engine (ie _add_engine must be called first)."""
        assert state["engine_id"] is not None, "The graph state must have an engine, before objects can be substituted."

        # Get graph
        graph = Graph(state)

        # Get engine spec
        engine = EngineSpec(state["nodes"]["engine"])

        # Get object parameters
        params = state["nodes"][name]
        spec = ObjectSpec(params)

        # Make sure to register object by reimporting the object.
        # NOTE: This will only register all engine implementations that are defined within the Object's class definition.
        #       You may need to manually reload separate engine implementations when launching environments in subprocesses.
        load(spec.config.entity_id)

        # Register object's engine graph
        engine_graph = engine._register_object(spec)
        nodes, actuators, sensors, connects = engine_graph.register()

        # Construct context
        context = {"ns": {"obj_name": name}}
        substitute_args(params["config"], context, only=["ns"])  # First resolve args within the context
        substitute_args(params, context, only=["ns"])  # Resolve rest of params

        # Resolve namespaces
        substitute_args(nodes, context, only=["ns"])  # Resolve rest of params
        substitute_args(actuators, context, only=["ns"])  # Resolve rest of params
        substitute_args(sensors, context, only=["ns"])  # Resolve rest of params
        substitute_args(connects, context, only=["ns"])  # Resolve rest of params

        # Get agnostic & engine-specific definition
        agnostic = dict(actuators=params["actuators"], sensors=params["sensors"], states=params["states"])
        specific = dict(actuators=actuators, sensors=sensors)

        # Replace node names
        for key in list(nodes.keys()):
            key_sub = substitute_args(key, context, only=["ns"])
            nodes[key_sub] = nodes.pop(key)

        # Sensors & actuators
        dependencies = []
        for obj_comp in ["actuators", "sensors"]:
            for obj_cname in params["config"][obj_comp]:
                try:
                    entry_lst = specific[obj_comp][obj_cname]
                except KeyError:
                    raise KeyError(
                        f'"{obj_cname}" was selected in {obj_comp} of "{name}", but there is no implementation for it in engine "{state["engine_id"]}".'
                    )

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
                        # Make sure that space of node is contained within agnostic space.
                        agnostic_space = Space.from_dict(obj_comp_params["space"])
                        node_space = Space.from_dict(node_comp_params["space"])
                        msg = (
                            f"The space of EngineNode `{node_name}.{node_comp}.{node_cname}` is different "
                            f"(dtype, shape, low, or high) from the space of `{name}.{obj_comp}.{obj_cname}`: \n\n"
                            f"{node_name}.{node_comp}.{node_cname}.space={node_space} \n\n"
                            f"{name}.{obj_comp}.{obj_cname}.space={agnostic_space} \n\n"
                        )
                        assert agnostic_space.contains_space(node_space), msg
                        node_comp_params.update(obj_comp_params)
                        # node_comp_params["address"] = f"{name}/{obj_comp}/{obj_cname}" # todo: set to None?
                        # sensor_addresses[f"{node_name}/{node_comp}/{node_cname}"] = f"{name}/{obj_comp}/{obj_cname}"
                    else:  # Actuators
                        agnostic_space = Space.from_dict(obj_comp_params["space"])
                        node_space = Space.from_dict(node_comp_params["space"])
                        msg = (
                            f"The space of EngineNode `{node_name}.{node_comp}.{node_cname}` is different "
                            f"(dtype, shape, low, or high) from the space of `{name}.{obj_comp}.{obj_cname}`: \n\n"
                            f"{name}.{node_comp}.{node_cname}.space={node_space} \n\n"
                            f"{name}.{obj_comp}.{obj_cname}.space={agnostic_space} \n\n"
                        )
                        assert agnostic_space.contains_space(node_space), msg

                        agnostic_processor = obj_comp_params.pop("processor")
                        node_comp_params.update(obj_comp_params)

                        if agnostic_processor is not None:
                            msg = (
                                f"A processor was defined for {node_name}.{node_comp}.{node_cname}, however the engine "
                                "implementation also has a processor defined. You can only have one processor."
                            )
                            assert node_comp_params["processor"] is None, msg
                            node_comp_params["processor"] = agnostic_processor

                        # Pop rate. Actuators are more-or-less inputs so have no rate?
                        node_comp_params.pop("rate")
                        # Reassign converter in case a node provides the implementation for multiple actuators
                        obj_comp_params["processor"] = agnostic_processor

        # Get set of node we are required to launch
        dependencies = list(set(dependencies))

        # Verify that no dependency is an unlisted actuator node.
        not_selected = [cname for cname in agnostic["actuators"] if cname not in params["config"]["actuators"]]
        for cname in not_selected:
            try:
                for entry in specific["actuators"][cname]:
                    node_name, node_comp, node_cname = (entry["name"], entry["component"], entry["cname"])
                    msg = (
                        f'There appears to be a dependency on enginenode "{node_name}" for the implementation of '
                        f'engine "{state["engine_id"]}" for object "{name}" to work. However, enginenode "{node_name}" is '
                        f'directly tied to an unselected actuator "{cname}". '
                        "The actuator must be selected to resolve the graph."
                    )
                    assert node_name not in dependencies, msg
            except KeyError:
                # We pass here, because if cname is not selected, but also not implemented,
                # we are sure that there is no dependency.
                pass

        # Add relevant nodes
        nodes = {name: NodeSpec(params) for name, params in nodes.items() if name in dependencies}
        engine.objects[name].nodes = list(nodes.keys())
        graph.add([n for _, n in nodes.items()])

        # Connect tick to engine
        for node_name, n in nodes.items():
            if "tick" not in n.config.inputs:
                continue
            graph.connect(source=engine.outputs.tick, target=n.inputs.tick)
            for cname, o in n.outputs.items():
                mapping = dict(
                    delay=0.0,
                    window=0,
                    skip=False,
                    processor=None,
                    space=o.space.to_dict(),
                    address=None,
                )
                cname = f"{node_name}/{cname}"
                engine.config.inputs.append(cname)
                with engine.inputs as i:
                    i[cname] = mapping
                graph.connect(source=o, target=engine.inputs[cname], skip=True)
                break  # We only need 1 output per node.

        # Interconnect nodes
        for source, target in connects:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            if not (source_name in dependencies or target_name in dependencies):
                continue  # Continue if both source and target are not included as dependencies
            # If both source and target are included as dependencies
            if source_name in dependencies and target_name in dependencies:
                state["connects"].append([source, target])

        # Replace agnostic connections
        for s, t in deepcopy(state["connects"]):
            s_name, s_comp, s_cname = s
            t_name, t_comp, t_cname = t
            if name not in [s_name, t_name]:
                continue
            if t_comp == "actuators":
                assert t_cname in actuators, f"Actuator `{s_cname}` of object `{name}` has no implementation in this engine."
                s = graph.get_view(s[0], s[1:])
                t = graph.get_view(t[0], t[1:])
                window, delay, skip = t.window, t.delay, t.skip
                # Disconnect "agnostic" connection
                if s_name == "environment":
                    action = s_cname
                    s = None
                else:
                    action = None
                    s = s
                graph.disconnect(source=s, target=t, action=action)
                # Connect node with engine node
                for p in actuators[t_cname]:
                    _name, _comp, _cname = p["name"], p["component"], p["cname"]
                    new_target = getattr(nodes[_name], _comp)[_cname]
                    graph.connect(source=s, target=new_target, action=action, window=window, delay=delay, skip=skip)
            elif s_comp == "sensors":
                assert s_cname in sensors, f"Sensor `{s_cname}` of object `{name}` has no implementation in this engine."
                s = graph.get_view(s[0], s[1:])
                t = graph.get_view(t[0], t[1:])
                window, delay, skip = t.window, t.delay, t.skip
                # Disconnect "agnostic" connection
                if t_name == "environment":
                    observation = t_cname
                    t = None
                else:
                    observation = None
                    t = t
                graph.disconnect(source=s, target=t, observation=observation)
                # Connect node with engine node
                for p in sensors[s_cname]:
                    _name, _comp, _cname = p["name"], p["component"], p["cname"]
                    new_source = getattr(nodes[_name], _comp)[_cname]
                    graph.connect(source=new_source, target=t, observation=observation, window=window, delay=delay, skip=skip)
            elif s_comp == "states":
                s = graph.get_view(s[0], s[1:])
                t = graph.get_view(t[0], t[1:])
                graph.disconnect(source=s, target=t)
                t.address = "%s/%s/%s" % s()

        # Remove disconnected object from graph
        d = graph.remove(name)
        assert len(d) == 0, f"Object {name} not fully disconnected."
