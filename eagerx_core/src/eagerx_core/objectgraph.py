import yaml
from tabulate import tabulate
from copy import deepcopy
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Union, Dict, Tuple, Optional, Any
from yaml import dump
yaml.Dumper.ignore_aliases = lambda *args: True  # todo: check if needed.
import rospy

from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string, get_attribute_from_module, substitute_args, msg_type_error
from eagerx_core.utils.network_utils import reset_graph, episode_graph, plot_graph, color_nodes, color_edges, is_stale
# from eagerx_core.entities import BaseConverter, SpaceConverter


class ObjectGraph:
    def __init__(self, state: Dict):
        self._state = state

    @classmethod
    def create(cls, actuators: Optional[List[Dict]] = None, sensors: Optional[List[Dict]] = None, states: Optional[List[Dict]] = None, nodes: Optional[List] = None):
        from eagerx_core.specs import SimNodeSpec
        if nodes is None:
            nodes = []
        if isinstance(nodes, SimNodeSpec):
            nodes = [nodes]

        from eagerx_core.entities import SimNode
        from eagerx_core.specs import ConverterSpec

        # Create actuator node
        outputs = []
        spec = SimNode.pre_make(None)
        spec.set_parameter('name', 'actuators')
        nodes.append(spec)
        for cname, params in actuators.items():
            spec.add_output(cname, msg_type=params['msg_type'], converter=ConverterSpec(params['converter']), space_converter=ConverterSpec(params['space_converter']))
            outputs.append(cname)
        spec.set_parameter('outputs', outputs)

        # Create sensor node
        inputs = []
        spec = SimNode.pre_make(None)
        spec.set_parameter('name', 'sensors')
        nodes.append(spec)
        for cname, params in sensors.items():
            spec.add_input(cname, msg_type=params['msg_type'], converter=ConverterSpec(params['converter']), space_converter=ConverterSpec(params['space_converter']))
            inputs.append(cname)
        spec.set_parameter('inputs', inputs)

        # Create state node
        simstates = []
        spec = SimNode.pre_make(None)
        spec.set_parameter('name', 'states')
        nodes.append(spec)
        for cname, params in states.items():
            spec.add_state(cname, msg_type=params['msg_type'], space_converter=ConverterSpec(params['space_converter']))
            simstates.append(cname)
        spec.set_parameter('states', simstates)

        # Create a state
        state = dict(nodes=dict(), connects=list())
        cls._add(state, nodes)
        return cls(state)

    def add(self, nodes: Union[Any, List]):
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
            name = node.get_parameter('name')
            assert name not in state['nodes'], 'There is already a node or object registered in this graph with name "%s".' % name

            # Add node to state
            state['nodes'][name] = dict()
            state['nodes'][name]['params'] = node._params
            state['nodes'][name]['default'] = node.params

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
            for source, target in deepcopy(self._state['connects']):
                if name in [source[0], target[0]]:
                    if source[0] == 'actuators':
                        actuator = source[2]
                        source = None
                    else:
                        actuator = None
                        source = source
                    if target[0] == 'sensors':
                        sensor = target[2]
                        target = None
                    else:
                        sensor = None
                        target = target
                    self.disconnect(source, target, actuator, sensor)
            self._state['nodes'].pop(name)

    def _remove(self, names: Union[str, List[str]]):
        """
        First removes all associated connects from self._state.
        Then, removes node/object from self._state.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            self._exist(self._state, name)
            for source, target in deepcopy(self._state['connects']):
                if name in [source[0], target[0]]:
                    if source[0] == 'actuators':
                        actuator = source[2]
                        source = None
                    else:
                        actuator = None
                        source = source
                    if target[0] == 'sensors':
                        sensor = target[2]
                        target = None
                    else:
                        sensor = None
                        target = target
                    self._disconnect(source, target, actuator, sensor)
            self._state['nodes'].pop(name)

    def add_component(self, name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        """
        Adds a component entry to the selection list.
        """
        # assert only action, only observation, only name, component, cname
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name, component, cname = ('actuators', 'outputs', actuator)
        if sensor:
            name, component, cname = ('sensors', 'inputs', sensor)
        self._add_component(name, component, cname)

        # if (name is not None) and (component is not None) and (cname is not None):  # component parameter
        self._add_component(name, component, cname)
        # if action:
        #     self._add_action(action)
        # if observation:
        #     self._add_observation(observation)

    def _add_component(self, name: str, component: str, cname: str):
        """
        Adds a component entry to the selection list.
        """
        # Check that cname exists
        self._exist(self._state, name, component=component, cname=cname)

        # Add cname to selection list if it is not already selected
        params = self._state['nodes'][name]['params']
        assert cname not in params['default'][component], '"%s" already selected in "%s" under %s.' % (cname, name, component)
        params['default'][component].append(cname)

    # def _add_action(self, action: str):
    #     """
    #     Adds disconnected action entry to 'env/actions' node in self._state.
    #     """
    #     assert action != 'set', 'Cannot define an action with the reserved name "set".'
    #     params_action = self._state['nodes']['env/actions']['params']
    #     if action not in params_action['outputs']:  # Action already registered
    #         params_action['outputs'][action] = dict()
    #         self._add_component('env/actions', 'outputs', action)
    #
    # def _add_observation(self, observation: str):
    #     """
    #     Adds disconnected observation entry to 'env/observations' node in self._state.
    #     """
    #     assert observation != 'actions_set', 'Cannot define an observations with the reserved name "actions_set".'
    #     params_obs = self._state['nodes']['env/observations']['params']
    #     if observation in params_obs['inputs']:
    #         assert len(params_obs['inputs'][observation]) == 0, 'Observation "%s" already exists and is connected.' % observation
    #     else:
    #         params_obs['inputs'][observation] = dict()
    #         self._add_component('env/observations', 'inputs', observation)

    def remove_component(self, name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        """
        Removes a component entry from the selection list. It will first disconnect all connections in connect.
        """
        # assert only action, only observation, only name, component, cname
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name, component, cname = ('actuators', 'outputs', actuator)
        if sensor:
            name, component, cname = ('sensors', 'inputs', sensor)
        self._remove_component(name, component, cname)

        # if (name is not None) and (component is not None) and (cname is not None):  # component parameter
        #     self._remove_component(name, component, cname)
        # if action:
        #     self._remove_action(action)
        # if observation:
        #     self._remove_observation(observation)

    def _remove_component(self, name: str, component: str, cname: str):
        """
        Removes a component entry from the selection list. It will first disconnect all connections in connect.
        """
        self._is_selected(self._state, name, component, cname)

        # Disconnect component entry
        self._disconnect_component(name, component, cname)

        # Remove cname from selection list
        params = self._state['nodes'][name]['params']
        params['default'][component].remove(cname)

    # def _remove_action(self, action: str):
    #     """
    #     Method to remove an action. It will first disconnect all connections in connect.
    #     """
    #     self._remove_component('env/actions', 'outputs', action, remove=False)
    #     params_action = self._state['nodes']['env/actions']['params']
    #     source = ['env/actions', 'outputs', action]
    #     connect_exists = False
    #     for idx, c in enumerate(self._state['connects']):
    #         if source == c[0]:
    #             connect_exists = True
    #             target = c[1]
    #             break
    #     assert not connect_exists, 'Action entry "%s" cannot be removed, because it is not disconnected. Connection with target %s still exists.' % (action, target)
    #     params_action['outputs'].pop(action)
    #
    # def _remove_observation(self, observation: str):
    #     """
    #     Method to remove an observation. It will first disconnect all connections in connect.
    #     """
    #     self._remove_component('env/observations', 'inputs', observation, remove=False)
    #     params_obs = self._state['nodes']['env/observations']['params']
    #     target = ['env/observations', 'inputs', observation]
    #     connect_exists = False
    #     for idx, c in enumerate(self._state['connects']):
    #         if target == c[1]:
    #             connect_exists = True
    #             source = c[0]
    #             break
    #     assert not connect_exists, 'Observation entry "%s" cannot be removed, because it is not disconnected. Connection with source %s still exists.' % (observation, source)
    #     # assert observation in params_obs['inputs'], 'Observation "%s" cannot be removed, because it does not exist.' % observation
    #     params_obs['inputs'].pop(observation)

    def connect(self,
                source: Optional[Tuple[str, str, str]] = None,
                target: Optional[Tuple[str, str, str]] = None,
                actuator: str = None, sensor: str = None, address: str = None,
                converter: Optional[Dict] = None,
                window: Optional[int] = None,
                delay: Optional[float] = None,
                skip: Optional[bool] = None,
                external_rate: Optional[float] = None):
        assert not address or (source is None and actuator is None and sensor is None), f'You cannot provide an external address "{address}" together with a sensor, actuator, or source.'
        assert not source or not actuator, f'You cannot specify an actuator if you wish to connect actuator "{actuator}", as the actuator will act as the source.'
        assert not target or not sensor, f'You cannot specify a target if you wish to connect sensor "{sensor}", as the sensor will act as the target.'
        assert not (actuator and sensor), f'You cannot connect an actuator directly to a sensor.'
        if address:
            self.set_parameter('address', address, *target)
            assert external_rate is not None, f'When providing an external address "{address}", an external rate must also be provided.'
            assert external_rate > 0, f'Invalid external rate "{external_rate}". External rate must be > 0.'
            return
        else:
            assert external_rate is None, f'An external rate may only be provided in combination with an address.'

        from eagerx_core.specs import ConverterSpec
        if isinstance(converter, ConverterSpec):
            converter = converter.params

        if actuator:  # source = actuator
            assert converter is None, f'Cannot specify an input converter when connecting actuator "{actuator}".'
            assert window is None, f'Cannot specify a window when connecting actuator "{actuator}".'
            assert delay is None, f'Cannot specify a delay when connecting actuator "{actuator}".'
            assert skip is None, f'Cannot specify a skip when connecting actuator "{actuator}".'
            source = ('actuators', 'outputs', actuator)
        elif sensor:  # target = sensor
            assert converter is None, f'Cannot specify an input converter when connecting sensor "{sensor}".'
            assert window is None, f'Cannot specify a window when connecting sensor "{sensor}".'
            assert delay is None, f'Cannot specify a delay when connecting sensor "{sensor}".'
            assert skip is None, f'Cannot specify a skip when connecting sensor "{sensor}".'
            target = ('sensors', 'inputs', sensor)
        self._connect(source, target, converter, window, delay, skip)

    def _connect(self,
                source: Optional[Tuple[str, str, str]] = None,
                target: Optional[Tuple[str, str, str]] = None,
                converter: Optional[Dict] = None,
                window: Optional[int] = None,
                delay: Optional[float] = None,
                skip: Optional[bool] = None):
        """
        Method to connect a source to a target. For actuators/sensors, first a (new) disconnected entry must be created,
        after which an additional call to connect_actuator/sensor is required before calling this method.
        For more info, see self.connect.
        """
        from eagerx_core.specs import ConverterSpec
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

        # Add properties to target params
        if converter is not None:
            self.set_parameter('converter', converter, target_name, target_comp, target_cname)
        if window is not None:
            self.set_parameter('window', window, target_name, target_comp, target_cname)
        if delay is not None:
            self.set_parameter('delay', delay, target_name, target_comp, target_cname)
        if skip is not None:
            self.set_parameter('skip', skip, target_name, target_comp, target_cname)

        # Add connection
        connect = [source, target]
        ObjectGraph.check_msg_type(source, target, self._state)
        self._state['connects'].append(connect)

    # def _connect_actuator(self, actuator, target, converter=None):
    #     """
    #     Method to connect a (previously added) actuator, that *precedes* self._connect(source, target).
    #     """
    #     params_actuator = self._state['nodes']['actuators']['params']
    #     assert actuator in params_actuator['outputs'], f'Actuator "{actuator}" must be added, before you can connect it.'
    #     name, component, cname = target
    #     params_target = self._state['nodes'][name]['params']
    #
    #     assert 'space_converter' in params_target[component][cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".' % (cname, component, name)
    #
    #     # Infer source properties (converter & msg_type) from target
    #     space_converter = params_target[component][cname]['space_converter']
    #     msg_type_C = get_cls_from_string(params_target[component][cname]['msg_type'])
    #     if converter:  # Overwrite msg_type_B if input converter specified
    #         msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
    #     else:
    #         msg_type_B = msg_type_C
    #
    #     # Set properties in node params of 'env/actions'
    #     if len(params_actuator['outputs'][action]) > 0:  # Action already registered
    #         space_converter_state = params_actuator['outputs'][action]['converter']
    #         msg_type_B_state = get_opposite_msg_cls(params_actuator['outputs'][action]['msg_type'], space_converter_state)
    #         assert msg_type_B == msg_type_B_state, 'Conflicting %s for action "%s" that is already used in another connection. Occurs with connection %s' % ('msg_types', action, tuple([name, component, cname]))
    #         if not space_converter == space_converter_state:
    #             rospy.logwarn('Conflicting %s for action "%s". Not using the space_converter of %s[%s][%s]' % ('space_converters', action, name, component, cname))
    #         msg_type_A = get_opposite_msg_cls(msg_type_B_state, space_converter_state)
    #     else:
    #         # Verify that converter is not modifying the msg_type (i.e. it is a processor).
    #         assert msg_type_B == msg_type_C, 'Cannot have a converter that maps to a different msg_type as the converted msg_type will not be compatible with the space_converter specified in the .yaml.'
    #         msg_type_A = get_opposite_msg_cls(msg_type_B, space_converter)
    #         params_actuator['outputs'][action]['msg_type'] = get_module_type_string(msg_type_A)
    #         params_actuator['outputs'][action]['converter'] = space_converter
    #         add_default_args(params_actuator['outputs'][action], component='outputs')
    #
    # def _connect_sensor(self, source, observation, converter):
    #     """
    #     Method to connect a (previously added & disconnected) sensor, that *precedes* self._connect(source, target).
    #     """
    #     params_obs = self._state['nodes']['env/observations']['params']
    #     assert observation in params_obs['inputs'], 'Observation "%s" must be added, before you can connect it.' % observation
    #     name, component, cname = source
    #     params_source = self._state['nodes'][name]['params']
    #
    #     assert converter is not None or 'space_converter' in params_source[component][cname], '"%s" does not have a space_converter defined under %s in the .yaml of "%s". Either specify it there, or add an input converter that acts as a space_converter to this connection.' % (cname, component, name)
    #
    #     # Infer target properties (converter & msg_type) from source
    #     msg_type_A = get_cls_from_string(params_source[component][cname]['msg_type'])
    #     output_converter = params_source[component][cname]['converter']
    #     msg_type_B = get_opposite_msg_cls(msg_type_A, output_converter)
    #     if converter is None:
    #         converter = params_source[component][cname]['space_converter']
    #     msg_type_C = get_opposite_msg_cls(msg_type_B, converter)
    #
    #     # Set properties in node params of 'env/observations'
    #     assert len(params_obs['inputs'][observation]) == 0, 'Observation "%s" already connected.' % observation
    #     params_obs['inputs'][observation]['msg_type'] = get_module_type_string(msg_type_C)
    #     add_default_args(params_obs['inputs'][observation], component='inputs')
    #     return converter

    def disconnect(self,
                   source: Optional[Tuple[str, str, str]] = None,
                   target: Optional[Tuple[str, str, str]] = None,
                   actuator: str = None, sensor: str = None):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        self._disconnect(source, target, actuator, sensor)

    def _disconnect(self,
                   source: Optional[Tuple[str, str, str]] = None,
                   target: Optional[Tuple[str, str, str]] = None,
                   actuator: str = None, sensor: str = None):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        assert not source or not actuator, f'You cannot specify a source if you wish to disconnect actuator "{actuator}", as the actuator will act as the source.'
        assert not target or not sensor, f'You cannot specify a target if you wish to disconnect sensor "{sensor}", as the sensor will act as the target.'
        assert not (sensor and actuator), f'You cannot disconnect an actuator from an sensor, as such a connection cannot exist.'
        if isinstance(source, tuple):
            source = list(source)
        if isinstance(target, tuple):
            target = list(target)

        # Create source & target entries
        if actuator:
            source = ['actuators', 'outputs', actuator]
        if sensor:
            target = ['sensors', 'inputs', sensor]

        # Check if connection exists
        self._is_selected(self._state, *target)

        # Check if connection exists
        if source is None:
            assert self.get_parameter('address', *target) is not None, f"Cannot disconnect. No source was provided, and the target={target} also does not have an address specified."
            self.set_parameter('address', None, *target)
            self.set_parameter('external_rate', False, *target)
        else:
            self._is_selected(self._state, *source)
            connect_exists = False
            idx_connect = None
            for idx, c in enumerate(self._state['connects']):
                if source == c[0] and target == c[1]:
                    connect_exists = True
                    idx_connect = idx
                    break
            assert connect_exists, f'The connection with source={source} and target={target} cannot be removed, because it does not exist.'

            # Pop the connection from the state
            self._state['connects'].pop(idx_connect)

            # Reset source params to disconnected state
            if actuator:
                pass
                # self._disconnect_action(action)
            else:
                # Nothing to do here (for now)
                source_name, source_comp, source_cname = source
                source_params = self._state['nodes'][source_name]['params']

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/skip/converter.
        if sensor:
            pass
            # self._disconnect_observation(observation)
        else:
            target_name, target_comp, target_cname = target
            target_params = self._state['nodes'][target_name]['params']
            target_params[target_comp][target_cname] = self._state['nodes'][target_name]['default'][target_comp][target_cname]

    def _disconnect_component(self, name: str, component: str, cname: str):
        """
        Disconnects all associated connects from self._state.
        """
        was_connected = False
        for source, target in deepcopy(self._state['connects']):
            self._is_selected(self._state, *source)
            self._is_selected(self._state, *target)
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            if source_name == 'actuators':
                actuator = source_cname
                source = None
            else:
                actuator = None
                source = source
            if target_name == 'sensors':
                sensor = target_cname
                target = None
            else:
                sensor = None
                target = target
            if name == source_name and component == source_comp and cname == source_cname:
                self.disconnect(source, target, actuator, sensor)
                was_connected = True
        return was_connected

    # def _disconnect_action(self, action: str):
    #     """
    #     Returns the action entry back to its disconnected state.
    #     That is, remove space_converter if it is not connected to any other targets.
    #     """
    #     params_action = self._state['nodes']['env/actions']['params']
    #     assert action in params_action['outputs'], 'Cannot disconnect action "%s", as it does not exist.' % action
    #     source = ['env/actions', 'outputs', action]
    #     connect_exists = False
    #     for idx, c in enumerate(self._state['connects']):
    #         if source == c[0]:
    #             connect_exists = True
    #             break
    #     if not connect_exists:
    #         params_action['outputs'][action] = dict()
    #
    # def _disconnect_observation(self, observation: str):
    #     """
    #     Returns the observation entry back to its disconnected state (i.e. empty dict).
    #     """
    #     params_obs = self._state['nodes']['env/observations']['params']
    #     assert observation in params_obs[
    #         'inputs'], 'Cannot disconnect observation "%s", as it does not exist.' % observation
    #     params_obs['inputs'][observation] = dict()

    def rename(self, old, new, name: Optional[str] = None, component: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        """
        Renames the node/object, or action/observation if specified.
        """
        self._correct_signature(name=name, component=component, sensor=sensor, actuator=actuator)
        if actuator:
            name = 'actuators'
            component = 'outputs'
        if sensor:
            name = 'sensors'
            component = 'inputs'
        if (name is not None) and (component is not None):  # component renaming
            self._rename_component(name, component, old_cname=old, new_cname=new)
        elif (name is None) and (component is None):  # node/object renaming
            self._rename_entity(old_name=old, new_name=new)
        else:
            raise ValueError('Either the arguments {name, component} are None, or they must both be specified.')

    def _rename_component(self, name: str, component: str, old_cname: str, new_cname: str):
        """
        Renames the component name (cname) of an entity (node/object) in _state['nodes'] and self._state[connects].
        We cannot change names for node/object components, because their python implementation could depend on it.
        """
        self._exist(self._state, name, component=component, cname=old_cname)
        default = self._state['nodes'][name]['default']
        params = self._state['nodes'][name]['params']

        # For now, we only support changing action/observation cnames
        assert name in ['sensors', 'actuators'], f'Cannot change "{old_cname}" of "{name}". Only name changes to observations and actions are supported.'
        assert new_cname not in params[component], f'"{new_cname}" already defined in "{name}" under {component}.'

        # Rename cname in params
        for d in (params, default):
            if component in d and old_cname in d[component]:
                assert new_cname not in d[component], f'"{new_cname}" already defined in "{name}" under {component}.'
                d[component][new_cname] = d[component].pop(old_cname)
            if component in d['default'] and old_cname in d['default'][component]:
                assert new_cname not in d['default'][component], f'"{new_cname}" already defined in "{name}" under {component}.'
                d['default'][component].remove(old_cname)
                d['default'][component].append(new_cname)

        # Rename cname in all connects
        for source, target in self._state['connects']:
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
        assert old_name not in ['sensors', 'actuators'], f'Node name "{old_name}" is fixed and cannot be changed.'
        assert new_name not in self._state['nodes'], f'There is already a node or object registered in this graph with name "{new_name}".'

        # Rename entity in params
        self._state['nodes'][new_name] = self._state['nodes'].pop(old_name)
        self._state['nodes'][new_name]['default']['default']['name'] = new_name
        self._state['nodes'][new_name]['params']['default']['name'] = new_name

        # Rename in all connects
        for source, target in self._state['connects']:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target

            if source_name == old_name:
                source[0] = new_name
            if target_name == old_name:
                target[0] = new_name

    def set_parameter(self, parameter: str, value: Any, name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        """
        A wrapper to set a single parameter. See set_parameters for more info.
        """
        return self.set_parameters({parameter: value}, name=name, component=component, cname=cname, actuator=actuator, sensor=sensor)

    def set_parameters(self, mapping: Dict[str, Any], name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        """
        Sets parameters in self._state, based on the node/object name. If a component and cname are specified, the
        parameter will be set there. Else, the parameter is set under the "default" key.
        For objects, parameters are set under their agnostic definitions of the components (so not bridge specific).
        If a converter is added, we check if the msg_type changes with the new converter. If so, the component is
        disconnected. See _set_converter for more info.
        """
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name = 'actuators'
            component = 'outputs'
            cname = actuator
        if sensor:
            name = 'sensors'
            component = 'inputs'
            cname = sensor
        self._exist(self._state, name, component=component, cname=cname)

        if (component is not None) and (cname is not None):  # component parameter
            for parameter, value in mapping.items():
                self._exist(self._state, name, component=component, cname=cname, parameter=parameter)
                if parameter == 'converter':
                    from eagerx_core.specs import ConverterSpec
                    if isinstance(value, ConverterSpec):
                        value = value.params
                    self._set_converter(name, component, cname, value)
                else:
                    self._state['nodes'][name]['params'][component][cname][parameter] = value
        else:  # Default parameter
            for parameter, value in mapping.items():
                self._exist(self._state, name, component=component, cname=cname, parameter=parameter)
                assert parameter not in ['sensors', 'actuators', 'targets', 'states', 'inputs', 'outputs'], 'You cannot modify component parameters with this function. Use _add/remove_component(..) instead.'
                assert parameter not in ['name'], 'You cannot rename with this function. Use rename_(name) instead.'
                default = self._state['nodes'][name]['params']['default']
                default[parameter] = value

    def _set_converter(self, name: str, component: str, cname: str, converter: Dict):
        """
        Replaces the converter specified for a node's/object's I/O.
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
        """
        self._exist(self._state, name, component=component, cname=cname, parameter='converter')
        params = self._state['nodes'][name]['params']

        # Check if converted msg_type of old converter is equal to the msg_type of newly specified converter
        msg_type = get_cls_from_string(params[component][cname]['msg_type'])
        converter_old = params[component][cname]['converter']
        msg_type_ros_old = get_opposite_msg_cls(msg_type, converter_old)
        msg_type_ros_new = get_opposite_msg_cls(msg_type, converter)
        if not msg_type_ros_new == msg_type_ros_old:
            was_disconnected = self._disconnect_component(name, component, cname)
        else:
            was_disconnected = False

        # If disconnected action/observation, we cannot add converter so raise error.
        assert not (was_disconnected and name in ['actuators', 'sensors']), f'Cannot change the converter of actuator/sensor "{cname}", as it changes the msg_type from "{msg_type_ros_old}" to "{msg_type_ros_new}"'

        # Replace converter
        params[component][cname]['converter'] = converter

    def get_parameter(self, parameter: str, name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None, default=None):
        """
        Get node/object parameters. If component and cname are specified, get the parameter of them instead.
        If default was specified, get default parameter instead. Else, raise an error.
        """
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name = 'actuators'
            component = 'outputs'
            cname = actuator
        if sensor:
            name = 'sensors'
            component = 'inputs'
            cname = sensor
        try:
            self._exist(self._state, name, component, cname, parameter=parameter)
            if (component is not None) and (cname is not None):  # component parameter
                return self._state['nodes'][name]['params'][component][cname][parameter]
            else:  # default parameter
                return self._state['nodes'][name]['params']['default'][parameter]
        except AssertionError:
            if default:
                return default
            else:
                raise

    def get_parameters(self, name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        """
        Get all node/object parameters. If component and cname are specified, get the parameters of them instead.
        """
        self._correct_signature(name, component, cname, actuator, sensor)
        if actuator:
            name = 'actuators'
            component = 'outputs'
            cname = actuator
        if sensor:
            name = 'sensors'
            component = 'inputs'
            cname = sensor
        self._exist(self._state, name, component, cname)
        if (component is not None) and (cname is not None):  # component parameter
            return deepcopy(self._state['nodes'][name]['params'][component][cname])
        else:  # default parameter
            return deepcopy(self._state['nodes'][name]['params']['default'])

    def _reset_converter(self, name: str, component: str, cname: str):
        """
        Replaces the converter specified for a node's/object's I/O defined in self._state[name]['default'].
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remove action entries if they are disconnect and the last connection.
        """
        default = self._state['nodes'][name]['default']
        self._exist(self._state, name, component=component, cname=cname, parameter='converter', check_default=True)

        # Grab converter from the default params
        converter_default = default[component][cname]['converter']

        # Replace the converter with the default converter
        self._set_converter(name, component, cname, converter_default)

    def register(self):
        """
        Set the addresses in all incoming components.
        Validate the graph.
        Create params that can be uploaded to the ROS param server.
        """
        # Check if valid graph.
        assert self.is_valid(plot=False), 'Graph not valid.'

        # Add addresses based on connections
        state = deepcopy(self._state)
        for source, target in state['connects']:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            address = '%s/%s/%s' % (source_name, source_comp, source_cname)
            state['nodes'][target_name]['params'][target_comp][target_cname]['address'] = address

            # For actions & observations, replace default args
            if source_name == 'env/actions':
                default = state['nodes'][target_name]['params']['default']
                context = {'default': default}
                cname_params = state['nodes'][source_name]['params'][source_comp][source_cname]
                substitute_args(cname_params, context, only=['default', 'ns'])
            if target_name == 'env/observations':
                default = state['nodes'][source_name]['params']['default']
                context = {'default': default}
                cname_params = state['nodes'][target_name]['params'][target_comp][target_cname]
                substitute_args(cname_params, context, only=['default', 'ns'])

        # Initialize param objects
        nodes = []
        objects = []
        render = None
        actions = None
        observations = None
        for name, entry in state['nodes'].items():
            params = entry['params']
            if 'node_type' in params:
                if name == 'env/actions':
                    actions = RxNodeParams(name, params)
                elif name == 'env/observations':
                    observations = RxNodeParams(name, params)
                elif name == 'env/render':
                    render = RxNodeParams(name, params)
                else:
                    nodes.append(RxNodeParams(name, params))
            else:
                objects.append(RxObjectParams(name, params))

        assert actions, 'No action node defined in the graph.'
        assert observations, 'No observation node defined in the graph.'
        return nodes, objects, actions, observations, render

    def save(self, path: str):
        with open(path, 'w') as outfile:
            yaml.dump(self._state, outfile, default_flow_style=False)
        pass

    def load(self, path: str):
        with open(path, "r") as stream:
            try:
                self._state = yaml.safe_load(stream)
                # self._state = yaml.load(path)
            except yaml.YAMLError as exc:
                print(exc)

    def update(self, entities: Optional[List[str]]=None):
        # todo: updates the default params to the yaml as specified in the config.
        # todo: update actual params with additional default args & new I/O & name changes & new bridge implementations
        # todo: if None, update all entities
        assert False, 'Not implemented'

    def gui(self):
        from eagerx_core.gui import launch_gui
        self._state = launch_gui(deepcopy(self._state))

    @staticmethod
    def _exist(state: Dict, name: str, component: Optional[str] = None, cname: Optional[str] = None, parameter: Optional[str] = None, check_default: Optional[bool] = False):
        """
        Check if provided entry exists.
        """
        # Check that node/object exists
        assert name in state['nodes'], 'There is no node or object registered in this graph with name "%s".' % name

        # See if we must check both default and current params.
        if check_default:
            check_params = (state['nodes'][name]['params'], state['nodes'][name]['default'])
        else:
            check_params = (state['nodes'][name]['params'],)

        # Check params
        for params in check_params:
            default = params['default']

            # Check that components and specific entry (cname) exists
            assert component is None or component in params, f'Component "{component}" not present in "{name}".'
            if component is None:
                assert cname is None, f'Cannot check if "{name}" exists, because no component was specified.'
            assert cname is None or cname in params[component], f'"{cname}" not defined in "{name}" under {component}.'

            # check that parameter exists
            if parameter is not None:
                if (component is not None) and (cname is not None):  # component parameter
                    assert parameter in params[component][cname], f'Cannot set parameter "{parameter}". Parameter does not exist in "{cname}" under {component}.'
                else:
                    assert parameter in default, f'Cannot set parameter "{parameter}". Parameter does not exist under "default" of {name}.'

    @staticmethod
    def _is_selected(state: Dict, name: str, component: str, cname: str):
        """
        Check if provided entry was selected in params.
        """
        ObjectGraph._exist(state, name, component, cname)
        params = state['nodes'][name]['params']
        component = 'outputs' if component == 'feedthroughs' else component
        assert cname in params['default'][component], '"%s" not selected in "%s" under "default" in %s. ' % (cname, name, component)

    @staticmethod
    def _correct_signature(name: Optional[str] = None, component: Optional[str] = None, cname: Optional[str] = None, actuator: Optional[str] = None, sensor: Optional[str] = None):
        # assert only actuator, only sensor, or only name, component, cname
        if (name is not None) and (component is not None) and (cname is not None):  # component parameter
            assert actuator is None, 'If {name, component, cname} are specified, actuator argument cannot be specified.'
            assert sensor is None, 'If {name, component, cname} are specified, sensor argument cannot be specified.'
        if name is not None:  # entity parameter
            assert actuator is None, 'If {name, component, cname} are specified, actuator argument cannot be specified.'
            assert sensor is None, 'If {name, component, cname} are specified, sensor argument cannot be specified.'
        if component is not None:  # entity parameter
            assert name is not None, 'Either both or None of component "%s" and name "%s" must be specified.' % (component, name)
            assert actuator is None, 'If {name, component, cname} are specified, actuator argument cannot be specified.'
            assert sensor is None, 'If {name, component, cname} are specified, sensor argument cannot be specified.'
        if cname is not None:  # entity parameter
            assert name is not None, 'Either both or None of component "%s" and name "%s" must be specified.' % (component, name)
            assert component is not None, 'If cname "%s" is specified, also component "%s" and name "%s" must be specified.' % (cname, component, name)
            assert actuator is None, 'If {name, component, cname} are specified, actuator argument cannot be specified.'
            assert sensor is None, 'If {name, component, cname} are specified, sensor argument cannot be specified.'
        if actuator:
            assert sensor is None, 'If actuator is specified, sensor must be None.'
            assert (name is None) and (component is None) and (cname is None), 'If actuator is specified, arguments {name, component, cname} cannot be specified.'
        if sensor:
            assert actuator is None, 'If observation is specified, actuator must be None.'
            assert (name is None) and (component is None) and (cname is None), 'If actuator is specified, arguments {name, component, cname} cannot be specified.'

    def is_valid(self, plot=True):
        return self._is_valid(self._state, plot=plot)

    @staticmethod
    def _is_valid(state, plot=True):
        state = deepcopy(state)
        ObjectGraph.check_msg_types_are_consistent(state)
        ObjectGraph.check_inputs_have_address(state)
        ObjectGraph.check_graph_is_acyclic(state, plot=plot)
        return True

    @staticmethod
    def check_msg_type(source, target, state):
        source_name, source_comp, source_cname = source
        source_params = state['nodes'][source_name]['params']
        target_name, target_comp, target_cname = target
        target_params = state['nodes'][target_name]['params']

        # Convert the source msg_type to target msg_type with converters:
        # msg_type_source --> output_converter --> msg_type_ROS --> input_converter --> msg_type_target
        msg_type_out = get_cls_from_string(source_params[source_comp][source_cname]['msg_type'])
        converter_out = source_params[source_comp][source_cname]['converter']
        msg_type_ros = get_opposite_msg_cls(msg_type_out, converter_out)
        converter_in = target_params[target_comp][target_cname]['converter']
        msg_type_in = get_opposite_msg_cls(msg_type_ros, converter_in)

        # Verify that this msg_type_in is the same as the msg_type specified in the target
        msg_type_in_yaml = get_cls_from_string(target_params[target_comp][target_cname]['msg_type'])

        msg_type_str = msg_type_error(source, target, msg_type_out, converter_out, msg_type_ros, converter_in, msg_type_in, msg_type_in_yaml)
        assert msg_type_in == msg_type_in_yaml, msg_type_str

    @staticmethod
    def check_msg_types_are_consistent(state):
        for source, target in state['connects']:
            ObjectGraph.check_msg_type(source, target, state)
        return True

    @staticmethod
    def _get_address(source: Tuple[str, str, str], target: Tuple[str, str, str]):
        """ Determine the address."""
        source_name, source_comp, source_cname = source
        target_name, target_comp, target_cname = target
        if source_name == 'actuators':
            assert not target_name == 'sensors', f'A direct connection between a sensor "{target_cname}" and actuator "{source_cname}" cannot exist.'
            address = f'$(ns obj_name)/{source_name}/{source_cname}'
        elif target_name == 'sensors':
            assert not source_name == 'actuators', f'A direct connection between a sensor "{target_cname}" and actuator "{source_cname}" cannot exist.'
            address = f'$(ns obj_name)/{target_name}/{target_cname}'
        else:
            address = f'{source_name}/{source_comp}/{source_cname}'
        return address

    @staticmethod
    def check_inputs_have_address(state):
        state = deepcopy(state)
        for source, target in state['connects']:
            address = ObjectGraph._get_address(source, target)
            target_name, target_comp, target_cname = target
            state['nodes'][target_name]['params'][target_comp][target_cname]['address'] = address

        for name, entry in state['nodes'].items():
            params = entry['params']
            for component in params['default']:
                if component not in ['inputs', 'outputs', 'targets', 'feedthroughs', 'states']:
                    continue
                for cname in params['default'][component]:
                    assert cname in params[component], f'"{cname}" was selected in {component} of "{name}", but has no implementation.'
                    if component not in ['inputs', 'targets', 'feedthroughs']: continue
                    if name in ['sensors', 'actuators']: continue
                    assert params[component][cname]['address'] is not None, f'"{cname}" was selected in {component} of "{name}", but no address was specified. Either deselect it, or connect it.'
        return True

    @staticmethod
    def check_graph_is_acyclic(state, plot=True):
        # Add nodes
        G = nx.MultiDiGraph()
        # label_mapping = {'env/observations/set': 'observations', 'env/render/done': 'render'}
        for node, params in state['nodes'].items():
            default = params['params']['default']
            if 'outputs' in default:
                for cname in default['outputs']:
                    name = '%s/%s' % (node, cname)
                    remain_active = True if node == 'actuators' else False
                    G.add_node(name, remain_active=remain_active, always_active=False, is_stale=False)
            if node == 'sensors':
                for cname in default['inputs']:
                    name = '%s/%s' % (node, cname)
                    G.add_node(name, remain_active=True, always_active=True, is_stale=False)

        # Add edges
        # for cname in state['nodes']['actuators']['params']['default']['outputs']:
        #     if cname == 'set': continue
        #     name = 'env/actions/%s' % cname
        #     label_mapping[name] = 'actions/%s' % cname
        #     G.add_edge('env/observations/set', name, # key='%s/%s' % ('inputs', 'observations_set'),
        #                feedthrough=False, style='solid', color='black', alpha=1.0, is_stale=False, skip=False,
        #                source=('env/observations', 'outputs', 'set'), target=('env/actions', 'inputs', 'observations_set'))
        target_comps = ['inputs']
        source_comps = ['outputs']
        for source, target in state['connects']:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            if source_comp in source_comps and target_comp in target_comps:
                # Determine source node name
                # if 'node_type' in state['nodes'][source_name]['params']:
                source_edge = '%s/%s' % (source_name, source_cname)
                # else:
                #     source_edge = '%s/%s/%s' % (source_name, source_comp, source_cname)
                # Determine target node name
                target_edges = []
                target_default = state['nodes'][target_name]['params']['default']
                # if 'node_type' in state['nodes'][target_name]['params']:
                for cname in target_default['outputs']:
                    target_edge = '%s/%s' % (target_name, cname)
                    target_edges.append(target_edge)
                if target_name == 'sensors':
                    for cname in target_default['inputs']:
                        target_edge = '%s/%s' % (target_name, cname)
                        target_edges.append(target_edge)
                # else:
                #     target_edge = '%s/%s/%s' % (target_name, target_comp, target_cname)
                    target_edges.append(target_edge)

                # Determine stale nodes in real_reset routine via feedthrough edges
                if target_comp == 'feedthroughs':
                    feedthrough = True
                else:
                    feedthrough = False

                # Determine edges that do not break DAG property (i.e. edges that are skipped)
                skip = state['nodes'][target_name]['params'][target_comp][target_cname]['skip']
                color = 'green' if skip else 'black'
                style = 'dotted' if skip else 'solid'

                # Add edge
                for target_edge in target_edges:
                    G.add_edge(source_edge, target_edge,
                               color=color, feedthrough=feedthrough, style=style, alpha=1.0,
                               is_stale=False, skip=skip, source=source, target=target)

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
            ax_env.set_title('Communication graph (episode)')
            _, _, _, pos = plot_graph(G, k=2, ax=ax_env)
            plt.show()

        # Assert if graph is a directed-acyclical graph (DAG)
        cycle_strs = ['Algebraic loops detected: ']
        for idx, connect in enumerate(cycles):
            connect.append(connect[0])
            s = ' Loop %s: ' % idx
            n = '\n' + ''.join([' ']*len(s)) + '...-->'
            s = '\n\n' + s + '...-->'
            for idx in range(len(connect)-1):
                tmp, target = connect[idx]
                source, tmp2 = connect[idx+1]
                source_name, source_comp, source_cname = source
                target_name, target_comp, target_cname = target
                assert source_name == target_name, 'Source and target not equal: %s, %s' % (source, target)
                connect_name = '%s/%s/%s][%s/%s/%s' % tuple(list(source) + list(target))
                node_name = ('Node: ' + source_name).center(len(connect_name), ' ')
                s += '[%s]-->' % connect_name
                n += '[%s]-->' % node_name
            s += '...'
            n += '...'
            cycle_strs.append(s)
            cycle_strs.append(n)
            connect.pop(-1)
        assert is_dag, ''.join(cycle_strs)
        assert len(not_active) == 0, 'Stale episode graph detected. Nodes "%s" will be stale, while they must be active (i.e. connected) in order for the graph to resolve (i.e. not deadlock).' % not_active

        # Create a shallow copy graph that excludes feedthrough edges
        F = reset_graph(G)
        not_active = is_stale(F)
        color_nodes(F)
        color_edges(F)

        # Plot graphs
        if plot:
            fig_reset, ax_reset = plt.subplots(nrows=1, ncols=1)
            ax_reset.set_title('Communication graph (reset)')
            _, _, _, pos = plot_graph(F, pos=pos, ax=ax_reset)
            plt.show()

        # Assert if reset graph is not stale
        has_real_reset = len([e for e, ft in nx.get_edge_attributes(G, 'feedthrough').items() if ft]) > 0
        assert len(not_active) == 0 or not has_real_reset, 'Stale reset graph detected. Nodes "%s" will be stale, while they must be active (i.e. connected) in order for the graph to resolve (i.e. not deadlock).' % not_active
        return True