import rospy
import yaml
from yaml import dump
yaml.Dumper.ignore_aliases = lambda *args: True  # todo: check if needed.
from typing import List, Union, Dict, Tuple, Optional
from eagerx_core.params import RxNodeParams, RxObjectParams, add_default_args
from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string
from copy import deepcopy


class RxGraph:
    def __init__(self, state: Dict):
        self._state = state

    @classmethod
    def create(cls, nodes: Optional[List[RxNodeParams]] = None, objects: Optional[List[RxObjectParams]] = None):
        if nodes is None:
            nodes = []
        if objects is None:
            objects = []
        if isinstance(nodes, RxNodeParams):
            nodes = [nodes]
        if isinstance(objects, RxObjectParams):
            objects = [objects]

        # Add action & observation node to list
        actions = RxNodeParams.create(name='env/actions', package_name='eagerx_core', config_name='actions', rate=1.0)
        observations = RxNodeParams.create(name='env/observations', package_name='eagerx_core', config_name='observations', rate=1.0)
        nodes += [actions, observations]

        # Create a state
        state = dict(nodes=dict(), connects=list())
        cls.__add(state, nodes)
        cls.__add(state, objects)
        return cls(state)

    def add(self, entities: Union[Union[RxNodeParams, RxObjectParams], List[Union[RxNodeParams, RxObjectParams]]]):
        self.__add(self._state, entities)

    @staticmethod
    def __add(state: Dict, entities: Union[Union[RxNodeParams, RxObjectParams], List[Union[RxNodeParams, RxObjectParams]]]):
        if not isinstance(entities, list):
            entities = [entities]

        for entity in entities:
            name = entity.name
            assert name not in state['nodes'], 'There is already a node or object registered in this graph with name "%s".' % name

            # Add node to state
            params = entity.params
            package_name = params['default']['package_name']
            config_name = params['default']['config_name']
            if isinstance(entity, RxNodeParams):
                params_default = RxNodeParams.create(name, package_name, config_name, rate=1).params
            else:
                params_default = RxObjectParams.create(name, package_name, config_name).params
            state['nodes'][name] = dict()
            state['nodes'][name]['params'] = deepcopy(params)
            state['nodes'][name]['default'] = params_default

    def _remove(self, names: Union[str, List[str]]):
        """
        First removes all associated connects from self._state.
        Then, removes node/object from self._state.
        **DOES NOT** remove observation entries if they are disconnected.
        **DOES NOT** remvoe action entries if they are disconnect and the last connection.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            assert name in self._state['nodes'], 'Cannot delete "%s" as there is no node/object with that name in the graph.' % name
            for idx, c in enumerate(deepcopy(self._state['connects'])):
                source = c[0]
                target = c[1]
                if name in [source[0], target[0]]:
                    self.disconnect(source, target)
            self._state['nodes'].pop(name)

    def remove(self, names: Union[str, List[str]]):
        """
        First removes all associated connects from self._state.
        Then, removes node/object from self._state.
        Also removes observation entries if they are disconnected.
        Also removes action entries if they are disconnect and the last connection.
        """
        if not isinstance(names, list):
            names = [names]
        for name in names:
            assert name in self._state['nodes'], 'Cannot delete "%s" as there is no node/object with that name in the graph.' % name
            for idx, c in enumerate(deepcopy(self._state['connects'])):
                source = c[0]
                target = c[1]
                if name in [source[0], target[0]]:
                    if source[0] == 'env/actions':
                        action = source[2]
                        source = None
                    else:
                        action = None
                        source = source
                    if target[0] == 'env/observations':
                        observation = target[2]
                        target = None
                    else:
                        observation = None
                        target = target
                    self.disconnect(source, target, action, observation)
            self._state['nodes'].pop(name)

    def _add_action(self, action: str):
        """
        Adds disconnected action entry to 'env/actions' node in self._state.
        """
        assert action != 'set', 'Cannot define an action with the reserved name "set".'
        params_action = self._state['nodes']['env/actions']['params']
        if action not in params_action['outputs']:  # Action already registered
            params_action['default']['outputs'].append(action)
            params_action['outputs'][action] = dict()

    def _add_observation(self, observation: str):
        """
        Adds disconnected observation entry to 'env/observations' node in self._state.
        """
        assert observation != 'actions_set', 'Cannot define an observations with the reserved name "actions_set".'
        params_obs = self._state['nodes']['env/observations']['params']
        if observation in params_obs['inputs']:
            assert len(params_obs['inputs'][observation]) == 0, 'Observation "%s" already exists and is connected.' % observation
        else:
            params_obs['default']['inputs'].append(observation)
            params_obs['inputs'][observation] = dict()

    def _connect_action(self, action, target, converter=None):
        """
        Method to connect a (previously added) action, that *precedes* self._connect(source, target).
        """
        params_action = self._state['nodes']['env/actions']['params']
        assert action in params_action['outputs'], 'Action "%s" must be added, before you can connect it.' % action
        name, component, cname = target
        if component == 'feedthroughs': component = 'outputs'
        params_target = self._state['nodes'][name]['params']

        assert 'space_converter' in params_target[component][cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".' % (cname, component, name)

        # Infer source properties (converter & msg_type) from target
        space_converter = params_target[component][cname]['space_converter']
        msg_type_C = get_cls_from_string(params_target[component][cname]['msg_type'])
        if converter:  # Overwrite msg_type_B if input converter specified
            msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
        else:
            msg_type_B = msg_type_C

        # Set properties in node params of 'env/actions'
        if len(params_action['outputs'][action]) > 0:  # Action already registered
            space_converter_state = params_action['outputs'][action]['converter']
            msg_type_B_state = get_opposite_msg_cls(params_action['outputs'][action]['msg_type'], space_converter_state)
            assert msg_type_B == msg_type_B_state, 'Conflicting %s for action "%s" that is already used in another connection. Occurs with connection %s' % ('msg_types', action, tuple([name, component, cname]))
            if not space_converter == space_converter_state:
                rospy.logwarn('Conflicting %s for action "%s". Not using the space_converter of %s[%s][%s]' % ('space_converters', action, name, component, cname))
            msg_type_A = get_opposite_msg_cls(msg_type_B_state, space_converter_state)
        else:
            # Verify that converter is not modifying the msg_type (i.e. it is a processor).
            assert msg_type_B == msg_type_C, 'Cannot have a converter that maps to a different msg_type as the converted msg_type will not be compatible with the space_converter specified in the .yaml.'
            msg_type_A = get_opposite_msg_cls(msg_type_B, space_converter)
            params_action['outputs'][action]['msg_type'] = get_module_type_string(msg_type_A)
            params_action['outputs'][action]['converter'] = space_converter
            add_default_args(params_action['outputs'][action], component='outputs')

    def _connect_observation(self, source, observation, converter):
        """
        Method to connect a (previously added & disconnected) observation, that *precedes* self._connect(source, target).
        """
        params_obs = self._state['nodes']['env/observations']['params']
        assert observation in params_obs['inputs'], 'Observation "%s" must be added, before you can connect it.' % observation
        name, component, cname = source
        params_source = self._state['nodes'][name]['params']

        assert converter is not None or 'space_converter' in params_source[component][cname], '"%s" does not have a space_converter defined under %s in the .yaml of "%s". Either specify it there, or add an input converter that acts as a space_converter to this connection.' % (cname, component, name)

        # Infer target properties (converter & msg_type) from source
        msg_type_A = get_cls_from_string(params_source[component][cname]['msg_type'])
        output_converter = params_source[component][cname]['converter']
        msg_type_B = get_opposite_msg_cls(msg_type_A, output_converter)
        if converter is None:
            converter = params_source[component][cname]['space_converter']
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)

        # Set properties in node params of 'env/observations'
        assert len(params_obs['inputs'][observation]) == 0, 'Observation "%s" already connected.' % observation
        params_obs['inputs'][observation]['msg_type'] = get_module_type_string(msg_type_C)
        add_default_args(params_obs['inputs'][observation], component='inputs')
        return converter

    def _disconnect_action(self, action: str):
        """
        Returns the action entry back to its disconnected state.
        That is, remove space_converter if it is not connected to any other targets.
        """
        params_action = self._state['nodes']['env/actions']['params']
        assert action in params_action['outputs'], 'Cannot disconnect action "%s", as it does not exist.' % action
        source = ['env/actions', 'outputs', action]
        connect_exists = False
        for idx, c in enumerate(self._state['connects']):
            if source == c[0]:
                connect_exists = True
                break
        if not connect_exists:
            params_action['outputs'][action] = dict()

    def _disconnect_observation(self, observation: str):
        """
        Returns the observation entry back to its disconnected state (i.e. empty dict).
        """
        params_obs = self._state['nodes']['env/observations']['params']
        assert observation in params_obs['inputs'], 'Cannot disconnect observation "%s", as it does not exist.' % observation
        params_obs['inputs'][observation] = dict()

    def _remove_action(self, action: str):
        """
        Method to remove an action. Can only remove existing and disconnected actions.
        """
        params_action = self._state['nodes']['env/actions']['params']
        source = ['env/actions', 'outputs', action]
        connect_exists = False
        for idx, c in enumerate(self._state['connects']):
            if source == c[0]:
                connect_exists = True
                target = c[1]
                break
        assert not connect_exists, 'Action entry "%s" cannot be removed, because it is not disconnected. Connection with target %s still exists.' % (action, target)
        assert action in params_action['outputs'], 'Action "%s" cannot be removed, because it does not exist.' % action

        params_action['outputs'].pop(action)
        params_action['default']['outputs'] = [cname for cname in params_action['default']['outputs'] if cname is not action]

    def _remove_observation(self, observation: str):
        """
        Method to remove an observation. Can only remove existing and disconnected observations.
        """
        params_obs = self._state['nodes']['env/observations']['params']
        target = ['env/observations', 'inputs', observation]
        connect_exists = False
        for idx, c in enumerate(self._state['connects']):
            if target == c[1]:
                connect_exists = True
                source = c[0]
                break
        assert not connect_exists, 'Observation entry "%s" cannot be removed, because it is not disconnected. Connection with source %s still exists.' % (observation, source)
        assert observation in params_obs['inputs'], 'Observation "%s" cannot be removed, because it does not exist.' % observation

        params_obs['inputs'].pop(observation)
        params_obs['default']['inputs'] = [cname for cname in params_obs['default']['inputs'] if cname is not observation]

    def _connect(self,
                source: Optional[Tuple[str, str, str]] = None,
                target: Optional[Tuple[str, str, str]] = None,
                converter: Optional[Dict] = None,
                window: Optional[int] = None,
                delay: Optional[float] = None):
        """
        Method to connect a source to a target. For actions/observations, first a disconnected entry must be created,
        after which an additional call to connect_action/observation is required. For more info, see self.connect.
        """
        if isinstance(source, tuple):
            source = list(source)
        if isinstance(target, tuple):
            target = list(target)

        # Perform checks on source
        source_name, source_comp, source_cname = source
        source_params = self._state['nodes'][source_name]['params']
        assert source_cname in source_params['default'][source_comp], '"%s" was not selected in %s of source "%s" during its initialization.' % (source_cname, source_comp, source_name)

        # Perform checks on target
        target_name, target_comp, target_cname = target
        target_params = self._state['nodes'][target_name]['params']
        if target_comp == 'feedthroughs':
            assert window is None or window > 0, 'Feedthroughs must have a window > 0, else no action can be fed through.'
            assert target_cname in target_params['default']['outputs'], '"%s" was not selected in %s of target "%s" during its initialization.' % (target_cname, 'outputs', target_name)
        else:
            assert target_cname in target_params['default'][target_comp], '"%s" was not selected in %s of target "%s" during its initialization.' % (target_cname, target_comp, target_name)

        # Add properties to target params
        if converter is not None:
            target_params[target_comp][target_cname]['converter'] = converter
        if window is not None:
            target_params[target_comp][target_cname]['window'] = window
        if delay is not None:
            target_params[target_comp][target_cname]['delay'] = delay

        # Add connection
        connect = [source, target]
        self._state['connects'].append(connect)

    def connect(self,
                source: Optional[Tuple[str, str, str]] = None,
                target: Optional[Tuple[str, str, str]] = None,
                action: str = None, observation: str = None,
                converter: Optional[Dict] = None,
                window: Optional[int] = None,
                delay: Optional[float] = None):
        """
        Method to connect source/action to target/observation. For actions/observations we first add a disconnected
        action/observation and immediately connect it.
        """
        assert not source or not action, 'You cannot specify a source if you wish to connect action "%s", as the action will act as the source.' % action
        assert not target or not observation, 'You cannot specify a target if you wish to connect observation "%s", as the observation will act as the target.' % observation
        assert not (observation and action), 'You cannot connect an action directly to an observation.'

        # Add action/observation entry & connect it to target/source
        if action:
            source = ('env/actions', 'outputs', action)
            self._add_action(action)
            self._connect_action(action, target, converter)
        if observation:
            target = ('env/observations', 'inputs', observation)
            self._add_observation(observation)
            converter = self._connect_observation(source, observation, converter)

        # Connect target & source as usual
        self._connect(source, target, converter, window, delay)

    def _disconnect(self,
                   source: Optional[Tuple[str, str, str]] = None,
                   target: Optional[Tuple[str, str, str]] = None,
                   action: str = None, observation: str = None, ):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        """
        assert not source or not action, 'You cannot specify a source if you wish to disconnect action "%s", as the action will act as the source.' % action
        assert not target or not observation, 'You cannot specify a target if you wish to disconnect observation "%s", as the observation will act as the target.' % observation
        assert not (observation and action), 'You cannot disconnect an action from an observation, as such a connection cannot exist.'
        if isinstance(source, tuple):
            source = list(source)
        if isinstance(target, tuple):
            target = list(target)

        # Create source & target entries
        if action:
            source = ['env/actions', 'outputs', action]
        if observation:
            target = ['env/observations', 'inputs', observation]

        # Check if connection exists
        connect_exists = False
        idx_connect = None
        for idx, c in enumerate(self._state['connects']):
            if source == c[0] and target == c[1]:
                connect_exists = True
                idx_connect = idx
                break
        assert connect_exists, 'The connection with source=%s and target=%s cannot be removed, because it does not exist.' % (source, target)

        # Pop the connection from the state
        self._state['connects'].pop(idx_connect)

        # Reset source params to disconnected state
        if action:
            self._disconnect_action(action)
        else:
            # Nothing to do here (for now)
            source_name, source_comp, source_cname = source
            source_params = self._state['nodes'][source_name]['params']

        # Reset target params to disconnected state (reset to go back to default yaml), i.e. reset window/delay/converter.
        if observation:
            self._disconnect_observation(observation)
        else:
            target_name, target_comp, target_cname = target
            target_params = self._state['nodes'][target_name]['params']
            target_params[target_comp][target_cname] = self._state['nodes'][target_name]['default'][target_comp][target_cname]

    def disconnect(self,
                   source: Optional[Tuple[str, str, str]] = None,
                   target: Optional[Tuple[str, str, str]] = None,
                   action: str = None, observation: str = None, ):
        """
        Disconnects a source from a target. The target is reset in self._state to its disconnected state.
        In case of an observation, the complete entry is always removed.
        In case of an action, it is removed if the action is not connected to any other target.
        """
        self._disconnect(source, target, action, observation)
        if action:
            connect_exists = False
            source = ['env/actions', 'outputs', action]
            for idx, c in enumerate(self._state['connects']):
                if source == c[0]:
                    connect_exists = True
                    break
            if not connect_exists:
                self._remove_action(action)
        if observation:
            self._remove_observation(observation)

    def register_graph(self):
        """
        Set the addresses in all incoming components.
        Validate the graph.
        Create params that can be uploaded to the ROS param server.
        """
        # Add addresses based on connections
        state = deepcopy(self._state)
        for source, target in state['connects']:
            source_name, source_comp, source_cname = source
            target_name, target_comp, target_cname = target
            address = '%s/%s/%s' % (source_name, source_comp, source_cname)
            state['nodes'][target_name]['params'][target_comp][target_cname]['address'] = address

        # Check if valid graph. todo: informative error message
        assert self.is_valid(state), 'Graph not valid.'

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

    def render(self, source: Tuple[str, str, str], rate: float, converter: Optional[Dict] = None, window: Optional[int] = None, delay: Optional[float] = None,
               package_name='eagerx_core', config_name='render', **kwargs):
        # Delete old render node from self._state['nodes'] if it exists
        if 'env/render' in self._state['nodes']:
            self.remove('env/render')

        # Add (new) render node to self._state['node']
        render = RxNodeParams.create('env/render', package_name, config_name, rate=rate, **kwargs)
        self.add(render)

        # Create connection
        target = ('env/render', 'inputs', 'image')
        self.connect(source=source, target=target, converter=converter, window=window, delay=delay)

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
        # todo: update actual params with additional default arss & new I/O & name changes & new bridge implementations
        # todo: if None, update all entities
        assert False, 'Not implemented'

    def gui(self):
        # todo: JELLE opens gui with state and outputs state
        assert False, 'Not implemented'
        self._state = RxGui(deepcopy(self._state))

    @staticmethod
    def is_valid(state):
        # todo: create individual checks for:
        #  - DAG (with/without reset node)
        #  - check compatibility with bridges together with which objects are supported where (tabulate?)
        RxGraph.check_msg_types(state)
        RxGraph.check_all_connected(state)
        return True

    @staticmethod
    def check_msg_types(state):
        for source, target in state['connects']:
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
            if target_comp == 'feedthroughs':
                msg_type_in_target = get_cls_from_string(target_params['outputs'][target_cname]['msg_type'])
            else:
                msg_type_in_target = get_cls_from_string(target_params[target_comp][target_cname]['msg_type'])

            msg_type_str = '\n\nConversion of msg_type from source="%s/%s/%s" ---> target="%s/%s/%s":\n\n' % tuple(source + target)
            msg_type_str += '>> msg_type_source:  %s (as specified in source)\n         ||\n         \/\n' % msg_type_out
            msg_type_str += '>> output_converter: %s \n         ||\n         \/\n' % converter_out
            msg_type_str += '>> msg_type_ROS:     %s \n         ||\n         \/\n' % msg_type_ros
            msg_type_str += '>> input_converter:  %s \n         ||\n         \/\n' % converter_in
            msg_type_str += '>> msg_type_target:  %s (inferred from converters)\n         /\ \n         || (These must be equal, but they are not!!)\n         \/\n' % msg_type_in
            msg_type_str += '>> msg_type_target:  %s (as specified in target)\n' % msg_type_in_target
            try:
                assert msg_type_in == msg_type_in_target, msg_type_str
            except:
                pass
        return True

    @staticmethod
    def check_all_connected(state):
        for name, entry in state['nodes'].items():
            params = entry['params']
            if 'node_type' in params:
                for component in params['default']:
                    if component not in ['inputs', 'outputs', 'targets', 'feedthroughs', 'states']:
                        continue
                    for cname in params['default'][component]:
                        assert cname in params[component], '"%s" was selected in %s of "%s", but has no implementation.' % (cname, component, name)
                        if component not in ['inputs', 'targets', 'feedthroughs']: continue
                        assert 'address' in params[component][cname], '"%s" was selected in %s of "%s", but no address was specified. Either deselect it, or connect it.' % (cname, component, name)
            else:
                for component in params['default']:
                    if component not in ['sensors', 'actuators', 'states']:
                        continue
                    for cname in params['default'][component]:
                        assert cname in params[component], '"%s" was selected in %s of "%s", but has no (agnostic) implementation.' % (cname, component, name)
                        if component not in ['actuators']: continue
                        assert 'address' in params[component][cname], '"%s" was selected in %s of "%s", but no address was specified. Either deselect it, or connect it.' % (cname, component, name)
        return True
