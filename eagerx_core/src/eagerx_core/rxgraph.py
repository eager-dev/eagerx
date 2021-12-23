import rospy
import yaml
from yaml import dump
from typing import List, Union, Dict, Tuple, Optional
from eagerx_core.params import RxNodeParams, RxObjectParams, add_default_args
from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string
from eagerx_core.utils.connection_utils import register_connections
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

    def add(self, entities: Union[Union[RxNodeParams, RxObjectParams], List[Union[RxNodeParams, RxObjectParams]]]):
        self.__add(self._state, entities)

    def remove(self, names: Union[str, List[str]]):
        if not isinstance(names, list):
            names = [names]
        for name in names:
            assert name in self._state['nodes'], 'Cannot delete "%s" as there is no node/object with that name in the graph.' % name
            self._state['nodes'].pop(name)
            for idx, c in enumerate(deepcopy(self._state['connects'])):
                source = c[0]
                target = c[1]
                if name in [source[0], target[0]]:
                    self.disconnect(source=source, target=target)

    def __connect_action(self, action, target, converter):
        name = target[0]
        component = target[1]
        if component == 'feedthroughs': component = 'outputs'
        cname = target[2]
        params_target = self._state['nodes'][name]['params']

        assert action != 'set', 'Cannot define an action with the reserved word "set".'
        assert 'space_converter' in params_target[component][cname], '"%s" does not have a space_converter defined under %s in the .yaml of object "%s".' % (cname, component, name)

        # Infer source properties (converter & msg_type) from target
        space_converter = params_target[component][cname]['space_converter']
        msg_type_C = get_cls_from_string(params_target[component][cname]['msg_type'])
        if converter:  # Overwrite msg_type_B if input converter specified
            msg_type_B = get_opposite_msg_cls(msg_type_C, converter)
        else:
            msg_type_B = msg_type_C

        # Set properties in node params of 'env/actions'
        params_action = self._state['nodes']['env/actions']['params']
        if action in params_action['outputs']:  # Action already registered
            space_converter_state = params_action['outputs'][action]['converter']
            msg_type_B_state = get_cls_from_string(params_action['outputs'][action]['msg_type'])
            assert msg_type_B == msg_type_B_state, 'Conflicting %s for action "%s" that is already used in another connection. Occurs with connection %s' % ('msg_types', action, tuple([name, component, cname]))
            if not space_converter == space_converter_state:
                rospy.logwarn('Conflicting %s for action "%s". Not using the space_converter of %s[%s][%s]' % ('space_converters', action, name, component, cname))
            msg_type_A = get_opposite_msg_cls(msg_type_B_state, space_converter_state)
        else:
            # Verify that converter is not modifying the msg_type (i.e. it is a processor).
            assert msg_type_B == msg_type_C, 'Cannot have a converter that maps to a different msg_type as the converted msg_type will not be compatible with the space_converter specified in the .yaml.'
            params_action['default']['outputs'].append(action)
            params_action['outputs'][action] = dict(msg_type=get_module_type_string(msg_type_B), converter=space_converter)
            add_default_args(params_action['outputs'][action], component='outputs')
            msg_type_A = get_opposite_msg_cls(msg_type_B, space_converter)

    def __connect_observation(self, source, observation, converter):
        name = source[0]
        component = source[1]
        cname = source[2]
        params_source = self._state['nodes'][name]['params']

        assert converter is not None or 'space_converter' in params_source[component][cname], '"%s" does not have a space_converter defined under %s in the .yaml of "%s". Either specify it there, or add an input converter that acts as a space_converter to this connection.' % (cname, component, name)

        # Infer target properties (converter & msg_type) from source
        msg_type_A = get_cls_from_string(params_source[component][cname]['msg_type'])
        if 'converter' in params_source[component][cname]:
            # Convert with output converter if specified
            # todo: how are output_converters specified for sensors?
            output_converter = params_source[component][cname]['converter']
            msg_type_B = get_opposite_msg_cls(msg_type_A, output_converter)
        else:
            msg_type_B = msg_type_A
        if converter is None:
            converter = params_source[component][cname]['space_converter']
        msg_type_C = get_opposite_msg_cls(msg_type_B, converter)

        # Set properties in node params of 'env/observations'
        params_obs = self._state['nodes']['env/observations']['params']
        assert observation not in params_obs['inputs'], 'Observation "%s" already defined.' % observation
        params_obs['default']['inputs'].append(observation)
        params_obs['inputs'][observation] = dict(msg_type=get_module_type_string(msg_type_B))
        add_default_args(params_obs['inputs'][observation], component='inputs')
        return converter

    def __disconnect_action(self, action):
        # todo: removes action iff there is no other node connected to this action.
        assert False, 'not implemented'

    def __disconnect_observation(self, observation):
        # todo: removes observation when connection is removed.
        assert False, 'not implemented'

    def connect(self,
                source: Optional[Tuple[str, str, str]] = None,
                target: Optional[Tuple[str, str, str]] = None,
                action: str = None, observation: str = None,
                converter: Optional[Dict] = None,
                window: Optional[int] = None,
                delay: Optional[float] = None):
        assert not source or not action, 'You cannot specify a source if you wish to connect action "%s", as the action will act as the source.' % action
        assert not target or not observation, 'You cannot specify a target if you wish to connect observation "%s", as the observation will act as the target.' % observation
        assert not (observation and action), 'You cannot connect an action directly to an observation.'

        if action:
            source = ('env/actions', 'outputs', action)
            self.__connect_action(action, target, converter)
        if observation:
            target = ('env/observations', 'inputs', observation)
            converter = self.__connect_observation(source, observation, converter)

        # Add input properties to corresponding node params
        # todo: check that, with feedthroughs, the corresponding output was actually selected in 'default'.
        # todo: add converter, window, delay to self._state[target[0]=name]['params'][target[1]=component][target[2]=cname]
        target_params = self._state['nodes'][target[0]]['params']
        if target[1] == 'feedthroughs':
            assert window is None or window > 0, 'Feedthroughs must have a window > 0, else no action can be fed through.'
            # assert
        connect = (source, target)
        self._state['connects'].append(connect)

    def disconnect(self,
                   source: Optional[Tuple[str, str, str]] = None,
                   target: Optional[Tuple[str, str, str]] = None,
                   action: str = None, observation: str = None, ):
        assert not source or not action, 'You cannot specify a source if you wish to disconnect action "%s", as the action will act as the source.' % action
        assert not target or not observation, 'You cannot specify a target if you wish to disconnect observation "%s", as the observation will act as the target.' % observation
        assert not (observation and action), 'You cannot disconnect an action from an observation, as such a connection cannot exist.'

        if action:
            source = ('env/actions', 'outputs', action)

        if observation:
            target = ('env/observations', 'inputs', observation)

        # Check if connection exists
        connect_exists = False
        for idx, c in enumerate(self._state['connects']):
            source = c[0]
            target = c[1]
            if source == c[0] and target == c[1]:
                connect_exists = True
                break
        assert connect_exists, 'The connection with source=%s and target=%s cannot be removed, because it does not exist.' % (source, target)

        # Reset source to state before connection todo: (nothing to do here?)
        if action:
            self.__disconnect_action(action)
        else:
            # todo: implement
            ...

        # Reset target to state from before connection (reset to go back to default yaml), i.e. reset window/delay/converter
        if observation:
            self.__disconnect_observation(observation)
        else:  # todo: different implementation for objects/nodes?
            ...

    def register_graph(self):
        # todo: called inside RxEnv
        # todo: convert self._state to RxObjectParams, RxNodeParams & connection list
        # todo: output params:  actions, observations, nodes, objects, nodes, render to be initialized
        assert False, 'Not implemented!'

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
        # todo: or use yaml.load(path)?
        with open(path, "r") as stream:
            try:
                self._state = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def update(self, entities: Optional[List[str]]=None):
        # todo: updates the default params to the yaml as specified in the config.
        # todo: update actual params with additional default arss & new I/O & name changes & new bridge implementations
        # todo: if None, update all entities
        assert False, 'Not implemented'

    def is_valid(self):
        # todo: test if valid (DAG checks)
        # todo: check if all specified inputs have been connected
        # todo: check if all msg_types are valid (including converters)
        # todo: issue warnings for incompatible bridges --> together with which objects are supported where (tabulate?)
        # todo: print compatible bridges
        pass

    def gui(self):
        # todo opens gui with state and outputs state
        assert False, 'Not implemented'
        self._state = RxGui(deepcopy(self._state))
