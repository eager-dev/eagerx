# ROS packages required
import rospy
import rosparam
from rosgraph.masterapi import Error

# EAGERX
from eagerx_core.params import RxNodeParams, RxObjectParams
from eagerx_core.utils.utils import load_yaml
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.node import RxNode
from eagerx_core.bridge import RxBridge
from eagerx_core.rxenv import RxEnvironment
from eagerx_core import RxMessageBroker

# OTHER IMPORTS
import gym
import numpy as np
from typing import List, Dict, Union
import multiprocessing


class Env(object):
    @staticmethod
    def define_actions():
        actions = RxNodeParams.create('env/actions', package_name='eagerx_core', config_name='actions')
        return actions

    @staticmethod
    def define_observations():
        observations = RxNodeParams.create('env/observations', 'eagerx_core', 'observations')
        return observations

    @staticmethod
    def define_supervisor():
        states = RxNodeParams.create('env/supervisor', 'eagerx_core', 'supervisor')
        return states

    def __init__(self, name: str, rate: int,
                 observations: RxNodeParams,
                 actions: RxNodeParams,
                 bridge: RxNodeParams,
                 nodes: List[RxNodeParams],
                 objects: List[RxObjectParams]) -> None:
        self.name = name
        self.ns = '/' + name
        self.rate = rate
        self.initialized = False
        self._bridge_name = bridge.name
        self._is_initialized = dict()
        self._launch_nodes = dict()
        self._sp_nodes = dict()
        self._event = multiprocessing.Event()

        # Initialize supervisor node
        self.mb, self.env_node, _ = self._init_supervisor(nodes, objects)

        # Initialize bridge
        self._init_bridge(bridge, nodes)

        # Initialize action & observation node
        self.act_node, self.obs_node, _, _ = self._init_actions_and_observations(actions, observations, self.mb)

        # Initialize nodes
        initialize_nodes(nodes, self.ns, self.name, self.mb, self._is_initialized, self._sp_nodes, self._launch_nodes, rxnode_cls=RxNode)

        # Register objects
        self.register_objects(objects)

    def _init_supervisor(self, nodes: List[RxNodeParams], objects: List[RxObjectParams]):
        # Initialize supervisor
        supervisor = self.define_supervisor()

        # Get all states from objects & nodes
        for i in nodes + objects:
            for cname in i.params['default']['states']:
                name = '%s/%s' % (i.name, cname)
                address = '%s/states/%s' % (i.name, cname)
                msg_type = i.params['states'][cname]['msg_type']
                space_converter = i.params['states'][cname]['space_converter']

                assert name not in supervisor.params['states'], 'Cannot have duplicate states. State "%s" is defined multiple times.' % name

                supervisor.params['states'][name] = dict(address=address, msg_type=msg_type, converter=space_converter)
                supervisor.params['default']['states'].append(name)

            # Get states from simnodes. WARNING: can make environment non-agnostic.
            if isinstance(i, RxObjectParams):
                for component in ('sensors', 'actuators'):
                    if component in i.params['default']:
                        for cname in i.params['default'][component]:
                            params_simnode = i.params[self._bridge_name][component][cname]
                            package_name, config_name = params_simnode['node_config'].split('/')
                            node_yaml = load_yaml(package_name, config_name)
                            if 'states' in params_simnode:
                                node_yaml['default']['states'] = params_simnode['states']
                            if 'states' in node_yaml['default']:
                                for simnode_cname in node_yaml['default']['states']:
                                    name = '%s/%s/%s/%s' % (i.name, component, cname, simnode_cname)
                                    address = '%s/%s/%s/states/%s' % (i.name, component, cname, simnode_cname)
                                    msg_type = node_yaml['states'][simnode_cname]['msg_type']
                                    space_converter = node_yaml['states'][simnode_cname]['space_converter']

                                    rospy.logwarn('Adding state "%s" to simulation nodes can potentially make the environment for object "%s" non-agnostic. Check "%s.yaml" in package "%s" for more info.' % (name, i.name, config_name, package_name))
                                    assert name not in supervisor.params[ 'states'], 'Cannot have duplicate states. State "%s" is defined multiple times.' % name

                                    supervisor.params['states'][name] = dict(address=address, msg_type=msg_type,
                                                                             converter=space_converter)
                                    supervisor.params['default']['states'].append(name)

        # Delete pre-existing parameters
        try:
            rosparam.delete_param('/%s' % self.name)
            rospy.loginfo('Pre-existing parameters under namespace "/%s" deleted.' % self.name)
        except Error:
            pass

        # Initialize message broker
        mb = RxMessageBroker(owner='%s/%s' % (self.ns, 'env'))

        # Create env node
        supervisor.params['default']['rate'] = self.rate
        supervisor_params = supervisor.get_params(ns=self.ns)
        rosparam.upload_params(self.ns, supervisor_params)
        rx_env = RxEnvironment(name='%s/%s' % (self.ns, supervisor.name), message_broker=mb, scheduler=None)
        rx_env.node_initialized()

        # Connect io
        mb.connect_io()
        return mb, rx_env.node, rx_env

    def _init_bridge(self, bridge: RxNodeParams, nodes: List[RxNodeParams]) -> None:
        # Check that reserved keywords are not already defined.
        assert 'node_names' not in bridge.params['default'], 'Keyword "%s" is a reserved keyword within the bridge params and cannot be used twice.' % 'node_names'
        assert 'target_addresses' not in bridge.params['default'], 'Keyword "%s" is a reserved keyword within the bridge params and cannot be used twice.' % 'target_addresses'

        # Extract node_names
        node_names = ['env/actions', 'env/observations', 'env/supervisor']
        target_addresses = []
        for i in nodes:
            node_names.append(i.params['default']['name'])
            if 'targets' in i.params['default']:
                for cname in i.params['default']['targets']:
                    address = i.params['targets'][cname]['address']
                    target_addresses.append(address)
        bridge.params['default']['node_names'] = node_names
        bridge.params['default']['target_addresses'] = target_addresses

        initialize_nodes(bridge, self.ns, self.name, self.mb, self._is_initialized, self._sp_nodes, self._launch_nodes, rxnode_cls=RxBridge)
        wait_for_node_initialization(self._is_initialized)  # Proceed after bridge is initialized

    def _init_actions_and_observations(self, actions: RxNodeParams, observations: RxNodeParams, message_broker):
        # Check that env has at least one input & output.
        assert len(observations.params['default']['inputs']) > 0, 'Environment "%s" must have at least one input (i.e. input).' % self.name
        assert len(actions.params['default']['outputs']) > 0, 'Environment "%s" must have at least one action (i.e. output).' % self.name

        # Check that all observation addresses are unique
        addresses_obs = [observations.params['inputs'][cname]['address'] for cname in observations.params['default']['inputs']]
        len(set(addresses_obs)) == len(addresses_obs), 'Duplicate observations found: %s. Make sure to only have unique observations' % (set([x for x in addresses_obs if addresses_obs.count(x) > 1]))

        # Create observation node
        observations.params['default']['rate'] = self.rate
        obs_params = observations.get_params(ns=self.ns)
        rosparam.upload_params(self.ns, obs_params)
        rx_obs = RxNode(name='%s/%s' % (self.ns, observations.name), message_broker=message_broker, scheduler=None)
        rx_obs.node_initialized()

        # Create action node
        actions.params['default']['rate'] = self.rate
        act_params = actions.get_params(ns=self.ns)
        rosparam.upload_params(self.ns, act_params)
        rx_act = RxNode(name='%s/%s' % (self.ns, actions.name), message_broker=message_broker, scheduler=None)
        rx_act.node_initialized()

        return rx_act.node, rx_obs.node, rx_act, rx_obs

    @property
    def observation_space(self):
        observation_space = dict()
        for name, buffer in self.obs_node.observation_buffer.items():
            observation_space[name] = buffer['converter'].get_space()
        return gym.spaces.Dict(spaces=observation_space)

    @property
    def action_space(self):
        action_space = dict()
        for name, buffer in self.act_node.action_buffer.items():
            action_space[name] = buffer['converter'].get_space()
        return gym.spaces.Dict(spaces=action_space)

    @property
    def state_space(self):
        state_space = dict()
        for name, buffer in self.env_node.state_buffer.items():
            state_space[name] = buffer['converter'].get_space()
        return gym.spaces.Dict(spaces=state_space)

    def _set_action(self, action):
        # Set actions in buffer
        for name, buffer in self.act_node.action_buffer.items():
            assert name in action, 'Action "%s" not specified. Must specify all actions in action_space.' % name
            buffer['msg'] = action[name]

    def _set_state(self, state):
        # Set states in buffer
        for name, msg in state.items():
            assert name in self.env_node.state_buffer, 'Cannot set unknown state "%s".' % name
            self.env_node.state_buffer[name]['msg'] = msg

    def _get_observation(self):
        # Get observations from buffer
        observation = dict()
        for name, buffer in self.obs_node.observation_buffer.items():
            observation[name] = buffer['msg']
        return observation

    def _initialize(self):
        assert not self.initialized, 'Environment already initialized. Cannot re-initialize pipelines. '

        # Wait for nodes to be initialized
        [node.node_initialized() for name, node in self._sp_nodes.items()]
        wait_for_node_initialization(self._is_initialized)

        # Initialize single process communication
        self.mb.connect_io(print_status=True)

        rospy.sleep(0.2)  # todo:sleep required
        rospy.loginfo('Nodes initialized.')

        # Perform first reset
        _ = self._reset()

        # todo: remove print status?
        # self.mb.print_io_status()

        # Nodes initialized
        self.initialized = True
        rospy.loginfo("Pipelines initialized.")

    def _reset(self):
        self.env_node.reset()
        return self._get_observation()

    def _step(self):
        self.env_node.step()
        return self._get_observation()

    def register_objects(self, objects: Union[List[RxObjectParams], RxObjectParams]):
        # todo: There might be timing issues... Currently solved with condition.
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        if not isinstance(objects, list):
            objects = [objects]

        # Register objects
        [self.env_node.register_object(o, self._bridge_name) for o in objects]

    def reset(self):
        # Initialize environment
        if not self.initialized:
            self._initialize()

        # Set desired reset states
        self._set_state(self.state_space.sample())
        # self._set_state({'N9': np.array([50], dtype='uint64')})

        # Perform reset
        observation = self._reset()
        return observation

    def step(self, action):
        # Check that nodes were previously initialized.
        assert self.initialized, 'Not yet initialized. Call .initialize_node_pipelines() before calling .step().'

        # Set actions in buffer
        self._set_action(action)

        # Send actions and wait for observations
        observation = self._step()
        reward = None
        is_done = False
        info = {}
        return observation, reward, is_done, info

    def close(self):
        for name in self._launch_nodes:
            self._launch_nodes[name].shutdown()
        try:
            rosparam.delete_param('/')
            rospy.loginfo('Pre-existing parameters under namespace "/" deleted.')
        except:
            pass