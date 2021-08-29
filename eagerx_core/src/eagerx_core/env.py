# ROS packages required
import rospy, rosparam
from std_msgs.msg import UInt64, String
from eagerx_core.params import RxNodeParams, RxInput, RxOutput, RxBridgeParams, RxObjectParams
from eagerx_core.utils.utils import get_attribute_from_module, launch_node, wait_for_node_initialization
from eagerx_core.node import RxNode
from eagerx_core.bridge import RxBridge
from eagerx_core import RxMessageBroker
from eagerx_core.params import RxNodeParams

from typing import List
from functools import partial
import multiprocessing
from threading import Condition
from rosgraph.masterapi import Error
from rx.scheduler import ThreadPoolScheduler

class Env(object):
    def __init__(self, name: str,
                 observations: List[RxInput],
                 actions: List[RxOutput],
                 bridge: RxBridgeParams,
                 nodes: List[RxNodeParams]) -> None:
        self.name = name
        self.ns = '/' + name
        self.bridge = bridge.name
        self.initialized = False
        self._dt = None
        self._params = None
        self._topics_in = None
        self._topics_out = None
        self._launch_nodes = dict()
        self._act_pub = dict()
        self._act_pub_reset = dict()
        self._obs_sub = dict()
        self._init_env(actions, observations)

        # Initialize register topic
        self.register_pub = rospy.Publisher(self.ns + '/register', String, queue_size=0)

        # Initialize nodes
        self.mb = RxMessageBroker(owner=self.ns + '/env')
        self.is_initialized = dict()
        self._sp_nodes = dict()
        self._init_bridge(bridge)
        self._init_nodes(nodes)

        # Initialize reset topics
        self.reset_pub = rospy.Publisher(self.ns + '/start_reset', UInt64, queue_size=0, latch=True)
        # self.reset_pub = rospy.Publisher(self.ns + '/real_reset', UInt64, queue_size=0, latch=True)
        rospy.sleep(0.1)  # todo: needed, else publisher might not yet be initialized

        # Initialize state topics
        self.sim_pub = rospy.Subscriber(self.ns + '/resettable/sim', String, self._register_states)
        self.real_pub = rospy.Subscriber(self.ns + '/resettable/real', String, self._register_states)
        self.states_reset = dict()

        # Message counter
        self.num_ticks = 0

        # Required for testing
        self.done = None
        self.obs_recv = None
        rospy.Subscriber(self.ns + '/reset', UInt64, lambda msg: self.__reset_handler(msg))
        self.cond_done = Condition()
        self.cond_obs = Condition()

        # Initialize waiting for observations & reset topics of inputs and states
        self.state_name = 'obj/states/position'
        rospy.Subscriber(self.ns + '/obj/actuators/ref_pos/applied/reset', UInt64, lambda msg: self.event.set())
        rospy.Subscriber(self.ns + '/obj/actuators/ref_pos/applied', UInt64, self.__obs_handler)
        self.event = multiprocessing.Event()

    def _init_env(self, actions: List[RxOutput], observations: List[RxInput]):
        # Check that env has at least one input.
        assert len(observations) > 0, 'Environment "%s" must have at least one input.' % self.name
        assert len(actions) > 0, 'Environment "%s" must have at least one output.' % self.name

        # Check that output rates are the same
        rates = []
        for o in actions:
            rates.append(o.rate)
        assert len(set(rates)) == 1, 'Environment "%s" can only have outputs with the same rate. Check the output rate.' % self.name

        # Delete pre-existing parameters
        try:
            rosparam.delete_param('/%s' % self.name)
            rospy.loginfo('Pre-existing parameters under namespace "/%s" deleted.' % self.name)
        except Error:
            pass

        # Define RxNodeParams for environment
        env = RxNodeParams(name='env', node_type='', module='', topics_in=observations, topics_out=actions)
        params = env.get_params(ns=self.ns)
        name = 'env'
        del params[name]['module'], params[name]['node_type'], params[name]['feedthrough_in'], params[name]['states_in']
        del params[name]['launch_locally'], params[name]['single_process']
        rosparam.upload_params(self.ns, params)

        # Calculate properties
        self._params = params[name]
        rate = self._params['rate']
        self._dt = 1 / rate

        # Initialize topics
        self._init_actions(self._params['topics_out'])
        self._init_observations(self._params['topics_in'])

    def _init_actions(self, topics_out):
        for i in topics_out:
            name = i['name']
            address = i['address']
            msg_type = i['msg_type']
            msg_module = i['msg_module']
            msg_type = get_attribute_from_module(msg_module, msg_type)
            self._act_pub[name] = rospy.Publisher(address, msg_type, queue_size=0)
            self._act_pub_reset[name] = rospy.Publisher(address + '/reset', UInt64, queue_size=0)

    def _init_observations(self, topics_in):
        for i in topics_in:
            name = i['name']
            address = i['address']
            msg_type = i['msg_type']
            msg_module = i['msg_module']
            msg_type = get_attribute_from_module(msg_module, msg_type)
            # self._obs_sub[name] = rospy.Subscriber(address, msg_type, partial(self.__obs_handler, name_topic))

    def _init_bridge(self, bridge_params: RxBridgeParams):
        # Prepare params
        params = bridge_params.get_params(ns=self.ns)
        name = params[list(params.keys())[0]]['name']
        launch_file = params[name]['launch_file']
        launch_locally = params[name]['launch_locally']
        single_process = params[name]['single_process']

        # Check if node name is unique
        assert rospy.get_param(self.ns + '/' + name, None) is None, 'Bridge name "%s" already exists. Node names must be unique.' % self.ns + '/' + name

        # Upload params to rosparam server
        rosparam.upload_params(self.ns, params)

        # Block env until bridge is initialized
        event = multiprocessing.Event()
        rospy.loginfo('Waiting for node "%s" to be initialized.' % (self.ns + '/' + name))
        rospy.Subscriber(self.ns + '/bridge/initialized', UInt64, lambda msg: event.set())

        # Initialize node
        if single_process:  # Initialize inside this process
            self._sp_nodes[self.ns + '/' + name] = RxBridge(name=self.ns + '/' + name, message_broker=self.mb, scheduler=None)
            self._sp_nodes[self.ns + '/' + name].node_initialized()
        else:
            if launch_locally and launch_file:  # Launch node as separate process
                self._launch_nodes[self.ns + '/' + name] = launch_node(launch_file, args=['name:=' + self.name])
                self._launch_nodes[self.ns + '/' + name].start()

        # Wait for bridge to be initialized
        event.wait()

    def _init_nodes(self, nodes_params: List[RxNodeParams]):
        # Upload parameters to ROS param server
        for node in nodes_params:
            params = node.get_params(ns=self.ns)
            #todo: how to robustly grasp node_name
            name = params[list(params.keys())[0]]['name']

            # Check if node name is unique
            assert rospy.get_param(self.ns + '/' + name, None) is None, 'Node name "%s" already exists. Node names must be unique.' % self.ns + '/' + name

            # Flag to check if node is initialized
            self.is_initialized[name] = False

            # Upload params to rosparam server
            rosparam.upload_params(self.ns, params)

        # Initialize nodes
        subs = []
        # thread_count = multiprocessing.cpu_count()
        # thread_pool_scheduler = ThreadPoolScheduler(thread_count)
        for node in nodes_params:
            params = node.__dict__
            name = params['name']
            launch_file = params['launch_file']
            launch_locally = params['launch_locally']
            single_process = params['single_process']

            # Block env until all nodes are initialized
            def initialized(msg, name):
                self.is_initialized[name] = True
            sub = rospy.Subscriber(self.ns + '/' + name + '/initialized', UInt64, partial(initialized, name=name))
            subs.append(sub)

            # Initialize node
            if single_process:  # Initialize inside this process
                self._sp_nodes[self.ns + '/' + name] = RxNode(name=self.ns + '/' + name, message_broker=self.mb, scheduler=None)
            else:
                if launch_locally and launch_file:  # Launch node as separate process
                    self._launch_nodes[self.ns + '/' + name] = launch_node(launch_file, args=['node_name:=' + name,
                                                                                              'name:=' + self.name])
                    self._launch_nodes[self.ns + '/' + name].start()

    def initialize_node_pipelines(self):
        if not self.initialized:
            # Wait for nodes to be initialized
            [node.node_initialized() for name, node in self._sp_nodes.items()]
            wait_for_node_initialization(self.is_initialized)

            # Initialize single process communication
            self.mb.connect_io(print_status=True)
            # self.mb.print_io_status()

            rospy.loginfo('Nodes initialized.')

            # Clear event, so that this thread is blocked until it receives '/N5/P5/reset'
            self.event.clear()
            rospy.sleep(0.01)

            # Send reset msg
            self.reset_pub.publish(UInt64())

            # After env receives '/rx/reset', we send '/rx/env/Pe/reset' via self.__reset_handler(msg)
            # Block until we receive '/N5/P5/reset'
            # todo: temporary, make dependent on env inputs
            # todo: currently blocking here because we send '/rx/start_reset' instead of '/rx/real_reset'
            self.event.wait()

            self.initialized = True
            rospy.loginfo("Pipelines initialized.")

    def register_object(self, object: RxObjectParams):
        # todo: There might be timing issues... Currently solved with condition.
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        params, nodes = object.get_params(ns=self.ns, bridge=self.bridge)

        # Check if object name is unique
        obj_name = list(params.keys())[0]
        assert rospy.get_param(self.ns + '/' + obj_name, None) is None, 'Object name "%s" already exists. Object names must be unique.' % self.ns + '/' + obj_name

        # Upload object params to rosparam server
        rosparam.upload_params(self.ns, params)

        # Upload parameters to ROS param server
        for node in nodes:
            params = node.get_params(ns=self.ns)
            # todo: how to robustly grasp node_name
            node_name = params[list(params.keys())[0]]['name']

            # Check if node name is unique
            assert rospy.get_param(self.ns + '/' + node_name, None) is None, 'Node name "%s" already exists. Node names must be unique.' % self.ns + '/' + node_name

            # Upload params to rosparam server
            rosparam.upload_params(self.ns, params)

        # Send register object request
        self.register_pub.publish(String(self.ns + '/' + obj_name))

    def step(self):
        # Check that nodes were previously initialized.
        assert self.initialized, 'Not yet initialized. Call .initialize_node_pipelines() before calling .step().'

        self.obs_recv = 0
        with self.cond_obs:
            for key, value in self._act_pub.items():
                msg = UInt64()
                msg.data = self.num_ticks
                value.publish(msg)
                self.cond_obs.wait()
        self.num_ticks += 1
        return None

    def reset(self):
        # Check that nodes were previously initialized.
        assert self.initialized, 'Not yet initialized. Call .initialize_node_pipelines() before calling .reset().'

        # Set done flag to False, to wait for real reset
        self.done = False
        self.event.clear()

        # Send state and real reset messages
        msg = UInt64()
        msg.data = self.num_ticks
        self.reset_pub.publish(msg)
        self._reset({self.state_name: msg})

        while not self.done:
            with self.cond_done:
                if self.done:
                    break
                self.step()
            rospy.sleep(0.0001)  # todo: 0.001 blocks single_process=True, because we are still stepping in env while done callback is running

        # Block until it receives '/N5/P5/reset'
        self.event.wait()
        self.num_ticks = 0
        rospy.loginfo("Reset performed")
        return None

    def close(self):
        for name in self._launch_nodes:
            self._launch_nodes[name].shutdown()
        try:
            rosparam.delete_param('/')
            rospy.loginfo('Pre-existing parameters under namespace "/" deleted.')
        except:
            pass

    def __obs_handler(self, msg):
        self.obs_recv += 1
        with self.cond_obs:
            if self.obs_recv == 8:
                self.cond_obs.notify_all()

    def __reset_handler(self, msg):
        for key, value in self._act_pub_reset.items():
                        msg = UInt64()
                        msg.data = self.num_ticks
                        value.publish(msg)

    def _reset(self, states):
        # Append namespace to names in states
        keys = list(states.keys())
        for key in keys:
            new_key = self.ns + '/' + key
            states[new_key] = states.pop(key)

        # Check if desired states were provided that cannot be reset (no corresponding node initialized)
        not_registered = [s for s in states.keys() if s not in self.states_reset.keys()]
        for s in not_registered:
            rospy.logwarn('State "%s" cannot be set as there is no (state)node initialized to reset it.' % s)

        # Check if desired states were provided for StateNodes that are initialized
        not_provided = [s for s in self.states_reset.keys() if s not in states.keys()]
        for s in not_provided:
            # Send True flag for states that were not provided
            msg = UInt64(data=1)
            self.states_reset[s]['pub_done'].publish(msg)

        provided = [s for s in self.states_reset.keys() if s in states.keys()]
        for s in provided:
            # Send desired state
            self.states_reset[s]['pub_set'].publish(states[s])

            # Send False flag for states that were not provided
            msg = UInt64(data=0)
            self.states_reset[s]['pub_done'].publish(msg)

    def _register_states(self,  msg, warn=False):
        state_address = msg.data

        if state_address not in self.states_reset.keys():
            self.states_reset[state_address] = {'name': state_address,
                                                'pub_set': rospy.Publisher(state_address + '/set', UInt64,
                                                                           queue_size=0),
                                                'pub_done': rospy.Publisher(state_address + '/done', UInt64,
                                                                            queue_size=0)}

            if warn:
                # If not yet registered, create publishers for the state.
                rospy.logwarn(
                    'State "%s" was not yet registered in environment "%s". Registered the publishers and paused '
                    'for a short moment (0.1 s). It is advisable to pre-register states to avoid a pause.' % (
                    state_address, self.name))
                rospy.sleep(0.1)
