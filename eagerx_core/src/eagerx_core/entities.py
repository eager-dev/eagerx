from typing import List, Dict, Optional, Union, Any
import abc
import inspect
import logging
import os
import time
from copy import deepcopy
from yaml import dump

import numpy as np
import psutil
import rospy
from genpy import Message
from std_msgs.msg import Bool, UInt64
from tabulate import tabulate

from eagerx_core.constants import TERMCOLOR, ERROR, SILENT, process
from eagerx_core.rxmessage_broker import RxMessageBroker
from eagerx_core.specs import EntitySpec, BaseNodeSpec, SimNodeSpec, ObjectSpec, ConverterSpec, BridgeSpec, NodeSpec, SimStateSpec, ResetNodeSpec
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.utils.utils import Msg, initialize_state, check_valid_rosparam_type


class Entity(object):
    @classmethod
    def make(cls, id, *args, **kwargs):
        from eagerx_core import registration
        spec = registration.make(cls, id, *args, **kwargs)
        try:
            cls.check_spec(spec)
        except AssertionError as e:
            print(e)
            raise
        return spec

    @classmethod
    def get_spec(cls, id):
        from eagerx_core import registration
        return registration.get_spec(cls, id)

    @classmethod
    def pre_make(cls, entity_type):
        return EntitySpec(dict(entity_type=entity_type))

    @classmethod
    def check_spec(cls, spec):
        pass


class BaseNode(Entity):
    def __init__(self, ns: str, message_broker: RxMessageBroker, name: str, config_name: str, package_name: str,
                 node_type: str, rate: float, process: int, inputs: List[Dict], outputs: List[Dict], states: List[Dict],
                 feedthroughs: List[Dict], targets: List[Dict], is_reactive: bool, real_time_factor: float,
                 simulate_delays: bool, *args, launch_file=None, color: str = 'grey', print_mode: int = TERMCOLOR, log_level:
                 int = ERROR, log_level_memory: int = SILENT, **kwargs):
        """
        The base class from which all (simulation) nodes and bridges inherit.

        All parameters that were uploaded via RxNodeParams.get_params(ns=..) to the rosparam server are stored in this object.

        Optional arguments are added, and may not necessarily be uploaded to the rosparam server.

        :param ns: Namespace of the environment. Corresponds to argument "name" provided to eagerx_core.rxenv.RxEnv.
        :param message_broker: Responsible for all I/O communication within this process. Node possibly share the same message broker.
        :param name: User specified node name.
        :param config_name: Config file name. Relates to <package_name>/config/../<config_name>.yaml
        :param package_name: ROS package name. Relates to <package_name>/config/../<config_name>.yaml
        :param node_type: The python implementation used by this node. Follows naming convention <module>/<NodeClassName>
        :param rate: Rate at which this node's callback is run.
        :param process: Process in which this node is launched. See :func:`~eagerx_core.constants.process` for all options.
        :param inputs: List of dicts containing the parameters of each input as specified in the <package_name>/config/../<config_name>.yaml.
        :param outputs: List of dicts containing the parameters of each output as specified in the <package_name>/config/../<config_name>.yaml.
        :param states: List of dicts containing the parameters of each state as specified in the <package_name>/config/../<config_name>.yaml.
        :param feedthroughs: List of dicts containing the parameters of each feedthrough.
        :param targets: List of dicts containing the parameters of each target as specified in the <package_name>/config/../<config_name>.yaml.
        :param is_reactive: Boolean flag. Specifies whether we run reactive or asynchronous.
        :param real_time_factor: Sets an upper bound of real_time factor. Wall-clock rate=real_time_factor*rate. If real_time_factor < 1 the simulation is slower than real time.
        :param simulate_delays: Simulate delays. You probably want to set this to False if running in the real-world.
        :param launch_file:
        :param color: A color specifying the color of logged messages & node color in the GUI.
        :param print_mode: Specifies the different methods for printing. See :func:`~eagerx_core.constants` for all print modes.
        :param log_level: Overall log level of this node. See :func:`~eagerx_core.constants` for all log levels.
        :param log_level_memory: Log level of memory diagnostics. See :func:`~eagerx_core.constants` for all log levels.
        """
        self.ns = ns
        self.name = name
        self.ns_name = '%s/%s' % (ns, name)
        self.message_broker = message_broker
        self.config_name = config_name
        self.package_name = package_name
        self.node_type = node_type
        self.rate = rate
        self.process = process
        self.launch_file = launch_file
        self.inputs = inputs
        self.outputs = outputs
        self.states = states
        self.feedthroughs = feedthroughs
        self.targets = targets
        self.is_reactive = is_reactive
        self.real_time_factor = real_time_factor
        self.simulate_delays = simulate_delays
        self.color = color
        self.print_mode = print_mode
        self.log_level = log_level
        effective_log_level = logging.getLogger('rosout').getEffectiveLevel()
        self.log_memory = effective_log_level >= log_level and log_level_memory >= effective_log_level
        self.initialize(*args, **kwargs)

    @staticmethod
    def get_msg_type(cls, component, cname):
        return cls.msg_types[component][cname]

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        params['node_type'] = params.pop('entity_type')
        params['default'] = dict(name=None, rate=None, process=0, inputs=[], outputs=[], states=[], color='grey',
                                 print_mode=TERMCOLOR, log_level=ERROR, log_level_memory=SILENT, launch_file=None)
        params.update(dict(inputs=dict(), outputs=dict(), states=dict()))
        return BaseNodeSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        return

    @abc.abstractmethod
    def initialize(self, *args, **kwargs):
        """A method to initialize this node."""
        pass


class Node(BaseNode):
    def __init__(self, **kwargs):
        """
        The node base class from which all nodes must inherit.

        Users are only expected to interact with the constructor & abstract methods of this class.

        Note that all node subclasses must:
        - Implement the abstract methods
        - Pass down all arguments (possibly inside kwargs), that are required by the node baseclass' constructor (NodeBase).
        - Define a static property that specifies the msg_type for every input, output, state and/or target.

        Example of such a static property:
        msg_types = {'inputs': {'in_1': UInt64,
                                'in_2': UInt64,
                                'in_3': String,
                                'tick': UInt64},
                     'outputs': {'out_1': UInt64,
                                 'out_2': UInt64},
                     'states': {'state_1': UInt64,
                                'state_2': UInt64}}

        :param kwargs: Arguments that are to be passed down to the baseclass. See NodeBase for this.
        """
        # Message counter
        self.num_ticks = 0

        # Track memory usage  & speed
        self.total_ticks = 0
        self.py = psutil.Process(os.getpid())
        self.pid = os.getpid()
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 200
        self.history = []
        self.headers = ["pid", "node", "ticks", "rss", "diff", "t0", "vms", "diff", "t0", "iter_time", "diff", "t0"]

        # Call baseclass constructor (which eventually calls .initialize())
        super().__init__(**kwargs)

    def reset_cb(self, **kwargs: Optional[Message]):
        self.num_ticks = 0
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg.info.done:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg.msgs[0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.reset(**kwargs)

    def callback_cb(self, **kwargs):
        self.iter_ticks += 1
        if self.log_memory and self.iter_ticks % self.print_iter == 0:
            if self.iter_start:
                # Time steps
                iter_stop = time.time()
                if self.iter_ticks > 0:
                    iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                else:
                    iter_time = float(iter_stop - self.iter_start)

                # Memory usage request
                mem_use = (np.array(self.py.memory_info()[0:2]) / 2. ** 30) * 1000  # memory use in MB...I think

                # Print info
                self.total_ticks += self.iter_ticks
                self.iter_ticks = 0

                # Log history
                if len(self.history) > 0:
                    delta_mem_rss = round(mem_use[0] - self.history[-1][3], 2)
                    delta_mem_vms = round(mem_use[1] - self.history[-1][6], 2)
                    delta_iter_time = round(iter_time - self.history[-1][9], 5)
                    cum_mem_rss = round(mem_use[0] - self.history[0][3], 2)
                    cum_mem_vms = round(mem_use[1] - self.history[0][6], 2)
                    cum_iter_time = round(iter_time - self.history[0][9], 5)
                    self.history.append([self.pid, self.name, self.total_ticks,
                                         round(mem_use[0], 1), delta_mem_rss, cum_mem_rss,
                                         round(mem_use[1], 1), delta_mem_vms, cum_mem_vms,
                                         iter_time, delta_iter_time, cum_iter_time])
                else:
                    self.history.append([self.pid, self.name, self.total_ticks, round(mem_use[0], 1), 0, 0, round(mem_use[1], 1), 0, 0, iter_time, 0, 0])
                rospy.loginfo('\n' + tabulate(self.history, headers=self.headers))
            self.iter_start = time.time()
        output = self.callback(**kwargs)
        self.num_ticks += 1
        return output

    def set_delay(self, delay: float, component: str, cname: str):
        assert delay >= 0, 'Delay must be non-negative.'
        for i in getattr(self, component):
            if i['name'] == cname:
                i['delay'] = delay

    @abc.abstractmethod
    def reset(self, **kwargs: Optional[Message]):
        """
        A method to reset this simulation node.

        Could be used to:
        - reset the internal state of this node if it has one.

        :param kwargs: Optionally the node states if any are defined in the node_ros_package/config/<node>.yaml.
        :return None
        """
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]) -> Dict[str, Union[Message, Bool]]:
        """
        The node callback that is performed at the specified node rate.

        All inputs specified in the package/config/<node>.yaml must be defined as optional arguments to this callback method.

        :param node_tick: The number of times this callback has run since the last reset.
        :param t_n: Time passed since last reset according to the provided rate (t_n = node_tick * 1/self.rate).
        :param kwargs: All selected inputs specified in package/config/<object>.yaml under <bridge>/<component>/<cname>.
        :return: A dict containing output messages.
        """
        pass

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        spec.set_parameter('launch_file', '$(find eagerx_core)/launch/rxbridge.launch')
        return NodeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class ResetNode(Node):
    """Reset node baseclass from which all nodes that perform a real reset routine inherit"""
    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        spec._params['targets'] = dict()
        spec._set({'default': dict(targets=[])})
        return ResetNodeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class SimNode(Node):
    """
    The simulation node baseclass from which all nodes, used to simulate sensors/actuators, must inherit.

    Users are only expected to interact with the constructor & abstract methods of this class.

    Note that all simulation node subclasses must:
     - Implement the abstract methods
     - Create a (placeholder) simulator object, that is passed down to the baseclass' constructor.
     - Pass down all arguments (possibly inside kwargs), that are required by the node baseclasses' constructor (see Node, NodeBase).
     - Define a static property that specifies the msg_type for every input, output, state and/or target.

    Example of such a static property:
    msg_types = {'inputs': {'in_1': UInt64,
                            'in_2': UInt64,
                            'in_3': String,
                            'tick': UInt64},
                 'outputs': {'out_1': UInt64,
                             'out_2': UInt64},
                 'states': {'state_1': UInt64,
                            'state_2': UInt64}}

    For more info see baseclasses Node and NodeBase.
    """
    def __init__(self, simulator: Any = None, object_params: Dict = None, **kwargs):
        """
        Simulation node class constructor.

        Note: This node only has access to the object_params & simulator if the node is launched inside the same
        process as the bridge.

        :param simulator: Simulator object. Passed along by the bridge if the node is launched inside the bridge process.
        :param object_params: A dictionary containing the following: First, all the parameters defined under "default"
        in the package/config/<object>.yaml. Secondly, it contains all object parameters that are specific for this bridge
        implementation under the keyword 'bridge". These are the parameters defined under "<bridge>" in the object_package/config/<object>.yaml.
        :param kwargs: Arguments that are to be passed down to the baseclass. See Node & NodeBase for this.
        """
        self.simulator = simulator
        self.object_params = object_params

        # Call baseclass constructor (which eventually calls .initialize())
        super().__init__(**kwargs)

    @abc.abstractmethod
    def reset(self, **kwargs: Optional[Message]) -> Optional[Dict[str, Message]]:
        """
        A method to reset this simulation node.

        Could be used to:
        - reset the internal state of a sensor/actuator.

        Important: Be careful to define states for simulation nodes, as you risk making your environment non-agnostic.
        Instead, always try to implement states of the environment as states of objects (i.e. inside the object's .yaml).

        :param kwargs: Optionally the node states if any are defined in the node_ros_package/config/<node>.yaml.
        :return None
        """
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]) -> Dict[str, Message]:
        """
        The simulation node callback that is performed at the specified node rate.

        All inputs specified in the package/config/<simnode>.yaml must be defined as optional arguments to this callback method.

        :param node_tick: The number of times this callback has run since the last reset.
        :param t_n: Time passed since last reset according to the provided rate (t_n = node_tick * 1/self.rate).
        :param kwargs: All selected inputs specified in package/config/<object>.yaml under <bridge>/<component>/<cname>.
        :return: A dict containing output messages.
        """
        pass

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        return SimNodeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class Bridge(BaseNode):
    """
    The bridge baseclass from which all bridges must inherit.

    Users are only expected to interact with the constructor & abstract methods of this class.

    Note that all bridge subclasses must:
     - Implement the abstract methods
     - Create a (placeholder) simulator object, that is passed down to the baseclass' constructor.
     - Pass down all arguments (possibly inside kwargs), that are required by the baseclass' constructor (NodeBase).
     - Define a static property that specifies the msg_type for every output and/or state.

        Example of such a static property:
        msg_types = {'outputs': {'tick': UInt64},
                     'states': {'state_1': UInt64}}
    For more info see baseclass NodeBase.
    """
    def __init__(self, target_addresses, node_names, is_reactive, real_time_factor, **kwargs):
        """
        Base class constructor.

        Note: Be sure to create a (placeholder) simulator object in the subclass' constructor, that is passed down
        to this constructor.

        :param simulator: Simulator object
        :param target_addresses: Addresses from which the bridge should expect "done" msgs.
        :param node_names: List of node names that are registered in the graph.
        :param is_reactive: Boolean flag. Specifies whether we run reactive or asynchronous.
        :param real_time_factor: Sets an upper bound of real_time factor. Wall-clock rate=real_time_factor*rate.
         If real_time_factor < 1 the simulation is slower than real time.
        :param kwargs: Arguments that are to be passed down to the node baseclass. See NodeBase for this.
        """
        self.simulator = None
        self.target_addresses = target_addresses
        self.node_names = node_names

        # Check real_time_factor & reactive args
        assert is_reactive or (not is_reactive and real_time_factor > 0), 'Cannot have a real_time_factor=0 while not reactive. Will result in synchronization issues. Set is_reactive=True or real_time_factor > 0'

        # Initialized nodes
        self.is_initialized = dict()

        # Message counter
        self.num_ticks = 0

        # Track memory usage  & speed
        self.total_ticks = 0
        self.py = psutil.Process(os.getpid())
        self.pid = os.getpid()
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 200
        self.history = []
        self.headers = ["pid", "node", "ticks", "rss", "diff", "t0", "vms", "diff", "t0", "iter_time", "diff", "t0"]

        # Call baseclass constructor (which eventually calls .initialize())
        super().__init__(is_reactive=is_reactive, real_time_factor=real_time_factor, **kwargs)

    def register_node(self, node_params):
        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        initialize_nodes(node_params, process.BRIDGE, self.ns, self.ns, self.message_broker, self.is_initialized, sp_nodes, launch_nodes)
        for name, node in sp_nodes.items():
            # Set simulator
            if hasattr(node.node, 'set_simulator'):
                node.node.set_simulator(self.simulator)
            # Initialize
            node.node_initialized()
        wait_for_node_initialization(self.is_initialized)
        return node_params, sp_nodes, launch_nodes

    def register_object(self, object_params, node_params, state_params):
        # Use obj_params to initialize object in simulator --> object info parameter dict is optionally added to simulation nodes
        _ = self.add_object(object_params, node_params, state_params)

        # Initialize states
        for i in state_params:
            i['state']['name'] = i['name']
            i['state']['simulator'] = self.simulator
            i['state']['object_params'] = object_params
            i['state']['ns'] = self.ns
            i['state'] = initialize_state(i['state'])

        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        node_args = dict(object_params=object_params, simulator=self.simulator)
        initialize_nodes(node_params, process.BRIDGE, self.ns, self.ns, self.message_broker, self.is_initialized, sp_nodes, launch_nodes, node_args=node_args)
        for name, node in sp_nodes.items():
            # Initialize
            node.node_initialized()
        wait_for_node_initialization(self.is_initialized)
        return state_params, sp_nodes, launch_nodes

    def pre_reset_cb(self, **kwargs):
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg.info.done:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg.msgs[0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.pre_reset(**kwargs)

    def reset_cb(self, **kwargs):
        self.num_ticks = 0
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg.info.done:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg.msgs[0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.reset(**kwargs)

    def callback_cb(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        self.iter_ticks += 1
        if self.log_memory and self.iter_ticks % self.print_iter == 0:
            if self.iter_start:
                # Time steps
                iter_stop = time.time()
                if self.iter_ticks > 0:
                    iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                else:
                    iter_time = float(iter_stop - self.iter_start)

                # Memory usage request
                mem_use = (np.array(self.py.memory_info()[0:2]) / 2. ** 30) * 1000  # memory use in MB...I think

                # Print info
                self.total_ticks += self.iter_ticks
                self.iter_ticks = 0

                # Log history
                if len(self.history) > 0:
                    delta_mem_rss = round(mem_use[0] - self.history[-1][3], 2)
                    delta_mem_vms = round(mem_use[1] - self.history[-1][6], 2)
                    delta_iter_time = round(iter_time - self.history[-1][9], 5)
                    cum_mem_rss = round(mem_use[0] - self.history[0][3], 2)
                    cum_mem_vms = round(mem_use[1] - self.history[0][6], 2)
                    cum_iter_time = round(iter_time - self.history[0][9], 5)
                    self.history.append([self.pid, self.name, self.total_ticks,
                                         round(mem_use[0], 1), delta_mem_rss, cum_mem_rss,
                                         round(mem_use[1], 1), delta_mem_vms, cum_mem_vms,
                                         iter_time, delta_iter_time, cum_iter_time])
                else:
                    self.history.append([self.pid, self.name, self.total_ticks, round(mem_use[0], 1), 0, 0, round(mem_use[1], 1), 0, 0, iter_time, 0, 0])
                rospy.loginfo('\n' + tabulate(self.history, headers=self.headers))
            self.iter_start = time.time()
        _ = self.callback(node_tick, t_n, **kwargs)

        # Fill output msg with number of node ticks
        self.num_ticks += 1
        return dict(tick=UInt64(data=node_tick + 1))

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        # Set default bridge params
        default = dict(name='bridge', is_reactive=True, real_time_factor=0, simulate_delays=True,
                       launch_file='$(find eagerx_core)/launch/rxbridge.launch', outputs=['tick'])
        spec._set({'default': default})
        return BridgeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)

    @abc.abstractmethod
    def add_object(self, object_params: Dict, node_params: List[Dict], state_params: List[Dict]) -> None:
        """
        Adds an object to the bridge's simulator object.

        This method varies for each simulator and how the simulator object is structured in the bridge uses.

        This method is called in .register_object(...).

        :param object_params: A dictionary containing the following: First, all the parameters defined under "default"
        in the package/config/<object>.yaml. Secondly, it contains all object parameters that are specific for this bridge
        implementation under the keyword 'bridge". These are the parameters defined under "<bridge>" in the object_package/config/<object>.yaml.
        :param node_params: A list containing the parameters of all the nodes that represent the object's simulated
        sensors & actuators.
        :param state_params: A list containing the parameters of all the simulated object states.
        :return: None
        """
        pass

    @abc.abstractmethod
    def pre_reset(self, **kwargs: Optional[Msg]) -> None:
        """
        A pre-reset method that is performed *before* all other (simulation) nodes are resetting.

        Could be useful to:
        - Perform some preliminary logic on the simulator such as pausing before resetting all states & nodes.
        - Reset the simulator states so that this state can be used in the simulation node reset procedures that follow.

        This method is called in .pre_reset_cb(...).

        :param kwargs: Optionally the bridge states if any are defined in the bridge_ros_package/config/<bridge>.yaml.
        :return: None
        """
        pass

    @abc.abstractmethod
    def reset(self, **kwargs: Optional[Msg]) -> None:
        """
        A reset method that is performed *after* all other (simulation) nodes have finished resetting.

        Could be useful to:
        - Perform some final logic on the simulator such as unpausing the simulator.

        This method is called in .reset_cb(...).

        :param kwargs: Optionally the bridge states if any are defined in the bridge_ros_package/config/<bridge>.yaml.
        :return: None
        """
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]) -> None:
        """
        The bridge callback that is performed at the specified rate.

        Note: The bridge *only* outputs the 'bridge/outputs/tick' after each callback. However, the user is not responsible
        for creating this message. This is taken care of outside of this method. Users cannot make the bridge output any
         other message. If you wish to broadcast output messages using info of the simulator, a simulation sensor node
         should be created.

        In this callback you could:
        - Step the simulator according to the specified rate.

        This method is called in .callback_cb(...).

        :param node_tick: The number of times this callback has run since the last reset.
        :param t_n: Time passed since last reset according to the provided rate (t_n = node_tick * 1/self.rate).
        :param kwargs: All outputs produced by the simulation nodes are defined as inputs to the bridge callback.
        Could be useful to inspect when debugging.
        :return: None
        """
        pass


class Object(Entity):
    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        params['default'] = dict(name=None, sensors=[], actuators=[], states=[])
        params['sensors'] = dict()
        params['actuators'] = dict()
        params['states'] = dict()
        return ObjectSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class BaseConverter(Entity):
    """
    Make sure to pass all arguments of the subclass' constructor through (**IMPORTANT**in the same order) to this
    baseclass' constructor and that it is of valid type: (str, int, list, float, bool, dict, NoneType).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        self.yaml_args = kwargs
        argspec = inspect.getfullargspec(self.__init__).args
        argspec.remove('self')
        for key, value in zip(argspec, args):
            self.yaml_args[key] = value
        check_valid_rosparam_type(self.yaml_args)
        self.initialize(*args, **kwargs)

    def get_yaml_definition(self):
        converter_type = self.__module__ + '/' + self.__class__.__name__
        yaml_dict = dict(converter_type=converter_type)
        yaml_dict.update(deepcopy(self.yaml_args))
        return yaml_dict

    @abc.abstractmethod
    def initialize(self):
        pass

    @abc.abstractmethod
    def convert(self, msg):
        pass

    @staticmethod
    @abc.abstractmethod
    def get_opposite_msg_type(cls, msg_type):
        pass

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        params['converter_type'] = params.pop('entity_type')
        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class Processor(BaseConverter):
    """
    Use this processor if the converted msg type is one-way and the msg_type after conversion is equal to
    the msg_type before conversion. In addition, make sure to specify the static attribute "MSG_TYPE".
    Make sure to pass all arguments of the subclass' constructor through (**IMPORTANT**in the same order) to this
    baseclass' constructor and that it is of valid type: (str, int, list, float, bool, dict, NoneType).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        if msg_type == cls.MSG_TYPE:
            return cls.MSG_TYPE
        else:
            raise ValueError('Message type "%s" not supported by this converter. Only msg_type "%s" is supported.' %(msg_type, cls.MSG_TYPE))

    def convert(self, msg):
        if isinstance(msg, self.MSG_TYPE):
            return self._convert(msg)
        else:
            raise ValueError('Message type "%s" not supported by this converter. Only msg_type "%s" is supported.' %(type(msg), self.MSG_TYPE))

    @abc.abstractmethod
    def _convert(self, msg):
        pass

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class Converter(BaseConverter):
    """
    Inherit your converter from this baseclass and implement the abstract methods. In addition, make sure to specify the
    static attributes "MSG_TYPE_A" and "MSG_TYPE_B", such that the correct conversion method can be inferred.
    Make sure to pass all arguments of the subclass' constructor through (**IMPORTANT**in the same order) to this
    baseclass' constructor and that it is of valid type: (str, int, list, float, bool, dict, NoneType).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        if msg_type == cls.MSG_TYPE_A:
            return cls.MSG_TYPE_B
        elif msg_type == cls.MSG_TYPE_B:
            return cls.MSG_TYPE_A
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.' %
                (msg_type, cls.MSG_TYPE_A, cls.MSG_TYPE_B))

    def convert(self, msg):
        if isinstance(msg, self.MSG_TYPE_A):
            return self.A_to_B(msg)
        elif isinstance(msg, self.MSG_TYPE_B):
            return self.B_to_A(msg)
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.' %
                (type(msg), self.MSG_TYPE_A, self.MSG_TYPE_B))

    @abc.abstractmethod
    def A_to_B(self, msg):
        pass

    @abc.abstractmethod
    def B_to_A(self, msg):
        pass

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class SpaceConverter(Converter):
    """
    Inherit your converter from this baseclass if the converter is used for actions/observations/states,
    such that the space can be inferred. See Converter for other abstract methods that must be implemented.
    """
    @abc.abstractmethod
    def get_space(self):
        pass

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class SimState(Entity):
    def __init__(self, ns, name, simulator, object_params, *args, color='grey', print_mode='termcolor', **kwargs):
        self.ns = ns
        self.name = name

        # If node is simulator, we will probably use this in reset
        self.simulator = simulator
        self.object_params = object_params
        self.color = color
        self.print_mode = print_mode
        self.initialize(*args, **kwargs)

    @classmethod
    def pre_make(cls, entity_type):
        spec = super().pre_make(entity_type)
        params = spec.params
        params['state_type'] = params.pop('entity_type')
        return SimStateSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        return

    @abc.abstractmethod
    def initialize(self, *args, **kwargs):
        """A method to initialize this simstate."""
        pass

    @abc.abstractmethod
    def reset(self, state, done):
        """A Method to reset the simstate."""
        pass