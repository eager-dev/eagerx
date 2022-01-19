# OTHER IMPORTS
import abc
import os
from typing import Dict, Union, List, Optional
import psutil
import time
import numpy as np
from tabulate import tabulate

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# EAGERx IMPORTS
from eagerx_core.constants import process
from eagerx_core.nodes import NodeBase
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.utils.utils import initialize_state, Msg


class BridgeBase(NodeBase):
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
    def __init__(self, simulator, target_addresses, node_names, **kwargs):
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
        super().__init__(**kwargs)
        self.simulator = simulator
        self.target_addresses = target_addresses
        self.node_names = node_names

        # Check real_time_factor & reactive args
        assert self.is_reactive or (not self.is_reactive and self.real_time_factor > 0), 'Cannot have a real_time_factor=0 while not reactive. Will result in synchronization issues. Set is_reactive=True or real_time_factor > 0'

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
        _ = self.add_object_to_simulator(object_params, node_params, state_params)

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

    @abc.abstractmethod
    def add_object_to_simulator(self, object_params: Dict, node_params: List[Dict], state_params: List[Dict]) -> None:
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
