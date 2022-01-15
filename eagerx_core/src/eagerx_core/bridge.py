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
from genpy.message import Message

# EAGERx IMPORTS
from eagerx_core.constants import process
from eagerx_core.nodes import NodeBase
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.utils.utils import initialize_state, Msg


class BridgeBase(NodeBase):
    def __init__(self, simulator, target_addresses, node_names, **kwargs):
        super(BridgeBase, self).__init__(**kwargs)
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
        params = self.add_object_to_simulator(object_params, node_params, state_params)

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
        node_args = dict(object_params=params, simulator=self.simulator)
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
        output = self.callback(node_tick, t_n, **kwargs)
        self.num_ticks += 1
        return output

    @abc.abstractmethod
    def add_object_to_simulator(self, object_params, node_params, state_params) -> Dict:
        pass

    @abc.abstractmethod
    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    @abc.abstractmethod
    def reset(self, **kwargs: Optional[Msg]):
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        pass
