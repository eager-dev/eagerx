import rospy
from std_msgs.msg import UInt64, String

# Rx imports
from eagerx_core.node import RxNode
from eagerx_core.utils.utils import get_attribute_from_module, launch_node, wait_for_node_initialization, get_param_with_blocking
import eagerx_core

# Memory usage
from functools import partial
from threading import current_thread, Condition
import os, psutil


class BridgeNode(object):
    def __init__(self, name, message_broker):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.params = get_param_with_blocking(self.name)

        # Initialize any simulator here, that can be used in each node
        # todo: Make a ThreadSafe simulator object
        self.simulator = None

        # Initialized nodes
        self.is_initialized = dict()

        # Message counter
        self.params = get_param_with_blocking(self.name)
        self.num_ticks = 0
        self.num_resets = 0
        self.dt = 1 / self.params['rate']

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20

    def register_object(self, obj_params):
        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        subs = []
        for node_params in obj_params:
            name = node_params['name']
            launch_file = node_params['launch_file']
            launch_locally = node_params['launch_locally']
            single_process = node_params['single_process']
            assert single_process, 'Only single_process simulation nodes are supported.'

            # Flag to check if node is initialized
            self.is_initialized[name] = False

            # Block env until all nodes are initialized
            def initialized(msg, name):
                self.is_initialized[name] = True
            sub = rospy.Subscriber(self.ns + '/' + name + '/initialized', UInt64, partial(initialized, name=name))
            subs.append(sub)

            # Initialize node (with reference to simulator)
            if single_process:  # Initialize inside this process
                sp_nodes[self.ns + '/' + name] = RxNode(name=self.ns + '/' + name, message_broker=self.mb,
                                                        scheduler=None, simulator=self.simulator)
            else:  # Not yet supported, because we cannot pass a reference to the simulator here.
                if launch_locally and launch_file:  # Launch node as separate process
                    launch_nodes[self.ns + '/' + name] = launch_node(launch_file, args=['node_name:=' + name, 'name:=' + self.ns])
                    launch_nodes[self.ns + '/' + name].start()

        [node.node_initialized() for name, node in sp_nodes.items()]
        return sp_nodes, launch_nodes

    def pre_reset(self, ticks):
        # todo:
        return 'PRE RESET RETURN VALUE'

    def post_reset(self):
        # todo:
        self.num_ticks = 0
        return 'POST RESET RETURN VALUE'

    def callback(self, topics_in):
        # Verify that # of ticks equals internal counter
        node_tick = topics_in['node_tick']
        if not self.num_ticks == node_tick:
            print('[%s]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * self.dt
        for i in self.params['topics_in']:
            name = i['name']
            if name in topics_in:
                t_i = topics_in[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    print('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks + 1
        for i in self.params['topics_out']:
            name = i['name']
            msg = UInt64()
            msg.data = Nc
            output_msgs[name] = msg
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs


class RxBridge(object):
    def __init__(self, name, message_broker, scheduler=None):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = message_broker
        self.initialized = False

        # Prepare input & output topics
        dt, topics_in, topics_out, self.bridge = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects = eagerx_core.init_bridge(self.ns, dt, self.bridge.callback, self.bridge.pre_reset,
                                             self.bridge.post_reset, self.bridge.register_object,
                                             topics_in, topics_out, self.mb, node_name=self.name, scheduler=scheduler)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)
        self.mb.connect_io()
        self.cond_reg = Condition()

        # Prepare closing routine
        rospy.on_shutdown(self._close)

    def node_initialized(self):
        with self.cond_reg:
            # Wait for all nodes to be initialized
            wait_for_node_initialization(self.bridge.is_initialized)

            # Notify env that node is initialized
            if not self.initialized:
                init_pub = rospy.Publisher(self.name + '/initialized', UInt64, queue_size=0, latch=True)
                init_pub.publish(UInt64(data=1))
                rospy.loginfo('Node "%s" initialized.' % self.name)
                self.initialized = True

    def _prepare_io_topics(self, name):
        # params = rospy.get_param(name)
        params = get_param_with_blocking(name)
        rate = params['rate']
        dt = 1 / rate

        # Get node
        node_cls = get_attribute_from_module(params['module'], params['node_type'])
        node = node_cls(name, self.mb)

        # Prepare input topics
        for i in params['topics_in']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

        # Prepare output topics
        for i in params['topics_out']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

        return dt, params['topics_in'], tuple(params['topics_out']), node

    def _close(self):
        return True
