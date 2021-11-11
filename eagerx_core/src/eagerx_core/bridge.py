import rospy
from std_msgs.msg import UInt64

# Rx imports
from eagerx_core.utils.utils import get_attribute_from_module, get_param_with_blocking, initialize_converter, initialize_state
from eagerx_core.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx_core.converter import IdentityConverter
import eagerx_core

# Memory usage
from threading import Condition
import os, psutil


class BridgeNode(object):
    def __init__(self, name, **kwargs):
        self.name = name
        self.ns = '/'.join(name.split('/')[:2])
        self.mb = None
        self.params = get_param_with_blocking(self.name)

        # Initialize any simulator here, that is passed as reference to each simnode
        self.simulator = None  # todo: Make a ThreadSafe simulator object

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

    def set_message_broker(self, message_broker):
        self.mb = message_broker

    def register_object(self, obj_params, node_params, state_params):
        assert self.mb is not None, 'Message broker for this bridge (%s) was not set. Must be set, before an object can be registered.'

        # Use obj_params to initialize object in simulator
        obj_params

        # Initialize states
        for i in state_params:
            i['state']['name'] = i['name']
            i['state']['simulator'] = self.simulator
            i['state'] = initialize_state(i['state'])

        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        initialize_nodes(node_params, self.ns, self.name, self.mb, self.is_initialized, sp_nodes, launch_nodes)
        for name, node in sp_nodes.items():
            # Set object parameters
            if hasattr(node.node, 'set_object_params'):
                node.node.set_object_params(obj_params)
            # Set simulator
            if hasattr(node.node, 'set_simulator'):
                node.node.set_simulator(self.simulator)
            # Initialize
            node.node_initialized()
        return state_params, sp_nodes, launch_nodes

    def pre_reset(self, ticks):
        return 'PRE RESET RETURN VALUE'

    def post_reset(self):
        self.num_ticks = 0
        return 'POST RESET RETURN VALUE'

    def callback(self, inputs):
        # Verify that # of ticks equals internal counter
        node_tick = inputs['node_tick']
        if not self.num_ticks == node_tick:
            print('[%s]: ticks not equal (%d, %d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * self.dt
        for i in self.params['inputs']:
            name = i['name']
            if name in inputs:
                t_i = inputs[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    print('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks + 1
        for i in self.params['outputs']:
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
        dt, inputs, outputs, self.bridge = self._prepare_io_topics(self.name)

        # Initialize reactive pipeline
        rx_objects = eagerx_core.init_bridge(self.ns, dt, self.bridge.callback, self.bridge.pre_reset,
                                             self.bridge.post_reset, self.bridge.register_object,
                                             inputs, outputs, self.mb, node_name=self.name, scheduler=scheduler)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

        # todo: resolve in a clean manner:
        #  Currently, we add '/realreset' to avoid a namespace clash between done flags used in both real_reset & state_reset
        self.mb.add_rx_objects(node_name=name + '/realreset', node=self)
        self.mb.connect_io()
        self.cond_reg = Condition()  # todo: remove?

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

    def _prepare_io_topics(self, name, **kwargs):
        params = get_param_with_blocking(name)
        rate = params['rate']
        dt = 1 / rate

        # Get node
        node_cls = get_attribute_from_module(params['module'], params['node_type'])
        node = node_cls(name)

        # Set message broker
        if hasattr(node, 'set_message_broker'):
            node.set_message_broker(self.mb)

        # Prepare input topics
        for i in params['inputs']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i:
                i['converter'] = initialize_converter(i['converter'])
            else:
                i['converter'] = IdentityConverter()

        # Prepare output topics
        for i in params['outputs']:
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            if 'converter' in i:
                i['converter'] = initialize_converter(i['converter'])
            else:
                i['converter'] = IdentityConverter()

        return dt, params['inputs'], tuple(params['outputs']), node

    def _close(self):
        return True
