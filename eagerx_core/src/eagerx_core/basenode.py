import abc
import os
import time
from typing import Dict, List, Union, Optional
from tabulate import tabulate
import logging

import numpy as np
import psutil
import rospy
from std_msgs.msg import UInt64, String, Bool
from genpy.message import Message
from sensor_msgs.msg import Image

from eagerx_core.constants import TERMCOLOR, ERROR, INFO, DEBUG, SILENT
from eagerx_core.utils.utils import initialize_converter, return_typehint, Msg
from eagerx_core.srv import ImageUInt8, ImageUInt8Response


class NodeBase:
    def __init__(self, ns, message_broker, name, config_name, package_name, node_type, rate, process,
                 inputs, outputs, states, feedthroughs,  targets, is_reactive, real_time_factor, launch_file=None,
                 color='grey', print_mode=TERMCOLOR, log_level=ERROR, log_level_memory=SILENT):
        """
        All parameters that were uploaded via RxNodeParams.get_params(ns=..) to the rosparam server are stored in this object.
        Optional arguments are added, and may not necessarily be uploaded via the rosparam server.
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
        self.color = color
        self.print_mode = print_mode
        self.log_level = log_level
        effective_log_level = logging.getLogger('rosout').getEffectiveLevel()
        self.log_memory = effective_log_level >= log_level and log_level_memory >= effective_log_level

    @staticmethod
    def get_msg_type(cls, component, cname):
        return cls.msg_types[component][cname]


class Node(NodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        """
        All arguments, in addition to the arguments to NodeBase, must be added here as (optional) args.
        Not specifying them here as argument in the constructor will raise an error.
        Make sure to pass down all required arguments to NodeBase.
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

    @abc.abstractmethod
    def reset(self, **kwargs: Optional[Message]) -> return_typehint(Message, done=True):
        """
        All states in the .yaml must be defined as optional arguments in the node subclass.
        If additional states may be specified later-on (not yet specified in yaml), then **kwargs must remain in the
        reset signature of the subclass.
        Additional states can then be found in the **kwargs object.
        """
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]) -> return_typehint(Message, done=True):
        """
        All inputs in the .yaml must be defined as optional arguments in the node subclass.
        If an input is required, add an assert that checks that input is not None.
        If additional inputs may be specified later-on (not yet specified in yaml), then **kwargs must remain in the
        callback signature of the subclass.
        Additional inputs can then be found in the **kwargs object.
        """
        pass


class SimNode(Node):
    def __init__(self, simulator=None, object_params=None, **kwargs):
        self.simulator = simulator
        self.object_params = object_params
        super().__init__(**kwargs)

    @abc.abstractmethod
    def reset(self, **kwargs: Optional[Message]):
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        pass


class ObservationsNode(Node):
    msg_types = {'inputs': {'actions_set': UInt64},
                 'outputs': {'set': UInt64}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Define observation buffers
        self.observation_buffer = dict()
        for i in self.inputs:
            if i['name'] == 'actions_set':
                continue
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.observation_buffer[i['name']] = {'msgs': None, 'converter': converter}

    def reset(self):
        # Set all messages to None
        for name, buffer in self.observation_buffer.items():
            buffer['msgs'] = None

    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Set all observations to messages in inputs
        for name, buffer in self.observation_buffer.items():
            buffer['msgs'] = kwargs[name].msgs

        # Send output_msg
        output_msgs = dict(set=UInt64())
        return output_msgs


class ActionsNode(Node):
    msg_types = {'inputs': {'observations_set': UInt64,
                            'step': UInt64},
                 'outputs': {'set': UInt64}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Define action/observation buffers
        self.action_buffer = dict()
        for i in self.outputs:
            if i['name'] == 'set':
                continue
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.action_buffer[i['name']] = {'msg': None, 'converter': converter}

    def reset(self):
        # Set all messages to None
        for name, buffer in self.action_buffer.items():
            buffer['msg'] = None
        # start_with an initial action message, so that the first observation can pass.
        return dict(set=UInt64())

    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Fill output_msg with buffered actions
        output_msgs = dict(set=UInt64())
        for name, buffer in self.action_buffer.items():
            output_msgs[name] = buffer['msg']
        return output_msgs


class RenderNode(Node):

    msg_types = {'inputs': {'image': Image},
                 'outputs': {'done': UInt64}}

    def __init__(self, display, **kwargs):
        super().__init__(**kwargs)
        self.display = display
        self.last_image = Image(data=[])
        self.render_toggle = False
        rospy.Service('%s/%s/get_last_image' % (self.ns, self.name), ImageUInt8, self._get_last_image)
        rospy.Subscriber('%s/%s/toggle' % (self.ns, self.name), Bool, self._set_render_toggle)

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo('START RENDERING!')
        else:
            rospy.loginfo('STOP RENDERING!')
        self.render_toggle = msg.data

    def _get_last_image(self, req):
        return ImageUInt8Response(image=self.last_image)

    def reset(self):
        self.last_image = Image()

    def callback(self, node_tick: int, t_n: float, image: Optional[Msg] = None):
        self.last_image = image.msgs[-1]
        if self.display and self.render_toggle:
            rospy.logwarn_once('Displaying functionality inside the render node has not yet been implemented.')

        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=UInt64)
        return output_msgs


class RealResetNode(Node):
    msg_types = {'inputs': {'in_1': UInt64,
                            'in_2': UInt64},
                 'outputs': {'out_1': UInt64,
                             'out_2': UInt64},
                 'states': {'state_1': UInt64},
                 'targets': {'target_1': UInt64}}

    def __init__(self, test_arg, **kwargs):
        super().__init__(**kwargs)

    def reset(self, state_1: Optional[UInt64] = None) -> None:
        return

    def callback(self, node_tick: int, t_n: float, in_1: Optional[Msg] = None, in_2: Optional[Msg] = None, target_1: Optional[Msg] = None) -> return_typehint(UInt64, done=True):
        # output type is always Dict[str, Union[UInt64, output_msg_types]] because done flags are also inside the output_msgs
        inputs = {'in_1': in_1,
                  'in_2': in_2}
        pop_keys = []
        for key, value in inputs.items():
            if value is None: pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in inputs:
                t_i = inputs[name].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.outputs:
            name = i['name']
            output_msgs[name] = UInt64(data=Nc)

        # Fill state done msg with number of node ticks
        for i in self.targets:
            name = i['name']
            msg = UInt64()
            if self.num_ticks > 2:
                msg.data = 1
            else:
                msg.data = 0
            output_msgs[name + '/done'] = msg
        return output_msgs


class ProcessNode(SimNode):
    msg_types = {'inputs': {'in_1': UInt64,
                            'in_2': UInt64,
                            'in_3': String,
                            'tick': UInt64},
                 'outputs': {'out_1': UInt64,
                             'out_2': UInt64},
                 'states': {'state_1': UInt64,
                            'state_2': UInt64}}

    def __init__(self, test_arg, **kwargs):
        super().__init__(**kwargs)

    def reset(self, state_1: Optional[UInt64] = None, state_2: Optional[UInt64] = None) -> return_typehint(UInt64):
        # Send initial message for outputs with 'start_with_msg' = True
        init_msgs = dict()
        for i in self.outputs:
            if not i['start_with_msg']: continue
            name = i['name']
            init_msgs[name] = UInt64(data=999)
        return init_msgs

    def callback(self, node_tick: int, t_n: float, in_1: Optional[Msg] = None, in_2: Optional[Msg] = None, in_3: Optional[Msg] = None, tick: Optional[Msg] = None) -> return_typehint(UInt64):
        inputs = {'in_1': in_1,
                  'in_2': in_2,
                  'tick': tick}
        pop_keys = []
        for key, value in inputs.items():
            if value is None: pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that # of ticks equals internal counter
        if not self.num_ticks == node_tick:
            rospy.logerr('[%s][callback]: ticks not equal (self.num_ticks=%d, node_tick=%d).' % (self.name, self.num_ticks, node_tick))
            pass

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in inputs:
                t_i = inputs[name].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.outputs:
            name = i['name']
            output_msgs[name] = UInt64(data=Nc)
        return output_msgs