import abc
import os
import time
from typing import Dict, List, Union

import numpy as np
import psutil
import rospy
from std_msgs.msg import UInt64, String, Bool
from genpy.message import Message
from sensor_msgs.msg import Image

from eagerx_core.constants import TERMCOLOR, ERROR
from eagerx_core.utils.utils import initialize_converter
from eagerx_core.srv import ImageUInt8, ImageUInt8Response

class NodeBase:
    def __init__(self, ns, message_broker, name, config_name, package_name, node_type, module, rate, process,
                 inputs, outputs, states, feedthroughs,  targets, is_reactive, real_time_factor, launch_file=None,
                 color='grey', print_mode=TERMCOLOR, log_level=ERROR):
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
        self.module = module
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


class Node(NodeBase):
    def __init__(self, **kwargs):
        """
        All arguments, in addition to the arguments to NodeBase, must be added here as (optional) args.
        Not specifying them here as argument in the constructor will raise an error.
        Make sure to pass down all required arguments to NodeBase.
        """
        super().__init__(**kwargs)

    def reset_cb(self, **kwargs):
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg['done']:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg['msg'][0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.reset(**kwargs)

    def callback_cb(self, **kwargs):
        return self.callback(**kwargs)

    @abc.abstractmethod
    def reset(self, **kwargs: Message):
        """
        All states in the .yaml must be defined as optional arguments in the node subclass.
        If additional states may be specified later-on (not yet specified in yaml), then **kwargs must remain in the
        reset signature of the subclass.
        Additional states can then be found in the **kwargs object.
        """
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        """
        All inputs in the .yaml must be defined as optional arguments in the node subclass.
        If an input is required, add an assert that checks that input is not None.
        If additional inputs may be specified later-on (not yet specified in yaml), then **kwargs must remain in the
        callback signature of the subclass.
        Additional inputs can then be found in the **kwargs object.
        """
        pass


class SimNode(Node):
    def __init__(self, **kwargs):
        self.simulator = None
        self.object_params = None
        super().__init__(**kwargs)

    def set_object_params(self, object_params):
        self.object_params = object_params

    def set_simulator(self, simulator):
        self.simulator = simulator

    @abc.abstractmethod
    def reset(self, **kwargs):
        pass

    @abc.abstractmethod
    def callback(self, node_tick: int, t_n: float, **kwargs):
        pass


class ObservationsNode(Node):
    def __init__(self, inputs, **kwargs):
        # Define observation buffers
        self.observation_buffer = dict()
        for i in inputs:
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.observation_buffer[i['name']] = {'msg': None, 'converter': converter}
        super().__init__(inputs=inputs, **kwargs)

    def reset(self):
        # Set all messages to None
        for name, buffer in self.observation_buffer.items():
            buffer['msg'] = None

    def callback(self, node_tick: int, t_n: float, **kwargs):
        # Set all observations to messages in inputs
        for name, buffer in self.observation_buffer.items():
            buffer['msg'] = kwargs[name]['msg']

        # Send output_msg
        output_msgs = dict(set=UInt64())
        return output_msgs


class ActionsNode(Node):
    def __init__(self, outputs, **kwargs):
        # Define action/observation buffers
        self.action_buffer = dict()
        for i in outputs:
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.action_buffer[i['name']] = {'msg': None, 'converter': converter}
        super().__init__(outputs=outputs, **kwargs)

    def reset(self):
        # Set all messages to None
        for name, buffer in self.action_buffer.items():
            buffer['msg'] = None

    def callback(self, node_tick: int, t_n: float, **kwargs):
        # Fill output_msg with buffered actions
        output_msgs = dict()
        for name, buffer in self.action_buffer.items():
            output_msgs[name] = buffer['msg']
        return output_msgs


class RenderNode(Node):
    def __init__(self, ns, name, **kwargs):
        self.last_image = Image(data=[])
        self.render_toggle = False
        rospy.Service('%s/%s/get_last_image' % (ns, name), ImageUInt8, self._get_last_image)
        rospy.Subscriber('%s/%s/toggle' % (ns, name), Bool, self._set_render_toggle)
        super().__init__(ns=ns, name=name, **kwargs)

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

    def callback(self, node_tick: int, t_n: float, image: Image = None):
        self.last_image = image['msg'][-1]
        if self.render_toggle:
            # todo: render image
            pass

        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=UInt64)
        return output_msgs


class RealResetNode(Node):
    # MSG_TYPE = {'in_1': 'std_msgs.msg/UInt64',
    #             'in_2': 'std_msgs.msg/UInt64',}

    def __init__(self, test_arg, **kwargs):

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20
        super().__init__(**kwargs)

    def reset(self, state_1: UInt64 = None):
        self.num_resets += 1
        if True:
            if self.num_resets % self.print_iter == 0:
                if self.iter_start:
                    iter_stop = time.time()
                    if self.iter_ticks > 0:
                        iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                    else:
                        iter_time = float(iter_stop - self.iter_start)
                    # Memory usage request
                    mem_use = (np.array(self.py.memory_info()[0:2]) / 2. ** 30) * 1000  # memory use in MB...I think

                    rospy.logdebug("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                        self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0

    def callback(self, node_tick: int, t_n: float,
                 in_1: Dict[str, Union[List[UInt64], float, int]] = None,
                 in_2: Dict[str, Union[List[UInt64], float, int]] = None,
                 target_1: Dict[str, Union[List[UInt64], float, int]] = None) -> Dict[str, UInt64]:
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
                t_i = inputs[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))
                    pass

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
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs


class ProcessNode(Node):
    def __init__(self, test_arg, **kwargs):
        # If node is simulator, we will probably use this in callback & reset
        self.simulator = None

        # Message counter
        self.num_ticks = 0
        self.num_resets = 0

        # Memory usage
        self.py = psutil.Process(os.getpid())
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 20
        super().__init__(**kwargs)

    def set_simulator(self, simulator):
        self.simulator = simulator

    def reset(self, state_1: UInt64 = None, state_2: UInt64 = None) -> Dict[str, UInt64]:
        self.num_resets += 1
        if True:
            if self.num_resets % self.print_iter == 0:
                if self.iter_start:
                    iter_stop = time.time()
                    if self.iter_ticks > 0:
                        iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                    else:
                        iter_time = float(iter_stop - self.iter_start)
                    # Memory usage request
                    mem_use = (np.array(self.py.memory_info()[0:2]) / 2. ** 30) * 1000  # memory use in MB...I think

                    rospy.logdebug("[%s] loop %d: iter_time %.4f sec, rss %.1f MB, vms %.1f MB" % (
                    self.name, self.num_resets, iter_time, mem_use[0], mem_use[1]))

                    self.iter_ticks = 0
                self.iter_start = time.time()
        self.num_ticks = 0

        # Send initial message for outputs with 'start_with_msg' = True
        init_msgs = dict()
        for i in self.outputs:
            if not i['start_with_msg']: continue
            name = i['name']
            init_msgs[name] = UInt64(data=999)
        return init_msgs

    def callback(self, node_tick: int, t_n: float,
                 in_1: Dict[str, Union[List[UInt64], float, int]] = None,
                 in_2: Dict[str, Union[List[UInt64], float, int]] = None,
                 in_3: Dict[str, Union[List[String], float, int]] = None,
                 tick: Dict[str, Union[List[UInt64], float, int]] = None) -> Dict[str, UInt64]:
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
                t_i = inputs[name]['t_i']
                if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))
                    pass

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.outputs:
            name = i['name']
            output_msgs[name] = UInt64(data=Nc)
        self.num_ticks += 1
        self.iter_ticks += 1
        return output_msgs