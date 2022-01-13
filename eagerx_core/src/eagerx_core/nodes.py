import abc
import os
import time
from typing import Optional, Dict, Any, Union, List
from tabulate import tabulate
import logging

import numpy as np
import psutil
import rospy
from std_msgs.msg import UInt64, Bool
from genpy.message import Message
from sensor_msgs.msg import Image

from eagerx_core.rxmessage_broker import RxMessageBroker
from eagerx_core.constants import TERMCOLOR, ERROR, SILENT  # , INFO, DEBUG
from eagerx_core.utils.utils import initialize_converter, Msg
from eagerx_core.srv import ImageUInt8, ImageUInt8Response


class NodeBase:
    def __init__(self, ns: str, message_broker: RxMessageBroker, name: str, config_name: str, package_name: str,
                 node_type: str, rate: float, process: int, inputs: List[Dict], outputs: List[Dict], states: List[Dict],
                 feedthroughs: List[Dict], targets: List[Dict], is_reactive: bool, real_time_factor: float,
                 launch_file=None, color: str = 'grey', print_mode: int = TERMCOLOR, log_level: int = ERROR,
                 log_level_memory: int = SILENT):
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
        super().__init__(**kwargs)
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
    def reset(self, **kwargs: Optional[Message]) -> Optional[Dict[str, Message]]:
        """
        A method to reset this simulation node.

        Could be used to:
        - reset the internal state of this node if it has one.

        :param kwargs: Optionally the node states if any are defined in the node_ros_package/config/<node>.yaml.
        :return: A dict *only* containing output message for outputs with their flag set to "start_with_msg=True".
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
        :return: A dict *only* containing output message for outputs with their flag set to "start_with_msg=True".
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
