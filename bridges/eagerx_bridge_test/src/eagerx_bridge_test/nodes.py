# OTHER IMPORTS
from typing import Optional

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64, String, Bool

# EAGERx IMPORTS
from eagerx_core.utils.utils import return_typehint, Msg
from eagerx_core.entities import Node, SimNode
from eagerx_core.registration import register


class RealResetNode(Node):
    msg_types = {'inputs': {'in_1': UInt64,
                            'in_2': UInt64},
                 'outputs': {'out_1': UInt64,
                             'out_2': UInt64},
                 'states': {'state_1': UInt64},
                 'targets': {'target_1': UInt64}}

    def __init__(self, test_arg, **kwargs):
        super().__init__(**kwargs)

    @staticmethod
    @register('RealReset', Node)
    def spec(spec, name, rate=1, process=1, inputs=['in_1'], outputs=['out_1'], states=['state_1'],
              color='white', test_arg='changed'):
        return spec

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
            msg = Bool()
            if self.num_ticks > 2:
                msg.data = True
            else:
                msg.data = False
            output_msgs[name + '/done'] = msg
        return output_msgs


class TestNode(SimNode):
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

    def reset(self, state_1: Optional[UInt64] = None, state_2: Optional[UInt64] = None) -> None:
        return

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


class ProcessNode(TestNode):
    @staticmethod
    @register('Process', Node)
    def spec(spec, name, rate=1, process=1, inputs=['in_1'], outputs=['out_1'], states=['state_1'],
              color='white', test_arg='changed'):
        return spec


class KalmanNode(TestNode):
    @staticmethod
    @register('KalmanFilter', Node)
    def spec(spec, name, rate=1, process=1, inputs=['in_1'], outputs=['out_1'], states=['state_1'],
              color='white', test_arg='changed'):
        return spec


class SimActuator(TestNode):
    @staticmethod
    @register('SimActuator', SimNode)
    def spec(spec, name, rate=1, process=1, inputs=['in_1'], outputs=['out_1'], states=['state_1'],
              color='white', test_arg='changed'):
        return spec


class SimSensor(TestNode):
    @staticmethod
    @register('SimSensor', SimNode)
    def spec(spec, name, rate=1, process=1, inputs=['in_1'], outputs=['out_1'], states=['state_1'],
              color='white', test_arg='changed'):
        return spec