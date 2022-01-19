# OTHER IMPORTS
from typing import Optional

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64

# EAGERx IMPORTS
from eagerx_core.entities import Bridge
from eagerx_core.registration import register
from eagerx_core.utils.utils import Msg


class TestBridgeNode(Bridge):
    msg_types = {'outputs': {'tick': UInt64},
                 'states': {'param_1': UInt64}}

    def __init__(self, num_substeps, nonreactive_address, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        simulator = None

        # If real_time bridge, assert that real_time_factor == 1 & is_reactive=False.

        # Initialize nonreactive input
        self.nonreactive_pub = rospy.Publisher(kwargs['ns'] + nonreactive_address, UInt64, queue_size=0, latch=True)
        super(TestBridgeNode, self).__init__(simulator=simulator, **kwargs)

    @staticmethod
    @register('test_bridge', Bridge)
    def spec(template, rate, is_reactive=True, real_time_factor=0, simulate_delays=True, log_level=10):
        bridge = template.params
        bridge['default']['rate'] = rate
        bridge['default']['is_reactive'] = is_reactive
        bridge['default']['real_time_factor'] = real_time_factor
        bridge['default']['simulate_delays'] = simulate_delays
        bridge['default']['log_level'] = log_level
        bridge['default']['color'] = 'magenta'

        # add param_1 as state
        # todo: add space_converter
        bridge['default']['states'].append('param_1')
        bridge['states'] = {'param_1': {'msg_type': 'std_msgs.msg/UInt64'}}
        return bridge

    def add_object_to_simulator(self, object_params, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo('Adding object "%s" of type "%s.yaml" from package "%s" to the simulator.' % (object_params['name'], object_params['config_name'], object_params['package_name']))
        return object_params

    def pre_reset(self, param_1: Optional[UInt64] = None):
        return 'PRE RESET RETURN VALUE'

    def reset(self, param_1: Optional[UInt64] = None):
        # Publish nonreactive input (this is only required for simulation setup)
        self.nonreactive_pub.publish(UInt64(data=0))
        return 'POST RESET RETURN VALUE'

    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Publish nonreactive input
        self.nonreactive_pub.publish(UInt64(data=node_tick))

        # Verify that # of ticks equals internal counter
        if not self.num_ticks == node_tick:
            rospy.logerr('[%s][callback]: ticks not equal (self.num_ticks=%d, node_tick=%d).' % (self.name, self.num_ticks, node_tick))

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i['name']
            if name in kwargs:
                t_i = kwargs[name].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    rospy.logerr('[%s][%s]: Not all t_i are smaller or equal to t_n.' % (self.name, name))
