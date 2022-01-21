# OTHER IMPORTS
from typing import Optional, List
from yaml import dump

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64

# EAGERx IMPORTS
from eagerx_core.constants import process, ERROR
from eagerx_core.entities import Bridge, SpaceConverter, BaseConverter
import eagerx_core.registration as register
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
    @register.spec('TestBridge', Bridge)
    def spec(spec, rate, process: Optional[int] = process.NEW_PROCESS, is_reactive: Optional[bool] = True,
             real_time_factor: Optional[float] = 0, simulate_delays: Optional[bool] = True,
             log_level: Optional[int] = ERROR, states: Optional[List[str]] = ['param_1']):
        # Modify default bridge params
        params = dict(rate=rate,
                      process=process,
                      is_reactive=is_reactive,
                      real_time_factor=real_time_factor,
                      simulate_delays=simulate_delays,
                      log_level=log_level,
                      color='magenta',
                      states=states)
        spec.set_parameters(params)

        # Add custom params
        custom = dict(num_substeps=10,  # Not used
                      nonreactive_address='/nonreactive_input_topic')  # Only required to test nonreactive inputs
        spec.set_parameters(custom)

        # Add state: "param_1"
        spec.add_state('param_1', msg_type=UInt64, space_converter=SpaceConverter.make('Space_RosUInt64', low=[0], high=[100], dtype='uint64'))
        return spec

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
