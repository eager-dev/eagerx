# OTHER
from typing import Optional, Dict, Union, List


# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.utils.utils import Msg
from eagerx.core.entities import Bridge
from eagerx.core.specs import BridgeSpec
from eagerx.utils.node_utils import launch_node


class RealBridge(Bridge):
    @staticmethod
    @register.spec("RealBridge", Bridge)
    def spec(
        spec: BridgeSpec,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        is_reactive: Optional[bool] = False,
        log_level: Optional[int] = ERROR,
    ):
        """
        Spec of the RealBridge

        :param spec: Not provided by the user.
        :param rate: Rate of the bridge
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: BRIDGE, 3: EXTERNAL}
        :param is_reactive: Run reactive or async
        :param log_level: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}
        :return: BridgeSpec
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(RealBridge)

        # Modify default bridge params
        params = dict(
            rate=rate,
            process=process,
            is_reactive=is_reactive,
            real_time_factor=1,
            simulate_delays=False,
            log_level=log_level,
            color="magenta",
        )
        spec.set_parameters(params)

    def initialize(self):
        self.simulator = dict()

    @register.bridge_params(driver_launch_file=None, launch_args=[])
    def add_object(self, agnostic_params, bridge_params, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(
            f'Adding object "{agnostic_params["name"]}" of type "{agnostic_params["entity_id"]}" to the simulator.'
        )

        # Extract relevant agnostic_params
        obj_name = agnostic_params["name"]

        if "driver_launch_file" in bridge_params:
            launch_file = bridge_params["driver_launch_file"]
            launch_args = bridge_params["launch_args"] if "launch_args" in bridge_params else []
            driver = launch_node(launch_file, launch_args)
            # driver.start()  # todo: can only be launched from main thread, unless disable_signals.

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(state=None, input=None)

    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    @register.states()
    def reset(self, **kwargs: Optional[Msg]):
        pass

    @register.outputs(tick=UInt64)
    def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        pass
