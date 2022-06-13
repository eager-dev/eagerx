# OTHER IMPORTS
from typing import Optional, List
from math import isclose

# ROS IMPORTS
import rospy

# EAGERx IMPORTS
from eagerx.core.constants import process, ERROR
from eagerx.core.entities import Engine
import eagerx.core.register as register
import gym


class TestEngineNode(Engine):
    @staticmethod
    @register.spec("TestEngine", Engine)
    def spec(
        spec,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        sync: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = ERROR,
        states: Optional[List[str]] = None,
    ):
        """TestEngine spec"""

        # Modify default engine params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.sync = sync
        spec.config.real_time_factor = real_time_factor
        spec.config.simulate_delays = simulate_delays
        spec.config.log_level = log_level
        spec.config.color = "magenta"
        spec.config.states = states if states else ["param_1"]

        # Add custom params
        spec.config.num_substeps = 10

    def initialize(self, num_substeps: int, nonreactive_address: str):
        # Initialize any simulator here, that is passed as reference to each enginenode
        self.simulator = None

    @register.engine_config(req_arg=None, xacro="$(find some_package)/urdf/object.urdf.xacro")
    def add_object(self, config, engine_config, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(f'Adding object "{config["name"]}" of type "{config["entity_id"]}" to the simulator.')

    def pre_reset(self, param_1=None):
        return "PRE RESET RETURN VALUE"

    @register.states(param_1=gym.spaces.Discrete(999))
    def reset(self, param_1=None):
        return "POST RESET RETURN VALUE"

    def callback(self, t_n: float):
        # Verify that # of ticks equals internal counter
        node_tick = t_n * self.rate
        if not isclose(self.num_ticks, node_tick):
            rospy.logerr(
                f"[{self.name}][callback]: ticks not equal (self.num_ticks={self.num_ticks}, node_tick={round(node_tick)})."
            )
