from typing import Dict, List, Union
import numpy as np

# IMPORT ROS
from std_msgs.msg import Float32MultiArray

# IMPORT EAGERX
from eagerx_core.basenode import Node
from eagerx_node_pid.controller import PID


class PIDNode(Node):
    def __init__(self, gains, **kwargs):
        super().__init__(**kwargs)
        self.controller = PID(u0=gains[0], kp=gains[1], kd=gains[2], ki=gains[3], dt=1/self.rate)

    def reset(self):
        self.controller.reset()

    def callback(self, node_tick: int, t_n: float,
                 yref: Dict[str, Union[List[Float32MultiArray], float, int]] = None,
                 y: Dict[str, Union[List[Float32MultiArray], float, int]] = None) -> Dict[str, Float32MultiArray]:
        angles = y['msg'][-1].data[:2]
        y = np.arctan2(angles[1], angles[0])
        yref = yref['msg'][-1].data[0]
        u = self.controller.next_action(y, ref=yref)
        return dict(u=Float32MultiArray(data=[u]))
