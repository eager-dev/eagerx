from typing import Optional
import numpy as np
from scipy.signal import butter

# IMPORT ROS
from std_msgs.msg import Float32MultiArray

# IMPORT EAGERX
from eagerx_core.utils.utils import return_typehint
from eagerx_core.nodes import Node


class LowPassFilter(Node):
    msg_types = {'inputs': {'signal': Float32MultiArray},
                 'outputs': {'filtered': Float32MultiArray}}

    def __init__(self, N, Wn, btype, **kwargs):
        super().__init__(**kwargs)
        for i in self.inputs:
            if i['name'] == 'signal':
                assert int(i['window']) >= N, \
                    'The window size of the signal {} is too small to create a filter with order {}.'.format(
                        i['window'], N)
        self.filter = butter(N, btype, output='sos', fs=1/self.rate)

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, signal: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        data = np.array()
        # for msg in signal.msgs:
        #
        #     np.concatenate(data, msg.data)
        # if data is not None:
        return dict(observation=Float32MultiArray(data=data))



