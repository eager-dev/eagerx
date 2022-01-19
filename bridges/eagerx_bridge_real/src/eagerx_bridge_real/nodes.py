from typing import Optional
import numpy as np
from scipy.signal import butter, sosfilt

# IMPORT ROS
from std_msgs.msg import Float32MultiArray

# IMPORT EAGERX
from eagerx_core.utils.utils import return_typehint
from eagerx_core.nodes import Node


class ButterworthFilter(Node):
    msg_types = {'inputs': {'signal': Float32MultiArray},
                 'outputs': {'filtered': Float32MultiArray}}

    def __init__(self, N, Wn, btype, **kwargs):
        super().__init__(**kwargs)
        for i in self.inputs:
            if i['name'] == 'signal':
                assert int(i['window']) >= N, \
                    'The window size of the signal {} is too small to create a filter with order {}.'.format(
                        i['window'], N)
        self.filter = butter(N, Wn, btype, output='sos', fs=self.rate)
        self.N = N

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, signal: Optional[Float32MultiArray] = None) -> \
            return_typehint(Float32MultiArray):
        msgs = signal.msgs
        if len(msgs) >= self.N:
            unfiltered = [msgs[i].data[0] for i in range(-self.N, 0)]
            filtered = msgs[-1].data if None in unfiltered else [sosfilt(self.filter, unfiltered)[-1]]
        elif len(msgs) > 0:
            filtered = msgs[-1].data
        else:
            filtered = [0.0]
        return dict(filtered=Float32MultiArray(data=filtered))