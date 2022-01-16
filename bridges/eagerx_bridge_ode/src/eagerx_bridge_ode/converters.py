# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# RX IMPORTS
from eagerx_core.converters import SpaceConverter, BaseProcessor
import numpy as np
from gym.spaces import Box


class Space_RosFloat32MultiArray(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

    def __init__(self, low=None, high=None, dtype='float32'):
        super().__init__(low, high, dtype)
        self.low = np.array(low)
        self.high = np.array(high)
        # assert isinstance(low, (int, float)), 'Low must be of type (int, float).'
        # assert isinstance(high, (int, float)), 'High must be of type (int, float).'
        self.dtype = dtype

    def get_space(self):
        # return Box(self.low, self.high, shape=[1,], dtype=self.dtype)
        return Box(self.low, self.high, dtype=self.dtype)

    def A_to_B(self, msg):
        return Float32MultiArray(data=msg)

    def B_to_A(self, msg):
        return np.squeeze(msg.data)

class AngleUnwrapper(BaseProcessor):
    MSG_TYPE = Float32MultiArray

    def __int__(self):
        super().__init__()

    def convert(self, msg):
        data = msg.data
        data[0] = np.unwrap(data[0])
        return Float32MultiArray(data)




