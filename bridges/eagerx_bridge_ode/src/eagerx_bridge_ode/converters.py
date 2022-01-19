# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# RX IMPORTS
from eagerx_core.converters import SpaceConverter, Processor
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

class AngleDecomposition(Processor):
    MSG_TYPE = Float32MultiArray

    def __init__(self, angle_idx=0):
        super().__init__(angle_idx)
        self.angle_idx = angle_idx

    def convert(self, msg):
        if msg.data == []:
            return msg
        data = np.squeeze(msg.data)
        new_data = np.concatenate((data[:self.angle_idx], [np.sin(data[self.angle_idx]), np.cos(data[self.angle_idx])]))
        new_data = np.concatenate((new_data, data[self.angle_idx+1:]))
        return Float32MultiArray(data=new_data)

class AngleNormalization(Processor):
    MSG_TYPE = Float32MultiArray

    def __init__(self, angle_idx=0):
        super().__init__(angle_idx)
        self.angle_idx = angle_idx

    def convert(self, msg):
        if msg.data == []:
            return msg
        data = np.squeeze(msg.data)
        data[self.angle_idx] = self._angle_normalize(data[self.angle_idx])
        return Float32MultiArray(data=data)

    @staticmethod
    def _angle_normalize(x):
        return ((x + np.pi) % (2 * np.pi)) - np.pi




