# ROS IMPORTS
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# RX IMPORTS
from eagerx_core.baseconverter import SpaceConverter
import numpy as np
from gym.spaces import Box
import gym


class OpenAISpaceFloat32MultiArray(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

    def __init__(self, id=None, space=None, low=None, high=None, shape=None, dtype='float32'):
        if id is not None and space in ['observation', 'action']:
            self.env = gym.make(id)
            if space == 'observation':
                self.space = self.env.observation_space
            else:  # space == 'action
                self.space = self.env.action_space
        else:
            self.space = None
            self.low = np.array(low)
            self.high = np.array(high)
            self.shape = shape
        self.dtype = dtype

    def get_space(self):
        if self.space is not None:
            return self.space
        else:
            return Box(self.low, self.high, shape=self.shape, dtype=self.dtype)

    def A_to_B(self, msg):
        return Float32MultiArray(data=msg)

    def B_to_A(self, msg):
        return np.array(msg.data, dtype=self.dtype)
