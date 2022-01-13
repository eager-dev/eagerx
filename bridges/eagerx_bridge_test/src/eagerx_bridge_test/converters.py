from eagerx_core.converters import SpaceConverter, Converter, BaseProcessor

# Converter specific
from std_msgs.msg import UInt64, String
from sensor_msgs.msg import Image
import numpy as np
from gym.spaces import Box


class Space_RosUInt64(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = UInt64

    def __init__(self, low, high, shape=None, dtype='uint64'):
        super().__init__(low, high, shape, dtype)
        self.low = np.array(low)
        self.high = np.array(high)
        self.shape = shape
        self.dtype = dtype

    def get_space(self):
        return Box(self.low, self.high, shape=self.shape, dtype=self.dtype)

    def A_to_B(self, msg):
        return UInt64(data=msg[0])

    def B_to_A(self, msg):
        return np.array([msg.data], dtype=self.dtype)


class Space_RosString(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = String

    def __init__(self, low, high, shape=None, dtype='uint64'):
        super().__init__(low, high, shape, dtype)
        self.low = low
        self.high = high
        if isinstance(low, list):
            self.low = np.array(self.low)
        if isinstance(high, list):
            self.high = np.array(self.high)
        if shape is not None:
            assert isinstance(low, (int, float)) or self.low.shape == shape, 'If a shape is defined, low must be of a list or type (int, float).'
            assert isinstance(high, (int, float)) or self.high.shape == shape, 'If a shape is defined, high must be a list of type (int, float).'
            self.low = low
            self.high = high
        self.shape = shape
        self.dtype = dtype

    def get_space(self):
        return Box(self.low, self.high, shape=self.shape, dtype=self.dtype)

    def A_to_B(self, msg):
        return String(data='string: %s ' % msg[0])

    def B_to_A(self, msg):
        return np.array([int(msg.data[8:])], dtype=self.dtype)


class RosImage_RosUInt64(Converter):
    MSG_TYPE_A = Image
    MSG_TYPE_B = UInt64

    def __init__(self, test_arg):
        super().__init__(test_arg)
        self.test_arg = test_arg

    def A_to_B(self, msg):
        return UInt64(data=999)

    def B_to_A(self, msg):
        return Image(data=[msg.data])


class RosString_RosUInt64(Converter):
    MSG_TYPE_A = String
    MSG_TYPE_B = UInt64

    def __init__(self, test_arg):
        super().__init__(test_arg)
        self.test_arg = test_arg

    def A_to_B(self, msg):
        return UInt64(data=int(msg.data[8:]))

    def B_to_A(self, msg):
        return String(data='string: %s' % msg.data)
