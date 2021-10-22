import abc

# Converter specific
from std_msgs.msg import UInt64
import numpy as np
from gym.spaces import Box


class BaseConverter(object):
    __metaclass__ = abc.ABCMeta

    def convert(self, msg):
        if isinstance(msg, self.MSG_TYPE_A):
            return self.A_to_B(msg)
        elif isinstance(msg, self.MSG_TYPE_B):
            return self.B_to_A(msg)
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.' %
                (type(msg), self.MSG_TYPE_A, self.MSG_TYPE_B))

    @abc.abstractmethod
    def A_to_B(self, msg):
        pass

    @abc.abstractmethod
    def B_to_A(self, msg):
        pass


class SpaceConverter(BaseConverter):
    @abc.abstractmethod
    def get_space(self):
        pass


class IdentityConverter(object):
    def convert(self, msg):
        return msg


class SpaceUInt64Converter(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = UInt64

    def __init__(self, low, high, shape=None, dtype='uint64'):
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


class IntUInt64Converter(BaseConverter):
    MSG_TYPE_A = int
    MSG_TYPE_B = UInt64

    def __init__(self, test_arg):
        self.test_arg = test_arg

    def A_to_B(self, msg):
        return UInt64(data=msg)

    def B_to_A(self, msg):
        return int(msg.data)
