import abc

# Converter specific
from std_msgs.msg import UInt64, String
from sensor_msgs.msg import Image
import numpy as np
from gym.spaces import Box


class BaseConverter(object):
    """
    Inherit your converter from this baseclass and implement the abstract methods. In addition, make sure to specify the
    static attributes "MSG_TYPE_A" and "MSG_TYPE_B", such that the correct conversion method can be inferred.
    """
    __metaclass__ = abc.ABCMeta

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        if msg_type == cls.MSG_TYPE_A:
            return cls.MSG_TYPE_B
        elif msg_type == cls.MSG_TYPE_B:
            return cls.MSG_TYPE_A
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.' %
                (msg_type, cls.MSG_TYPE_A, cls.MSG_TYPE_B))

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
    """
    Inherit your converter from this baseclass if the converter is used for actions/observations/states,
    such that the space can be inferred. See BaseConverter for other abstract methods that must be implemented.
    """
    @abc.abstractmethod
    def get_space(self):
        pass


class IdentityConverter(object):
    """ The Identity converter that mimics the API of any other converter but simply passes through all messages."""
    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        return msg_type

    def convert(self, msg):
        return msg


class BaseProcessor(object):
    """
    Use this processor if the converted msg type is one-way and the msg_type after conversion is equal to
    the msg_type before conversion. In addition, make sure to specify the static attribute "MSG_TYPE".
    """
    __metaclass__ = abc.ABCMeta

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        if msg_type == cls.MSG_TYPE:
            return cls.MSG_TYPE
        else:
            raise ValueError('Message type "%s" not supported by this converter. Only msg_type "%s" is supported.' %(msg_type, cls.MSG_TYPE))

    def convert(self, msg):
        if isinstance(msg, self.MSG_TYPE):
            return self._convert(msg)
        else:
            raise ValueError('Message type "%s" not supported by this converter. Only msg_type "%s" is supported.' %(type(msg), self.MSG_TYPE))

    @abc.abstractmethod
    def _convert(self, msg):
        pass

###########################################################################
# Specific implementations ################################################
###########################################################################


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


class SpaceStringConverter(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = String

    def __init__(self, low, high, shape=None, dtype='uint64'):
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


class ImageUInt64Converter(BaseConverter):
    MSG_TYPE_A = Image
    MSG_TYPE_B = UInt64

    def __init__(self, test_arg):
        self.test_arg = test_arg

    def A_to_B(self, msg):
        return UInt64(data=999)

    def B_to_A(self, msg):
        return Image(data=[msg.data])


class StringUInt64Converter(BaseConverter):
    MSG_TYPE_A = String
    MSG_TYPE_B = UInt64

    def __init__(self, test_arg):
        self.test_arg = test_arg

    def A_to_B(self, msg):
        return UInt64(data=int(msg.data[8:]))

    def B_to_A(self, msg):
        return String(data='string: %s' % msg.data)
