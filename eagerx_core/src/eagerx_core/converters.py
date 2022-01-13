from eagerx_core.utils.utils import check_valid_rosparam_type
from copy import deepcopy
import inspect
import abc

# Converter specific
from std_msgs.msg import UInt64, String
from sensor_msgs.msg import Image
import numpy as np
from gym.spaces import Box


class BaseConverter(object):
    """
    Make sure to pass all arguments of the subclass' constructor through (**IMPORTANT**in the same order) to this
    baseclass' constructor and that it is of valid type: (str, int, list, float, bool, dict, NoneType).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        self.yaml_args = kwargs
        argspec = inspect.getfullargspec(self.__init__).args
        argspec.remove('self')
        for key, value in zip(argspec, args):
            self.yaml_args[key] = value
        check_valid_rosparam_type(self.yaml_args)

    def get_yaml_definition(self):
        converter_type = self.__module__ + '/' + self.__class__.__name__
        yaml_dict = dict(converter_type=converter_type)
        yaml_dict.update(deepcopy(self.yaml_args))
        return yaml_dict

    @abc.abstractmethod
    def convert(self, msg):
        pass

    @staticmethod
    @abc.abstractmethod
    def get_opposite_msg_type(cls, msg_type):
        pass


class Converter(BaseConverter):
    """
    Inherit your converter from this baseclass and implement the abstract methods. In addition, make sure to specify the
    static attributes "MSG_TYPE_A" and "MSG_TYPE_B", such that the correct conversion method can be inferred.
    Make sure to pass all arguments of the subclass' constructor through (**IMPORTANT**in the same order) to this
    baseclass' constructor and that it is of valid type: (str, int, list, float, bool, dict, NoneType).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

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


class SpaceConverter(Converter):
    """
    Inherit your converter from this baseclass if the converter is used for actions/observations/states,
    such that the space can be inferred. See Converter for other abstract methods that must be implemented.
    """
    @abc.abstractmethod
    def get_space(self):
        pass


class Identity(BaseConverter):
    """ The Identity converter that mimics the API of any other converter but simply passes through all messages."""

    def __init__(self):
        super().__init__()

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        return msg_type

    def convert(self, msg):
        return msg


class BaseProcessor(BaseConverter):
    """
    Use this processor if the converted msg type is one-way and the msg_type after conversion is equal to
    the msg_type before conversion. In addition, make sure to specify the static attribute "MSG_TYPE".
    Make sure to pass all arguments of the subclass' constructor through (**IMPORTANT**in the same order) to this
    baseclass' constructor and that it is of valid type: (str, int, list, float, bool, dict, NoneType).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

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
