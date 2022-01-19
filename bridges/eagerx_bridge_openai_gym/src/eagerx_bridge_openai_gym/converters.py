# ROS IMPORTS
from std_msgs.msg import Float32MultiArray, Float32, Bool
from sensor_msgs.msg import Image

# RX IMPORTS
from eagerx_core.converters import SpaceConverter
import numpy as np
from gym.spaces import Box
import gym


class GymSpaceFloat32MultiArray(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

    def __init__(self, id=None, space=None, low=None, high=None, shape=None, dtype='float32'):
        super().__init__(id, space, low, high, shape, dtype)
        if id is not None and space in ['observation', 'action']:
            self.env = gym.make(id)
            if space == 'observation':
                self.space = self.env.observation_space
            else:  # space == 'action
                self.space = self.env.action_space
            if isinstance(self.space, gym.spaces.Discrete):
                self.space = gym.spaces.Box(low=0, high=self.space.n-1, shape=(1,), dtype='int64')
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


class SpaceImage(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Image

    def __init__(self, low=None, high=None, shape=None, dtype='float32'):
        super().__init__(low, high, shape, dtype)
        if shape is not None and len(shape) == 2:
            shape = shape.append(3)
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
        image = self._change_dtype_np_image(msg, 'uint8')
        height = image.shape[0]
        width = image.shape[1]
        msg = Image(data=image.reshape(-1).tolist(), height=height, width=width)
        return msg

    def B_to_A(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self._change_dtype_np_image(image, self.dtype)
        return image

    def _change_dtype_np_image(self, image, dtype):
        if dtype and image.dtype != dtype:
            if image.dtype in ('float32', 'float64') and dtype == 'uint8':
                image = (image * 255).astype(dtype)
            elif image.dtype == 'uint8' and dtype in ('float32', 'float64'):
                image = image.astype(dtype) / 255
            else:
                message = 'Cannot convert observations from {} to {}.'
                raise NotImplementedError(message.format(image.dtype, dtype))
        return image


class SpaceFloat32(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32

    def __init__(self, low=None, high=None, dtype='float32'):
        super().__init__(low, high, dtype)
        self.low = low
        self.high = high
        assert isinstance(low, (int, float)), 'Low must be of type (int, float).'
        assert isinstance(high, (int, float)), 'High must be of type (int, float).'
        self.dtype = dtype

    def get_space(self):
        return Box(self.low, self.high, shape=[1,], dtype=self.dtype)

    def A_to_B(self, msg):
        return Float32(data=msg)

    def B_to_A(self, msg):
        return msg.data


class SpaceBool(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Bool

    def __init__(self):
        super().__init__()
        self.dtype = np.bool

    def get_space(self):
        return Box(False, True, shape=[1,], dtype=self.dtype)

    def A_to_B(self, msg):
        return Float32(data=msg)

    def B_to_A(self, msg):
        return msg.data