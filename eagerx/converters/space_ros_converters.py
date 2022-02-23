# ROS IMPORTS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32, Bool

# RX IMPORTS
from eagerx.core import register as register
from eagerx.core.entities import SpaceConverter
from eagerx.core.specs import ConverterSpec

# OTHER
import numpy as np
from gym.spaces import Box


class Space_Float32MultiArray(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

    @staticmethod
    @register.spec("Space_Float32MultiArray", SpaceConverter)
    def spec(spec: ConverterSpec, low=None, high=None, dtype="float32"):
        # Initialize spec with default arguments
        spec.initialize(Space_Float32MultiArray)
        params = dict(low=low, high=high, dtype=dtype)
        spec.config.update(params)

    def initialize(self, low=None, high=None, dtype="float32"):
        self.low = np.array(low, dtype=dtype)
        self.high = np.array(high, dtype=dtype)
        self.dtype = dtype

    def get_space(self):
        # return Box(self.low, self.high, shape=[1,], dtype=self.dtype)
        return Box(self.low, self.high, dtype=self.dtype)

    def A_to_B(self, msg):
        return Float32MultiArray(data=msg)

    def B_to_A(self, msg):
        return np.array(msg.data, dtype=self.dtype)


class Space_Image(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Image

    @staticmethod
    @register.spec("Space_Image", SpaceConverter)
    def spec(spec: ConverterSpec, low=None, high=None, shape=None, dtype="uint64"):
        # Initialize spec with default arguments
        spec.initialize(Space_Image)

        params = dict(low=low, high=high, shape=shape, dtype=dtype)
        spec.config.update(params)

    def initialize(self, low=None, high=None, shape=None, dtype="float32"):
        if shape is not None and len(shape) == 2:
            shape = shape.append(3)
        self.low = low
        self.high = high
        if isinstance(low, list):
            self.low = np.array(self.low)
        if isinstance(high, list):
            self.high = np.array(self.high)
        if shape is not None:
            assert (
                isinstance(low, (int, float)) or self.low.shape == shape
            ), "If a shape is defined, low must be of a list or type (int, float)."
            assert (
                isinstance(high, (int, float)) or self.high.shape == shape
            ), "If a shape is defined, high must be a list of type (int, float)."
            self.low = low
            self.high = high
        self.shape = shape
        self.dtype = dtype

    def get_space(self):
        return Box(self.low, self.high, shape=self.shape, dtype=self.dtype)

    def A_to_B(self, msg):
        image = self._change_dtype_np_image(msg, "uint8")
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
            if image.dtype in ("float32", "float64") and dtype == "uint8":
                image = (image * 255).astype(dtype)
            elif image.dtype == "uint8" and dtype in ("float32", "float64"):
                image = image.astype(dtype) / 255
            else:
                message = "Cannot convert observations from {} to {}."
                raise NotImplementedError(message.format(image.dtype, dtype))
        return image


class Space_Float32(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32

    @staticmethod
    @register.spec("Space_Float32", SpaceConverter)
    def spec(spec: ConverterSpec, low=None, high=None, dtype="float32"):
        # Initialize spec with default arguments
        spec.initialize(Space_Float32)

        params = dict(low=low, high=high, dtype=dtype)
        spec.config.update(params)

    def initialize(self, low=None, high=None, dtype="float32"):
        self.low = low
        self.high = high
        assert isinstance(low, (int, float)), "Low must be of type (int, float)."
        assert isinstance(high, (int, float)), "High must be of type (int, float)."
        self.dtype = dtype

    def get_space(self):
        return Box(
            self.low,
            self.high,
            shape=[
                1,
            ],
            dtype=self.dtype,
        )

    def A_to_B(self, msg):
        return Float32(data=msg)

    def B_to_A(self, msg):
        return msg.data


class Space_Bool(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Bool

    @staticmethod
    @register.spec("Space_Bool", SpaceConverter)
    def spec(spec: ConverterSpec):
        pass

    def initialize(self):
        self.dtype = np.bool

    def get_space(self):
        return Box(
            False,
            True,
            shape=[
                1,
            ],
            dtype=self.dtype,
        )

    def A_to_B(self, msg):
        return Float32(data=msg)

    def B_to_A(self, msg):
        return msg.data
