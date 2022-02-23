# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# RX IMPORTS
import eagerx.core.register as register
from eagerx.core.entities import SpaceConverter

# OTHER
import numpy as np
from gym.spaces import Box
import gym


class GymSpace_Float32MultiArray(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

    @staticmethod
    @register.spec("GymSpace_Float32MultiArray", SpaceConverter)
    def spec(
        spec,
        gym_id: str = None,
        space: str = None,
        low=None,
        high=None,
        shape=None,
        dtype="float32",
    ):
        # Initialize converter
        spec.initialize(GymSpace_Float32MultiArray)

        spec.config.gym_id = gym_id
        spec.config.space = space
        spec.config.low = low
        spec.config.high = high
        spec.config.shape = shape
        spec.config.dtype = dtype

    def initialize(self, gym_id=None, space=None, low=None, high=None, shape=None, dtype="float32"):
        if gym_id is not None and space in ["observation", "action"]:
            self.env = gym.make(gym_id)
            if space == "observation":
                self.space = self.env.observation_space
            else:  # space == 'action
                self.space = self.env.action_space
            if isinstance(self.space, gym.spaces.Discrete):
                self.space = gym.spaces.Box(low=0, high=self.space.n - 1, shape=(1,), dtype="int64")
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
