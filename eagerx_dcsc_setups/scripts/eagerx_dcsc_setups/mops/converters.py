# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# RX IMPORTS
import eagerx.core.register as register
from eagerx.core.entities import Processor
import numpy as np


class AngleDecomposition(Processor):
    MSG_TYPE = Float32MultiArray

    @staticmethod
    @register.spec('AngleDecomposition', Processor)
    def spec(spec, angle_idx: int = 0):
        spec.set_parameter('angle_idx', angle_idx)

    def initialize(self, angle_idx=0):
        self.angle_idx = angle_idx

    def convert(self, msg):
        if msg.data == []:
            return msg
        data = np.squeeze(msg.data)
        new_data = np.concatenate((data[:self.angle_idx], [np.sin(data[self.angle_idx]), np.cos(data[self.angle_idx])]))
        new_data = np.concatenate((new_data, data[self.angle_idx+1:]))
        return Float32MultiArray(data=new_data)


