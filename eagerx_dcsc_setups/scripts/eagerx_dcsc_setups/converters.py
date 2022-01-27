# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# RX IMPORTS
from eagerx_core.converters import BaseProcessor
import numpy as np

class AngleDecomposition(BaseProcessor):
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

class GetIndex(BaseProcessor):
    MSG_TYPE = Float32MultiArray

    def __init__(self, index=0):
        super().__init__(index)
        self.index = index

    def convert(self, msg):
        if msg.data == []:
            return msg
        data = msg.data
        if isinstance(data, list) and len(data) > self.index:
            data = [data[self.index]]
        return Float32MultiArray(data=data)
