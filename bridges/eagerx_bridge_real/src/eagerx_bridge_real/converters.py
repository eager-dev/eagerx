# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# RX IMPORTS
from eagerx_core.converters import BaseProcessor
import numpy as np


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




