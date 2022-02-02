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
        assert angle_idx == 0, 'Converter currently only works when angle_idx = 0 and '
        self.angle_idx = angle_idx

    def convert(self, msg):
        if msg.data == []:
            return msg
        angle = msg.data[self.angle_idx]
        new_data = np.concatenate(([np.sin(angle), np.cos(angle)], msg.data[self.angle_idx+1:]), axis=0)
        return Float32MultiArray(data=new_data)


