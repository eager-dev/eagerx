from typing import Union, List
from std_msgs.msg import Float32MultiArray

from eagerx.core import register as register
from eagerx.core.entities import Processor


class GetIndex_Float32MultiArray(Processor):
    MSG_TYPE = Float32MultiArray

    @staticmethod
    @register.spec("GetIndex_Float32MultiArray", Processor)
    def spec(spec, index: Union[int, List[int]]):
        # Initialize spec with default arguments
        spec.initialize(GetIndex_Float32MultiArray)

        if isinstance(index, int):
            index = [index]
        for i in index:
            assert isinstance(i, int) and i >= 0, f'Incorrect index "{i}". Indices must be >= 0.'
        spec.config.index = index

    def initialize(self, index):
        self.index = index

    def convert(self, msg):
        if msg.data == []:
            return msg
        data = msg.data
        if isinstance(data, list) and len(data) > self.index:
            data = [data[self.index]]
        return Float32MultiArray(data=data)
