from typing import Union, List
import numpy as np
from eagerx.core import register as register
from eagerx.core.entities import Processor


class GetIndex(Processor):
    DTYPE = "float32"

    @staticmethod
    @register.spec("GetIndex", Processor)
    def spec(spec, index: Union[int, List[int]]):
        if isinstance(index, int):
            index = [index]
        for i in index:
            assert isinstance(i, int) and i >= 0, f'Incorrect index "{i}". Indices must be >= 0.'
        spec.config.index = index

    def initialize(self, index):
        self.index = index

    def convert(self, msg):
        if len(msg) == 0:
            raise NotImplementedError("TODO: What to do if length of msg is zero?")
        return msg[self.index].astype(self.DTYPE)


class ToUint8(Processor):
    DTYPE = "uint8"

    @staticmethod
    @register.spec("ToUint8", Processor)
    def spec(spec):
        pass

    def initialize(self):
        pass

    def convert(self, msg: np.ndarray):
        return msg.astype(self.DTYPE)


class IdentityProcessor(Processor):
    DTYPE = "uint64"

    @staticmethod
    @register.spec("IdentityProcessor", Processor)
    def spec(spec):
        pass

    def initialize(self):
        pass

    def convert(self, msg):
        return msg
