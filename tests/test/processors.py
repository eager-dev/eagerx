from typing import Union, List
import numpy as np
from eagerx.core.entities import Processor


class GetIndex(Processor):
    @classmethod
    def make(cls, index: Union[int, List[int]]):
        spec = cls.get_specification()
        if isinstance(index, int):
            index = [index]
        for i in index:
            assert isinstance(i, int) and i >= 0, f'Incorrect index "{i}". Indices must be >= 0.'
        spec.config.index = index
        return spec

    def initialize(self, spec):
        self.index = spec.config.index

    def convert(self, msg):
        if len(msg) == 0:
            raise NotImplementedError("TODO: What to do if length of msg is zero?")
        return msg[self.index].astype("float32")


class ToUint8(Processor):
    @classmethod
    def make(cls):
        return cls.get_specification()

    def initialize(self, spec):
        pass

    def convert(self, msg: np.ndarray):
        return msg.astype("uint8")


class IdentityProcessor(Processor):
    @classmethod
    def make(cls):
        return cls.get_specification()

    def initialize(self, spec):
        pass

    def convert(self, msg):
        return msg
