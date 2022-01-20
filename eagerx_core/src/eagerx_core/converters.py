from eagerx_core.entities import BaseConverter
from eagerx_core.registration import register


class Identity(BaseConverter):
    """ The Identity converter that mimics the API of any other converter but simply passes through all messages."""

    def __init__(self):
        super().__init__()

    @staticmethod
    @register('Identity', BaseConverter)
    def spec(spec):
        return spec

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        return msg_type

    def convert(self, msg):
        return msg


