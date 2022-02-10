from eagerx.core.entities import BaseConverter
import eagerx.core.register as register


class Identity(BaseConverter):
    """The Identity converter that mimics the API of any other converter but simply passes through all messages."""

    def __init__(self):
        super().__init__()

    @staticmethod
    @register.spec("Identity", BaseConverter)
    def spec(spec):
        pass

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        return msg_type

    def convert(self, msg):
        return msg

    def initialize(self):
        pass
