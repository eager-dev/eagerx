from eagerx.core.entities import Processor
import eagerx.core.register as register


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
