from typing import Optional
from eagerx.core.entities import EngineState


class TestEngineState(EngineState):
    @classmethod
    def make(cls, test_arg: Optional[str] = "test_argument"):
        spec = cls.get_specification()

        # Modify parameters based on arguments
        spec.config.test_arg = test_arg
        spec.config.test_arg = spec.config.test_arg
        return spec

    def initialize(self, spec, simulator):
        self.test_arg = spec.config.test_arg
        self.simulator = simulator

    def reset(self, state):
        self.backend.logdebug(f"INSIDE SIMSTATE RESET: {state}")
