import numpy as np
from eagerx.core.entities import EngineState
import eagerx.core.register as register


class OdeEngineState(EngineState):
    @staticmethod
    @register.spec("OdeSimState", EngineState)
    def spec(spec):
        pass

    def initialize(self):
        self.obj_name = self.agnostic_params["name"]

    def reset(self, state, done):
        self.simulator[self.obj_name]["state"] = np.squeeze(state.data)


class OdeParameters(EngineState):
    @staticmethod
    @register.spec("OdeParameters", EngineState)
    def spec(spec, indices):
        spec.set_parameter("indices", indices)

    def initialize(self, indices):
        self.obj_name = self.agnostic_params["name"]
        self.indices = indices

    def reset(self, state, done):
        for i in self.indices:
            self.simulator[self.obj_name]["ode_params"][i] = state.data[i]
