from typing import Optional, List
import numpy as np

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray

# IMPORT EAGERX
from eagerx.core.constants import process
from eagerx.utils.utils import return_typehint, Msg
from eagerx.core.entities import EngineNode
import eagerx.core.register as register


class OdeOutput(EngineNode):
    @staticmethod
    @register.spec("OdeOutput", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        color: Optional[str] = "cyan",
    ):
        """OdeOutput spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(OdeOutput)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=["tick"],
            outputs=["observation"],
        )
        spec.set_parameters(params)

    def initialize(self):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.obj_name = self.agnostic_params["name"]

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64)
    @register.outputs(observation=Float32MultiArray)
    def callback(self, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        )
        data = self.simulator[self.obj_name]["state"]
        return dict(observation=Float32MultiArray(data=data))


class ActionApplied(EngineNode):
    @staticmethod
    @register.spec("ActionApplied", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        color: Optional[str] = "cyan",
    ):
        """ActionApplied spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(ActionApplied)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=["tick", "action_applied"],
            outputs=["action_applied"],
        )
        spec.set_parameters(params)

    def initialize(self):
        pass
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        # assert self.process == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        # self.obj_name = self.agnostic_params['name']

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64, action_applied=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action_applied: Optional[Float32MultiArray] = None,
    ) -> return_typehint(Float32MultiArray):
        if len(action_applied.msgs) > 0:
            data = action_applied.msgs[-1].data
        else:
            data = [0]
        return dict(action_applied=Float32MultiArray(data=data))


class OdeInput(EngineNode):
    @staticmethod
    @register.spec("OdeInput", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        default_action: List,
        process: Optional[int] = process.BRIDGE,
        color: Optional[str] = "green",
    ):
        """OdeInput spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(OdeInput)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=["tick", "action"],
            outputs=["action_applied"],
        )
        spec.set_parameters(params)

        # Set custom node params
        spec.set_parameter("default_action", default_action)

    def initialize(self, default_action):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.obj_name = self.agnostic_params["name"]
        self.default_action = np.array(default_action)

    @register.states()
    def reset(self):
        self.simulator[self.obj_name]["input"] = np.squeeze(np.array(self.default_action))

    @register.inputs(tick=UInt64, action=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action: Optional[Float32MultiArray] = None,
    ) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        )

        # Set action in simulator for next step.
        self.simulator[self.obj_name]["input"] = np.squeeze(action.msgs[-1].data)

        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])
