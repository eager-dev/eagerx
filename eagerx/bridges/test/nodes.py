# OTHER IMPORTS
from typing import Optional, List
from math import isclose

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64, String, Bool

# EAGERx IMPORTS
from eagerx.core.constants import process
from eagerx.utils.utils import return_typehint, Msg
from eagerx.core.entities import Node, ResetNode, EngineNode, SpaceConverter
import eagerx.core.register as register


class RealResetNode(ResetNode):
    def initialize(self, test_arg, test_kwarg="test"):

        pass

    @staticmethod
    @register.spec("RealReset", ResetNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.ENVIRONMENT,
        inputs: Optional[List[str]] = ["in_1"],
        outputs: Optional[List[str]] = ["out_1"],
        states: Optional[List[str]] = ["state_1"],
        targets: Optional[List[str]] = ["target_1"],
        color: Optional[str] = "blue",
        test_arg: Optional[str] = "test_argument",
    ):
        """Create the default spec that is compatible with __init__(...), .callback(...), and .reset(...)"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(RealResetNode)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=inputs,
            outputs=outputs,
            targets=targets,
            states=states,
        )
        spec.set_parameters(params)

        # Modify extra node params
        spec.set_parameter("test_arg", test_arg)

        # set input parameters
        space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_1")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_2")

        # Set outputs parameters
        spec.set_parameter("space_converter", space_converter, "outputs", "out_1")
        spec.set_parameter("space_converter", space_converter, "outputs", "out_2")

        # Set states parameters
        spec.set_parameter("space_converter", space_converter, "states", "state_1")
        return spec

    @register.states(state_1=UInt64)
    def reset(self, state_1: Optional[UInt64] = None) -> None:
        return

    @register.inputs(in_1=UInt64, in_2=UInt64)
    @register.outputs(out_1=UInt64, out_2=UInt64)
    @register.targets(target_1=UInt64)
    def callback(
        self,
        t_n: float,
        in_1: Optional[Msg] = None,
        in_2: Optional[Msg] = None,
        target_1: Optional[Msg] = None,
    ) -> return_typehint(UInt64, done=True):
        # output type is always Dict[str, Union[UInt64, output_msg_types]] because done flags are also inside the output_msgs
        inputs = {"in_1": in_1, "in_2": in_2}
        pop_keys = []
        for key, value in inputs.items():
            if value is None:
                pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that all timestamps are smaller or equal to node time
        for i in self.inputs:
            name = i["name"]
            if name in inputs:
                t_i = inputs[name].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    rospy.logerr(f"[{self.name}][{name}]: Not all t_i are smaller or equal to t_n.")

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.outputs:
            name = i["name"]
            output_msgs[name] = UInt64(data=Nc)

        # Fill state done msg with number of node ticks
        for i in self.targets:
            name = i["name"]
            msg = Bool()
            if self.num_ticks > 5:
                msg.data = True
            else:
                msg.data = False
            output_msgs[name + "/done"] = msg
        return output_msgs


class TestNode(EngineNode):
    def initialize(self, test_arg):
        pass

    @register.states(state_1=UInt64, state_2=UInt64)
    def reset(self, state_1: Optional[UInt64] = None, state_2: Optional[UInt64] = None) -> None:
        return

    @register.inputs(in_1=UInt64, in_2=UInt64, in_3=String, tick=UInt64)
    @register.outputs(out_1=UInt64, out_2=UInt64)
    def callback(
        self,
        t_n: float,
        in_1: Optional[Msg] = None,
        in_2: Optional[Msg] = None,
        in_3: Optional[Msg] = None,
        tick: Optional[Msg] = None,
    ) -> return_typehint(UInt64):
        inputs = {"in_1": in_1, "in_2": in_2, "tick": tick}
        pop_keys = []
        for key, value in inputs.items():
            if value is None:
                pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that # of ticks equals internal counter
        node_tick = t_n * self.rate
        if self.is_reactive and not isclose(self.num_ticks, node_tick):
            rospy.logerr(
                f"[{self.name}][callback]: ticks not equal (self.num_ticks={self.num_ticks}, node_tick={round(node_tick)})."
            )
            pass

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for i in self.inputs:
            name = i["name"]
            if name in inputs:
                t_i = inputs[name].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    rospy.logerr(f"[{self.name}][{name}]: Not all t_i are smaller or equal to t_n.")

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for i in self.outputs:
            name = i["name"]
            output_msgs[name] = UInt64(data=Nc)
        return output_msgs


class ProcessNode(TestNode):
    @staticmethod
    @register.spec("Process", Node)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.ENVIRONMENT,
        inputs: Optional[List[str]] = ["in_1"],
        outputs: Optional[List[str]] = ["out_1"],
        states: Optional[List[str]] = ["state_1"],
        color: Optional[str] = "white",
        test_arg: Optional[str] = "test_argument",
    ):
        """ProcessNode spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(ProcessNode)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=inputs,
            outputs=outputs,
            states=states,
        )
        spec.set_parameters(params)

        # Modify extra node params
        spec.set_parameter("test_arg", test_arg)

        # Add inputs
        space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.set_parameters(dict(window=0), "inputs", "tick")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_1")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_2")
        space_converter = SpaceConverter.make("Space_RosString", [0], [100], dtype="uint64")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_3")

        # Add outputs
        space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.set_parameter("space_converter", space_converter, "outputs", "out_1")
        spec.set_parameter("space_converter", space_converter, "outputs", "out_2")

        # Add states
        space_converter = SpaceConverter.make("Space_RosUInt64", low=[0], high=[100], dtype="uint64")
        spec.set_parameter("space_converter", space_converter, "states", "state_1")
        spec.set_parameter("space_converter", space_converter, "states", "state_2")
        return spec


class KalmanNode(TestNode):
    @staticmethod
    @register.spec("KalmanFilter", Node)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.ENVIRONMENT,
        inputs: Optional[List[str]] = ["in_1"],
        outputs: Optional[List[str]] = ["out_1"],
        states: Optional[List[str]] = ["state_1"],
        color: Optional[str] = "grey",
        test_arg: Optional[str] = "test_argument",
    ):
        """KalmanNode spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(KalmanNode)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=inputs,
            outputs=outputs,
            states=states,
        )
        spec.set_parameters(params)

        # Modify extra node params
        spec.set_parameter("test_arg", test_arg)

        # Remove unused inputs
        spec.remove_input("tick")
        spec.remove_input("in_3")

        # Set input parameters
        space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_1")
        spec.set_parameters(dict(window=0, space_converter=space_converter), "inputs", "in_2")

        # Set output parameters
        space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.set_parameter("space_converter", space_converter, "outputs", "out_1")
        spec.set_parameter("space_converter", space_converter, "outputs", "out_2")

        # Set state parameters
        space_converter = SpaceConverter.make("Space_RosUInt64", low=[0], high=[100], dtype="uint64")
        spec.set_parameter("space_converter", space_converter, "states", "state_1")
        spec.set_parameter("space_converter", space_converter, "states", "state_2")
        return spec


class TestActuator(TestNode):
    @staticmethod
    @register.spec("TestActuator", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = ["tick", "in_1", "in_2", "in_3"],
        outputs: Optional[List[str]] = ["out_1"],
        color: Optional[str] = "green",
        test_arg: Optional[str] = "test_argument",
    ):
        """TestActuator spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(TestActuator)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=inputs,
            outputs=outputs,
        )
        spec.set_parameters(params)

        # Modify extra node params
        spec.set_parameter("test_arg", test_arg)

        # Set state parameters
        space_converter = SpaceConverter.make("Space_RosUInt64", low=[0], high=[100], dtype="uint64")
        spec.set_parameter("space_converter", space_converter, "states", "state_1")
        spec.set_parameter("space_converter", space_converter, "states", "state_2")
        return spec


class TestSensor(TestNode):
    @staticmethod
    @register.spec("TestSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = ["tick_1", "in_1"],
        outputs: Optional[List[str]] = ["out_1"],
        states: Optional[List[str]] = ["state_1"],
        color: Optional[str] = "cyan",
        test_arg: Optional[str] = "test_argument",
    ):
        """TestSensor spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(TestSensor)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=inputs,
            outputs=outputs,
            states=states,
        )
        spec.set_parameters(params)

        # Modify extra node params
        spec.set_parameter("test_arg", test_arg)

        # Set state parameters
        space_converter = SpaceConverter.make("Space_RosUInt64", low=[0], high=[100], dtype="uint64")
        spec.set_parameter("space_converter", space_converter, "states", "state_1")
        spec.set_parameter("space_converter", space_converter, "states", "state_2")
        return spec
