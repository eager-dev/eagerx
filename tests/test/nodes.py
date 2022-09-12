from eagerx import register, Space
from eagerx import Node, ResetNode, EngineNode
from eagerx.utils.utils import Msg
from eagerx import process

# OTHER IMPORTS
import numpy as np
from typing import Optional, List
from math import isclose


class RealResetNode(ResetNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENVIRONMENT,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        targets: Optional[List[str]] = None,
        color: Optional[str] = "blue",
        test_arg: Optional[str] = "test_argument",
    ):
        """Create the default spec that is compatible with __init__(...), .callback(...), and .reset(...)"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["in_1"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["out_1"]
        spec.config.states = states if isinstance(states, list) else ["state_1"]
        spec.config.targets = targets if isinstance(targets, list) else ["target_1"]

        # Modify extra params
        spec.config.test_arg = test_arg

        # set input parameters
        spec.inputs.in_1.window = 0
        spec.inputs.in_2.window = 0
        return spec

    def initialize(self, spec):
        self.test_arg = spec.config.test_arg

    @register.states(state_1=Space(low=0, high=99, shape=(), dtype="int64"))
    def reset(self, state_1: Optional[int] = None) -> None:
        if "in_1" in self.inputs:
            self.set_delay(0.0, "inputs", "in_1")

    @register.inputs(in_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     in_2=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    @register.outputs(out_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                      out_2=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    @register.targets(target_1=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    def callback(
        self,
        t_n: float,
        in_1: Optional[Msg] = None,
        in_2: Optional[Msg] = None,
        target_1: Optional[Msg] = None,
    ):
        # output type is always Dict[str, Union[UInt64, output_msg_types]] because done flags are also inside the output_msgs
        inputs = {"in_1": in_1, "in_2": in_2}
        pop_keys = []
        for key, value in inputs.items():
            if value is None:
                pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that all timestamps are smaller or equal to node time
        for cname, i in self.inputs.items():
            if cname in inputs:
                t_i = inputs[cname].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    self.backend.logerr(f"[{self.name}][{cname}]: Not all t_i are smaller or equal to t_n.")

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for cname, i in self.outputs.items():
            output_msgs[cname] = np.array([Nc], dtype="uint64")

        # Fill state done msg with number of node ticks
        for cname, i in self.targets.items():
            if self.num_ticks > 5:
                output_msgs[cname + "/done"] = True
            else:
                output_msgs[cname + "/done"] = False
        return output_msgs


class TestNode(Node):
    def initialize(self, test_arg):
        pass

    @register.states(state_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     state_2=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    def reset(self, state_1=None, state_2=None) -> None:
        return

    @register.inputs(in_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     in_2=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     in_3=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     tick=Space(shape=(), dtype="int64"))
    @register.outputs(out_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                      out_2=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    def callback(
        self,
        t_n: float,
        in_1: Optional[Msg] = None,
        in_2: Optional[Msg] = None,
        in_3: Optional[Msg] = None,
        tick: Optional[Msg] = None,
    ):
        inputs = {"in_1": in_1, "in_2": in_2, "in_3": in_3, "tick": tick}
        pop_keys = []
        for key, value in inputs.items():
            if value is None:
                pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that # of ticks equals internal counter
        node_tick = t_n * self.rate
        if self.sync and not isclose(self.num_ticks, node_tick):
            self.backend.logerr(
                f"[{self.name}][callback]: ticks not equal (self.num_ticks={self.num_ticks}, node_tick={round(node_tick)})."
            )
            pass

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for cname, i in self.inputs.items():
            if cname in inputs:
                t_i = inputs[cname].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    self.backend.logerr(f"[{self.name}][{cname}]: Not all t_i are smaller or equal to t_n.")

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for cname, i in self.outputs.items():
            output_msgs[cname] = np.array([Nc], dtype="uint64")
        return output_msgs


class ProcessNode(TestNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENVIRONMENT,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        color: Optional[str] = "white",
        test_arg: Optional[str] = "test_argument",
    ):
        """ProcessNode spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["in_1"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["out_1"]
        spec.config.states = states if states else ["state_1"]

        # Modify extra params
        spec.config.test_arg = test_arg

        # set input parameters
        spec.inputs.in_1.window = 0
        spec.inputs.in_2.window = 0
        spec.inputs.in_3.window = 0
        spec.inputs.tick.window = 0
        return spec


class KalmanNode(TestNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENVIRONMENT,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        color: Optional[str] = "grey",
        test_arg: Optional[str] = "test_argument",
    ):
        """KalmanNode spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["in_1"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["out_1"]
        spec.config.states = states if isinstance(states, list) else ["state_1"]

        # Modify extra params
        spec.config.test_arg = test_arg

        # Test getting parameters
        _ = spec.inputs.tick.window

        # Set input parameters
        spec.inputs.in_1.window = 0
        spec.inputs.in_2.window = 0
        return spec


class TestEngineNode(EngineNode):
    def initialize(self, spec, simulator):
        pass

    @register.states(state_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     state_2=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    def reset(self, state_1=None, state_2=None) -> None:
        return

    @register.inputs(in_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     in_2=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     in_3=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                     tick=Space(shape=(), dtype="int64"))
    @register.outputs(out_1=Space(low=0, high=100, shape=(1,), dtype="uint64"),
                      out_2=Space(low=0, high=100, shape=(1,), dtype="uint64"))
    def callback(
        self,
        t_n: float,
        in_1: Optional[Msg] = None,
        in_2: Optional[Msg] = None,
        in_3: Optional[Msg] = None,
        tick: Optional[Msg] = None,
    ):
        inputs = {"in_1": in_1, "in_2": in_2, "in_3": in_3, "tick": tick}
        pop_keys = []
        for key, value in inputs.items():
            if value is None:
                pop_keys.append(key)
        [inputs.pop(i) for i in pop_keys]

        # Verify that # of ticks equals internal counter
        node_tick = t_n * self.rate
        if self.sync and not isclose(self.num_ticks, node_tick):
            self.backend.logerr(
                f"[{self.name}][callback]: ticks not equal (self.num_ticks={self.num_ticks}, node_tick={round(node_tick)})."
            )
            pass

        # Verify that all timestamps are smaller or equal to node time
        t_n = node_tick * (1 / self.rate)
        for cname, i in self.inputs.items():
            if cname in inputs:
                t_i = inputs[cname].info.t_in
                if len(t_i) > 0 and not all((t.sim_stamp - t_n) <= 1e-7 for t in t_i if t is not None):
                    self.backend.logerr(f"[{self.name}][{cname}]: Not all t_i are smaller or equal to t_n.")

        # Fill output msg with number of node ticks
        output_msgs = dict()
        Nc = self.num_ticks
        for cname, i in self.outputs.items():
            output_msgs[cname] = np.array([Nc], dtype="uint64")
        return output_msgs


class TestActuator(TestEngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "green",
        test_arg: Optional[str] = "test_argument",
    ):
        """TestActuator spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick", "in_1", "in_2", "in_3"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["out_1"]

        # Modify extra params
        spec.config.test_arg = test_arg
        return spec


class TestSensor(TestEngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
        test_arg: Optional[str] = "test_argument",
    ):
        """TestSensor spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick_1", "in_1"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["out_1"]
        spec.config.states = states if isinstance(states, list) else ["state_1"]

        # Modify extra params
        spec.config.test_arg = test_arg
        return spec
