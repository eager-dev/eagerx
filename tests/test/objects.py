# OTHER IMPORTS
from typing import Optional, List
from gym.spaces import Box

# EAGERx IMPORTS
import eagerx
from eagerx import register
from eagerx import specs
from tests.test.engine import TestEngine


class Arm(eagerx.Object):
    @classmethod
    @register.sensors(N6=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                      N7=Box(low=0, high=100, shape=(1,), dtype="uint64"), )
    @register.actuators(N8=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                        N12=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                        ref_vel=Box(low=0, high=100, shape=(1,), dtype="uint64"))
    @register.engine_states(N9=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                            N10=Box(low=0, high=100, shape=(1,), dtype="uint64"))
    def make(
        cls,
        name: str = None,
        sensors: Optional[List[str]] = None,
        actuators: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        position: Optional[List[str]] = None,
        orientation: Optional[List[str]] = None,
        string: Optional[str] = "test_arg",
        test_string: Optional[str] = "$(config string)",
        test_list: Optional[str] = "$(config orientation)",
        low: Optional[int] = 0,
    ) -> specs.ObjectSpec:
        """Object spec of Arm"""
        spec = cls.get_specification()

        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if isinstance(sensors, list) else ["N6", "N7"]
        spec.config.actuators = actuators if isinstance(actuators, list) else ["ref_vel"]
        spec.config.states = states if isinstance(states, list) else ["N9", "N10"]

        # Change custom params
        spec.config.position = position if isinstance(position, list) else [0, 0, 0]
        spec.config.orientation = orientation if isinstance(orientation, list) else [0, 0, 0]
        spec.config.string = string
        spec.config.test_string = test_string
        spec.config.test_list = test_list
        spec.config.low = low

        # Test ObjectSpec
        spec.sensors.N6 = spec.sensors.N6
        spec.config.name = spec.config.name

        # Add sensor properties
        spec.sensors.N6.rate = 15
        spec.sensors.N7.rate = 2

        # Set actuator properties
        spec.actuators.N8.rate = 15
        spec.actuators.N12.rate = 15
        spec.actuators.ref_vel.rate = 1

        # Test properties
        spec.actuators.N8.rate = spec.actuators.N8.rate
        spec.actuators.N8 = spec.actuators.N8
        return spec

    @staticmethod
    @register.engine(TestEngine)
    def test_engine(spec: specs.ObjectSpec, graph: eagerx.EngineGraph):
        """Engine-specific implementation of the Arm with the test engine."""
        # Set engine_config
        spec.engine.req_arg = "TEST"
        spec.engine.xacro = "$(find some_package)/urdf/arm.urdf.xacro"

        # Create engine_states
        from tests.test.enginestates import TestEngineState
        spec.engine.states.N9 = TestEngineState.make(test_arg="arg_N9")

        # Create sensor engine nodes
        from tests.test.nodes import TestSensor, TestActuator
        N6 = TestSensor.make(
            "N6",
            rate=spec.sensors.N6.rate,
            process=2,
            inputs=["tick"],
            outputs=["out_1"],
            states=["state_1"],
            test_arg=spec.engine.req_arg,
        )
        N7 = TestSensor.make(
            "N7",
            rate=spec.sensors.N7.rate,
            process=2,
            inputs=["tick", "in_1"],
            outputs=["out_1"],
            states=[],
            test_arg=spec.config.test_string,
        )

        # Create actuator engine nodes
        N8 = TestActuator.make(
            "N8",
            rate=spec.actuators.N8.rate,
            process=2,
            inputs=["tick", "in_2", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        ref_vel = TestActuator.make(
            "ref_vel",
            rate=spec.actuators.ref_vel.rate,
            process=2,
            inputs=["tick", "in_1", "in_2"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )

        # Test SpecificSpec:
        _ = spec.__str__()
        spec.engine.states.N9.test_arg = "test2"

        # Test EngineGraph: Add/remove sensor
        graph.add(N6)
        _ = graph.__str__()
        graph.add_component(N6.outputs.out_2)
        graph.remove_component(N6.outputs.out_2)
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.remove(N6)
        N6.set_graph(None)

        # Test EngineGraph: Add/remove actuator
        graph.add(N8)
        graph.connect(actuator="N8", target=N8.inputs.in_3)
        graph.remove("N8")
        N8.set_graph(None)

        # Test EngineGraph: Remove component
        graph.add(N6)
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.remove_component(N6.outputs.out_1)
        graph.add_component(N6.outputs.out_1)
        graph.remove("N6")
        N6.set_graph(None)

        # Test EngineGraph: Remove component
        graph.add(N8)
        graph.connect(actuator="N8", target=N8.inputs.in_3)
        graph.remove_component(N8.inputs.in_3)
        graph.add_component(N8.inputs.in_3)
        graph.remove("N8")
        N8.set_graph(None)

        # Test EngineGraph: parameters
        graph.add([N6, N8])
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.connect(actuator="N8", target=N8.inputs.in_3)
        graph.get(N6.inputs.in_1.window)
        graph.set({"test_arg": "NEW_ARG"}, N6.config)
        graph.get(actuator="N8", parameter="space")
        graph.get(sensor="N6", parameter="window")
        graph.get(sensor="N6")
        graph.get(actuator="N8")
        graph.get(N6.outputs.out_1)
        graph.set(graph.get(N6.outputs.out_1), N6.outputs.out_1)
        graph.get(N6)
        graph.remove(["N6", "N8"])
        N6.set_graph(None)
        N8.set_graph(None)

        # Add nodes to graph and connect them to actuators/sensors
        graph.add([N6, N7, N8, ref_vel])

        # Connect sensors & actuators to engine nodes
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.disconnect(source=N6.outputs.out_1, sensor="N6")
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.connect(source=N7.outputs.out_1, sensor="N7")
        graph.connect(actuator="N8", target=N8.inputs.in_3)
        graph.connect(actuator="ref_vel", target=ref_vel.inputs.in_1)

        # Interconnect engine nodes
        graph.connect(
            source=N8.outputs.out_1,
            target=N7.inputs.in_1,
            skip=True,
        )
        graph.connect(source=N6.outputs.out_1, target=N8.inputs.in_2, window=1)
        graph.connect(source=N6.outputs.out_1, target=ref_vel.inputs.in_2, delay=0.0)

        # Test EngineGraph: Test acyclic check
        import matplotlib.pyplot as plt

        plt.ion()

        graph.add_component(N8.inputs.in_1)
        graph.connect(source=N7.outputs.out_1, target=N8.inputs.in_1)
        graph.disconnect(source=N8.outputs.out_1, target=N7.inputs.in_1)
        graph.connect(source=N8.outputs.out_1, target=N7.inputs.in_1)
        try:
            graph.is_valid(plot=True)
        except AssertionError as e:
            if "Algebraic" in e.args[0]:
                pass
            else:
                raise
        # Reconnect
        graph.remove_component(N8.inputs.in_1)
        graph.disconnect(source=N8.outputs.out_1, target=N7.inputs.in_1)
        graph.connect(source=N8.outputs.out_1, target=N7.inputs.in_1, skip=True)

        # Connect multiple engine nodes to same actuator
        N11 = TestActuator.make(
            "N11",
            rate=spec.actuators.N8.rate,
            process=2,
            inputs=["tick", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        graph.add(N11)
        graph.connect(actuator="N8", target=N11.inputs.in_3)


class Viper(Arm):
    @classmethod
    @register.sensors(N6=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                      N7=Box(low=0, high=100, shape=(1,), dtype="uint64"), )
    @register.actuators(N8=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                        N12=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                        ref_vel=Box(low=0, high=100, shape=(1,), dtype="uint64"))
    @register.engine_states(N9=Box(low=0, high=100, shape=(1,), dtype="uint64"),
                            N10=Box(low=0, high=100, shape=(1,), dtype="uint64"))
    def make(
        cls,
        name: str,
        sensors: Optional[List[str]] = None,
        actuators: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        position: Optional[List[float]] = None,
        orientation: Optional[List[float]] = None,
        string: Optional[str] = "test_arg",
        test_string: Optional[str] = "$(config string)",
        test_list: Optional[str] = "$(config orientation)",
        low: Optional[int] = 0,
    ):
        """Object spec of Viper"""
        spec = cls.get_specification()

        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if isinstance(sensors, list) else ["N6", "N7"]
        spec.config.actuators = actuators if isinstance(actuators, list) else ["ref_vel"]
        spec.config.states = states if isinstance(states, list) else ["N9", "N10"]

        # Change custom params
        spec.config.position = position if isinstance(position, list) else [0, 0, 0]
        spec.config.orientation = orientation if isinstance(orientation, list) else [0, 0, 0]
        spec.config.string = string
        spec.config.test_string = test_string
        spec.config.test_list = test_list
        spec.config.low = low

        # Add sensor properties
        spec.sensors.N6.rate = 15
        spec.sensors.N7.rate = 2

        # Set actuator properties
        spec.actuators.N8.rate = 15
        spec.actuators.N12.rate = 15
        spec.actuators.ref_vel.rate = 1
        return spec

    @staticmethod
    @register.engine(TestEngine)
    def test_engine(spec: specs.ObjectSpec, graph: eagerx.EngineGraph):
        """Engine-specific implementation of the Viper with the test engine."""
        # Set object arguments
        spec.engine.req_arg = "TEST ARGUMENT"
        spec.engine.xacro = "$(find some_package)/urdf/viper.urdf.xacro"

        # Create engine_states
        from tests.test.enginestates import TestEngineState
        spec.engine.states.N9 = TestEngineState.make(test_arg="arg_N9")
        spec.engine.states.N10 = TestEngineState.make(test_arg="arg_N10")

        # Create sensor engine nodes
        from tests.test.nodes import TestSensor, TestActuator
        N6 = TestSensor.make(
            "N6",
            rate=spec.sensors.N6.rate,
            process=2,
            inputs=["tick"],
            outputs=["out_1"],
            states=["state_1"],
            test_arg=spec.engine.req_arg,
        )
        N7 = TestSensor.make(
            "N7",
            rate=spec.sensors.N7.rate,
            process=2,
            inputs=["tick", "in_1"],
            outputs=["out_1"],
            states=[],
            test_arg=spec.config.test_string,
        )

        # Create actuator engine nodes
        N8 = TestActuator.make(
            "N8",
            rate=spec.actuators.N8.rate,
            process=2,
            inputs=["tick", "in_2", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        ref_vel = TestActuator.make(
            "ref_vel",
            rate=spec.actuators.ref_vel.rate,
            process=2,
            inputs=["tick", "in_1", "in_2"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )

        # Add IdentityProcessor to N6
        from tests.test.processors import IdentityProcessor
        IdentityProcessor = IdentityProcessor.make()
        N6.outputs.out_1.processor = IdentityProcessor

        # Add nodes to graph and connect them to actuators/sensors
        graph.add([N6, N7, N8, ref_vel])

        # Connect sensors & actuators to engine nodes
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.disconnect(source=N6.outputs.out_1, sensor="N6")
        graph.connect(source=N6.outputs.out_1, sensor="N6")
        graph.connect(source=N7.outputs.out_1, sensor="N7")
        graph.connect(actuator="N8", target=N8.inputs.in_3)
        graph.connect(actuator="ref_vel", target=ref_vel.inputs.in_1)

        # Interconnect engine nodes
        graph.connect(
            source=N8.outputs.out_1,
            target=N7.inputs.in_1,
            skip=True,
        )
        graph.connect(source=N6.outputs.out_1, target=N8.inputs.in_2)
        graph.connect(source=N6.outputs.out_1, target=ref_vel.inputs.in_2)

        # Test Adding converters to actuators
        N12 = TestActuator.make(
            "N12",
            rate=spec.actuators.N12.rate,
            process=2,
            inputs=["tick", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        graph.add(N12)
        graph.connect(actuator="N12", target=N12.inputs.in_3)
