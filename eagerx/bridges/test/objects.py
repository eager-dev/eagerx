# OTHER IMPORTS
from typing import Optional, List

# ROS IMPORTS
from std_msgs.msg import UInt64, String

# EAGERx IMPORTS
from eagerx.bridges.test.bridge import TestBridgeNode
from eagerx.core.entities import Object, EngineNode, SpaceConverter, EngineState, BaseConverter, Converter, Processor
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Arm(Object):
    entity_id = "Arm"

    @staticmethod
    @register.sensors(N6=UInt64, N7=UInt64)
    @register.actuators(N8=String, ref_vel=UInt64, N12=UInt64)
    @register.simstates(N9=UInt64, N10=UInt64)
    @register.config(position=[0, 0, 0], orientation=[0, 0, 0], low=None, string=None, test_string=None, test_list=None)
    def agnostic(spec: ObjectSpec, rate):
        """Agnostic definition of the Arm object"""
        # Set state properties: space_converters
        spec.sensors.N6.space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.sensors.N7.space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.sensors.N6.rate = rate
        spec.sensors.N7.rate = 2

        # Set actuator properties: space_converters
        spec.actuators.N8.space_converter = SpaceConverter.make("Space_RosString", [0], [100], dtype="uint64")
        # spec.actuators.N12.space_converter = SpaceConverter.make("Space_RosString", [0], [100], dtype="uint64")
        spec.actuators.N12.space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.actuators.ref_vel.space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.actuators.N8.rate = rate
        spec.actuators.N12.rate = rate
        spec.actuators.ref_vel.rate = 1

        # Set state properties: space_converters
        spec.states.N9.space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")
        spec.states.N10.space_converter = SpaceConverter.make("Space_RosUInt64", [0], [100], dtype="uint64")

        # Test AgnosticSpec
        spec.actuators.N8.rate = spec.actuators.N8.rate
        spec.actuators.N8 = spec.actuators.N8

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
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
    ):
        """Object spec of Arm"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        Arm.initialize_spec(spec)

        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors else ["N6", "N7"]
        spec.config.actuators = actuators if actuators else ["ref_vel"]
        spec.config.states = states if states else ["N9", "N10"]

        # Change custom params
        spec.config.position = position if position else [0, 0, 0]
        spec.config.orientation = orientation if orientation else [0, 0, 0]
        spec.config.string = string
        spec.config.test_string = test_string
        spec.config.test_list = test_list
        spec.config.low = low

        # Test ObjectSpec
        spec.sensors.N6 = spec.sensors.N6
        spec.config.name = spec.config.name

        # Add agnostic definition
        Arm.agnostic(spec, rate=15)

    @staticmethod
    @register.bridge(entity_id, TestBridgeNode)
    def test_bridge(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation of the Arm with the test bridge."""
        # Set bridge_config
        spec.TestBridge.req_arg = "TEST"
        spec.TestBridge.xacro = "$(find some_package)/urdf/arm.urdf.xacro"

        # Create simstates
        spec.TestBridge.states.N9 = EngineState.make("TestEngineState", test_arg="arg_N9")

        # Create sensor engine nodes
        N6 = EngineNode.make(
            "TestSensor",
            "N6",
            rate=spec.sensors.N6.rate,
            process=2,
            inputs=["tick", "in_1"],
            outputs=["out_1"],
            states=["state_1"],
            test_arg=spec.TestBridge.req_arg,
        )
        N7 = EngineNode.make(
            "TestSensor",
            "N7",
            rate=spec.sensors.N7.rate,
            process=2,
            inputs=["tick", "in_1"],
            outputs=["out_1"],
            states=[],
            test_arg=spec.config.test_string,
        )

        # Create actuator engine nodes
        N8 = EngineNode.make(
            "TestActuator",
            "N8",
            rate=spec.actuators.N8.rate,
            process=2,
            inputs=["tick", "in_2", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        ref_vel = EngineNode.make(
            "TestActuator",
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
        spec.TestBridge.states.N9.test_arg = "test2"

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
        graph.get(N6.outputs.out_1.converter)
        graph.set({"test_arg": "NEW_ARG"}, N6.config)
        graph.get(actuator="N8", parameter="converter")
        graph.get(sensor="N6", parameter="converter")
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
        id = BaseConverter.make("Identity")
        graph.connect(
            source=N8.outputs.out_1,
            target=N7.inputs.in_1,
            skip=True,
        )
        graph.connect(source=N6.outputs.out_1, target=N8.inputs.in_2, window=1, converter=id)
        graph.connect(source=N6.outputs.out_1, target=ref_vel.inputs.in_2, delay=0.0)

        # Connect simnode with external address (cannot be actuator or sensor)
        graph.connect(
            address="$(ns env_name)/nonreactive_input_topic",
            target=N6.inputs.in_1,
            external_rate=20,
        )
        graph.disconnect(target=N6.inputs.in_1)
        graph.connect(
            address="$(ns env_name)/nonreactive_input_topic",
            target=N6.inputs.in_1,
            external_rate=20,
        )

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
        N11 = EngineNode.make(
            "TestActuator",
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
    entity_id = "Viper"

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
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
        # Performs all the steps to fill-in the params with registered info about all functions.
        Viper.initialize_spec(spec)

        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors else ["N6", "N7"]
        spec.config.actuators = actuators if actuators else ["ref_vel"]
        spec.config.states = states if states else ["N9", "N10"]

        # Change custom params
        spec.config.position = position if position else [0, 0, 0]
        spec.config.orientation = orientation if orientation else [0, 0, 0]
        spec.config.string = string
        spec.config.test_string = test_string
        spec.config.test_list = test_list
        spec.config.low = low

        # Add agnostic definition
        Viper.agnostic(spec, rate=15)

    @staticmethod
    @register.bridge(entity_id, TestBridgeNode)
    def test_bridge(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation of the Viper with the test bridge."""
        # Set object arguments
        spec.TestBridge.req_arg = "TEST ARGUMENT"
        spec.TestBridge.xacro = "$(find some_package)/urdf/viper.urdf.xacro"

        # Create simstates
        spec.TestBridge.states.N9 = EngineState.make("TestEngineState", test_arg="arg_N9")
        spec.TestBridge.states.N10 = EngineState.make("TestEngineState", test_arg="arg_N10")

        # Create sensor engine nodes
        N6 = EngineNode.make(
            "TestSensor",
            "N6",
            rate=spec.sensors.N6.rate,
            process=2,
            inputs=["tick", "in_1"],
            outputs=["out_1"],
            states=["state_1"],
            test_arg=spec.TestBridge.req_arg,
        )
        N7 = EngineNode.make(
            "TestSensor",
            "N7",
            rate=spec.sensors.N7.rate,
            process=2,
            inputs=["tick", "in_1"],
            outputs=["out_1"],
            states=[],
            test_arg=spec.config.test_string,
        )

        # Create actuator engine nodes
        N8 = EngineNode.make(
            "TestActuator",
            "N8",
            rate=spec.actuators.N8.rate,
            process=2,
            inputs=["tick", "in_2", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        ref_vel = EngineNode.make(
            "TestActuator",
            "ref_vel",
            rate=spec.actuators.ref_vel.rate,
            process=2,
            inputs=["tick", "in_1", "in_2"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )

        # Add IdentityProcessor to N6
        IdentityProcessor = Processor.make("IdentityProcessor")
        N6.outputs.out_1.converter = IdentityProcessor

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

        # Connect simnode with external address (cannot be actuator or sensor)
        graph.connect(
            address="$(ns env_name)/nonreactive_input_topic",
            target=N6.inputs.in_1,
            external_rate=20,
        )
        graph.disconnect(target=N6.inputs.in_1)
        graph.connect(
            address="$(ns env_name)/nonreactive_input_topic",
            target=N6.inputs.in_1,
            external_rate=20,
        )

        # Test Adding converters to actuators
        N12 = EngineNode.make(
            "TestActuator",
            "N12",
            rate=spec.actuators.N12.rate,
            process=2,
            inputs=["tick", "in_3"],
            outputs=["out_1"],
            test_arg=spec.config.test_string,
            color="green",
        )
        graph.add(N12)
        RosString_RosUInt64 = Converter.make("RosString_RosUInt64", test_arg="test")
        graph.connect(actuator="N12", target=N12.inputs.in_3, converter=RosString_RosUInt64)
