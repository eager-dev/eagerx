# OTHER IMPORTS
from typing import Optional, List
from yaml import dump

# ROS IMPORTS
from std_msgs.msg import UInt64, String, Bool

# EAGERx IMPORTS
from eagerx_bridge_test.bridge import TestBridgeNode
from eagerx_core.entities import Object, Node, SimNode, SpaceConverter, Converter, SimState
import eagerx_core.registration as register


class Arm(Object):
    @staticmethod
    @register.sensors(N6=UInt64, N7=UInt64)
    @register.actuators(N8=String, ref_vel=UInt64)
    @register.simstates(N9=UInt64, N10=UInt64)
    @register.agnostic_params(position=[0, 0, 0], orientation=[0, 0, 0], rate=15, low=None, string=None, test_string=None, test_list=None)
    def agnostic(spec):
        """Agnostic definition of the Arm object"""
        # Set state properties: space_converters
        spec.set_space_converter('sensors', 'N6', SpaceConverter.make('Space_RosUInt64', [0], [100], dtype='uint64'))
        spec.set_agnostic_parameter('sensors', 'N6', 'rate', '${default rate}')
        spec.set_space_converter('sensors', 'N7', SpaceConverter.make('Space_RosUInt64', [0], [100], dtype='uint64'))
        spec.set_agnostic_parameter('sensors', 'N7', 'rate', 2)

        # Set actuator properties: space_converters
        spec.set_space_converter('actuators', 'N8', SpaceConverter.make('Space_RosString', [0], [100], dtype='uint64'))
        spec.set_agnostic_parameter('actuators', 'N8', 'rate', '${default rate}')
        spec.set_space_converter('actuators', 'ref_vel', SpaceConverter.make('Space_RosUInt64', [0], [100], dtype='uint64'))
        spec.set_agnostic_parameter('actuators', 'ref_vel', 'rate', 1)

        # Set state properties: space_converters
        spec.set_space_converter('states', 'N9', SpaceConverter.make('Space_RosUInt64', [0], [100], dtype='uint64'))
        spec.set_space_converter('states', 'N10', SpaceConverter.make('Space_RosUInt64', [0], [100], dtype='uint64'))
        return spec

    @staticmethod
    @register.spec('Arm', Object)
    def spec(spec, name: str, sensors: Optional[List[str]] = ['N6', 'N7'], actuators: Optional[List[str]] = ['ref_vel'],
             states: Optional[List[str]] = ['N9', 'N10'], position: Optional[List[str]] = [0, 0, 0],
             orientation: Optional[List[str]] = [0, 0, 0], string: Optional[str] = 'test_arg',
             test_string: Optional[str] = '$(default string)', test_list: Optional[str] = '$(default orientation)',
             low: Optional[int] = 0):
        """Object spec of Arm"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(Arm)

        # Modify default node params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        default = dict(name=name, sensors=sensors, actuators=actuators, states=states)
        spec.set_parameters(default)

        # Add custom params
        params = dict(position=position, orientation=orientation, string=string, test_string=test_string,
                      test_list=test_list, low=low, rate=15)
        spec.set_parameters(params)

        # Add bridge-specific implementation
        # NO BRIDGE-SPECIFIC IMPLEMENTATION. SEE "Viper" SUBCLASS.
        return spec


class Viper(Arm):
    @staticmethod
    @register.spec('Viper', Object)
    def spec(spec, name: str, sensors: Optional[List[str]] = ['N6', 'N7'], actuators: Optional[List[str]] = ['ref_vel'],
             states: Optional[List[str]] = ['N9', 'N10'], position: Optional[List[str]] = [0, 0, 0],
             orientation: Optional[List[str]] = [0, 0, 0], string: Optional[str] = 'test_arg',
             test_string: Optional[str] = '$(default string)', test_list: Optional[str] = '$(default orientation)',
             low: Optional[int] = 0):
        """Object spec of Viper"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(Viper)

        # Modify default node params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        default = dict(name=name, sensors=sensors, actuators=actuators, states=states)
        spec.set_parameters(default, level='default')

        # Add custom params
        params = dict(position=position, orientation=orientation, string=string, test_string=test_string,
                      test_list=test_list, low=low, rate=15)
        spec.set_parameters(params, level='default')

        # Add bridge implementation
        Viper.test_bridge(spec)
        return spec

    @classmethod
    @register.bridge(TestBridgeNode)  # This decorator pre-initializes bridge implementation with default object_params
    def test_bridge(cls, spec, bridge_id, graph):
        """Engine-specific implementation of the Viper with the test bridge."""
        # Set object arguments
        object_params = dict(req_arg='TEST ARGUMENT',
                             xacro='$(find some_package)/urdf/viper.urdf.xacro')
        spec.set_parameters(object_params, level=bridge_id)

        # Create sensor simnodes
        N6 = SimNode.make('SimSensor', 'N6', rate=None, process=2, inputs=['tick', 'in_1'], outputs=['out_1'], states=['state_1'], test_arg='$(default test_string)')
        N7 = SimNode.make('SimSensor', 'N7', rate=None, process=2, inputs=['tick', 'in_1'], outputs=['out_1'], states=[], test_arg='$(default test_string)')

        # Create actuator simnodes
        N8 = SimNode.make('SimActuator', 'N8', rate=None, process=2, inputs=['tick', 'in_2', 'in_3'], outputs=['out_1'], test_arg='$(default test_string)', color='green')
        ref_vel = SimNode.make('SimActuator', 'ref_vel', rate=None, process=2, inputs=['tick', 'in_1', 'in_2'], outputs=['out_1'], test_arg='$(default test_string)', color='green')

        # Create simstates
        N9 = SimState.make('TestSimState', test_arg='arg_N9')
        N10 = SimState.make('TestSimState', test_arg='arg_N10')

        # Add nodes to graph and connect them to actuators/sensors/states
        graph.add([N6, N7, N8, ref_vel])

        # todo: implement graph functions
        #  graph._connect_state(),
        #  implement is_valid(),
        #  check_graph_is_acyclic,
        #  check_msg_type,
        #  check_msg_types_are_consistent,
        #  check_inputs_have_address

        # Connect sensors & actuators to simnodes
        graph.connect(source=('N6', 'outputs', 'out_1'), sensor='N6')
        graph.disconnect(source=('N6', 'outputs', 'out_1'), sensor='N6')
        graph.connect(source=('N6', 'outputs', 'out_1'), sensor='N6')
        graph.connect(source=('N7', 'outputs', 'out_1'), sensor='N7')
        graph.connect(actuator='N8',                     target=('N8', 'inputs', 'in_3'))
        graph.connect(actuator='ref_vel',                target=('ref_vel', 'inputs', 'in_1'))

        # Interconnect simnodes
        graph.connect(source=('N6', 'outputs', 'out_1'), target=('N7', 'inputs', 'in_1'))
        graph.connect(source=('N6', 'outputs', 'out_1'), target=('N8', 'inputs', 'in_2'))
        graph.connect(source=('N6', 'outputs', 'out_1'), target=('ref_vel', 'inputs', 'in_2'))

        # Connect simnode with external address (cannot be actuator or sensor)
        graph.connect(address='$(ns env_name)/nonreactive_input_topic', target=('N6', 'inputs', 'in_1'), external_rate=20)
        graph.disconnect(target=('N6', 'inputs', 'in_1'))
        graph.connect(address='$(ns env_name)/nonreactive_input_topic', target=('N6', 'inputs', 'in_1'), external_rate=20)

        graph.is_valid(plot=True)
        graph.register()
        return spec
