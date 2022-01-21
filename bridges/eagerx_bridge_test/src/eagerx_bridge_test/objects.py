# OTHER IMPORTS
from typing import Optional, List
from yaml import dump

# ROS IMPORTS
from std_msgs.msg import UInt64, String, Bool

# EAGERx IMPORTS
from eagerx_core.constants import process, ERROR
from eagerx_core.utils.utils import return_typehint, Msg
from eagerx_core.entities import Object, Node, SimNode, SpaceConverter, Converter
import eagerx_core.registration as register


class Pendulum(Object):
    id = 'Pendulum'

    @staticmethod
    @register.sensors(id, Object, F6=UInt64, F7=UInt64)
    @register.actuators(id, Object, F8=String, ref_vel=UInt64)
    @register.states(id, Object, F9=String, F10=UInt64)
    @register.spec(id, Object)
    def spec(spec, name: str, sensors: Optional[List[str]] = ['N6', 'N7'], actuators: Optional[List[str]] = ['ref_vel'],
             states: Optional[List[str]] = ['N9', 'N10'], position: Optional[List[str]] = [0, 0, 0],
             orientation: Optional[List[str]] = [0, 0, 0], string: Optional[str] = 'test_arg',
             test_string: Optional[str] = '$(default string)', test_list: Optional[str] = '$(default orientation)',
             low: Optional[int] = 0):
        """Create the default spec that is compatible with __init__(...), .callback(...), and .reset(...)"""
        # todo: should receive a spec with all agnostic entries for sensors/actuators/states pre-defined
        # todo: only "change rates" and optionally add space_converters, etc...
        # todo: only allowed to modify agnostic params?
        # todo: allow bridge_specific_params to be coupled with agnostic params
        # todo: Create placeholder string that ties to a default argument to a bridge_specific or space_converter argument
        # todo: create it with ${ns node_name} default arg}
        # todo: resolve when get_params --> do not allow coupled args to be changed. --> replace with coupled counterpart
        # todo: create a big context dict with all [node_names]['default']
        # todo: spec.couple_component(simnode_id='', simnode_args) --> how to know simnode_args? get_spec... and verify that all arguments were provided.
        # todo: set_parameter(Placeholder(name=, parameter=)) --> converts to string command (to be replaced with substitute args)
        # todo: if argument is Placeholder(), then it cannot be changed in GUI. When saving, revert to string notation. When loading, revert back to placeholder.

        # Modify default node params
        default = dict(name=name,
                       sensors=sensors,
                       actuators=actuators,
                       states=states)
        spec.set_parameters(default)

        # Add custom params
        params = dict(position=position,
                      orientation=orientation,
                      string=string,
                      test_string=test_string,
                      test_list=test_list,
                      low=low)
        spec.set_parameters(params)

        # Add (agnostic) sensor (same for actuator &
        spec.add_sensor('N6', msg_type=UInt64, rate=20, space_converter=SpaceConverter.make('Space_RosUInt64', low=Placeholder(name=name, parameter='low'), high=[0], dtype='uint64'))

        # Add bridge-specific implementation
        # todo: first add bridge --> get BridgeObjectSpec from a registered Bridge.
        # todo: add_sensors/actuators/states to bridge implementation (based on registered id).
        # todo: only create node in .build(ns=...)? Or, make it here already and extract what is needed from the SimNode spec.
        spec.add_bridge_implementation(id='TestBridge', object_args=[])
        spec.add_sensor(id='TestBridge', cname=cname, node=SimNode.make('SimSensor', name=f'$(ns obj_name)/sensors/{cname}'))

        return spec


class Viper(Object):
    id = 'Viper'

    @staticmethod
    @register.sensors(id, Object,   N6=UInt64, N7=UInt64)
    @register.actuators(id, Object, N8=String, ref_vel=UInt64)
    @register.simstates(id, Object, N9=String, N10=UInt64)
    @register.spec(id, Object)
    def spec(spec, name: str, sensors: Optional[List[str]] = ['N6', 'N7'], actuators: Optional[List[str]] = ['ref_vel'],
             states: Optional[List[str]] = ['N9', 'N10'], position: Optional[List[str]] = [0, 0, 0],
             orientation: Optional[List[str]] = [0, 0, 0], string: Optional[str] = 'test_arg',
             test_string: Optional[str] = '$(default string)', test_list: Optional[str] = '$(default orientation)',
             low: Optional[int] = 0):
        """Create the default spec that is compatible with __init__(...), .callback(...), and .reset(...)"""
        # todo: should receive a spec with all agnostic entries for sensors/actuators/states pre-defined
        # todo: only "change rates" and optionally add space_converters, etc...
        # todo: only allowed to modify agnostic params?
        # todo: allow bridge_specific_params to be coupled with agnostic params
        # todo: Create placeholder string that ties to a default argument to a bridge_specific or space_converter argument
        # todo: create it with ${ns node_name} default arg}
        # todo: resolve when get_params --> do not allow coupled args to be changed. --> replace with coupled counterpart
        # todo: create a big context dict with all [node_names]['default']
        # todo: spec.couple_component(simnode_id='', simnode_args) --> how to know simnode_args? get_spec... and verify that all arguments were provided.
        # todo: set_parameter(Placeholder(name=, parameter=)) --> converts to string command (to be replaced with substitute args)
        # todo: if argument is Placeholder(), then it cannot be changed in GUI. When saving, revert to string notation. When loading, revert back to placeholder.

        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(Viper.entity_id)

        # Modify default node params
        default = dict(name=name,
                       sensors=sensors,
                       actuators=actuators,
                       states=states)
        spec.set_parameters(default)

        # Add custom params
        params = dict(position=position,
                      orientation=orientation,
                      string=string,
                      test_string=test_string,
                      test_list=test_list,
                      low=low)
        spec.set_parameters(params)

        # Add (agnostic) sensor (same for actuator &
        spec.add_sensor('N6', msg_type=UInt64, rate=20, space_converter=SpaceConverter.make('Space_RosUInt64', low=[0], high=[0], dtype='uint64'))

        # Add bridge-specific implementation
        # todo: first add bridge --> get BridgeObjectSpec from a registered Bridge.
        # todo: add_sensors/actuators/states to bridge implementation (based on registered id).
        # todo: only create node in .build(ns=...)? Or, make it here already and extract what is needed from the SimNode spec.
        spec.add_bridge_implementation(id='TestBridge', object_args=[])
        spec.add_sensor(id='TestBridge', cname=cname, node=SimNode.make('SimSensor', name=f'$(ns obj_name)/sensors/{cname}'))

        return spec

    # todo: register.object_params(....) on top of add_object_to_simulator
    # todo: rename add_object_to_simulator to add_object(object_params, node_params, simstate_params)
    # todo: make bridge_id register add_object --> make coupling with sensors, actuators, & states
    # todo: Must receive a spec that is bridge specific and directly linked to the object spec (or we copy out into it after return)
    # todo: agnostic-params cannot become engine-specific --> hence, they cannot be variable.
    # todo: make all additional variables engine-specific & pass as object_params
    # todo: couple sensor/actuator/state cname to specific nodes inside register.bridge(..)
    # @staticmethod
    # @register.bridge(id, bridge_id)
    # def bridge_implementation(spec):
    #     pass