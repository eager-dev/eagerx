import rospy
from eagerx_core.utils.node_utils import launch_roscore
launch_roscore()  # First launch roscore
rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.DEBUG)

# ROS packages required
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv, RxGraph
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten
from eagerx_core.converters import Identity
from yaml import dump

import eagerx_bridge_test
from eagerx_core.entities import Object, Node, ResetNode, SimNode, Bridge, Converter, SpaceConverter, Processor

if __name__ == '__main__':
    # Process configuration (optional)
    node_p = process.ENVIRONMENT
    bridge_p = process.ENVIRONMENT
    rate = 7

    # todo: PLACEHOLDERS
    # Make sure to replace all default node args (e.g. rate) with object args, then replace args inside nodes.
    # Allow engine-specific params to be modified via agnostic params by coupling them with a Placeholder(name=, parameter)
    # When saving a graph, convert all Placeholders to $(ph $(name) parameter).
    # When loading a graph, convert all (ph $(name) parameter) to Placeholders.
    # How to couple space_converter inside other actions/observations? Create a weakref when copying space_converter.
    # Do not allow coupled arguments to be changed.
    # Lock coupled arguments, and substitute the placeholders when hovering/showing dialog box.
    # Inside graph.register(), create a big context dict and resolve all placeholders

    # todo: BRIDGE_SPECIFIC IMPLEMENTATION
    # Agnostic-params cannot become engine-specific --> hence, they cannot be variable.
    # Make all additional variables engine-specific & pass as object_params

    # todo: CHECK_SPEC IMPLEMENTATION
    # Check that all simnode rates are not None, similar with names etc...
    # Simnodes cannot have the same agnostic params as object --> else clash with $(default arg) placeholders --> directly resolve after objectgraph has been created.
    # Do all agnostic definitions have a space_converter?
    # Are empty components removed (e.g. inputs/outputs/targets/states or sensors/actuators/states)

    # todo: OTHER
    # replace register.init and register.add_object with automatic inspect.signature to find args. Make add_object args explicit.
    # graph check if address is None, instead of checking whether it is present.
    # graph check if cname not already connected (possibly with external address before connecting
    # Perform type checking inside register_types.
    # Move .get_params(ns) to Specs, and rename function to "build"
    # Convert None --> 'null' (and 'null' --> None, when grabbing from rosparam server)
    # Implement a function (different from make) that accepts {delays, <component>_converters, etc} to make a simnode inside object.get_params().

    N3 = ResetNode.make('RealReset', 'N3', rate=rate, process=node_p, inputs=['in_1', 'in_2'], targets=['target_1'])
    N1 = Node.make('Process', 'N1', rate=1.0, process=node_p)
    viper = Object.make('Viper', 'obj', position=[1, 1, 1], actuators=['N8'], sensors=['N6'])
    arm = Object.make('Arm', 'arm', position=[1, 1, 1], actuators=['N8'], sensors=['N6'])
    bridge = Bridge.make('TestBridge', rate=20)
    sim_actuator = SimNode.make('SimActuator', 'sim_actuator', rate=1.0, process=node_p)
    sim_sensor = SimNode.make('SimSensor', 'sim_sensor', rate=1.0, process=node_p)

    KF = Node.make('KalmanFilter', 'KF', rate=rate, process=node_p, inputs=['in_1', 'in_2'], outputs=['out_1', 'out_2'])

    # Define converter (optional)
    RosString_RosUInt64 = Converter.make('RosString_RosUInt64', test_arg='test')
    RosImage_RosUInt64 = Converter.make('RosImage_RosUInt64', test_arg='test')

    # Define converter (optional)
    from eagerx_bridge_test.converters import RosString_RosUInt64, RosImage_RosUInt64
    RosString_RosUInt64 = RosString_RosUInt64(test_arg='test')
    RosImage_RosUInt64 = RosImage_RosUInt64(test_arg='test')

    # Define nodes
    N1 = RxNode.create('N1', 'eagerx_bridge_test', 'process',   rate=1.0, process=node_p)
    N3 = RxNode.create('N3', 'eagerx_bridge_test', 'realreset', rate=rate, process=node_p, targets=['target_1'], inputs=['in_1', 'in_2'])
    KF = RxNode.create('KF', 'eagerx_bridge_test', 'kf',        rate=20.0, process=node_p, inputs=['in_1', 'in_2'], outputs=['out_1', 'out_2'])

    # Define object
    viper = RxObject.create('obj', 'eagerx_bridge_test', 'viper', position=[1, 1, 1], actuators=['N8'], sensors=['N6'])

    # Define graph
    graph = RxGraph.create(nodes=[N3, KF], objects=[viper])
    graph.render (source=(viper.name, 'sensors', 'N6'),     rate=1, converter=RosImage_RosUInt64, display=False)
    graph.render (source=(viper.name, 'sensors', 'N6'),     rate=1, converter=RosImage_RosUInt64, display=False)
    graph.connect(source=(viper.name, 'sensors', 'N6'),     observation='obs_1', delay=0.0)
    graph.connect(source=(KF.name, 'outputs', 'out_1'),     observation='obs_3', delay=0.0)
    graph.connect(source=(viper.name, 'sensors', 'N6'),     target=(KF.name, 'inputs', 'in_1'), delay=1.0)
    graph.connect(action='act_2',                           target=(KF.name, 'inputs', 'in_2'), skip=True)
    graph.connect(action='act_2',                           target=(N3.name, 'feedthroughs', 'out_1'), delay=1.0)
    graph.connect(source=(viper.name, 'sensors', 'N6'),     target=(N3.name, 'inputs', 'in_1'))
    graph.connect(source=(viper.name, 'states', 'N9'),      target=(N3.name, 'targets', 'target_1'))
    graph.connect(source=(N3.name, 'outputs', 'out_1'),     target=(viper.name, 'actuators', 'N8'), delay=1.0, converter=RosString_RosUInt64)

    # Set & get parameters
    _ = graph.get_parameter('converter', action='act_2')
    graph.set_parameter('window', 1, observation='obs_1')
    _ = graph.get_parameter('converter', observation='obs_1')
    _ = graph.get_parameter('test_arg', name=N3.name)
    _ = graph.get_parameters(viper.name, 'sensors', 'N6')
    graph.set_parameter('test_arg', 'Modified with set_parameter', name=N3.name)
    graph.set_parameter('test_arg', 'Modified with set_parameter', name=N3.name)
    graph.set_parameter('position', [1, 1, 1], name=viper.name)

    # Replace output converter
    graph.set_parameter('converter', RosString_RosUInt64, name=viper.name, component='sensors', cname='N6')  # Disconnects all connections (obs_1, KF, N3)
    graph.set_parameter('converter', Identity(), name=viper.name, component='sensors', cname='N6')  # Disconnects all connections (obs_1, KF, N3)
    graph.render (source=(viper.name, 'sensors', 'N6'),     rate=1, converter=RosImage_RosUInt64)  # Reconnect
    graph.connect(source=(viper.name, 'sensors', 'N6'), observation='obs_1', delay=0.0)  # Reconnect
    graph.connect(source=(viper.name, 'sensors', 'N6'), target=(KF.name, 'inputs', 'in_1'), delay=1.0)  # Reconnect
    graph.connect(source=(viper.name, 'sensors', 'N6'), target=(N3.name, 'inputs', 'in_1'))  # Reconnect

    # Remove component. For action/observation use graph._remove_action/observation(...) instead.
    # graph.remove_component(observation='obs_1')
    # graph.remove_component(KF.name, 'outputs', 'out_1')
    graph.remove_component(N3.name, 'inputs', 'in_2')

    # Rename entity (object/node) and all associated connections
    graph.rename(KF.name, 'KF2')
    graph.rename('KF2', KF.name)

    # Rename action & observation
    graph.rename('act_2', 'act_1', name='env/actions', component='outputs')
    graph.rename('obs_3', 'obs_2', observation='obs_2')

    # Remove & add action (without action terminal removal)
    graph.disconnect(action='act_1', target=(KF.name, 'inputs', 'in_2'))
    graph.connect(action='act_1', target=(KF.name, 'inputs', 'in_2'), converter=None, delay=None, window=None, skip=True)

    # Remove & add observation (with observation terminal removal)
    graph.disconnect(source=(viper.name, 'sensors', 'N6'), observation='obs_1')
    graph.add_component(observation='obs_1')  # Add input terminal
    graph.connect(source=(viper.name, 'sensors', 'N6'), observation='obs_1', converter=None, delay=None, window=None)

    # Remove & add other input
    graph.disconnect(source=(viper.name, 'sensors', 'N6'), target=(KF.name, 'inputs', 'in_1'))
    graph.connect(source=(viper.name, 'sensors', 'N6'), target=(KF.name, 'inputs', 'in_1'))

    # Works with other sources as well, but then specify "source" instead of "action" as optional arg to connect(..) and disconnect(..).
    graph.disconnect(source=(viper.name, 'sensors', 'N6'), observation='obs_1', remove=False)  # NOTE: with the remove=False flag, we avoid removing terminal 'obs_1'

    # GUI routine for making connections
    source = (viper.name, 'sensors', 'N6')
    target = ('env/observations', 'inputs', 'obs_1')
    # GUI: Identify if source/target is action/observation
    observation = target[2] if target[0] == 'env/observations' else None
    action = source[2] if source[0] == 'env/actions' else None
    params = graph.get_parameters(name=target[0], component=target[1], cname=target[2])  # Grab already defined parameters from input component
    if len(params) == 0:  # If observation, dict will be empty.
        converter = graph.get_parameter(parameter='space_converter', name=source[0], component=source[1], cname=source[2],
                                        default=Identity)  # grab space_converter from source (viper.name, 'sensors', 'N6')
        delay, window = 0, 0
    else:  # If not observation, these values will always be present
        converter, delay, window = params['converter'], params['delay'], params['window']
    # GUI: open dialogue box where users can modify converter, delay, window etc... Use previous params to set initial values.
    # GUI: converter, delay, window = ConnectionOptionsDialogueBox(converter, delay, window)
    target = None if observation else target  # If we have an observation, it will be the target instead in .connect(..)
    source = None if action else source  # If we have an action, it will be the source instead in .connect(..)
    # GUI: use the modified params via the dialogue box to connect.
    graph.connect(source=source, target=target, action=action, observation=observation, converter=converter, delay=delay, window=window)

    # TEST Test with KF having skipped all inputs at t=0
    graph.remove_component(KF.name, 'inputs', 'in_1')

    # graph.gui()

    # Test save & load functionality
    graph.save('./test.graph')
    graph.load('./test.graph')

    # Define bridge
    bridge = RxBridge.create('eagerx_bridge_test', 'bridge', rate=20, is_reactive=True, real_time_factor=0, process=bridge_p)

    # Initialize Environment
    env = EAGERxEnv(name='rx',
                    rate=rate,
                    graph=graph,
                    bridge=bridge,
                    reset_fn=lambda env: {'obj/N9': env.state_space.sample()['obj/N9'],
                                          'obj/N10': env.state_space.sample()['obj/N10'],
                                          'bridge/param_1': env.state_space.sample()['bridge/param_1']}
                    )
    env = Flatten(env)

    # First reset
    obs = env.reset()
    env.render(mode='human')
    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(20):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            # rgb = env.render(mode='rgb_array')
        obs = env.reset()
    print('\n[Finished]')

    # todo: DOCUMENTATION
    #  - RxNode.create(...), RxBridge.create(...), RxObject.create(...)
    #  - RxEnv, EAGERxEnv
    #  - Converter, Processor, SpaceConverter
    #  - SimState

    # todo: THINGS TO KEEP IN MIND:
    #  - If output converters are used on simnodes, you risk breaking the object's simulation graph (as some simnodes might expect an non-converted message).
    #  - The order in which you define env actions matters when including input converters. Namely, the first space_converter is chosen.
    #  - The exact moment of switching to a real reset cannot be predicted by any node, thus this introduces
    #  race-conditions in the timing of the switch that cannot be mitigated with a reactive scheme.
    #  - Similarly, it cannot be predicted whether a user has tried to register an object before calling "env.reset()".
    #  Hence, we cannot completely rule out timing issues with a reactive scheme. Could therefore cause a deadlock (but
    #  chance is very slim, and only at the moment of initialization).
    #  - Currently, we assume that **all** nodes & objects are registered and initialized before the user calls reset.
    #  Hence, we cannot adaptively register new objects or controllers after some episodes.
    #  - If we have **kwargs in callback/reset signature, the node.py implementation supports adding inputs/states.
    #  - Only objects can have nonreactive inputs. In that case, the bridge is responsible for sending flag msgs (num_msgs_send).
    #  The bridges knows which inputs are nonreactive when the object is registered.
    #  - Nodes **must** at all times publish an output. Even, when a node did not received any new inputs and wishes to not publish.
    #  Perhaps, this constraint could be softened in the async setting, however the nodes that send "None", would then
    #  not be agnostic (as they would break in the case is_reactive=True).
    #  - Every package that contains eagerx nodes/objects/converters must start with "eagerx", else they cannot be added within the GUI.
    #  - Nonreactive inputs that have 'start_with_msg' could mess up the reset.
    #  - Delays are ignored when running async.
    #  - In the bridge definition of an object, or inside the config of simnodes, there cannot be converters defined for
    #  the components related to the sensor and actuator. Reason for this is that if a converter would already be defined there,
    #  then it is not possible anymore to add another one in the agnostic side.