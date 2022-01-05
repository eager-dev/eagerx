# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv, RxGraph
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten
from eagerx_core.baseconverter import StringUInt64Converter, ImageUInt64Converter

from yaml import dump

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.DEBUG)

    # Define converter (optional)
    StringUInt64Converter = StringUInt64Converter(test_arg='test')
    ImageUInt64Converter = ImageUInt64Converter(test_arg='test')

    # Process configuration (optional)
    node_p = process.ENVIRONMENT
    bridge_p = process.ENVIRONMENT
    rate = 7

    # Define nodes
    N1 = RxNode.create('N1', 'eagerx_core', 'process',   rate=1.0, process=node_p)
    N3 = RxNode.create('N3', 'eagerx_core', 'realreset', rate=rate, process=node_p, targets=['target_1'], inputs=['in_1', 'in_2'])
    KF = RxNode.create('KF', 'eagerx_core', 'kf',        rate=20.0, process=node_p, inputs=['in_1', 'in_2'], outputs=['out_1'])

    # Define object
    viper = RxObject.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N8'], sensors=['N6'])

    # Define graph
    graph = RxGraph.create(nodes=[N3, KF], objects=[viper])
    graph.render (source=(viper.name, 'sensors', 'N6'),     rate=1, converter=ImageUInt64Converter)
    graph.render (source=(viper.name, 'sensors', 'N6'),     rate=1, converter=ImageUInt64Converter)
    graph.connect(source=(viper.name, 'sensors', 'N6'),     observation='obs_1', delay=0.0)
    graph.connect(source=(KF.name,    'outputs', 'out_1'),  observation='obs_3', delay=0.0)
    graph.connect(source=(viper.name, 'sensors', 'N6'),     target=(KF.name, 'inputs', 'in_1'), delay=1.0)
    graph.connect(action='act_2',                           target=(KF.name, 'inputs', 'in_2'))
    graph.connect(action='act_2',                           target=(N3.name, 'feedthroughs', 'out_1'), delay=1.0)
    graph.connect(source=(viper.name, 'sensors', 'N6'),     target=(N3.name, 'inputs', 'in_1'))
    graph.connect(source=(viper.name, 'states', 'N9'),      target=(N3.name, 'targets', 'target_1'))
    graph.connect(source=(N3.name,    'outputs', 'out_1'),  target=(viper.name, 'actuators', 'N8'), delay=1.0, converter=StringUInt64Converter)

    # Replace output converter
    graph._replace_converter(viper.name, 'sensors', 'N6', StringUInt64Converter)  # Disconnects all connections (obs_1, KF, N3)
    graph._reset_converter(viper.name, 'sensors', 'N6')  # Resets converter back to default --> IdentityConverter
    graph.render (source=(viper.name, 'sensors', 'N6'),     rate=1, converter=ImageUInt64Converter)  # Reconnect
    graph.connect(source=(viper.name, 'sensors', 'N6'),     observation='obs_1', delay=0.0)  # Reconnect
    graph.connect(source=(viper.name, 'sensors', 'N6'),     target=(KF.name, 'inputs', 'in_1'), delay=1.0)  # Reconnect
    graph.connect(source=(viper.name, 'sensors', 'N6'),     target=(N3.name, 'inputs', 'in_1'))  # Reconnect

    # Remove component. For action/observation use graph._remove_action/observation(...) instead.
    graph._remove_component(N3.name, 'inputs', 'in_2')

    # Rename entity (object/node) and all associated connections
    graph.rename(KF.name, 'KF2')
    KF.name = 'KF2'

    # Rename action & observation
    graph.rename_component('env/actions', 'outputs', 'act_2', 'act_1')
    graph.rename_component('env/observations', 'inputs', 'obs_3', 'obs_2')

    # Remove & add action (with action terminal removal)
    graph.disconnect(action='act_1', target=(KF.name, 'inputs', 'in_2'))
    graph.connect(action='act_1', target=(KF.name, 'inputs', 'in_2'), converter=None, delay=None, window=None)

    # Remove & add observation (with observation terminal removal)
    graph.disconnect(source=(viper.name, 'sensors', 'N6'), observation='obs_1')
    graph.connect(source=(viper.name, 'sensors', 'N6'), observation='obs_1', converter=None, delay=None, window=None)

    # Remove & add action (without action terminal removal)
    graph._disconnect(action='act_1', target=(KF.name, 'inputs', 'in_2'))
    graph.connect(action='act_1', target=(KF.name, 'inputs', 'in_2'), converter=None, delay=None, window=None)

    # Remove & add observation (without observation terminal removal)
    graph._disconnect(source=(viper.name, 'sensors', 'N6'), observation='obs_1')
    graph.connect(source=(viper.name, 'sensors', 'N6'), observation='obs_1', converter=None, delay=None, window=None)

    # Remove & add other input
    graph.disconnect(source=(viper.name, 'sensors', 'N6'), target=(KF.name, 'inputs', 'in_1'))
    graph.connect(source=(viper.name, 'sensors', 'N6'), target=(KF.name, 'inputs', 'in_1'))

    # Completely remove & add new action (GUI METHOD)
    graph.disconnect(action='act_1', target=(KF.name, 'inputs', 'in_2'))
    graph._add_action(action='act_1')
    graph._connect_action(action='act_1', target=(KF.name, 'inputs', 'in_2'), converter=None)  # add input_converter here if specified
    graph._connect(source=('env/actions', 'outputs', 'act_1'), target=(KF.name, 'inputs', 'in_2'), converter=None, delay=None, window=None)  # add input_converter, delay, window here if specified

    # Completely remove & add new action (GUI METHOD)
    graph.disconnect(action='act_1', target=(KF.name, 'inputs', 'in_2'))
    graph._add_action(action='act_1')
    graph._connect_action(action='act_1', target=(KF.name, 'inputs', 'in_2'), converter=None)  # add input_converter here if specified
    graph._connect(source=('env/actions', 'outputs', 'act_1'), target=(KF.name, 'inputs', 'in_2'), converter=None, delay=None, window=None)  # add input_converter, delay, window here if specified

    # Completely remove & add new observation (GUI METHOD)
    graph.disconnect(source=(viper.name, 'sensors', 'N6'), observation='obs_1')
    graph._add_observation(observation='obs_1')
    converter = graph._connect_observation(source=(viper.name, 'sensors', 'N6'), observation='obs_1', converter=None)  # add input_converter here if specified
    graph._connect(source=(viper.name, 'sensors', 'N6'), target=('env/observations', 'inputs', 'obs_1'), converter=converter, delay=None, window=None)  # add input_converter, delay, window here if specified

    # Test save & load functionality
    graph.save('./test.graph')
    graph.load('./test.graph')

    # Define bridge
    bridge = RxBridge.create('eagerx_core', 'bridge', rate=20, is_reactive=True, real_time_factor=0, process=bridge_p)

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

    # todo: CONVERTERS
    #  - Converters & objects (what is and is not supported?) --> remove converters from objects

    # todo: OTHER
    # todo: Separate test bridge into a separate ROS package outside of eagerx_core
    # todo: add networkx to dependency

    # todo: REACTIVE PROTOCOL
    # todo: non_reactive input sometimes misses input msgs (send>recv) --> why?
    # todo: Find out why connection is repeatedly created every new episode --> env.render(..)

    # todo: GUI
    #  - Create a "check_graph" button in the GUI that runs is_valid()
    #  - Do not show converter for states in GUI
    #  - Do not show space_converter in actions/observations

    # todo: CREATE GITHUB ISSUES FOR:
    # todo: How to infer all available and compatible converters (.yaml interface, find all subclasses of BaseConverter?)
    # todo: Get msg_type from python implementation to avoid differences between .yaml and .py
    # todo: Change 'default' to 'config' after yaml has been loaded.
    # todo: Create a general OpenAI bridge + wrapper that excludes is_done & reward from observations --> sum received rewards.
    # todo: Implement display functionality inside the render node.
    # todo: Create a register_node function in the RxNode class to initialize a node inside the process of another node.
    # todo: How to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write? NamedTuples are immutable (but their elements probably aren't)
    # todo: Create a ThreadSafe simulator object (that can be safely accessed from multiple simulation nodes at once)
    # todo: CheckEnv(env): i/o correct, fully connected & DAG when RealReset (check graph without all nodes dependent on Env's actions)
    #       (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: Put a timeout on nonreactive inputs (based on ticks), to hold msgs if after tick, and repeat of timeout
    #       reached (count how many repeats)
    # todo: Currently, we add bridge node twice to message_broker with '/dynamically_registered' appended to avoid a
    #       namespace clash between done flags used in both real_reset & state_reset
    # todo: Create batch dimension on all nodes (with $(batch_dim) in .yaml used in e.g. converter definitions), so that
    #       complete envs can be batched. Bridge must have a grid edge length param, simnodes that use global position
    #       info must correct for grid position.
    # todo: Implement rxpipeline in c++ for increased speed. Interface with both python & cpp Node baseclasses.
    # todo: Create an option to turn on/off delays when running async --> rx.operators.delay(duetime, scheduler=None)
    # todo: Create a node diagnostics topic that contains 'info on node Hz, input HZs, input recv vs send & other flags

    # todo: THINGS TO KEEP IN MIND:
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
    #  - nodes **must** at all times publish an output. Even, when a node did not received any new inputs and wishes to not publish.
    #  Perhaps, this constraint could be softened in the async setting, however the nodes that send "None", would then
    #  not be agnostic (as they would break in the case is_reactive=True).
    #  - Every package that contains eagerx nodes/objects/converters must start with "eagerx", else they cannot be added within the GUI.
    #  - Nonreactive inputs that 'start_with_msg' could mess up the reset.
    #  - Delays are ignored when running async.

