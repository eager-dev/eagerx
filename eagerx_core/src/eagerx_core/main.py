# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv
from eagerx_core.utils.node_utils import configure_connections, launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten


if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define converter (optional)
    ImageUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/ImageUInt64Converter', 'test_arg': 'test'}
    StringUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/StringUInt64Converter', 'test_arg': 'test'}

    # Process configuration (optional)
    rate = 10
    node_p = process.NEW_PROCESS
    bridge_p = process.NEW_PROCESS

    # Define nodes
    N1 = RxNode.create('N1', 'eagerx_core', 'process',   rate=1.0, process=node_p)
    N3 = RxNode.create('N3', 'eagerx_core', 'realreset', rate=rate, process=node_p, targets=['target_1'])
    KF = RxNode.create('KF', 'eagerx_core', 'kf',        rate=20.0, process=node_p, inputs=['in_1', 'in_2'], outputs=['out_1'])

    # Define object
    viper = RxObject.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N8'], sensors=['N6'])

    # Define action/observations
    actions, observations = EAGERxEnv.create_actions(), EAGERxEnv.create_observations()

    # Define render (optional)
    render = EAGERxEnv.create_render(rate=1)

    # Connect nodes
    # connections = [{'source': (viper, 'sensors', 'N6'), 'target': (render, 'inputs', 'image'), 'converter': ImageUInt64Converter},
    #                {'source': (viper, 'sensors', 'N6'), 'target': (observations, 'obs_1'), 'delay': 0.0},
    #                {'source': (actions, 'act_1'),       'target': (viper, 'actuators', 'N8'), 'delay': 0.0},
    #                ]

    # connections = [{'source': (viper, 'sensors', 'N6'), 'target': (render, 'inputs', 'image'), 'converter': ImageUInt64Converter},
    #                {'source': (viper, 'sensors', 'N6'), 'target': (KF, 'inputs', 'in_1')},
    #                {'source': (actions, 'act_1'),       'target': (KF, 'inputs', 'in_2')},
    #                {'source': (KF, 'out_1'),            'target': (observations, 'obs_1'), 'delay': 0.0},
    #                {'source': (actions, 'act_1'),       'target': (viper, 'actuators', 'N8'), 'converter': StringUInt64Converter, 'delay': 1.0},
    #                ]

    connections = [{'source': (viper, 'sensors', 'N6'), 'target': (render, 'inputs', 'image'), 'converter': ImageUInt64Converter},
                   {'source': (viper, 'sensors', 'N6'), 'target': (observations, 'obs_1'), 'delay': 0.0},
                   {'source': (KF, 'out_1'),            'target': (observations, 'obs_2'), 'delay': 0.0},
                   {'source': (viper, 'sensors', 'N6'), 'target': (KF, 'inputs', 'in_1')},
                   {'source': (actions, 'act_1'),       'target': (KF, 'inputs', 'in_2')},
                   {'source': (viper, 'sensors', 'N6'), 'target': (N3, 'inputs', 'in_1')},
                   {'source': (viper, 'states', 'N9'),  'target': (N3, 'targets', 'target_1')},
                   {'source': (actions, 'act_1'),       'target': (N3, 'feedthroughs', 'out_1')},
                   {'source': (N3, 'out_1'),            'target': (viper, 'actuators', 'N8'), 'converter': StringUInt64Converter, 'delay': 1.0},
                   ]
    configure_connections(connections)

    # Define bridge
    bridge = RxBridge.create('eagerx_core', 'bridge', rate=20, num_substeps=10, process=bridge_p,
                             is_reactive=True, real_time_factor=0)

    # Initialize Environment
    env = EAGERxEnv(name='rx',
                    rate=rate,
                    actions=actions,
                    observations=observations,
                    bridge=bridge,
                    # nodes=[],
                    # nodes=[KF],
                    nodes=[N3, KF],
                    objects=[viper],
                    # render=render,
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
        for i in range(200000):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            # rgb = env.render(mode='rgb_array')
        obs = env.reset()
    print('\n[Finished]')

    # todo: CONVERTERS
    #  - Create a .yaml interface for converters, similar to objects & nodes
    #  - Create a converter baseclass for processor converters that map to the same datatype
    #     --> assert in regular converter that msg_types are not equal
    #     --> Implement same functions for processor class as for converters

    # todo: IMPROVE CONFIGURE CONNECTIONS
    #  - Separate logic if dealing with either action or observation node
    #  - Separate checks on msg_types to the end (addresses set for all selected I/O, actuators/sensors, etc...)
    # todo: get msg_type from python implementation? --> Avoid differences between .yaml and .py --> Or create a check when creating such a node?
    # todo: msg_type check on msg_type inside object.yaml and simnode.yaml (possibly with an input/output converter in-between)
    # todo: msg_type check for "{'source': (actions, 'act_1'), 'target': (viper, 'actuators', 'N8'), 'converter': StringUInt64Converter},"
    #       Here, we do use both the converter and space_converters ---> will result in error and must be checked before initialization.

    # todo: OTHER
    # todo: initialize converters as usual, but give them a converter.to_dict() function that converts them to .yaml format that can be uploaded to the rosparam server
    # todo: OpenAI gym bridge + Wrapper that excludes is_done & reward from observations --> Sum received rewards.
    # todo: Improve load_yaml(..) --> loop through all files (also inside subdirectories) in config dir --> Error on duplicate .yaml file names.

    # todo: REACTIVE PROTOCOL
    # todo: Remove observe_on and experiment with other schedulers.
    # todo: non_reactive input sometimes misses input msgs (send>recv) --> why?
    # todo: Find out why connection is repeatedly created every new episode --> env.render(..)

    # todo: ASYNCHRONOUS IMPLEMENTATION
    # todo: What is the effect of async simulation with action & observation node?

    # todo: CREATE GITHUB ISSUES FOR:
    # todo; Currently, converters must always convert to different datatypes. Create a pre-processor class that converts to the same type.
    # todo: Create a general OpenAI bridge
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
    # todo: Create an option to turn on/off delays when running async --> rx.operators.delay(duetime, scheduler=None)
    # todo: Create a node diagonostics topic that contains 'info on node Hz, input HZs, input recv vs send & other flags

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

