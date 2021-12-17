# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv
from eagerx_core.utils.node_utils import configure_connections, launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten


if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.DEBUG)

    # Define converter (optional)
    ImageUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/ImageUInt64Converter', 'test_arg': 'test'}
    StringUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/StringUInt64Converter', 'test_arg': 'test'}

    # Process configuration (optional)
    node_p = process.ENVIRONMENT
    bridge_p = process.ENVIRONMENT

    # Define nodes
    N1 = RxNode.create('N1', 'eagerx_core', 'process',   rate=1.0, process=node_p)
    N3 = RxNode.create('N3', 'eagerx_core', 'realreset', rate=20.0, process=node_p, targets=['target_1'])
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
    bridge = RxBridge.create('eagerx_core', 'bridge', rate=20, num_substeps=10, process=bridge_p, is_reactive=True, real_time_factor=0)

    # Initialize Environment
    env = EAGERxEnv(name='rx',
                    rate=20,
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
        for i in range(2):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            # rgb = env.render(mode='rgb_array')
        obs = env.reset()
    print('\n[Finished]')

    # True & real_time_factor!= 0: rospy.rate behavior --> implement rospy.sleep() inside bridge tick
    # False & real_time_factor=0: ERROR --> cannot run as fast as possible asynchronous (without a common clock)
    # True & real_time_factor = 0,  # --> as fast as possible and disregard real_time_factor

    # todo: Improve load_yaml(..) --> loop through all files (also inside subdirectories) in config dir --> Error on duplicate .yaml file names.
    # todo: non_reactive input sometimes misses input msgs (send>recv) --> why?
    # todo: get msg_type from python implementation? --> Avoid differences between .yaml and .py --> Or create a check when creating such a node?
    # todo: msg_type check on msg_type inside object.yaml and simnode.yaml (possibly with an input/output converter in-between)
    # todo: msg_type check for "{'source': (actions, 'act_1'), 'target': (viper, 'actuators', 'N8'), 'converter': StringUInt64Converter},"
    #       Here, we do use both the converter and space_converters ---> will result in error and must be checked before initialization.
    # todo: Find out why connection is repeatedly created every new episode --> env.render(..)

    # todo: ASYNCHRONOUS IMPLEMENTATION
    # todo: make real_time_factor implementation faster, with time.sleep?
    # todo: change reset msg recv logic from msg_rcv=msg_send to msg_rcv > msg_send --> Can be problematic if we miss first few messages.
    # todo: why not latch init_msg to 'rx/end_reset'?
    # todo: add 'ops.share" with gen_msg
    # todo: what to do with assertion in "rxoperators.regroup_msgs"
    # todo: implement real_time rx pipeline
    #  - Filter None messages --> no problem because flags are not counting msg_send==msg_recv
    #  - Latch input channels that are non-reactive & non-deterministic.
    #  - How to deal with delay? --> Ignore when running async? Or implement as delay block? Must be removed when running in reality.
    #  - What action to feedthrough if we receive 0 or 2? Always select the action we received last? What to do at t=0?
    #  - What is the effect of async simulation with action & observation node?
    #    If running async, sample last action from action_node? Threadsafe?

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

    # todo: REPO STRUCTURE
    #  - eagerx_core --> should be installed as a ros package via sudo apt-get install eagerx_core --> (add supported ros version?)
    #    Perhaps, with installing eagerx_core, subdirectories "bridge", "object", "Nodes" is created where other objects etc.. are installed in.
    #  - eagerx_bridge_... --> should be installed as a ros package via sudo apt-get install eagerx_bridge_... (add supported ros version?)
    #  - eagerx_object_... --> should be cloned inside workspace where you wish to use that object --> motivation: you want easy access to object config yaml
    #  - eagerx_node_... --> should be cloned inside workspace where you wish to use that node --> motivation: you want easy access to node config yaml

