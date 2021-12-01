# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv
from eagerx_core.utils.node_utils import configure_connections, launch_roscore
from eagerx_core.constants import process
if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define converter (optional)
    IntUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/IntUInt64Converter', 'test_arg': 'test'}
    ImageUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/ImageUInt64Converter', 'test_arg': 'test'}
    StringUInt64Converter = {'converter_type': 'eagerx_core.baseconverter/StringUInt64Converter', 'test_arg': 'test'}

    # Process configuration (optional)
    node_p = process.ENVIRONMENT
    bridge_p = process.ENVIRONMENT

    # Define nodes
    N1 = RxNode.create('N1', 'eagerx_core', 'process',   rate=1.0, process=node_p, outputs=['out_1', 'out_2'])
    N3 = RxNode.create('N3', 'eagerx_core', 'realreset', rate=1.0, process=node_p, targets=['target_1'])
    N4 = RxNode.create('N4', 'eagerx_core', 'process',   rate=3.3, process=node_p, output_converters={'out_1': StringUInt64Converter})
    N5 = RxNode.create('N5', 'eagerx_core', 'process',   rate=4,   process=node_p, inputs=['in_1'])
    KF = RxNode.create('KF', 'eagerx_core', 'kf',        rate=1,   process=node_p, inputs=['in_1', 'in_2'])

    # Define object
    viper = RxObject.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N8'])

    # Define action/observations
    actions, observations = EAGERxEnv.create_actions(), EAGERxEnv.create_observations()

    # Define render
    render = EAGERxEnv.create_render(rate=1)

    # Connect nodes
    connections = [{'source': (viper, 'sensors', 'N6'), 'target': (render, 'inputs', 'image'), 'converter': ImageUInt64Converter},
                   {'source': (KF, 'out_1'),            'target': (observations, 'obs_1'), 'delay': 0.0},
                   {'source': (viper, 'sensors', 'N7'), 'target': (KF, 'inputs', 'in_1')},
                   {'source': (actions, 'act_1'),       'target': (KF, 'inputs', 'in_2')},
                   {'source': (actions, 'act_1'),       'target': (N1, 'inputs', 'in_1')},
                   {'source': (viper, 'sensors', 'N6'), 'target': (N3, 'inputs', 'in_1')},
                   {'source': (viper, 'states', 'N9'),  'target': (N3, 'targets', 'target_1')},
                   {'source': (N1, 'out_2'),            'target': (N3, 'feedthroughs', 'out_1')},
                   {'source': (N3, 'out_1'),            'target': (N4, 'inputs', 'in_1')},
                   {'source': (N4, 'out_1'),            'target': (N5, 'inputs', 'in_1'), 'converter': StringUInt64Converter},
                   {'source': (N5, 'out_1'),            'target': (viper, 'actuators', 'N8'), 'converter': StringUInt64Converter, 'delay': 1.0},
                   ]
    configure_connections(connections)

    # Define bridge
    bridge = RxBridge.create('eagerx_core', 'bridge', rate=1, num_substeps=10, process=bridge_p)

    # Initialize Environment
    env = EAGERxEnv(name='rx',
                    rate=1,
                    actions=actions,
                    observations=observations,
                    bridge=bridge,
                    nodes=[N1, N3, N4, N5, KF],
                    objects=[viper],
                    render=render,
                    reset_fn=lambda env: {'obj/N9': env.state_space.sample()['obj/N9'],
                                          'N1/state_1': env.state_space.sample()['N1/state_1'],
                                         }
                    )

    # First reset
    obs = env.reset()
    env.render(mode='human')
    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(2):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            rgb = env.render(mode='rgb_array')
        obs = env.reset()
    print('\n[Finished]')

    # todo: implement real_time rx pipeline
    # todo: add states to bridge (for domain randomization)

    # todo: CREATE GITHUB ISSUES FOR:
    # todo: Create a register_node function in the RxNode class to initialize a node inside the process of another node.
    # todo: How to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write?
    # todo: Create a ThreadSafe simulator object (that can be safely accessed from multiple simulation nodes at once)
    # todo: Bridge states that resemble simulator parameters that a user may want to vary between episodes (domain
    #       randomization)
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
    # todo: The order in which you define env actions matters when including input converters. Namely, the first space_converter is chosen.
    # todo: The exact moment of switching to a real reset cannot be predicted by any node, thus this introduces
    #  race-conditions in the timing of the switch that cannot be mitigated with a reactive scheme.
    # todo: Similarly, it cannot be predicted whether a user has tried to register an object before calling "env.reset()".
    #  Hence, we cannot completely rule out timing issues with a reactive scheme. Could therefore cause a deadlock (but
    #  chance is very slim, and only at the moment of initialization).
    # todo: Currently, we assume that **all** nodes & objects are registered and initialized before the user calls reset.
    #  Hence, we cannot adaptively register new objects or controllers after some episodes.
    # todo: If we have **kwargs in callback/reset signature, the node.py implementation supports adding inputs/states.
    # todo: Only objects can have nonreactive inputs. In that case, the bridge is responsible for sending flag msgs (num_msgs_send).
    #  The bridges knows which inputs are nonreactive when the object is registered.

