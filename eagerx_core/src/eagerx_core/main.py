# ROS packages required
import rospy
from eagerx_core.params import RxObjectParams, RxNodeParams, RxBridgeParams
from eagerx_core.env import Env
from eagerx_core.utils.utils import launch_roscore
from eagerx_core.utils.node_utils import configure_connections
from time import sleep

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define converter (optional)
    IntUInt64Converter = {'converter_type': 'eagerx_core.converter/IntUInt64Converter', 'test_arg': 'test'}

    # Define nodes
    sp = True
    N1 = RxNodeParams.create('N1', 'eagerx_core', 'process',   rate=1.0, single_process=sp, outputs=['out_1', 'out_2'])
    N3 = RxNodeParams.create('N3', 'eagerx_core', 'realreset', rate=1.0, single_process=sp, targets=['target_1'])
    N4 = RxNodeParams.create('N4', 'eagerx_core', 'process',   rate=3.3, single_process=sp, output_converters={'out_1': IntUInt64Converter})
    N5 = RxNodeParams.create('N5', 'eagerx_core', 'process',   rate=6, single_process=sp, output_converters={'out_1': IntUInt64Converter})
    KF = RxNodeParams.create('KF', 'eagerx_core', 'kf',        rate=1, single_process=sp, inputs=['in_1', 'in_2'])

    # Define object
    viper = RxObjectParams.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N8'])

    # Define action/observations/states
    actions, observations = Env.define_actions(), Env.define_observations()

    # Connect nodes
    connections = [{'source': (KF, 'out_1'),            'target': (observations, 'obs_1'), 'delay': 0.0},
                   {'source': (viper, 'sensors', 'N7'), 'target': (KF, 'inputs', 'in_1')},
                   {'source': (actions, 'act_1'),       'target': (KF, 'inputs', 'in_2')},
                   {'source': (actions, 'act_1'),       'target': (N1, 'inputs', 'in_1')},
                   {'source': (viper, 'sensors', 'N6'), 'target': (N3, 'inputs', 'in_1')},
                   {'source': (viper, 'states', 'N9'),  'target': (N3, 'targets', 'target_1')},
                   {'source': (N1, 'out_2'),            'target': (N3, 'feedthroughs', 'out_1')},
                   {'source': (N3, 'out_1'),            'target': (N4, 'inputs', 'in_1')},
                   {'source': (N4, 'out_1'),            'target': (N5, 'inputs', 'in_1'), 'converter': IntUInt64Converter},
                   {'source': (N5, 'out_1'),            'target': (viper, 'actuators', 'N8'), 'converter': IntUInt64Converter, 'delay': 1.0},
                   ]
    configure_connections(connections)

    # Define bridge
    bridge = RxBridgeParams.create('eagerx_core', 'bridge', rate=1, num_substeps=10, single_process=True)

    # Initialize Environment
    env = Env(name='rx',
              rate=1,
              actions=actions,
              observations=observations,
              bridge=bridge,
              nodes=[N1, N3, N4, N5, KF],
              objects=[viper])

    # First reset
    obs = env.reset()
    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(2):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
        obs = env.reset()
    print('\n[Finished]')
    sleep(100000)

    # todo: CheckEnv(env): i/o correct, fully connected & DAG when RealReset (check graph without all nodes dependent on Env's actions)
    # todo: Visualize and perform checks on DAGs (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: How to combine GUI with custom env?

    # todo: implement real_time rx pipeline

    # todo: CLEAN-UP ACTIONS
    # todo: pass (default) args to node_cls(...). Currently done via the paramserver? Make explicit in constructor.
    # todo: make baseclasses for bridge, node, simstate
    # todo: replace reset info with rospy.logdebug(...), so that we log it if warn level is debug
    # todo: change structure of callback/reset input: unpack to descriptive arguments e.g. reset(tick, state)
    # todo: add msg_types as static properties to node.py implementation (make class more descriptive)
    # todo: create publishers in RxMessageBroker (outputs,  node_outputs)
    # todo: cleanup eagerx_core.__init__: refactor to separate rxpipelines.py, rxoperators.py
    # todo: print statements of callback inside ProcessNode: color specified as additional argument

    # todo: CREATE ISSUES FOR:
    # todo: create a register_node function in the RxNode class to initialize a node inside the process of another node.
    # todo; how to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write?
