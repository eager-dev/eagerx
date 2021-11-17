# ROS packages required
import rospy
from eagerx_core.params import RxObjectParams, RxNodeParams
from eagerx_core.env import Env
from eagerx_core.utils.utils import launch_roscore
from eagerx_core.utils.node_utils import configure_connections

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define converter (optional)
    IntUInt64Converter = {'converter_type': 'eagerx_core.converter/IntUInt64Converter', 'test_arg': 'test'}

    # Define nodes
    # todo: remove output addresses from create fn? Input as list, inside .create change to dict with naming convention
    # todo: Add state selection via list? Then, add names via naming convention?
    # todo: add default states & outputs in yaml
    sp = False
    N1 = RxNodeParams.create('N1', 'eagerx_core', 'process',   rate=1, single_process=sp, outputs={'out_1': 'N1/P1', 'out_2': 'N2/P2'})
    N3 = RxNodeParams.create('N3', 'eagerx_core', 'realreset', rate=1, single_process=sp, outputs={'out_1': 'N3/P3'})
    N4 = RxNodeParams.create('N4', 'eagerx_core', 'process',   rate=3, single_process=sp, outputs={'out_1': 'N4/P4'}, output_converters={'out_1': IntUInt64Converter})
    N5 = RxNodeParams.create('N5', 'eagerx_core', 'process',   rate=6, single_process=sp, outputs={'out_1': 'N5/P5'}, output_converters={'out_1': IntUInt64Converter})
    KF = RxNodeParams.create('KF', 'eagerx_core', 'kf',        rate=1, single_process=sp, outputs={'out_1': 'KF/S'})

    # Define object
    # todo: why no error with 'single_process=sp'?
    # todo: select states here
    viper = RxObjectParams.create('obj', 'eagerx_core', 'viper', single_process=sp, position=[1, 1, 1], actuators=['N8'])

    # Define action/observations/states
    # todo: create config.yaml for these dicts
    actions, observations, states = Env.define_actions(), Env.define_observations(), Env.define_states()

    # Connect nodes
    connections = [{'source': (KF, 'out_1'),            'target': (observations, 'obs_1', 'all'), 'delay': 0.0},
                   {'source': (viper, 'sensors', 'N7'), 'target': (KF, 'inputs', 'in_1')},
                   {'source': (actions, 'act_1'),       'target': (KF, 'inputs', 'in_2')},
                   {'source': (actions, 'act_1'),       'target': (N1, 'inputs', 'in_1'), 'delay': 1.0},
                   {'source': (viper, 'sensors', 'N6'), 'target': (N3, 'inputs', 'in_1')},
                   {'source': (N1, 'out_2'),            'target': (N3, 'feedthroughs', 'out_1')},
                   {'source': (N3, 'out_1'),            'target': (N4, 'inputs', 'in_1')},
                   {'source': (N4, 'out_1'),            'target': (N5, 'inputs', 'in_1'), 'converter': IntUInt64Converter},
                   {'source': (N5, 'out_1'),            'target': (viper, 'actuators', 'N8'), 'converter': IntUInt64Converter, 'delay': 1.0},
                   {'source': (states, 'N1'),           'target': (N1, 'states', 'state_1')},
                   {'source': (states, 'N3'),           'target': (N3, 'states', 'state_1')},
                   {'source': (states, 'N4'),           'target': (N4, 'states', 'state_1')},
                   {'source': (states, 'N5'),           'target': (N5, 'states', 'state_1')},
                   {'source': (states, 'KF'),           'target': (KF, 'states', 'state_1')},
                   {'source': (states, 'N9'),           'target': (viper, 'states', 'N9')},
                   {'source': (states, 'N9'),           'target': (N3, 'targets', 'target_1')},
                   {'source': (states, 'N10'),          'target': (viper, 'states', 'N10')},
                   ]
    configure_connections(connections)

    # Define bridge
    bridge = RxNodeParams.create('bridge', 'eagerx_core', 'bridge', rate=1, num_substeps=10, single_process=sp)

    # Initialize Environment
    env = Env(name='rx',
              rate=1,
              actions=actions,
              observations=observations,
              states=states,
              bridge=bridge,
              nodes=[N1, N3, N4, N5, KF],
              objects=[viper])

    # First reset
    # todo: why sleep required in _initialize?
    # todo: how to be sure that all sim-nodes have been initialized here?
    obs = env.reset()
    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(2):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
        obs = env.reset()
    print('\n[Finished]')
    rospy.sleep(100000)

    # todo: CheckEnv(env): i/o correct, fully connected & DAG when RealReset (check graph without all nodes dependent on Env's actions)
    # todo: Visualize and perform checks on DAGs (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: How to combine GUI with custom env?

    # todo: implement real_time rx pipeline

    # todo: creat RxNode config.yaml for observation, action & state node
    # todo: change address naming conventions, remove address specification from node creation API
    # todo: Avoid blocking at initialization: make publishers latched? Wait for all simnodes to be initialized.

    # todo: automatically add all states of each node to env. I.e. avoid explicit registry of every state connection.
    #       how to then determine the alias name of each state inside the environment?
    #       if an (object)state is connected to a target(state) of a ResetNode, only add state to env once.
    #       seems to already go correctly when you add add target(state) under the same key as the (object)state
    #       Make sure that target address is the same as address of node state so that '/state/done' arrives to bridge.

    # todo: make all msg_reset publishers latched?
    # todo: create a register_node function in the RxNode class to initialize a node inside the process of another node.
    # todo; how to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write?

    # todo: CLEAN-UP ACTIONS
    # todo: Combine action, observation, supervisor node into a single environment node.
    # todo: pass (default) args to node_cls(...). Currently done via the paramserver? Make explicit in constructor.
    # todo: make baseclasses for bridge, node, simstate
    # todo: replace rospy.sleep(..) with time.sleep(..)
    # todo: replace reset info with rospy.logdebug(...), so that we log it if warn level is debug
    # todo: change structure of callback/reset input: unpack to descriptive arguments e.g. reset(tick, state)
    # todo: add msg_types as static properties to node.py implementation (make class more descriptive)
    # todo: change rate type to float (instead of int)
    # todo: create publishers in RxMessageBroker (outputs,  node_outputs)
    # todo: cleanup eagerx_core.__init__: refactor to separate rxpipelines.py, rxoperators.py
    # todo: print statements of callback inside ProcessNode: color specified as additional argument
